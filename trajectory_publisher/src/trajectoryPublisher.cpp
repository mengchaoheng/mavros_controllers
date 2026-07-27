/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Trajectory Publisher
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "trajectory_publisher/trajectoryPublisher.h"

#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

namespace {
double roundToFourDecimals(double value) { return std::round(value * 10000.0) / 10000.0; }

int clampTrajectoryType(int type) {
  return std::max(TRAJ_FIGURE8_HORIZONTAL, std::min(TRAJ_FAST_CIRCLE, type));
}

bool getDoubleParamEither(const ros::NodeHandle& nh, const std::string& primary, const std::string& secondary,
                          double& value) {
  if (nh.getParam(primary, value)) {
    return true;
  }
  if (!secondary.empty() && nh.getParam(secondary, value)) {
    return true;
  }
  return false;
}

double readDoubleParamEither(const ros::NodeHandle& nh, const std::string& primary, const std::string& secondary,
                             double fallback) {
  double value = fallback;
  getDoubleParamEither(nh, primary, secondary, value);
  return value;
}

double clampDouble(double value, double low, double high) { return std::max(low, std::min(high, value)); }

void computeQuinticCoefficients(double p0, double v0, double a0, double pf, double vf, double af, double duration,
                                Eigen::Matrix<double, 6, 1>& coeffs) {
  const double T = std::max(duration, 1e-3);
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;

  coeffs(0) = p0;
  coeffs(1) = v0;
  coeffs(2) = 0.5 * a0;
  coeffs(3) = (20.0 * (pf - p0) - (8.0 * vf + 12.0 * v0) * T - (3.0 * a0 - af) * T2) / (2.0 * T3);
  coeffs(4) = (30.0 * (p0 - pf) + (14.0 * vf + 16.0 * v0) * T + (3.0 * a0 - 2.0 * af) * T2) /
              (2.0 * T4);
  coeffs(5) = (12.0 * (pf - p0) - (6.0 * vf + 6.0 * v0) * T - (a0 - af) * T2) / (2.0 * T5);
}

double shortestAngularDistance(double from, double to) { return std::atan2(std::sin(to - from), std::cos(to - from)); }
}  // namespace

trajectoryPublisher::trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      motion_selector_(0),
      tracking_requested_(false),
      start_transition_pending_(false),
      mavpose_received_(false),
      controller_direct_mode_(false),
      first_reconfigure_(true),
      transition_active_(false),
      omega_mode_(TRAJ_OMEGA_FIXED),
      transition_stage_(0) {
  trajectoryPub_ = nh_.advertise<nav_msgs::Path>("trajectory_publisher/trajectory", 1);
  referencePub_ = nh_.advertise<geometry_msgs::TwistStamped>("reference/setpoint", 1);
  flatreferencePub_ = nh_.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 1);
  yawreferencePub_ = nh_.advertise<std_msgs::Float32>("reference/yaw", 1);
  rawreferencePub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
  global_rawreferencePub_ = nh_.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 1);
  motionselectorSub_ =
      nh_.subscribe("trajectory_publisher/motionselector", 1, &trajectoryPublisher::motionselectorCallback, this,
                    ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &trajectoryPublisher::mavposeCallback, this,
                              ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &trajectoryPublisher::mavtwistCallback, this,
                               ros::TransportHints().tcpNoDelay());
  mavstate_sub_ = nh_.subscribe("mavros/state", 1, &trajectoryPublisher::mavstateCallback, this,
                                ros::TransportHints().tcpNoDelay());
  direct_mode_sub_ =
      nh_.subscribe("trajectory_publisher/direct_mode", 1, &trajectoryPublisher::directModeCallback, this,
                    ros::TransportHints().tcpNoDelay());

  std::string arming_service;
  std::string set_mode_service;
  nh_private_.param<std::string>("arming_service", arming_service, "/mavros/cmd/arming");
  nh_private_.param<std::string>("set_mode_service", set_mode_service, "/mavros/set_mode");
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(arming_service);
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_service);

  nh_private_.param<double>("updaterate", controlUpdate_dt_, 0.01);
  controlUpdate_dt_ = std::max(0.001, controlUpdate_dt_);
  trajloop_timer_ = nh_.createTimer(ros::Duration(0.1), &trajectoryPublisher::loopCallback, this);
  refloop_timer_ = nh_.createTimer(ros::Duration(controlUpdate_dt_), &trajectoryPublisher::refCallback, this);
  offboard_manager_timer_ =
      nh_.createTimer(ros::Duration(0.1), &trajectoryPublisher::offboardManagerCallback, this);

  trajtriggerServ_ = nh_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);
  private_trajtriggerServ_ = nh_private_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);

  nh_private_.param<double>("initpos_x", init_pos_x_, 0.0);
  nh_private_.param<double>("initpos_y", init_pos_y_, 0.0);
  nh_private_.param<double>("initpos_z", init_pos_z_, 1.0);
  nh_private_.param<double>("horizon", primitive_duration_, 1.0);
  nh_private_.param<double>("maxjerk", max_jerk_, 10.0);
  readTrajectoryType();
  readOmegaProfiles();
  const bool omega_param_present =
      nh_private_.getParam("omega_value", omega_value_) || nh_private_.getParam("omegaValue", omega_value_) ||
      nh_private_.getParam("shape_omega", omega_value_);
  if (!omega_param_present) {
    omega_value_ = omegaDefaultForTrajectory(trajectory_type_);
  }
  omega_value_ = clampToOmegaRange(omega_value_, trajectory_type_);
  nh_private_.param<int>("omega_mode", omega_mode_, TRAJ_OMEGA_FIXED);
  nh_private_.param<double>("omega_start", omega_start_, 0.5);
  nh_private_.param<double>("omega_end", omega_end_, 1.5);
  nh_private_.param<double>("omega_duration", omega_duration_, 20.0);
  nh_private_.param<double>("path_preview_cycles", path_preview_cycles_, 10.0);
  path_preview_cycles_ = std::max(1.0, path_preview_cycles_);
  nh_private_.param<bool>("trajectory_yaw_lock", trajectory_yaw_lock_, false);
  nh_private_.param<double>("trajectory_yaw_fixed", trajectory_yaw_fixed_, 0.0);
  nh_private_.param<bool>("takeoff_before_trajectory", takeoff_before_trajectory_, true);
  nh_private_.param<double>("takeoff_position_tolerance", takeoff_position_tolerance_, 0.25);
  nh_private_.param<double>("takeoff_velocity_tolerance", takeoff_velocity_tolerance_, 0.5);
  nh_private_.param<double>("trajectory_start_ramp_duration", trajectory_start_ramp_configured_duration_, 2.0);
  nh_private_.param<bool>("adaptive_trajectory_start_ramp", adaptive_trajectory_start_ramp_, true);
  nh_private_.param<double>("trajectory_start_ramp_min_duration", trajectory_start_ramp_min_duration_, 0.5);
  nh_private_.param<double>("trajectory_start_ramp_velocity_limit", trajectory_start_ramp_velocity_limit_, 2.0);
  nh_private_.param<double>("trajectory_start_ramp_acceleration_limit", trajectory_start_ramp_acceleration_limit_, 6.0);
  nh_private_.param<double>("trajectory_switch_transition_duration", trajectory_switch_transition_duration_, 3.0);
  nh_private_.param<double>("trajectory_switch_transition_min_duration", trajectory_switch_transition_min_duration_, 1.0);
  nh_private_.param<double>("trajectory_switch_transition_max_duration", trajectory_switch_transition_max_duration_, 12.0);
  nh_private_.param<double>("trajectory_switch_transition_velocity_limit", trajectory_switch_transition_velocity_limit_, 1.2);
  nh_private_.param<double>("trajectory_switch_transition_acceleration_limit",
                            trajectory_switch_transition_acceleration_limit_, 2.5);
  nh_private_.param<double>("trajectory_switch_stop_speed_threshold", trajectory_switch_stop_speed_threshold_, 0.2);
  nh_private_.param<double>("trajectory_stop_deceleration_limit", trajectory_stop_deceleration_limit_, 5.0);
  nh_private_.param<double>("trajectory_stop_min_duration", trajectory_stop_min_duration_, 0.3);
  nh_private_.param<int>("number_of_primitives", num_primitives_, 7);
  nh_private_.param<int>("reference_type", pubreference_type_, 2);
  nh_private_.param<bool>("auto_offboard", auto_offboard_, false);
  nh_private_.param<bool>("auto_arm", auto_arm_, false);
  nh_private_.param<int>("preflight_setpoint_count", preflight_setpoint_count_, 100);
  nh_private_.param<double>("offboard_request_interval", offboard_request_interval_, 2.0);
  nh_private_.param<double>("arm_request_interval", arm_request_interval_, 2.0);
  preflight_setpoint_count_ = std::max(1, preflight_setpoint_count_);
  offboard_request_interval_ = std::max(0.1, offboard_request_interval_);
  arm_request_interval_ = std::max(0.1, arm_request_interval_);
  direct_setpoint_count_ = 0;
  last_offboard_request_ = ros::Time(0);
  last_arm_request_ = ros::Time(0);
  trajectory_start_ramp_configured_duration_ = std::max(0.0, trajectory_start_ramp_configured_duration_);
  trajectory_start_ramp_duration_ = trajectory_start_ramp_configured_duration_;

  if ((auto_offboard_ || auto_arm_) && pubreference_type_ != REF_SETPOINTRAW) {
    ROS_WARN("auto_offboard/auto_arm are only active when reference_type=%d (direct PX4 mode).",
             REF_SETPOINTRAW);
  }

  readShapeParams();

  inputs_.resize(num_primitives_);

  if (trajectory_type_ == 0) {  // Polynomial Trajectory
    if (num_primitives_ == 7) {
      inputs_.at(0) << 0.0, 0.0, 0.0;
      inputs_.at(1) << 1.0, 0.0, 0.0;
      inputs_.at(2) << -1.0, 0.0, 0.0;
      inputs_.at(3) << 0.0, 1.0, 0.0;
      inputs_.at(4) << 0.0, -1.0, 0.0;
      inputs_.at(5) << 0.0, 0.0, 1.0;
      inputs_.at(6) << 0.0, 0.0, -1.0;
    }

    for (int i = 0; i < num_primitives_; i++) {
      motionPrimitives_.emplace_back(std::make_shared<polynomialtrajectory>());
      primitivePub_.push_back(
          nh_.advertise<nav_msgs::Path>("trajectory_publisher/primitiveset" + std::to_string(i), 1));
      inputs_.at(i) = inputs_.at(i) * max_jerk_;
    }
  } else {
    trajectory_type_ = clampTrajectoryType(trajectory_type_);
    num_primitives_ = 1;
    motionPrimitives_.emplace_back(std::make_shared<shapetrajectory>(trajectory_type_));
    primitivePub_.push_back(nh_.advertise<nav_msgs::Path>("trajectory_publisher/primitiveset", 1));
  }

  p_targ << init_pos_x_, init_pos_y_, init_pos_z_;
  v_targ << 0.0, 0.0, 0.0;
  a_targ << 0.0, 0.0, 0.0;
  j_targ << 0.0, 0.0, 0.0;
  p_mav_ << init_pos_x_, init_pos_y_, 0.0;
  v_mav_ << 0.0, 0.0, 0.0;
  shape_origin_ << init_pos_x_, init_pos_y_, init_pos_z_;
  shape_axis_ << 0.0, 0.0, 1.0;
  yaw_targ_ = 0.0;
  shape_phase_shift_ = 0.0;
  transition_position_coeffs_.setZero();
  transition_yaw_coeffs_.setZero();
  transition_final_target_ << init_pos_x_, init_pos_y_, init_pos_z_;
  holding_target_ = transition_final_target_;
  holding_yaw_ = yaw_targ_;
  transition_segment_duration_ = 0.0;
  motion_selector_ = 0;
  trajectory_started_ = false;
  start_time_ = ros::Time::now();
  omega_schedule_start_time_ = start_time_;

  initializePrimitives(trajectory_type_);
  updateTakeoffTarget();
  holding_target_ = p_mav_;
  holding_yaw_ = trajectory_yaw_lock_ ? trajectory_yaw_fixed_ : 0.0;
  setHoldingReference();

  dynamic_reconfigure::Server<trajectory_publisher::TrajectoryPublisherConfig>::CallbackType dyn_cb;
  dyn_cb = boost::bind(&trajectoryPublisher::dynamicReconfigureCallback, this, _1, _2);
  dyn_server_ =
      std::make_shared<dynamic_reconfigure::Server<trajectory_publisher::TrajectoryPublisherConfig>>(nh_private_);
  dyn_server_->setCallback(dyn_cb);
}

void trajectoryPublisher::readTrajectoryType() {
  trajectory_type_ = TRAJ_FIGURE8_HORIZONTAL;

  std::string traj_name;
  if (nh_private_.getParam("trajName", traj_name)) {
    trajectory_type_ = shapetrajectory::typeFromName(traj_name, trajectory_type_);
    return;
  }

  int traj_id = trajectory_type_;
  if (nh_private_.getParam("trajName", traj_id)) {
    trajectory_type_ = traj_id;
    return;
  }

  nh_private_.param<int>("trajectory_type", trajectory_type_, TRAJ_FIGURE8_HORIZONTAL);
}

void trajectoryPublisher::readOmegaProfiles() {
  figure8_horizontal_omega_profile_ = {
      readDoubleParamEither(nh_private_, "figure8_horizontal_omega_default", "figure8HorizontalOmegaDefault", 0.5),
      readDoubleParamEither(nh_private_, "figure8_horizontal_omega_min", "figure8HorizontalOmegaMin", 0.05),
      readDoubleParamEither(nh_private_, "figure8_horizontal_omega_max", "figure8HorizontalOmegaMax", 1.5)};
  figure8_vertical_omega_profile_ = {
      readDoubleParamEither(nh_private_, "figure8_vertical_omega_default", "figure8VerticalOmegaDefault", 0.5),
      readDoubleParamEither(nh_private_, "figure8_vertical_omega_min", "figure8VerticalOmegaMin", 0.05),
      readDoubleParamEither(nh_private_, "figure8_vertical_omega_max", "figure8VerticalOmegaMax", 1.5)};
  helix_flip_omega_profile_ = {readDoubleParamEither(nh_private_, "helix_flip_omega_default", "helixFlipOmegaDefault", 1.0),
                               readDoubleParamEither(nh_private_, "helix_flip_omega_min", "helixFlipOmegaMin", 0.05),
                               readDoubleParamEither(nh_private_, "helix_flip_omega_max", "helixFlipOmegaMax", 4.0)};
  helix_flip_y_omega_profile_ = {
      readDoubleParamEither(nh_private_, "helix_flip_y_omega_default", "helixFlipYOmegaDefault", 1.0),
      readDoubleParamEither(nh_private_, "helix_flip_y_omega_min", "helixFlipYOmegaMin", 0.05),
      readDoubleParamEither(nh_private_, "helix_flip_y_omega_max", "helixFlipYOmegaMax", 4.0)};
  flip_loop_sine_omega_profile_ = {
      readDoubleParamEither(nh_private_, "flip_loop_sine_omega_default", "flipLoopSineOmegaDefault", 1.0),
      readDoubleParamEither(nh_private_, "flip_loop_sine_omega_min", "flipLoopSineOmegaMin", 0.05),
      readDoubleParamEither(nh_private_, "flip_loop_sine_omega_max", "flipLoopSineOmegaMax", 4.0)};
  fast_circle_omega_profile_ = {
      readDoubleParamEither(nh_private_, "fast_circle_omega_default", "fastCircleOmegaDefault", 0.8),
      readDoubleParamEither(nh_private_, "fast_circle_omega_min", "fastCircleOmegaMin", 0.05),
      readDoubleParamEither(nh_private_, "fast_circle_omega_max", "fastCircleOmegaMax", 3.5)};
}

double trajectoryPublisher::omegaDefaultForTrajectory(int type) const {
  switch (type) {
    case TRAJ_FIGURE8_VERTICAL:
      return clampToOmegaRange(figure8_vertical_omega_profile_.default_value, type);
    case TRAJ_HELIX_FLIP:
      return clampToOmegaRange(helix_flip_omega_profile_.default_value, type);
    case TRAJ_HELIX_FLIP_Y:
      return clampToOmegaRange(helix_flip_y_omega_profile_.default_value, type);
    case TRAJ_FLIP_LOOP_SINE:
      return clampToOmegaRange(flip_loop_sine_omega_profile_.default_value, type);
    case TRAJ_FAST_CIRCLE:
      return clampToOmegaRange(fast_circle_omega_profile_.default_value, type);
    case TRAJ_FIGURE8_HORIZONTAL:
    default:
      return clampToOmegaRange(figure8_horizontal_omega_profile_.default_value, type);
  }
}

std::pair<double, double> trajectoryPublisher::omegaRangeForTrajectory(int type) const {
  OmegaProfile profile = figure8_horizontal_omega_profile_;
  switch (type) {
    case TRAJ_FIGURE8_VERTICAL:
      profile = figure8_vertical_omega_profile_;
      break;
    case TRAJ_HELIX_FLIP:
      profile = helix_flip_omega_profile_;
      break;
    case TRAJ_HELIX_FLIP_Y:
      profile = helix_flip_y_omega_profile_;
      break;
    case TRAJ_FLIP_LOOP_SINE:
      profile = flip_loop_sine_omega_profile_;
      break;
    case TRAJ_FAST_CIRCLE:
      profile = fast_circle_omega_profile_;
      break;
    case TRAJ_FIGURE8_HORIZONTAL:
    default:
      break;
  }

  const double min_value = std::max(0.01, std::min(profile.min, profile.max));
  const double max_value = std::max(min_value, profile.max);
  return std::make_pair(min_value, max_value);
}

double trajectoryPublisher::clampToOmegaRange(double value, int type) const {
  const std::pair<double, double> range = omegaRangeForTrajectory(type);
  return clampDouble(value, range.first, range.second);
}

void trajectoryPublisher::readShapeParams() {
  nh_private_.param<double>("figure8_horizontal_Ax", shape_params_.figure8_horizontal_Ax, 2.0);
  nh_private_.param<double>("figure8_horizontal_Ay", shape_params_.figure8_horizontal_Ay, 2.0);
  nh_private_.param<double>("figure8_horizontal_Hc", shape_params_.figure8_horizontal_Hc, 3.0);
  nh_private_.param<double>("figure8_horizontal_theta0", shape_params_.figure8_horizontal_theta0, 0.0);

  nh_private_.param<double>("figure8_vertical_Ay", shape_params_.figure8_vertical_Ay, 2.0);
  nh_private_.param<double>("figure8_vertical_Az", shape_params_.figure8_vertical_Az, 2.0);
  nh_private_.param<double>("figure8_vertical_Hc", shape_params_.figure8_vertical_Hc, 3.0);
  nh_private_.param<double>("figure8_vertical_theta0", shape_params_.figure8_vertical_theta0, -M_PI / 4.0);

  nh_private_.param<double>("helix_flip_Ay", shape_params_.helix_flip_Ay, 2.0);
  nh_private_.param<double>("helix_flip_Az", shape_params_.helix_flip_Az, 2.0);
  nh_private_.param<double>("helix_flip_Hc", shape_params_.helix_flip_Hc, 3.0);
  nh_private_.param<double>("helix_flip_Vx", shape_params_.helix_flip_Vx, 0.30);
  nh_private_.param<double>("helix_flip_theta0", shape_params_.helix_flip_theta0, 0.0);

  nh_private_.param<double>("helix_flip_y_Ax", shape_params_.helix_flip_y_Ax, 2.0);
  nh_private_.param<double>("helix_flip_y_Az", shape_params_.helix_flip_y_Az, 2.0);
  nh_private_.param<double>("helix_flip_y_Hc", shape_params_.helix_flip_y_Hc, 3.0);
  nh_private_.param<double>("helix_flip_y_Vy", shape_params_.helix_flip_y_Vy, 0.30);
  nh_private_.param<double>("helix_flip_y_theta0", shape_params_.helix_flip_y_theta0, 0.0);

  nh_private_.param<double>("flip_loop_sine_Ay", shape_params_.flip_loop_sine_Ay, 2.0);
  nh_private_.param<double>("flip_loop_sine_Az", shape_params_.flip_loop_sine_Az, 2.0);
  nh_private_.param<double>("flip_loop_sine_Hc", shape_params_.flip_loop_sine_Hc, 3.0);
  nh_private_.param<double>("flip_loop_sine_Vx", shape_params_.flip_loop_sine_Vx, 0.0);
  nh_private_.param<double>("flip_loop_sine_theta0", shape_params_.flip_loop_sine_theta0, 0.0);

  nh_private_.param<double>("fast_circle_Ax", shape_params_.fast_circle_Ax, 3.0);
  nh_private_.param<double>("fast_circle_Ay", shape_params_.fast_circle_Ay, 3.0);
  nh_private_.param<double>("fast_circle_Hc", shape_params_.fast_circle_Hc, 3.0);
  nh_private_.param<double>("fast_circle_theta0", shape_params_.fast_circle_theta0, 0.0);
}

void trajectoryPublisher::updateReference() {
  curr_time_ = ros::Time::now();
  if (transition_active_) {
    if (current_state_.mode != "OFFBOARD" || !current_state_.armed) {
      transition_active_ = false;
      trajectory_started_ = false;
      start_transition_pending_ = tracking_requested_;
      start_time_ = curr_time_;
      omega_schedule_start_time_ = curr_time_;
      holding_target_ = p_mav_;
      holding_yaw_ = yaw_targ_;
      setHoldingReference();
      return;
    }
    updateTransitionReference();
    return;
  }

  if (!tracking_requested_) {
    trajectory_started_ = false;
    setHoldingReference();
    return;
  }

  if (start_transition_pending_) {
    trajectory_started_ = false;
    holding_target_ = p_mav_;
    holding_yaw_ = yaw_targ_;
    setHoldingReference();
    if (!mavpose_received_ || current_state_.mode != "OFFBOARD" || !current_state_.armed) {
      return;
    }
    start_transition_pending_ = false;
    startTrajectoryTransition();
    updateTransitionReference();
    return;
  }

  if (current_state_.mode != "OFFBOARD" || !current_state_.armed) {
    trajectory_started_ = false;
    start_transition_pending_ = true;
    start_time_ = curr_time_;
    omega_schedule_start_time_ = curr_time_;
    holding_target_ = p_mav_;
    holding_yaw_ = yaw_targ_;
    setHoldingReference();
    return;
  }

  if (!takeoff_before_trajectory_) {
    if (!trajectory_started_) {
      start_time_ = curr_time_;
      omega_schedule_start_time_ = curr_time_;
      trajectory_started_ = true;
    }
    trigger_time_ = (curr_time_ - start_time_).toSec();
    const double omega_time = std::max(0.0, (curr_time_ - omega_schedule_start_time_).toSec());
    evaluateTrajectoryReference(trigger_time_, omega_time);
    return;
  }

  if (!trajectory_started_) {
    if (!takeoffTargetReached()) {
      setTakeoffReference();
      return;
    }

    start_time_ = curr_time_;
    omega_schedule_start_time_ = curr_time_;
    updateTrajectoryStartRampDuration();
    trajectory_started_ = true;
    ROS_INFO("Takeoff reference reached, starting %s.", shapetrajectory::typeName(trajectory_type_));
  }

  trigger_time_ = (curr_time_ - start_time_).toSec();
  const double omega_time = std::max(0.0, (curr_time_ - omega_schedule_start_time_).toSec());
  evaluateTrajectoryReference(trigger_time_, omega_time);
  applyTrajectoryStartRamp(trigger_time_);
}

void trajectoryPublisher::evaluateTrajectoryReference(double trajectory_time, double omega_time) {
  std::shared_ptr<shapetrajectory> shape =
      std::dynamic_pointer_cast<shapetrajectory>(motionPrimitives_.at(motion_selector_));
  if (shape) {
    p_targ = shape->getPosition(trajectory_time, omega_time);
    v_targ = shape->getVelocity(trajectory_time, omega_time);
    if (pubreference_type_ != 0) {
      a_targ = shape->getAcceleration(trajectory_time, omega_time);
      j_targ = shape->getJerk(trajectory_time, omega_time);
    }
  } else {
    p_targ = motionPrimitives_.at(motion_selector_)->getPosition(trajectory_time);
    v_targ = motionPrimitives_.at(motion_selector_)->getVelocity(trajectory_time);
    if (pubreference_type_ != 0) {
      a_targ = motionPrimitives_.at(motion_selector_)->getAcceleration(trajectory_time);
      j_targ = motionPrimitives_.at(motion_selector_)->getJerk(trajectory_time);
    }
  }
  updateReferenceYaw(trajectory_time, omega_time);
}

void trajectoryPublisher::initializePrimitives(int type) {
  if (type == 0) {
    for (int i = 0; i < motionPrimitives_.size(); i++)
      motionPrimitives_.at(i)->generatePrimitives(p_mav_, v_mav_, inputs_.at(i));
  } else {
    applyShapeParams();
  }
}

void trajectoryPublisher::applyShapeParams() {
  for (int i = 0; i < motionPrimitives_.size(); i++) {
    std::shared_ptr<shapetrajectory> shape = std::dynamic_pointer_cast<shapetrajectory>(motionPrimitives_.at(i));
    if (shape) {
      shape->setType(trajectory_type_);
      shape->setParams(shape_params_);
      shape->setOmega(omega_mode_, omega_value_, omega_start_, omega_end_, omega_duration_);
      shape->setPhaseShift(shape_phase_shift_);
      shape->setPathPreviewCycles(path_preview_cycles_);
      shape->initPrimitives(shape_origin_, shape_axis_, omega_value_);
    }
  }
}

void trajectoryPublisher::resetTrajectoryStart() {
  shape_phase_shift_ = 0.0;
  applyShapeParams();
  updateTakeoffTarget();
  transition_active_ = false;
  trajectory_started_ = false;
  start_transition_pending_ = true;
  holding_target_ = p_mav_;
  holding_yaw_ = yaw_targ_;
  start_time_ = ros::Time::now();
  omega_schedule_start_time_ = start_time_;
  setHoldingReference();
}

void trajectoryPublisher::startTracking() {
  tracking_requested_ = true;
  resetTrajectoryStart();
  ROS_INFO("Trajectory tracking requested: waiting for pose, OFFBOARD and armed before moving to %s start.",
           shapetrajectory::typeName(trajectory_type_));
}

void trajectoryPublisher::stopTracking() {
  if (!tracking_requested_ && !transition_active_) {
    return;
  }
  tracking_requested_ = false;
  start_transition_pending_ = false;
  trajectory_started_ = false;
  start_time_ = ros::Time::now();
  omega_schedule_start_time_ = start_time_;
  startStopTransition();
}

void trajectoryPublisher::startStopTransition() {
  transition_active_ = false;
  const Eigen::Vector3d position_start = p_mav_;
  const Eigen::Vector3d velocity_start = v_mav_;
  const double speed = velocity_start.norm();
  holding_yaw_ = yaw_targ_;

  if (speed <= trajectory_switch_stop_speed_threshold_) {
    holding_target_ = position_start;
    setHoldingReference();
    ROS_INFO("Trajectory tracking stopped; holding current position.");
    return;
  }

  const double stop_deceleration = std::max(0.1, trajectory_stop_deceleration_limit_);
  const double stop_min_duration = std::max(0.1, trajectory_stop_min_duration_);
  const double brake_duration =
      clampDouble(1.5 * speed / stop_deceleration, stop_min_duration,
                  std::max(stop_min_duration, trajectory_switch_transition_max_duration_));
  holding_target_ = position_start + 0.5 * velocity_start * brake_duration;
  transition_final_target_ = holding_target_;
  startTransitionSegment(position_start, velocity_start, Eigen::Vector3d::Zero(), holding_target_, yaw_targ_,
                         holding_yaw_, brake_duration, 3);
  transition_active_ = true;
  ROS_INFO("Trajectory tracking stopping: braking for %.2f s over %.2f m, then holding.", brake_duration,
           (holding_target_ - position_start).norm());
}

void trajectoryPublisher::setHoldingReference() {
  p_targ = holding_target_;
  v_targ.setZero();
  a_targ.setZero();
  j_targ.setZero();
  yaw_targ_ = holding_yaw_;
}

void trajectoryPublisher::rebaseOmegaSchedule() {
  if (!trajectory_started_ || transition_active_) {
    shape_phase_shift_ = 0.0;
    omega_schedule_start_time_ = ros::Time::now();
    return;
  }

  const ros::Time now = ros::Time::now();
  const double omega_time = std::max(0.0, (now - omega_schedule_start_time_).toSec());
  std::shared_ptr<shapetrajectory> shape =
      std::dynamic_pointer_cast<shapetrajectory>(motionPrimitives_.at(motion_selector_));
  if (shape) {
    shape_phase_shift_ += shape->getPhaseAdvance(omega_time);
  }
  omega_schedule_start_time_ = now;
}

void trajectoryPublisher::updatePrimitives() {
  for (int i = 0; i < motionPrimitives_.size(); i++) motionPrimitives_.at(i)->generatePrimitives(p_mav_, v_mav_);
}

void trajectoryPublisher::updateTakeoffTarget() {
  if (trajectory_type_ == 0) {
    takeoff_target_ << init_pos_x_, init_pos_y_, init_pos_z_;
  } else {
    takeoff_target_ = motionPrimitives_.at(motion_selector_)->getPosition(0.0);
  }
}

double trajectoryPublisher::estimateTransitionDuration(double distance, double speed) const {
  const double min_duration = std::max(0.1, trajectory_switch_transition_min_duration_);
  const double configured_duration = std::max(min_duration, trajectory_switch_transition_duration_);
  const double max_duration = std::max(configured_duration, trajectory_switch_transition_max_duration_);
  const double velocity_limit = std::max(0.05, trajectory_switch_transition_velocity_limit_);
  const double acceleration_limit = std::max(0.05, trajectory_switch_transition_acceleration_limit_);

  constexpr double kSmoothstepMaxRate = 1.875;
  constexpr double kSmoothstepMaxAccel = 5.773502691896258;
  double duration = configured_duration;
  if (distance > 1e-3) {
    duration = std::max(duration, kSmoothstepMaxRate * distance / velocity_limit);
    duration = std::max(duration, std::sqrt(kSmoothstepMaxAccel * distance / acceleration_limit));
  }
  if (speed > 1e-3) {
    duration = std::max(duration, 2.0 * speed / acceleration_limit);
  }
  if (!std::isfinite(duration)) {
    duration = configured_duration;
  }
  return clampDouble(duration, min_duration, max_duration);
}

void trajectoryPublisher::startTransitionSegment(const Eigen::Vector3d& position_start,
                                                 const Eigen::Vector3d& velocity_start,
                                                 const Eigen::Vector3d& acceleration_start,
                                                 const Eigen::Vector3d& position_goal, double yaw_start,
                                                 double yaw_goal, double duration, int stage) {
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  transition_segment_duration_ = std::max(0.1, duration);
  transition_start_time_ = ros::Time::now();
  transition_stage_ = stage;

  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Matrix<double, 6, 1> coeffs;
    computeQuinticCoefficients(position_start(axis), velocity_start(axis), acceleration_start(axis),
                               position_goal(axis), zero(axis), zero(axis), transition_segment_duration_, coeffs);
    transition_position_coeffs_.col(axis) = coeffs;
  }

  const double yaw_goal_unwrapped = yaw_start + shortestAngularDistance(yaw_start, yaw_goal);
  computeQuinticCoefficients(yaw_start, 0.0, 0.0, yaw_goal_unwrapped, 0.0, 0.0,
                             transition_segment_duration_, transition_yaw_coeffs_);
}

void trajectoryPublisher::startTrajectoryTransition() {
  transition_final_target_ = takeoff_target_;
  transition_active_ = true;
  const double yaw_goal = trajectory_yaw_lock_ ? trajectory_yaw_fixed_ : 0.0;

  const Eigen::Vector3d position_start = p_mav_;
  const Eigen::Vector3d velocity_start = v_mav_;
  const Eigen::Vector3d acceleration_start = Eigen::Vector3d::Zero();
  const double speed = velocity_start.norm();

  if (speed > trajectory_switch_stop_speed_threshold_) {
    const double brake_duration =
        clampDouble(2.0 * speed / std::max(0.05, trajectory_switch_transition_acceleration_limit_),
                    std::max(0.1, trajectory_switch_transition_min_duration_),
                    std::max(trajectory_switch_transition_min_duration_, trajectory_switch_transition_max_duration_));
    const Eigen::Vector3d stop_point = position_start + 0.5 * velocity_start * brake_duration;
    startTransitionSegment(position_start, velocity_start, acceleration_start, stop_point, yaw_targ_, yaw_goal,
                           brake_duration, 1);
    ROS_INFO("Trajectory switch transition: braking for %.2f s before moving to %s.", brake_duration,
             shapetrajectory::typeName(trajectory_type_));
    return;
  }

  const double distance = (transition_final_target_ - position_start).norm();
  const double duration = estimateTransitionDuration(distance, speed);
  startTransitionSegment(position_start, velocity_start, acceleration_start, transition_final_target_, yaw_targ_,
                         yaw_goal, duration, 2);
  ROS_INFO("Trajectory switch transition: moving to %s start in %.2f s.", shapetrajectory::typeName(trajectory_type_),
           duration);
}

void trajectoryPublisher::evaluateTransitionSegment(double time) {
  const double t = clampDouble(time, 0.0, transition_segment_duration_);
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;

  for (int axis = 0; axis < 3; ++axis) {
    const auto c = transition_position_coeffs_.col(axis);
    p_targ(axis) = c(0) + c(1) * t + c(2) * t2 + c(3) * t3 + c(4) * t4 + c(5) * t5;
    v_targ(axis) = c(1) + 2.0 * c(2) * t + 3.0 * c(3) * t2 + 4.0 * c(4) * t3 + 5.0 * c(5) * t4;
    a_targ(axis) = 2.0 * c(2) + 6.0 * c(3) * t + 12.0 * c(4) * t2 + 20.0 * c(5) * t3;
    j_targ(axis) = 6.0 * c(3) + 24.0 * c(4) * t + 60.0 * c(5) * t2;
  }

  const auto y = transition_yaw_coeffs_;
  yaw_targ_ = y(0) + y(1) * t + y(2) * t2 + y(3) * t3 + y(4) * t4 + y(5) * t5;
}

void trajectoryPublisher::updateTransitionReference() {
  const double elapsed = (curr_time_ - transition_start_time_).toSec();
  evaluateTransitionSegment(elapsed);

  if (elapsed < transition_segment_duration_) {
    return;
  }

  if (transition_stage_ == 1) {
    const Eigen::Vector3d stop_point = p_targ;
    const double distance = (transition_final_target_ - stop_point).norm();
    const double duration = estimateTransitionDuration(distance, 0.0);
    const double yaw_goal = trajectory_yaw_lock_ ? trajectory_yaw_fixed_ : 0.0;
    startTransitionSegment(stop_point, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), transition_final_target_,
                           yaw_targ_, yaw_goal, duration, 2);
    evaluateTransitionSegment(0.0);
    ROS_INFO("Trajectory switch transition: stopped, moving to start in %.2f s.", duration);
    return;
  }

  if (transition_stage_ == 3) {
    holding_target_ = p_targ;
    holding_yaw_ = yaw_targ_;
    transition_active_ = false;
    trajectory_started_ = false;
    setHoldingReference();
    ROS_INFO("Trajectory tracking stopped; braking complete, holding position.");
    return;
  }

  transition_active_ = false;
  trajectory_started_ = false;
  start_time_ = curr_time_;
  omega_schedule_start_time_ = curr_time_;
  setTakeoffReference();
}

void trajectoryPublisher::updateReferenceYaw(double trajectory_time, double omega_time) {
  yaw_targ_ = trajectory_yaw_lock_ ? trajectory_yaw_fixed_ : 0.0;
  if (trajectory_yaw_lock_) {
    return;
  }
  if (trajectory_type_ == 0) {
    return;
  }
  std::shared_ptr<shapetrajectory> shape = std::dynamic_pointer_cast<shapetrajectory>(motionPrimitives_.at(motion_selector_));
  if (shape) {
    yaw_targ_ = shape->getYaw(trajectory_time, omega_time);
  }
}

void trajectoryPublisher::updateTrajectoryStartRampDuration() {
  trajectory_start_ramp_duration_ = trajectory_start_ramp_configured_duration_;
  if (!adaptive_trajectory_start_ramp_ || trajectory_start_ramp_duration_ <= 0.0) {
    return;
  }

  const double max_duration = trajectory_start_ramp_duration_;
  const double min_duration = std::min(std::max(0.0, trajectory_start_ramp_min_duration_), max_duration);
  const double velocity_limit = std::max(trajectory_start_ramp_velocity_limit_, 1e-3);
  const double acceleration_limit = std::max(trajectory_start_ramp_acceleration_limit_, 1e-3);

  double max_distance = (p_mav_ - takeoff_target_).norm();
  double max_speed = v_mav_.norm();
  double max_acceleration = 0.0;
  constexpr int kSamples = 20;
  for (int i = 1; i <= kSamples; ++i) {
    const double sample_time = max_duration * static_cast<double>(i) / static_cast<double>(kSamples);
    const Eigen::Vector3d pos = motionPrimitives_.at(motion_selector_)->getPosition(sample_time);
    const Eigen::Vector3d vel = motionPrimitives_.at(motion_selector_)->getVelocity(sample_time);
    const Eigen::Vector3d acc = motionPrimitives_.at(motion_selector_)->getAcceleration(sample_time);
    max_distance = std::max(max_distance, (pos - takeoff_target_).norm());
    max_speed = std::max(max_speed, vel.norm());
    max_acceleration = std::max(max_acceleration, acc.norm());
  }

  constexpr double kSmoothstepMaxRate = 1.875;
  constexpr double kSmoothstepMaxAccel = 5.773502691896258;
  double estimated_duration = min_duration;
  if (max_distance > 0.0) {
    estimated_duration = std::max(estimated_duration, kSmoothstepMaxRate * max_distance / velocity_limit);
    estimated_duration =
        std::max(estimated_duration, std::sqrt(kSmoothstepMaxAccel * max_distance / acceleration_limit));
  }
  if (max_speed > 0.0) {
    estimated_duration = std::max(estimated_duration, 2.0 * kSmoothstepMaxRate * max_speed / acceleration_limit);
  }

  if (!std::isfinite(estimated_duration)) {
    estimated_duration = max_duration;
  }
  trajectory_start_ramp_duration_ = std::min(std::max(estimated_duration, min_duration), max_duration);
  ROS_INFO("Adaptive trajectory start ramp: T=%.3f s, cap=%.3f s, distance=%.3f m, speed=%.3f m/s, acc=%.3f m/s^2",
           trajectory_start_ramp_duration_, max_duration, max_distance, max_speed, max_acceleration);
}

void trajectoryPublisher::setTakeoffReference() {
  p_targ = takeoff_target_;
  v_targ << 0.0, 0.0, 0.0;
  a_targ << 0.0, 0.0, 0.0;
  j_targ << 0.0, 0.0, 0.0;
  yaw_targ_ = trajectory_yaw_lock_ ? trajectory_yaw_fixed_ : 0.0;
}

bool trajectoryPublisher::takeoffTargetReached() {
  return (p_mav_ - takeoff_target_).norm() < takeoff_position_tolerance_ &&
         v_mav_.norm() < takeoff_velocity_tolerance_;
}

void trajectoryPublisher::applyTrajectoryStartRamp(double trajectory_time) {
  if (trajectory_start_ramp_duration_ <= 0.0 || trajectory_time >= trajectory_start_ramp_duration_) {
    return;
  }

  const double x = std::max(0.0, std::min(1.0, trajectory_time / trajectory_start_ramp_duration_));
  const double x2 = x * x;
  const double x3 = x2 * x;
  const double x4 = x3 * x;
  const double x5 = x4 * x;
  const double scale = 10.0 * x3 - 15.0 * x4 + 6.0 * x5;
  const double scale_dot = (30.0 * x2 - 60.0 * x3 + 30.0 * x4) / trajectory_start_ramp_duration_;
  const double scale_ddot =
      (60.0 * x - 180.0 * x2 + 120.0 * x3) /
      (trajectory_start_ramp_duration_ * trajectory_start_ramp_duration_);
  const double scale_dddot =
      (60.0 - 360.0 * x + 360.0 * x2) / std::pow(trajectory_start_ramp_duration_, 3.0);

  const double yaw_nom = yaw_targ_;
  const double yaw_start = trajectory_yaw_lock_ ? trajectory_yaw_fixed_ : 0.0;
  const Eigen::Vector3d p_nom = p_targ;
  const Eigen::Vector3d v_nom = v_targ;
  const Eigen::Vector3d a_nom = a_targ;
  const Eigen::Vector3d j_nom = j_targ;
  const Eigen::Vector3d dp = p_nom - takeoff_target_;

  p_targ = takeoff_target_ + scale * dp;
  v_targ = scale * v_nom + scale_dot * dp;
  a_targ = scale * a_nom + 2.0 * scale_dot * v_nom + scale_ddot * dp;
  j_targ = scale * j_nom + 3.0 * scale_dot * a_nom + 3.0 * scale_ddot * v_nom + scale_dddot * dp;
  yaw_targ_ = yaw_start + scale * shortestAngularDistance(yaw_start, yaw_nom);
}

void trajectoryPublisher::pubrefTrajectory(int selector) {
  refTrajectory_ = motionPrimitives_.at(selector)->getSegment();
  refTrajectory_.header.stamp = ros::Time::now();
  refTrajectory_.header.frame_id = "map";
  trajectoryPub_.publish(refTrajectory_);
}

void trajectoryPublisher::pubprimitiveTrajectory() {
  for (int i = 0; i < motionPrimitives_.size(); i++) {
    primTrajectory_ = motionPrimitives_.at(i)->getSegment();
    primTrajectory_.header.stamp = ros::Time::now();
    primTrajectory_.header.frame_id = "map";
    primitivePub_.at(i).publish(primTrajectory_);
  }
}

void trajectoryPublisher::pubrefState() {
  geometry_msgs::TwistStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.twist.angular.x = p_targ(0);
  msg.twist.angular.y = p_targ(1);
  msg.twist.angular.z = p_targ(2);
  msg.twist.linear.x = v_targ(0);
  msg.twist.linear.y = v_targ(1);
  msg.twist.linear.z = v_targ(2);
  referencePub_.publish(msg);
}

void trajectoryPublisher::pubflatrefState() {
  controller_msgs::FlatTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.type_mask = pubreference_type_;
  msg.position.x = p_targ(0);
  msg.position.y = p_targ(1);
  msg.position.z = p_targ(2);
  msg.velocity.x = v_targ(0);
  msg.velocity.y = v_targ(1);
  msg.velocity.z = v_targ(2);
  msg.acceleration.x = a_targ(0);
  msg.acceleration.y = a_targ(1);
  msg.acceleration.z = a_targ(2);
  msg.jerk.x = j_targ(0);
  msg.jerk.y = j_targ(1);
  msg.jerk.z = j_targ(2);
  msg.snap.x = 0.0;
  msg.snap.y = 0.0;
  msg.snap.z = 0.0;
  flatreferencePub_.publish(msg);
}

void trajectoryPublisher::pubyawState() {
  std_msgs::Float32 msg;
  msg.data = static_cast<float>(yaw_targ_);
  yawreferencePub_.publish(msg);
}

void trajectoryPublisher::pubrefSetpointRaw() {
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  // MAVROS receives local setpoints in ROS ENU and converts them to PX4 NED.
  // Position, velocity and acceleration are sent together so PX4 can use
  // velocity/acceleration as feed-forward terms in its internal controller.
  msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  msg.position.x = p_targ(0);
  msg.position.y = p_targ(1);
  msg.position.z = p_targ(2);
  msg.velocity.x = v_targ(0);
  msg.velocity.y = v_targ(1);
  msg.velocity.z = v_targ(2);
  msg.acceleration_or_force.x = a_targ(0);
  msg.acceleration_or_force.y = a_targ(1);
  msg.acceleration_or_force.z = a_targ(2);
  msg.yaw = yaw_targ_;
  msg.yaw_rate = 0.0;
  rawreferencePub_.publish(msg);
  if (direct_setpoint_count_ < preflight_setpoint_count_) {
    ++direct_setpoint_count_;
  }
}

void trajectoryPublisher::pubrefSetpointRawGlobal() {
  mavros_msgs::GlobalPositionTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.type_mask = 0;
  msg.coordinate_frame = 5;
  msg.latitude = 47.397742;
  msg.longitude = 8.545594;
  msg.altitude = 500.0;
  msg.velocity.x = v_targ(0);
  msg.velocity.y = v_targ(1);
  msg.velocity.z = v_targ(2);
  msg.acceleration_or_force.x = a_targ(0);
  msg.acceleration_or_force.y = a_targ(1);
  msg.acceleration_or_force.z = a_targ(2);
  msg.yaw = yaw_targ_;
  msg.yaw_rate = 0.0;
  global_rawreferencePub_.publish(msg);
}

void trajectoryPublisher::mavstateCallback(const mavros_msgs::State::ConstPtr& msg) {
  if (current_state_.connected != msg->connected) {
    direct_setpoint_count_ = 0;
  }
  current_state_ = *msg;
}

void trajectoryPublisher::directModeCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (controller_direct_mode_ == msg->data) {
    return;
  }

  controller_direct_mode_ = msg->data;
  ROS_INFO("PX4 direct trajectory route %s.", controller_direct_mode_ ? "enabled" : "disabled");
}

void trajectoryPublisher::loopCallback(const ros::TimerEvent& event) {
  pubrefTrajectory(motion_selector_);
  pubprimitiveTrajectory();
}

void trajectoryPublisher::refCallback(const ros::TimerEvent& event) {
  updateReference();
  pubyawState();
  switch (pubreference_type_) {
    case REF_TWIST:
      pubrefState();
      break;
    case REF_SETPOINTRAW:
      pubrefSetpointRaw();
      break;
    default:
      pubflatrefState();
      break;
  }

  if (controller_direct_mode_ && pubreference_type_ != REF_SETPOINTRAW) {
    pubrefSetpointRaw();
  }
}

void trajectoryPublisher::offboardManagerCallback(const ros::TimerEvent& event) {
  if (pubreference_type_ != REF_SETPOINTRAW || (!auto_offboard_ && !auto_arm_) || !current_state_.connected) {
    return;
  }

  if (direct_setpoint_count_ < preflight_setpoint_count_) {
    ROS_INFO_THROTTLE(1.0, "Priming PX4 OFFBOARD input: %d/%d direct setpoints.",
                      direct_setpoint_count_, preflight_setpoint_count_);
    return;
  }

  const ros::Time now = ros::Time::now();
  if (current_state_.mode != "OFFBOARD") {
    if (!auto_offboard_ || now - last_offboard_request_ < ros::Duration(offboard_request_interval_)) {
      return;
    }

    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = "OFFBOARD";
    if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
      ROS_INFO("PX4 OFFBOARD mode requested.");
    } else {
      ROS_WARN("PX4 OFFBOARD mode request failed; will retry.");
    }
    last_offboard_request_ = now;
    return;
  }

  const ros::Time last_arm_related_request =
      last_arm_request_ > last_offboard_request_ ? last_arm_request_ : last_offboard_request_;
  if (auto_arm_ && !current_state_.armed &&
      now - last_arm_related_request >= ros::Duration(arm_request_interval_)) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
      ROS_INFO("PX4 arming requested.");
    } else {
      ROS_WARN("PX4 arming request failed; will retry.");
    }
    last_arm_request_ = now;
  }
}

bool trajectoryPublisher::triggerCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  if (req.data) {
    if (!mavpose_received_) {
      res.success = false;
      res.message = "cannot start trajectory: local position has not been received";
      return true;
    }
    startTracking();
  } else {
    stopTracking();
  }

  res.success = true;
  res.message = tracking_requested_ ? "trajectory start accepted" : "trajectory stop accepted";
  return true;
}

void trajectoryPublisher::motionselectorCallback(const std_msgs::Int32& selector_msg) {
  if (selector_msg.data >= TRAJ_FIGURE8_HORIZONTAL && selector_msg.data <= TRAJ_FAST_CIRCLE) {
    trajectory_type_ = selector_msg.data;
    if (tracking_requested_) {
      resetTrajectoryStart();
    } else {
      shape_phase_shift_ = 0.0;
      applyShapeParams();
      updateTakeoffTarget();
    }
    ROS_INFO("Trajectory switched by motionselector: %s", shapetrajectory::typeName(trajectory_type_));
  }
}

void trajectoryPublisher::mavposeCallback(const geometry_msgs::PoseStamped& msg) {
  p_mav_(0) = msg.pose.position.x;
  p_mav_(1) = msg.pose.position.y;
  p_mav_(2) = msg.pose.position.z;
  if (!mavpose_received_) {
    mavpose_received_ = true;
    if (!tracking_requested_ && !transition_active_) {
      holding_target_ = p_mav_;
      holding_yaw_ = yaw_targ_;
      setHoldingReference();
      ROS_INFO("Local position received; standby reference locked to current position.");
    }
  }
  updatePrimitives();
}

void trajectoryPublisher::mavtwistCallback(const geometry_msgs::TwistStamped& msg) {
  v_mav_(0) = msg.twist.linear.x;
  v_mav_(1) = msg.twist.linear.y;
  v_mav_(2) = msg.twist.linear.z;
  updatePrimitives();
}

void trajectoryPublisher::seedDynamicReconfigure(trajectory_publisher::TrajectoryPublisherConfig& config) {
  config.trajName = trajectory_type_ == 0 ? TRAJ_FIGURE8_HORIZONTAL : clampTrajectoryType(trajectory_type_);
  config.omega_mode = std::max(TRAJ_OMEGA_FIXED, std::min(TRAJ_OMEGA_QUADRATIC, omega_mode_));
  config.omega_value = roundToFourDecimals(omega_value_);
  config.omega_start = roundToFourDecimals(omega_start_);
  config.omega_end = roundToFourDecimals(omega_end_);
  config.omega_duration = roundToFourDecimals(omega_duration_);
  config.path_preview_cycles = roundToFourDecimals(path_preview_cycles_);
  config.trajectory_yaw_lock = trajectory_yaw_lock_;
  config.trajectory_yaw_fixed = roundToFourDecimals(trajectory_yaw_fixed_);
  config.figure8_horizontal_omega_default = roundToFourDecimals(figure8_horizontal_omega_profile_.default_value);
  config.figure8_horizontal_omega_min = roundToFourDecimals(figure8_horizontal_omega_profile_.min);
  config.figure8_horizontal_omega_max = roundToFourDecimals(figure8_horizontal_omega_profile_.max);
  config.figure8_vertical_omega_default = roundToFourDecimals(figure8_vertical_omega_profile_.default_value);
  config.figure8_vertical_omega_min = roundToFourDecimals(figure8_vertical_omega_profile_.min);
  config.figure8_vertical_omega_max = roundToFourDecimals(figure8_vertical_omega_profile_.max);
  config.helix_flip_omega_default = roundToFourDecimals(helix_flip_omega_profile_.default_value);
  config.helix_flip_omega_min = roundToFourDecimals(helix_flip_omega_profile_.min);
  config.helix_flip_omega_max = roundToFourDecimals(helix_flip_omega_profile_.max);
  config.helix_flip_y_omega_default = roundToFourDecimals(helix_flip_y_omega_profile_.default_value);
  config.helix_flip_y_omega_min = roundToFourDecimals(helix_flip_y_omega_profile_.min);
  config.helix_flip_y_omega_max = roundToFourDecimals(helix_flip_y_omega_profile_.max);
  config.flip_loop_sine_omega_default = roundToFourDecimals(flip_loop_sine_omega_profile_.default_value);
  config.flip_loop_sine_omega_min = roundToFourDecimals(flip_loop_sine_omega_profile_.min);
  config.flip_loop_sine_omega_max = roundToFourDecimals(flip_loop_sine_omega_profile_.max);
  config.fast_circle_omega_default = roundToFourDecimals(fast_circle_omega_profile_.default_value);
  config.fast_circle_omega_min = roundToFourDecimals(fast_circle_omega_profile_.min);
  config.fast_circle_omega_max = roundToFourDecimals(fast_circle_omega_profile_.max);

  config.figure8_horizontal_Ax = roundToFourDecimals(shape_params_.figure8_horizontal_Ax);
  config.figure8_horizontal_Ay = roundToFourDecimals(shape_params_.figure8_horizontal_Ay);
  config.figure8_horizontal_Hc = roundToFourDecimals(shape_params_.figure8_horizontal_Hc);
  config.figure8_horizontal_theta0 = roundToFourDecimals(shape_params_.figure8_horizontal_theta0);
  config.figure8_vertical_Ay = roundToFourDecimals(shape_params_.figure8_vertical_Ay);
  config.figure8_vertical_Az = roundToFourDecimals(shape_params_.figure8_vertical_Az);
  config.figure8_vertical_Hc = roundToFourDecimals(shape_params_.figure8_vertical_Hc);
  config.figure8_vertical_theta0 = roundToFourDecimals(shape_params_.figure8_vertical_theta0);
  config.helix_flip_Ay = roundToFourDecimals(shape_params_.helix_flip_Ay);
  config.helix_flip_Az = roundToFourDecimals(shape_params_.helix_flip_Az);
  config.helix_flip_Hc = roundToFourDecimals(shape_params_.helix_flip_Hc);
  config.helix_flip_Vx = roundToFourDecimals(shape_params_.helix_flip_Vx);
  config.helix_flip_theta0 = roundToFourDecimals(shape_params_.helix_flip_theta0);
  config.helix_flip_y_Ax = roundToFourDecimals(shape_params_.helix_flip_y_Ax);
  config.helix_flip_y_Az = roundToFourDecimals(shape_params_.helix_flip_y_Az);
  config.helix_flip_y_Hc = roundToFourDecimals(shape_params_.helix_flip_y_Hc);
  config.helix_flip_y_Vy = roundToFourDecimals(shape_params_.helix_flip_y_Vy);
  config.helix_flip_y_theta0 = roundToFourDecimals(shape_params_.helix_flip_y_theta0);
  config.flip_loop_sine_Ay = roundToFourDecimals(shape_params_.flip_loop_sine_Ay);
  config.flip_loop_sine_Az = roundToFourDecimals(shape_params_.flip_loop_sine_Az);
  config.flip_loop_sine_Hc = roundToFourDecimals(shape_params_.flip_loop_sine_Hc);
  config.flip_loop_sine_Vx = roundToFourDecimals(shape_params_.flip_loop_sine_Vx);
  config.flip_loop_sine_theta0 = roundToFourDecimals(shape_params_.flip_loop_sine_theta0);
  config.fast_circle_Ax = roundToFourDecimals(shape_params_.fast_circle_Ax);
  config.fast_circle_Ay = roundToFourDecimals(shape_params_.fast_circle_Ay);
  config.fast_circle_Hc = roundToFourDecimals(shape_params_.fast_circle_Hc);
  config.fast_circle_theta0 = roundToFourDecimals(shape_params_.fast_circle_theta0);

  config.takeoff_before_trajectory = takeoff_before_trajectory_;
  config.takeoff_position_tolerance = roundToFourDecimals(takeoff_position_tolerance_);
  config.takeoff_velocity_tolerance = roundToFourDecimals(takeoff_velocity_tolerance_);
  config.adaptive_trajectory_start_ramp = adaptive_trajectory_start_ramp_;
  config.trajectory_start_ramp_duration = roundToFourDecimals(trajectory_start_ramp_configured_duration_);
  config.trajectory_start_ramp_min_duration = roundToFourDecimals(trajectory_start_ramp_min_duration_);
  config.trajectory_start_ramp_velocity_limit = roundToFourDecimals(trajectory_start_ramp_velocity_limit_);
  config.trajectory_start_ramp_acceleration_limit = roundToFourDecimals(trajectory_start_ramp_acceleration_limit_);
  config.trajectory_switch_transition_duration = roundToFourDecimals(trajectory_switch_transition_duration_);
  config.trajectory_switch_transition_min_duration = roundToFourDecimals(trajectory_switch_transition_min_duration_);
  config.trajectory_switch_transition_max_duration = roundToFourDecimals(trajectory_switch_transition_max_duration_);
  config.trajectory_switch_transition_velocity_limit = roundToFourDecimals(trajectory_switch_transition_velocity_limit_);
  config.trajectory_switch_transition_acceleration_limit =
      roundToFourDecimals(trajectory_switch_transition_acceleration_limit_);
  config.trajectory_switch_stop_speed_threshold = roundToFourDecimals(trajectory_switch_stop_speed_threshold_);
  config.trajectory_stop_deceleration_limit = roundToFourDecimals(trajectory_stop_deceleration_limit_);
  config.trajectory_stop_min_duration = roundToFourDecimals(trajectory_stop_min_duration_);
}

bool trajectoryPublisher::activeShapeGeometryChanged(
    const trajectory_publisher::TrajectoryPublisherConfig& config) const {
  switch (trajectory_type_) {
    case TRAJ_FIGURE8_VERTICAL:
      return shape_params_.figure8_vertical_Ay != config.figure8_vertical_Ay ||
             shape_params_.figure8_vertical_Az != config.figure8_vertical_Az ||
             shape_params_.figure8_vertical_Hc != config.figure8_vertical_Hc ||
             shape_params_.figure8_vertical_theta0 != config.figure8_vertical_theta0;
    case TRAJ_HELIX_FLIP:
      return shape_params_.helix_flip_Ay != config.helix_flip_Ay ||
             shape_params_.helix_flip_Az != config.helix_flip_Az ||
             shape_params_.helix_flip_Hc != config.helix_flip_Hc ||
             shape_params_.helix_flip_Vx != config.helix_flip_Vx ||
             shape_params_.helix_flip_theta0 != config.helix_flip_theta0;
    case TRAJ_HELIX_FLIP_Y:
      return shape_params_.helix_flip_y_Ax != config.helix_flip_y_Ax ||
             shape_params_.helix_flip_y_Az != config.helix_flip_y_Az ||
             shape_params_.helix_flip_y_Hc != config.helix_flip_y_Hc ||
             shape_params_.helix_flip_y_Vy != config.helix_flip_y_Vy ||
             shape_params_.helix_flip_y_theta0 != config.helix_flip_y_theta0;
    case TRAJ_FLIP_LOOP_SINE:
      return shape_params_.flip_loop_sine_Ay != config.flip_loop_sine_Ay ||
             shape_params_.flip_loop_sine_Az != config.flip_loop_sine_Az ||
             shape_params_.flip_loop_sine_Hc != config.flip_loop_sine_Hc ||
             shape_params_.flip_loop_sine_Vx != config.flip_loop_sine_Vx ||
             shape_params_.flip_loop_sine_theta0 != config.flip_loop_sine_theta0;
    case TRAJ_FAST_CIRCLE:
      return shape_params_.fast_circle_Ax != config.fast_circle_Ax ||
             shape_params_.fast_circle_Ay != config.fast_circle_Ay ||
             shape_params_.fast_circle_Hc != config.fast_circle_Hc ||
             shape_params_.fast_circle_theta0 != config.fast_circle_theta0;
    case TRAJ_FIGURE8_HORIZONTAL:
    default:
      return shape_params_.figure8_horizontal_Ax != config.figure8_horizontal_Ax ||
             shape_params_.figure8_horizontal_Ay != config.figure8_horizontal_Ay ||
             shape_params_.figure8_horizontal_Hc != config.figure8_horizontal_Hc ||
             shape_params_.figure8_horizontal_theta0 != config.figure8_horizontal_theta0;
  }
}

bool trajectoryPublisher::configChangesTrajectory(const trajectory_publisher::TrajectoryPublisherConfig& config) const {
  return trajectory_type_ != config.trajName || activeShapeGeometryChanged(config);
}

void trajectoryPublisher::updateOmegaProfilesFromConfig(
    const trajectory_publisher::TrajectoryPublisherConfig& config) {
  figure8_horizontal_omega_profile_ = {config.figure8_horizontal_omega_default, config.figure8_horizontal_omega_min,
                                       config.figure8_horizontal_omega_max};
  figure8_vertical_omega_profile_ = {config.figure8_vertical_omega_default, config.figure8_vertical_omega_min,
                                     config.figure8_vertical_omega_max};
  helix_flip_omega_profile_ = {config.helix_flip_omega_default, config.helix_flip_omega_min,
                               config.helix_flip_omega_max};
  helix_flip_y_omega_profile_ = {config.helix_flip_y_omega_default, config.helix_flip_y_omega_min,
                                 config.helix_flip_y_omega_max};
  flip_loop_sine_omega_profile_ = {config.flip_loop_sine_omega_default, config.flip_loop_sine_omega_min,
                                   config.flip_loop_sine_omega_max};
  fast_circle_omega_profile_ = {config.fast_circle_omega_default, config.fast_circle_omega_min,
                                config.fast_circle_omega_max};
}

void trajectoryPublisher::updateShapeParamsFromConfig(
    const trajectory_publisher::TrajectoryPublisherConfig& config) {
  shape_params_.figure8_horizontal_Ax = config.figure8_horizontal_Ax;
  shape_params_.figure8_horizontal_Ay = config.figure8_horizontal_Ay;
  shape_params_.figure8_horizontal_Hc = config.figure8_horizontal_Hc;
  shape_params_.figure8_horizontal_theta0 = config.figure8_horizontal_theta0;
  shape_params_.figure8_vertical_Ay = config.figure8_vertical_Ay;
  shape_params_.figure8_vertical_Az = config.figure8_vertical_Az;
  shape_params_.figure8_vertical_Hc = config.figure8_vertical_Hc;
  shape_params_.figure8_vertical_theta0 = config.figure8_vertical_theta0;
  shape_params_.helix_flip_Ay = config.helix_flip_Ay;
  shape_params_.helix_flip_Az = config.helix_flip_Az;
  shape_params_.helix_flip_Hc = config.helix_flip_Hc;
  shape_params_.helix_flip_Vx = config.helix_flip_Vx;
  shape_params_.helix_flip_theta0 = config.helix_flip_theta0;
  shape_params_.helix_flip_y_Ax = config.helix_flip_y_Ax;
  shape_params_.helix_flip_y_Az = config.helix_flip_y_Az;
  shape_params_.helix_flip_y_Hc = config.helix_flip_y_Hc;
  shape_params_.helix_flip_y_Vy = config.helix_flip_y_Vy;
  shape_params_.helix_flip_y_theta0 = config.helix_flip_y_theta0;
  shape_params_.flip_loop_sine_Ay = config.flip_loop_sine_Ay;
  shape_params_.flip_loop_sine_Az = config.flip_loop_sine_Az;
  shape_params_.flip_loop_sine_Hc = config.flip_loop_sine_Hc;
  shape_params_.flip_loop_sine_Vx = config.flip_loop_sine_Vx;
  shape_params_.flip_loop_sine_theta0 = config.flip_loop_sine_theta0;
  shape_params_.fast_circle_Ax = config.fast_circle_Ax;
  shape_params_.fast_circle_Ay = config.fast_circle_Ay;
  shape_params_.fast_circle_Hc = config.fast_circle_Hc;
  shape_params_.fast_circle_theta0 = config.fast_circle_theta0;
}

void trajectoryPublisher::dynamicReconfigureCallback(trajectory_publisher::TrajectoryPublisherConfig& config,
                                                     uint32_t level) {
  if (first_reconfigure_) {
    seedDynamicReconfigure(config);
    first_reconfigure_ = false;
    return;
  }

  config.trajName = clampTrajectoryType(config.trajName);
  config.omega_mode = std::max(TRAJ_OMEGA_FIXED, std::min(TRAJ_OMEGA_QUADRATIC, config.omega_mode));
  config.omega_value = roundToFourDecimals(config.omega_value);
  config.omega_start = roundToFourDecimals(config.omega_start);
  config.omega_end = roundToFourDecimals(config.omega_end);
  config.omega_duration = roundToFourDecimals(config.omega_duration);
  config.path_preview_cycles = std::max(1.0, roundToFourDecimals(config.path_preview_cycles));
  config.trajectory_yaw_fixed = roundToFourDecimals(config.trajectory_yaw_fixed);
  config.figure8_horizontal_omega_default = roundToFourDecimals(config.figure8_horizontal_omega_default);
  config.figure8_horizontal_omega_min = roundToFourDecimals(config.figure8_horizontal_omega_min);
  config.figure8_horizontal_omega_max = roundToFourDecimals(config.figure8_horizontal_omega_max);
  config.figure8_vertical_omega_default = roundToFourDecimals(config.figure8_vertical_omega_default);
  config.figure8_vertical_omega_min = roundToFourDecimals(config.figure8_vertical_omega_min);
  config.figure8_vertical_omega_max = roundToFourDecimals(config.figure8_vertical_omega_max);
  config.helix_flip_omega_default = roundToFourDecimals(config.helix_flip_omega_default);
  config.helix_flip_omega_min = roundToFourDecimals(config.helix_flip_omega_min);
  config.helix_flip_omega_max = roundToFourDecimals(config.helix_flip_omega_max);
  config.helix_flip_y_omega_default = roundToFourDecimals(config.helix_flip_y_omega_default);
  config.helix_flip_y_omega_min = roundToFourDecimals(config.helix_flip_y_omega_min);
  config.helix_flip_y_omega_max = roundToFourDecimals(config.helix_flip_y_omega_max);
  config.flip_loop_sine_omega_default = roundToFourDecimals(config.flip_loop_sine_omega_default);
  config.flip_loop_sine_omega_min = roundToFourDecimals(config.flip_loop_sine_omega_min);
  config.flip_loop_sine_omega_max = roundToFourDecimals(config.flip_loop_sine_omega_max);
  config.fast_circle_omega_default = roundToFourDecimals(config.fast_circle_omega_default);
  config.fast_circle_omega_min = roundToFourDecimals(config.fast_circle_omega_min);
  config.fast_circle_omega_max = roundToFourDecimals(config.fast_circle_omega_max);
  config.trajectory_start_ramp_duration = roundToFourDecimals(config.trajectory_start_ramp_duration);
  config.trajectory_start_ramp_min_duration = roundToFourDecimals(config.trajectory_start_ramp_min_duration);
  config.trajectory_start_ramp_velocity_limit = roundToFourDecimals(config.trajectory_start_ramp_velocity_limit);
  config.trajectory_start_ramp_acceleration_limit = roundToFourDecimals(config.trajectory_start_ramp_acceleration_limit);
  config.trajectory_switch_transition_duration = roundToFourDecimals(config.trajectory_switch_transition_duration);
  config.trajectory_switch_transition_min_duration =
      roundToFourDecimals(config.trajectory_switch_transition_min_duration);
  config.trajectory_switch_transition_max_duration =
      roundToFourDecimals(config.trajectory_switch_transition_max_duration);
  config.trajectory_switch_transition_velocity_limit =
      roundToFourDecimals(config.trajectory_switch_transition_velocity_limit);
  config.trajectory_switch_transition_acceleration_limit =
      roundToFourDecimals(config.trajectory_switch_transition_acceleration_limit);
  config.trajectory_switch_stop_speed_threshold = roundToFourDecimals(config.trajectory_switch_stop_speed_threshold);
  config.trajectory_stop_deceleration_limit = roundToFourDecimals(config.trajectory_stop_deceleration_limit);
  config.trajectory_stop_min_duration = roundToFourDecimals(config.trajectory_stop_min_duration);

  updateOmegaProfilesFromConfig(config);

  const bool trajectory_changed = trajectory_type_ != config.trajName;
  const bool geometry_changed = !trajectory_changed && activeShapeGeometryChanged(config);
  if (trajectory_changed) {
    config.omega_value = roundToFourDecimals(omegaDefaultForTrajectory(config.trajName));
    config.omega_start = roundToFourDecimals(omegaRangeForTrajectory(config.trajName).first);
    config.omega_end = roundToFourDecimals(omegaRangeForTrajectory(config.trajName).second);
  } else {
    config.omega_value = clampToOmegaRange(config.omega_value, config.trajName);
  }

  const bool omega_changed = omega_mode_ != config.omega_mode || omega_value_ != config.omega_value ||
                             omega_start_ != config.omega_start || omega_end_ != config.omega_end ||
                             omega_duration_ != config.omega_duration;
  if (!trajectory_changed && !geometry_changed && omega_changed) {
    rebaseOmegaSchedule();
  }

  trajectory_type_ = config.trajName;
  omega_mode_ = config.omega_mode;
  omega_value_ = config.omega_value;
  omega_start_ = config.omega_start;
  omega_end_ = config.omega_end;
  omega_duration_ = config.omega_duration;
  path_preview_cycles_ = config.path_preview_cycles;
  trajectory_yaw_lock_ = config.trajectory_yaw_lock;
  trajectory_yaw_fixed_ = config.trajectory_yaw_fixed;
  updateShapeParamsFromConfig(config);
  takeoff_before_trajectory_ = config.takeoff_before_trajectory;
  takeoff_position_tolerance_ = config.takeoff_position_tolerance;
  takeoff_velocity_tolerance_ = config.takeoff_velocity_tolerance;
  adaptive_trajectory_start_ramp_ = config.adaptive_trajectory_start_ramp;
  trajectory_start_ramp_configured_duration_ = config.trajectory_start_ramp_duration;
  trajectory_start_ramp_min_duration_ = config.trajectory_start_ramp_min_duration;
  trajectory_start_ramp_velocity_limit_ = config.trajectory_start_ramp_velocity_limit;
  trajectory_start_ramp_acceleration_limit_ = config.trajectory_start_ramp_acceleration_limit;
  trajectory_switch_transition_duration_ = config.trajectory_switch_transition_duration;
  trajectory_switch_transition_min_duration_ = config.trajectory_switch_transition_min_duration;
  trajectory_switch_transition_max_duration_ = config.trajectory_switch_transition_max_duration;
  trajectory_switch_transition_velocity_limit_ = config.trajectory_switch_transition_velocity_limit;
  trajectory_switch_transition_acceleration_limit_ = config.trajectory_switch_transition_acceleration_limit;
  trajectory_switch_stop_speed_threshold_ = config.trajectory_switch_stop_speed_threshold;
  trajectory_stop_deceleration_limit_ = config.trajectory_stop_deceleration_limit;
  trajectory_stop_min_duration_ = config.trajectory_stop_min_duration;

  if (trajectory_changed || geometry_changed) {
    if (tracking_requested_) {
      resetTrajectoryStart();
      ROS_INFO("Trajectory reconfigured: trajName=%s omega=%.4f, smooth transition to start.",
               shapetrajectory::typeName(trajectory_type_), omega_value_);
    } else {
      shape_phase_shift_ = 0.0;
      applyShapeParams();
      updateTakeoffTarget();
      ROS_INFO("Trajectory configured while stopped: trajName=%s omega=%.4f.",
               shapetrajectory::typeName(trajectory_type_), omega_value_);
    }
  } else {
    applyShapeParams();
    if (omega_changed) {
      ROS_INFO("Trajectory omega schedule restarted from its start value: trajName=%s omega_start=%.4f",
               shapetrajectory::typeName(trajectory_type_),
               omega_mode_ == TRAJ_OMEGA_FIXED ? omega_value_ : omega_start_);
    }
  }

}
