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

using namespace std;
using namespace Eigen;

trajectoryPublisher::trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      motion_selector_(0),
      first_reconfigure_(true),
      omega_value_param_present_(false) {
  trajectoryPub_ = nh_.advertise<nav_msgs::Path>("trajectory_publisher/trajectory", 1);
  referencePub_ = nh_.advertise<geometry_msgs::TwistStamped>("reference/setpoint", 1);
  flatreferencePub_ = nh_.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 1);
  rawreferencePub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
  positionreferencePub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
  global_rawreferencePub_ = nh_.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 1);
  motionselectorSub_ =
      nh_.subscribe("trajectory_publisher/motionselector", 1, &trajectoryPublisher::motionselectorCallback, this,
                    ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &trajectoryPublisher::mavposeCallback, this,
                              ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity", 1, &trajectoryPublisher::mavtwistCallback, this,
                               ros::TransportHints().tcpNoDelay());
  mavstate_sub_ = nh_.subscribe("mavros/state", 1, &trajectoryPublisher::mavstateCallback, this,
                                ros::TransportHints().tcpNoDelay());

  nh_private_.param<double>("updaterate", controlUpdate_dt_, 1.0 / 250.0);
  controlUpdate_dt_ = std::max(0.001, controlUpdate_dt_);
  trajloop_timer_ = nh_.createTimer(ros::Duration(0.1), &trajectoryPublisher::loopCallback, this);
  refloop_timer_ = nh_.createTimer(ros::Duration(controlUpdate_dt_), &trajectoryPublisher::refCallback, this);

  trajtriggerServ_ = nh_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);

  nh_private_.param<double>("initpos_x", init_pos_x_, 0.0);
  nh_private_.param<double>("initpos_y", init_pos_y_, 0.0);
  nh_private_.param<double>("initpos_z", init_pos_z_, 1.0);
  nh_private_.param<double>("horizon", primitive_duration_, 1.0);
  nh_private_.param<double>("maxjerk", max_jerk_, 10.0);
  omega_value_param_present_ = nh_private_.getParam("omegaValue", omega_value_);
  if (!omega_value_param_present_ && !nh_private_.getParam("shapeOmega", omega_value_)) {
    nh_private_.param<double>("shape_omega", omega_value_, 1.0);
  }
  readOmegaRanges();
  readShapeParams();
  if (!nh_private_.getParam("waitStartBeforeTrajectory", takeoff_before_trajectory_)) {
    nh_private_.param<bool>("takeoff_before_trajectory", takeoff_before_trajectory_, true);
  }
  if (!nh_private_.getParam("startPositionTolerance", takeoff_position_tolerance_)) {
    nh_private_.param<double>("takeoff_position_tolerance", takeoff_position_tolerance_, 0.25);
  }
  if (!nh_private_.getParam("startVelocityTolerance", takeoff_velocity_tolerance_)) {
    nh_private_.param<double>("takeoff_velocity_tolerance", takeoff_velocity_tolerance_, 0.5);
  }
  nh_private_.param<double>("startHoldDuration", start_hold_duration_, 3.0);
  if (!nh_private_.getParam("trajectoryStartRampDuration", trajectory_start_ramp_duration_)) {
    nh_private_.param<double>("trajectory_start_ramp_duration", trajectory_start_ramp_duration_, 2.0);
  }
  shape_trajectory_mode_ = nh_private_.getParam("trajName", trajectory_type_);
  if (!shape_trajectory_mode_) {
    nh_private_.param<int>("trajectory_type", trajectory_type_, TRAJ_FIGURE8_HORIZONTAL);
  }
  shape_trajectory_mode_ = shape_trajectory_mode_ || trajectory_type_ != 0;
  nh_private_.param<int>("number_of_primitives", num_primitives_, 7);
  nh_private_.param<int>("reference_type", pubreference_type_, 2);

  inputs_.resize(num_primitives_);

  if (!shape_trajectory_mode_) {  // Polynomial Trajectory

    if (num_primitives_ == 7) {
      inputs_.at(0) << 0.0, 0.0, 0.0;  // Constant jerk inputs for minimim time trajectories
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
  } else {  // Shape trajectories

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
  motion_selector_ = 0;
  trajectory_started_ = false;
  start_hold_started_ = false;
  start_time_ = ros::Time::now();

  initializePrimitives(trajectory_type_);
  updateTakeoffTarget();
  setTakeoffReference();

  dynamic_reconfigure::Server<trajectory_publisher::TrajectoryPublisherConfig>::CallbackType dyn_cb;
  dyn_cb = boost::bind(&trajectoryPublisher::dynamicReconfigureCallback, this, _1, _2);
  dyn_server_.setCallback(dyn_cb);
}

std::pair<double, double> trajectoryPublisher::readOmegaRange(const std::string& prefix, double default_min,
                                                              double default_max) {
  double range_min;
  double range_max;
  nh_private_.param<double>(prefix + "Min", range_min, default_min);
  nh_private_.param<double>(prefix + "Max", range_max, default_max);
  range_min = std::max(0.0, range_min);
  range_max = std::max(range_min, range_max);
  return std::make_pair(range_min, range_max);
}

void trajectoryPublisher::readOmegaRanges() {
  legacy_omega_range_ = readOmegaRange("legacyOmega", 0.05, 5.0);
  figure8_horizontal_omega_range_ = readOmegaRange("figure8HorizontalOmega", 0.5, 1.5);
  figure8_vertical_omega_range_ = readOmegaRange("figure8VerticalOmega", 0.5, 1.5);
  helix_flip_omega_range_ = readOmegaRange("helixFlipOmega", 2.0, 4.0);
  helix_flip_y_omega_range_ = readOmegaRange("helixFlipYOmega", 2.0, 4.0);
  flip_loop_sine_omega_range_ = readOmegaRange("flipLoopSineOmega", 2.0, 4.0);
  fast_circle_omega_range_ = readOmegaRange("fastCircleOmega", 1.5, 3.5);
  race_track_c_omega_range_ = readOmegaRange("raceTrackCOmega", 0.05, 5.0);
}

void trajectoryPublisher::readShapeParams() {
  nh_private_.param<double>("figure8HorizontalAx", shape_params_.figure8_horizontal_Ax, 2.0);
  nh_private_.param<double>("figure8HorizontalAy", shape_params_.figure8_horizontal_Ay, 2.0);
  nh_private_.param<double>("figure8HorizontalHc", shape_params_.figure8_horizontal_Hc, 3.0);
  nh_private_.param<double>("figure8HorizontalTheta0", shape_params_.figure8_horizontal_theta0, 0.0);
  nh_private_.param<double>("figure8VerticalAy", shape_params_.figure8_vertical_Ay, 2.0);
  nh_private_.param<double>("figure8VerticalAz", shape_params_.figure8_vertical_Az, 2.0);
  nh_private_.param<double>("figure8VerticalHc", shape_params_.figure8_vertical_Hc, 3.0);
  nh_private_.param<double>("figure8VerticalTheta0", shape_params_.figure8_vertical_theta0, -M_PI / 4.0);
  nh_private_.param<double>("helixFlipAy", shape_params_.helix_flip_Ay, 2.0);
  nh_private_.param<double>("helixFlipAz", shape_params_.helix_flip_Az, 2.0);
  nh_private_.param<double>("helixFlipHc", shape_params_.helix_flip_Hc, 3.0);
  nh_private_.param<double>("helixFlipVx", shape_params_.helix_flip_Vx, 0.30);
  nh_private_.param<double>("helixFlipTheta0", shape_params_.helix_flip_theta0, 0.0);
  nh_private_.param<double>("helixFlipYAx", shape_params_.helix_flip_y_Ax, 2.0);
  nh_private_.param<double>("helixFlipYAz", shape_params_.helix_flip_y_Az, 2.0);
  nh_private_.param<double>("helixFlipYHc", shape_params_.helix_flip_y_Hc, 3.0);
  nh_private_.param<double>("helixFlipYVy", shape_params_.helix_flip_y_Vy, 0.30);
  nh_private_.param<double>("helixFlipYTheta0", shape_params_.helix_flip_y_theta0, 0.0);
  nh_private_.param<double>("flipLoopSineAy", shape_params_.flip_loop_sine_Ay, 2.0);
  nh_private_.param<double>("flipLoopSineAz", shape_params_.flip_loop_sine_Az, 2.0);
  nh_private_.param<double>("flipLoopSineHc", shape_params_.flip_loop_sine_Hc, 3.0);
  nh_private_.param<double>("flipLoopSineVx", shape_params_.flip_loop_sine_Vx, 0.0);
  nh_private_.param<double>("flipLoopSineTheta0", shape_params_.flip_loop_sine_theta0, 0.0);
  nh_private_.param<double>("fastCircleAx", shape_params_.fast_circle_Ax, 3.0);
  nh_private_.param<double>("fastCircleAy", shape_params_.fast_circle_Ay, 3.0);
  nh_private_.param<double>("fastCircleHc", shape_params_.fast_circle_Hc, 3.0);
  nh_private_.param<double>("fastCircleTheta0", shape_params_.fast_circle_theta0, 0.0);
}

std::pair<double, double> trajectoryPublisher::omegaRangeForTrajectory(int type) const {
  switch (type) {
    case TRAJ_FIGURE8_HORIZONTAL:
      return figure8_horizontal_omega_range_;
    case TRAJ_FIGURE8_VERTICAL:
      return figure8_vertical_omega_range_;
    case TRAJ_HELIX_FLIP:
      return helix_flip_omega_range_;
    case TRAJ_HELIX_FLIP_Y:
      return helix_flip_y_omega_range_;
    case TRAJ_FLIP_LOOP_SINE:
      return flip_loop_sine_omega_range_;
    case TRAJ_FAST_CIRCLE:
      return fast_circle_omega_range_;
    case TRAJ_RACE_TRACK_C:
      return race_track_c_omega_range_;
    default:
      return legacy_omega_range_;
  }
}

double trajectoryPublisher::clampToOmegaRange(double value, int type) const {
  const std::pair<double, double> range = omegaRangeForTrajectory(type);
  return std::max(range.first, std::min(range.second, value));
}

bool trajectoryPublisher::activeShapeGeometryChanged(
    const trajectory_publisher::TrajectoryPublisherConfig& config) const {
  switch (trajectory_type_) {
    case TRAJ_FIGURE8_HORIZONTAL:
      return shape_params_.figure8_horizontal_Ax != config.figure8HorizontalAx ||
             shape_params_.figure8_horizontal_Ay != config.figure8HorizontalAy ||
             shape_params_.figure8_horizontal_Hc != config.figure8HorizontalHc ||
             shape_params_.figure8_horizontal_theta0 != config.figure8HorizontalTheta0;
    case TRAJ_FIGURE8_VERTICAL:
      return shape_params_.figure8_vertical_Ay != config.figure8VerticalAy ||
             shape_params_.figure8_vertical_Az != config.figure8VerticalAz ||
             shape_params_.figure8_vertical_Hc != config.figure8VerticalHc ||
             shape_params_.figure8_vertical_theta0 != config.figure8VerticalTheta0;
    case TRAJ_HELIX_FLIP:
      return shape_params_.helix_flip_Ay != config.helixFlipAy ||
             shape_params_.helix_flip_Az != config.helixFlipAz ||
             shape_params_.helix_flip_Hc != config.helixFlipHc ||
             shape_params_.helix_flip_Vx != config.helixFlipVx ||
             shape_params_.helix_flip_theta0 != config.helixFlipTheta0;
    case TRAJ_HELIX_FLIP_Y:
      return shape_params_.helix_flip_y_Ax != config.helixFlipYAx ||
             shape_params_.helix_flip_y_Az != config.helixFlipYAz ||
             shape_params_.helix_flip_y_Hc != config.helixFlipYHc ||
             shape_params_.helix_flip_y_Vy != config.helixFlipYVy ||
             shape_params_.helix_flip_y_theta0 != config.helixFlipYTheta0;
    case TRAJ_FLIP_LOOP_SINE:
      return shape_params_.flip_loop_sine_Ay != config.flipLoopSineAy ||
             shape_params_.flip_loop_sine_Az != config.flipLoopSineAz ||
             shape_params_.flip_loop_sine_Hc != config.flipLoopSineHc ||
             shape_params_.flip_loop_sine_Vx != config.flipLoopSineVx ||
             shape_params_.flip_loop_sine_theta0 != config.flipLoopSineTheta0;
    case TRAJ_FAST_CIRCLE:
      return shape_params_.fast_circle_Ax != config.fastCircleAx ||
             shape_params_.fast_circle_Ay != config.fastCircleAy ||
             shape_params_.fast_circle_Hc != config.fastCircleHc ||
             shape_params_.fast_circle_theta0 != config.fastCircleTheta0;
    default:
      return false;
  }
}

void trajectoryPublisher::updateShapeParamsFromConfig(
    const trajectory_publisher::TrajectoryPublisherConfig& config) {
  shape_params_.figure8_horizontal_Ax = config.figure8HorizontalAx;
  shape_params_.figure8_horizontal_Ay = config.figure8HorizontalAy;
  shape_params_.figure8_horizontal_Hc = config.figure8HorizontalHc;
  shape_params_.figure8_horizontal_theta0 = config.figure8HorizontalTheta0;
  shape_params_.figure8_vertical_Ay = config.figure8VerticalAy;
  shape_params_.figure8_vertical_Az = config.figure8VerticalAz;
  shape_params_.figure8_vertical_Hc = config.figure8VerticalHc;
  shape_params_.figure8_vertical_theta0 = config.figure8VerticalTheta0;
  shape_params_.helix_flip_Ay = config.helixFlipAy;
  shape_params_.helix_flip_Az = config.helixFlipAz;
  shape_params_.helix_flip_Hc = config.helixFlipHc;
  shape_params_.helix_flip_Vx = config.helixFlipVx;
  shape_params_.helix_flip_theta0 = config.helixFlipTheta0;
  shape_params_.helix_flip_y_Ax = config.helixFlipYAx;
  shape_params_.helix_flip_y_Az = config.helixFlipYAz;
  shape_params_.helix_flip_y_Hc = config.helixFlipYHc;
  shape_params_.helix_flip_y_Vy = config.helixFlipYVy;
  shape_params_.helix_flip_y_theta0 = config.helixFlipYTheta0;
  shape_params_.flip_loop_sine_Ay = config.flipLoopSineAy;
  shape_params_.flip_loop_sine_Az = config.flipLoopSineAz;
  shape_params_.flip_loop_sine_Hc = config.flipLoopSineHc;
  shape_params_.flip_loop_sine_Vx = config.flipLoopSineVx;
  shape_params_.flip_loop_sine_theta0 = config.flipLoopSineTheta0;
  shape_params_.fast_circle_Ax = config.fastCircleAx;
  shape_params_.fast_circle_Ay = config.fastCircleAy;
  shape_params_.fast_circle_Hc = config.fastCircleHc;
  shape_params_.fast_circle_theta0 = config.fastCircleTheta0;
}

void trajectoryPublisher::updateReference() {
  curr_time_ = ros::Time::now();
  if (!takeoff_before_trajectory_) {
    if (current_state_.mode != "OFFBOARD") {  /// Reset start_time_ when not in offboard
      start_time_ = ros::Time::now();
    }
    trigger_time_ = (curr_time_ - start_time_).toSec();

    p_targ = motionPrimitives_.at(motion_selector_)->getPosition(trigger_time_);
    v_targ = motionPrimitives_.at(motion_selector_)->getVelocity(trigger_time_);
    if (pubreference_type_ != 0) {
      a_targ = motionPrimitives_.at(motion_selector_)->getAcceleration(trigger_time_);
      j_targ = motionPrimitives_.at(motion_selector_)->getJerk(trigger_time_);
    }
    return;
  }

  if (current_state_.mode != "OFFBOARD") {
    trajectory_started_ = false;
    start_hold_started_ = false;
    start_time_ = curr_time_;
    setTakeoffReference();
    return;
  }

  if (!trajectory_started_) {
    if (!current_state_.armed || !takeoffTargetReached()) {
      start_hold_started_ = false;
      setTakeoffReference();
      return;
    }

    if (start_hold_duration_ > 0.0) {
      if (!start_hold_started_) {
        start_hold_begin_time_ = curr_time_;
        start_hold_started_ = true;
        ROS_INFO("Trajectory start point reached, holding for %.2f seconds.", start_hold_duration_);
      }
      if ((curr_time_ - start_hold_begin_time_).toSec() < start_hold_duration_) {
        setTakeoffReference();
        return;
      }
    }

    start_time_ = curr_time_;
    trajectory_started_ = true;
    start_hold_started_ = false;
    ROS_INFO("Start hold complete, starting trajectory.");
  }

  trigger_time_ = (curr_time_ - start_time_).toSec();

  p_targ = motionPrimitives_.at(motion_selector_)->getPosition(trigger_time_);
  v_targ = motionPrimitives_.at(motion_selector_)->getVelocity(trigger_time_);
  if (pubreference_type_ != 0) {
    a_targ = motionPrimitives_.at(motion_selector_)->getAcceleration(trigger_time_);
    j_targ = motionPrimitives_.at(motion_selector_)->getJerk(trigger_time_);
  }
  applyTrajectoryStartRamp(trigger_time_);
}

void trajectoryPublisher::initializePrimitives(int type) {
  if (!shape_trajectory_mode_) {
    for (int i = 0; i < motionPrimitives_.size(); i++)
      motionPrimitives_.at(i)->generatePrimitives(p_mav_, v_mav_, inputs_.at(i));
  } else {
    for (int i = 0; i < motionPrimitives_.size(); i++) {
      std::shared_ptr<shapetrajectory> shape = std::dynamic_pointer_cast<shapetrajectory>(motionPrimitives_.at(i));
      if (shape) {
        shape->setType(trajectory_type_);
        shape->setParams(shape_params_);
        shape->setPhaseShift(shape_phase_shift_);
        shape->initPrimitives(shape_origin_, shape_axis_, omega_value_);
      }
    }
  }
}

void trajectoryPublisher::applyShapeParams() {
  for (int i = 0; i < motionPrimitives_.size(); i++) {
    std::shared_ptr<shapetrajectory> shape = std::dynamic_pointer_cast<shapetrajectory>(motionPrimitives_.at(i));
    if (shape) {
      shape->setType(trajectory_type_);
      shape->setParams(shape_params_);
      shape->setPhaseShift(shape_phase_shift_);
      shape->initPrimitives(shape_origin_, shape_axis_, omega_value_);
    }
  }
}

void trajectoryPublisher::resetTrajectoryStart() {
  shape_phase_shift_ = 0.0;
  applyShapeParams();
  updateTakeoffTarget();
  trajectory_started_ = false;
  start_hold_started_ = false;
  start_time_ = ros::Time::now();
  setTakeoffReference();
}

void trajectoryPublisher::updatePrimitives() {
  for (int i = 0; i < motionPrimitives_.size(); i++) motionPrimitives_.at(i)->generatePrimitives(p_mav_, v_mav_);
}

void trajectoryPublisher::updateTakeoffTarget() {
  if (!shape_trajectory_mode_) {
    takeoff_target_ << init_pos_x_, init_pos_y_, init_pos_z_;
  } else {
    takeoff_target_ = motionPrimitives_.at(motion_selector_)->getPosition(0.0);
  }
}

void trajectoryPublisher::setTakeoffReference() {
  p_targ = takeoff_target_;
  v_targ << 0.0, 0.0, 0.0;
  a_targ << 0.0, 0.0, 0.0;
  j_targ << 0.0, 0.0, 0.0;
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
      (60.0 - 360.0 * x + 360.0 * x2) /
      std::pow(trajectory_start_ramp_duration_, 3.0);

  const Eigen::Vector3d p_nom = p_targ;
  const Eigen::Vector3d v_nom = v_targ;
  const Eigen::Vector3d a_nom = a_targ;
  const Eigen::Vector3d j_nom = j_targ;
  const Eigen::Vector3d dp = p_nom - takeoff_target_;

  p_targ = takeoff_target_ + scale * dp;
  v_targ = scale * v_nom + scale_dot * dp;
  a_targ = scale * a_nom + 2.0 * scale_dot * v_nom + scale_ddot * dp;
  j_targ = scale * j_nom + 3.0 * scale_dot * a_nom + 3.0 * scale_ddot * v_nom + scale_dddot * dp;
}

void trajectoryPublisher::pubrefTrajectory(int selector) {
  // Publish current trajectory the publisher is publishing
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
  flatreferencePub_.publish(msg);
}

void trajectoryPublisher::pubrefSetpointRaw() {
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = 0;
  msg.position.x = p_targ(0);
  msg.position.y = p_targ(1);
  msg.position.z = p_targ(2);
  msg.velocity.x = v_targ(0);
  msg.velocity.y = v_targ(1);
  msg.velocity.z = v_targ(2);
  msg.acceleration_or_force.x = a_targ(0);
  msg.acceleration_or_force.y = a_targ(1);
  msg.acceleration_or_force.z = a_targ(2);
  rawreferencePub_.publish(msg);
}

void trajectoryPublisher::pubrefPositionSetpoint() {
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = p_targ(0);
  msg.pose.position.y = p_targ(1);
  msg.pose.position.z = p_targ(2);
  msg.pose.orientation.w = 1.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  positionreferencePub_.publish(msg);
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
  global_rawreferencePub_.publish(msg);
}

void trajectoryPublisher::mavstateCallback(const mavros_msgs::State::ConstPtr& msg) { current_state_ = *msg; }

void trajectoryPublisher::loopCallback(const ros::TimerEvent& event) {
  // Slow Loop publishing trajectory information
  pubrefTrajectory(motion_selector_);
  pubprimitiveTrajectory();
}

void trajectoryPublisher::refCallback(const ros::TimerEvent& event) {
  // Fast Loop publishing reference states
  updateReference();
  switch (pubreference_type_) {
    case REF_TWIST:
      pubrefState();
      break;
    case REF_SETPOINTRAW:
      pubrefSetpointRaw();
      // pubrefSetpointRawGlobal();
      break;
    case REF_POSITION:
      pubrefPositionSetpoint();
      break;
    default:
      pubflatrefState();
      break;
  }
}

bool trajectoryPublisher::triggerCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  unsigned char mode = req.data;

  resetTrajectoryStart();
  res.success = true;
  res.message = "trajectory triggered";
  return true;
}

void trajectoryPublisher::motionselectorCallback(const std_msgs::Int32& selector_msg) {
  motion_selector_ = selector_msg.data;
  updatePrimitives();
  resetTrajectoryStart();
}

void trajectoryPublisher::mavposeCallback(const geometry_msgs::PoseStamped& msg) {
  p_mav_(0) = msg.pose.position.x;
  p_mav_(1) = msg.pose.position.y;
  p_mav_(2) = msg.pose.position.z;
  updatePrimitives();
}

void trajectoryPublisher::mavtwistCallback(const geometry_msgs::TwistStamped& msg) {
  v_mav_(0) = msg.twist.linear.x;
  v_mav_(1) = msg.twist.linear.y;
  v_mav_(2) = msg.twist.linear.z;
  updatePrimitives();
}

void trajectoryPublisher::dynamicReconfigureCallback(trajectory_publisher::TrajectoryPublisherConfig& config,
                                                     uint32_t level) {
  if (first_reconfigure_ && !omega_value_param_present_) {
    config.omegaValue = omega_value_;
  }
  config.omegaValue = clampToOmegaRange(config.omegaValue, config.trajName);

  const bool trajectory_changed = trajectory_type_ != config.trajName;
  const bool geometry_changed = !trajectory_changed && activeShapeGeometryChanged(config);
  const bool speed_changed = omega_value_ != config.omegaValue;
  const bool restart = trajectory_changed || geometry_changed;

  if (speed_changed && !restart && (!takeoff_before_trajectory_ || trajectory_started_)) {
    const double elapsed_time = std::max(0.0, (ros::Time::now() - start_time_).toSec());
    shape_phase_shift_ += (omega_value_ - config.omegaValue) * elapsed_time;
  }

  trajectory_type_ = config.trajName;
  omega_value_ = config.omegaValue;
  updateShapeParamsFromConfig(config);
  trajectory_start_ramp_duration_ = config.trajectoryStartRampDuration;
  takeoff_before_trajectory_ = config.waitStartBeforeTrajectory;
  takeoff_position_tolerance_ = config.startPositionTolerance;
  takeoff_velocity_tolerance_ = config.startVelocityTolerance;
  start_hold_duration_ = config.startHoldDuration;
  first_reconfigure_ = false;

  if (restart) {
    resetTrajectoryStart();
    ROS_INFO("Trajectory geometry reconfigured: trajName=%d omega=%.2f rad/s", trajectory_type_, omega_value_);
  } else {
    applyShapeParams();
    if (speed_changed) {
      ROS_INFO("Trajectory speed reconfigured: trajName=%d omega=%.2f rad/s", trajectory_type_, omega_value_);
    }
  }
}
