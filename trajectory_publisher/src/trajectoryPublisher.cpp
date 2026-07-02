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

namespace {
bool usesLegacyShapeOmega(int trajectory_type) {
  return trajectory_type == TRAJ_CIRCLE || trajectory_type == TRAJ_LAMNISCATE || trajectory_type == TRAJ_STATIONARY;
}
}  // namespace

trajectoryPublisher::trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), motion_selector_(0) {
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

  trajloop_timer_ = nh_.createTimer(ros::Duration(0.1), &trajectoryPublisher::loopCallback, this);
  refloop_timer_ = nh_.createTimer(ros::Duration(0.01), &trajectoryPublisher::refCallback, this);

  trajtriggerServ_ = nh_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);

  nh_private_.param<double>("initpos_x", init_pos_x_, 0.0);
  nh_private_.param<double>("initpos_y", init_pos_y_, 0.0);
  nh_private_.param<double>("initpos_z", init_pos_z_, 1.0);
  nh_private_.param<double>("updaterate", controlUpdate_dt_, 0.01);
  nh_private_.param<double>("horizon", primitive_duration_, 1.0);
  nh_private_.param<double>("maxjerk", max_jerk_, 10.0);
  if (!nh_private_.getParam("shapeOmega", shape_omega_)) {
    nh_private_.param<double>("shape_omega", shape_omega_, 1.5);
  }
  if (!nh_private_.getParam("trajIntensity", traj_intensity_)) {
    nh_private_.param<double>("traj_intensity", traj_intensity_, 0.75);
  }
  if (!nh_private_.getParam("Tend", traj_base_duration_)) {
    nh_private_.param<double>("traj_base_duration", traj_base_duration_, 10.0);
  }
  if (!nh_private_.getParam("helixTurns", helix_turns_)) {
    nh_private_.param<double>("helix_turns", helix_turns_, 5.0);
  }
  if (!nh_private_.getParam("raceTrackMaxSpeed", race_track_max_speed_)) {
    nh_private_.param<double>("race_track_max_speed", race_track_max_speed_, 19.4);
  }
  if (!nh_private_.getParam("trajectorySpeed", trajectory_speed_)) {
    trajectory_speed_ = std::max(0.05, 2.0 * shape_omega_);
  }
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
    nh_private_.param<int>("trajectory_type", trajectory_type_, 6);
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

void trajectoryPublisher::updateReference() {
  curr_time_ = ros::Time::now();
  if (!takeoff_before_trajectory_) {
    if (current_state_.mode != "OFFBOARD") {  /// Reset start_time_ when not in offboard
      start_time_ = ros::Time::now();
    }
    trigger_time_ = (curr_time_ - start_time_).toSec();

    p_targ = motionPrimitives_.at(motion_selector_)->getPosition(trigger_time_);
    v_targ = motionPrimitives_.at(motion_selector_)->getVelocity(trigger_time_);
    if (pubreference_type_ != 0) a_targ = motionPrimitives_.at(motion_selector_)->getAcceleration(trigger_time_);
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
  if (pubreference_type_ != 0) a_targ = motionPrimitives_.at(motion_selector_)->getAcceleration(trigger_time_);
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
        shape->setBenchmarkParams(traj_intensity_, traj_base_duration_, helix_turns_, race_track_max_speed_);
        shape->setTrajectorySpeed(trajectory_speed_);
        const double omega = usesLegacyShapeOmega(trajectory_type_) ? shape_omega_ : trajectory_speed_ / 2.0;
        shape->initPrimitives(shape_origin_, shape_axis_, omega);
      }
    }
    // TODO: Pass in parameters for primitive trajectories
  }
}

void trajectoryPublisher::applyShapeParams() {
  for (int i = 0; i < motionPrimitives_.size(); i++) {
    std::shared_ptr<shapetrajectory> shape = std::dynamic_pointer_cast<shapetrajectory>(motionPrimitives_.at(i));
    if (shape) {
      shape->setType(trajectory_type_);
      shape->setBenchmarkParams(traj_intensity_, traj_base_duration_, helix_turns_, race_track_max_speed_);
      shape->setTrajectorySpeed(trajectory_speed_);
      const double omega = usesLegacyShapeOmega(trajectory_type_) ? shape_omega_ : trajectory_speed_ / 2.0;
      shape->initPrimitives(shape_origin_, shape_axis_, omega);
    }
  }
}

void trajectoryPublisher::resetTrajectoryStart() {
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

  const Eigen::Vector3d p_nom = p_targ;
  const Eigen::Vector3d v_nom = v_targ;
  const Eigen::Vector3d a_nom = a_targ;
  const Eigen::Vector3d dp = p_nom - takeoff_target_;

  p_targ = takeoff_target_ + scale * dp;
  v_targ = scale * v_nom + scale_dot * dp;
  a_targ = scale * a_nom + 2.0 * scale_dot * v_nom + scale_ddot * dp;
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
  const bool restart = trajectory_type_ != config.trajName ||
                       traj_intensity_ != config.trajIntensity || traj_base_duration_ != config.Tend ||
                       helix_turns_ != config.helixTurns || race_track_max_speed_ != config.raceTrackMaxSpeed ||
                       trajectory_speed_ != config.trajectorySpeed ||
                       trajectory_start_ramp_duration_ != config.trajectoryStartRampDuration ||
                       takeoff_before_trajectory_ != config.waitStartBeforeTrajectory ||
                       start_hold_duration_ != config.startHoldDuration;

  trajectory_type_ = config.trajName;
  shape_omega_ = config.shapeOmega;
  traj_intensity_ = config.trajIntensity;
  traj_base_duration_ = config.Tend;
  helix_turns_ = config.helixTurns;
  race_track_max_speed_ = config.raceTrackMaxSpeed;
  trajectory_speed_ = config.trajectorySpeed;
  trajectory_start_ramp_duration_ = config.trajectoryStartRampDuration;
  takeoff_before_trajectory_ = config.waitStartBeforeTrajectory;
  takeoff_position_tolerance_ = config.startPositionTolerance;
  takeoff_velocity_tolerance_ = config.startVelocityTolerance;
  start_hold_duration_ = config.startHoldDuration;

  applyShapeParams();
  if (restart) {
    resetTrajectoryStart();
    ROS_INFO("Trajectory reconfigured: trajName=%d speed=%.2f m/s trajIntensity=%.2f", trajectory_type_,
             trajectory_speed_, traj_intensity_);
  }
}
