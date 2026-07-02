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
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "geometric_controller/geometric_controller.h"
#include "geometric_controller/jerk_tracking_control.h"
#include "geometric_controller/nonlinear_attitude_control.h"
#include "geometric_controller/nonlinear_geometric_control.h"

#include <algorithm>
#include <cmath>

using namespace Eigen;
using namespace std;
// Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), node_state(WAITING_FOR_HOME_POSE) {
  referenceSub_ =
      nh_.subscribe("reference/setpoint", 1, &geometricCtrl::targetCallback, this, ros::TransportHints().tcpNoDelay());
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  yawreferenceSub_ =
      nh_.subscribe("reference/yaw", 1, &geometricCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
  multiDOFJointSub_ = nh_.subscribe("command/trajectory", 1, &geometricCtrl::multiDOFJointCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  mavstateSub_ =
      nh_.subscribe("mavros/state", 1, &geometricCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,
                              ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &geometricCtrl::mavtwistCallback, this,
                               ros::TransportHints().tcpNoDelay());
  ctrltriggerServ_ = nh_.advertiseService("trigger_rlcontroller", &geometricCtrl::ctrltriggerCallback, this);
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback,
                                   this);  // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback,
                                      this);  // Define timer for constant loop rate

  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  posehistoryPub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_service_ = nh_.advertiseService("land", &geometricCtrl::landCallback, this);

  if (!nh_private_.getParam("mav_name", mav_name_)) {
    nh_private_.param<string>("mavname", mav_name_, "iris");
  }
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("offboard_manager_only", offboard_manager_only_, false);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 10.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);

  double dx, dy, dz;
  nh_private_.param<double>("drag_dx", dx, 0.0);
  nh_private_.param<double>("drag_dy", dy, 0.0);
  nh_private_.param<double>("drag_dz", dz, 0.0);
  D_ << dx, dy, dz;

  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.02202);
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.0);
  nh_private_.param<int>("controllerName", controller_type_, 0);
  const bool has_attctrl_constant = nh_private_.getParam("attctrl_constant", attctrl_tau_);
  nh_private_.param<double>("Kp_x", Kpos_x_, 10.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 10.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 6.0);
  nh_private_.param<double>("Kv_y", Kvel_y_, 6.0);
  nh_private_.param<double>("Kv_z", Kvel_z_, 6.0);
  nh_private_.param<double>("KR_x", KR_x_, 3.0);
  nh_private_.param<double>("KR_y", KR_y_, 3.0);
  nh_private_.param<double>("KR_z", KR_z_, 2.0);
  if (!nh_private_.getParam("indiAccelFeedback", indi_accel_feedback_)) {
    nh_private_.param<double>("indi_accel_feedback", indi_accel_feedback_, 0.25);
  }
  if (!nh_private_.getParam("indiFilterCutoffHz", indi_filter_cutoff_hz_)) {
    nh_private_.param<double>("indi_filter_cutoff_hz", indi_filter_cutoff_hz_, 30.0);
  }
  if (!nh_private_.getParam("indiMaxCorrectionAcc", indi_max_correction_acc_)) {
    nh_private_.param<double>("indi_max_correction_acc", indi_max_correction_acc_, 5.0);
  }
  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
  nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
  nh_private_.param<double>("init_pos_z", initTargetPos_z_, 2.0);
  nh_private_.param<bool>("use_position_takeoff", use_position_takeoff_, true);
  nh_private_.param<double>("position_takeoff_tolerance", position_takeoff_tolerance_, 0.25);
  nh_private_.param<double>("position_takeoff_velocity_tolerance", position_takeoff_velocity_tolerance_, 0.35);
  nh_private_.param<double>("position_takeoff_hold_duration", position_takeoff_hold_duration_, 4.0);

  targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
  position_takeoff_target_ = targetPos_;
  latest_reference_pos_ = targetPos_;
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  filteredAcc_ << 0.0, 0.0, 0.0;
  previousVel_ << 0.0, 0.0, 0.0;
  updateControllerGains();
  if (!has_attctrl_constant) {
    attctrl_tau_ = attitudeTauFromKR();
  }

  bool jerk_enabled = false;
  if (!jerk_enabled) {
    if (ctrl_mode_ == ERROR_GEOMETRIC) {
      controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau_);
    } else {
      controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau_);
    }
  } else {
    controller_ = std::make_shared<JerkTrackingControl>();
  }
}
geometricCtrl::~geometricCtrl() {
  // Destructor
}

void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped &msg) {
  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.twist.angular);
  targetVel_ = toEigen(msg.twist.linear);
  latest_reference_pos_ = targetPos_;
  received_reference_ = true;

  if (reference_request_dt_ > 0)
    targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
  else
    targetAcc_ = Eigen::Vector3d::Zero();
}

void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget &msg) {
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.position);
  targetVel_ = toEigen(msg.velocity);
  latest_reference_pos_ = targetPos_;
  received_reference_ = true;

  if (msg.type_mask == 1) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = Eigen::Vector3d::Zero();

  } else if (msg.type_mask == 2) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

  } else if (msg.type_mask == 4) {
    targetAcc_ = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

  } else {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = toEigen(msg.snap);
  }
}

void geometricCtrl::yawtargetCallback(const std_msgs::Float32 &msg) {
  if (!velocity_yaw_) mavYaw_ = double(msg.data);
}

void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) {
  trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;
  latest_reference_pos_ = targetPos_;
  received_reference_ = true;

  targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  targetJerk_ = Eigen::Vector3d::Zero();
  targetSnap_ = Eigen::Vector3d::Zero();

  if (!velocity_yaw_) {
    Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y,
                         pt.transforms[0].rotation.z);
    Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
    mavYaw_ = rpy(2);
  }
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
  if (!received_home_pose) {
    received_home_pose = true;
    home_pose_ = msg.pose;
    ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  mavPos_ = toEigen(msg.pose.position);
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  mavVel_ = toEigen(msg.twist.linear);
  mavRate_ = toEigen(msg.twist.angular);
}

bool geometricCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  node_state = LANDING;
  return true;
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent &event) {
  switch (node_state) {
    case WAITING_FOR_HOME_POSE:
      waitForPredicate(&received_home_pose, "Waiting for home pose...");
      ROS_INFO("Got pose! Drone Ready to be armed.");
      node_state = MISSION_EXECUTION;
      break;

    case MISSION_EXECUTION: {
      if (offboard_manager_only_) {
        appendPoseHistory();
        pubPoseHistory();
        break;
      }

      if (velocity_yaw_) {
        mavYaw_ = getVelocityYaw(mavVel_);
      }

      if (use_position_takeoff_ && !position_takeoff_complete_) {
        if (received_reference_ && !position_takeoff_target_locked_) {
          position_takeoff_target_ = latest_reference_pos_;
          position_takeoff_target_locked_ = true;
          ROS_INFO_STREAM("Position takeoff target locked to first reference: " << position_takeoff_target_.transpose());
        }
        targetPos_ = position_takeoff_target_;
        targetVel_.setZero();
        targetAcc_.setZero();
        q_des << 1.0, 0.0, 0.0, 0.0;
        pubReferencePose(targetPos_, q_des);
        pubPositionTakeoffSetpoint();

        if (positionTakeoffReady()) {
          if (!position_takeoff_hold_started_) {
            position_takeoff_hold_started_ = true;
            position_takeoff_hold_begin_ = ros::Time::now();
            ROS_INFO("Position takeoff target reached, holding before body-rate tracking.");
          } else if ((ros::Time::now() - position_takeoff_hold_begin_).toSec() >=
                     position_takeoff_hold_duration_) {
            position_takeoff_complete_ = true;
            position_takeoff_hold_started_ = false;
            desired_att_initialized_ = false;
            accel_indi_initialized_ = false;
            ROS_INFO("Position takeoff complete, switching to %s body-rate tracking.", controllerName());
          }
        } else {
          position_takeoff_hold_started_ = false;
        }

        appendPoseHistory();
        pubPoseHistory();
        break;
      }

      if (feedthrough_enable_) {
        computeBodyRateCmd(cmdBodyRate_, targetAcc_);
      } else {
        switch (controller_type_) {
          case 1:
            cmdBodyRate_ = controllerPDGeometric();
            break;
          case 2:
            cmdBodyRate_ = controllerLee();
            break;
          case 3:
            cmdBodyRate_ = controllerJohnson();
            break;
          case 4:
            cmdBodyRate_ = controllerSunDFBC();
            break;
          case 5:
            cmdBodyRate_ = controllerSunDFBCINDI();
            break;
          case 6:
            cmdBodyRate_ = controllerTal();
            break;
          case 7:
            cmdBodyRate_ = controllerGeometricINDI();
            break;
          default:
            computeBodyRateCmd(cmdBodyRate_, controlPosition(targetPos_, targetVel_, targetAcc_));
            break;
        }
      }
      pubReferencePose(targetPos_, q_des);
      pubRateCommands(cmdBodyRate_, q_des);
      appendPoseHistory();
      pubPoseHistory();
      break;
    }

    case LANDING: {
      geometry_msgs::PoseStamped landingmsg;
      landingmsg.header.stamp = ros::Time::now();
      landingmsg.pose = home_pose_;
      landingmsg.pose.position.z = landingmsg.pose.position.z + 1.0;
      target_pose_pub_.publish(landingmsg);
      node_state = LANDED;
      ros::spinOnce();
      break;
    }
    case LANDED:
      ROS_INFO("Landed. Please set to position control and disarm.");
      cmdloop_timer_.stop();
      break;
  }
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }

void geometricCtrl::statusloopCallback(const ros::TimerEvent &event) {
  if (sim_enable_) {
    // Enable OFFBoard mode and arm automatically
    // This will only run if the vehicle is simulated
    mavros_msgs::SetMode offb_set_mode;
    arm_cmd_.request.value = true;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
      if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    } else {
      if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
}

void geometricCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude) {
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_.publish(msg);
}

void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  msg.type_mask = 128;  // Ignore orientation messages
  msg.orientation.w = target_attitude(0);
  msg.orientation.x = target_attitude(1);
  msg.orientation.y = target_attitude(2);
  msg.orientation.z = target_attitude(3);
  msg.thrust = cmd(3);

  angularVelPub_.publish(msg);
}

void geometricCtrl::pubPositionTakeoffSetpoint() {
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = position_takeoff_target_(0);
  msg.pose.position.y = position_takeoff_target_(1);
  msg.pose.position.z = position_takeoff_target_(2);
  msg.pose.orientation.w = 1.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;

  target_pose_pub_.publish(msg);
}

void geometricCtrl::pubPoseHistory() {
  nav_msgs::Path msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posehistory_vector_;

  posehistoryPub_.publish(msg);
}

void geometricCtrl::pubSystemStatus() {
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

void geometricCtrl::appendPoseHistory() {
  posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
  if (posehistory_vector_.size() > posehistory_window_) {
    posehistory_vector_.pop_back();
  }
}

geometry_msgs::PoseStamped geometricCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d &position,
                                                                  Eigen::Vector4d &orientation) {
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

void geometricCtrl::updateControllerGains() {
  Kpos_ << Kpos_x_, Kpos_y_, Kpos_z_;
  Kvel_ << Kvel_x_, Kvel_y_, Kvel_z_;
  KR_ << KR_x_, KR_y_, KR_z_;
}

double geometricCtrl::attitudeTauFromKR() const {
  const double mean_KR = std::max(0.1, (KR_x_ + KR_y_ + KR_z_) / 3.0);
  return 2.0 / mean_KR;
}

void geometricCtrl::syncLegacyAttitudeGain() {
  attctrl_tau_ = attitudeTauFromKR();
  if (controller_) {
    controller_->setAttitudeControlTimeConstant(attctrl_tau_);
  }
}

Eigen::Vector3d geometricCtrl::accelerationIndiCorrection(const Eigen::Vector3d &nominal_acc) {
  if (indi_accel_feedback_ <= 0.0) {
    accel_indi_initialized_ = false;
    return Eigen::Vector3d::Zero();
  }

  const ros::Time now = ros::Time::now();
  const double dt = accel_indi_initialized_ ? std::max(1e-3, (now - previousControlTime_).toSec()) : 0.01;
  const Eigen::Vector3d measured_acc = accel_indi_initialized_ ? (mavVel_ - previousVel_) / dt : nominal_acc;
  const double cutoff = std::max(1.0, indi_filter_cutoff_hz_);
  const double tau = 1.0 / (2.0 * M_PI * cutoff);
  const double alpha = std::max(0.0, std::min(1.0, dt / (tau + dt)));

  filteredAcc_ = accel_indi_initialized_ ? filteredAcc_ + alpha * (measured_acc - filteredAcc_) : measured_acc;
  previousVel_ = mavVel_;
  previousControlTime_ = now;
  accel_indi_initialized_ = true;

  Eigen::Vector3d correction = indi_accel_feedback_ * (nominal_acc - filteredAcc_);
  if (indi_max_correction_acc_ > 0.0 && correction.norm() > indi_max_correction_acc_) {
    correction = indi_max_correction_acc_ / correction.norm() * correction;
  }
  return correction;
}

const char *geometricCtrl::controllerName() const {
  switch (controller_type_) {
    case 1:
      return "geometric";
    case 2:
      return "lee";
    case 3:
      return "johnson";
    case 4:
      return "sun_dfbc";
    case 5:
      return "sun_dfbc_indi";
    case 6:
      return "tal";
    case 7:
      return "geometric_indi";
    default:
      return "legacy";
  }
}

Eigen::Vector3d geometricCtrl::rotorDragAcceleration(const Eigen::Vector3d &reference_acc,
                                                     const Eigen::Vector3d &reference_vel) {
  const Eigen::Vector4d q_ref = acc2quaternion(reference_acc - gravity_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);
  return R_ref * D_.asDiagonal() * R_ref.transpose() * reference_vel;
}

Eigen::Vector3d geometricCtrl::logSO3(const Eigen::Matrix3d &R) const {
  const double cos_angle = std::max(-1.0, std::min(1.0, 0.5 * (R.trace() - 1.0)));
  const double angle = std::acos(cos_angle);
  const Eigen::Matrix3d skew = 0.5 * (R - R.transpose());
  Eigen::Vector3d vee;
  vee << skew(2, 1), skew(0, 2), skew(1, 0);
  if (angle < 1e-6) {
    return vee;
  }
  return angle / std::sin(angle) * vee;
}

Eigen::Vector3d geometricCtrl::leeSO3Error(const Eigen::Matrix3d &R, const Eigen::Matrix3d &Rd) const {
  const Eigen::Matrix3d eR_hat = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
  Eigen::Vector3d eR;
  eR << eR_hat(2, 1), eR_hat(0, 2), eR_hat(1, 0);
  return eR;
}

Eigen::Vector3d geometricCtrl::quaternionAttitudeError(const Eigen::Vector4d &q,
                                                       const Eigen::Vector4d &qd) const {
  const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
  const Eigen::Vector4d q_inv = inverse.asDiagonal() * q;
  const Eigen::Vector4d qe = quatMultiplication(q_inv, qd);
  Eigen::Vector3d error;
  error << 2.0 * std::copysign(1.0, qe(0)) * qe(1), 2.0 * std::copysign(1.0, qe(0)) * qe(2),
      2.0 * std::copysign(1.0, qe(0)) * qe(3);
  return error;
}

Eigen::Vector3d geometricCtrl::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc) {
  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3d a_ref = target_acc;
  if (velocity_yaw_) {
    mavYaw_ = getVelocityYaw(mavVel_);
  }

  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - gravity_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

  const Eigen::Vector3d pos_error = mavPos_ - target_pos;
  const Eigen::Vector3d vel_error = mavVel_ - target_vel;

  // Position Controller
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

  // Rotor Drag compensation
  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag

  // Reference acceleration
  const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - gravity_;

  return a_des;
}

Eigen::Vector3d geometricCtrl::outerLoopAcceleration() {
  const Eigen::Vector3d pos_error = mavPos_ - targetPos_;
  const Eigen::Vector3d vel_error = mavVel_ - targetVel_;
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);
  const Eigen::Vector3d a_rd = rotorDragAcceleration(targetAcc_, targetVel_);
  return a_fb + targetAcc_ - a_rd;
}

void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des) {
  // Reference attitude
  q_des = acc2quaternion(a_des, mavYaw_);

  controller_->Update(mavAtt_, q_des, a_des, targetJerk_);  // Calculate BodyRate
  bodyrate_cmd.head(3) = controller_->getDesiredRate();
  double thrust_command = controller_->getDesiredThrust().z();
  // Convert thrust command to PX4 normalized collective thrust.
  bodyrate_cmd(3) =
      std::max(0.0, std::min(1.0, norm_thrust_const_ * thrust_command + norm_thrust_offset_));
}

Eigen::Vector4d geometricCtrl::computeCascadedBodyRateCmd(const Eigen::Vector3d &specific_force,
                                                          const Eigen::Vector3d &attitude_error) {
  q_des = acc2quaternion(specific_force, mavYaw_);

  const Eigen::Matrix3d R = quat2RotMatrix(mavAtt_);
  const Eigen::Matrix3d Rd = quat2RotMatrix(q_des);
  const ros::Time now = ros::Time::now();
  Eigen::Vector3d omega_ref = Eigen::Vector3d::Zero();

  if (desired_att_initialized_) {
    const double dt = std::max(1e-3, (now - previousDesiredAttTime_).toSec());
    const Eigen::Matrix3d Rdot = (Rd - previousDesiredRot_) / dt;
    const Eigen::Matrix3d omega_hat = 0.5 * (previousDesiredRot_.transpose() * Rdot -
                                            Rdot.transpose() * previousDesiredRot_);
    omega_ref << omega_hat(2, 1), omega_hat(0, 2), omega_hat(1, 0);
  }
  previousDesiredRot_ = Rd;
  previousDesiredAttTime_ = now;
  desired_att_initialized_ = true;

  Eigen::Vector4d bodyrate_cmd;
  bodyrate_cmd.head(3) = omega_ref + KR_.asDiagonal() * attitude_error;
  const double thrust_command = specific_force.dot(R.col(2));
  // Convert thrust command to PX4 normalized collective thrust.
  bodyrate_cmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * thrust_command + norm_thrust_offset_));
  return bodyrate_cmd;
}

Eigen::Vector4d geometricCtrl::controllerPDGeometric() {
  // Algorithm model/control law:
  //   p_dot = v
  //   v_dot = g*e3 - c*b3, where b3 = R*e3 and c = T/m
  //   R_dot = R*hat(Omega)
  //   J*Omega_dot = tau - Omega x J*Omega
  //   aCmd = a_d + Kp*(p_d-p) + Kv*(v_d-v)
  //   c = ||g*e3-aCmd||, b3d = (g*e3-aCmd)/c
  //   OmegaDotCmd = KR*Log(R'*Rd) + KOmega*(R'*Rd*OmegaD-Omega)
  //                 + alphaD expressed in the body frame
  //   tau = Omega x J*Omega + J*OmegaDotCmd.
  //
  // ROS/MAVROS adaptation: this node can command body rates and collective
  // thrust only. The force/moment inversion above is kept as the reference
  // formula; the implemented command is omega_ref + KR*Log(R'*Rd).
  const Eigen::Vector3d aCmd = outerLoopAcceleration();
  const Eigen::Vector3d specific_force = aCmd - gravity_;
  const Eigen::Vector4d qd = acc2quaternion(specific_force, mavYaw_);
  const Eigen::Vector3d rErr = logSO3(quat2RotMatrix(mavAtt_).transpose() * quat2RotMatrix(qd));
  return computeCascadedBodyRateCmd(specific_force, rErr);
}

Eigen::Vector4d geometricCtrl::controllerLee() {
  // Algorithm model/control law:
  //   p_dot = v
  //   v_dot = g*e3 - T/m*R*e3
  //   R_dot = R*hat(Omega)
  //   J*Omega_dot = tau - Omega x J*Omega
  // Lee Eq. (19)-(23), rewritten for this NED/FRD plant:
  //   F = Kx*(p-p_d) + Kv*(v-v_d) + m*(g*e3-a_d)
  //   b3c = F/||F||, T = F'*(R*e3)
  //   eR = 0.5*vee(Rc'*R - R'*Rc)
  //   eOmega = Omega - R'*Rc*OmegaC
  //   tau = -KR*eR - KOmega*eOmega + Omega x J*Omega
  //         - J*(hat(Omega)*R'*Rc*OmegaC - R'*Rc*OmegaCDot).
  //
  // ROS/MAVROS adaptation: keep Lee's separate attitude error definition,
  // but send the cascaded body-rate command instead of tau.
  const Eigen::Vector3d aCmd = outerLoopAcceleration();
  const Eigen::Vector3d specific_force = aCmd - gravity_;
  const Eigen::Vector4d qd = acc2quaternion(specific_force, mavYaw_);
  const Eigen::Matrix3d R = quat2RotMatrix(mavAtt_);
  const Eigen::Matrix3d Rc = quat2RotMatrix(qd);
  const Eigen::Vector3d eR = leeSO3Error(R, Rc);
  return computeCascadedBodyRateCmd(specific_force, -eR);
}

Eigen::Vector4d geometricCtrl::controllerJohnson() {
  // Algorithm model/control law:
  //   p_dot = v
  //   v_dot = g*e3 - T/m*R*e3
  //   R_dot = R*hat(Omega)
  //   J*Omega_dot = tau - Omega x J*Omega
  // Johnson and Beard Eq. (13), (18)-(21):
  //   ea = [p-p_d; v-v_d; integral(p-p_d)]
  //   fd = -K*ea + m*(a_d-g*e3)
  //   k_d = -fd/||fd||, T = ||fd||
  //   r = Log(R'*Rid)
  //   tau = Omega x J*Omega + J*omegaDotD_B
  //         + Jl(r)'*Kr*r + Komega*(R'*Rid*omegaD-Omega).
  //
  // ROS/MAVROS adaptation: use the PD outer-loop path with Ki=0, and represent
  // the torque law by body-rate feedback on r.
  const Eigen::Vector3d aCmd = outerLoopAcceleration();
  const Eigen::Vector3d specific_force = aCmd - gravity_;
  const Eigen::Vector4d qd = acc2quaternion(specific_force, mavYaw_);
  const Eigen::Vector3d r = logSO3(quat2RotMatrix(mavAtt_).transpose() * quat2RotMatrix(qd));
  return computeCascadedBodyRateCmd(specific_force, r);
}

Eigen::Vector4d geometricCtrl::controllerSunDFBC() {
  // Algorithm model/control law:
  //   Sun DFBC uses the same plant model as NMPC but computes the command
  //   explicitly:
  //     accD = Kxi*(xi_d-xi) + Kv*(v_d-v) + xi_ddot_d,
  //     T*R_d*e3 = m*(g*e3-accD) + R*f_a^B,
  //     alphaD = Kq_red*q_red + kq_yaw*q_yaw
  //              + KOmega*(OmegaR-Omega) + alphaR,
  //     tauD = J*alphaD + Omega x J*Omega.
  //
  // ROS/MAVROS adaptation: this package does not command tauD. DFBC's
  // explicit acceleration command is mapped to Rd, thrust, and body rates.
  const Eigen::Vector3d accD = outerLoopAcceleration();
  const Eigen::Vector3d specific_force = accD - gravity_;
  const Eigen::Vector4d qd = acc2quaternion(specific_force, mavYaw_);
  const Eigen::Vector3d qErr = quaternionAttitudeError(mavAtt_, qd);
  return computeCascadedBodyRateCmd(specific_force, qErr);
}

Eigen::Vector4d geometricCtrl::controllerSunDFBCINDI() {
  // Algorithm model/control law:
  //   Outer loop is Sun DFBC. Inner loop applies Sun Eq. (32)-(35):
  //     tauCmd = tau_f + J*(alphaD-alpha_f),
  //   using filtered achieved actuator/moment feedback from the benchmark
  //   allocation state.
  //
  // ROS/MAVROS adaptation: actuator/moment INDI is not available through the
  // body-rate interface. Keep the translational acceleration INDI only:
  //   accCmd = accD + Kindi*(accD-accF).
  const Eigen::Vector3d accD = outerLoopAcceleration();
  const Eigen::Vector3d accCmd = accD + accelerationIndiCorrection(accD);
  const Eigen::Vector3d specific_force = accCmd - gravity_;
  const Eigen::Vector4d qd = acc2quaternion(specific_force, mavYaw_);
  const Eigen::Vector3d qErr = quaternionAttitudeError(mavAtt_, qd);
  return computeCascadedBodyRateCmd(specific_force, qErr);
}

Eigen::Vector4d geometricCtrl::controllerTal() {
  // Algorithm model/control law:
  //   Paper model: v_dot = g*i_z + tau*b_z + f_ext/m, tau < 0 for lift.
  //   Benchmark model: v_dot = g*e3 - T/m*R*e3, so tau = -T/m.
  //   aCmd = Kp*(p_d-p) + Kv*(v_d-v) + Ka*(a_d-a_f) + a_d
  //   (tau*b_z)_c = (tau*b_z)_f + aCmd - a_f
  //   xi_e comes from the incremental attitude command.
  //   OmegaDotCmd = Ktheta*xi_e + Komega*(Omega_ref-Omega_f)
  //                 + alpha_ref
  //   tauCmd = mu_f + J*(OmegaDotCmd-OmegaDot_f).
  //
  // ROS/MAVROS adaptation: keep Tal's acceleration-feedback INDI outer loop;
  // PX4 handles the body-rate inner loop, so tauCmd is not produced here.
  const Eigen::Vector3d aNominal = outerLoopAcceleration();
  const Eigen::Vector3d aCmd = aNominal + accelerationIndiCorrection(aNominal);
  const Eigen::Vector3d specific_force = aCmd - gravity_;
  const Eigen::Vector4d qd = acc2quaternion(specific_force, mavYaw_);
  const Eigen::Vector3d qErr = quaternionAttitudeError(mavAtt_, qd);
  return computeCascadedBodyRateCmd(specific_force, qErr);
}

Eigen::Vector4d geometricCtrl::controllerGeometricINDI() {
  // Algorithm model/control law:
  //   Plant model:
  //     v_dot = g*e3 - T/m*b3,  J*Omega_dot = tau - Omega x J*Omega.
  //   Outer loop:
  //     aCmd = Kp*(p_d-p) + Kv*(v_d-v) + a_d.
  //   Incremental thrust-vector law, GINDI Eq. (55):
  //     T*b3 = T0*b30 - m*(aCmd-vDot0).
  //   Attitude/rate law:
  //     OmegaDotCmd = Ktheta*Log(R'*Rd)
  //                   + Komega*(OmegaRef-OmegaF) + alphaRef
  //     tau = tau0F + J*(OmegaDotCmd-OmegaDot0F).
  //
  // ROS/MAVROS adaptation: retain GINDI's translational increment and map the
  // resulting thrust vector to a cascaded body-rate command.
  const Eigen::Vector3d aCmd = outerLoopAcceleration();
  const Eigen::Vector3d aIndi = aCmd + accelerationIndiCorrection(aCmd);
  const Eigen::Vector3d specific_force = aIndi - gravity_;
  const Eigen::Vector4d qd = acc2quaternion(specific_force, mavYaw_);
  const Eigen::Vector3d rErr = logSO3(quat2RotMatrix(mavAtt_).transpose() * quat2RotMatrix(qd));
  return computeCascadedBodyRateCmd(specific_force, rErr);
}

Eigen::Vector3d geometricCtrl::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
  Eigen::Vector3d a_fb =
      -(Kpos_.asDiagonal() * pos_error) - (Kvel_.asDiagonal() * vel_error);  // feedback term for trajectory error

  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  return a_fb;
}

bool geometricCtrl::positionTakeoffReady() const {
  if (current_state_.mode != "OFFBOARD" || !current_state_.armed) {
    return false;
  }

  const double position_error = (mavPos_ - position_takeoff_target_).norm();
  const double velocity = mavVel_.norm();
  return position_error <= position_takeoff_tolerance_ && velocity <= position_takeoff_velocity_tolerance_;
}

Eigen::Vector4d geometricCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

bool geometricCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  unsigned char mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
  return true;
}

void geometricCtrl::dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config,
                                               uint32_t level) {
  if (controller_type_ != config.controllerName) {
    controller_type_ = config.controllerName;
    desired_att_initialized_ = false;
    accel_indi_initialized_ = false;
    ROS_INFO("Reconfigure request : controllerName = %s ", controllerName());
  }
  if (max_fb_acc_ != config.max_acc) {
    max_fb_acc_ = config.max_acc;
    ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
  }
  if (attctrl_tau_ != config.attctrl_constant) {
    attctrl_tau_ = config.attctrl_constant;
    if (controller_) {
      controller_->setAttitudeControlTimeConstant(attctrl_tau_);
    }
    ROS_INFO("Reconfigure request : attctrl_constant = %.2f ", config.attctrl_constant);
  }
  if (norm_thrust_const_ != config.normalizedthrust_constant) {
    norm_thrust_const_ = config.normalizedthrust_constant;
    ROS_INFO("Reconfigure request : normalizedthrust_constant = %.2f ", config.normalizedthrust_constant);
  }
  if (norm_thrust_offset_ != config.normalizedthrust_offset) {
    norm_thrust_offset_ = config.normalizedthrust_offset;
    ROS_INFO("Reconfigure request : normalizedthrust_offset = %.2f ", config.normalizedthrust_offset);
  }
  if (Kpos_x_ != config.Kp_x) {
    Kpos_x_ = config.Kp_x;
    ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
  }
  if (Kpos_y_ != config.Kp_y) {
    Kpos_y_ = config.Kp_y;
    ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
  }
  if (Kpos_z_ != config.Kp_z) {
    Kpos_z_ = config.Kp_z;
    ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
  }
  if (Kvel_x_ != config.Kv_x) {
    Kvel_x_ = config.Kv_x;
    ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
  }
  if (Kvel_y_ != config.Kv_y) {
    Kvel_y_ = config.Kv_y;
    ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
  }
  if (Kvel_z_ != config.Kv_z) {
    Kvel_z_ = config.Kv_z;
    ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
  }
  if (KR_x_ != config.KR_x) {
    KR_x_ = config.KR_x;
    ROS_INFO("Reconfigure request : KR_x = %.2f ", config.KR_x);
  }
  if (KR_y_ != config.KR_y) {
    KR_y_ = config.KR_y;
    ROS_INFO("Reconfigure request : KR_y = %.2f ", config.KR_y);
  }
  if (KR_z_ != config.KR_z) {
    KR_z_ = config.KR_z;
    ROS_INFO("Reconfigure request : KR_z = %.2f ", config.KR_z);
  }
  if (indi_accel_feedback_ != config.indiAccelFeedback) {
    indi_accel_feedback_ = config.indiAccelFeedback;
    ROS_INFO("Reconfigure request : indiAccelFeedback = %.2f ", config.indiAccelFeedback);
  }
  if (indi_filter_cutoff_hz_ != config.indiFilterCutoffHz) {
    indi_filter_cutoff_hz_ = config.indiFilterCutoffHz;
    accel_indi_initialized_ = false;
    ROS_INFO("Reconfigure request : indiFilterCutoffHz = %.2f ", config.indiFilterCutoffHz);
  }
  if (indi_max_correction_acc_ != config.indiMaxCorrectionAcc) {
    indi_max_correction_acc_ = config.indiMaxCorrectionAcc;
    ROS_INFO("Reconfigure request : indiMaxCorrectionAcc = %.2f ", config.indiMaxCorrectionAcc);
  }
  updateControllerGains();
}
