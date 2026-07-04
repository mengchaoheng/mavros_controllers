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
#include "geometric_controller/legacy_geometric_controller.h"
#include "geometric_controller/main_geometric_indi_controller.h"
#include "geometric_controller/main_geometric_controller.h"
#include "geometric_controller/main_johnson_controller.h"
#include "geometric_controller/main_lee_controller.h"
#include "geometric_controller/main_sun_dfbc_controller.h"
#include "geometric_controller/main_tal_controller.h"

#include <algorithm>
#include <cmath>

using namespace Eigen;
using namespace std;

namespace {
double roundToFourDecimals(double value) { return std::round(value * 10000.0) / 10000.0; }

double sanitizeCmdloopRate(double rate) {
  if (!std::isfinite(rate)) {
    return 250.0;
  }
  return std::max(20.0, std::min(500.0, rate));
}

int sanitizeControllerType(int controller_type) {
  if (controller_type < static_cast<int>(geometric_controller::ControllerType::LEGACY_GEOMETRIC) ||
      controller_type > static_cast<int>(geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI)) {
    return static_cast<int>(geometric_controller::ControllerType::LEGACY_GEOMETRIC);
  }
  return controller_type;
}
}  // namespace

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
  nh_private_.param<double>("cmdloop_rate", cmdloop_rate_, 250.0);
  cmdloop_rate_ = sanitizeCmdloopRate(cmdloop_rate_);
  cmdloop_timer_ = nh_.createTimer(ros::Duration(1.0 / cmdloop_rate_), &geometricCtrl::cmdloopCallback,
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

  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
  nh_private_.param<int>("controller_type", controller_type_,
                         static_cast<int>(geometric_controller::ControllerType::LEGACY_GEOMETRIC));
  controller_type_ = sanitizeControllerType(controller_type_);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 10.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);

  double dx, dy, dz;
  nh_private_.param<double>("drag_dx", dx, 0.0);
  nh_private_.param<double>("drag_dy", dy, 0.0);
  nh_private_.param<double>("drag_dz", dz, 0.0);
  D_ << dx, dy, dz;

  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.0220);
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.0);
  nh_private_.param<double>("Kp_x", Kpos_x_, 10.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 10.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 20.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 5.0);
  nh_private_.param<double>("Kv_y", Kvel_y_, 5.0);
  nh_private_.param<double>("Kv_z", Kvel_z_, 10.0);
  nh_private_.param<double>("KR_r", Krot_r_, 4.0);
  nh_private_.param<double>("KR_p", Krot_p_, 4.0);
  nh_private_.param<double>("KR_y", Krot_y_, 4.0);
  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
  nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
  nh_private_.param<double>("init_pos_z", initTargetPos_z_, 2.0);

  targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  selectActiveController(controller_type_);
}
geometricCtrl::~geometricCtrl() {
  // Destructor
}

void geometricCtrl::updateCommandLoopRate(double rate_hz) {
  const double sanitized_rate = sanitizeCmdloopRate(rate_hz);
  if (std::abs(cmdloop_rate_ - sanitized_rate) < 1e-9) {
    return;
  }
  cmdloop_rate_ = sanitized_rate;
  cmdloop_timer_.setPeriod(ros::Duration(1.0 / cmdloop_rate_), true);
  ROS_INFO("Reconfigure request : cmdloop_rate = %.4f Hz", cmdloop_rate_);
}

void geometricCtrl::selectActiveController(int controller_type) {
  const int sanitized_type = sanitizeControllerType(controller_type);
  controller_type_ = sanitized_type;
  const auto requested_type = static_cast<geometric_controller::ControllerType>(sanitized_type);

  switch (requested_type) {
    case geometric_controller::ControllerType::LEGACY_GEOMETRIC:
      active_controller_type_ = requested_type;
      active_controller_ = std::make_shared<geometric_controller::LegacyGeometricController>();
      break;
    case geometric_controller::ControllerType::MAIN_GEOMETRIC:
      active_controller_type_ = requested_type;
      active_controller_ = std::make_shared<geometric_controller::MainGeometricController>();
      break;
    case geometric_controller::ControllerType::MAIN_LEE:
      active_controller_type_ = requested_type;
      active_controller_ = std::make_shared<geometric_controller::MainLeeController>();
      break;
    case geometric_controller::ControllerType::MAIN_JOHNSON:
      active_controller_type_ = requested_type;
      active_controller_ = std::make_shared<geometric_controller::MainJohnsonController>();
      break;
    case geometric_controller::ControllerType::MAIN_SUN_DFBC:
      active_controller_type_ = requested_type;
      active_controller_ = std::make_shared<geometric_controller::MainSunDFBCController>();
      break;
    case geometric_controller::ControllerType::MAIN_SUN_DFBC_INDI:
      active_controller_type_ = requested_type;
      active_controller_ = std::make_shared<geometric_controller::MainSunDFBCController>(true);
      break;
    case geometric_controller::ControllerType::MAIN_TAL:
      active_controller_type_ = requested_type;
      active_controller_ = std::make_shared<geometric_controller::MainTalController>();
      break;
    case geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI:
      active_controller_type_ = requested_type;
      active_controller_ = std::make_shared<geometric_controller::MainGeometricINDIController>();
      break;
    default:
      ROS_WARN("Controller type %d is reserved but not implemented yet. Falling back to legacy_geometric.",
               sanitized_type);
      active_controller_type_ = geometric_controller::ControllerType::LEGACY_GEOMETRIC;
      active_controller_ = std::make_shared<geometric_controller::LegacyGeometricController>();
      controller_type_ = static_cast<int>(active_controller_type_);
      break;
  }

  active_controller_->reset(getVehicleState());
  ROS_INFO("Active controller: %s", active_controller_->name().c_str());
}

geometric_controller::VehicleState geometricCtrl::getVehicleState() const {
  geometric_controller::VehicleState state;
  state.position = mavPos_;
  state.velocity = mavVel_;
  state.body_rate = mavRate_;
  state.attitude = mavAtt_;
  state.yaw = mavYaw_;
  return state;
}

geometric_controller::FlatReference geometricCtrl::getFlatReference() {
  if (velocity_yaw_) {
    mavYaw_ = getVelocityYaw(mavVel_);
  }

  geometric_controller::FlatReference reference;
  reference.position = targetPos_;
  reference.velocity = targetVel_;
  reference.acceleration = targetAcc_;
  reference.jerk = targetJerk_;
  reference.snap = targetSnap_;
  reference.yaw = mavYaw_;
  return reference;
}

geometric_controller::ControllerParams geometricCtrl::getControllerParams() const {
  geometric_controller::ControllerParams params;
  params.ctrl_mode = ctrl_mode_;
  params.feedthrough_enable = feedthrough_enable_;
  params.velocity_yaw = velocity_yaw_;
  params.gravity = gravity_;
  params.drag = D_;
  params.Kp << Kpos_x_, Kpos_y_, Kpos_z_;
  params.Kv << Kvel_x_, Kvel_y_, Kvel_z_;
  params.KR << Krot_r_, Krot_p_, Krot_y_;
  params.max_feedback_acc = max_fb_acc_;
  params.normalizedthrust_constant = norm_thrust_const_;
  params.normalizedthrust_offset = norm_thrust_offset_;
  return params;
}

void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped &msg) {
  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.twist.angular);
  targetVel_ = toEigen(msg.twist.linear);

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
      const geometric_controller::VehicleState state = getVehicleState();
      const geometric_controller::FlatReference reference = getFlatReference();
      const geometric_controller::ControllerParams params = getControllerParams();
      const geometric_controller::ControllerCommand command =
          active_controller_->update(state, reference, params, event.current_real.toSec() - event.last_real.toSec());

      q_des = command.attitude;
      cmdBodyRate_.head(3) = command.body_rate;
      cmdBodyRate_(3) = command.thrust;
      pubReferencePose(command.reference_position, command.attitude);
      pubRateCommands(cmdBodyRate_, command.attitude);
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
  config.controller_type = sanitizeControllerType(config.controller_type);
  config.cmdloop_rate = roundToFourDecimals(sanitizeCmdloopRate(config.cmdloop_rate));
  config.max_acc = roundToFourDecimals(config.max_acc);
  config.normalizedthrust_constant = roundToFourDecimals(config.normalizedthrust_constant);
  config.normalizedthrust_offset = roundToFourDecimals(config.normalizedthrust_offset);
  config.Kp_x = roundToFourDecimals(config.Kp_x);
  config.Kp_y = roundToFourDecimals(config.Kp_y);
  config.Kp_z = roundToFourDecimals(config.Kp_z);
  config.Kv_x = roundToFourDecimals(config.Kv_x);
  config.Kv_y = roundToFourDecimals(config.Kv_y);
  config.Kv_z = roundToFourDecimals(config.Kv_z);
  config.KR_r = roundToFourDecimals(config.KR_r);
  config.KR_p = roundToFourDecimals(config.KR_p);
  config.KR_y = roundToFourDecimals(config.KR_y);

  if (controller_type_ != config.controller_type) {
    selectActiveController(config.controller_type);
    config.controller_type = controller_type_;
  }

  updateCommandLoopRate(config.cmdloop_rate);

  if (max_fb_acc_ != config.max_acc) {
    max_fb_acc_ = config.max_acc;
    ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
  }
  if (norm_thrust_const_ != config.normalizedthrust_constant) {
    norm_thrust_const_ = config.normalizedthrust_constant;
    ROS_INFO("Reconfigure request : normalizedthrust_constant = %.3f ", config.normalizedthrust_constant);
  }
  if (norm_thrust_offset_ != config.normalizedthrust_offset) {
    norm_thrust_offset_ = config.normalizedthrust_offset;
    ROS_INFO("Reconfigure request : normalizedthrust_offset = %.3f ", config.normalizedthrust_offset);
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
  if (Krot_r_ != config.KR_r) {
    Krot_r_ = config.KR_r;
    ROS_INFO("Reconfigure request : KR_r  = %.2f  ", config.KR_r);
  }
  if (Krot_p_ != config.KR_p) {
    Krot_p_ = config.KR_p;
    ROS_INFO("Reconfigure request : KR_p  = %.2f  ", config.KR_p);
  }
  if (Krot_y_ != config.KR_y) {
    Krot_y_ = config.KR_y;
    ROS_INFO("Reconfigure request : KR_y  = %.2f  ", config.KR_y);
  }

}
