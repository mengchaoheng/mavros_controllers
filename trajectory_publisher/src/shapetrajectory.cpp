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
 * @brief Shape Trajectory Library
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "trajectory_publisher/shapetrajectory.h"

#include <algorithm>
#include <cctype>

namespace {
std::string normalizeName(std::string name) {
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); });
  std::replace(name.begin(), name.end(), '-', '_');
  return name;
}
}  // namespace

shapetrajectory::shapetrajectory(int type)
    : trajectory(),
      type_(type),
      dt_(0.1),
      T_(10.0),
      omega_value_(1.0),
      omega_start_(1.0),
      omega_end_(1.0),
      omega_duration_(20.0),
      omega_mode_(TRAJ_OMEGA_FIXED),
      phase_shift_(0.0),
      path_preview_cycles_(1.0) {
  traj_axis_ << 0.0, 0.0, 1.0;
  traj_origin_ << 0.0, 0.0, 1.0;
}

shapetrajectory::~shapetrajectory() {}

double shapetrajectory::sanitizePositive(double value, double fallback) {
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }
  return value;
}

void shapetrajectory::initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega) {
  traj_origin_ = pos;
  traj_axis_ = axis;
  omega_value_ = sanitizePositive(omega, omega_value_);
  if (omega_mode_ == TRAJ_OMEGA_FIXED) {
    omega_start_ = omega_value_;
    omega_end_ = omega_value_;
    T_ = 2.0 * M_PI / sanitizePositive(omega_value_, 1.0);
  } else {
    T_ = sanitizePositive(omega_duration_, 20.0);
  }
}

void shapetrajectory::setType(int type) { type_ = type; }

void shapetrajectory::setParams(const Params& params) { params_ = params; }

void shapetrajectory::setOmega(int mode, double value, double start, double end, double duration) {
  omega_mode_ = std::max(TRAJ_OMEGA_FIXED, std::min(TRAJ_OMEGA_QUADRATIC, mode));
  omega_value_ = sanitizePositive(value, 1.0);
  omega_start_ = sanitizePositive(start, omega_value_);
  omega_end_ = sanitizePositive(end, omega_value_);
  omega_duration_ = sanitizePositive(duration, 20.0);
  if (omega_mode_ == TRAJ_OMEGA_FIXED) {
    omega_start_ = omega_value_;
    omega_end_ = omega_value_;
    T_ = 2.0 * M_PI / omega_value_;
  } else {
    T_ = omega_duration_;
  }
}

void shapetrajectory::setPhaseShift(double phase_shift) { phase_shift_ = phase_shift; }

void shapetrajectory::setPathPreviewCycles(double cycles) {
  path_preview_cycles_ = std::max(1.0, sanitizePositive(cycles, 1.0));
}

int shapetrajectory::typeFromName(const std::string& name, int fallback) {
  const std::string normalized = normalizeName(name);
  if (normalized == "figure8_horizontal") return TRAJ_FIGURE8_HORIZONTAL;
  if (normalized == "figure8_vertical") return TRAJ_FIGURE8_VERTICAL;
  if (normalized == "helix_flip") return TRAJ_HELIX_FLIP;
  if (normalized == "helix_flip_y") return TRAJ_HELIX_FLIP_Y;
  if (normalized == "flip_loop_sine") return TRAJ_FLIP_LOOP_SINE;
  if (normalized == "fast_circle") return TRAJ_FAST_CIRCLE;
  if (normalized == "circle") return TRAJ_FAST_CIRCLE;
  if (normalized == "lamniscate" || normalized == "figure8" || normalized == "figure_8") {
    return TRAJ_FIGURE8_HORIZONTAL;
  }
  return fallback;
}

const char* shapetrajectory::typeName(int type) {
  switch (type) {
    case TRAJ_FIGURE8_HORIZONTAL:
      return "figure8_horizontal";
    case TRAJ_FIGURE8_VERTICAL:
      return "figure8_vertical";
    case TRAJ_HELIX_FLIP:
      return "helix_flip";
    case TRAJ_HELIX_FLIP_Y:
      return "helix_flip_y";
    case TRAJ_FLIP_LOOP_SINE:
      return "flip_loop_sine";
    case TRAJ_FAST_CIRCLE:
      return "fast_circle";
    default:
      return "figure8_horizontal";
  }
}

void shapetrajectory::thetaState(double time, double& theta, double& theta_dot, double& theta_ddot,
                                 double& theta_3) const {
  const double t = std::max(0.0, time);
  const double duration = sanitizePositive(omega_duration_, 20.0);

  if (omega_mode_ == TRAJ_OMEGA_FIXED) {
    theta = typeTheta0() + phase_shift_ + omega_value_ * t;
    theta_dot = omega_value_;
    theta_ddot = 0.0;
    theta_3 = 0.0;
    return;
  }

  const double t_clamped = std::min(t, duration);
  const double omega_delta = omega_end_ - omega_start_;
  if (omega_mode_ == TRAJ_OMEGA_LINEAR) {
    theta = omega_start_ * t_clamped + 0.5 * omega_delta * t_clamped * t_clamped / duration;
    theta_dot = omega_start_ + omega_delta * t_clamped / duration;
    theta_ddot = omega_delta / duration;
    theta_3 = 0.0;
  } else {
    theta = omega_start_ * t_clamped + omega_delta * std::pow(t_clamped, 3.0) / (3.0 * duration * duration);
    theta_dot = omega_start_ + omega_delta * std::pow(t_clamped / duration, 2.0);
    theta_ddot = 2.0 * omega_delta * t_clamped / (duration * duration);
    theta_3 = 2.0 * omega_delta / (duration * duration);
  }

  if (t > duration) {
    theta += omega_end_ * (t - duration);
    theta_dot = omega_end_;
    theta_ddot = 0.0;
    theta_3 = 0.0;
  }
  theta += typeTheta0() + phase_shift_;
}

double shapetrajectory::typeTheta0() const {
  switch (type_) {
    case TRAJ_FIGURE8_VERTICAL:
      return params_.figure8_vertical_theta0;
    case TRAJ_HELIX_FLIP:
      return params_.helix_flip_theta0;
    case TRAJ_HELIX_FLIP_Y:
      return params_.helix_flip_y_theta0;
    case TRAJ_FLIP_LOOP_SINE:
      return params_.flip_loop_sine_theta0;
    case TRAJ_FAST_CIRCLE:
      return params_.fast_circle_theta0;
    case TRAJ_FIGURE8_HORIZONTAL:
    default:
      return params_.figure8_horizontal_theta0;
  }
}

shapetrajectory::ScalarState shapetrajectory::trigDerivatives(double amplitude, double theta, double theta_dot,
                                                              double theta_ddot, double theta_3,
                                                              const std::string& kind) {
  const double s = std::sin(theta);
  const double c = std::cos(theta);
  ScalarState out;

  if (kind == "sin") {
    out.p = amplitude * s;
    out.v = amplitude * c * theta_dot;
    out.a = amplitude * (c * theta_ddot - s * theta_dot * theta_dot);
    out.j = amplitude * (c * theta_3 - 3.0 * s * theta_dot * theta_ddot -
                         c * std::pow(theta_dot, 3.0));
  } else {
    out.p = amplitude * c;
    out.v = -amplitude * s * theta_dot;
    out.a = amplitude * (-c * theta_dot * theta_dot - s * theta_ddot);
    out.j = amplitude * (s * std::pow(theta_dot, 3.0) - 3.0 * c * theta_dot * theta_ddot -
                         s * theta_3);
  }
  return out;
}

void shapetrajectory::setHeadingFromVelocity(ReferenceState& ref, double default_yaw) const {
  const double speed2 = ref.v.x() * ref.v.x() + ref.v.y() * ref.v.y();
  if (speed2 < 1e-10) {
    ref.yaw = default_yaw;
    ref.yaw_rate = 0.0;
    ref.yaw_acceleration = 0.0;
    return;
  }

  const double num = ref.v.x() * ref.a.y() - ref.v.y() * ref.a.x();
  const double den_dot = 2.0 * (ref.v.x() * ref.a.x() + ref.v.y() * ref.a.y());
  const double num_dot = ref.v.x() * ref.j.y() - ref.v.y() * ref.j.x();
  ref.yaw = std::atan2(ref.v.y(), ref.v.x());
  ref.yaw_rate = num / speed2;
  ref.yaw_acceleration = (num_dot * speed2 - num * den_dot) / (speed2 * speed2);
}

shapetrajectory::ReferenceState shapetrajectory::evaluate(double time) const {
  ReferenceState ref;
  double theta, theta_dot, theta_ddot, theta_3;
  thetaState(time, theta, theta_dot, theta_ddot, theta_3);

  const double x0 = traj_origin_.x();
  const double y0 = traj_origin_.y();
  const double t = std::max(0.0, time);

  switch (type_) {
    case TRAJ_FIGURE8_VERTICAL: {
      const ScalarState y = trigDerivatives(params_.figure8_vertical_Ay, theta, theta_dot, theta_ddot, theta_3, "sin");
      const ScalarState z =
          trigDerivatives(params_.figure8_vertical_Az, 2.0 * theta, 2.0 * theta_dot, 2.0 * theta_ddot,
                          2.0 * theta_3, "sin");
      ref.p << x0, y0 + y.p, params_.figure8_vertical_Hc + z.p;
      ref.v << 0.0, y.v, z.v;
      ref.a << 0.0, y.a, z.a;
      ref.j << 0.0, y.j, z.j;
      ref.yaw = 0.0;
      break;
    }
    case TRAJ_HELIX_FLIP: {
      const ScalarState y = trigDerivatives(params_.helix_flip_Ay, theta, theta_dot, theta_ddot, theta_3, "sin");
      const ScalarState z = trigDerivatives(-params_.helix_flip_Az, theta, theta_dot, theta_ddot, theta_3, "cos");
      ref.p << x0 + params_.helix_flip_Vx * t, y0 + y.p, params_.helix_flip_Hc + z.p;
      ref.v << params_.helix_flip_Vx, y.v, z.v;
      ref.a << 0.0, y.a, z.a;
      ref.j << 0.0, y.j, z.j;
      ref.yaw = 0.0;
      break;
    }
    case TRAJ_HELIX_FLIP_Y: {
      const ScalarState x = trigDerivatives(params_.helix_flip_y_Ax, theta, theta_dot, theta_ddot, theta_3, "sin");
      const ScalarState z = trigDerivatives(-params_.helix_flip_y_Az, theta, theta_dot, theta_ddot, theta_3, "cos");
      ref.p << x0 + x.p, y0 + params_.helix_flip_y_Vy * t, params_.helix_flip_y_Hc + z.p;
      ref.v << x.v, params_.helix_flip_y_Vy, z.v;
      ref.a << x.a, 0.0, z.a;
      ref.j << x.j, 0.0, z.j;
      ref.yaw = 0.0;
      break;
    }
    case TRAJ_FLIP_LOOP_SINE: {
      const ScalarState y = trigDerivatives(params_.flip_loop_sine_Ay, theta, theta_dot, theta_ddot, theta_3, "sin");
      const ScalarState z = trigDerivatives(-params_.flip_loop_sine_Az, theta, theta_dot, theta_ddot, theta_3, "cos");
      ref.p << x0 + params_.flip_loop_sine_Vx * t, y0 + y.p, params_.flip_loop_sine_Hc + z.p;
      ref.v << params_.flip_loop_sine_Vx, y.v, z.v;
      ref.a << 0.0, y.a, z.a;
      ref.j << 0.0, y.j, z.j;
      ref.yaw = 0.0;
      break;
    }
    case TRAJ_FAST_CIRCLE: {
      const ScalarState x = trigDerivatives(params_.fast_circle_Ax, theta, theta_dot, theta_ddot, theta_3, "cos");
      const ScalarState y = trigDerivatives(params_.fast_circle_Ay, theta, theta_dot, theta_ddot, theta_3, "sin");
      ref.p << x0 + x.p, y0 + y.p, params_.fast_circle_Hc;
      ref.v << x.v, y.v, 0.0;
      ref.a << x.a, y.a, 0.0;
      ref.j << x.j, y.j, 0.0;
      setHeadingFromVelocity(ref, 0.0);
      break;
    }
    case TRAJ_FIGURE8_HORIZONTAL:
    default: {
      const ScalarState x =
          trigDerivatives(params_.figure8_horizontal_Ax, theta, theta_dot, theta_ddot, theta_3, "sin");
      const ScalarState y =
          trigDerivatives(params_.figure8_horizontal_Ay, 2.0 * theta, 2.0 * theta_dot, 2.0 * theta_ddot,
                          2.0 * theta_3, "sin");
      ref.p << x0 + x.p, y0 + y.p, params_.figure8_horizontal_Hc;
      ref.v << x.v, y.v, 0.0;
      ref.a << x.a, y.a, 0.0;
      ref.j << x.j, y.j, 0.0;
      setHeadingFromVelocity(ref, 0.0);
      break;
    }
  }

  return ref;
}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc,
                                         Eigen::Vector3d jerk) {}

Eigen::Vector3d shapetrajectory::getPosition(double time) { return evaluate(time).p; }

Eigen::Vector3d shapetrajectory::getVelocity(double time) { return evaluate(time).v; }

Eigen::Vector3d shapetrajectory::getAcceleration(double time) { return evaluate(time).a; }

Eigen::Vector3d shapetrajectory::getJerk(double time) { return evaluate(time).j; }

double shapetrajectory::getYaw(double time) { return evaluate(time).yaw; }

double shapetrajectory::getYawRate(double time) { return evaluate(time).yaw_rate; }

double shapetrajectory::getYawAcceleration(double time) { return evaluate(time).yaw_acceleration; }

nav_msgs::Path shapetrajectory::getSegment() {
  Eigen::Vector3d targetPosition;
  Eigen::Vector4d targetOrientation;
  nav_msgs::Path segment;

  targetOrientation << 1.0, 0.0, 0.0, 0.0;
  geometry_msgs::PoseStamped targetPoseStamped;

  double preview_duration = this->getDuration();
  if (type_ == TRAJ_HELIX_FLIP || type_ == TRAJ_HELIX_FLIP_Y || type_ == TRAJ_FLIP_LOOP_SINE) {
    preview_duration *= path_preview_cycles_;
  }

  for (double t = 0; t < preview_duration; t += this->getsamplingTime()) {
    targetPosition = this->getPosition(t);
    targetPoseStamped = vector3d2PoseStampedMsg(targetPosition, targetOrientation);
    segment.poses.push_back(targetPoseStamped);
  }
  return segment;
}

geometry_msgs::PoseStamped shapetrajectory::vector3d2PoseStampedMsg(Eigen::Vector3d position,
                                                                    Eigen::Vector4d orientation) {
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
