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
#include <cmath>

shapetrajectory::shapetrajectory(int type)
    : trajectory(),
      N(0),
      dt_(0.1),
      T_(10.0),
      type_(type),
      traj_intensity_(0.75),
      traj_base_duration_(10.0),
      helix_turns_(5.0),
      race_track_max_speed_(19.4),
      trajectory_speed_(0.3) {
  traj_omega_ = 2.0;
  traj_axis_ << 0.0, 0.0, 1.0;
  traj_radial_ << 1.0, 0.0, 0.0;
  traj_origin_ << 0.0, 0.0, 1.0;
};

shapetrajectory::~shapetrajectory(){

};

void shapetrajectory::initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega) {
  // Generate primitives based on current state for smooth trajectory
  traj_origin_ = pos;
  traj_omega_ = omega;
  traj_axis_ = axis;
  traj_radial_ << 2.0, 0.0, 0.0;
  switch (type_) {
    case TRAJ_FIGURE8_HORIZONTAL:
    case TRAJ_FIGURE8_VERTICAL:
      T_ = 2.0 * M_PI / figure8Omega(2.5, 1.5);
      break;
    case TRAJ_FAST_CIRCLE:
      T_ = 2.0 * M_PI / fixedSpeedOmega(3.0);
      break;
    case TRAJ_RACE_TRACK_C:
      T_ = 2.0 * M_PI / fixedSpeedOmega(8.75);
      break;
    case TRAJ_HELIX_FLIP:
    case TRAJ_HELIX_FLIP_Y:
    case TRAJ_FLIP_LOOP_SINE:
      T_ = traj_base_duration_;
      break;
    default:
      T_ = 2.0 * M_PI / std::max(traj_omega_, 1e-6);
      break;
  }
}

void shapetrajectory::setBenchmarkParams(double intensity, double base_duration, double helix_turns,
                                         double race_track_max_speed) {
  traj_intensity_ = clamp(intensity, 0.0, 1.0);
  traj_base_duration_ = std::max(1.0, base_duration);
  helix_turns_ = std::max(0.25, helix_turns);
  race_track_max_speed_ = std::max(0.5, race_track_max_speed);
}

void shapetrajectory::setTrajectorySpeed(double speed) { trajectory_speed_ = std::max(0.05, speed); }

void shapetrajectory::setType(int type) { type_ = type; }

double shapetrajectory::clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double shapetrajectory::periodForAccel(double length_coeff, double accel_limit, double min_period, double max_period) {
  const double period = 2.0 * M_PI * std::sqrt(std::max(length_coeff, 1e-6) / std::max(accel_limit, 1e-6));
  return clamp(period, min_period, max_period);
}

double shapetrajectory::fixedSpeedOmega(double length_scale) const {
  return trajectory_speed_ / std::max(length_scale, 1e-6);
}

double shapetrajectory::figure8Omega(double long_amp, double short_amp) const {
  const double speed_scale = std::sqrt(std::pow(long_amp, 2.0) + std::pow(2.0 * short_amp, 2.0));
  return fixedSpeedOmega(speed_scale);
}

double shapetrajectory::regularAccel() const { return 2.0 + 4.0 * traj_intensity_; }

double shapetrajectory::helixAccel() const { return 1.2 + 1.0 * traj_intensity_; }

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc,
                                         Eigen::Vector3d jerk) {}

Eigen::Vector3d shapetrajectory::getPosition(double time) {
  Eigen::Vector3d position;
  const double long_amp = 2.5;
  const double short_amp = 1.5;
  const double cruise_height = std::max(1.0, traj_origin_(2));
  double theta;
  double omega;
  double radius;
  double h_start;
  double h_center;

  switch (type_) {
    case TRAJ_ZERO:

      position << 0.0, 0.0, 0.0;
      break;

    case TRAJ_CIRCLE:

      theta = traj_omega_ * time;
      position = std::cos(theta) * traj_radial_ + std::sin(theta) * traj_axis_.cross(traj_radial_) +
                 (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_ + traj_origin_;
      break;

    case TRAJ_LAMNISCATE:  // Lemniscate of Genero

      theta = traj_omega_ * time;
      position = std::cos(theta) * traj_radial_ + std::sin(theta) * std::cos(theta) * traj_axis_.cross(traj_radial_) +
                 (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_ + traj_origin_;
      break;
    case TRAJ_STATIONARY:  // Lemniscate of Genero

      position = traj_origin_;
      break;
    case TRAJ_FIGURE8_HORIZONTAL:

      omega = figure8Omega(long_amp, short_amp);
      position << traj_origin_(0) + long_amp * std::sin(omega * time),
          traj_origin_(1) + short_amp * std::sin(2.0 * omega * time), cruise_height;
      break;
    case TRAJ_FIGURE8_VERTICAL:

      omega = figure8Omega(long_amp, short_amp);
      theta = -M_PI / 4.0 + omega * time;
      position << traj_origin_(0), traj_origin_(1) + long_amp * std::sin(theta),
          1.0 + short_amp - short_amp * std::sin(2.0 * theta);
      break;
    case TRAJ_HELIX_FLIP:

      omega = 2.0 * M_PI * helix_turns_ / traj_base_duration_;
      radius = helixAccel() / std::pow(omega, 2.0);
      h_start = std::max(1.5, 0.25 * radius);
      h_center = h_start + radius;
      position << traj_origin_(0) + radius * helix_turns_ / traj_base_duration_ * time,
          traj_origin_(1) + radius * std::sin(omega * time), h_center - radius * std::cos(omega * time);
      break;
    case TRAJ_HELIX_FLIP_Y:

      omega = 2.0 * M_PI * helix_turns_ / traj_base_duration_;
      radius = helixAccel() / std::pow(omega, 2.0);
      h_start = std::max(1.5, 0.25 * radius);
      h_center = h_start + radius;
      position << traj_origin_(0) + radius * std::sin(omega * time),
          traj_origin_(1) + radius * helix_turns_ / traj_base_duration_ * time,
          h_center - radius * std::cos(omega * time);
      break;
    case TRAJ_FLIP_LOOP_SINE:

      omega = 2.0 * M_PI * helix_turns_ / traj_base_duration_;
      radius = helixAccel() / std::pow(omega, 2.0);
      h_start = std::max(1.5, 0.25 * radius);
      h_center = h_start + radius;
      position << traj_origin_(0), traj_origin_(1) + radius * std::sin(omega * time),
          h_center - radius * std::cos(omega * time);
      break;
    case TRAJ_FAST_CIRCLE:

      radius = 3.0;
      omega = fixedSpeedOmega(radius);
      position << traj_origin_(0) + radius * std::cos(omega * time),
          traj_origin_(1) + radius * std::sin(omega * time), cruise_height;
      break;
    case TRAJ_RACE_TRACK_C:

      omega = fixedSpeedOmega(8.75);
      theta = M_PI / 2.0 + omega * time;
      position << 3.75 + 8.75 * std::cos(theta), 0.50 + 4.50 * std::sin(theta), cruise_height;
      break;
  }
  return position;
}

Eigen::Vector3d shapetrajectory::getVelocity(double time) {
  Eigen::Vector3d velocity;
  const double long_amp = 2.5;
  const double short_amp = 1.5;
  double theta;
  double omega;
  double radius;

  switch (type_) {
    case TRAJ_CIRCLE:

      velocity = traj_omega_ * traj_axis_.cross(getPosition(time) - traj_origin_);
      break;
    case TRAJ_STATIONARY:

      velocity << 0.0, 0.0, 0.0;
      break;

    case TRAJ_LAMNISCATE:  // Lemniscate of Genero

      theta = traj_omega_ * time;
      velocity = traj_omega_ *
                 (-std::sin(theta) * traj_radial_ +
                  (std::pow(std::cos(theta), 2) - std::pow(std::sin(theta), 2)) * traj_axis_.cross(traj_radial_) +
                  (std::sin(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_);
      break;

    default:
      velocity << 0.0, 0.0, 0.0;
      break;
    case TRAJ_FIGURE8_HORIZONTAL:
      omega = figure8Omega(long_amp, short_amp);
      velocity << long_amp * omega * std::cos(omega * time),
          2.0 * short_amp * omega * std::cos(2.0 * omega * time), 0.0;
      break;
    case TRAJ_FIGURE8_VERTICAL:
      omega = figure8Omega(long_amp, short_amp);
      theta = -M_PI / 4.0 + omega * time;
      velocity << 0.0, long_amp * omega * std::cos(theta),
          -2.0 * short_amp * omega * std::cos(2.0 * theta);
      break;
    case TRAJ_HELIX_FLIP:
      omega = 2.0 * M_PI * helix_turns_ / traj_base_duration_;
      radius = helixAccel() / std::pow(omega, 2.0);
      velocity << radius * helix_turns_ / traj_base_duration_, radius * omega * std::cos(omega * time),
          radius * omega * std::sin(omega * time);
      break;
    case TRAJ_HELIX_FLIP_Y:
      omega = 2.0 * M_PI * helix_turns_ / traj_base_duration_;
      radius = helixAccel() / std::pow(omega, 2.0);
      velocity << radius * omega * std::cos(omega * time), radius * helix_turns_ / traj_base_duration_,
          radius * omega * std::sin(omega * time);
      break;
    case TRAJ_FLIP_LOOP_SINE:
      omega = 2.0 * M_PI * helix_turns_ / traj_base_duration_;
      radius = helixAccel() / std::pow(omega, 2.0);
      velocity << 0.0, radius * omega * std::cos(omega * time), radius * omega * std::sin(omega * time);
      break;
    case TRAJ_FAST_CIRCLE:
      radius = 3.0;
      omega = fixedSpeedOmega(radius);
      velocity << -radius * omega * std::sin(omega * time), radius * omega * std::cos(omega * time), 0.0;
      break;
    case TRAJ_RACE_TRACK_C:
      omega = fixedSpeedOmega(8.75);
      theta = M_PI / 2.0 + omega * time;
      velocity << -8.75 * omega * std::sin(theta), 4.50 * omega * std::cos(theta), 0.0;
      break;
  }
  return velocity;
}

Eigen::Vector3d shapetrajectory::getAcceleration(double time) {
  Eigen::Vector3d acceleration;
  const double long_amp = 2.5;
  const double short_amp = 1.5;
  double theta;
  double omega;
  double radius;

  switch (type_) {
    case TRAJ_CIRCLE:

      acceleration = traj_omega_ * traj_axis_.cross(getVelocity(time));
      break;
    case TRAJ_LAMNISCATE:

      theta = traj_omega_ * time;
      acceleration = std::pow(traj_omega_, 2) *
                     (-std::cos(theta) * traj_radial_ -
                      2.0 * std::sin(2.0 * theta) * traj_axis_.cross(traj_radial_));
      break;
    case TRAJ_STATIONARY:

      acceleration << 0.0, 0.0, 0.0;
      break;
    default:
      acceleration << 0.0, 0.0, 0.0;
      break;
    case TRAJ_FIGURE8_HORIZONTAL:
      omega = figure8Omega(long_amp, short_amp);
      acceleration << -long_amp * std::pow(omega, 2.0) * std::sin(omega * time),
          -4.0 * short_amp * std::pow(omega, 2.0) * std::sin(2.0 * omega * time), 0.0;
      break;
    case TRAJ_FIGURE8_VERTICAL:
      omega = figure8Omega(long_amp, short_amp);
      theta = -M_PI / 4.0 + omega * time;
      acceleration << 0.0, -long_amp * std::pow(omega, 2.0) * std::sin(theta),
          4.0 * short_amp * std::pow(omega, 2.0) * std::sin(2.0 * theta);
      break;
    case TRAJ_HELIX_FLIP:
      omega = 2.0 * M_PI * helix_turns_ / traj_base_duration_;
      radius = helixAccel() / std::pow(omega, 2.0);
      acceleration << 0.0, -radius * std::pow(omega, 2.0) * std::sin(omega * time),
          radius * std::pow(omega, 2.0) * std::cos(omega * time);
      break;
    case TRAJ_HELIX_FLIP_Y:
      omega = 2.0 * M_PI * helix_turns_ / traj_base_duration_;
      radius = helixAccel() / std::pow(omega, 2.0);
      acceleration << -radius * std::pow(omega, 2.0) * std::sin(omega * time), 0.0,
          radius * std::pow(omega, 2.0) * std::cos(omega * time);
      break;
    case TRAJ_FLIP_LOOP_SINE:
      omega = 2.0 * M_PI * helix_turns_ / traj_base_duration_;
      radius = helixAccel() / std::pow(omega, 2.0);
      acceleration << 0.0, -radius * std::pow(omega, 2.0) * std::sin(omega * time),
          radius * std::pow(omega, 2.0) * std::cos(omega * time);
      break;
    case TRAJ_FAST_CIRCLE:
      radius = 3.0;
      omega = fixedSpeedOmega(radius);
      acceleration << -radius * std::pow(omega, 2.0) * std::cos(omega * time),
          -radius * std::pow(omega, 2.0) * std::sin(omega * time), 0.0;
      break;
    case TRAJ_RACE_TRACK_C:
      omega = fixedSpeedOmega(8.75);
      theta = M_PI / 2.0 + omega * time;
      acceleration << -8.75 * std::pow(omega, 2.0) * std::cos(theta),
          -4.50 * std::pow(omega, 2.0) * std::sin(theta), 0.0;
      break;
  }
  return acceleration;
}

nav_msgs::Path shapetrajectory::getSegment() {
  Eigen::Vector3d targetPosition;
  Eigen::Vector4d targetOrientation;
  nav_msgs::Path segment;

  targetOrientation << 1.0, 0.0, 0.0, 0.0;
  geometry_msgs::PoseStamped targetPoseStamped;

  for (double t = 0; t < this->getDuration(); t += this->getsamplingTime()) {
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
