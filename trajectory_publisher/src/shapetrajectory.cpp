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
      trajectory_speed_(0.3),
      phase_shift_(0.0) {
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
    case TRAJ_FAST_CIRCLE:
    case TRAJ_HELIX_FLIP:
    case TRAJ_HELIX_FLIP_Y:
    case TRAJ_FLIP_LOOP_SINE:
    case TRAJ_RACE_TRACK_C:
      T_ = 2.0 * M_PI / activeOmega();
      break;
    case TRAJ_STATIONARY:
      T_ = 10.0;
      break;
    default:
      T_ = 2.0 * M_PI / activeOmega();
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

void shapetrajectory::setParams(const Params& params) { params_ = params; }

void shapetrajectory::setPhaseShift(double phase_shift) { phase_shift_ = phase_shift; }

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

double shapetrajectory::activeOmega() const { return std::max(traj_omega_, 1e-6); }

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc,
                                         Eigen::Vector3d jerk) {}

Eigen::Vector3d shapetrajectory::getPosition(double time) {
  Eigen::Vector3d position;
  double theta;
  const double omega = traj_omega_;
  const double x0 = traj_origin_(0);
  const double y0 = traj_origin_(1);

  switch (type_) {
    case TRAJ_ZERO:

      position << 0.0, 0.0, 0.0;
      break;

    case TRAJ_CIRCLE:

      theta = phase_shift_ + traj_omega_ * time;
      position = std::cos(theta) * traj_radial_ + std::sin(theta) * traj_axis_.cross(traj_radial_) +
                 (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_ + traj_origin_;
      break;

    case TRAJ_LAMNISCATE:  // Lemniscate of Genero

      theta = phase_shift_ + traj_omega_ * time;
      position = std::cos(theta) * traj_radial_ + std::sin(theta) * std::cos(theta) * traj_axis_.cross(traj_radial_) +
                 (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_ + traj_origin_;
      break;
    case TRAJ_STATIONARY:  // Lemniscate of Genero

      position = traj_origin_;
      break;
    case TRAJ_FIGURE8_HORIZONTAL:

      theta = params_.figure8_horizontal_theta0 + phase_shift_ + omega * time;
      position << x0 + params_.figure8_horizontal_Ax * std::sin(theta),
          y0 + params_.figure8_horizontal_Ay * std::sin(2.0 * theta), params_.figure8_horizontal_Hc;
      break;
    case TRAJ_FIGURE8_VERTICAL:

      theta = params_.figure8_vertical_theta0 + phase_shift_ + omega * time;
      position << x0, y0 + params_.figure8_vertical_Ay * std::sin(theta),
          params_.figure8_vertical_Hc + params_.figure8_vertical_Az * std::sin(2.0 * theta);
      break;
    case TRAJ_HELIX_FLIP:

      theta = params_.helix_flip_theta0 + phase_shift_ + omega * time;
      position << x0 + params_.helix_flip_Vx * time, y0 + params_.helix_flip_Ay * std::sin(theta),
          params_.helix_flip_Hc - params_.helix_flip_Az * std::cos(theta);
      break;
    case TRAJ_HELIX_FLIP_Y:

      theta = params_.helix_flip_y_theta0 + phase_shift_ + omega * time;
      position << x0 + params_.helix_flip_y_Ax * std::sin(theta), y0 + params_.helix_flip_y_Vy * time,
          params_.helix_flip_y_Hc - params_.helix_flip_y_Az * std::cos(theta);
      break;
    case TRAJ_FLIP_LOOP_SINE:

      theta = params_.flip_loop_sine_theta0 + phase_shift_ + omega * time;
      position << x0 + params_.flip_loop_sine_Vx * time, y0 + params_.flip_loop_sine_Ay * std::sin(theta),
          params_.flip_loop_sine_Hc - params_.flip_loop_sine_Az * std::cos(theta);
      break;
    case TRAJ_FAST_CIRCLE:

      theta = params_.fast_circle_theta0 + phase_shift_ + omega * time;
      position << x0 + params_.fast_circle_Ax * std::cos(theta),
          y0 + params_.fast_circle_Ay * std::sin(theta), params_.fast_circle_Hc;
      break;
    case TRAJ_RACE_TRACK_C:

      theta = M_PI / 2.0 + phase_shift_ + omega * time;
      position << 3.75 + 8.75 * std::cos(theta), 0.50 + 4.50 * std::sin(theta),
          std::max(1.0, traj_origin_(2));
      break;
  }
  return position;
}

Eigen::Vector3d shapetrajectory::getVelocity(double time) {
  Eigen::Vector3d velocity;
  double theta;
  const double omega = traj_omega_;

  switch (type_) {
    case TRAJ_CIRCLE:

      velocity = traj_omega_ * traj_axis_.cross(getPosition(time) - traj_origin_);
      break;
    case TRAJ_STATIONARY:

      velocity << 0.0, 0.0, 0.0;
      break;

    case TRAJ_LAMNISCATE:  // Lemniscate of Genero

      theta = phase_shift_ + traj_omega_ * time;
      velocity = traj_omega_ *
                 (-std::sin(theta) * traj_radial_ +
                  (std::pow(std::cos(theta), 2) - std::pow(std::sin(theta), 2)) * traj_axis_.cross(traj_radial_) +
                  (std::sin(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_);
      break;

    default:
      velocity << 0.0, 0.0, 0.0;
      break;
    case TRAJ_FIGURE8_HORIZONTAL:
      theta = params_.figure8_horizontal_theta0 + phase_shift_ + omega * time;
      velocity << params_.figure8_horizontal_Ax * omega * std::cos(theta),
          2.0 * params_.figure8_horizontal_Ay * omega * std::cos(2.0 * theta), 0.0;
      break;
    case TRAJ_FIGURE8_VERTICAL:
      theta = params_.figure8_vertical_theta0 + phase_shift_ + omega * time;
      velocity << 0.0, params_.figure8_vertical_Ay * omega * std::cos(theta),
          2.0 * params_.figure8_vertical_Az * omega * std::cos(2.0 * theta);
      break;
    case TRAJ_HELIX_FLIP:
      theta = params_.helix_flip_theta0 + phase_shift_ + omega * time;
      velocity << params_.helix_flip_Vx, params_.helix_flip_Ay * omega * std::cos(theta),
          params_.helix_flip_Az * omega * std::sin(theta);
      break;
    case TRAJ_HELIX_FLIP_Y:
      theta = params_.helix_flip_y_theta0 + phase_shift_ + omega * time;
      velocity << params_.helix_flip_y_Ax * omega * std::cos(theta), params_.helix_flip_y_Vy,
          params_.helix_flip_y_Az * omega * std::sin(theta);
      break;
    case TRAJ_FLIP_LOOP_SINE:
      theta = params_.flip_loop_sine_theta0 + phase_shift_ + omega * time;
      velocity << params_.flip_loop_sine_Vx, params_.flip_loop_sine_Ay * omega * std::cos(theta),
          params_.flip_loop_sine_Az * omega * std::sin(theta);
      break;
    case TRAJ_FAST_CIRCLE:
      theta = params_.fast_circle_theta0 + phase_shift_ + omega * time;
      velocity << -params_.fast_circle_Ax * omega * std::sin(theta),
          params_.fast_circle_Ay * omega * std::cos(theta), 0.0;
      break;
    case TRAJ_RACE_TRACK_C:
      theta = M_PI / 2.0 + phase_shift_ + omega * time;
      velocity << -8.75 * omega * std::sin(theta), 4.50 * omega * std::cos(theta), 0.0;
      break;
  }
  return velocity;
}

Eigen::Vector3d shapetrajectory::getAcceleration(double time) {
  Eigen::Vector3d acceleration;
  double theta;
  const double omega = traj_omega_;

  switch (type_) {
    case TRAJ_CIRCLE:

      acceleration = traj_omega_ * traj_axis_.cross(getVelocity(time));
      break;
    case TRAJ_LAMNISCATE:

      theta = phase_shift_ + traj_omega_ * time;
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
      theta = params_.figure8_horizontal_theta0 + phase_shift_ + omega * time;
      acceleration << -params_.figure8_horizontal_Ax * std::pow(omega, 2.0) * std::sin(theta),
          -4.0 * params_.figure8_horizontal_Ay * std::pow(omega, 2.0) * std::sin(2.0 * theta), 0.0;
      break;
    case TRAJ_FIGURE8_VERTICAL:
      theta = params_.figure8_vertical_theta0 + phase_shift_ + omega * time;
      acceleration << 0.0, -params_.figure8_vertical_Ay * std::pow(omega, 2.0) * std::sin(theta),
          -4.0 * params_.figure8_vertical_Az * std::pow(omega, 2.0) * std::sin(2.0 * theta);
      break;
    case TRAJ_HELIX_FLIP:
      theta = params_.helix_flip_theta0 + phase_shift_ + omega * time;
      acceleration << 0.0, -params_.helix_flip_Ay * std::pow(omega, 2.0) * std::sin(theta),
          params_.helix_flip_Az * std::pow(omega, 2.0) * std::cos(theta);
      break;
    case TRAJ_HELIX_FLIP_Y:
      theta = params_.helix_flip_y_theta0 + phase_shift_ + omega * time;
      acceleration << -params_.helix_flip_y_Ax * std::pow(omega, 2.0) * std::sin(theta), 0.0,
          params_.helix_flip_y_Az * std::pow(omega, 2.0) * std::cos(theta);
      break;
    case TRAJ_FLIP_LOOP_SINE:
      theta = params_.flip_loop_sine_theta0 + phase_shift_ + omega * time;
      acceleration << 0.0, -params_.flip_loop_sine_Ay * std::pow(omega, 2.0) * std::sin(theta),
          params_.flip_loop_sine_Az * std::pow(omega, 2.0) * std::cos(theta);
      break;
    case TRAJ_FAST_CIRCLE:
      theta = params_.fast_circle_theta0 + phase_shift_ + omega * time;
      acceleration << -params_.fast_circle_Ax * std::pow(omega, 2.0) * std::cos(theta),
          -params_.fast_circle_Ay * std::pow(omega, 2.0) * std::sin(theta), 0.0;
      break;
    case TRAJ_RACE_TRACK_C:
      theta = M_PI / 2.0 + phase_shift_ + omega * time;
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
