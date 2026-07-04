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

#ifndef TRAJECTORY_PUBLISHER_SHAPETRAJECTORY_H
#define TRAJECTORY_PUBLISHER_SHAPETRAJECTORY_H

#include <cmath>
#include <string>

#include "trajectory_publisher/trajectory.h"

#define TRAJ_FIGURE8_HORIZONTAL 1
#define TRAJ_FIGURE8_VERTICAL 2
#define TRAJ_HELIX_FLIP 3
#define TRAJ_HELIX_FLIP_Y 4
#define TRAJ_FLIP_LOOP_SINE 5
#define TRAJ_FAST_CIRCLE 6

#define TRAJ_OMEGA_FIXED 0
#define TRAJ_OMEGA_LINEAR 1
#define TRAJ_OMEGA_QUADRATIC 2

class shapetrajectory : public trajectory {
 public:
  struct Params {
    double figure8_horizontal_Ax = 2.0;
    double figure8_horizontal_Ay = 2.0;
    double figure8_horizontal_Hc = 3.0;
    double figure8_horizontal_theta0 = 0.0;

    double figure8_vertical_Ay = 2.0;
    double figure8_vertical_Az = 2.0;
    double figure8_vertical_Hc = 3.0;
    double figure8_vertical_theta0 = -M_PI / 4.0;

    double helix_flip_Ay = 2.0;
    double helix_flip_Az = 2.0;
    double helix_flip_Hc = 3.0;
    double helix_flip_Vx = 0.30;
    double helix_flip_theta0 = 0.0;

    double helix_flip_y_Ax = 2.0;
    double helix_flip_y_Az = 2.0;
    double helix_flip_y_Hc = 3.0;
    double helix_flip_y_Vy = 0.30;
    double helix_flip_y_theta0 = 0.0;

    double flip_loop_sine_Ay = 2.0;
    double flip_loop_sine_Az = 2.0;
    double flip_loop_sine_Hc = 3.0;
    double flip_loop_sine_Vx = 0.0;
    double flip_loop_sine_theta0 = 0.0;

    double fast_circle_Ax = 3.0;
    double fast_circle_Ay = 3.0;
    double fast_circle_Hc = 3.0;
    double fast_circle_theta0 = 0.0;
  };

 private:
  struct ScalarState {
    double p = 0.0;
    double v = 0.0;
    double a = 0.0;
    double j = 0.0;
  };

  struct ReferenceState {
    Eigen::Vector3d p{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a{Eigen::Vector3d::Zero()};
    Eigen::Vector3d j{Eigen::Vector3d::Zero()};
    double yaw = 0.0;
    double yaw_rate = 0.0;
    double yaw_acceleration = 0.0;
  };

  int type_;
  double dt_;
  double T_;
  Eigen::Vector3d traj_axis_;
  Eigen::Vector3d traj_origin_;
  double omega_value_;
  double omega_start_;
  double omega_end_;
  double omega_duration_;
  int omega_mode_;
  double phase_shift_;
  double path_preview_cycles_;
  Params params_;

  static double sanitizePositive(double value, double fallback);
  static ScalarState trigDerivatives(double amplitude, double theta, double theta_dot, double theta_ddot,
                                     double theta_3, const std::string& kind);
  ReferenceState evaluate(double time) const;
  void thetaState(double time, double& theta, double& theta_dot, double& theta_ddot, double& theta_3) const;
  void setHeadingFromVelocity(ReferenceState& ref, double default_yaw) const;
  double typeTheta0() const;

 public:
  shapetrajectory(int type);
  virtual ~shapetrajectory();
  void initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega);
  void setType(int type);
  void setParams(const Params& params);
  void setOmega(int mode, double value, double start, double end, double duration);
  void setPhaseShift(double phase_shift);
  void setPathPreviewCycles(double cycles);
  static int typeFromName(const std::string& name, int fallback);
  static const char* typeName(int type);
  void generatePrimitives(Eigen::Vector3d pos);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d jerk);
  Eigen::Vector3d getPosition(double time);
  Eigen::Vector3d getVelocity(double time);
  Eigen::Vector3d getAcceleration(double time);
  Eigen::Vector3d getJerk(double time);
  double getYaw(double time);
  double getYawRate(double time);
  double getYawAcceleration(double time);
  double getsamplingTime() { return dt_; };
  double getDuration() { return T_; };
  nav_msgs::Path getSegment();
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation);
};
#endif  // TRAJECTORY_PUBLISHER_SHAPETRAJECTORY_H
