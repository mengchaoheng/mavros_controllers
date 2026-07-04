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

#ifndef TRAJECTORYPUBLISHER_H
#define TRAJECTORYPUBLISHER_H

#include <stdio.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavconn/mavlink_dialect.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include "controller_msgs/FlatTarget.h"
#include <trajectory_publisher/TrajectoryPublisherConfig.h>
#include "trajectory_publisher/polynomialtrajectory.h"
#include "trajectory_publisher/shapetrajectory.h"
#include "trajectory_publisher/trajectory.h"

#define REF_TWIST 8
#define REF_SETPOINTRAW 16

using namespace std;
using namespace Eigen;
class trajectoryPublisher {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher trajectoryPub_;
  ros::Publisher referencePub_;
  ros::Publisher flatreferencePub_;
  ros::Publisher yawreferencePub_;
  ros::Publisher rawreferencePub_;
  ros::Publisher global_rawreferencePub_;
  std::vector<ros::Publisher> primitivePub_;
  ros::Subscriber motionselectorSub_;
  ros::Subscriber mavposeSub_;
  ros::Subscriber mavtwistSub_;
  ros::Subscriber mavstate_sub_;
  ros::ServiceServer trajtriggerServ_;
  ros::Timer trajloop_timer_;
  ros::Timer refloop_timer_;
  ros::Time start_time_, curr_time_;

  nav_msgs::Path refTrajectory_;
  nav_msgs::Path primTrajectory_;
  mavros_msgs::State current_state_;

  struct OmegaProfile {
    double default_value;
    double min;
    double max;
  };

  int trajectory_type_;
  Eigen::Vector3d p_targ, v_targ, a_targ, j_targ;
  Eigen::Vector3d p_mav_, v_mav_;
  Eigen::Vector3d shape_origin_, shape_axis_;
  Eigen::Matrix<double, 6, 3> transition_position_coeffs_;
  Eigen::Matrix<double, 6, 1> transition_yaw_coeffs_;
  Eigen::Vector3d transition_final_target_;
  double yaw_targ_;
  double trajectory_yaw_fixed_;
  double omega_value_;
  double omega_start_;
  double omega_end_;
  double omega_duration_;
  double shape_phase_shift_;
  double path_preview_cycles_;
  double transition_segment_duration_;
  double trajectory_switch_transition_duration_;
  double trajectory_switch_transition_min_duration_;
  double trajectory_switch_transition_max_duration_;
  double trajectory_switch_transition_velocity_limit_;
  double trajectory_switch_transition_acceleration_limit_;
  double trajectory_switch_stop_speed_threshold_;
  double theta_ = 0.0;
  double controlUpdate_dt_;
  double primitive_duration_;
  double trigger_time_;
  double init_pos_x_, init_pos_y_, init_pos_z_;
  double takeoff_position_tolerance_;
  double takeoff_velocity_tolerance_;
  double trajectory_start_ramp_configured_duration_;
  double trajectory_start_ramp_duration_;
  double trajectory_start_ramp_min_duration_;
  double trajectory_start_ramp_velocity_limit_;
  double trajectory_start_ramp_acceleration_limit_;
  double max_jerk_;
  int pubreference_type_;
  int num_primitives_;
  int motion_selector_;
  bool takeoff_before_trajectory_;
  bool adaptive_trajectory_start_ramp_;
  bool trajectory_yaw_lock_;
  bool trajectory_started_;
  bool first_reconfigure_;
  bool transition_active_;
  int omega_mode_;
  int transition_stage_;
  Eigen::Vector3d takeoff_target_;
  ros::Time transition_start_time_;
  std::shared_ptr<dynamic_reconfigure::Server<trajectory_publisher::TrajectoryPublisherConfig>> dyn_server_;
  shapetrajectory::Params shape_params_;
  OmegaProfile figure8_horizontal_omega_profile_;
  OmegaProfile figure8_vertical_omega_profile_;
  OmegaProfile helix_flip_omega_profile_;
  OmegaProfile helix_flip_y_omega_profile_;
  OmegaProfile flip_loop_sine_omega_profile_;
  OmegaProfile fast_circle_omega_profile_;

  std::vector<std::shared_ptr<trajectory>> motionPrimitives_;
  std::vector<Eigen::Vector3d> inputs_;

 public:
  trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  void updateReference();
  void pubrefTrajectory(int selector);
  void pubprimitiveTrajectory();
  void pubrefState();
  void pubflatrefState();
  void pubyawState();
  void pubrefSetpointRaw();
  void pubrefSetpointRawGlobal();
  void initializePrimitives(int type);
  void applyShapeParams();
  void updatePrimitives();
  void readShapeParams();
  void readOmegaProfiles();
  void readTrajectoryType();
  void resetTrajectoryStart();
  void startTrajectoryTransition();
  void startTransitionSegment(const Eigen::Vector3d& position_start, const Eigen::Vector3d& velocity_start,
                              const Eigen::Vector3d& acceleration_start, const Eigen::Vector3d& position_goal,
                              double yaw_start, double yaw_goal, double duration, int stage);
  void updateTransitionReference();
  void evaluateTransitionSegment(double time);
  double estimateTransitionDuration(double distance, double speed) const;
  double omegaDefaultForTrajectory(int type) const;
  std::pair<double, double> omegaRangeForTrajectory(int type) const;
  double clampToOmegaRange(double value, int type) const;
  bool activeShapeGeometryChanged(const trajectory_publisher::TrajectoryPublisherConfig& config) const;
  void updateOmegaProfilesFromConfig(const trajectory_publisher::TrajectoryPublisherConfig& config);
  void updateReferenceYaw(double trajectory_time);
  void seedDynamicReconfigure(trajectory_publisher::TrajectoryPublisherConfig& config);
  void updateShapeParamsFromConfig(const trajectory_publisher::TrajectoryPublisherConfig& config);
  bool configChangesTrajectory(const trajectory_publisher::TrajectoryPublisherConfig& config) const;
  void dynamicReconfigureCallback(trajectory_publisher::TrajectoryPublisherConfig& config, uint32_t level);
  void updateTakeoffTarget();
  void updateTrajectoryStartRampDuration();
  void setTakeoffReference();
  bool takeoffTargetReached();
  void applyTrajectoryStartRamp(double trajectory_time);
  void loopCallback(const ros::TimerEvent& event);
  void refCallback(const ros::TimerEvent& event);
  bool triggerCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  void motionselectorCallback(const std_msgs::Int32& selector);
  void mavposeCallback(const geometry_msgs::PoseStamped& msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped& msg);
  void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);
};

#endif
