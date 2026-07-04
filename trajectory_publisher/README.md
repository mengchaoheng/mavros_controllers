# trajectory_publisher

## Overview

Trajectory publisher publishes analytic flat-output references for the geometric controller. The current launch setup uses a selectable trajectory interface so one launch file can switch between supported shapes online through `rqt_reconfigure`.

## Supported Trajectories

The trajectory names follow `main.m`:

- `figure8_horizontal`
- `figure8_vertical`
- `helix_flip`
- `helix_flip_y`
- `flip_loop_sine`
- `fast_circle`

Older names are treated as aliases where needed, but new launch files should use the names above.

## Online Tuning

The trajectory publisher supports dynamic reconfigure for:

- `trajName`
- `omega_mode`
- `omega_value`
- trajectory-specific shape parameters
- trajectory-specific omega defaults/ranges
- `trajectory_yaw_lock`
- `trajectory_yaw_fixed`
- `path_preview_cycles`
- takeoff and trajectory-switch transition timing

Changing trajectory shape or trajectory type transitions through a smooth switch-to-start behavior. Changing `omega_value` updates the frequency without forcing a restart to the trajectory start point.

## Parameters

- `/trajectory_publisher/trajName` (default: `figure8_horizontal` in SITL launch)
- `/trajectory_publisher/omega_mode`
- `/trajectory_publisher/omega_value`
- `/trajectory_publisher/path_preview_cycles`
- `/trajectory_publisher/trajectory_yaw_lock`
- `/trajectory_publisher/trajectory_yaw_fixed`
- `/trajectory_publisher/initpos_z`
- `/trajectory_publisher/reference_type`
- `/trajectory_publisher/takeoff_before_trajectory`
- `/trajectory_publisher/adaptive_trajectory_start_ramp`
- `/trajectory_publisher/trajectory_start_ramp_duration`
- `/trajectory_publisher/trajectory_start_ramp_min_duration`
- `/trajectory_publisher/trajectory_start_ramp_velocity_limit`
- `/trajectory_publisher/trajectory_start_ramp_acceleration_limit`
- `/trajectory_publisher/trajectory_switch_transition_duration`
- `/trajectory_publisher/trajectory_switch_transition_min_duration`
- `/trajectory_publisher/trajectory_switch_transition_max_duration`
- `/trajectory_publisher/trajectory_switch_transition_velocity_limit`
- `/trajectory_publisher/trajectory_switch_transition_acceleration_limit`
- `/trajectory_publisher/trajectory_switch_stop_speed_threshold`

## Topics

- Published Topics
  - `reference/trajectory` ([nav_msgs/Path](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Path.html))
  - `reference/flatsetpoint` (`controller_msgs/FlatTarget`)
  - `reference/yaw` (`std_msgs/Float32`)

- Subscribed Topics
  - `/mavros/local_position/pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html))
  - `/mavros/local_position/velocity_local` ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))
