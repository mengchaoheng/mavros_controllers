# trajectory_publisher

## Overview

Trajectory publisher publishes analytic flat-output references for the geometric controller. The current launch setup uses a selectable trajectory interface so one launch file can switch between supported shapes online through `rqt_reconfigure`.

`sitl_trajectory_direct.launch` uses one controller selector for all control paths:

```text
controller_type:=8 (px4_direct)
trajectory_publisher -> /mavros/setpoint_raw/local -> PX4 position controller

controller_type:=0..7
trajectory_publisher -> geometric_controller -> /mavros/setpoint_raw/attitude -> PX4
```

Select either path with `controller_type`:

```bash
roslaunch trajectory_publisher sitl_trajectory_direct.launch controller_type:=8
roslaunch trajectory_publisher sitl_trajectory_direct.launch controller_type:=0
```

`px4_direct` (`8`) is the default. It bypasses the external control law and makes `trajectory_publisher` stream P/V/A/yaw directly to PX4. Values `0..7` run the corresponding external controller. The same selector is available online in `rqt_reconfigure`, and changing it updates the active PX4 setpoint route. `geometric_controller` owns SITL OFFBOARD switching and arming for every selection.

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
- `/geometric_controller/controller_type` (`0..7`: external controllers, `8`: `px4_direct`)
- `/trajectory_publisher/omega_mode`
- `/trajectory_publisher/omega_value`
- `/trajectory_publisher/path_preview_cycles`
- `/trajectory_publisher/trajectory_yaw_lock`
- `/trajectory_publisher/trajectory_yaw_fixed`
- `/trajectory_publisher/initpos_z`
- `/trajectory_publisher/reference_type`
- `/trajectory_publisher/auto_offboard` (direct mode only; default: `false`)
- `/trajectory_publisher/auto_arm` (direct mode only; default: `false`)
- `/trajectory_publisher/preflight_setpoint_count` (default: `100`)
- `/trajectory_publisher/offboard_request_interval` (default: `2.0`)
- `/trajectory_publisher/arm_request_interval` (default: `2.0`)
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
  - `/mavros/setpoint_raw/local` (`mavros_msgs/PositionTarget`, when `reference_type=16`)

- Subscribed Topics
  - `/mavros/local_position/pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html))
  - `/mavros/local_position/velocity_local` ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))
