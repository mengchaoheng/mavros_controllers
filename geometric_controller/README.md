# geometric_controller

Trajectory tracking controller using [mavros](https://github.com/mavlink/mavros) in PX4 OFFBOARD mode.

The package now has a small controller framework: the ROS node owns subscriptions, publications, state machine, launch parameters, and dynamic reconfigure; each controller owns only the control calculation and returns a PX4/MAVROS body-rate plus normalized-thrust command.

## Controller Layout

- ROS wrapper: `geometric_controller/src/geometric_controller.cpp`
- Shared controller data types: `geometric_controller/include/geometric_controller/controller_types.h`
- Shared controller interface: `geometric_controller/include/geometric_controller/controller_base.h`
- Original controller: `geometric_controller/src/legacy_geometric_controller.cpp`
- Shared SO(3), heading, flatness, and reference-rate utilities: `geometric_controller/src/main_controller_math.cpp`
- main.m-style controllers:
  - `geometric_controller/src/main_geometric_controller.cpp`
  - `geometric_controller/src/main_lee_controller.cpp`
  - `geometric_controller/src/main_johnson_controller.cpp`
  - `geometric_controller/src/main_sun_dfbc_controller.cpp`
  - `geometric_controller/src/main_tal_controller.cpp`
  - `geometric_controller/src/main_geometric_indi_controller.cpp`

`pubReferencePose()` and `pubRateCommands()` remain shared ROS publishing functions in `geometric_controller.cpp`. Controller classes do not publish ROS messages directly.

## Supported Controllers

The active controller is selected with `/geometric_controller/controller_type`, either from launch or `rqt_reconfigure`.

```bash
roslaunch geometric_controller sitl_trajectory_track.launch controller_type:=1
```

Values:

- `0`: `legacy_geometric`
- `1`: `main_geometric`
- `2`: `main_lee`
- `3`: `main_johnson`
- `4`: `main_sun_dfbc`
- `5`: `main_sun_dfbc_indi`
- `6`: `main_tal`
- `7`: `main_geometric_indi`

All controllers output:

```text
body_rate_sp = Omega_ref + KR * attitude_error
thrust_norm = normalizedthrust_constant * thrust_accel + normalizedthrust_offset
```

PX4's internal body-rate controller is the angular-rate loop. Therefore this ROS controller does not expose `KOmega`. The terms that are angular-rate feedback, angular acceleration feedforward, moment commands, or actuator INDI in `main.m` are documented in the corresponding controller source files as PX4-side or unavailable-through-this-interface terms.

## Parameters

- `/geometric_controller/mavname` (default: `iris`)
- `/geometric_controller/ctrl_mode` (default: `MODE_BODYRATE`)
- `/geometric_controller/controller_type` (default: `0`)
- `/geometric_controller/cmdloop_rate` (default: `250.0`)
- `/geometric_controller/enable_sim` (default: `true`)
- `/geometric_controller/enable_gazebo_state` (default: `false`)
- `/geometric_controller/max_acc` (default: `10.0`)
- `/geometric_controller/yaw_heading` (default: `0.0`)
- `/geometric_controller/drag_dx` (default: `0.0`)
- `/geometric_controller/drag_dy` (default: `0.0`)
- `/geometric_controller/drag_dz` (default: `0.0`)
- `/geometric_controller/Kp_x`, `Kp_y`, `Kp_z`
- `/geometric_controller/Kv_x`, `Kv_y`, `Kv_z`
- `/geometric_controller/KR_r`, `KR_p`, `KR_y`
- `/geometric_controller/normalizedthrust_constant` (default: `0.0220`)
- `/geometric_controller/normalizedthrust_offset` (default: `0.0`)

`Kp`, `Kv`, and `KR` are shared controller gains, following the same idea as the unified gain namespace in `main.m`. The legacy controller internally maps `KR` to the old attitude-control time constant as `attctrl_tau = 1 / KR`; this is an implementation detail and is not a public parameter.

`max_acc` clamps feedback acceleration from the position loop. It is not the physical thrust limit of the PX4 model.

## PX4 Iris Thrust Scaling

The launch files and defaults assume the PX4/Gazebo Classic `iris` model.

The controller computes collective thrust in acceleration units:

```text
thrust_accel = T / m
```

MAVROS/PX4 expects normalized collective thrust:

```text
thrust_norm = normalizedthrust_constant * thrust_accel + normalizedthrust_offset
```

The physical relation is:

```text
normalizedthrust_constant = m / T_max_total = 1 / (T_max_total / m)
```

For the Iris model:

```text
T_i,max = motorConstant * maxRotVelocity^2
T_max_total = 4 * T_i,max
a_T,max = T_max_total / m
```

With typical Iris values:

```text
m = 0.75 kg
motorConstant = 1.51e-6 N/(rad/s)^2
maxRotVelocity = 2372.6 rad/s
T_i,max ~= 8.5 N
T_max_total ~= 34 N
a_T,max ~= 45.3 m/s^2
m / T_max_total ~= 0.022
```

The net maximum vertical acceleration after gravity is approximately:

```text
a_up,max = a_T,max - g
```

## PX4 Gazebo Classic Environment

Use `~/PX4-Autopilot` as the PX4 path:

```bash
cd ~/PX4-Autopilot
source ~/mavros_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 posix_sitl.launch
```

For older PX4-Autopilot Gazebo layouts, the Gazebo package path may instead be:

```bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

## Topics

The geometric controller publishes and subscribes the following topics.

- Published Topics
  - `command/bodyrate_command` ([mavros_msgs/AttitudeTarget](http://docs.ros.org/api/mavros_msgs/html/msg/AttitudeTarget.html))
  - `reference/pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html))

- Subscribed Topics
  - `reference/setpoint` ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))
  - `reference/flatsetpoint` (`controller_msgs/FlatTarget`)
  - `reference/yaw` (`std_msgs/Float32`)
  - `/mavros/state` ([mavros_msgs/State](http://docs.ros.org/api/mavros_msgs/html/msg/State.html))
  - `/mavros/local_position/pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html))
  - `/mavros/local_position/velocity_local` ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))

## Contact

Jaeyoung Lim 	jalim@student.ethz.ch
