# mavros_controllers
[![Build Test](https://github.com/Jaeyoung-Lim/mavros_controllers/workflows/Build%20Test/badge.svg)](https://github.com/Jaeyoung-Lim/mavros_controllers/actions?query=workflow%3A%22Build+Test%22) [![DOI](https://zenodo.org/badge/140596755.svg)](https://zenodo.org/badge/latestdoi/140596755)

Controllers for controlling MAVs using the [mavros](https://github.com/mavlink/mavros) package in OFFBOARD mode.


## Overview
The repository contains controllers for controlling MAVs using the mavros package. The following packages are included in the repo
- geometric_controller: Trajectory tracking controller based on geometric control
- controller_msgs: custom message definitions
- trajectory_publisher: Node publishing setpoints as states from motion primitives / trajectories for the controller to follow.

[![Multiple drone](https://user-images.githubusercontent.com/5248102/87020057-a3e25200-c1d3-11ea-9f76-cd010cb8329a.gif)](https://user-images.githubusercontent.com/5248102/87020057-a3e25200-c1d3-11ea-9f76-cd010cb8329a.gif)

[![Hovering drone](https://img.youtube.com/vi/FRaPGjX9m-c/0.jpg)](https://youtu.be/FRaPGjX9m-c "Hovering done")

[![Circular trajectory tracking](https://img.youtube.com/vi/IEyocdnlYw0/0.jpg)](https://youtu.be/IEyocdnlYw0 "Circular trajectory tracking")

## Getting Started
### Install PX4 SITL(Only to Simulate)
Follow the instructions as shown in the [ROS with Gazebo Simulation PX4 Documentation](https://dev.px4.io/master/en/simulation/ros_interface.html)
To check if the necessary environment is setup correctly, you can run the gazebo SITL using the following command

```bash
cd <Firmware_directory>
DONT_RUN=1 make px4_sitl_default gazebo
```
To source the PX4 environment, run the following commands

```bash
cd ~/PX4-Autopilot
source ~/mavros_ws/devel/setup.bash
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 posix_sitl.launch
```

You can run the rest of the roslaunch files in the same terminal

```bash
 roslaunch px4 posix_sitl.launch
```

You will need to source the PX4 environment in every new terminal you open to launch mavros_controllers. 

### Installing mavros_controllers

Create a catkin workspace:

This folder will probably be already created since the previous process would have created it. If it is not present, do:

```bash
mkdir -p ~/mavros_ws/src
cd ~/mavros_ws
catkin init
catkin config --merge-devel
cd ~/mavros_ws/src
wstool init
```

###### Clone this repository

```bash
cd ~/mavros_ws/src
git clone https://github.com/Jaeyoung-Lim/mavros_controllers
```

Now continue either with wstool to automatically download dependencies or download them manually.

###### With wstool

wstool automates the installation of dependencies and updates all packages. If you have no problem updating the packages required by mavros_controllers and/or any other packages, follow this procedure. If not, follow the next 'Manually Download dependencies and build' section.

```bash
cd ~/mavros_ws
wstool merge -t src src/mavros_controllers/dependencies.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO
catkin build
source ~/mavros_ws/devel/setup.bash
```


###### Manually Download dependencies and build

If you did not install with wstool, you need to manually download the dependencies:
- [catkin_simple](https://github.com/catkin/catkin_simple)
- [eigen_catkin](https://github.com/ethz-asl/eigen_catkin)
- [mav_comm](https://github.com/ethz-asl/mav_comm)

Do:

```bash
cd ~/mavros_ws/src
git clone https://github.com/catkin/catkin_simple
git clone https://github.com/ethz-asl/eigen_catkin
git clone https://github.com/ethz-asl/mav_comm
```

Build all the packages:

```bash
cd ~/mavros_ws
catkin build
source ~/mavros_ws/devel/setup.bash
```

## Running the code
Remember to source the workspace `setup.bash` before sourcing the PX4 environment.
```bash
cd <Firmware_directory>
source ~/mavros_ws/devel/setup.bash    # (necessary)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```
The following launch file starts the generic SITL trajectory-tracking case with online controller and trajectory switching.

``` bash
roslaunch geometric_controller sitl_trajectory_track.launch
```

If the UAV does not takeoff, please open QGroundControl and enable virtual joystick as mentioned [here](https://docs.qgroundcontrol.com/master/en/SettingsView/VirtualJoystick.html)

## Nodes
`mavros_controllers` include the following packages.
### geometric_controller

The geometric controller publishes and subscribes the following topics.
- Parameters
    - /geometric_controller/mavname (default: "iris")
    - /geometric_controller/ctrl_mode (default: MODE_BODYRATE)
    - /geometric_controller/enable_sim (default: true)
    - /geometric_controller/enable_gazebo_state (default: false)
    - /geometric_controller/max_acc (default: 10.0; ROS outer-loop feedback acceleration clip)
    - /geometric_controller/yaw_heading (default: 0.0)
    - /geometric_controller/drag_dx (default: 0.0)
    - /geometric_controller/drag_dy (default: 0.0)
    - /geometric_controller/drag_dz (default: 0.0)
    - /geometric_controller/KR_x (default: 3.0; attitude error to body-rate gain)
    - /geometric_controller/KR_y (default: 3.0; attitude error to body-rate gain)
    - /geometric_controller/KR_z (default: 2.0; attitude error to body-rate gain)
    - /geometric_controller/normalizedthrust_constant (default: 0.02206 for Iris specific-force commands)
    - /geometric_controller/normalizedthrust_offset (default: 0.0)

- Published Topics
	- command/bodyrate_command ( [mavros_msgs/AttitudeTarget](http://docs.ros.org/api/mavros_msgs/html/msg/AttitudeTarget.html) )
	- reference/pose ( [geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html) )

- Subscribed Topics
	- reference/setpoint ( [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) )
	- /mavros/state ( [mavros_msgs/State](http://docs.ros.org/api/mavros_msgs/html/msg/State.html) )
	- /mavros/local_position/pose ( [geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html) )
	- /gazebo/model_states( [gazebo_msgs/ModelStates](http://docs.ros.org/kinetic/api/gazebo_msgs/html/msg/ModelState.html) )
	- /mavros/local_position/velocity( [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) )

### trajectory_publisher

Trajectory publisher publishes continous trajectories to the trajectory_controller.
- Parameters
    - /trajectory_publisher/initpos_x (default: 0.0)
    - /trajectory_publisher/initpos_y (default: 0.0)
    - /trajectory_publisher/initpos_z (default: 1.0)
    - /trajectory_publisher/updaterate (default: 0.01)
    - /trajectory_publisher/horizon (default: 1.0)
    - /trajectory_publisher/maxjerk (default: 10.0)
    - /trajectory_publisher/trajName (default: 1)
    - /trajectory_publisher/trajIntensity (default: 0.75)
    - /trajectory_publisher/Tend (default: 10.0)
    - /trajectory_publisher/helixTurns (default: 5.0)
    - /trajectory_publisher/raceTrackMaxSpeed (default: 19.4)
    - /trajectory_publisher/trajectorySpeed (default: 0.3 m/s in the generic launch)
    - /trajectory_publisher/shapeOmega (legacy circle/lamniscate angular rate)
    - /trajectory_publisher/waitStartBeforeTrajectory (default: true)
    - /trajectory_publisher/startPositionTolerance (default: 0.25)
    - /trajectory_publisher/startVelocityTolerance (default: 0.5)
    - /trajectory_publisher/startHoldDuration (default: 3.0)
    - /trajectory_publisher/number_of_primitives (default: 7, legacy polynomial mode)

- Published Topics
	- reference/trajectory ( [nav_msgs/Path](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Path.html) )
	- reference/setpoint ( [geometry_msgs/TwistStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Twist.html) )

- Subscribed Topics
    - /trajectory_publisher/motionselector ([std_msgs/int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html));
    - /mavros/local_position/pose ( [geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html) )
    - /mavros/local_position/velocity( [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) )

## Contact
Jaeyoung Lim 	jalim@ethz.ch

## Citation
In case you use this work as an academic context, please cite as the following.
```
@misc{jaeyoung_lim_2019_2619313,
  author       = {Jaeyoung Lim},
  title        = {{mavros_controllers - Aggressive trajectory 
                   tracking using mavros for PX4 enabled vehicles}},
  month        = mar,
  year         = 2019,
  doi          = {10.5281/zenodo.2652888},
  url          = {https://doi.org/10.5281/zenodo.2652888}
}
```

## References
[1] Lee, Taeyoung, Melvin Leoky, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." Decision and Control (CDC), 2010 49th IEEE Conference on. IEEE, 2010.

[2] Faessler, Matthias, Antonio Franchi, and Davide Scaramuzza. "Differential flatness of quadrotor dynamics subject to rotor drag for accurate tracking of high-speed trajectories." IEEE Robot. Autom. Lett 3.2 (2018): 620-626.

### Build issues:


###### catkin_simple() or eigen_catkin() not found

 This should not have happened if you clone the catkin_simple and eigen_catkin repositories. Try again:

```bash
cd ~/mavros_ws/src
git clone https://github.com/catkin/catkin_simple
git clone https://github.com/ethz-asl/eigen_catkin
cd ~/mavros_ws
catkin build mavros_controllers
source ~/mavros_ws/devel/setup.bash
```

- Refer to [this issue](https://github.com/Jaeyoung-Lim/mavros_controllers/issues/61).

###### iris.sdf model not found: 

Try:
```bash
cd <Firmware_directory>
make px4_sitl_default sitl_gazebo
```

or refer to [this issue](https://github.com/PX4/Firmware/issues?utf8=%E2%9C%93&q=%2Firis%2Firis.sdf+) the [ROS with Gazebo Simulation PX4 Documentation](https://dev.px4.io/master/en/simulation/ros_interface.html). 

## Online Controller and Trajectory Switching

The benchmark controller and trajectory names follow `main.m`. The ROS node exposes them through `dynamic_reconfigure`, so they can be switched while the nodes are running. `controllerName=0` keeps the original package controller path unchanged.

Start a SITL tracking case first:

```bash
roslaunch geometric_controller sitl_trajectory_track.launch
```

`sitl_trajectory_track.launch` is the generic online-switching entry point. By default it starts with `visualization=false`, `controllerName=0` (`legacy`), `trajName=1` (`circle`), and `trajectorySpeed=0.3` m/s. The older `*_circle.launch` and `*_lamniscate.launch` files are kept as compatibility examples.

The launch file starts `rqt_reconfigure` automatically. In the GUI:

- select `/geometric_controller` and change `controllerName`
- select `/trajectory_publisher` and change `trajName`
- tune gains online with `Kp_x/y/z`, `Kv_x/y/z`, `KR_x/y/z`
- tune INDI feedback with `indiAccelFeedback` and `indiFilterCutoffHz`
- keep `waitStartBeforeTrajectory` enabled to hold the new trajectory start point until the vehicle reaches it

For the original package controller, `KR` is converted internally to the old attitude time constant with `tau = 2 / mean(KR_x, KR_y, KR_z)`.
For the active body-rate controllers, `Kp=[10,10,10]` and `Kv=[6,6,6]` match `main.m`; the current ROS default attitude-to-body-rate gain is `KR=[3,3,2]`. The sent angular-rate command has no body-rate magnitude limit; for `geometric_indi`, the INDI thrust-vector attitude is mapped through `KR * attitude_error` without finite-differencing the feedback-shaped attitude.
The thrust sent to PX4 is `clamp(normalizedthrust_constant * thrust + normalizedthrust_offset, 0, 1)`.
In the current body-rate adaptation, `thrust` is specific force in m/s^2, so the Iris-aligned scale is `m/Tmax = 0.75 / (4 * 8.5) ~= 0.02206`, close to `MPC_THR_HOVER / g = 0.216 / 9.81 ~= 0.02202`. If this node is changed to command physical thrust in newtons, use `1/Tmax ~= 0.02941` instead.
At startup and after each `trajName` change, the trajectory publisher commands the trajectory start point with zero velocity and zero acceleration. The trajectory time starts only after the vehicle is armed, in OFFBOARD mode, inside `startPositionTolerance`/`startVelocityTolerance`, and after hovering there for `startHoldDuration`.

The same switches can be done from the command line:

```bash
rosrun dynamic_reconfigure dynparam set /geometric_controller controllerName 0
rosrun dynamic_reconfigure dynparam set /trajectory_publisher trajName 1
rosrun dynamic_reconfigure dynparam set /trajectory_publisher trajectorySpeed 0.3
```

Controller choices:

| value | `controllerName` |
| --- | --- |
| 0 | `legacy` |
| 1 | `geometric` |
| 2 | `lee` |
| 3 | `johnson` |
| 7 | `geometric_indi` |

Trajectory choices:

| value | `trajName` |
| --- | --- |
| 0 | `zero` |
| 1 | `circle` |
| 2 | `lamniscate` |
| 3 | `stationary` |
| 4 | `figure8_horizontal` |
| 5 | `figure8_vertical` |
| 6 | `helix_flip` |
| 7 | `helix_flip_y` |
| 8 | `flip_loop_sine` |
| 9 | `fast_circle` |
| 10 | `race_track_c` |

Example tracking cases:

```bash
# Lee controller on fast_circle
rosrun dynamic_reconfigure dynparam set /geometric_controller controllerName 2
rosrun dynamic_reconfigure dynparam set /trajectory_publisher trajName 9

# Geometric INDI on race_track_c
rosrun dynamic_reconfigure dynparam set /geometric_controller controllerName 7
rosrun dynamic_reconfigure dynparam set /trajectory_publisher trajName 10
```

Trajectory intensity and timing are also online parameters:

```bash
rosrun dynamic_reconfigure dynparam set /geometric_controller max_acc 3.0
rosrun dynamic_reconfigure dynparam set /trajectory_publisher trajIntensity 0.75
rosrun dynamic_reconfigure dynparam set /trajectory_publisher Tend 10.0
rosrun dynamic_reconfigure dynparam set /trajectory_publisher helixTurns 5.0
rosrun dynamic_reconfigure dynparam set /trajectory_publisher raceTrackMaxSpeed 19.4
rosrun dynamic_reconfigure dynparam set /trajectory_publisher trajectorySpeed 0.3
rosrun dynamic_reconfigure dynparam set /trajectory_publisher startPositionTolerance 0.25
rosrun dynamic_reconfigure dynparam set /trajectory_publisher startVelocityTolerance 0.5
rosrun dynamic_reconfigure dynparam set /trajectory_publisher startHoldDuration 3.0
```

For slow constant-speed flight, use `trajName=1` (`circle`) and tune `trajectorySpeed` directly. Start around `0.3` m/s, then increase gradually after the vehicle can hold the path cleanly.

Switching `trajName` resets the trajectory start state and waits at the new start point before playing the trajectory.
