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
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
```
To source the PX4 environment, run the following commands

```bash
cd ~/PX4-Autopilot
source ~/mavros_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```

For older PX4-Autopilot Gazebo layouts, the Gazebo package path may instead be:

```bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

For current PX4-Autopilot Gazebo Classic layouts, use:

```bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```

You can run the rest of the roslaunch files in the same terminal:

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
cd ~/PX4-Autopilot
source ~/mavros_ws/devel/setup.bash    # (necessary)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```
The following launch file enables the geometric controller to follow a selectable trajectory

``` bash
roslaunch geometric_controller sitl_trajectory_track.launch
```

The controller can be selected online through `rqt_reconfigure` or at launch:

```bash
roslaunch geometric_controller sitl_trajectory_track.launch controller_type:=1
```

`controller_type` values are:

- `0`: `legacy_geometric`
- `1`: `main_geometric`
- `2`: `main_lee`
- `3`: `main_johnson`
- `4`: `main_sun_dfbc`
- `5`: `main_sun_dfbc_indi`
- `6`: `main_tal`
- `7`: `main_geometric_indi`

All controllers publish PX4/MAVROS body-rate setpoints and normalized collective thrust. The ROS side does not own an angular-rate loop gain such as `KOmega`; PX4's internal rate controller is treated as the inner loop.

If the UAV does not takeoff, please open QGroundControl and enable virtual joystick as mentioned [here](https://docs.qgroundcontrol.com/master/en/SettingsView/VirtualJoystick.html)

## Nodes
`mavros_controllers` include the following packages.
### geometric_controller

The geometric controller publishes and subscribes the following topics.
- Controller layout
    - The ROS node wrapper is implemented in `geometric_controller/src/geometric_controller.cpp`.
    - Shared controller I/O structs and the controller base class are in `geometric_controller/include/geometric_controller/controller_types.h` and `controller_base.h`.
    - The original controller was moved into `geometric_controller/src/legacy_geometric_controller.cpp`.
    - The new main.m-style controllers live in separate files: `main_geometric_controller.cpp`, `main_lee_controller.cpp`, `main_johnson_controller.cpp`, `main_sun_dfbc_controller.cpp`, `main_tal_controller.cpp`, and `main_geometric_indi_controller.cpp`.
    - Common SO(3), flatness, attitude-reference, and reference-rate helper functions are in `main_controller_math.cpp`.
    - `pubReferencePose()` and `pubRateCommands()` remain common ROS publishing functions in `geometric_controller.cpp`.

- Parameters
    - /geometric_controller/mavname (default: "iris")
    - /geometric_controller/ctrl_mode (default: MODE_BODYRATE)
    - /geometric_controller/controller_type (default: 0)
    - /geometric_controller/cmdloop_rate (default: 250.0)
    - /geometric_controller/enable_sim (default: true)
    - /geometric_controller/enable_gazebo_state (default: false)
    - /geometric_controller/max_acc (default: 10.0)
    - /geometric_controller/yaw_heading (default: 0.0)
    - /geometric_controller/drag_dx (default: 0.0)
    - /geometric_controller/drag_dy (default: 0.0)
    - /geometric_controller/drag_dz (default: 0.0)
    - /geometric_controller/Kp_x, Kp_y, Kp_z
    - /geometric_controller/Kv_x, Kv_y, Kv_z
    - /geometric_controller/KR_r, KR_p, KR_y
    - /geometric_controller/normalizedthrust_constant (default: 0.0220)
    - /geometric_controller/normalizedthrust_offset (default: 0.0)

- PX4 Iris thrust scaling
    - The SITL model is assumed to be the PX4/Gazebo Classic `iris`.
    - The controller computes collective thrust in acceleration units, `thrust_accel = T / m`.
    - MAVROS/PX4 expects normalized thrust, so the conversion is `thrust_norm = normalizedthrust_constant * thrust_accel + normalizedthrust_offset`.
    - Physically, `normalizedthrust_constant = m / T_max_total = 1 / (T_max_total / m)`.
    - For the Iris model, `T_i,max = motorConstant * maxRotVelocity^2`, `T_max_total = 4 * T_i,max`, and `a_T,max = T_max_total / m`.
    - With typical Iris values `m = 0.75 kg`, `motorConstant = 1.51e-6 N/(rad/s)^2`, and `maxRotVelocity = 2372.6 rad/s`, each rotor produces about `8.5 N`; total thrust is about `34 N`; `a_T,max` is about `45.3 m/s^2`; therefore `m / T_max_total` is about `0.022`.
    - Net maximum vertical acceleration after gravity is approximately `a_T,max - g`.
    - /geometric_controller/max_acc is a feedback-acceleration clamp, not the physical motor thrust limit.

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
    - /trajectory_publisher/trajectory_type (default: 0)
    - /trajectory_publisher/number_of_primitives (default: 7)
    - /trajectory_publisher/shape_radius (default: 1.0)

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

References
----------

[1] T. Lee, M. Leok, and N. H. McClamroch, “Geometric Tracking Control of a Quadrotor UAV on SE(3),” Mar. 10, 2010, arXiv: arXiv:1003.2005. doi: [10.48550/arXiv.1003.2005](https://doi.org/10.48550/arXiv.1003.2005).

[2] T. Lee, M. Leok, and N. H. McClamroch, “Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3),” Nov. 12, 2010, arXiv: arXiv:1003.2005. doi: [10.48550/arXiv.1003.2005](https://doi.org/10.48550/arXiv.1003.2005).

[3] F. Bullo and R. M. Murray, “Proportional derivative (PD) control on the Euclidean group,” 1995.

[4] Y. Yu, S. Yang, M. Wang, C. Li, and Z. Li, “High performance full attitude control of a quadrotor on SO (3),” in 2015 IEEE International Conference on Robotics and Automation (ICRA), Seattle, WA, USA: IEEE, 2015, pp. 1698–1703. doi: [10.1109/icra.2015.7139416](https://doi.org/10.1109/icra.2015.7139416).

[5] D. Brescianini, M. Hehn, and R. D’Andrea, “Nonlinear Quadrocopter Attitude Control: Technical Report,” ETH Zurich, 2013. doi: [10.3929/ETHZ-A-009970340](https://doi.org/10.3929/ETHZ-A-009970340).

[6] M. Faessler, A. Franchi, and D. Scaramuzza, “Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories,” IEEE Robotics and Automation Letters, vol. 3, no. 2, pp. 620–626, Apr. 2018, doi: [10.1109/LRA.2017.2776353](https://doi.org/10.1109/LRA.2017.2776353).

[7] J. Sola, “Quaternion kinematics for the error-state Kalman filter,” arXiv:1711.02508 [cs], Nov. 2017, Accessed: Sep. 26, 2020. [Online]. Available: [http://arxiv.org/abs/1711.02508](http://arxiv.org/abs/1711.02508)

[8] D. Brescianini and R. D’Andrea, “Tilt-Prioritized Quadrocopter Attitude Control,” IEEE Transactions on Control Systems Technology, vol. 28, no. 2, pp. 376–387, Mar. 2020, doi: [10.1109/TCST.2018.2873224](https://doi.org/10.1109/TCST.2018.2873224).

[9] J. Johnson and R. Beard, “Globally-Attractive Logarithmic Geometric Control of a Quadrotor for Aggressive Trajectory Tracking,” Dec. 01, 2021, arXiv: arXiv:2109.07025. doi: [10.48550/arXiv.2109.07025](https://doi.org/10.48550/arXiv.2109.07025).

[10] J. Sola, J. Deray, and D. Atchuthan, “A micro Lie theory for state estimation in robotics,” Dec. 08, 2021, arXiv: arXiv:1812.01537. doi: [10.48550/arXiv.1812.01537](https://doi.org/10.48550/arXiv.1812.01537).

[11] E. Tal and S. Karaman, “Accurate Tracking of Aggressive Quadrotor Trajectories Using Incremental Nonlinear Dynamic Inversion and Differential Flatness,” IEEE Trans. Contr. Syst. Technol., vol. 29, no. 3, pp. 1203–1218, May 2021, doi: [10.1109/tcst.2020.3001117](https://doi.org/10.1109/tcst.2020.3001117).

[12] S. Sun, A. Romero, P. Foehn, E. Kaufmann, and D. Scaramuzza, “A Comparative Study of Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor Agile Flight,” Feb. 23, 2022, arXiv: arXiv:2109.01365. Accessed: May 27, 2022. [Online]. Available: [http://arxiv.org/abs/2109.01365](http://arxiv.org/abs/2109.01365)

[13] G. Lu, W. Xu, and F. Zhang, “On-Manifold Model Predictive Control for Trajectory Tracking on Robotic Systems,” IEEE Transactions on Industrial Electronics, vol. 70, no. 9, pp. 9192–9202, Sep. 2023, doi: [10.1109/TIE.2022.3212397](https://doi.org/10.1109/TIE.2022.3212397).

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
cd ~/PX4-Autopilot
make px4_sitl_default gazebo-classic_iris
```

or refer to the [PX4-Autopilot issues](https://github.com/PX4/PX4-Autopilot/issues?q=%2Firis%2Firis.sdf+) and the [ROS with Gazebo Simulation PX4 Documentation](https://dev.px4.io/master/en/simulation/ros_interface.html). 
