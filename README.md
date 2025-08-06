# [NMPC Nav Control](https://github.com/JorgeDFR/nmpc_nav_control)

This repository provides a ROS package with a Nonlinear Model Predictive Control (NMPC) framework for mobile robot navigation using the `acados` open source library. This package supports various robot models, including omnidirectional, differential, and tricycle steering geometries, and allows for path following and pose navigation.

## Features

**Version 1.0.0**

With this version, the package provides the following features:
- Supports multiple robot steering geometries: omnidirectional (`omni4`), differential (`diff`), and tricycle (`tric`).
- Provides both pose navigation and path following.
- Publishes robot control status and actual paths.
- Real-time robot pose and steering angle updates via TF transforms.
- Provides Python scripts for modify and generate acados models for all the robot steering geometries.
- Provides a `FindACADOS.cmake` file that simplifies the integration of acados by automatically locating its headers and libraries.

## ROS

**ROS 1**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp/)
- [rospy](https://wiki.ros.org/rospy/)
- [tf2](https://index.ros.org/p/tf2/)
- [tf2_ros](https://index.ros.org/p/tf2_ros/)
- [geometry_msgs](https://index.ros.org/p/geometry_msgs/)
- [nav_msgs](https://index.ros.org/p/nav_msgs/)
- [itrci_nav](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/tree/main/itrci_nav)
- [parametric_trajectories_common](https://gitlab.inesctec.pt/mrdt/msc/trajectory-control-and-collision-avoidance-for-a-mobile-robot/parametric_trajectories_stack/-/tree/main)
- [Eigen3](https://neuro.debian.net/pkgs/libeigen3-dev.html)
- [acados](https://docs.acados.org/)

## Parameters

- `steering_geometry` (string, **required**): The robot’s steering geometry. Supported values: `omni4`, `diff`, `tric`
- `global_frame_id` (string, default: `map`): The frame ID for the global reference frame
- `base_frame_id` (string, default: `base_footprint`): The frame ID for the robot base
- `control_freq` (int, default: `40`): The control loop frequency (Hz)
- `transform_timeout` (double, default: `0.1`): Timeout for TF transformations (s)
- `max_active_path_length` (double, default: `5.0`): Maximum active path length (m)
- `final_position_error` (double, default: `0.01`): Final position tolerance for reaching the goal (m)
- `final_orientation_error` (double, default: `1.0`): Final orientation tolerance for reaching the goal (deg)

#### Omnidirectional (`omni4`)
- `rob_dist_between_front_back_wh` (double, **required**): Distance between the front and back wheels (m)
- `rob_dist_between_left_right_wh` (double, **required**): Distance between the left and right wheels (m)
- `rob_wh_vel_time_const` (double, **required**): Robot wheel velocity time constant (s)
- `rob_wh_max_vel` (double, **required**): Robot wheel maximum linear velocity (m/s)
- `rob_wh_max_ace` (double, **required**): Robot wheel maximum linear acceleration (m/s^2)
- `cost_matrix_weights_state_diag` (array, **required**): Cost matrix weights regarding the state (diagonal elements)
- `cost_matrix_weights_input_diag` (array, **required**): Cost matrix weights regarding the input (diagonal elements)

#### Differential (`diff`)
- `rob_dist_between_wh` (double, **required**): Distance between the left and right wheels (m)
- `rob_wh_vel_time_const` (double, **required**): Robot wheel velocity time constant (s)
- `rob_wh_max_vel` (double, **required**): Robot wheel maximum linear velocity (m/s)
- `rob_wh_max_ace` (double, **required**): Robot wheel maximum linear acceleration (m/s^2)
- `cost_matrix_weights_state_diag` (array, **required**): Cost matrix weights regarding the state (diagonal elements)
- `cost_matrix_weights_input_diag` (array, **required**): Cost matrix weights regarding the input (diagonal elements)

#### Tricycle (`tric`)
- `steering_wheel_frame_id` (string, **required**): Frame ID for the steering wheel.
- `rob_dist_between_steering_back_wh` (double, **required**): Distance between the steering wheel and rear wheels (m)
- `rob_wh_vel_time_const` (double, **required**): Robot wheel velocity time constant (s)
- `rob_steer_wh_angle_time_const` (double, **required**): Robot steering wheel angle time constant (s)
- `rob_wh_max_vel` (double, **required**): Robot wheel maximum linear velocity (m/s)
- `rob_wh_max_ace` (double, **required**): Robot wheel maximum linear acceleration (m/s^2)
- `rob_steer_wh_min_angle` (double, **required**): Robot steering wheel minimum angle (deg)
- `rob_steer_wh_max_angle` (double, **required**): Robot steering wheel maximum angle (deg)
- `rob_steer_wh_max_angle_var` (double, **required**): Robot steering wheel maximum angle variation (deg/s)
- `cost_matrix_weights_state_diag` (array, **required**): Cost matrix weights regarding the state (diagonal elements)
- `cost_matrix_weights_input_diag` (array, **required**): Cost matrix weights regarding the input (diagonal elements)

## Subscribes Topics

- `pose_goal` (`geometry_msgs::PoseStamped`): Target pose for the robot to reach.
- `path_no_stack_up` (`itrci_nav::ParametricPathSet`): Path for the robot to follow (v1)
- `path_no_stack_up_2` (`itrci_nav::ParametricPathSet2`): Path for the robot to follow (v2)
- `control_command` (`std_msgs::String`): Command to control robot behavior (`break`, `idle`)

## Publishes

- `cmd_vel` (`geometry_msgs::Twist`): Velocity commands generated by the controller
- `control_status` (`itrci_nav::parametric_trajectories_control_status`): Status of the controller, including the robot's behavior and path progress
- `actual_path` (`itrci_nav::ParametricPathSet`): The robot’s current path
- `debug_discretized_path` (`nav_msgs::Path`): Discretized path for debugging purposes when following a parametric trajectory

### Services

None.

### Actions

None.

## Usage

### Compilation

**ROS 1**

```sh
# ROS 1 environment setup
source /opt/ros/noetic/setup.bash

# Create workspace
mkdir -p ~/catkin_ws/src

# Clone the repository
cd ~/catkin_ws/src
git clone https://github.com/JorgeDFR/nmpc_nav_control

# Generate acados solvers libraries
cd ~/catkin_ws/src/nmpc_nav_control
python3 scripts/generate_acados_libs.py config/nmpc_nav_control_acados_models.yaml

# Build
cd ~/catkin_ws
catkin build nmpc_nav_control
source devel/setup.bash
```

### Launch

**ROS 1**

```sh
# Launch main node
roslaunch nmpc_nav_control run_nmpc_nav_control.launch

# Optional: Generate updated acados solvers libraries via ROS
roslaunch nmpc_nav_control run_nmpc_nav_control_generate_libs.launch
```

## Acknowledges

- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)

## Contacts

For any questions please contact me at jorge.d.ribeiro@inesctec.pt.