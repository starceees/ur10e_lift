# UR10e Lift Simulation and Trajectory Control

This repository contains a set of ROS 2 packages for simulating a UR10e robotic arm mounted on a vertical lift mechanism. The packages provide the robot description, Gazebo launch files and several trajectory planners for commanding the robot in simulation or visualisation in RViz.

## Repository Layout

- **`ur10e_lift_description`** – Xacro/URDF model of the lift and robot, launch files for Ignition Gazebo and RViz and helper Python nodes.  It installs the robot model and configuration for `ros2_control`.
- **`ur10e_lift_trajectory_control`** – C++ nodes that execute trajectories based on waypoints.  Includes a basic joint trajectory planner, an inverse kinematics (IK) planner that follows Cartesian waypoints and a spline-based planner.  Launch files start these nodes together with `robot_state_publisher`.
- **`ur10e_lift_control`** – Empty Python package skeleton used for lint tests.
- **`ur_description`** – Submodule containing the Universal Robots description packages.  Initialise this submodule or ensure the `ur_description` package is available in your workspace.

## Building

1. Create and source a ROS 2 workspace (e.g. for Humble):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone --recurse-submodules <repository_url>
   cd ..
   source /opt/ros/humble/setup.bash
   colcon build
   ```
2. Remember to update submodules with `git submodule update --init --recursive` if the `ur_description` folder is empty.

## Usage

### Gazebo Simulation

Launch Ignition Gazebo with the empty world defined in this repository:
```bash
ros2 launch ur10e_lift_description gz_sim.launch.py
```
To spawn the robot and open RViz:
```bash
ros2 launch ur10e_lift_description spawn.launch.py
```
This launch file loads the URDF with `ros_gz_sim` and starts RViz configured with `ur10e_lift.rviz`.

### RViz Only

The robot model can also be viewed in RViz without Gazebo using:
```bash
ros2 launch ur10e_lift_description rviz_only.launch.py
```

### Trajectory Planners

`ur10e_lift_trajectory_control` provides multiple planners.  Example launch files are located in the `launch` directory.

- **Joint trajectory planner** – `trajectory_planner` node reads joint-space waypoints from a text file.  The node initialises publishers and subscriptions as shown below:
  ```cpp
  joint_names_ = {"lift_joint", "ur10e_shoulder_pan_joint", "ur10e_shoulder_lift_joint",
                  "ur10e_elbow_joint", "ur10e_wrist_1_joint", "ur10e_wrist_2_joint",
                  "ur10e_wrist_3_joint"};
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/planned_trajectory", 10);
  waypoint_file_sub_ = this->create_subscription<std_msgs::msg::String>("/load_waypoint_file", 10,
      std::bind(&TrajectoryPlanner::waypoint_file_callback, this, std::placeholders::_1));
  ```
- **IK planner** – `ik_trajectory_planner` reads Cartesian waypoints and generates joint commands using a simple analytic IK.  Waypoints and the current path are visualised with markers.
- **Spline planner** – `spline_trajectory_planner` creates cubic splines between Cartesian waypoints and solves IK along the path.

Launch one of the planners with:
```bash
ros2 launch ur10e_lift_trajectory_control trajectory_control.launch.py        # joint space
ros2 launch ur10e_lift_trajectory_control ik_trajectory_control.launch.py     # Cartesian IK
ros2 launch ur10e_lift_trajectory_control spline_trajectory_control.launch.py # cubic splines
```
Each planner subscribes to a waypoint file topic to start executing a path.  Example waypoint files are provided under `ur10e_lift_trajectory_control/waypoints/`.

### Example Robot Controller

For testing the robot without trajectories, `robot_controller.py` publishes joint states at 100 Hz and runs a small demo sequence:
```python
self.timer = self.create_timer(0.01, self.publish_joint_states)  # 100Hz
self.demo_timer = self.create_timer(4.0, self.demo_sequence_step)
```
It gradually lifts the platform, rotates joints and then returns to the home position while logging the commands.

### Data Visualisation

The script `plot_trajectory_data.py` plots CSV logs produced by the planners.  It displays joint states, end‑effector position and 3D path information.

## World and Robot Description

The lift mechanism is defined in `ur10e_lift.urdf.xacro`.  The platform is attached via a prismatic `lift_joint` which allows vertical motion in simulation:
```xml
  <joint name="lift_joint" type="prismatic">
    <parent link="lift_base"/>
    <child link="lift_platform"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0.0" upper="${lift_stroke}" effort="1000" velocity="0.3"/>
    <dynamics damping="5.0"/>
  </joint>
```
This joint is interfaced to Gazebo via a `ros2_control` plugin so that controllers can command its position.

## License

The packages are released under the MIT license; see the individual `LICENSE` files for details.
