# Walker Package

A ROS 2 package implementing a simple robot controller that uses laser scan data to navigate while avoiding obstacles. The robot moves forward until it detects obstacles, then rotates to find a clear path.

## Overview

The package implements a state machine architecture with two main states:
- Moving State: Robot moves forward when path is clear
- Rotating State: Robot rotates when obstacles are detected

The controller uses laser scan data to detect obstacles in five regions:
- Front
- Front-Left
- Front-Right
- Left
- Right

## Dependencies

- ROS 2 Humble
- C++17 or later
- sensor_msgs
- geometry_msgs
- rclcpp

## Building the Package

1. Create a ROS 2 workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this package into the src directory

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select walker
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Running the Simulation

1. First, launch Gazebo with your preferred world (ensure Gazebo and your robot model are properly set up)

2. In a new terminal, run the walker node using the launch file:
```bash
ros2 launch walker walker_launch.py
```

## Recording Bag Files

The launch file includes functionality to record bag files of the simulation.

1. To run the simulation with bag file recording enabled:
```bash
ros2 launch walker walker_launch.py record_data:=true
```

This will create a bag file in your current directory with the naming format: `walker_data_YYYY_MM_DD_HH_MM_SS`

2. To disable bag file recording (default behavior):
```bash
ros2 launch walker walker_launch.py record_data:=false
```

## Working with Bag Files

### Inspecting a Bag File

To view information about the recorded bag file:
```bash
ros2 bag info walker_data_<timestamp>
```

### Playing Back a Bag File

1. First, ensure Gazebo is **not** running, as it would conflict with the playback

2. Play the bag file:
```bash
ros2 bag play walker_data_<timestamp>
```

3. In another terminal, you can visualize the robot's behavior using RViz:
```bash
ros2 run rviz2 rviz2
```

## Package Structure
```
walker/
├── include/
│   ├── robot_control.hpp
│   └── robot_states.hpp
├── src/
│   ├── robot_control.cpp
│   └── robot_states.cpp
├── launch/
│   └── walker_launch.py
├── CMakeLists.txt
└── package.xml
```

## Parameters

- ROTATION_SPEED: 0.3 rad/s
- FORWARD_SPEED: 0.6 m/s
- DISTANCE_THRESHOLD: 0.7 m
- ROTATION_DURATION: 10.0 seconds

## Topics

### Subscribed Topics
- `/scan` (sensor_msgs/msg/LaserScan): Laser scan data for obstacle detection

### Published Topics
- `/cmd_vel` (geometry_msgs/msg/Twist): Robot velocity commands

## License

TODO: Add license information

## Maintainer

- Navdeep Singh (singh.nav2100@gmail.com)