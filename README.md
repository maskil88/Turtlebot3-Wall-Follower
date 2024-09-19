# TurtleBot3 Left Wall-Following Robot with Camera and LIDAR

## Project Description
This project implements a left-wall-following robot using the TurtleBot3 platform. The robot is equipped with a LIDAR sensor for obstacle avoidance, a camera for detecting green blocks, and a path visualization feature. The robot autonomously navigates through both enclosed and open mazes by maintaining a set distance from the left wall, turning appropriately, and detecting the end of the maze using a green block.

## Features
- **Left Wall-Following**: The robot uses LIDAR data to maintain a specific distance from the left wall.
- **Obstacle Avoidance**: Detects obstacles in front using LIDAR and makes decisions to avoid collisions.
- **Camera-Based Maze Exit Detection**: The robot uses a camera to detect green blocks as an indication of the end of the maze.
- **Path Visualization**: Visualizes the robot's path based on odometry data for easier debugging.
  
## Files that have been modified to implement wall following capability

- `turtlebot3_gazebo/src/`
  - `turtlebot3_drive.cpp`: Main node controlling the robot's behavior.
  - `WallFollower.cpp`: Implements the left wall-following logic using LIDAR data.
  - `LidarSensor.cpp`: Handles LIDAR sensor data to detect obstacles and walls.
  - `MovementController.cpp`: Publishes velocity commands to the robot's wheels.
  - `CameraProcessor.cpp`: Processes camera feed to detect green blocks.
  - `PathViz.cpp`: Publishes the robot's path for visualization.
- `turtlebot3_gazebo/include/turtlebot3_gazebo/`
  - `turtlebot3_drive.hpp`: Header file for the `Turtlebot3Drive` node.
  - `WallFollower.hpp`: Header file for wall-following logic.
  - `LidarSensor.hpp`: Header file for handling LIDAR data.
  - `MovementController.hpp`: Header file for controlling robot movement.
  - `CameraProcessor.hpp`: Header file for processing camera images.
  - `PathViz.hpp`: Header file for visualizing the robot's path.
- `turtlebot3_gazebo/models/turtlebot3_burger/`
  - `model.sdf`: Camera configuration on turtlebot burger.
- `turtlebot3_gazebo/`
  - `CMakeLists.txt`
- `turtlebot3_gazebo/worlds`
  - `empty_world.world`: Maps drawn into empty world.
  
## Dependencies
- **ROS 2 Humble**: The project is built using ROS 2. Install ROS 2 Humble from the official [ROS website](https://docs.ros.org/en/humble/Installation.html).
- **OpenCV**: The project requires OpenCV for image processing.
- **TurtleBot3**: Ensure TurtleBot3 simulation packages are installed.
- **LIDAR and Camera**: The robot uses LIDAR and camera for sensor input.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/turtlebot3-wall-following.git
   cd turtlebot3-wall-following

2. follow the links below to install ROS 2 Humble, OpenCV,
3. Build the workspace:
   ```bash
   colcon build
6. Source the workspace:
   ```bash
   source install/setup.bash


## Usage Instructions

1. Launch the TurtleBot3 simulation with the maze:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
2. Run the robot:
   ```bash
   ros2 run turtlebot3_gazebo turtlebot3_drive

4. Visualize path:
   ```bash
   ros2 run rviz2 rviz2

5. Visualize camera feed (optional):
   ```bash
   ros2 run image_view image_view image:=/camera/image_raw
   
## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


# TurtleBot3
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/turtlebot3/logo_turtlebot3.png" width="300">

[![kinetic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/kinetic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/kinetic-devel)
[![melodic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/melodic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/melodic-devel)
[![noetic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/noetic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/noetic-devel)

[![dashing-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/dashing-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/dashing-devel)
[![foxy-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/foxy-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/foxy-devel)
[![galactic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/galactic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/galactic-devel)
[![humble-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/humble-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble-devel)

## ROBOTIS e-Manual for TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)

## Wiki for turtlebot3_simulations Packages
- http://wiki.ros.org/turtlebot3_simulations (metapackage)
- http://wiki.ros.org/turtlebot3_fake
- http://wiki.ros.org/turtlebot3_gazebo

## Open Source related to TurtleBot3
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [turtlebot3_applications_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs)
- [turtlebot3_applications](https://github.com/ROBOTIS-GIT/turtlebot3_applications)
- [turtlebot3_autorace](https://github.com/ROBOTIS-GIT/turtlebot3_autorace)
- [turtlebot3_deliver](https://github.com/ROBOTIS-GIT/turtlebot3_deliver)
- [hls_lfcd_lds_driver](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver)
- [ld08_driver](https://github.com/ROBOTIS-GIT/ld08_driver)
- [open_manipulator_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_msgs)
- [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [open_manipulator_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_simulations)
- [open_manipulator_perceptions](https://github.com/ROBOTIS-GIT/open_manipulator_perceptions)
- [open_manipulator_with_tb3_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_msgs)
- [open_manipulator_with_tb3](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3)
- [open_manipulator_with_tb3_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_simulations)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [OpenCR-Hardware](https://github.com/ROBOTIS-GIT/OpenCR-Hardware)
- [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR)

## Documents and Videos related to TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)
- [ROBOTIS e-Manual for OpenManipulator](http://emanual.robotis.com/docs/en/platform/openmanipulator/)
- [ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [Website for TurtleBot Series](http://www.turtlebot.com/)
- [e-Book for TurtleBot3](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51/)
- [Videos for TurtleBot3 ](https://www.youtube.com/playlist?list=PLRG6WP3c31_XI3wlvHlx2Mp8BYqgqDURU)
