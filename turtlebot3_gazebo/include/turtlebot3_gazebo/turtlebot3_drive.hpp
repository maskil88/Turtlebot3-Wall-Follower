// #ifndef TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
// #define TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_

// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <nav_msgs/msg/path.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Quaternion.h>

// // #include "LidarSensor.hpp"
// // #include "MovementController.hpp"
// // #include "PathViz.hpp"
// // #include "WallFollower.hpp"

// #define DEG2RAD (M_PI / 180.0)
// #define RAD2DEG (180.0 / M_PI)

// // #define NUM_LIDAR_ANGLES 7

// // LIDAR indices for each angle
// #define FRONT 0                     // Forward (0°)
// #define LEFT_FRONT 1                // Left-Front (30°)
// #define LEFT_FRONT_INTERMEDIATE 2   // Left-Front Intermediate (60°)
// #define LEFT 3                      // Left (90°)
// #define FAR_LEFT 4                  // Far-Left (120°)
// #define RIGHT 5                     // Right (270°)
// #define RIGHT_FRONT 6               // Right-Front (330°)

// // State definitions for the robot's behaviour
// #define GET_TB3_DIRECTION 0         // Default state
// #define TB3_DRIVE_FORWARD 1         // Forward state
// #define TB3_RIGHT_TURN    2         // Turn-right state
// #define TB3_LEFT_TURN     3         // Turn-left state
// #define TB3_PAUSE_BEFORE_ALIGN 4    // Pause state
// #define TB3_ALIGN_WITH_WALL 5       // Alignment state


// class Turtlebot3Drive : public rclcpp::Node
// {
//   public:

//     //---Constructor & Destructor---------------------------------------------------------------
//     Turtlebot3Drive();
//     ~Turtlebot3Drive();

//   private:

//     // ROS topic publishers
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
//     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;  // Path publisher for RViz visualization

//     // ROS topic subscribers
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

//     // Variables
//     double robot_pose_;
//     double prev_robot_pose_;

//     // LIDAR angle data (0-6)
//     double scan_data_[NUM_LIDAR_ANGLES];

//     // Path message for RViz visualization
//     nav_msgs::msg::Path path_msg_;

//     // ROS timer
//     rclcpp::TimerBase::SharedPtr update_timer_;
//     rclcpp::TimerBase::SharedPtr pause_timer_;    // Timer for pause before alignment

//     // Main control loop for wall-following
//     void update_callback();

//     // Update the robot's velocity to the given input
//     void update_cmd_vel(double linear, double angular);

//     // Handles laser scan data
//     void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
//     // Handles odometry data and path-finding
//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

//     // Callback function for the pause timer
//     void pause_timer_callback();
// };

// #endif 