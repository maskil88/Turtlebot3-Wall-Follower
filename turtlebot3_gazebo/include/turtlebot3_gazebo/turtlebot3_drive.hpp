#ifndef TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
#define TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>  // Added for Path visualization
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

// Define LIDAR indices for each angle
#define FRONT 0                     // Forward (0°)
#define LEFT_FRONT 1                // Left-Front (30°)
#define LEFT_FRONT_INTERMEDIATE 2   // Left-Front Intermediate (60°)
#define LEFT 3                      // Left (90°)
#define FAR_LEFT 4                  // Far-Left (120°)
#define RIGHT 5                     // Right (270°)
#define RIGHT_FRONT 6               // Right-Front (330°)

// Define the states for the robot's behavior
#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3
#define TB3_PAUSE_BEFORE_ALIGN 4   // Pause state
#define TB3_ALIGN_WITH_WALL    5   // Alignment state

class Turtlebot3Drive : public rclcpp::Node
{
public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;  // Path publisher for RViz visualization

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Variables
  double robot_pose_;
  double prev_robot_pose_;

  // Path message for RViz visualization
  nav_msgs::msg::Path path_msg_;

  // LIDAR data for 7 angles (0°, 30°, 60°, 90°, 120°, 270°, 330°)
  double scan_data_[7];  // C-style array for storing LIDAR data

  uint8_t turtlebot3_state_num;  // Current state of the robot

  // ROS timers
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr pause_timer_;  // Timer for pause before alignment
  
  // Function prototypes
  void update_callback();  // Main control loop for wall-following
  void update_cmd_vel(double linear, double angular);  // Send velocity commands to the robot
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);  // Handle laser scan data
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);  // Handle odometry data and path
  void pause_timer_callback();  // Callback function for the pause timer

  // Camera subscription and detection flag
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  bool green_block_detected_;

  // Function prototypes
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  bool detect_green_block(const cv::Mat& image);
};

#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_