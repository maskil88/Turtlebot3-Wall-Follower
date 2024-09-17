#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <memory>

using namespace std::chrono_literals;

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
  // Initialise LIDAR angle array
  for( int i = 0; i < NUM_LIDAR_ANGLES; i++){ scan_data_[i] = 0.0; }

  // Initialise position variables
  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  // Initialise publishers
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Path publisher to visualize the robot's path
  // path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", qos);


  // Initialise LIDAR subscriber
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Turtlebot3Drive::scan_callback, \
      this, \
      std::placeholders::_1));
  
  // Initialise odometer subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  // Initialise ROS timers
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlebot3Drive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");
}
//---
Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}
//---
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;

  // Create a message with current odometry data
  // geometry_msgs::msg::PoseStamped pose_stamped;
  // pose_stamped.header.stamp = msg->header.stamp;
  // pose_stamped.header.frame_id = "odom";  // Frame in which the path is published

  // // Set the position and orientation of the turtlebot
  // pose_stamped.pose.position.x = msg->pose.pose.position.x;
  // pose_stamped.pose.position.y = msg->pose.pose.position.y;
  // pose_stamped.pose.position.z = 0.0;
  // pose_stamped.pose.orientation = msg->pose.pose.orientation;  // Use the same orientation as odometry

  // // Append the new pose to the path message
  // path_msg_.poses.push_back(pose_stamped);

  // // Publish the path message
  // path_pub_->publish(path_msg_);
}
//---
void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // LIDAR angles: 0, 30, 60, 90, 120, 270, 330
  uint16_t scan_angle[7] = {0, 37, 60, 90, 120, 270, 330};

  for (int num = 0; num < NUM_LIDAR_ANGLES; num++)
  {
    if(std::isinf(msg->ranges.at(scan_angle[num]))) { scan_data_[num] = msg->range_max; } 
    else { scan_data_[num] = msg->ranges.at(scan_angle[num]); }
  }
}
//---
void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}
//---
void Turtlebot3Drive::update_callback()
{
  // Left wall-following parameters
  double target_distance = 0.35;         // Desired distance from the left wall
  double distance_tolerance = 0.05;      // Allowable deviation from the target distance
  double min_turn_distance = 0.1;        // Minimum distance before making a turn
  double check_forward_dist = 0.4;       // Distance threshold for obstacles in front
  double linear_speed = 0.16;            // Speed to move forward
  double angular_speed = 0.45;           // Speed to turn left or right
  double proportional_gain = 1.5;        // Proportional control gain for smoother wall following

  // CASE 1: Wall detected in front of the turtlebot If there is an obstacle in front (wall detected in front)
  if (scan_data_[FRONT] < check_forward_dist)
  {
    // Slow down and turn right to avoid the wall
    update_cmd_vel(0.0, -angular_speed);
  }

  // CASE 2: Left wall detected within the acceptable range
  else if ( scan_data_[LEFT_FRONT] > (target_distance - distance_tolerance) && 
            scan_data_[LEFT_FRONT] < (target_distance + distance_tolerance) )
  {
    // Move forward
    update_cmd_vel(linear_speed, 0.0);
  }

  // CASE 3: Robot is too far from the left wall (turn left smoothly)
  else if ( scan_data_[LEFT_FRONT] > (target_distance + distance_tolerance) )
  {
    // Proportional control to adjust turning angle
    double error = scan_data_[LEFT_FRONT] - target_distance;
    double proportional_turn = proportional_gain * error;
    update_cmd_vel(linear_speed, std::min(proportional_turn, angular_speed));  // Turn left smoothly
  }

  // CASE 4: Robot is too close to the left wall (turn right slightly)
  else if ( scan_data_[LEFT_FRONT] < (target_distance - distance_tolerance) )
  {
    // Proportional control to adjust turning angle
    double error = target_distance - scan_data_[LEFT_FRONT];
    double proportional_turn = proportional_gain * error;
    update_cmd_vel(linear_speed, -std::min(proportional_turn, angular_speed));  // Turn right smoothly
  }

  // CASE 5: No wall detected on the left (Turn left to find the wall)
  else if ( std::isinf(scan_data_[LEFT_FRONT]) )
  {
    // Slow left turn in place
    update_cmd_vel( 0.0, 1.3 * angular_speed );
  }
}

// Main function
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}
