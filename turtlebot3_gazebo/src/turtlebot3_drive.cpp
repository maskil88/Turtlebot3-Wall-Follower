#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <memory>

using namespace std::chrono_literals;

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;
  scan_data_[3] = 0.0;
  scan_data_[4] = 0.0;
  scan_data_[5] = 0.0;
  scan_data_[6] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
  
  // Path publisher to visualize the robot's path
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Turtlebot3Drive::scan_callback, \
      this, \
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlebot3Drive::update_callback, this));

  // Path message setup
  path_msg_.header.frame_id = "odom";  // Path frame (should match the odometry frame)

  green_block_detected_ = false;

  // Initialize image subscriber
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&Turtlebot3Drive::image_callback, this, std::placeholders::_1));


  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
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

  // Create a PoseStamped message with current odometry data
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = msg->header.stamp;
  pose_stamped.header.frame_id = "odom";  // Frame in which the path is published

  // Set the position and orientation of the robot
  pose_stamped.pose.position.x = msg->pose.pose.position.x;
  pose_stamped.pose.position.y = msg->pose.pose.position.y;
  pose_stamped.pose.position.z = 0.0;  // Assuming the robot is on a 2D plane
  pose_stamped.pose.orientation = msg->pose.pose.orientation;  // Use the same orientation as odometry

  // Append the new pose to the path message
  path_msg_.poses.push_back(pose_stamped);

  // Publish the path message
  path_pub_->publish(path_msg_);
}

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Updated LIDAR angles: 0 (Forward), 30 (Left-Front), 60 (Left-Front Intermediate), 90 (Left), 120 (Far-Left), 270 (Right), 330 (Right-Front)
  uint16_t scan_angle[7] = {0, 37, 60, 90, 120, 270, 330};  // Adjust for 7 angles

  for (int num = 0; num < 7; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = msg->range_max;
    } else {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

void Turtlebot3Drive::pause_timer_callback()
{
  // After the pause, switch to the alignment state
  turtlebot3_state_num = TB3_ALIGN_WITH_WALL;

  // Stop the timer once the transition is done
  pause_timer_->cancel();
}

/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
  // Parameters for left-wall-following
  double target_distance = 0.35;           // Desired distance from the left wall (meters)
  double distance_tolerance = 0.05;       // Allowable deviation from the target distance
  double min_turn_distance = 0.1;         // Minimum distance before making a turn
  double check_forward_dist = 0.4;        // Distance threshold for obstacles in front
  double linear_speed = 0.16;             // Speed to move forward
  double angular_speed = 0.45;             // Speed to turn left or right
  double proportional_gain = 1.5;         // Proportional control gain for smoother wall following

  if (green_block_detected_) {
    // Stop the robot
    update_cmd_vel(0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Green block detected. Stopping the robot.");
    // Optionally, cancel the timer to stop further updates
    update_timer_->cancel();
    return;
  }

  // Case 1: If there is an obstacle in front (wall detected in front)
  if (scan_data_[FRONT] < check_forward_dist) {
    // Slow down and turn right in place to avoid the wall in front
    update_cmd_vel(0.0, -angular_speed);  // Turn right slowly
  }
  // Case 2: If the robot detects a left wall within the acceptable range
  else if (scan_data_[LEFT_FRONT] > (target_distance - distance_tolerance) && scan_data_[LEFT_FRONT] < (target_distance + distance_tolerance)) {
    // Move forward smoothly
    update_cmd_vel(linear_speed, 0.0);
  }
  // Case 3: The robot is too far from the left wall (Turn left slightly)
  else if (scan_data_[LEFT_FRONT] > (target_distance + distance_tolerance)) {
    // Proportional control to adjust turning angle
    double error = scan_data_[LEFT_FRONT] - target_distance;
    double proportional_turn = proportional_gain * error;
    update_cmd_vel(linear_speed, std::min(proportional_turn, angular_speed));  // Turn left smoothly
  }
  // Case 4: The robot is too close to the left wall (Turn right slightly)
  else if (scan_data_[LEFT_FRONT] < (target_distance - distance_tolerance)) {
    // Proportional control to adjust turning angle
    double error = target_distance - scan_data_[LEFT_FRONT];
    double proportional_turn = proportional_gain * error;
    update_cmd_vel(linear_speed, -std::min(proportional_turn, angular_speed));  // Turn right smoothly
  }
  // Case 5: No wall detected on the left (Turn left in place to find the wall)
  else if (std::isinf(scan_data_[LEFT_FRONT])) {
    // Slow left turn in place
    update_cmd_vel(0.0, 1.3*angular_speed);
  }


}

void Turtlebot3Drive::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    green_block_detected_ = detect_green_block(cv_ptr->image);
  }
  catch(cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

bool Turtlebot3Drive::detect_green_block(const cv::Mat& image)
{
  int green_count = 0;
  for(int i = 0; i < image.rows; i++)
  {
    for(int j = 0; j < image.cols; j++)
    {
      int B = image.at<cv::Vec3b>(i, j)[0];
      int G = image.at<cv::Vec3b>(i, j)[1];
      int R = image.at<cv::Vec3b>(i, j)[2];

      // Detect green pixels where G is significantly larger than R and B
      if(G > 3 * R && G > 3 * B)
      {
        green_count++;
      }
    }
  }

  // Threshold to decide if a green block is detected
  RCLCPP_INFO(this->get_logger(), "Green pixel count: %d", green_count);
  return green_count > 50000; // Adjust threshold as needed
}


/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}