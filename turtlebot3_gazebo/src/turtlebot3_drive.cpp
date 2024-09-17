//---Includes--------------------------------------------------------------------------------------------
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlebot3_gazebo/LidarSensor.hpp"
#include "turtlebot3_gazebo/MovementController.hpp"
#include "turtlebot3_gazebo/PathViz.hpp"
#include "turtlebot3_gazebo/WallFollower.hpp"

using namespace std::chrono_literals;

//---Turtlebot3Drive Implementation---------------------------------------------------------------------
class Turtlebot3Drive : public rclcpp::Node
{
  public:
      //---Constructor----------------------------------------------------------------------------------
      Turtlebot3Drive()
      : Node("turtlebot3_drive_node"),
        lidar_sensor_(),
        movement_controller_(this->shared_from_this()),
        path_viz_(this->shared_from_this()),
        wall_follower_(lidar_sensor_, movement_controller_)
      {
          // Initialise LIDAR subscriber
          scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
              "scan", rclcpp::SensorDataQoS(),
              std::bind(&Turtlebot3Drive::scan_callback, this, std::placeholders::_1));

          // Initialise odometry subscriber
          odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
              "odom", rclcpp::QoS(rclcpp::KeepLast(10)),
              std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

          // ROS timer
          update_timer_ = this->create_wall_timer(
              10ms, std::bind(&Turtlebot3Drive::update_callback, this));

          RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");
      }

  private:

      // Turtlebot variable objects
      LidarSensor lidar_sensor_;
      MovementController movement_controller_;
      PathViz path_viz_;
      WallFollower wall_follower_;

      // Subscriber variables
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
      rclcpp::TimerBase::SharedPtr update_timer_;

      // Handles LIDAR scan data
      void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { lidar_sensor_.processScan(msg); }

      // Handles odometry data and path-finding
      void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
      {
          geometry_msgs::msg::PoseStamped pose_stamped;
          pose_stamped.header.stamp = msg->header.stamp;
          pose_stamped.header.frame_id = "odom";
          pose_stamped.pose.position = msg->pose.pose.position;
          pose_stamped.pose.orientation = msg->pose.pose.orientation;

          path_viz_.updatePath(pose_stamped);
      }

      // Main control loop for wall-following
      void update_callback() { wall_follower_.update(); }
};
//---
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Turtlebot3Drive>());
    rclcpp::shutdown();

    return 0;
}
