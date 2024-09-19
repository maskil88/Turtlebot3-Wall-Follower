#include "turtlebot3_gazebo/PathViz.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

PathViz::PathViz(rclcpp::Node* node)
    : node_(node)
{
    //Initialise relevant variables
    node_ = node;

    // Initialise path publisher
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("path", qos);

    // Initialise odometry subscriber
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        qos,
        std::bind(&PathViz::odom_callback, this, std::placeholders::_1));
    path_msg_.header.frame_id = "odom";
}
//---
PathViz::~PathViz() {}
//---
void PathViz::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Create a message with current odometry data
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = msg->header.stamp;
    pose_stamped.header.frame_id = "odom";

    // Set position and orientation of the robot
    pose_stamped.pose.position.x = msg->pose.pose.position.x;
    pose_stamped.pose.position.y = msg->pose.pose.position.y;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation = msg->pose.pose.orientation;

    // Append the new pose to the path message
    path_msg_.poses.push_back(pose_stamped);

    // Publish the path message
    path_pub_->publish(path_msg_);
}
