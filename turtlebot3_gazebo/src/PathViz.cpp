#include "turtlebot3_gazebo/PathViz.hpp"

PathViz::PathViz(rclcpp::Node::SharedPtr node)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    path_pub_ = node->create_publisher<nav_msgs::msg::Path>("path", qos);
}

void PathViz::updatePath(const geometry_msgs::msg::PoseStamped& pose_stamped)
{
    path_msg_.poses.push_back(pose_stamped);
    path_pub_->publish(path_msg_);
}
