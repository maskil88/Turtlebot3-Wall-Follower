#include "turtlebot3_gazebo/MovementController.hpp"

MovementController::MovementController(rclcpp::Node::SharedPtr node)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
}

void MovementController::updateCmdVel(double linear, double angular)
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;
    cmd_vel_pub_->publish(cmd_vel);
}
