#include "turtlebot3_gazebo/MovementController.hpp"

MovementController::MovementController(rclcpp::Node* node)
    : node_(node)
{
    node_ = node;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
}
//---
MovementController::~MovementController() {}
//---
void MovementController::update_cmd_vel(double linear, double angular)
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    // Publish new command velocity
    cmd_vel_pub_->publish(cmd_vel);
}
