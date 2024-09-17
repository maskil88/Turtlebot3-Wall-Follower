#ifndef MOVEMENT_CONTROLLER_HPP_
#define MOVEMENT_CONTROLLER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class MovementController
{
    public:
        //---Constructor & Destructor-----------------------------------------------------
        MovementController(rclcpp::Node::SharedPtr node);
        ~MovementController() = default;

        // Update the robot's velocity
        void updateCmdVel(double linear, double angular);

    private:
        
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

#endif