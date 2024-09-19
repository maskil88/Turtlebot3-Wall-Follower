#ifndef MOVEMENT_CONTROLLER_HPP_
#define MOVEMENT_CONTROLLER_HPP_

//---Includes------------------------------------------------------------------------------
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

//---MovementController Interface----------------------------------------------------------
class MovementController
{
    public:

        //---Constructor & Destructor------------------------------------------------------
        MovementController(rclcpp::Node* node);
        ~MovementController();

        // Updates the command velocity specified by inputs
        void update_cmd_vel(double linear, double angular);

    private:

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;   // Twist subscriber variable
        rclcpp::Node* node_;

};

#endif