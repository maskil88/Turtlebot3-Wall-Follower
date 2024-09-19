#ifndef PATH_VIZ_HPP_
#define PATH_VIZ_HPP_

//---Includes-------------------------------------------------------------------------
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

//---PathViz Interface----------------------------------------------------------------
class PathViz
{
    public:

        //---Constructor & Destructor-------------------------------------------------
        PathViz(rclcpp::Node* node);
        ~PathViz();

    private:

        // Handles odometry data
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;            // Path publishing variable
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;     // Odometry subscriber variable
        nav_msgs::msg::Path path_msg_;                                          // Path message variable
        rclcpp::Node* node_;
};

#endif
