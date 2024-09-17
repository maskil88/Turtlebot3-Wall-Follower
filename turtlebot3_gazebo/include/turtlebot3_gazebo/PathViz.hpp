#ifndef PATH_VIZ_HPP_
#define PATH_VIZ_HPP_

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class PathViz
{
public:
    PathViz(rclcpp::Node::SharedPtr node);
    ~PathViz() = default;

    void updatePath(const geometry_msgs::msg::PoseStamped& pose_stamped);

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
};

#endif
