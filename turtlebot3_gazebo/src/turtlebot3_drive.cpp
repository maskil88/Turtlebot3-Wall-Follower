#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

//---Includes-----------------------------------------------------------------------------
#include <memory>
using namespace std::chrono_literals;

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
    // Initialise class objects
    lidar_sensor_ = std::make_unique<LidarSensor>(this);
    movement_controller_ = std::make_unique<MovementController>(this);
    camera_processor_ = std::make_unique<CameraProcessor>(this);
    path_viz_ = std::make_unique<PathViz>(this);
    wall_follower_ = std::make_unique<WallFollower>(
        this, 
        lidar_sensor_.get(), 
        movement_controller_.get(), 
        camera_processor_.get()
    );

    // Initialise timer
    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Turtlebot3Drive::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialized");
}
//---
Turtlebot3Drive::~Turtlebot3Drive()
{
    RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}
//---
void Turtlebot3Drive::update_callback()
{
    wall_follower_->update();
}
//---
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Turtlebot3Drive>());
    rclcpp::shutdown();

    return 0;
}