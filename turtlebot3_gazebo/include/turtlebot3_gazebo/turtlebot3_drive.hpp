#ifndef TURTLEBOT3_DRIVE_HPP_
#define TURTLEBOT3_DRIVE_HPP_

//---Includes---------------------------------------------------------------
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "LidarSensor.hpp"
#include "MovementController.hpp"
#include "CameraProcessor.hpp"
#include "PathViz.hpp"
#include "WallFollower.hpp"

//---Turtlebot3Drive Interface----------------------------------------------
class Turtlebot3Drive : public rclcpp::Node
{
    public:

        //---Constructor & Destructor---------------------------------------
        Turtlebot3Drive();
        ~Turtlebot3Drive();

    private:

        // Main loop for updating TurtleBot actions
        void update_callback();

        std::unique_ptr<LidarSensor> lidar_sensor_;
        std::unique_ptr<MovementController> movement_controller_;
        std::unique_ptr<CameraProcessor> camera_processor_;
        std::unique_ptr<PathViz> path_viz_;
        std::unique_ptr<WallFollower> wall_follower_;
        rclcpp::TimerBase::SharedPtr update_timer_;
};

#endif 