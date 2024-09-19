#ifndef WALL_FOLLOWER_HPP_
#define WALL_FOLLOWER_HPP_

//---Includes-------------------------------------------------------------------------
#include <rclcpp/rclcpp.hpp>
#include "LidarSensor.hpp"
#include "MovementController.hpp"
#include "CameraProcessor.hpp"

//---WallFollower Interface-----------------------------------------------------------
class WallFollower
{
    public:

        //---Constructor & Destructor--------------------------------------------------
        WallFollower(rclcpp::Node* node, 
                    LidarSensor* lidar_sensor,
                    MovementController* movement_controller,
                    CameraProcessor* camera_processor);
        ~WallFollower();

        // Updates the wall-following algorithm based on the TurtleBot's current position and surroundings
        void update();

    private:

        LidarSensor* lidar_sensor_;
        MovementController* movement_controller_;
        CameraProcessor* camera_processor_;
        rclcpp::Node* node_;
};

#endif