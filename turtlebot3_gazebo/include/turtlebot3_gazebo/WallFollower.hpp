#ifndef WALL_FOLLOWER_HPP_
#define WALL_FOLLOWER_HPP_

#include "LidarSensor.hpp"
#include "MovementController.hpp"

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

// LIDAR indices for each angle
#define FRONT 0                     // Forward (0°)
#define LEFT_FRONT 1                // Left-Front (30°)
#define LEFT_FRONT_INTERMEDIATE 2   // Left-Front Intermediate (60°)
#define LEFT 3                      // Left (90°)
#define FAR_LEFT 4                  // Far-Left (120°)
#define RIGHT 5                     // Right (270°)
#define RIGHT_FRONT 6               // Right-Front (330°)

// State definitions for the robot's behaviour
#define GET_TB3_DIRECTION 0         // Default state
#define TB3_DRIVE_FORWARD 1         // Forward state
#define TB3_RIGHT_TURN    2         // Turn-right state
#define TB3_LEFT_TURN     3         // Turn-left state
#define TB3_PAUSE_BEFORE_ALIGN 4    // Pause state
#define TB3_ALIGN_WITH_WALL 5       // Alignment state

class WallFollower
{
public:
    WallFollower(LidarSensor& lidar, MovementController& mover);
    ~WallFollower() = default;

    void update();

private:
    LidarSensor& lidar_sensor_;
    MovementController& movement_controller_;
};

#endif
