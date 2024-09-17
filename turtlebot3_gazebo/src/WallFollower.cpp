#include "turtlebot3_gazebo/WallFollower.hpp"

WallFollower::WallFollower(LidarSensor& lidar, MovementController& mover)
: lidar_sensor_(lidar), movement_controller_(mover)
{
}

void WallFollower::update()
{
    double target_distance = 0.35;
    double distance_tolerance = 0.05;
    double min_turn_distance = 0.1;
    double check_forward_dist = 0.4;
    double linear_speed = 0.16;
    double angular_speed = 0.45;
    double proportional_gain = 1.5;

    auto scan_data = lidar_sensor_.getScanData();

    if (scan_data[FRONT] < check_forward_dist)
    {
        movement_controller_.updateCmdVel(0.0, -angular_speed);
    }
    else if (scan_data[LEFT_FRONT] > (target_distance - distance_tolerance) &&
             scan_data[LEFT_FRONT] < (target_distance + distance_tolerance))
    {
        movement_controller_.updateCmdVel(linear_speed, 0.0);
    }
    else if (scan_data[LEFT_FRONT] > (target_distance + distance_tolerance))
    {
        double error = scan_data[LEFT_FRONT] - target_distance;
        double proportional_turn = proportional_gain * error;
        movement_controller_.updateCmdVel(linear_speed, std::min(proportional_turn, angular_speed));
    }
    else if (scan_data[LEFT_FRONT] < (target_distance - distance_tolerance))
    {
        double error = target_distance - scan_data[LEFT_FRONT];
        double proportional_turn = proportional_gain * error;
        movement_controller_.updateCmdVel(linear_speed, -std::min(proportional_turn, angular_speed));
    }
    else if (std::isinf(scan_data[LEFT_FRONT]))
    {
        movement_controller_.updateCmdVel(0.0, 1.3 * angular_speed);
    }
}
