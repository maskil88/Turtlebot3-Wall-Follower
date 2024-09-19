#include "turtlebot3_gazebo/WallFollower.hpp"

WallFollower::WallFollower(rclcpp::Node* node,
                           LidarSensor* lidar_sensor,
                           MovementController* movement_controller,
                           CameraProcessor* camera_processor)
{
    // Initialise relevant variables
    node_ = node;
    lidar_sensor_ = lidar_sensor;
    movement_controller_ = movement_controller;
    camera_processor_ = camera_processor;
}
//---
WallFollower::~WallFollower() {}
//---
void WallFollower::update()
{
    // Left wall-following parameter
    double target_distance = 0.35;          // Desired distance from the left wall (meters)
    double distance_tolerance = 0.05;       // Allowable deviation from the target distance
    double min_turn_distance = 0.1;         // Minimum distance before making a turn
    double check_forward_dist = 0.4;        // Distance threshold for obstacles in front
    double linear_speed = 0.16;             // Speed to move forward
    double angular_speed = 0.45;            // Speed to turn left or right
    double proportional_gain = 1.5;         // Proportional control gain for smoother wall following

    // Stop the TurtleBot if the maze exit is detected
    if (camera_processor_->is_green_block_detected())
    {
        movement_controller_->update_cmd_vel(0.0, 0.0);
        RCLCPP_INFO(node_->get_logger(), "Green block detected. Stopping the robot.");
        return;
    }

    // Read LIDAR data
    double* scan_data = lidar_sensor_->get_scan_data();

    // CASE 1: If there is an obstacle in front (wall detected in front)
    if (scan_data[FRONT] < check_forward_dist)
    {
        // Slow down and turn right in place to avoid the wall in front
        movement_controller_->update_cmd_vel(0.0, -angular_speed);  // Turn right slowly
    }

    // CASE 2: If the robot detects a left wall within the acceptable range
    else if (scan_data[LEFT_FRONT] > (target_distance - distance_tolerance) && scan_data[LEFT_FRONT] < (target_distance + distance_tolerance))
    {
        // Move forward smoothly
        movement_controller_->update_cmd_vel(linear_speed, 0.0);
    }
    
    // CASE 3: The robot is too far from the left wall (Turn left slightly)
    else if (scan_data[LEFT_FRONT] > (target_distance + distance_tolerance))
    {
        // Proportional control to adjust turning angle
        double error = scan_data[LEFT_FRONT] - target_distance;
        double proportional_turn = proportional_gain * error;
        movement_controller_->update_cmd_vel(linear_speed, std::min(proportional_turn, angular_speed));  // Turn left smoothly
    }

    // CASE 4: The robot is too close to the left wall (Turn right slightly)
    else if (scan_data[LEFT_FRONT] < (target_distance - distance_tolerance))
    {
        // Proportional control to adjust turning angle
        double error = target_distance - scan_data[LEFT_FRONT];
        double proportional_turn = proportional_gain * error;
        movement_controller_->update_cmd_vel(linear_speed, -std::min(proportional_turn, angular_speed));  // Turn right smoothly
    }

    // CASE 5: No wall detected on the left (Turn left in place to find the wall)
    else if (std::isinf(scan_data[LEFT_FRONT]))
    {
        // Slow left turn in place
        movement_controller_->update_cmd_vel(0.0, 1.3 * angular_speed);
    }
}
