#ifndef LIDAR_SENSOR_HPP_
#define LIDAR_SENSOR_HPP_

//---Includes----------------------------------------------------------------------------------
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

//---#defines----------------------------------------------------------------------------------
#define NUM_LIDAR_ANGLES 7

#define FRONT 0                     // Forward (0°)
#define LEFT_FRONT 1                // Left-Front (30°)
#define LEFT_FRONT_INTERMEDIATE 2   // Left-Front Intermediate (60°)
#define LEFT 3                      // Left (90°)
#define FAR_LEFT 4                  // Far-Left (120°)
#define RIGHT 5                     // Right (270°)
#define RIGHT_FRONT 6               // Right-Front (330°)

//---LidarSensor Interface----------------------------------------------------------------------
class LidarSensor
{
    public:

        //---Constructor & Destructor-----------------------------------------------------------
        LidarSensor(rclcpp::Node* node);
        ~LidarSensor();

        // Returns an array of LIDAR scan angles
        double* get_scan_data();

    private:

        // Handles LIDAR data
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; // LIDAR subscription variable
        double scan_data_[NUM_LIDAR_ANGLES];                                    // Array of LIDAR angles
        rclcpp::Node* node_;
};

#endif