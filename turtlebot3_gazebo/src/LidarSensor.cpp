#include "turtlebot3_gazebo/LidarSensor.hpp"

LidarSensor::LidarSensor(rclcpp::Node* node)
    : node_(node)
{
    // Initialise relevant variables
    node_ = node;
    for(int i = 0; i < NUM_LIDAR_ANGLES; i++){ scan_data_[i] = 0.0; }

    // Initialise LIDAR subscriber
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(&LidarSensor::scan_callback, this, std::placeholders::_1));
}
//---
LidarSensor::~LidarSensor(){}

void LidarSensor::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    uint16_t scan_angle[NUM_LIDAR_ANGLES] = {0, 40, 60, 90, 120, 270, 330};

    for (int num = 0; num < NUM_LIDAR_ANGLES; num++)
    {
        if(std::isinf(msg->ranges.at(scan_angle[num]))){ scan_data_[num] = msg->range_max; } 
        else{ scan_data_[num] = msg->ranges.at(scan_angle[num]); }
    }
}
//---
double* LidarSensor::get_scan_data(){ return scan_data_; }
