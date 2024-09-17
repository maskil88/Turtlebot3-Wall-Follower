#include "turtlebot3_gazebo/LidarSensor.hpp"
#include <cmath>

LidarSensor::LidarSensor()
{
    scan_data_.fill(0.0);
}
//---
void LidarSensor::processScan(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
{
    uint16_t scan_angle[NUM_LIDAR_ANGLES] = {0, 37, 60, 90, 120, 270, 330};

    for (int num = 0; num < NUM_LIDAR_ANGLES; num++)
    {
        if(std::isinf(msg->ranges.at(scan_angle[num]))) { scan_data_[num] = msg->range_max; }
        else {scan_data_[num] = msg->ranges.at(scan_angle[num]); }
    }
}
//---
std::array<double, NUM_LIDAR_ANGLES> LidarSensor::getScanData() const
{
    return scan_data_;
}