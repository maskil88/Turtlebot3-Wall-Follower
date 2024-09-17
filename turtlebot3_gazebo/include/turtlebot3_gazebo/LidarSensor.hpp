#ifndef LIDAR_SENSOR_HPP_
#define LIDAR_SENSOR_HPP_

#include <sensor_msgs/msg/laser_scan.hpp>
#include <array>

#define NUM_LIDAR_ANGLES (7)

class LidarSensor
{
    public:
        LidarSensor();
        ~LidarSensor() = default;

        void processScan(const sensor_msgs::msg::LaserScan::SharedPtr& msg);
        std::array<double, NUM_LIDAR_ANGLES> getScanData() const;

    private:
        std::array<double, NUM_LIDAR_ANGLES> scan_data_;
};

#endif