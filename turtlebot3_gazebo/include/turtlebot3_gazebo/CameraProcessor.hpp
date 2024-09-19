#ifndef CAMERA_PROCESSOR_HPP_
#define CAMERA_PROCESSOR_HPP_

//---Includes and #defines--------------------------------------------------------
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define GREEN_COUNT_THRESHOLD 50000

//---CameraProcessor Interface----------------------------------------------------
class CameraProcessor
{
    public:
        
        //---Constructor & Destructor---------------------------------------------
        CameraProcessor(rclcpp::Node* node);
        ~CameraProcessor();

        // Returns a boolean indicating whether the end of the maze has been reached
        bool is_green_block_detected();

    private:

        // Handles camera image data
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

        // Contains logic to determine if the end of the maze has been reached
        bool detect_green_block(const cv::Mat& image);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;    // Camera subscriber variable
        bool green_block_detected_;                                             // Maze exit flag
        rclcpp::Node* node_;
};

#endif
