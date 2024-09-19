#include "turtlebot3_gazebo/CameraProcessor.hpp"

CameraProcessor::CameraProcessor(rclcpp::Node* node)
{
    // Intialise relevant variables
    node_ = node;
    green_block_detected_ = false;

    // Initialise image subscriber
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&CameraProcessor::image_callback, this, std::placeholders::_1));
}
//---
CameraProcessor::~CameraProcessor() {}
//---
bool CameraProcessor::is_green_block_detected() { return green_block_detected_; }
//---
void CameraProcessor::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try 
    {
        // Read image data
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        green_block_detected_ = detect_green_block(cv_ptr->image);
    } 
    catch (cv_bridge::Exception &e) 
    {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
}
//---
bool CameraProcessor::detect_green_block(const cv::Mat &image)
{
    int green_count = 0;

    // Process image data
    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            int B = image.at<cv::Vec3b>(i, j)[0];
            int G = image.at<cv::Vec3b>(i, j)[1];
            int R = image.at<cv::Vec3b>(i, j)[2];

            // Detect green pixels where G is significantly larger than R and B
            if (G > 3 * R && G > 3 * B) { green_count++; }
        }
    }

    // RCLCPP_INFO(node_->get_logger(), "Green pixel count: %d", green_count);
    return green_count > GREEN_COUNT_THRESHOLD;
}
