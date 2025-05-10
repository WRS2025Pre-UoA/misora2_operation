#include <iostream>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/type_adapter.hpp>

template<>
struct rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = cv::Mat;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    // 画像のエンコーディングを判定
    std::string encoding;
    if (source.channels() == 1)
    {
      encoding = "mono8";  // グレースケール画像
    }
    else if (source.channels() == 3)
    {
      encoding = "bgr8";  // カラー画像
    }
    destination = *(cv_bridge::CvImage(std_msgs::msg::Header(), encoding, source).toImageMsg());
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(source, source.encoding);  // "bgr8" は適切なエンコーディングに変更
    
    destination = cv_ptr->image;
  }
};
