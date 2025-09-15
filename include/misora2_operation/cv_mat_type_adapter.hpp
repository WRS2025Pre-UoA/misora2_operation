#include <iostream>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
// #include <cv_bridge/cv_bridge.h>
#include <rclcpp/type_adapter.hpp>

template<>
struct rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = cv::Mat;
  using ros_message_type = sensor_msgs::msg::Image;

  // OpenCV → ROS2
  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    std::string encoding;
    if (source.channels() == 1)
    {
      encoding = "mono8";  // グレースケール画像
    }
    else if (source.channels() == 3)
    {
      encoding = "bgr8";  // 常にBGRで送信
    }
    destination = *(cv_bridge::CvImage(std_msgs::msg::Header(), encoding, source).toImageMsg());
  }

  // ROS2 → OpenCV
  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    cv_bridge::CvImagePtr cv_ptr;

    if (source.encoding == "rgb8")
    {
      // 受信がRGBの場合は、cv_bridgeで取り込み後にBGRへ変換
      cv_ptr = cv_bridge::toCvCopy(source, "rgb8");
      cv::cvtColor(cv_ptr->image, destination, cv::COLOR_RGB2BGR);
    }
    else
    {
      // BGRやmono8などはそのまま
      cv_ptr = cv_bridge::toCvCopy(source, source.encoding);
      destination = cv_ptr->image;
    }
  }
};
