#ifndef MISORA_GUI_COMPONENT_HPP
#define MISORA_GUI_COMPONENT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <algorithm>


#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/type_adapter.hpp>

#include "misora2_operation/gui_tool.hpp"
#include "misora2_operation/cv_mat_type_adapter.hpp"

using namespace std::chrono_literals;

namespace component_operator_gui
{
class MisoraGUI : public rclcpp::Node
{
public:
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;

    // ボタンの設定-------------------------------------------------
    cv::Mat mat;// ボタンの画像
    // 描画設定
    int width = 300, height = 350;//画面サイズ
    int btn_width = 140, btn_height = 50;//ボタンのサイズ
    int x_offset = 5,y_offset = 25;//ボタンの最初の位置
    int btn_per_row = 2;//一行に何個ボタン設置するか
    int btn_space_row = 5, btn_space_col = 30;//ボタン間のスペース

    // 中身
    std::vector<std::string> buttons_name;//表示しているボタンのリスト
    std::vector<Button> buttons_; // ボタン位置、サイズのリスト
    cv::Size btn_size = cv::Size(btn_width,btn_height);

    // 最新の値を格納確認するリスト----------------------------------
    // std::vector<std::map<std::string, bool>> receive_list;
    std::string latest_topic = "None";
    bool latest_qr = false;

    // バルブの開閉状況カウント変数 open / close---------------------
    int bulb_state_count = 0;

    //　受け取ったメッセージを格納-------------------------------------
    std_msgs::msg::String id, result_data;
    cv::Mat temporary_image;
    std::unique_ptr<cv::Mat> result_image;

    std::vector<std::string> trigger_list = {"pressure", "qr", "cracks", "metal_loss"};
    explicit MisoraGUI(const rclcpp::NodeOptions &options);
    MisoraGUI() : MisoraGUI(rclcpp::NodeOptions{}) {}

private:
    cv ::Mat setup();
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    void timer_callback();
    void mouse_click_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void rewriteImage(cv::Point sp, cv::Point ep, std::string text, int btn_W, int btn_H, cv::Scalar color) const;
    void process(std::string topic_name);

    rclcpp::Publisher<MyAdaptedType>::SharedPtr publish_gui_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr click_;
    rclcpp::TimerBase::SharedPtr view_;

    // std::map<std::string, rclcpp::Time> button_press_times_;
    
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> bool_triggers_;
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> receive_data_;
    std::map<std::string, rclcpp::Subscription<MyAdaptedType>::SharedPtr> receive_image_;

    rclcpp::Subscription<MyAdaptedType>::SharedPtr receive_raw_image_;// MISORAから来る生画像

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dt_qr_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dt_data_publisher_;
    rclcpp::Publisher<MyAdaptedType>::SharedPtr dt_image_publisher_;
    
};

} // namespace component_operator_gui

#endif // MISORA_GUI_COMPONENT_HPP