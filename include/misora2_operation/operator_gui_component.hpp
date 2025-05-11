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
#include <std_msgs/msg/float64.hpp>
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
    const int k = 500;
    // 中身
    std::vector<std::string> buttons_name_;//表示しているボタンのリスト
    std::vector<Button> buttons_; // ボタン位置、サイズのリスト
    cv::Size btn_size = cv::Size(btn_width,btn_height);

    std::vector<Button> another_box_;
    
    // 最新の値を格納確認するリスト----------------------------------
    std::string latest_topic = "None";
    bool latest_qr = false;

    // バルブの開閉状況カウント変数 open / close---------------------
    int bulb_state_count = 0;

    //　受け取ったメッセージを格納-------------------------------------
    std_msgs::msg::String qr_id, result_data;// 確認ノードへ報告を行う時に必要なid 結果を格納
    cv::Mat temporary_image, receive_image, receive_qr_image;// sensor_msgsで送られてくるので一時的にcv::Matへ misora空の生画像temp 検出ノードからreceive

    // 画面が起動しているかflag
    bool send_confirm_flag = false;
    std::vector<std::string> trigger_list = {"pressure", "qr", "cracks", "metal_loss"};
    std::vector<std::string> confirm_list = {"pressure", "cracks", "metal_loss"};

    explicit MisoraGUI(const rclcpp::NodeOptions &options);
    MisoraGUI() : MisoraGUI(rclcpp::NodeOptions{}) {}

private:
    cv::Mat setup();// ボタン画面生成生成
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);// 検出結果をうけとった時に行う処理関数
    void timer_callback();// 定期的にボタン画像を流す
    void mouse_click_callback(const geometry_msgs::msg::Point::SharedPtr msg);// ボタン画面にクリックした時の座標をもとに行う処理関数
    void rewriteButton(Button btn, std::string text, cv::Scalar color) const;// 指定したボタンの色、表示内容を変更
    void rewriteMessage();
    void process(std::string topic_name);// クリックしたボタンに対応した処理を行う関数

    rclcpp::Publisher<MyAdaptedType>::SharedPtr publish_gui_;// ボタン画面を流すpublisher
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr click_;// ボタンクリック座標を受け取るsubscriber
    rclcpp::TimerBase::SharedPtr view_;// ボタン画面を定期的に流すタイマー
    rclcpp::TimerBase::SharedPtr color_reset_timer_;// 一定時間待って、ボタンの色を白に戻す
    rclcpp::TimerBase::SharedPtr message_reset_timer_;//
    rclcpp::TimerBase::SharedPtr reopen_window_;// 確認画面が表示されてるときにsendボタンが押されたとき
    
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> bool_triggers_;// 連続処理信号

    std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> receive_data_;// 検出結果をうけとる　
    std::map<std::string, rclcpp::Subscription<MyAdaptedType>::SharedPtr> receive_image_;// 検出画像をうけとる

    rclcpp::Subscription<MyAdaptedType>::SharedPtr receive_raw_image_;// MISORAから来る生画像

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dt_qr_id_publisher_;// 確認ノードへidを送る
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dt_data_publisher_;// 確認ノードへ検出結果を送る
    rclcpp::Publisher<MyAdaptedType>::SharedPtr dt_image_publisher_;// 確認ノードへ検出画像を送る
    rclcpp::Publisher<MyAdaptedType>::SharedPtr dt_qr_image_publisher_;// 確認ノードへ検出画像を送る
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dt_flag_subscriber_;// 送信をしたか否か send or back
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dt_flag_;// 画面起動の信号
    
};

} // namespace component_operator_gui

#endif // MISORA_GUI_COMPONENT_HPP