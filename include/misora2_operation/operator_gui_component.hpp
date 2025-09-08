#ifndef MISORA_GUI_COMPONENT_HPP
#define MISORA_GUI_COMPONENT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <algorithm>
#include <cctype>
#include <filesystem>

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

// listenerを扱うファイル
#include "tf2_ros/transform_listener.h"
// バッファを扱うファイル
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/msg/transform_stamped.hpp"


#include "misora2_operation/gui_tool.hpp"
#include "misora2_operation/cv_mat_type_adapter.hpp"
#include "misora2_custom_msg/msg/custom.hpp"
#include "misora2_custom_msg/msg/digital.hpp"
#include "misora2_custom_msg/msg/pos.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

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
    std::vector<Button> buttons_; // ボタン位置、サイズのリスト
    cv::Size btn_size = cv::Size(btn_width,btn_height);
    std::vector<Button> another_box_;
    
    // ボタン名----------------------------------------------------
    std::vector<std::string> buttons_name_;//表示しているボタンのリスト
    const std::string pressure_btn_name = "pressure";
    const std::string qr_btn_name = "qr";
    const std::string cracks_btn_name = "cracks";
    const std::string metal_loss_btn_name = "metal_loss";
    const std::string metal_loss_send_btn_name = "ML-send";
    const std::string V_maneuve_btn_name = "V_maneuve";
    const std::string V_stateOP_btn_name = "V_stateOP";
    const std::string V_stateCL_btn_name = "V_stateCL";
    const std::string debrisN_btn_name = "deb_Normal";
    const std::string debrisD_btn_name = "deb_Debris";
    const std::string debrisC_btn_name = "deb_Clear";
    const std::string missing_btn_name = "missing";

    // 最新の値を格納確認するリスト----------------------------------
    std::string latest_topic = "None";
    bool latest_qr = false;
    // MISORAからの画像が飛んできたか否か----------------------------
    bool misora_image_flag = false;

    //　受け取ったメッセージを格納-------------------------------------
    // std_msgs::msg::String qr_id, result_data;// 確認ノードへ報告を行う時に必要なid 結果を格納
    // cv::Mat temporary_image, receive_image, receive_qr_image;// sensor_msgsで送られてくるので一時的にcv::Matへ misora空の生画像temp 検出ノードからreceive
    struct qr_set{
        std::string id; // QRコードのID
        cv::Mat image; // QRコードの画像
        rclcpp::Time stamp; // 受信時刻
    }qr_data;
    struct result_set{
        std::string data; // 検出結果
        cv::Mat image; // 検出時の画像
        rclcpp::Time stamp; // 受信時刻
    }result_data;
    cv::Mat temporary_image; // MISORAから送信されてくる生画像
    // ボタンの条件分岐で使うリスト------------------------------------------------------
    std::vector<std::string> trigger_list = {pressure_btn_name, qr_btn_name, cracks_btn_name};
    // std::vector<std::string> confirm_list = {pressure_btn_name, cracks_btn_name, "metal_loss"};

    // MISORAから送られてくる位置情報の処理----------------------------------------------
    // tfの設定
    std::string target_frame_id, source_frame_id;
    misora2_custom_msg::msg::Pos pos_data;
    
    // 減肉関連----------------------------------------------------------------
    double ML_min = std::numeric_limits<double>::max();// 減肉の最小な値
    cv::Mat ML_image = cv::Mat::zeros(640, 480, CV_8UC1);// 減肉の最小時の画像
    cv::Mat ML_temp_image; // 減肉の一時的な画像を保存
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
    std::string input_func(std::string show_message);// 金属損失の処理　手入力
    void pos_pub_callback();// 位置情報を定期的に送信
    void data_pub_callback();// デジタルツインへ一定間隔で検査し、揃ったら報告
    void on_timer(); // tfの位置情報を定期的に取得してpos_dataに格納する関数

    rclcpp::Publisher<MyAdaptedType>::SharedPtr publish_gui_;// ボタン画面を流すpublisher
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr click_;// ボタンクリック座標を受け取るsubscriber
    rclcpp::TimerBase::SharedPtr view_;// ボタン画面を定期的に流すタイマー
    rclcpp::TimerBase::SharedPtr color_reset_timer_;// 一定時間待って、ボタンの色を白に戻す
    rclcpp::TimerBase::SharedPtr error_message_timer_;// MISORAからの画像がないとき実行できないと表示
    // rclcpp::TimerBase::SharedPtr reopen_window_;// 確認画面が表示されてるときにsendボタンが押されたとき
    
    // std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> bool_triggers_;// 連続処理信号
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr triggers_;// 連続処理信号 pressure, qr, cracks

    // std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> receive_data_;// 検出結果をうけとる　
    // std::map<std::string, rclcpp::Subscription<MyAdaptedType>::SharedPtr> receive_image_;// 検出画像をうけとる

    std::map<std::string, rclcpp::Subscription<misora2_custom_msg::msg::Custom>::SharedPtr> receive_results_;// 検出画像をうけとる

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr receive_raw_image_;// MISORAから来る生画像
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr received_image_metal_;// misoraからの減肉画像を受け取る
    // デジタルツイン関連----------------------------------------------------------------------------------------
    rclcpp::Publisher<misora2_custom_msg::msg::Digital>::SharedPtr dt_data_publisher_;// デジタルツイン報告ノードへ送る
    rclcpp::TimerBase::SharedPtr data_pub_;
    // tf2関連-----------------------------------------------------------
    // バッファの宣言
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // listenerの宣言
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::TimerBase::SharedPtr pos_timer_{nullptr};
    rclcpp::TimerBase::SharedPtr pos_pub_;
    rclcpp::Publisher<misora2_custom_msg::msg::Pos>::SharedPtr dt_pos_publisher_;// デジタルツインノードposへ位置情報を送信
    
};

} // namespace component_operator_gui

#endif // MISORA_GUI_COMPONENT_HPP