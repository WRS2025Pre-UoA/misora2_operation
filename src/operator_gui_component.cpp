#include "misora2_operation/operator_gui_component.hpp"

namespace component_operator_gui
{
MisoraGUI::MisoraGUI(const rclcpp::NodeOptions &options) 
    : Node("misora_gui", options)
{
    // ミッションごとにモードを変更------------------------------------------------------------------------------------------
    this->declare_parameter("mode", "P6");
    std::string param = this->get_parameter("mode").as_string();
    RCLCPP_INFO(this->get_logger(), "Received my_parameter: %s", param.c_str());
    
    // ミッションごとのボタン表示名------------------------------------------------------------------------------------------
    std::map<std::string, std::vector<std::string>> pub_sub_topics = {
        {"P1", {"pressure", "qr", "V_maneuve", "send"}},
        {"P2", {"pressure", "qr", "V_maneuve", "V_state", "send"}},
        {"P3", {"cracks", "qr", "metal_loss", "send"}},
        {"P4", {"qr", "disaster", "debris", "V_maneuve", "send"}},
        {"P6", {"pressure", "qr", "debris", "disaster", "missing", "send"}}
    };

    // 連続処理のトリガー bool_publisher初期化-----------------------------------------------------------------------------
    if (pub_sub_topics.find(param) != pub_sub_topics.end()) {
        for (const auto &topic : pub_sub_topics[param]) {
            if(std::find(trigger_list.begin(), trigger_list.end(), topic) != trigger_list.end()){
                bool_triggers_[topic] = this->create_publisher<std_msgs::msg::Bool>(topic+"_trigger", 10);
                RCLCPP_INFO(this->get_logger(), "Created publisher for topic: %s", topic.c_str());
            }
            
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid profile: %s", param.c_str());
    } 

    // MISORA側から送られてくる結果、画像を受け取るsubscriber初期化----------------------------------------------------------
    if (pub_sub_topics.find(param) != pub_sub_topics.end()) {
        for (const auto &topic : pub_sub_topics[param]) {
            if(std::find(trigger_list.begin(), trigger_list.end(), topic) != trigger_list.end()){
                receive_data_[topic] = this->create_subscription<std_msgs::msg::String>(topic+"_result_data", 10,
                    [this, topic](const std_msgs::msg::String::SharedPtr msg){
                    if(topic == "qr") {
                        latest_qr = true;
                        id = *msg;
                    }
                    else {
                        latest_topic = topic;
                        result_data = *msg;
                    }
                    rewriteMessage();
                });// 受け取り時の処理
                receive_image_[topic] = this->create_subscription<MyAdaptedType>(topic+"_result_image",10,
                    [this,topic](const cv::Mat msg){
                        
                    if(not(topic == "qr")) temporary_image = msg;
                });// 受け取り時の処理
                RCLCPP_INFO(this->get_logger(), "Created subscriber for topic: %s", topic.c_str());
            }
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid profile: %s", param.c_str());
    } 

    // MISORAから未加工な画像(pressureなどが処理に掛ける画像)　disaster_reportやdebris_removalのため
    receive_raw_image_ = this->create_subscription<MyAdaptedType>("raw_image",10,
        [this](const cv::Mat msg){
            temporary_image = msg;
        });

    // デジタルツインへ上げるpublisher初期化-----------------------------------------------------------------------------------
    dt_qr_publisher_ = (this->create_publisher<std_msgs::msg::String>("id", 10));
    dt_data_publisher_ = (this->create_publisher<std_msgs::msg::String>("result_data", 10));
    dt_image_publisher_ = (this->create_publisher<MyAdaptedType>("result_image", 10));// Myadaptation

    // ボタン表示設定-------------------------------------------------------------------------------------------------------
    // ミッションごとに表示するボタンのリストを作成
    buttons_name = pub_sub_topics[param];

    // ボタンの画像を送信するpublisher初期化
    publish_gui_ = this->create_publisher<MyAdaptedType>("gui_with_buttons",1);

    // ボタンのクリック判定subscriberを作成
    click_ = this->create_subscription<geometry_msgs::msg::Point>(
        "gui_with_buttons_mouse_left", 10, std::bind(&MisoraGUI::mouse_click_callback, this, std::placeholders::_1));

    // ボタンの画面生成
    mat = setup();

    // 定期的にボタン画面をpublishするtimer関数
    view_ = this->create_wall_timer(100ms, std::bind(&MisoraGUI::timer_callback, this));
}

// 画面初期化
cv::Mat MisoraGUI::setup(){
    DrawTool canvas(width,height,0);//画面描画

    for(size_t i = 0; i < buttons_name.size(); i++){
        int row = i / btn_per_row;  // 行数
        int col = i % btn_per_row;  // 列数

        // ボタン位置を更新
        Button btn(cv::Point(x_offset + col * (btn_width + btn_space_row), y_offset + row * (btn_height + btn_space_col)),cv::Size(btn_width,btn_height));
        buttons_.push_back(btn); // ボタンをリストに追加
        canvas.drawButton(btn, buttons_name[i], cv::Scalar(255, 255, 255), -1, cv::LINE_AA, 0.78, cv::Scalar(0,0,0), 2);
    }

    Button btn(cv::Point(x_offset,buttons_.back().pos.y+btn_height+15),cv::Size(btn_width*2+btn_space_row,btn_height+30));
    buttons_.push_back(btn);
    canvas.drawButton(buttons_.back(), "Receive: None", cv::Scalar(0, 0, 0), -1, cv::LINE_AA, 0.78, cv::Scalar(255,255,255), 2);

    return canvas.getImage();
}

// 定期的にpublish
void MisoraGUI::timer_callback() {
    if(not(mat.empty()))publish_gui_->publish(mat);
}


void MisoraGUI::process(std::string topic_name) {
    if(std::find(trigger_list.begin(), trigger_list.end(), topic_name) != trigger_list.end()){ //被災者の顔写真を送るのか、QRのデコードならいらないかも
        std_msgs::msg::Bool msg_b;
        msg_b.data = true;
        // RCLCPP_INFO_STREAM(this->get_logger(),"Prepare bool message to " << topic_name+"_trigger" << " " << msg_b.data);
        bool_triggers_[topic_name]->publish(msg_b);
    }
    else if(not(topic_name == "send")){ //MISORA PCから送られてきた画像をそのまま流す

        result_data.data = "OK";

        if(topic_name == "V_state" && bulb_state_count == 0){
            result_data.data = "OPEN";
        }
        else if(topic_name == "V_state" && bulb_state_count == 1){
            result_data.data = "CLOSE";
        }

        if(topic_name == "V_maneuve") result_data.data = "1";//完了の信号
        
        latest_topic = topic_name;
        rewriteMessage();
    }

    if(topic_name == "send" and latest_topic != "None" and latest_qr == true and not(temporary_image.empty())){
        dt_data_publisher_->publish(result_data);
        dt_qr_publisher_->publish(id);
        result_image = std::make_unique<cv::Mat>(temporary_image);
        RCLCPP_INFO_STREAM(this->get_logger(), "address "<<&(result_image->data));
        dt_image_publisher_->publish(std::move(result_image));

        latest_topic = "None";
        latest_qr = false;
        temporary_image.release();
        result_data.data = "None";
        id.data = "None";
        result_image = std::make_unique<cv::Mat>();
        rewriteMessage();
    }
    else if(topic_name == "send" and (latest_topic == "None" or latest_qr == false or (temporary_image.empty())))RCLCPP_INFO_STREAM(this->get_logger(),"Not Prepared data");

}

void MisoraGUI::mouse_click_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    cv::Point point(msg->x, msg->y);
    for(size_t i = 0; i < buttons_.size(); i++){
        
        // ボタンの矩形領域を定義
        cv::Rect button_rect(buttons_[i].pos, buttons_[i].size);
        // クリック位置がボタンの範囲内にあるかチェック
        if (button_rect.contains(point)) {
            cv::Point sp = cv::Point(button_rect.x, button_rect.y);
            cv::Point ep = cv::Point(button_rect.x + btn_size.width, button_rect.y + btn_size.height);
            std::string button_name_ = buttons_name[i];

            if(button_name_ == "V_state" && bulb_state_count == 0){
                bulb_state_count = 1;
                rewriteButton(sp,ep,button_name_,btn_size.width,btn_size.height,cv::Scalar(0,0,255));
            }
            else if(button_name_ == "V_state" && bulb_state_count == 1){
                bulb_state_count = 0;
                rewriteButton(sp,ep,button_name_,btn_size.width,btn_size.height,cv::Scalar(255,0,0));
            }
            else rewriteButton(sp,ep,button_name_,btn_size.width,btn_size.height,cv::Scalar(0,0,255));
            // クリックされたボタンを赤色にした状態でGUIを再描画
            publish_gui_->publish(mat);
            
            process(button_name_);

            // if(std::find(trigger_list.begin(), trigger_list.end(), button_name_) != trigger_list.end()) std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            // else std::this_thread::sleep_for(std::chrono::milliseconds(100));
            color_reset_timer_ = this->create_wall_timer(
                100ms,  // 1秒後
                [this, sp, ep, button_name_]() {
                    rewriteButton(sp, ep, button_name_, btn_size.width, btn_size.height, cv::Scalar(255, 255, 255));
                    publish_gui_->publish(mat);
                    color_reset_timer_->cancel(); // 一回で止める
                }
            );
            
            // rewriteButton(sp,ep,button_name_,btn_size.width,btn_size.height,cv::Scalar(255,255,255));
            publish_gui_->publish(mat);

            // RCLCPP_INFO(this->get_logger(), "Button '%s' clicked",button_name_.c_str());
        }
    }
}

void MisoraGUI::rewriteButton(cv::Point sp, cv::Point ep, std::string text, int btn_W, int btn_H, cv::Scalar color) const {
    cv::rectangle(mat, sp, ep, color, cv::FILLED);
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, 2, &baseline);
    cv::Point tp(sp.x + (btn_W - text_size.width) / 2, sp.y + (btn_H + text_size.height) / 2);
    cv::putText(mat, text, tp, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, cv::Scalar(0, 0, 0), 2);
}

void MisoraGUI::rewriteMessage(){
    std::string qr_text;
    if(latest_qr) qr_text ="qr ";
    else qr_text = "";
    std::string t = "Receive: " + qr_text + latest_topic;
    if(latest_topic == "V_state" and bulb_state_count == 1)t += "OP";
    else if(latest_topic == "V_state" and bulb_state_count==0)t+="CL";

    cv::Rect button_rect(buttons_.back().pos, buttons_.back().size);
    cv::Point receive_btn_sp = cv::Point(button_rect.x, button_rect.y);
    cv::Point receive_btn_ep = cv::Point(receive_btn_sp.x+buttons_.back().size.width,receive_btn_sp.y+buttons_.back().size.height);

    cv::rectangle(mat, receive_btn_sp, receive_btn_ep, 0, cv::FILLED);
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(t, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, 2, &baseline);
    cv::Point tp = cv::Point(buttons_.back().pos.x + (buttons_.back().size.width - text_size.width) / 2, buttons_.back().pos.y+ (buttons_.back().size.height + text_size.height) / 2);
    cv::putText(mat, t, tp, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, cv::Scalar(255, 255, 255), 2);
}

} // namespace component_operator_gui

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_operator_gui::MisoraGUI)