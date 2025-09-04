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
    
    // tf2の初期化 target_frame,source_frameのパラメータ取得----------------------------------------------------------------
    // target_frame_id misora2の軌跡
    this->declare_parameter("target_frame_id", "base_link");
    target_frame_id = this->get_parameter("target_frame_id").as_string();
    // source_frame_id 地図
    this->declare_parameter("source_frame_id", "map");
    source_frame_id = this->get_parameter("source_frame_id").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(), "Received target_frame_id : " << target_frame_id << ", source_frame_id : " << source_frame_id );

    // ミッションごとのボタン表示名------------------------------------------------------------------------------------------
    std::map<std::string, std::vector<std::string>> pub_sub_topics = {
        {"P1", {pressure_btn_name, qr_btn_name, V_maneuve_btn_name}},
        {"P2", {pressure_btn_name, qr_btn_name, V_maneuve_btn_name, V_stateOP_btn_name, V_stateCL_btn_name}},
        {"P3", {cracks_btn_name, qr_btn_name, metal_loss_btn_name, metal_loss_send_btn_name}},
        {"P4", {qr_btn_name, debrisN_btn_name, debrisD_btn_name, debrisC_btn_name, V_maneuve_btn_name}},
        {"P5", {qr_btn_name}},
        {"P6", {pressure_btn_name, qr_btn_name, debrisN_btn_name, debrisD_btn_name, debrisC_btn_name,missing_btn_name}}
    };

    // 連続処理のトリガー bool_publisher初期化-----------------------------------------------------------------------------
    if (pub_sub_topics.find(param) != pub_sub_topics.end()) {
        for (const auto &topic : pub_sub_topics[param]) {
            if(std::find(trigger_list.begin(), trigger_list.end(), topic) != trigger_list.end()){
                bool_triggers_[topic] = this->create_publisher<std_msgs::msg::Bool>(topic+"_trigger", 10);
                RCLCPP_INFO(this->get_logger(), "Created publisher for topic: %s", topic.c_str());// pressure,qr,cracks
            }
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid profile: %s", param.c_str());
    } 

    // MISORA側から送られてくる結果、画像を受け取るsubscriber初期化----------------------------------------------------------
    if (pub_sub_topics.find(param) != pub_sub_topics.end()) {
        for (const auto &topic : pub_sub_topics[param]) {
            if(std::find(trigger_list.begin(), trigger_list.end(), topic) != trigger_list.end()){
                receive_results_[topic] = this->create_subscription<misora2_custom_msg::msg::Custom>(topic+"_results", 10,
                    [this, topic](const misora2_custom_msg::msg::Custom::SharedPtr msg){
                    if(topic == "qr") {
                        latest_qr = true;
                        qr_data.id = msg->result;
                        qr_data.image = cv_bridge::toCvCopy(msg->image, msg->image.encoding)->image;
                        qr_data.stamp = this->now();
                    }
                    else {
                        latest_topic = topic;
                        result_data.data = msg->result;
                        result_data.image = cv_bridge::toCvCopy(msg->image, msg->image.encoding)->image;
                        result_data.stamp = this->now();
                    }
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
            if(!msg.empty()){
                temporary_image = msg;
                misora_image_flag = true;
            }
            else misora_image_flag = false;
        });
    // 減肉用画像を常に受け取り更新する
    received_image_metal_ = this->create_subscription<sensor_msgs::msg::Image>("raw_image_metal",10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg){
            if(!msg->data.empty()){
                ML_temp_image = cv_bridge::toCvCopy(msg, msg->encoding)->image;
            }
        });
    // tf2関連 ----------------------------------------------------------------------------------------
    pos_data.x = 0.0f;
    pos_data.y = 0.0f;
    pos_data.z = 0.0f;
    pos_data.roll = 0.0f;
    pos_data.pitch = 0.0f;
    pos_data.yaw = 0.0f;
    // バッファの生成
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // listenerの生成
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pos_timer_ = this->create_wall_timer(10ms, std::bind(&MisoraGUI::on_timer, this));
    pos_pub_ = this->create_wall_timer(1s, std::bind(&MisoraGUI::pos_pub_callback, this)); 
    dt_pos_publisher_ = (this->create_publisher<misora2_custom_msg::msg::Pos>("pos_data", 1));// publisherデジタルツインへ位置情報を送信
    // デジタルツインへ上げるpublisher初期化-----------------------------------------------------------------------------------
    dt_data_publisher_ = (this->create_publisher<misora2_custom_msg::msg::Digital>("digital_data", 1));// 送信上限1でもいい気がする
    data_pub_ = this->create_wall_timer(1s, std::bind(&MisoraGUI::data_pub_callback, this));
    
   
    
    // ボタン表示設定-------------------------------------------------------------------------------------------------------
    // ミッションごとに表示するボタンのリストを作成
    buttons_name_ = pub_sub_topics[param];

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

// 定期的にpublish--------------------------------------------------------------------------------------------------------------------------------------------------------
void MisoraGUI::timer_callback() {
    rewriteMessage();
    if(not(mat.empty()))publish_gui_->publish(mat);
}

// ボタンごとの信号処理--------------------------------------------------------------------------------------------------------------------------------------------------------
void MisoraGUI::process(std::string topic_name) {
    if(std::find(trigger_list.begin(), trigger_list.end(), topic_name) != trigger_list.end()){ //被災者の顔写真を送るのか、QRのデコードならいらないかも
        std_msgs::msg::Bool msg_b;
        msg_b.data = true;
        // RCLCPP_INFO_STREAM(this->get_logger(),"Prepare bool message to " << topic_name+"_trigger" << " " << msg_b.data);
        bool_triggers_[topic_name]->publish(msg_b);
    }
    else if(topic_name == metal_loss_btn_name){
        metal_loss_func();
    }
    else { //MISORA PCから送られてきた画像をそのまま流す
        if(topic_name == metal_loss_send_btn_name){
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3) << ML_min;
            result_data.data = oss.str();
            result_data.image = ML_image;
        }
        else {
            if(topic_name == V_stateOP_btn_name)result_data.data = "OPEN";
            else if(topic_name == V_stateCL_btn_name)result_data.data = "CLOSE";
            else if(topic_name == V_maneuve_btn_name)result_data.data = "100";
            else if(topic_name == debrisC_btn_name)result_data.data = "CLEARED";
            else if(topic_name == debrisD_btn_name)result_data.data = "DEBRIS";
            else if(topic_name == debrisN_btn_name)result_data.data = "NORMAL";
            else if(topic_name == missing_btn_name)result_data.data = "VICTIM";
            result_data.image = temporary_image.clone();
        }
        result_data.stamp = this->now();
        RCLCPP_INFO_STREAM(this->get_logger(), "Prepared data: " << result_data.data);
        // dt_data_publisher_->publish(result_data);
        // dt_image_publisher_->publish(result_image);
        latest_topic = topic_name;
        // rewriteMessage();
    }
}

// クリック判定--------------------------------------------------------------------------------------------------------------------------------------------------------
void MisoraGUI::mouse_click_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    cv::Point point(msg->x, msg->y);
    for(size_t i = 0; i < buttons_.size(); i++){
        
        // ボタンの矩形領域を定義
        cv::Rect button_rect(buttons_[i].pos, buttons_[i].size);
        // クリック位置がボタンの範囲内にあるかチェック
        if (button_rect.contains(point)) {
            std::string button_name = buttons_name_[i];
            if(misora_image_flag){ // send以外のボタンを押したときMISORAからの画像が送られていなければ、処理を実行しない判定式
                rewriteButton(buttons_[i],button_name,cv::Scalar(0,0,255));
                // クリックされたボタンを赤色にした状態でGUIを再描画
                publish_gui_->publish(mat);
                
                process(button_name);
            
                color_reset_timer_ = this->create_wall_timer(
                    100ms,  // 1秒後
                    [this, i, button_name]() {
                        rewriteButton(buttons_[i], button_name,  cv::Scalar(255, 255, 255));
                        publish_gui_->publish(mat);
                        color_reset_timer_->cancel(); // 一回で止める
                    }
                );
                
                publish_gui_->publish(mat);
                RCLCPP_INFO_STREAM(this->get_logger(),"Push button when received image");
            }
            else {
                cv::Mat no_image = cv::Mat::zeros(height,width,CV_8UC3);
                std::vector<std::string> text1 = {"No Image","From MISORA2","No executable"};
                // その後文字を入力
                for(int i=1;i<=3;i++){
                    int baseline = 0;
                    cv::Size text_size = cv::getTextSize(text1[i-1], cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 1, 2, &baseline);
                    // std::cout << text_size << std::endl;
                    cv::Point tp = cv::Point(int(width-text_size.width)/2,int(i*height/3) - int(i*height/6-text_size.height)/2);
                    cv::Scalar color = cv::Scalar(0, 0, 255);
                    cv::putText(no_image, text1[i-1], tp, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 1, color, 4);
                }
                publish_gui_->publish(no_image);
                error_message_timer_ = this->create_wall_timer(
                            500ms,  // 1秒後
                            [this]() {
                                publish_gui_->publish(mat);
                                error_message_timer_->cancel(); // 一回で止める
                            }
                        );
                // publish_gui_->publish(mat);    
                RCLCPP_INFO_STREAM(this->get_logger(),"Push button when unreceived image"); 
            }
            // RCLCPP_INFO(this->get_logger(), "Button '%s' clicked",button_name.c_str());
        }
    }
    
    
}

// 減肉処理---------------------------------------------------------------------------------------------------------------------------------------------------------
void MisoraGUI::metal_loss_func(){
    // 減肉処理を手入力で行う
    // 入力画面表示
    std::string show_message = "Input Metal loss value [mm: 0.0-9.0]";
    std::string text = "";
    while(true){
        cv::Mat background = cv::Mat::zeros(360, 640, CV_8UC3);

        // 上部にメッセージを表示
        int baseline_msg = 0;
        cv::Size msg_size = cv::getTextSize(show_message, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.8, 2, &baseline_msg);
        cv::Point msg_pt((640 - msg_size.width) / 2, 50); // 上から50pxの位置
        cv::putText(background, show_message, msg_pt, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.8, cv::Scalar(200,200,200), 2);

        // 下部に入力数字を表示（中央よりやや下）
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 1, 2, &baseline);
        cv::Point tp((640 - text_size.width) / 2, 220); // y=220くらいに表示
        cv::putText(background, text, tp, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 1, cv::Scalar(255,255,255), 2);

        cv::imshow("Input Metal loss value", background);
        int key = cv::waitKey(10);
        // 数字0～9と.のみ許可
        if ((key >= '0' && key <= '9') || key == '.') {
            // .がすでに含まれていたら追加しない
            if (key != '.' || text.find('.') == std::string::npos) {
                text += static_cast<char>(key);
            }
        }
        // バックスペース対応（必要なら）
        else if (key == 8 && !text.empty()) {
            text.pop_back();
        }
        else if (key == 13 || key == 10){
            if(!text.empty()){
                if(ML_min >= std::stof(text)){
                    ML_min = std::stof(text);
                    ML_image = ML_temp_image.clone();
                    RCLCPP_INFO_STREAM(this->get_logger(), "Prepared min data: " << ML_min);
                }
                // 検出した値をdoubleに変換
                
                // result_data.data = std::to_string(num);
                // result_data.image = temporary_image;
                // result_data.stamp = this->now();
                // latest_topic = metal_loss_btn_name;
                
                cv::destroyAllWindows();
                break;

            }
        }
    }
}

// tf2関連--------------------------------------------------------------------------------------------------------------------------------------------------------
void MisoraGUI::on_timer(){ 
    geometry_msgs::msg::TransformStamped t;

    try {
      // 座標変換結果の取得
      // 第一引数は"target frame"
      // 第二引数は"source frame"
      // 第三引数は変換するタイミング
      // tf2::TimePointZeroは変換可能な最新の時刻
      t = tf_buffer_->lookupTransform(
        target_frame_id, source_frame_id,
        tf2::TimePointZero);
    }
    // 変換を行えなかった時のための例外処理
    catch (const tf2::TransformException & ex) {
    //   RCLCPP_INFO(
    //     this->get_logger(), "Could not transform base_link to dynamic_frame: %s",
    //     ex.what());
      return;
    }

    // 座標変換結果の設定
    auto & trans_xyz = t.transform.translation;

    auto & quat_msg = t.transform.rotation;
    tf2::Quaternion quat(
        quat_msg.x,
        quat_msg.y,
        quat_msg.z,
        quat_msg.w);
    // クォータニオンー＞RPY変換
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    pos_data.x = trans_xyz.x;
    pos_data.y = trans_xyz.y;
    pos_data.z = trans_xyz.z;
    pos_data.roll = roll;
    pos_data.pitch = pitch;
    pos_data.yaw = yaw;
    // 座標変換結果の位置情報の出力
    // RCLCPP_INFO_STREAM(this->get_logger(), "x,y,z: " << trans_xyz.x << ", " <<  trans_xyz.y << ", " << trans_xyz.z);
    // RCLCPP_INFO_STREAM(this->get_logger(), "r,p,y: " << roll << ", " << pitch << ", " << yaw);
}

// デジタルツインへ送るデータのpublish--------------------------------------------------------------------------------------------------------
void MisoraGUI::pos_pub_callback(){
    dt_pos_publisher_->publish(pos_data);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Published pos_data");
}
void MisoraGUI::data_pub_callback(){
    if(!qr_data.id.empty() && !result_data.data.empty()){
        misora2_custom_msg::msg::Digital digital_data;
        digital_data.id = qr_data.id;
        digital_data.result = result_data.data;
        digital_data.image = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result_data.image).toImageMsg());
        dt_data_publisher_->publish(digital_data);
        // 送信したらデータをクリア
        qr_data.id.clear();
        qr_data.image.release();
        qr_data.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

        result_data.data.clear();
        result_data.image.release();
        result_data.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    // RCLCPP_INFO_STREAM(this->get_logger(), "Published pos_data");
}
// GUI回りーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
// 画面初期化--------------------------------------------------------------------------------------------------------------------------------------------------------
cv::Mat MisoraGUI::setup(){
    DrawTool canvas(width,height,0);//画面描画

    for(size_t i = 0; i < buttons_name_.size(); i++){
        int row = i / btn_per_row;  // 行数
        int col = i % btn_per_row;  // 列数

        // ボタン位置を更新
        Button btn(cv::Point(x_offset + col * (btn_width + btn_space_row), y_offset + row * (btn_height + btn_space_col)),cv::Size(btn_width,btn_height));
        buttons_.push_back(btn); // ボタンをリストに追加
        canvas.drawButton_new(btn, buttons_name_[i], cv::Scalar(255, 255, 255), -1, cv::LINE_8, 0.78, cv::Scalar(0,0,0), 1);
    }

    Button receive_qr_box_(cv::Point(0,buttons_.back().pos.y+btn_height+10),cv::Size(width,btn_height));// 今 qrを受け取っているかを表示
    another_box_.push_back(receive_qr_box_);
    canvas.drawButton_new(another_box_[0], "ID: None", cv::Scalar(0, 0, 0), -1, cv::LINE_AA, 1, cv::Scalar(255,255,255), 1);

    Button receive_box_(cv::Point(0,another_box_[0].pos.y+btn_height-5),cv::Size(width,btn_height));// 今 何を受け取っているかを表示
    another_box_.push_back(receive_box_);
    canvas.drawButton_new(another_box_[1], "Value: None", cv::Scalar(0, 0, 0), -1, cv::LINE_AA, 1, cv::Scalar(255,255,255), 1);

    return canvas.getImage();
}

// ボタンの色変更--------------------------------------------------------------------------------------------------------------------------------------------------------
void MisoraGUI::rewriteButton(Button btn, std::string text, cv::Scalar color) const {
    cv::Point sp = cv::Point(btn.pos.x, btn.pos.y);
    cv::Point ep = cv::Point(btn.pos.x + btn.size.width, btn.pos.y + btn.size.height);
    cv::rectangle(mat, sp, ep, color, cv::FILLED);
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, 1, &baseline);
    cv::Point tp(sp.x + (btn.size.width - text_size.width) / 2, sp.y + (btn.size.height + text_size.height) / 2);
    cv::putText(mat, text, tp, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, cv::Scalar(0, 0, 0), 1);
    
}

// 画面表示文字の変更--------------------------------------------------------------------------------------------------------------------------------------------------------
void MisoraGUI::rewriteMessage(){
// 値が更新されたらその都度-------------------------------------------------------------------------------
// or
// publish guiで定期的に　今-------------------------------------------------------------------------------
    std::string qr_text,receive_text;
    std::vector<std::string> text_list;

    if(!qr_data.id.empty()) qr_text ="ID: "+ qr_data.id;
    else qr_text = "ID: None";
    text_list.push_back(qr_text);

    if(!result_data.data.empty()){
        receive_text = "Value: " + result_data.data;
    }
    else receive_text = "Other: None";
    text_list.push_back(receive_text);

    // qr-----------------------------------------
    for(int n = 0; n < 2; n++){
        // まず黒いボックスで上書き
        cv::Point receive_btn_sp = cv::Point(another_box_[n].pos.x, another_box_[n].pos.y);
        cv::Point receive_btn_ep = cv::Point(receive_btn_sp.x+another_box_[n].size.width,receive_btn_sp.y+another_box_[n].size.height);
        cv::rectangle(mat, receive_btn_sp, receive_btn_ep, 0, cv::FILLED);

        // その後文字を入力
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text_list[n], cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 1, 1, &baseline);
        cv::Point tp = cv::Point(another_box_[n].pos.x + (another_box_[n].size.width - text_size.width) / 2, another_box_[n].pos.y+ (another_box_[n].size.height + text_size.height) / 2);
        cv::Scalar color = cv::Scalar(255, 255, 255);
        cv::putText(mat, text_list[n], tp, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 1, color, 1);
    }
}

} // namespace component_operator_gui

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_operator_gui::MisoraGUI)