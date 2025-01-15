#include "rosaudio_controller.h"

Rosaudio_Controller::Rosaudio_Controller() : nh_("~"), running_(false), userdata_(nullptr) {}


    // 检查话题是否存在并订阅
bool Rosaudio_Controller::ros_init(const std::string& topic_name) {
    // 获取当前所有活跃的话题
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    // 遍历话题列表，检查是否存在目标话题
    for (const auto& topic_info : topic_infos) {
        if (topic_info.name == topic_name) {
            ROS_INFO("Topic '%s' exists. Subscribing...", topic_name.c_str());
            sub_ = nh_.subscribe(topic_name, 10000, &Rosaudio_Controller::callback, this);
            printf("%s",topic_name.c_str());
            running_ = true;
            return true;  // 话题存在，返回 true
        }
    }
    ROS_WARN("Topic '%s' does not exist.", topic_name.c_str());
    return false;  // 话题不存在，返回 false
}
// 设置回调函数
void Rosaudio_Controller::setaudioCallback(const std::function<void(void* userdata, uint8_t* stream, int len)>& cb) {
    audiocallback_ = cb;
}
// 设置用户数据
void  Rosaudio_Controller::setUserdata(void* userdata) {
    userdata_ = userdata;
}
// 调用回调函数
void Rosaudio_Controller::invokeaudioCallback(void* userdata, uint8_t* stream, int len) const {
    if (audiocallback_) {
        audiocallback_(userdata, stream, len);
    } else {
        ROS_ERROR("Audio callback is not set!");
    }
}
// 取消订阅
void Rosaudio_Controller::ros_shutdown()  {
    if (running_) {
        if (sub_) {
            sub_.shutdown();
            ROS_INFO("Unsubscribed from topic.");
        }
        running_ = false;
    } else {
        ROS_WARN("No active subscription to shutdown.");
    }
}
// 主循环
void  Rosaudio_Controller::spin(){
    ros::Rate rate(50); // 循环频率 10Hz
    while (ros::ok() && running_) {
        ros::spinOnce();
        rate.sleep();
    }
}
// 话题消息的回调函数
void Rosaudio_Controller::callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    int audio_data_len = msg->data.size();

    // 将浮点数据转换为字节数据
    std::vector<uint8_t> audio_data_buffer(audio_data_len * sizeof(float));
    std::memcpy(audio_data_buffer.data(), msg->data.data(), audio_data_len * sizeof(float));

    // 获取指向字节数据的指针
    uint8_t* audio_data = audio_data_buffer.data();

    // 调用用户定义的回调函数
    invokeaudioCallback(userdata_, audio_data, audio_data_len * sizeof(float));

    // 打印日志信息（可选）
   //ROS_INFO("Audio data received: raw length = %d, bytes = %lu", audio_data_len, static_cast<unsigned long>(audio_data_len * sizeof(float)));
}


