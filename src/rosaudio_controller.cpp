#include "rosaudio_controller.h"
/*
 * Copyright (c) 2025 zzl410
 *
 * This software is licensed under the MIT License. 
 * You may obtain a copy of the license at:
 * https://opensource.org/licenses/MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * For more details, visit: https://github.com/zzl410/whisper_ros1
 */

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


