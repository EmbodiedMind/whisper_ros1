#ifndef ROSAUDIO_CONTROLLER_H
#define ROSAUDIO_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/master.h>
#include <vector>
#include <string>
#include <atomic>
#include <functional>

class Rosaudio_Controller {
public:
    Rosaudio_Controller();
    bool ros_init(const std::string& topic_name);
    void setaudioCallback(const std::function<void(void* userdata, uint8_t* stream, int len)>& cb);
    void setUserdata(void* userdata);
    void ros_shutdown();
    void spin();

private:
    void callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void invokeaudioCallback(void* userdata, uint8_t* stream, int len) const;

    std::function<void(void* userdata, uint8_t* stream, int len)> audiocallback_;
    void* userdata_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::atomic<bool> running_;
};

#endif // ROSAUDIO_CONTROLLER_H
