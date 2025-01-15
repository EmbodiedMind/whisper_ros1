#pragma once
#include <thread>  // 包含线程库

#include "rosaudio_controller.h"
#include <atomic>
#include <cstdint>
#include <vector>
#include <mutex>


//
// SDL Audio capture
//

class rosaudio_asyns {
public:
    rosaudio_asyns(int len_ms);
    ~rosaudio_asyns();

    bool init(const std::string& rostopic_name, int sample_rate);

    // start capturing audio via the provided SDL callback
    // keep last len_ms seconds of audio in a circular buffer
    bool resume();
    bool pause();
    bool clear();

    // callback to be called by SDL
    void callback(uint8_t * stream, int len);

    // get audio data from the circular buffer
    void get(int ms, std::vector<float> & audio);
    bool isThreadRunning() const;  // 获取线程状态的公共方法


private:
     void spinThread();

    int m_len_ms = 0;
    int m_sample_rate = 0;
    std::atomic_bool m_thread_closed;  //** 新增：线程关闭信号值
    std::atomic_bool m_running;
    std::mutex       m_mutex;
    std::thread m_spin_thread;  // 线程对象
    std::vector<float> m_audio;
    size_t             m_audio_pos = 0;
    size_t             m_audio_len = 0;
    Rosaudio_Controller ros_controller_;
};

