#include "common-ros.h"

#include <cstdio>
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

rosaudio_asyns::rosaudio_asyns(int len_ms) 
: ros_controller_() {
    m_len_ms = len_ms;
    m_running = false;
    m_thread_closed = true;  //** Added: Initial state is closed
}


rosaudio_asyns::~rosaudio_asyns() {
    pause();  // Ensure the thread stops
    if (m_spin_thread.joinable()) {
        m_spin_thread.join();  // Wait for the thread to finish
    }
}

bool rosaudio_asyns::init(const std::string& rostopic_name, int sample_rate) 
   {
    m_sample_rate = sample_rate;
    if (ros_controller_.ros_init(rostopic_name)) {
        ros_controller_.setaudioCallback(
            [](void* userdata, uint8_t* stream, int len) {
               auto* self = static_cast<rosaudio_asyns*>(userdata);
                self->callback(stream, len);
            }
        );
        ros_controller_.setUserdata(this);


        m_audio.resize((m_sample_rate*m_len_ms)/1000*2);
        return true;
    }
    return false;
}
 
// Thread function
void rosaudio_asyns::spinThread() {
    m_thread_closed = false;  //** Added: Thread starts running
    ros_controller_.spin();   // Run spin()
    m_thread_closed = true;   //** Added: Thread is closed
}

bool rosaudio_asyns::resume() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_running) {
        ROS_WARN("%s: already running!", __func__);
        return false;
    }
    m_running = true;
    if (m_spin_thread.joinable()) {
        m_spin_thread.join();
    }
    m_spin_thread = std::thread(&rosaudio_asyns::spinThread, this);  // Start the thread
    return true;
}

bool rosaudio_asyns::pause() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_running) {
        fprintf(stderr, "%s: already paused!\n", __func__);
        return false;
    }

    ros_controller_.ros_shutdown();
    m_running = false;
    if (m_spin_thread.joinable()) {
        m_spin_thread.join();  // Wait for the thread to finish
    }

    return true;
}

bool rosaudio_asyns::clear() {
    if (!m_running) {
        fprintf(stderr, "%s: not running!\n", __func__);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        m_audio_pos = 0;
        m_audio_len = 0;
    }

    return true;
}

// Callback to be called by rosaudio
void rosaudio_asyns::callback(uint8_t * stream, int len) {
    if (!m_running || !stream || len <= 0 || len % sizeof(float) != 0) {
        ROS_WARN("Invalid callback parameters or system not running.");
        return;
    }

    size_t n_samples = len / sizeof(float);

    if (n_samples > m_audio.size()) {
        ROS_WARN("Audio data exceeds buffer size. Truncating...");
        n_samples = m_audio.size();
        stream += (len - (n_samples * sizeof(float)));
    }

   // fprintf(stderr, "%s: %zu samples, pos %zu, len %zu\n", __func__, n_samples, m_audio_pos, m_audio_len);

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_audio_pos + n_samples > m_audio.size()) {
            const size_t n0 = m_audio.size() - m_audio_pos;

            memcpy(&m_audio[m_audio_pos], stream, n0 * sizeof(float));
            memcpy(&m_audio[0], stream + n0 * sizeof(float), (n_samples - n0) * sizeof(float));

            m_audio_pos = (m_audio_pos + n_samples) % m_audio.size();
            m_audio_len = m_audio.size();
        } else {
            memcpy(&m_audio[m_audio_pos], stream, n_samples * sizeof(float));

            m_audio_pos = (m_audio_pos + n_samples) % m_audio.size();
            m_audio_len = std::min(m_audio_len + n_samples, m_audio.size());
        }
    }
}

void rosaudio_asyns::get(int ms, std::vector<float> & result) {


    if (!m_running) {
        fprintf(stderr, "%s: not running!\n", __func__);
        return;
    }

    result.clear();

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (ms <= 0) {
            ms = m_len_ms;
        }

        size_t n_samples = (m_sample_rate * ms) / 1000;
        if (n_samples > m_audio_len) {
            n_samples = m_audio_len;
        }

        result.resize(n_samples);

        int s0 = m_audio_pos - n_samples;
        if (s0 < 0) {
            s0 += m_audio.size();
        }

        if (s0 + n_samples > m_audio.size()) {
            const size_t n0 = m_audio.size() - s0;

            memcpy(result.data(), &m_audio[s0], n0 * sizeof(float));
            memcpy(&result[n0], &m_audio[0], (n_samples - n0) * sizeof(float));
        } else {
            memcpy(result.data(), &m_audio[s0], n_samples * sizeof(float));
        }
    }
}
// Public method to get thread status
bool rosaudio_asyns::isThreadRunning() const {
    return !m_thread_closed;  //** Added: Returns whether the thread is closed
}
