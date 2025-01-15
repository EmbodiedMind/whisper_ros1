// Real-time speech recognition of input from a microphone
//
// A very quick-n-dirty implementation serving mainly as a proof of concept.
//
#include "common-ros.h"
#include "common.h"
#include "whisper.h"
#include <ros/ros.h>
#include <cassert>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <std_msgs/String.h>



// 参数结构体
struct whisper_params {
    int32_t n_threads  = std::min(8, (int32_t) std::thread::hardware_concurrency());
    int32_t step_ms    = 0;
    int32_t length_ms  = 30000;
    int32_t keep_ms    = 1000;
    int32_t capture_id = -1;
    int32_t max_tokens = 32;
    int32_t audio_ctx  = 0;

    float vad_thold    = 0.6f;
    float freq_thold   = 100.0f;

    bool translate     = false;
    bool no_fallback   = false;
    bool print_special = false;
    bool no_context    = true;
    bool no_timestamps = false;
    bool tinydiarize   = false;
    bool save_audio    = false; // save audio to wav file
    bool use_gpu       = false;
    bool flash_attn    = false;

    std::string language  = "zh";

    std::string model     = "/path/to/ggml-medium.bin";
    std::string rosnode_name = "whisper_ros1_node";
    std::string subscriber_topic = "/audio_stream";
    std::string publisher_topic = "/txt_stream";

    std::string fname_out;
};

// 打印参数使用说明
void whisper_print_usage(const whisper_params & params) {
    ROS_INFO("\n");
    ROS_INFO("ROS parameters usage:");
    ROS_INFO("  _n_threads:=N         [default: %d] number of threads to use during computation", params.n_threads);
    ROS_INFO("  _step_ms:=N           [default: %d] audio step size in milliseconds", params.step_ms);
    ROS_INFO("  _length_ms:=N         [default: %d] audio length in milliseconds", params.length_ms);
    ROS_INFO("  _keep_ms:=N           [default: %d] audio to keep from previous step in ms", params.keep_ms);
    ROS_INFO("  _max_tokens:=N        [default: %d] maximum number of tokens per audio chunk", params.max_tokens);
    ROS_INFO("  _audio_ctx:=N         [default: %d] audio context size (0 - all)", params.audio_ctx);
    ROS_INFO("  _vad_thold:=N         [default: %.2f] voice activity detection threshold", params.vad_thold);
    ROS_INFO("  _freq_thold:=N        [default: %.2f] high-pass frequency cutoff", params.freq_thold);
    ROS_INFO("  _translate:=true/false [default: %s] translate from source language to english", params.translate ? "true" : "false");
    ROS_INFO("  _no_fallback:=true/false [default: %s] do not use temperature fallback while decoding", params.no_fallback ? "true" : "false");
    ROS_INFO("  _print_special:=true/false [default: %s] print special tokens", params.print_special ? "true" : "false");
    ROS_INFO("  _no_context:=true/false [default: %s] keep context between audio chunks", params.no_context ? "false" : "true");
    ROS_INFO("  _language:=LANG       [default: %s] spoken language", params.language.c_str());
    ROS_INFO("  _model:=FNAME         [default: %s] model path", params.model.c_str());
    ROS_INFO("  _fname_out:=FNAME     [default: %s] text output file name", params.fname_out.c_str());
    ROS_INFO("  _tinydiarize:=true/false [default: %s] enable tinydiarize (requires a tdrz model)", params.tinydiarize ? "true" : "false");
    ROS_INFO("  _save_audio:=true/false [default: %s] save the recorded audio to a file", params.save_audio ? "true" : "false");
    ROS_INFO("  _use_gpu:=true/false  [default: %s] disable GPU inference", params.use_gpu ? "false" : "true");
    ROS_INFO("  _flash_attn:=true/false [default: %s] flash attention during inference", params.flash_attn ? "true" : "false");
    ROS_INFO("\n");
}

// 从ROS参数服务器加载参数
static bool whisper_params_load(whisper_params & params) {
    ros::NodeHandle nh("~"); // 使用私有命名空间获取参数

    // 从ROS参数服务器获取参数
    nh.param("n_threads", params.n_threads, params.n_threads);
    nh.param("step_ms", params.step_ms, params.step_ms);
    nh.param("length_ms", params.length_ms, params.length_ms);
    nh.param("keep_ms", params.keep_ms, params.keep_ms);
    nh.param("capture_id", params.capture_id, params.capture_id);
    nh.param("max_tokens", params.max_tokens, params.max_tokens);
    nh.param("audio_ctx", params.audio_ctx, params.audio_ctx);

    nh.param("vad_thold", params.vad_thold, params.vad_thold);
    nh.param("freq_thold", params.freq_thold, params.freq_thold);

    nh.param("translate", params.translate, params.translate);
    nh.param("no_fallback", params.no_fallback, params.no_fallback);
    nh.param("print_special", params.print_special, params.print_special);
    nh.param("no_context", params.no_context, params.no_context);
    nh.param("no_timestamps", params.no_timestamps, params.no_timestamps);
    nh.param("tinydiarize", params.tinydiarize, params.tinydiarize);
    nh.param("save_audio", params.save_audio, params.save_audio);
    nh.param("use_gpu", params.use_gpu, params.use_gpu);
    nh.param("flash_attn", params.flash_attn, params.flash_attn);

    nh.param("language", params.language, params.language);
    nh.param("model", params.model, params.model);
    nh.param("rosnode_name", params.rosnode_name, params.rosnode_name);
    nh.param("subscriber_topic", params.subscriber_topic, params.subscriber_topic);
    nh.param("publisher_topic", params.publisher_topic, params.publisher_topic);
    nh.param("fname_out", params.fname_out, params.fname_out);

    return true;
}

int main(int argc, char ** argv) {

    ros::init(argc, argv, "whisper_ros1_node");
    ros::NodeHandle nh;
    whisper_params params;

    if (!whisper_params_load(params)) {
        ROS_ERROR("Failed to load parameters from ROS parameter server.");
        return 1;
    }

    // 打印参数使用说明
    whisper_print_usage(params);

    // 打印加载的参数
    ROS_INFO("Loaded parameters:");
    ROS_INFO("  n_threads: %d", params.n_threads);
    ROS_INFO("  step_ms: %d", params.step_ms);
    ROS_INFO("  language: %s", params.language.c_str());
    ROS_INFO("  model: %s", params.model.c_str());


    params.keep_ms   = std::min(params.keep_ms,   params.step_ms);
    params.length_ms = std::max(params.length_ms, params.step_ms);

    const int n_samples_step = (1e-3*params.step_ms  )*WHISPER_SAMPLE_RATE;
    const int n_samples_len  = (1e-3*params.length_ms)*WHISPER_SAMPLE_RATE;
    const int n_samples_keep = (1e-3*params.keep_ms  )*WHISPER_SAMPLE_RATE;
    const int n_samples_30s  = (1e-3*30000.0         )*WHISPER_SAMPLE_RATE;

    const bool use_vad = n_samples_step <= 0; // sliding window mode uses VAD

    const int n_new_line = !use_vad ? std::max(1, params.length_ms / params.step_ms - 1) : 1; // number of steps to print new line

    params.no_timestamps  = !use_vad;
    params.no_context    |= use_vad;
    params.max_tokens     = 0;

 


    // 创建发布器
    ros::Publisher transcription_pub = nh.advertise<std_msgs::String>(params.publisher_topic, 10);

   
    rosaudio_asyns audio(params.length_ms) ;

    if(!audio.init(params.subscriber_topic, WHISPER_SAMPLE_RATE)){
        fprintf(stderr, "%s: audio.init() failed!\n", __func__);
        return 1;
    }

    audio.resume();



    // whisper init
    if (params.language != "auto" && whisper_lang_id(params.language.c_str()) == -1){
        fprintf(stderr, "error: unknown language '%s'\n", params.language.c_str());
        whisper_print_usage(params);
        exit(0);
    }

    struct whisper_context_params cparams = whisper_context_default_params();

    cparams.use_gpu    = params.use_gpu;
    cparams.flash_attn = params.flash_attn;

    struct whisper_context * ctx = whisper_init_from_file_with_params(params.model.c_str(), cparams);

    std::vector<float> pcmf32    (n_samples_30s, 0.0f);
    std::vector<float> pcmf32_old;
    std::vector<float> pcmf32_new(n_samples_30s, 0.0f);

    std::vector<whisper_token> prompt_tokens;

    // print some info about the processing
    {
        fprintf(stderr, "\n");
        if (!whisper_is_multilingual(ctx)) {
            if (params.language != "en" || params.translate) {
                params.language = "en";
                params.translate = false;
                fprintf(stderr, "%s: WARNING: model is not multilingual, ignoring language and translation options\n", __func__);
            }
        }
        fprintf(stderr, "%s: processing %d samples (step = %.1f sec / len = %.1f sec / keep = %.1f sec), %d threads, lang = %s, task = %s, timestamps = %d ...\n",
                __func__,
                n_samples_step,
                float(n_samples_step)/WHISPER_SAMPLE_RATE,
                float(n_samples_len )/WHISPER_SAMPLE_RATE,
                float(n_samples_keep)/WHISPER_SAMPLE_RATE,
                params.n_threads,
                params.language.c_str(),
                params.translate ? "translate" : "transcribe",
                params.no_timestamps ? 0 : 1);

        if (!use_vad) {
            fprintf(stderr, "%s: n_new_line = %d, no_context = %d\n", __func__, n_new_line, params.no_context);
        } else {
            fprintf(stderr, "%s: using VAD, will transcribe on speech activity\n", __func__);
        }

        fprintf(stderr, "\n");
    }

    int n_iter = 0;

    bool is_running = true;

    std::ofstream fout;
    if (params.fname_out.length() > 0) {
        fout.open(params.fname_out);
        if (!fout.is_open()) {
            fprintf(stderr, "%s: failed to open output file '%s'!\n", __func__, params.fname_out.c_str());
            return 1;
        }
    }

    wav_writer wavWriter;
    // save wav file
    if (params.save_audio) {
        // Get current date/time for filename
        time_t now = time(0);
        char buffer[80];
        strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", localtime(&now));
        std::string filename = std::string(buffer) + ".wav";

        wavWriter.open(filename, WHISPER_SAMPLE_RATE, 16, 1);
    }
    printf("[Start speaking]\n");
    fflush(stdout);

    auto t_last  = std::chrono::high_resolution_clock::now();
    const auto t_start = t_last;

    // main audio loop
    while (is_running) {
        

        if (params.save_audio) {
            wavWriter.write(pcmf32_new.data(), pcmf32_new.size());
        }
        // handle Ctrl + C
        is_running = audio.isThreadRunning();

        if (!is_running) {
            break;
        }

        // process new audio

        if (!use_vad) {
            while (true) {
                audio.get(params.step_ms, pcmf32_new);

                if ((int) pcmf32_new.size() > 2*n_samples_step) {
                    fprintf(stderr, "\n\n%s: WARNING: cannot process audio fast enough, dropping audio ...\n\n", __func__);
                    audio.clear();
                    continue;
                }

                if ((int) pcmf32_new.size() >= n_samples_step) {
                    audio.clear();
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            const int n_samples_new = pcmf32_new.size();

            // take up to params.length_ms audio from previous iteration
            const int n_samples_take = std::min((int) pcmf32_old.size(), std::max(0, n_samples_keep + n_samples_len - n_samples_new));

            //printf("processing: take = %d, new = %d, old = %d\n", n_samples_take, n_samples_new, (int) pcmf32_old.size());

            pcmf32.resize(n_samples_new + n_samples_take);

            for (int i = 0; i < n_samples_take; i++) {
                pcmf32[i] = pcmf32_old[pcmf32_old.size() - n_samples_take + i];
            }

            memcpy(pcmf32.data() + n_samples_take, pcmf32_new.data(), n_samples_new*sizeof(float));

            pcmf32_old = pcmf32;
        } else {
            const auto t_now  = std::chrono::high_resolution_clock::now();
            const auto t_diff = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_last).count();

            if (t_diff < 2000) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                continue;
            }

            audio.get(2000, pcmf32_new);

            if (::vad_simple(pcmf32_new, WHISPER_SAMPLE_RATE, 1000, params.vad_thold, params.freq_thold, false)) {
                audio.get(params.length_ms, pcmf32);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                continue;
            }

            t_last = t_now;
        }

        // run the inference
        {
            whisper_full_params wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);

            wparams.print_progress   = false;
            wparams.print_special    = params.print_special;
            wparams.print_realtime   = false;
            wparams.print_timestamps = !params.no_timestamps;
            wparams.translate        = params.translate;
            wparams.single_segment   = !use_vad;
            wparams.max_tokens       = params.max_tokens;
            wparams.language         = params.language.c_str();
            wparams.n_threads        = params.n_threads;

            wparams.audio_ctx        = params.audio_ctx;

            wparams.tdrz_enable      = params.tinydiarize; // [TDRZ]

            // disable temperature fallback
            //wparams.temperature_inc  = -1.0f;
            wparams.temperature_inc  = params.no_fallback ? 0.0f : wparams.temperature_inc;

            wparams.prompt_tokens    = params.no_context ? nullptr : prompt_tokens.data();
            wparams.prompt_n_tokens  = params.no_context ? 0       : prompt_tokens.size();

            if (whisper_full(ctx, wparams, pcmf32.data(), pcmf32.size()) != 0) {
                fprintf(stderr, "%s: failed to process audio\n", argv[0]);
                return 6;
            }

            // print result;
            {
                if (!use_vad) {
                    printf("\33[2K\r");

                    // print long empty line to clear the previous line
                    printf("%s", std::string(100, ' ').c_str());

                    printf("\33[2K\r");
                } else {
                    const int64_t t1 = (t_last - t_start).count()/1000000;
                    const int64_t t0 = std::max(0.0, t1 - pcmf32.size()*1000.0/WHISPER_SAMPLE_RATE);

                   // printf("\n");
                   // printf("### Transcription %d START | t0 = %d ms | t1 = %d ms\n", n_iter, (int) t0, (int) t1);
                   // printf("\n");
                }

                const int n_segments = whisper_full_n_segments(ctx);

                std::string transcription_result;

                for (int i = 0; i < n_segments; ++i) {
                    const char * text = whisper_full_get_segment_text(ctx, i);

                    if (params.no_timestamps) {
                        printf("%s", text);
                        fflush(stdout);
                        transcription_result += text;
                        if (params.fname_out.length() > 0) {
                            fout << text;
                        }
                    } else {
                        const int64_t t0 = whisper_full_get_segment_t0(ctx, i);
                        const int64_t t1 = whisper_full_get_segment_t1(ctx, i);

                        std::string output = "[" + to_timestamp(t0, false) + " --> " + to_timestamp(t1, false) + "]  " + text;

                        if (whisper_full_get_segment_speaker_turn_next(ctx, i)) {
                            output += " [SPEAKER_TURN]";
                        }
                        

                        output += "\n";
                        transcription_result += output;
                        printf("%s", output.c_str());
                        fflush(stdout);

                        if (params.fname_out.length() > 0) {
                            fout << output;
                        }
                    }

                        std_msgs::String msg;
                        msg.data = transcription_result;
                        transcription_pub.publish(msg);
                }

                if (params.fname_out.length() > 0) {
                    fout << std::endl;
                }

                if (use_vad) {
                    printf("\n");
                    printf("### Transcription %d END\n", n_iter);
                }
            }

            ++n_iter;

            if (!use_vad && (n_iter % n_new_line) == 0) {
                printf("\n");

                // keep part of the audio for next iteration to try to mitigate word boundary issues
                pcmf32_old = std::vector<float>(pcmf32.end() - n_samples_keep, pcmf32.end());

                // Add tokens of the last full length segment as the prompt
                if (!params.no_context) {
                    prompt_tokens.clear();

                    const int n_segments = whisper_full_n_segments(ctx);
                    for (int i = 0; i < n_segments; ++i) {
                        const int token_count = whisper_full_n_tokens(ctx, i);
                        for (int j = 0; j < token_count; ++j) {
                            prompt_tokens.push_back(whisper_full_get_token_id(ctx, i, j));
                        }
                    }
                }
            }
            fflush(stdout);
        }
    }

    audio.pause();

    whisper_print_timings(ctx);
    whisper_free(ctx);

    return 0;
}
