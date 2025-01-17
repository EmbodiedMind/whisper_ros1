# Whisper ROS1 Node
![whisper_ros1_logo](https://github.com/user-attachments/assets/54aad0fe-5b17-40e7-94cc-3bb76ed85e61)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)

This repository contains a ROS1 node for real-time speech-to-text transcription using the **Whisper** model. The node subscribes to an audio stream and publishes the transcribed text to a specified topic. This project is built on top of [whisper.cpp](https://github.com/ggerganov/whisper.cpp) by Georgi Gerganov, which provides an efficient implementation of the Whisper model in C++.

---

## Demo

Check out a live demonstration of the Whisper ROS1 node in action:

！[Whisper ROS1 Demo](https://github.com/user-attachments/assets/cb17b09d-cc6a-45b1-91e2-fe8e7a8581ab)

---

## Prerequisites

- **g++-10**: Ensure you have g++ version 10 or higher installed on your system.

  ```bash
  sudo apt-get install g++-10
  ```

  If you already have a different version of g++ installed and want to use g++-10 as the default, you can update the alternatives:

  ```bash
  sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100
  sudo update-alternatives --config g++
  ```

  **Note for Ubuntu 18.04 (Bionic Beaver) Users**:  
  The default repositories for Ubuntu 18.04 do not include g++-10. To install g++-10, you need to add the Ubuntu Toolchain PPA:

  ```bash
  sudo apt update
  sudo apt install software-properties-common
  sudo add-apt-repository ppa:ubuntu-toolchain-r/test
  sudo apt update
  sudo apt install g++-10
  ```

  After installation, set g++-10 as the default compiler:

  ```bash
  sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100
  ```

---

## Installation

1. **Clone the Repository**:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/zzl410/whisper_ros1.git
   cd whisper_ros1
   ```

2. **Build the Whisper Library**:

   Before running `catkin_make`, you need to compile the Whisper library:

   ```bash
   # Enter the whisper source directory
   cd whisper_ros1/lib/whisper.cpp
   # Create and enter the build directory
   mkdir build
   cd build
   # Configure the project
   cmake ..
   # Build the library
   make
   # Install the library (requires sudo)
   sudo make install
   ```

3. **Download the Whisper Model**:

   The Whisper model needs to be downloaded and placed in the appropriate directory. You can use the provided script to download the medium model:

   ```bash
   sh ./lib/whisper.cpp/models/download-ggml-model.sh medium
   ```

   Alternatively, you can manually download the model from the following locations:

   - [Hugging Face](https://huggingface.co/ggerganov/whisper.cpp/tree/main)
   - [GGML](https://ggml.ggerganov.com)

4. **Update the Model Path**:

   Edit the `./launch/whisper.launch` file to point to the correct path of the downloaded model:

   ```xml
   <param name="model" value="/path/to/ggml-medium.bin" />
   ```

   Replace `/path/to/ggml-medium.bin` with the actual path to your model file.

5. **Build the ROS Workspace**:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

6. **Source the Workspace**:

   ```bash
   source devel/setup.bash
   ```

---

## Building a ROS Node for Audio Publishing

To build a ROS node that publishes audio data, you can refer to the following Git repository for guidance:

- [ROS Audio Publisher Node](https://github.com/zzl410/audio_publisher)

---

## Running the Node

To start the Whisper ROS1 node, run the following command:

```bash
roslaunch whisper_ros1 whisper.launch
```

---

### Node Configuration

The node is configured via the `whisper.launch` file. Below are the key parameters you can adjust:

| Parameter          | Description                                                                 |
|--------------------|-----------------------------------------------------------------------------|
| `n_threads`        | Number of threads to use for processing.                                    |
| `step_ms`          | Audio step length in milliseconds.                                          |
| `length_ms`        | Length of audio to process in milliseconds.                                 |
| `keep_ms`          | Length of audio to keep in the buffer in milliseconds.                      |
| `capture_id`       | Audio device ID.                                                            |
| `max_tokens`       | Maximum number of tokens in the transcription.                              |
| `audio_ctx`        | Audio context length.                                                       |
| `vad_thold`        | VAD (Voice Activity Detection) threshold.                                   |
| `freq_thold`       | Frequency threshold.                                                        |
| `translate`        | Whether to translate the audio to English.                                  |
| `no_fallback`      | Disable fallback to smaller models.                                         |
| `print_special`    | Print special tokens.                                                       |
| `no_context`       | Disable context from previous audio.                                        |
| `no_timestamps`    | Disable timestamps in the transcription.                                    |
| `tinydiarize`      | Enable tinydiarize mode.                                                    |
| `save_audio`       | Save the processed audio.                                                   |
| `use_gpu`          | Enable GPU acceleration (if supported).                                     |
| `flash_attn`       | Enable flash attention.                                                     |
| `language`         | Language of the audio (e.g., "zh" for Chinese).                             |
| `rosnode_name`     | Name of the ROS node.                                                       |
| `subscriber_topic` | Topic to subscribe to for audio input.                                      |
| `publisher_topic`  | Topic to publish the transcribed text.                                      |
| `fname_out`        | Output file name (if saving transcription to a file).                       |

---

### VAD (Voice Activity Detection)

The node uses a basic VAD detector to determine when to transcribe audio. The `vad_thold` parameter controls the sensitivity of the VAD. A higher value will make the detector more sensitive to silence. A value around `0.6` is generally recommended, but you may need to tune it for your specific use case.

When silence is detected, the node will transcribe the last `--length` milliseconds of audio and output a transcription block suitable for parsing.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- **Whisper model** by [OpenAI](https://openai.com/).
- **[whisper.cpp](https://github.com/ggerganov/whisper.cpp)** by [Georgi Gerganov](https://github.com/ggerganov).
