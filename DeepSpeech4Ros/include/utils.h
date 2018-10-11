//
// Created by rfeldhans on 18.08.18.
//

#ifndef DEEPSPEECH_4_ROS_UTILS_H
#define DEEPSPEECH_4_ROS_UTILS_H


#include <string>
#include "ros/ros.h"

namespace utils{

    struct config{
        std::string ros_node_name;
        std::string ros_recognize_speech_topic;
        std::string jack_client_name;
        std::string jack_server_name;
        std::string jack_input_port_name;
        std::string model_path;
        std::string alphabet_path;
        std::string lm_path;
        std::string trie_path;
        int sample_rate;
        int max_audio_length;
    };

    void read_config(config &config, std::string config_file);

    bool array_zero(float* array, size_t size);

}

#endif //DEEPSPEECH_4_ROS_UTILS_H
