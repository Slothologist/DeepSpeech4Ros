//
// Created by rfeldhans on 18.08.18.
//

#ifndef DEEPSPEECH_4_ROS_UTILS_H
#define DEEPSPEECH_4_ROS_UTILS_H


#include <string>
#include <ros/ros.h>

namespace utils{

    struct config{
        std::string ros_node_name;
        std::string model_path;
        std::string alphabet_path;
        std::string lm_path;
        std::string trie_path;
        int max_audio_length;
        std::string esiaf_input_topic;
        std::string esiaf_speech_topic;
    };

    void read_config(config &config, std::string config_file);

}

#endif //DEEPSPEECH_4_ROS_UTILS_H
