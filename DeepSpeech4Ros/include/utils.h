//
// Created by rfeldhans on 18.08.18.
//

#ifndef DEEPSPEECH_4_ROS_UTILS_H
#define DEEPSPEECH_4_ROS_UTILS_H


#include <jack/jack.h>
#include <string>
#include "ros/ros.h"

namespace utils{

    struct config{
        std::string ros_node_name;
        std::string ros_change_config_topic;
        const char* jack_client_name;
        const char* jack_server_name = nullptr;
    };

    void read_config(config* config, std::string config_file);

}

#endif //DEEPSPEECH_4_ROS_UTILS_H
