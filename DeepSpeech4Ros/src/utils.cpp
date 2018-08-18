//
// Created by rfeldhans on 18.08.18.
//


#include "../include/utils.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>


namespace utils {

    void read_config(config *config, std::string config_file) {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(config_file, pt);
        config->ros_node_name = pt.get<std::string>("ros_node_name");
        config->ros_change_config_topic = pt.get<std::string>("ros_change_config_topic");
        config->jack_client_name = pt.get<std::string>("jack_client_name").c_str();
        //config->jack_server_name = pt.get<std::string>("jack_server_name").c_str(); TODO: read if there, don't if not
    }

}