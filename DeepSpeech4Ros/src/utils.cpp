//
// Created by rfeldhans on 18.08.18.
//


#include "../include/utils.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string/predicate.hpp>

#define PB_MODEL "output_graph.pb"
#define ALPHABET "alphabet.txt"
#define LM_BINARY "lm.binary"
#define TRIE "trie"


namespace utils {

    void read_config(config *config, std::string config_file) {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(config_file, pt);
        config->ros_node_name = pt.get<std::string>("ros_node_name");
        config->ros_change_config_topic = pt.get<std::string>("ros_change_config_topic");
        config->jack_client_name = pt.get<std::string>("jack_client_name").c_str();
        //config->jack_server_name = pt.get<std::string>("jack_server_name").c_str(); TODO: read if there, don't if not

        // build paths of model from singular directory given in config file
        std::string model_path = pt.get<std::string>("model_path");
        if(!boost::algorithm::ends_with(model_path, "/"))
            model_path = model_path.append("/"); // add "/" to circumvent filename merging with lowest directory

        // TODO: way to complicated and hacky to remain.
        // Just using something like config->model_path = (model_path.append(PB_MODEL)).c_str() does not work because
        // the lifetime of the char* retrieved by c_str() ends when this function returns
        char* mp = new char[(model_path + PB_MODEL).length()];
        strcpy(mp, (model_path + PB_MODEL).c_str());
        config->model_path = mp;

        mp = new char[(model_path + ALPHABET).length()];
        strcpy(mp, (model_path + ALPHABET).c_str());
        config->alphabet_path = mp;

        mp = new char[(model_path + LM_BINARY).length()];
        strcpy(mp, (model_path + LM_BINARY).c_str());
        config->lm_path = mp;

        mp = new char[(model_path + TRIE).length()];
        strcpy(mp, (model_path + TRIE).c_str());
        config->trie_path = mp;

        config->sample_rate = pt.get<int>("sample_rate");
        config->max_audio_length = pt.get<int>("max_audio_length");
        assert(config->max_audio_length%1000 == 0);

    }

}