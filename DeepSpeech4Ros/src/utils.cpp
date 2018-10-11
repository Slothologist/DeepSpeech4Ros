//
// Created by rfeldhans on 18.08.18.
//


#include "../include/utils.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string/predicate.hpp>

#define PB_MODEL "output_graph.pbmm"
#define ALPHABET "alphabet.txt"
#define LM_BINARY "lm.binary"
#define TRIE "trie"


namespace utils {

    void read_config(config& config, std::string config_file) {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(config_file, pt);
        config.ros_node_name = pt.get<std::string>("ros_node_name");
        config.ros_recognize_speech_topic = pt.get<std::string>("ros_recognize_speech_topic");
        config.jack_client_name = pt.get<std::string>("jack_client_name");
        config.jack_server_name = pt.get<std::string>("jack_server_name");
        config.jack_input_port_name = pt.get<std::string>("jack_input_port_name");

        // build paths of model from singular directory given in config file
        std::string model_path = pt.get<std::string>("model_path");
        if(!boost::algorithm::ends_with(model_path, "/"))
            model_path = model_path.append("/"); // add "/" to circumvent filename merging with lowest directory

        config.model_path = config.alphabet_path = config.lm_path = config.trie_path = model_path;
        config.model_path = config.model_path.append(PB_MODEL);
        config.alphabet_path = config.alphabet_path.append(ALPHABET);
        config.lm_path = config.lm_path.append(LM_BINARY);
        config.trie_path = config.trie_path.append(TRIE);

        config.sample_rate = pt.get<int>("sample_rate");
        config.max_audio_length = pt.get<int>("max_audio_length");
        assert(config.max_audio_length%1000 == 0);

    }

    bool array_zero(float* array, size_t size){
        // find maximum and minimum value using OpenMp
        float max_val = 0.0;
        float min_val = 0.0;
        #pragma omp parallel for reduction(max : max_val), reduction(min : min_val)
        for (int i = 0; i < size; i++) {
            if(array[i] > max_val)
                max_val = array[i];
            else if (array[i] < min_val)
                min_val = array[i];
        }
        // min and max are only equal if still 0.0
        return max_val == min_val;
    }

}