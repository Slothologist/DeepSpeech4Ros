//
// Created by rfeldhans on 18.08.18.
//


#include "../include/utils.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <regex>

#define PB_MODEL "output_graph.pbmm"
#define ALPHABET "alphabet.txt"
#define LM_BINARY "lm.binary"
#define TRIE "trie"


namespace utils {

    void autoExpandEnvironmentVariables(std::string &text) {
        static std::regex env("\\$\\{([^}]+)\\}");
        std::smatch match;
        while (std::regex_search(text, match, env)) {
            const char *s = getenv(match[1].str().c_str());
            const std::string var(s == NULL ? "" : s);
            text.replace(match[0].first, match[0].second, var);
        }
    }

    void read_config(config& config, std::string config_file) {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(config_file, pt);
        config.ros_node_name = pt.get<std::string>("ros_node_name");

        // build paths of model from singular directory given in config file
        std::string model_path = pt.get<std::string>("model_path");
        if(!boost::algorithm::ends_with(model_path, "/"))
            model_path = model_path.append("/"); // add "/" to circumvent filename merging with lowest directory
        autoExpandEnvironmentVariables(model_path);

        config.model_path = config.alphabet_path = config.lm_path = config.trie_path = model_path;
        config.model_path = config.model_path.append(PB_MODEL);
        config.alphabet_path = config.alphabet_path.append(ALPHABET);
        config.lm_path = config.lm_path.append(LM_BINARY);
        config.trie_path = config.trie_path.append(TRIE);

        config.max_audio_length = pt.get<int>("max_audio_length");
        assert(config.max_audio_length%1000 == 0);

        config.esiaf_input_topic = pt.get<std::string>("esiaf_input_topic");
        config.esiaf_speech_topic = pt.get<std::string>("esiaf_speech_topic");

    }

}