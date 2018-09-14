
// ros includes
#include "ros/ros.h"

// inludes from this project
#include "../include/utils.h"
#include "../include/deepspeech.h"

#define N_CEP 26
#define N_CONTEXT 9
#define BEAM_WIDTH 500
#define LM_WEIGHT 1.75f
#define WORD_COUNT_WEIGHT 1.00f
#define VALID_WORD_COUNT_WEIGHT 1.00f


utils::config* cfg;

int main(int argc, char *argv[]) {

    const char* model_path = "/home/rfeldhans/programming/audio/deepspeech_install/model/models/output_graph.pb";
    const char* alphabet_path = "/home/rfeldhans/programming/audio/deepspeech_install/model/models/alphabet.txt";
    const char* lm_path = "/home/rfeldhans/programming/audio/deepspeech_install/model/models/lm.binary";
    const char* trie_path = "/home/rfeldhans/programming/audio/deepspeech_install/model/models/trie";

    DeepSpeech::Model model = DeepSpeech::Model(model_path, N_CEP, N_CONTEXT, alphabet_path, BEAM_WIDTH);
    model.enableDecoderWithLM(alphabet_path, lm_path, trie_path, LM_WEIGHT, WORD_COUNT_WEIGHT, VALID_WORD_COUNT_WEIGHT);

    // parse config
    cfg = new utils::config();
    read_config(cfg, argv[1]);

    // ros stuff
    ros::init(argc, argv, cfg->ros_node_name);
    ros::NodeHandle n;

    // Jack stuff
}