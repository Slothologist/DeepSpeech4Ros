
// ros includes
#include "ros/ros.h"

// inludes from this project
#include "../include/utils.h"
#include "../include/deepspeech.h"
#include "../include/DsSTT.h"

#define N_CEP 26
#define N_CONTEXT 9
#define BEAM_WIDTH 500
#define LM_WEIGHT 1.75f
#define WORD_COUNT_WEIGHT 1.00f
#define VALID_WORD_COUNT_WEIGHT 1.00f

utils::config* cfg;
DeepSpeech::Model* model;



int main(int argc, char *argv[]) {

    // parse config
    cfg = new utils::config();
    read_config(cfg, argv[1]);

    // initialize deepspeech
    model = new DeepSpeech::Model(cfg->model_path, N_CEP, N_CONTEXT, cfg->alphabet_path, BEAM_WIDTH);
    model->enableDecoderWithLM(cfg->alphabet_path, cfg->lm_path, cfg->trie_path, LM_WEIGHT, WORD_COUNT_WEIGHT, VALID_WORD_COUNT_WEIGHT);

    // ros stuff
    ros::init(argc, argv, cfg->ros_node_name);
    ros::NodeHandle n;

    // Jack stuff
}