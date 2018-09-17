
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



void process_sample(){

    size_t buffer_size = 0;
    char* buffer = nullptr;
    int sampleRate = 0;

    // prepare audio

    // aquire result
    struct DsSTT::ds_result* result = DsSTT::LocalDsSTT(model, (const short*)buffer,
                                                        buffer_size / 2, sampleRate);
    free(buffer);

    // handle result if existing
    if (result) {
        if (result->string) {
            printf("%s\n", result->string);
            free(result->string);
        }

        printf("cpu_time_overall=%.05f cpu_time_mfcc=%.05f "
                       "cpu_time_infer=%.05f\n",
               result->cpu_time_overall,
               result->cpu_time_mfcc,
               result->cpu_time_infer);


        free(result);
    } else{

    }
}


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