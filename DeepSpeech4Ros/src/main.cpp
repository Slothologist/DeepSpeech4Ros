
// ros includes
#include "ros/ros.h"

// jack includes
#include <jack/jack.h>

// inludes from this project
#include "../include/utils.h"
#include "../include/deepspeech.h"
#include "../include/DsSTT.h"

// constants used by deepspeech.
#define N_CEP 26
#define N_CONTEXT 9
#define BEAM_WIDTH 500
#define LM_WEIGHT 1.75f
#define VALID_WORD_COUNT_WEIGHT 1.00f

// config for this deepspeech node
utils::config* cfg;

// model used by deepspeech
ModelState* model;

// buffer and its size to temporarily save audio received from jack
size_t buffer_size;
void* buffer;

// jack client and input
jack_client_t *client;
jack_port_t *input_port;



void process_sample(){

    // prepare audio

    // sample down audio

    // aquire result
    DsSTT::ds_result result = DsSTT::LocalDsSTT(model, (const short*)buffer,
                                                        buffer_size / 2, cfg->sample_rate);
    free(buffer);

    if (result.string) {
        printf("%s\n", result.string);
    }
    printf("cpu_time_overall=%.05f\n",
           result.cpu_time_overall);

}

void initialize_deepspeech(){
    int status = DS_CreateModel(cfg->model_path, N_CEP, N_CONTEXT, cfg->alphabet_path, BEAM_WIDTH, &model);
    if (status != 0) {
        fprintf(stderr, "Could not create model.\n");
        exit(1);
    }
    status = DS_EnableDecoderWithLM(model,
                                    cfg->alphabet_path,
                                    cfg->lm_path,
                                    cfg->trie_path,
                                    LM_WEIGHT,
                                    VALID_WORD_COUNT_WEIGHT);
    if (status != 0) {
        fprintf(stderr, "Could not enable CTC decoder with LM.\n");
        exit(1);
    }
}

void initialize_jack(){

    auto jack_server_name = (int) JackNullOption;
    const char **ports;
    const char *client_name = cfg->jack_client_name;
    const char *server_name = cfg->jack_server_name;
    jack_options_t options = JackNullOption;
    jack_status_t jack_status;

    if (server_name != nullptr) {
        jack_server_name |= JackServerName;
        options = (jack_options_t) jack_server_name;
    }

    /* open a client connection to the JACK server */
    client = jack_client_open(client_name, options, &jack_status, server_name);
    if (client == nullptr) {
        fprintf(stderr, "jack_client_open() failed, "
                        "status = 0x%2.0x\n", jack_status);
        if (jack_status & JackServerFailed) {
            fprintf(stderr, "Unable to connect to JACK server\n");
        }
        exit(1);
    }
    if (jack_status & JackServerStarted) {
        fprintf(stderr, "JACK server started\n");
    }
    if (jack_status & JackNameNotUnique) {
        client_name = jack_get_client_name(client);
        fprintf(stderr, "unique name `%s' assigned\n", client_name);
    }

    // create buffer for holding sound longer than one jack process_frame() call
    buffer_size = jack_get_sample_rate(client) * sizeof(jack_default_audio_sample_t) * cfg->max_audio_length/1000;
    buffer = malloc(buffer_size);
}


int main(int argc, char *argv[]) {

    // parse config
    cfg = new utils::config();
    read_config(cfg, argv[1]);

    initialize_deepspeech();

    initialize_jack();

    // ros initilisation
    ros::init(argc, argv, cfg->ros_node_name);
    ros::NodeHandle n;

    // destructing stuff and freeing buffers
    DS_DestroyModel(model);
    free(buffer);
    // freeing the config's paths is a bit ugly, but here we go
    delete(cfg->model_path);
    delete(cfg->alphabet_path);
    delete(cfg->lm_path);
    delete(cfg->trie_path);

    return 0;
}