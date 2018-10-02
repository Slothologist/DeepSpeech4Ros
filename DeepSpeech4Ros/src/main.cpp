
// ros includes
#include "ros/ros.h"
#include "speech_rec_pipeline_msgs/RecognizeSpeech.h"

// jack includes
#include <jack/jack.h>
#include <sox.h>

// inludes from this project
#include "../include/utils.h"
#include "../include/deepspeech.h"
#include "../include/DsSTT.h"
#include "../include/resampler.h"
#include "../include/ringbuffer.h"

/******************************************************************
 * Constants and Variables
 ******************************************************************/

// constants used by deepspeech.
#define N_CEP 26
#define N_CONTEXT 9
#define BEAM_WIDTH 500
#define LM_WEIGHT 1.75f
#define VALID_WORD_COUNT_WEIGHT 1.00f

#define DEEPSPEECH_SAMPLE_RATE 16000

// config for this deepspeech node
utils::config* cfg;

// model used by deepspeech
ModelState* model;

// buffer and its size to temporarily save audio received from jack
ringbuffer::Ringbuffer* jack_buffer;
size_t jack_buffer_size;

// jack client and input
jack_client_t *client;
jack_port_t *input_port;


/******************************************************************
 * Jack and Ros Callback functions
 ******************************************************************/


int process_jack_frame(jack_nframes_t nframes, void *arg){
    // aquire jack_audio_samples
    jack_default_audio_sample_t* in = (jack_default_audio_sample_t *) jack_port_get_buffer(input_port, nframes);

    // fill the ringbuffer with the audio from jack
    jack_buffer->push(in, nframes);

    return 0;
}


bool process_sample(speech_rec_pipeline_msgs::RecognizeSpeech::Request &req,
                    speech_rec_pipeline_msgs::RecognizeSpeech::Response &res){
    // resample audio
    size_t deepspeech_buffer_size;
    short *deepspeech_buffer = nullptr;

    // get length of audio requested
    jack_nframes_t requested_audio_length = (jack_nframes_t)req.recognize_last_audio_in_ms * DEEPSPEECH_SAMPLE_RATE/1000;

    // get audio from ringbuffer
    req.recognize_last_audio_in_ms;
    jack_default_audio_sample_t *jack_sample = nullptr;
    int actual_audio_length = jack_buffer->pop(jack_sample, requested_audio_length);
    if (requested_audio_length != actual_audio_length){
        ROS_INFO("requested_audio_length (%d) and actual_audio_length(%d) differ, using smaller actual_audio_length!",
                 requested_audio_length, actual_audio_length);
    }

    // resample call
    resampling::resample_jack_to_deepspeech(jack_sample,
                                            (size_t) actual_audio_length,
                                            client,
                                            deepspeech_buffer,
                                            &deepspeech_buffer_size,
                                            DEEPSPEECH_SAMPLE_RATE);
    free(jack_sample);

    // aquire result
    DsSTT::ds_result result = DsSTT::LocalDsSTT(model, (const short*)deepspeech_buffer,
                                                deepspeech_buffer_size / 2, cfg->sample_rate);
    free(deepspeech_buffer);

    if (result.string) {
        ROS_INFO("Result: %s\n", result.string);
    } else{
        ROS_INFO("No Result!");
    }
    ROS_INFO("cpu_time_overall=%.05f\n",
           result.cpu_time_overall);
    // assemble response
    res.recognized_speech = result.string;

    return 1;
}

/******************************************************************
 * Initialization Methods
 ******************************************************************/


void initialize_deepspeech(){
    int status = DS_CreateModel(cfg->model_path, N_CEP, N_CONTEXT, cfg->alphabet_path, BEAM_WIDTH, &model);
    if (status != 0) {
        ROS_ERROR("Could not create model.");
        exit(1);
    }
    status = DS_EnableDecoderWithLM(model,
                                    cfg->alphabet_path,
                                    cfg->lm_path,
                                    cfg->trie_path,
                                    LM_WEIGHT,
                                    VALID_WORD_COUNT_WEIGHT);
    if (status != 0) {
        ROS_ERROR("Could not enable CTC decoder with LM.");
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
        ROS_ERROR("jack_client_open() failed, "
                "status = 0x%2.0x", jack_status);
        if (jack_status & JackServerFailed) {
            ROS_ERROR("Unable to connect to JACK server");
        }
        exit(1);
    }
    if (jack_status & JackServerStarted) {
        ROS_ERROR("JACK server started");
    }
    if (jack_status & JackNameNotUnique) {
        client_name = jack_get_client_name(client);
        ROS_ERROR("unique name `%s' assigned", client_name);
    }

    // create buffer for holding sound longer than one jack process_frame() call
    jack_buffer_size = jack_get_sample_rate(client) * sizeof(jack_default_audio_sample_t) * cfg->max_audio_length/1000;
    jack_buffer = new ringbuffer::Ringbuffer((unsigned long) cfg->max_audio_length);

    /* tell the JACK server to call `process()' whenever
       there is work to be done.
    */

    jack_set_process_callback(client, process_jack_frame, nullptr);
}

void initialize_ros(int argc, char * *argv){
    ros::init(argc, argv, cfg->ros_node_name);
    ros::NodeHandle n;

    ros::ServiceServer change_config_service = n.advertiseService(cfg->ros_change_config_topic, process_sample);
}

/********************************************************************************
 * Initiaization Methods End
 ********************************************************************************/



int main(int argc, char *argv[]) {

    // parse config
    cfg = new utils::config();
    read_config(cfg, argv[1]);

    // initialize all the things!
    initialize_deepspeech();
    initialize_jack();
    initialize_ros(argc, argv);

    // main loop

    // destructing stuff and freeing buffers
    DS_DestroyModel(model);
    // freeing the config's paths is a bit ugly, but here we go
    delete(cfg->model_path);
    delete(cfg->alphabet_path);
    delete(cfg->lm_path);
    delete(cfg->trie_path);

    return 0;
}