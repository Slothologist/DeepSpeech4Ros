
//std includes
#include <cmath>

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
#define LM_WEIGHT 1.50f
#define VALID_WORD_COUNT_WEIGHT 2.25f

#define DEEPSPEECH_SAMPLE_RATE 16000

// config for this deepspeech node
utils::config cfg;

// model used by deepspeech
ModelState* model;

// buffer and its size to temporarily save audio received from jack
ringbuffer::Ringbuffer* jack_buffer;

// jack client and input
jack_client_t *client;
jack_port_t *input_port;

// ros node handle and services
ros::NodeHandle* ros_node_handle;
ros::ServiceServer rec_speech_service = {};


/******************************************************************
 * Jack and Ros Callback functions
 ******************************************************************/

void jack_shutdown(void *arg) {
    ROS_ERROR("jack shutdown...");
    exit(1);
}

sox_format_t

int process_jack_frame(jack_nframes_t nframes, void *arg){
    // aquire jack_audio_samples
    jack_default_audio_sample_t* in = (jack_default_audio_sample_t *) jack_port_get_buffer(input_port, nframes);


    // fill the ringbuffer with the audio from jack if there was audio
    if(!utils::array_zero(in, nframes))
        jack_buffer->push(in, nframes);

    return 0;
}


bool process_sample(speech_rec_pipeline_msgs::RecognizeSpeech::Request &req,
                    speech_rec_pipeline_msgs::RecognizeSpeech::Response &res){
    // get jack sample rate
    jack_nframes_t jack_sample_rate = jack_get_sample_rate(client);


    // get length of audio requested
    jack_nframes_t requested_audio_length = (jack_nframes_t) lround(req.recognize_last_audio_in_ms * jack_sample_rate / 1000);
    ROS_DEBUG("requested %d audio frames", requested_audio_length);

    // get audio from ringbuffer
    jack_default_audio_sample_t* jack_sample = new jack_default_audio_sample_t[requested_audio_length];
    int actual_audio_length = jack_buffer->pop(jack_sample, requested_audio_length);
    if (requested_audio_length != actual_audio_length){
        ROS_INFO("requested_audio_length (%d) and actual_audio_length (%d) differ, using smaller actual_audio_length!",
                 requested_audio_length, actual_audio_length);
    }

    size_t deepspeech_buffer_size = (size_t) lround(actual_audio_length * DEEPSPEECH_SAMPLE_RATE / jack_sample_rate);
    short *deepspeech_buffer = new short[deepspeech_buffer_size];
    size_t deepspeech_buffer_written = 0;

    // resample call
    resampling::resample_jack_to_deepspeech(jack_sample,
                                            (size_t) actual_audio_length,
                                            client,
                                            deepspeech_buffer,
                                            deepspeech_buffer_size,
                                            deepspeech_buffer_written,
                                            DEEPSPEECH_SAMPLE_RATE);
    delete(jack_sample);
    ROS_DEBUG("Deepspeech buffer now with length %lu", deepspeech_buffer_written);

    // aquire result
    DsSTT::ds_result result = DsSTT::LocalDsSTT(model, (const short*)deepspeech_buffer,
                                                deepspeech_buffer_size / 2, cfg.sample_rate);
    delete(deepspeech_buffer);

    // print out result
    ROS_INFO("cpu_time_overall=%.05f",
             result.cpu_time_overall);
    if (result.string) {
        ROS_INFO("Result: %s", result.string);
    } else{
        ROS_INFO("No Result!");
        return false;
    }

    // assemble response
    res.recognized_speech = result.string;

    return true;
}

/******************************************************************
 * Initialization Methods
 ******************************************************************/


void initialize_deepspeech(){
    int status = DS_CreateModel(cfg.model_path.c_str(), N_CEP, N_CONTEXT, cfg.alphabet_path.c_str(), BEAM_WIDTH, &model);
    if (status != 0) {
        ROS_ERROR("Could not create model.");
        exit(1);
    }
    status = DS_EnableDecoderWithLM(model,
                                    cfg.alphabet_path.c_str(),
                                    cfg.lm_path.c_str(),
                                    cfg.trie_path.c_str(),
                                    LM_WEIGHT,
                                    VALID_WORD_COUNT_WEIGHT);
    if (status != 0) {
        ROS_ERROR("Could not enable CTC decoder with LM.");
        exit(1);
    }
    ROS_INFO("Deepspeech initialized...");
}

void initialize_jack(){

    auto jack_server_name = (int) JackNullOption;
    const char *client_name = cfg.jack_client_name.c_str();
    const char *server_name = cfg.jack_server_name == "default" ? nullptr : cfg.jack_server_name.c_str();
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
    // create buffer for holding sound longer than one jack process_jack_frame() call
    jack_buffer = new ringbuffer::Ringbuffer((unsigned long) (cfg.max_audio_length/1000 * jack_get_sample_rate(client)));

    // tell the JACK server to call `process_jack_frame()' whenever there is work to be done and jack_shutdown() when jack shuts downs
    jack_set_process_callback(client, process_jack_frame, nullptr);
    jack_on_shutdown(client, jack_shutdown, nullptr);

    // register the input port
    input_port = jack_port_register(client, cfg.jack_input_port_name.c_str(), JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);

    if (jack_activate(client)) {
        ROS_ERROR("cannot activate client");
        exit(1);
    }
    ROS_INFO("Jack initialized...");
}

void initialize_ros(int argc, char * *argv){
    ros::init(argc, argv, cfg.ros_node_name);
    ros_node_handle = new ros::NodeHandle;

    rec_speech_service = ros_node_handle->advertiseService(cfg.ros_recognize_speech_topic, process_sample);
    ROS_INFO("ROS initialized...");
}

/********************************************************************************
 * Initiaization Methods End
 ********************************************************************************/



int main(int argc, char *argv[]) {

    // parse config
    read_config(cfg, argv[1]);

    // initialize all the things!
    initialize_deepspeech();
    initialize_ros(argc, argv);
    initialize_jack();

    ROS_INFO("Node ready and rockin'");

    // main loop
    while(ros::ok()){
        ros::spin();
    }

    // destructing stuff and freeing buffers
    DS_DestroyModel(model);
    jack_client_close(client);

    return 0;
}