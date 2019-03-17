
// esiaf include
#include <esiaf_ros/esiaf_ros.h>

// deepspeech include
#include <deepspeech/deepspeech.h>

//std includes
#include <cmath>

// ros includes
#include "ros/ros.h"

// inludes from this project
#include "../include/utils.h"
#include "../include/DsSTT.h"
#include "../include/ringbuffer.h"

/******************************************************************
 * Constants and Variables
 ******************************************************************/

// constants used by deepspeech.
#define N_CEP 26
#define N_CONTEXT 9
#define BEAM_WIDTH 500
#define LM_ALPHA 0.75f
#define LM_BETA 1.85f

// config for this deepspeech node
utils::config cfg;

// model used by deepspeech
ModelState* model;

// buffer and its size to temporarily save audio received from jack
ringbuffer::Ringbuffer* jack_buffer;

// ros node handle and services
ros::NodeHandle* ros_node_handle;

boost::function<void(const std::vector<int8_t> &, const esiaf_ros::RecordingTimeStamps &)> simple_esiaf_callback;
void esiaf_handler(const std::vector<int8_t> &signal, const esiaf_ros::RecordingTimeStamps & timeStamps){ simple_esiaf_callback(signal, timeStamps); };


/*
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
 */

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
                                    LM_ALPHA,
                                    LM_BETA);
    if (status != 0) {
        ROS_ERROR("Could not enable CTC decoder with LM.");
        exit(1);
    }
    ROS_INFO("Deepspeech initialized...");
}

void initialize_ros(int argc, char * *argv){
    ros::init(argc, argv, cfg.ros_node_name);
    ros_node_handle = new ros::NodeHandle;

    ROS_INFO("ROS initialized...");
}

void initialize_esiaf(){

    ROS_INFO("starting esiaf initialisation...");
    esiaf_ros::esiaf_handle *eh = esiaf_ros::initialize_esiaf(ros_node_handle, esiaf_ros::NodeDesignation::Other);

    //create format for output topic
    esiaf_ros::EsiafAudioTopicInfo topicInfo;

    // format needed by deepspeech
    esiaf_ros::EsiafAudioFormat allowedFormat;
    allowedFormat.rate = esiaf_ros::Rate::RATE_16000;
    allowedFormat.channels = 1;
    allowedFormat.bitrate = esiaf_ros::Bitrate::BIT_INT_16_SIGNED;
    allowedFormat.endian = esiaf_ros::Endian::LittleEndian;

    topicInfo.allowedFormat = allowedFormat;
    topicInfo.topic = cfg.esiaf_input_topic;

    // notify esiaf about the output topic
    ROS_INFO("adding input topic....");

    float scaling_factor = sizeof(int8_t) / sizeof(int16_t);

    simple_esiaf_callback = [&](const std::vector<int8_t> &signal,
                                const esiaf_ros::RecordingTimeStamps &timeStamps){

        int16_t* signal_in_16bit = (int16_t*) signal.data();
        jack_buffer->push(signal_in_16bit, (size_t) scaling_factor * signal.size());
    };

    esiaf_ros::add_input_topic(eh, topicInfo, esiaf_handler);

    // start esiaf
    ROS_INFO("starting esiaf...");

    esiaf_ros::start_esiaf(eh);
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
    initialize_esiaf();

    ROS_INFO("Node ready and rockin'");

    // main loop
    while(ros::ok()){
        ros::spin();
    }

    // destructing stuff and freeing buffers
    DS_DestroyModel(model);

    return 0;
}