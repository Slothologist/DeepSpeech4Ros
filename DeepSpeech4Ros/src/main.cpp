
// esiaf include
#include <esiaf_ros/esiaf_ros.h>
#include <esiaf_ros/SpeechInfo.h>
#include <esiaf_ros/SpeechHypothesis.h>

// deepspeech include
#include <deepspeech/deepspeech.h>

//std includes

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
ringbuffer::Ringbuffer* ring_buffer;

// ros node handle and publisher
ros::NodeHandle* ros_node_handle;
ros::Publisher speechPublisher;

// esiaf handle
esiaf_ros::esiaf_handle *eh;

// callback function for when new audio is availble
boost::function<void(const std::vector<int8_t> &, const esiaf_ros::RecordingTimeStamps &)> simple_esiaf_callback;
void esiaf_handler(const std::vector<int8_t> &signal, const esiaf_ros::RecordingTimeStamps & timeStamps){ simple_esiaf_callback(signal, timeStamps); };

// callback function for when a vad has finished detecting speech
boost::function<void()> esiaf_vad_callback;
void esiaf_vad_handler(){ esiaf_vad_callback(); };

// ros times to keep track of the timeframe of the buffer and thus the speech signal
ros::Time buffer_start_time = ros::TIME_MIN;
ros::Time buffer_end_time = ros::TIME_MIN;

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
    speechPublisher = ros_node_handle->advertise<esiaf_ros::SpeechInfo>(cfg.esiaf_speech_topic, 1000);

    ROS_INFO("ROS initialized...");
}

void initialize_esiaf(){

    ROS_INFO("starting esiaf initialisation...");
    eh = esiaf_ros::initialize_esiaf(ros_node_handle, esiaf_ros::NodeDesignation::SpeechRec);

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

    simple_esiaf_callback = [&](const std::vector<int8_t> &signal,
                                const esiaf_ros::RecordingTimeStamps &timeStamps){
        ROS_INFO("esiaf callback");
        int16_t* signal_in_16bit = (int16_t*) signal.data();
        ring_buffer->push(signal_in_16bit, signal.size()/(sizeof(int16_t)/sizeof(int8_t)));
        buffer_end_time = timeStamps.finish;
        buffer_start_time = timeStamps.start;
    };
    esiaf_ros::add_input_topic(eh, topicInfo, esiaf_handler);


    esiaf_vad_callback = [&](){
        ROS_INFO("vad callback");

        size_t deepspeech_buffer_size = ring_buffer->getSize();
        int16_t* deepspeech_buffer = (int16_t*) malloc(deepspeech_buffer_size * sizeof * deepspeech_buffer);
        ring_buffer->pop(deepspeech_buffer, deepspeech_buffer_size);


        // aquire result
        DsSTT::ds_result result = DsSTT::LocalDsSTT(model, (const short*)deepspeech_buffer,
                                                    deepspeech_buffer_size / 2, 16000);
        //free(deepspeech_buffer);

        // print out result
        ROS_INFO("cpu_time_overall=%.05f",
                 result.cpu_time_overall);
        if (result.string) {
            ROS_INFO("Result: %s", result.string);
        } else{
            ROS_INFO("No Result!");
            return false;
        }

        // assemble output
        esiaf_ros::SpeechInfo info;
        esiaf_ros::SpeechHypothesis hypo;
        hypo.probability = 1.0;
        hypo.recognizedSpeech = result.string;

        std::vector<esiaf_ros::SpeechHypothesis> hypvec;
        hypvec.push_back(hypo);
        info.hypotheses = hypvec;
        info.duration.start = buffer_start_time;
        info.duration.finish = buffer_end_time;

        speechPublisher.publish(info);

        buffer_start_time = ros::TIME_MIN;
        buffer_end_time = ros::TIME_MIN;

        return true;
    };
    esiaf_ros::add_vad_finished_callback(eh, topicInfo.topic, esiaf_vad_handler);

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
    ring_buffer = new ringbuffer::Ringbuffer(cfg.max_audio_length);

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
    esiaf_ros::quit_esiaf(eh);

    return 0;
}