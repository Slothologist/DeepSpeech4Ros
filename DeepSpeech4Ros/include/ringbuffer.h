//
// Created by rfeldhans on 30.09.18.
//

#ifndef DEEPSPEECH_4_ROS_RINGBUFFER_H
#define DEEPSPEECH_4_ROS_RINGBUFFER_H

#include <jack/jack.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>


namespace ringbuffer{

    /**
     * A ringbuffer to safe recent audio received from jack. Has the following requirements:
     * - threadsafe
     * - old data can always be overwritten
     * -
     * - needs to emit a configurable amount of data
     *
     * implementation uses a underlying boost circular buffer
     */
class Ringbuffer{

public:

    explicit Ringbuffer(boost::circular_buffer<jack_default_audio_sample_t>::size_type size);

    /**
     * writes *amount* of jack_default_audio_sample_t to the ringbuffer
     * @param audio
     */
    void push(jack_default_audio_sample_t* audio, jack_nframes_t amount);

    /**
     * reads the last *amount* of jack_default_audio_sample_t to the ringbuffer. wipes the ringbuffer afterwards.
     * @param audio
     * @param amount
     * @return the amount of jack_default_audio_sample_t actually returned. should be equal to amount, but could be
     * less, if for example
     */
    jack_nframes_t pop(jack_default_audio_sample_t* audio, jack_nframes_t amount);

private:
    // the underlying circular_buffer
    boost::circular_buffer<jack_default_audio_sample_t> buffer;

    // the size of the underlying boost::circular_buffer and thus the ringbuffer
    size_t size;

    // mutex lock for thread safety
    boost::mutex mutex_lock;


};

}

#endif //DEEPSPEECH_4_ROS_RINGBUFFER_H
