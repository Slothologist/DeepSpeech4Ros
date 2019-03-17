//
// Created by rfeldhans on 30.09.18.
//

#ifndef DEEPSPEECH_4_ROS_RINGBUFFER_H
#define DEEPSPEECH_4_ROS_RINGBUFFER_H

#include <boost/circular_buffer.hpp>
#include <mutex>


namespace ringbuffer{

    /**
     * A ringbuffer to safe recent audio received from esiaf. Has the following requirements:
     * - threadsafe
     * - old data can always be overwritten
     * -
     * - needs to emit a configurable amount of data
     *
     * implementation uses a underlying boost circular buffer
     */
class Ringbuffer{

public:

    explicit Ringbuffer(boost::circular_buffer<int16_t>::size_type size);

    /**
     * writes *amount* of floats to the ringbuffer
     * @param audio
     */
    void push(int16_t* audio, size_t amount);

    /**
     * reads the last *amount* of floats to the ringbuffer. wipes the ringbuffer afterwards.
     * @param audio
     * @param amount
     */
    void pop(int16_t* audio, size_t amount);

    size_t getSize();

private:
    // the underlying circular_buffer
    boost::circular_buffer<int16_t> buffer;

    // mutex lock for thread safety
    std::mutex mutex_lock;


};

}

#endif //DEEPSPEECH_4_ROS_RINGBUFFER_H
