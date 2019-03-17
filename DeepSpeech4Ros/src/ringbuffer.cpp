//
// Created by rfeldhans on 30.09.18.
//

#include "../include/ringbuffer.h"


namespace ringbuffer{

    Ringbuffer::Ringbuffer(boost::circular_buffer<int16_t >::size_type size):
        buffer(size)
    {

    }

    void Ringbuffer::push(int16_t* audio, size_t amount){
        mutex_lock.lock();
        for(int i = 0; i < amount;i++){
            // push every sample individually... oh boy the performance!
            buffer.push_back(audio[i]);
        }
        mutex_lock.unlock();
    }

    size_t Ringbuffer::getSize() {
        return buffer.size();
    }

    void Ringbuffer::pop(int16_t* audio, size_t amount){
        mutex_lock.lock();

        size_t actual_num_of_returned_samples = 0;
        if(buffer.size() < amount){
            actual_num_of_returned_samples = buffer.size();
        } else{
            actual_num_of_returned_samples = amount;
        }

        size_t running_variable = actual_num_of_returned_samples;
        while(!buffer.empty() && running_variable){
            // basically the same problem as in push()... the performance may be very bad.
            // note that audio is written to in reverse because of buffer.back()
            audio[running_variable] = buffer.back();
            buffer.pop_back();
            running_variable--;
        }
        /**
         * Clear the buffer. Reasoning: Suppose your buffer size is equal to 10 seconds and it is written fully.
         * You then analyse the last 5 sec and thus remove them from the buffer. After writing another second of audio
         * to the buffer you then want to analyze, again, the last 5 sec. Without clearing the buffer, the buffer would
         * return the last recorded second and another 4 sec, which would have been recorded even before the audio of
         * the first analysis. This makes no sense and would also lead to clipping problems.
         */
        buffer.clear();

        mutex_lock.unlock();
    }
}
