//
// Created by rfeldhans on 30.09.18.
//

#include "../include/ringbuffer.h"


namespace ringbuffer{

    Ringbuffer::Ringbuffer(boost::circular_buffer<jack_default_audio_sample_t>::size_type size):
        buffer(size)
    {
        this->size = size;
    }

    void Ringbuffer::push(jack_default_audio_sample_t* audio, jack_nframes_t amount){
        boost::mutex::scoped_lock lock(mutex_lock);
        for(int i = 0; i < amount;i++){
            // push every jack sample individually... oh boy the performance!
            buffer.push_front(audio[i]);
        }
        lock.unlock();
    }

    jack_nframes_t Ringbuffer::pop(jack_default_audio_sample_t* audio, jack_nframes_t amount){
        boost::mutex::scoped_lock lock(mutex_lock);

        jack_nframes_t actual_num_of_returned_samples = 0;
        if(buffer.size() < amount){
            actual_num_of_returned_samples = (jack_nframes_t) buffer.size();
        } else{
            actual_num_of_returned_samples = amount;
        }
        audio = (jack_default_audio_sample_t*) malloc(sizeof(jack_default_audio_sample_t)* actual_num_of_returned_samples);

        jack_nframes_t running_variable = actual_num_of_returned_samples;
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

        lock.unlock();
        return actual_num_of_returned_samples;
    }
}
