//
// Created by rfeldhans on 20.09.18.
//

#ifndef DEEPSPEECH_4_ROS_RESAMPLER_H
#define DEEPSPEECH_4_ROS_RESAMPLER_H

#include <jack/jack.h>

namespace resampling{

    /**
     * resamples audio gotten from jack for deepspeech use
     */
    void resample_jack_to_deepspeech(jack_default_audio_sample_t* jack_buffer,
                                     size_t jack_buffer_size,
                                     jack_client_t* jack_client,
                                     short* deepspeech_buffer,
                                     size_t deepspeech_buffer_size,
                                     size_t& deepspeech_buffer_written,
                                     int deepspeech_sample_rate);

}

#endif //DEEPSPEECH_4_ROS_RESAMPLER_H
