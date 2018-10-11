//
// Created by rfeldhans on 20.09.18.
//

#include "../include/resampler.h"
#include <soxr.h>
#include <stdlib.h>

#include "ros/ros.h"

namespace resampling{




    void resample_jack_to_deepspeech(jack_default_audio_sample_t* jack_buffer,
                                     size_t jack_buffer_size,
                                     jack_client_t* jack_client,
                                     short* deepspeech_buffer,
                                     size_t deepspeech_buffer_size,
                                     size_t& deepspeech_buffer_written,
                                     int deepspeech_sample_rate){
        // get jack sample rate
        jack_nframes_t jack_sample_rate = jack_get_sample_rate(jack_client);

        // prepare output (deepspeech) buffer
        ROS_DEBUG("jack sample rate: %iu", jack_sample_rate);
        ROS_DEBUG("deepspeech sample rate: %i", deepspeech_sample_rate);
        ROS_DEBUG("jack buffer size: %lu", jack_buffer_size);

        // resample
        soxr_io_spec_t io_spec = soxr_io_spec(SOXR_FLOAT32_I, SOXR_INT16_I);
        soxr_error_t error = soxr_oneshot(jack_sample_rate, deepspeech_sample_rate, 1,                           /* Rates and # of chans. */
                                          jack_buffer, jack_buffer_size, NULL,                                   /* Input. */
                                          deepspeech_buffer, deepspeech_buffer_size, &deepspeech_buffer_written, /* Output. */
                                          &io_spec, NULL, NULL);                                                 /* soxr_io_spec_t, soxr_quality_spec_t, soxr_runtime_spec_t */

        if(error){
            ROS_INFO("Error while resampling: %s", soxr_strerror(error));
        }
        ROS_DEBUG("length deepspeech buffer: %lu", deepspeech_buffer_size);
    }

}