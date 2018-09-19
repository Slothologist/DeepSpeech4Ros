//
// Created by rfeldhans on 17.09.18.
// Taken from https://github.com/mozilla/DeepSpeech/blob/v0.1.1/native_client/client.cc
//

#ifndef DEEPSPEECH_4_ROS_DSSTT_H
#define DEEPSPEECH_4_ROS_DSSTT_H

#include "deepspeech.h"

namespace DsSTT {

    typedef struct {
        const char* string;
        double cpu_time_overall;
    } ds_result;

    ds_result
    LocalDsSTT(ModelState* aCtx, const short* aBuffer, size_t aBufferSize,
               int aSampleRate)
    {
        ds_result res = {0};

        clock_t ds_start_time = clock();

        res.string = DS_SpeechToText(aCtx, aBuffer, aBufferSize, aSampleRate);

        clock_t ds_end_infer = clock();

        res.cpu_time_overall =
                ((double) (ds_end_infer - ds_start_time)) / CLOCKS_PER_SEC;

        return res;
    }

}//namespace

#endif //DEEPSPEECH_4_ROS_DSSTT_H
