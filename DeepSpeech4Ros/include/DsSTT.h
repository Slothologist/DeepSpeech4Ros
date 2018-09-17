//
// Created by rfeldhans on 17.09.18.
// Taken from https://github.com/mozilla/DeepSpeech/blob/v0.1.1/native_client/client.cc
//

#ifndef DEEPSPEECH_4_ROS_DSSTT_H
#define DEEPSPEECH_4_ROS_DSSTT_H

#include "deepspeech.h"

namespace DsSTT {

struct ds_result {
    char* string;
    double cpu_time_overall;
    double cpu_time_mfcc;
    double cpu_time_infer;
};

// DsSTT() instrumented
struct ds_result* LocalDsSTT(DeepSpeech::Model *aCtx, const short *aBuffer, size_t aBufferSize,
           int aSampleRate){
            float *mfcc;
            struct ds_result *res = (struct ds_result *) malloc(sizeof(struct ds_result));
            if (!res) {
                return NULL;
            }

            clock_t ds_start_time = clock();
            clock_t ds_end_mfcc = 0, ds_end_infer = 0;

            int n_frames = 0;
            aCtx->getInputVector(aBuffer, aBufferSize, aSampleRate, &mfcc, &n_frames);
            ds_end_mfcc = clock();

            res->string = aCtx->infer(mfcc, n_frames);
            ds_end_infer = clock();

            free(mfcc);

            res->cpu_time_overall =
                    ((double) (ds_end_infer - ds_start_time)) / CLOCKS_PER_SEC;
            res->cpu_time_mfcc =
                    ((double) (ds_end_mfcc - ds_start_time)) / CLOCKS_PER_SEC;
            res->cpu_time_infer =
                    ((double) (ds_end_infer - ds_end_mfcc)) / CLOCKS_PER_SEC;

            return res;
        }

}//namespace

#endif //DEEPSPEECH_4_ROS_DSSTT_H
