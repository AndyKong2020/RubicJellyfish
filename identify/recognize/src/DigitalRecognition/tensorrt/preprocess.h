//
// Created by bismarck on 23-3-18.
//

#ifndef DIGITALRECOGNITION_PREPROCESS_H
#define DIGITALRECOGNITION_PREPROCESS_H

#include <cuda_runtime_api.h>

#define INPUT_MAT_TYPE CV_8UC1
#define INPUT_VAR_TYPE unsigned char
#define MEAN 116.28f
#define SCALE 57.12f

void cuda_preprocess(INPUT_VAR_TYPE* input, float* output, int size, cudaStream_t stream);

#endif //DIGITALRECOGNITION_PREPROCESS_H
