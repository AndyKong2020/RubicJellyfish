#include "preprocess.h"

__global__ void preprocess_kernel(INPUT_VAR_TYPE* input, float* output, int size) {
    int position = blockDim.x * blockIdx.x + threadIdx.x;
    if (position >= size) return;
    output[position] = ((float)input[position] - MEAN) / SCALE;
}

void cuda_preprocess(INPUT_VAR_TYPE* input, float* output, int size, cudaStream_t stream) {
    int threads = 256;
    int blocks = size / (float)threads;
    preprocess_kernel<<<blocks, threads, 0, stream>>>(input, output, size);
}
