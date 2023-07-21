//
// Created by bismarck on 23-3-13.
//

#ifndef DIGITALRECOGNITION_MODELMANAGER_H
#define DIGITALRECOGNITION_MODELMANAGER_H

#define INIT_CACHE_SIZE 10

#include <cstdint>
#include <atomic>
#include <deque>
#include <NvInfer.h>
#include <opencv2/opencv.hpp>

#include "../InferResult.h"
#include "../common.h"
#include "preprocess.h"


#define CHECK(status) \
    do\
    {\
        auto ret = (status);\
        if (ret != 0)\
        {\
            std::cerr << "Cuda failure: " << ret << std::endl;\
            abort();\
        }\
    } while (0)


class ModelManager {
private:
    friend class InferResultAsync;
    friend class InferRequest;

    struct binding {
        float* data[2];
    };

    std::deque<std::atomic<bool>> memoryUsing;
    std::deque<INPUT_VAR_TYPE*> input_p;
    std::deque<binding> binding_p;

    CUdevice device;
    nvinfer1::IRuntime* runtime{nullptr};
    nvinfer1::ICudaEngine* engine{nullptr};
    std::deque<nvinfer1::IExecutionContext*> context_p;

    static InferResult postprocess(const float* res);
    void preprocess(cv::Mat& img, int idx, cudaStream_t stream);
    void scaleMemoryPoll(int size=INIT_CACHE_SIZE);
    int getMemorySlot();

public:
    void init();
    ~ModelManager();
    InferResult infer_sync(cv::Mat& img);
    InferResultAsync infer_async(cv::Mat&& img);
    InferResultAsync infer_async(cv::Mat& img);
};


#endif //DIGITALRECOGNITION_MODELMANAGER_H
