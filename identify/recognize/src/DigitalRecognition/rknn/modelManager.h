//
// Created by bismarck on 23-3-13.
//

#ifndef DIGITALRECOGNITION_MODELMANAGER_H
#define DIGITALRECOGNITION_MODELMANAGER_H

#define INIT_CACHE_SIZE 10

#include <rknn_api.h>
#include <opencv2/opencv.hpp>
#include <cstdint>
#include <mutex>
#include <atomic>
#include "../common.h"

class InferResult;
class InferResultAsync;

#define NPU_NUM 3

class ModelManager {
private:
    friend class InferResultAsync;

    rknn_context ctx[NPU_NUM];
    std::mutex ctxMtx[NPU_NUM];
    rknn_input inputs[NPU_NUM];
    rknn_output outputs[NPU_NUM];

    uint8_t* model;
    long modelSize;

    std::atomic<int> nowIdx{0};

    void preprocess(cv::Mat& img, int idx);
    InferResult postprocess(int idx);
    int getIdx();

public:
    ~ModelManager();

    void init();
    InferResult infer_sync(cv::Mat& img);
    InferResultAsync infer_async(cv::Mat& img);
};


#endif //DIGITALRECOGNITION_MODELMANAGER_H
