//
// Created by bismarck on 23-3-13.
//

#include "modelManager.h"
#include "../InferResult.h"

#include <cstring>
#include <future>

InferResult datasetId2InferResult[] = {
        {1, true,  1},
        {2, false, 1},
        {3, false, 1},
        {4, false, 1},
        {5, false, 1},
        {6, false, 1},
        {7, false, 1},
        {8, true,  1},
        {0, false, 0},
};

void ModelManager::init() {
    FILE *fp = fopen(MODEL_PATH, "rb");
    if (fp == nullptr) {
        printf("model open fail!\n");
        std::terminate();
    }
    fseek(fp, 0, SEEK_END);
    modelSize = ftell(fp);
    model = new uint8_t[modelSize];
    fseek(fp, 0, SEEK_SET);
    if (modelSize != fread(model, 1, modelSize, fp)) {
        printf("read model fail!\n");
        free(model);
        std::terminate();
    }
    if (fp) {
        fclose(fp);
    }

    int ret;
    for (int i = 0; i < NPU_NUM; i++) {
        ret = rknn_init(ctx + i, model, modelSize, 0, nullptr);
        if (ret < 0) {
            printf("rknn init fail!\n");
        }
    }

    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < NPU_NUM; i++) {
        inputs[i].index = 0;
        inputs[i].type = RKNN_TENSOR_UINT8;
        inputs[i].fmt = RKNN_TENSOR_NHWC;
        inputs[i].buf = new uint8_t[width * height];
        outputs[i].want_float = 1;

        ret = rknn_inputs_set(ctx[i], 1, inputs + i);
        if (ret < 0) {
            printf("rknn input set fail!\n");
        }
    }
    rknn_set_core_mask(ctx[0], RKNN_NPU_CORE_0);
    rknn_set_core_mask(ctx[1], RKNN_NPU_CORE_1);
    rknn_set_core_mask(ctx[2], RKNN_NPU_CORE_2);
}

void ModelManager::preprocess(cv::Mat &img, int idx) {
    if (img.channels() > 1) {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
    if (img.rows != height || img.cols != width) {
        cv::resize(img, img, cv::Size(width, height));
    }
    if (img.type() != CV_32FC1) {
        img.convertTo(img, CV_32FC1);
    }
    inputs[idx].size = width * height;
    memcpy(inputs[idx].buf, img.data, width * height);
}

InferResult ModelManager::postprocess(int idx) {
    auto shape = outputs[idx].size / 4;
    const auto *output_buffer = (float*)outputs[idx].buf;
    InferResult maxRes{255, false, 0};
    for (int i = 0; i < (int) shape - 1; i++) {
        if (maxRes.id == 255 || output_buffer[i] > maxRes.confidence) {
            maxRes = datasetId2InferResult[i];
            maxRes.confidence = output_buffer[i];
        }
    }
    rknn_outputs_release(ctx[idx], 1, outputs + idx);
    return maxRes;
}

InferResult ModelManager::infer_sync(cv::Mat &img) {
    int idx = getIdx();
    ctxMtx[idx].lock();
    preprocess(img, idx);
    if (rknn_run(ctx[idx], nullptr) < 0) {
        printf("rknn run fail!");
        return {};
    }
    if (rknn_outputs_get(ctx[idx], 1, outputs+idx, nullptr) < 0) {
        printf("rknn get output fail!");
        return {};
    }
    auto res = postprocess(idx);
    ctxMtx[idx].unlock();
    return res;
}

InferResultAsync ModelManager::infer_async(cv::Mat &img) {
    int idx = getIdx();
    ctxMtx[idx].lock();
    preprocess(img, idx);
    return InferResultAsync(std::async([this, idx]() -> InferResult {
        if (rknn_run(ctx[idx], nullptr) < 0) {
            printf("rknn run fail!");
            return {};
        }
        if (rknn_outputs_get(ctx[idx], 1, outputs+idx, nullptr) < 0) {
            printf("rknn get output fail!");
            return {};
        }
        auto res = postprocess(idx);
        ctxMtx[idx].unlock();
        return res;
    }).share());
}

int ModelManager::getIdx() {
    nowIdx++;
    if (nowIdx >= NPU_NUM) {
        nowIdx -= NPU_NUM;
    }
    return nowIdx;
}

ModelManager::~ModelManager() {
    for(int i = 0; i < NPU_NUM; i++) {
        delete[] (uint8_t*)inputs[i].buf;
        rknn_destroy(ctx[i]);
        delete[] model;
    }
}
