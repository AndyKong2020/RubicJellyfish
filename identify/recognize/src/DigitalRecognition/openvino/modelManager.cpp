//
// Created by bismarck on 23-3-13.
//

#include "modelManager.h"
#include "../InferResult.h"

#include <cstring>

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
    model = core.compile_model(MODEL_PATH, "AUTO", {
            { ov::hint::performance_mode.name(), ov::hint::PerformanceMode::THROUGHPUT }
    });
}

void ModelManager::preprocess(cv::Mat &img, ov::InferRequest& req) {
    auto input_port = model.input();
    if (img.channels() > 1) {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
    if (img.rows != height || img.cols != width) {
        cv::resize(img, img, cv::Size(width, height));
    }
    if (img.type() != CV_32FC1) {
        img.convertTo(img, CV_32FC1);
    }
    auto input = req.get_tensor(input_port);
    std::cout << "==========\nsize: " << input.get_byte_size() << "\n===========\n";
    memcpy(input.data(), img.ptr(), width * height * 4);
}

InferResult ModelManager::postprocess(const ov::Tensor &output) {
    auto shape = output.get_shape();
    const auto *output_buffer = output.data<const float>();
    InferResult maxRes{0, false, 0.2};
    for (int i = 0; i < (int) shape[1] - 1; i++) {
        if (output_buffer[i] > maxRes.confidence) {
            maxRes = datasetId2InferResult[i];
            maxRes.confidence = output_buffer[i];
        }
    }
    return maxRes;
}

InferResult ModelManager::infer_sync(cv::Mat &img) {
    ov::InferRequest infer_request = model.create_infer_request();
    preprocess(img, infer_request);
    infer_request.infer();
    auto output = infer_request.get_tensor(output_name);
    return postprocess(output);
}

InferResultAsync ModelManager::infer_async(cv::Mat &img) {
    ov::InferRequest infer_request = model.create_infer_request();
    preprocess(img, infer_request);
    infer_request.start_async();
    return InferResultAsync{std::move(infer_request)};
}

