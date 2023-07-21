//
// Created by bismarck on 23-3-13.
//

#include <iostream>
#include <functional>
#include <chrono>

#include <opencv2/opencv.hpp>
#include "infer.h"

#define ANKERL_NANOBENCH_IMPLEMENT
#include "nanobench.h"

using namespace std::chrono_literals;
ModelManager modelManager{};

std::function<void()> asyncFunctionCreator(int batch, cv::Mat& img) {
    return [batch, &img](){
        auto* futures = new InferResultAsync[batch];
        for (int i = 0; i < batch; i++) {
            futures[i] = modelManager.infer_async(img);
        }
        for (int i = 0; i < batch; i++) {
            auto res = futures[i].get();
        }
        delete[] futures;
    };
}


int main() {
    cv::Mat img = cv::imread("../test.png");
    modelManager.init();
    InferResult res{};

    auto bench = ankerl::nanobench::Bench().batch(1).minEpochIterations(100).timeUnit(1us, "us");
    bench.run("Sync", [&res, &img] {
        res = modelManager.infer_sync(img);
    });
    for (int i = 1; i < 6; i++) {
        bench.run(std::string("Async batch ") + std::to_string(i), asyncFunctionCreator(i, img));
    }

    std::cout << "Id: " << res.id << std::endl << "Confidence: " << res.confidence << std::endl;
}
