//
// Created by bismarck on 23-3-15.
//

#ifndef DIGITALRECOGNITION_MODELMANAGER_H
#define DIGITALRECOGNITION_MODELMANAGER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "../InferResult.h"

class ModelManager {
private:
    std::vector<cv::Mat> templates;

public:
    void init();
    InferResult infer_sync(cv::Mat& img);
    InferResult _infer_sync(cv::Mat &&img);
    InferResultAsync infer_async(cv::Mat& img);
};


#endif //DIGITALRECOGNITION_MODELMANAGER_H
