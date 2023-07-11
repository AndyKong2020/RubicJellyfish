//
// Created by bismarck on 23-3-15.
//

#include "modelManager.h"
#include <future>

InferResult templateId2InferResult[] = {
        {1, true, 0},
        {2, false, 0},
        {3, false, 0},
        {4, false, 0},
        {5, false, 0}
};

void ModelManager::init() {
    for (int i = 0; i < 10; i++) {
        templates.push_back(cv::imread(TEMPLATES_PATH + std::to_string(i + 1) + ".jpg", cv::IMREAD_GRAYSCALE));
    }
}

InferResult ModelManager::_infer_sync(cv::Mat &&img) {
    infer_sync(img);
}

InferResult ModelManager::infer_sync(cv::Mat &img) {
    float lgt = 100.f;
    cv::Mat img_res(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
    cv::Mat img_mean = img(cv::Rect((int)(img.cols / 4.0), 0, (int)(img.cols / 2.0), img.rows));//remove LEDs
    cv::Mat img_gray = img * lgt / cv::mean(img_mean)[0];
    double r_max;
    double r_min;
    double num_max = 0.1;
    int res;
    cv::minMaxIdx(img_gray, &r_min, &r_max);
    cv::inRange(img_gray, lgt, 255, img_res);
    for (int i = 0; i < templates.size(); i++) {
        cv::Mat result;
        cv::Mat img_model;
        cv::Size newSize((int)((float) (templates[i].cols) / (float) (templates[i].rows) * (float) (img_res.rows)),
                         (int) img_res.rows);
        cv::resize(templates[i], img_model, newSize);
        cv::matchTemplate(img_model, img_res, result, cv::TM_CCOEFF_NORMED);
        cv::minMaxLoc(result, &r_min, &r_max, NULL, NULL);
        if (num_max < r_max) {
            num_max = r_max;
            res = i;
        }
    }
    if (res > 4) {
        res -= 5;
    }
    InferResult result = templateId2InferResult[res];
    result.confidence = (float)num_max;
    return result;
}

InferResultAsync ModelManager::infer_async(cv::Mat &img) {
    cv::Mat _img = img.clone();
    return InferResultAsync(std::async(
            std::launch::async,
            std::bind(&ModelManager::_infer_sync, this, std::placeholders::_1),
            std::move(_img)).share());
}
