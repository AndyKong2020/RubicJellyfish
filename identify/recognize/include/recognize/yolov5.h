//
// Created by nuaa on 23-7-4.
//

#ifndef SRC_YOLOV5_H
#define SRC_YOLOV5_H

#include <opencv2/dnn.hpp>
#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
struct Detection
{
    int class_id;
    float confidence;
    cv::Rect box;
};


struct Resize
{
    cv::Mat resized_image;
    int dw;
    int dh;
};
ov::CompiledModel yolo_init(const std::string& xml);
std::vector<Detection> yolov5_identify(cv::Mat _image,ov::CompiledModel compiled_model);
#endif //SRC_YOLOV5_H
