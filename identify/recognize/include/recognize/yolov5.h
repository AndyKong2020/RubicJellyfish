//
// Created by nuaa on 23-7-4.
//

#ifndef SRC_YOLOV5_H
#define SRC_YOLOV5_H

#include <opencv2/dnn.hpp>
#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>

class yolo{
public:
    ov::Core ie;
    std::shared_ptr<ov::Model> model;
    ov::CompiledModel compiled_model;

    void readNet(const std::string& onnx){
        model = ie.read_model(onnx);
        ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
        // Specify input image format
        ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
        // Specify preprocess pipeline to input image without resizing
        ppp.input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale({255., 255., 255.});
        //  Specify model's input layout
        ppp.input().model().set_layout("NCHW");
        // Specify output results format
        ppp.output().tensor().set_element_type(ov::element::f32);
        compiled_model = ie.compile_model(model, "CPU");
    }
};
int yolov5_identify(cv::Mat _image,yolo _yolov5);
#endif //SRC_YOLOV5_H
