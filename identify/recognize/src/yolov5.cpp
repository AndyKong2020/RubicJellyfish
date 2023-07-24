//
// Created by nuaa on 23-7-2.
//
#include <opencv2/dnn.hpp>
#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include "recognize/yolov5.h"

using namespace std;

const float SCORE_THRESHOLD = 0.2;
const float NMS_THRESHOLD = 0.4;
const float CONFIDENCE_THRESHOLD = 0.4;

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


Resize resize_and_pad(cv::Mat& img, cv::Size new_shape) {
    float width = img.cols;
    float height = img.rows;
    float r = float(new_shape.width / max(width, height));
    int new_unpadW = int(round(width * r));
    int new_unpadH = int(round(height * r));
    Resize resize;
    cv::resize(img, resize.resized_image, cv::Size(new_unpadW, new_unpadH), 0, 0, cv::INTER_AREA);

    resize.dw = abs(new_shape.width - new_unpadW);
    resize.dh = abs(new_shape.height - new_unpadH);
    cv::Scalar color = cv::Scalar(100, 100, 100);
    cv::copyMakeBorder(resize.resized_image, resize.resized_image, 0, resize.dh, 0, resize.dw, cv::BORDER_CONSTANT, color);

    return resize;
}

int yolov5_identify(cv::Mat _image,yolo _yolov5){

    cv::Mat img = _image;
    Resize res = resize_and_pad(img, cv::Size(640, 480));

    float *input_data = (float *) res.resized_image.data;
    ov::Tensor input_tensor = ov::Tensor(_yolov5.compiled_model.input().get_element_type(), _yolov5.compiled_model.input().get_shape(), input_data);



    ov::InferRequest infer_request = _yolov5.compiled_model.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();



    const ov::Tensor &output_tensor = infer_request.get_output_tensor();
    ov::Shape output_shape = output_tensor.get_shape();
    float *detections = output_tensor.data<float>();



    std::vector<cv::Rect> boxes;
    vector<int> class_ids;
    vector<float> confidences;

    for (int i = 0; i < output_shape[1]; i++){
        float *detection = &detections[i * output_shape[2]];

        float confidence = detection[4];
        if (confidence >= CONFIDENCE_THRESHOLD){
            float *classes_scores = &detection[5];
            cv::Mat scores(1, output_shape[2] - 5, CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

            if (max_class_score > SCORE_THRESHOLD){

                confidences.push_back(confidence);

                class_ids.push_back(class_id.x);

                float x = detection[0];
                float y = detection[1];
                float w = detection[2];
                float h = detection[3];

                float xmin = x - (w / 2);
                float ymin = y - (h / 2);

                boxes.push_back(cv::Rect(xmin, ymin, w, h));
            }
        }
    }
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
    std::vector<Detection> output;
    for (int i = 0; i < nms_result.size(); i++)
    {
        Detection result;
        int idx = nms_result[i];
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);
    }


    for (int i = 0; i < output.size(); i++)
    {
        auto detection = output[i];
        auto box = detection.box;
        auto classId = detection.class_id;
        auto confidence = detection.confidence;
        float rx = (float)img.cols / (float)(res.resized_image.cols - res.dw);
        float ry = (float)img.rows / (float)(res.resized_image.rows - res.dh);
        box.x = rx * box.x;
        box.y = ry * box.y;
        box.width = rx * box.width;
        box.height = ry * box.height;
        cout << "Bbox" << i + 1 << ": Class: " << classId << " "
             << "Confidence: " << confidence << " Scaled coords: [ "
             << "cx: " << (float)(box.x + (box.width / 2)) / img.cols << ", "
             << "cy: " << (float)(box.y + (box.height / 2)) / img.rows << ", "
             << "w: " << (float)box.width / img.cols << ", "
             << "h: " << (float)box.height / img.rows << " ]" << endl;
        float xmax = box.x + box.width;
        float ymax = box.y + box.height;
        cv::rectangle(img, cv::Point(box.x, box.y), cv::Point(xmax, ymax), cv::Scalar(0, 255, 0), 3);
        cv::rectangle(img, cv::Point(box.x, box.y - 20), cv::Point(xmax, box.y), cv::Scalar(0, 255, 0), cv::FILLED);
        cv::imshow("result",img);
        cv::waitKey(1);
        cv::putText(img, std::to_string(classId), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
    cv::imwrite("./detection_cpp.png", img);

    return 0;
}