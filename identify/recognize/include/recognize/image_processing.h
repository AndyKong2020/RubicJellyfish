//
// Created by nuaa on 23-7-2.
//

#ifndef SRC_IMAGE_PROCESSING_H
#define SRC_IMAGE_PROCESSING_H
#include <iostream>
#include <vector>
#include<map>
#include <string>
#include <fstream>
#include <numeric>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
//Eigen头文件必须在opencv2/core/eigen.hpp前
#include<Eigen/Core>
#include <zbar.h>
#include "recognize/image.h"
#include <opencv4/opencv2/opencv.hpp>
#include "openvino/modelManager.h"
#include "../DigitalRecognition/InferResult.h"


#define LABEL_NUM 4
using namespace std;
using namespace cv;

typedef struct
{
    string data;
    string type;
    vector<Point> location;
} decodedObject;

class image_processing {
public :

    Mat cameraMatrix;

    int tracked_num = 0;
    ModelManager modelManager;
    InferResultAsync typeCall;
    void Picture_process(image_processing &image);
    Mat img_gray, img_bgr, img_hsv, img_h, led_mask, img_out;
    Mat target_coor;
    Mat img_show, ROI_bgr, coordinate, NUM_bgr, Image;
    Point2f target;
    Point point;
    map<int, int> HP;
    RotatedRect image_threshold(const Mat& srcImg);
    int tool_tohsv(const Mat& Img);
    uint8_t l1 = 0,l2 = 0,l3 = 0;
    uint8_t h1 = 0 ,h2 = 0 ,h3 = 0;
    bool image_check(RotatedRect &target,const uint8_t &minsize,const uint8_t &maxsize,const uint8_t &task_id,const uint8_t &num,cv::Rect &res);
    float d_res = 0;
    int minsize = 0;
    int maxsize = 5000;
    int element = 5;
    int kind = 0;
};
const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
void decode_wechat(Mat& im);
bool decode_dis(cv::Mat &im);
#endif //SRC_IMAGE_PROCESSING_H
