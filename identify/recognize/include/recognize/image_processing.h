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
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
//Eigen头文件必须在opencv2/core/eigen.hpp前
#include<Eigen/Core>

#include <opencv4/opencv2/opencv.hpp>
#include "openvino/modelManager.h"
#include "../DigitalRecognition/InferResult.h"


#define LABEL_NUM 4
using namespace std;
using namespace cv;


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
    int image_threshold(const Mat& srcImg);
    int tool_tohsv(const Mat& Img);

};
const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}
#endif //SRC_IMAGE_PROCESSING_H
