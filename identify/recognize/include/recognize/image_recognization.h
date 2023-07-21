//
// Created by nuaa on 23-7-2.
//

#ifndef SRC_IMAGE_RECOGNIZATION_H
#define SRC_IMAGE_RECOGNIZATION_H
#include <iostream>
#include <vector>
#include<map>
#include <string>
#include <fstream>
#include <numeric>

//Eigen头文件必须在opencv2/core/eigen.hpp前
#include<Eigen/Core>

#include <opencv4/opencv2/opencv.hpp>
#include "openvino/modelManager.h"


#define LABEL_NUM 4
using namespace std;
using namespace cv;


class image_recognization {
public :

    Mat cameraMatrix;

    int tracked_num = 0;
    ModelManager modelManager;
    void Picture_process(image_recognization &image);
    Mat img_gray, img_bgr, img_hsv, img_h, led_mask, img_out;
    Mat target_coor;
    Mat img_show, ROI_bgr, coordinate, NUM_bgr, Image;
    Point2f target;

    map<int, int> HP;


};
#endif //SRC_IMAGE_RECOGNIZATION_H
