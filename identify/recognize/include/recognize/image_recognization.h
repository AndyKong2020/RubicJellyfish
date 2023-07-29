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
    uint8_t img_x;
    uint8_t img_y;
    float height;
    float depth;
    float plane_depth;
    void setDepth(const uint8_t img_x,const uint8_t img_y,const float depth);

};
#endif //SRC_IMAGE_RECOGNIZATION_H
