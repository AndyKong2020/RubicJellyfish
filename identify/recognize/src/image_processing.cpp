//
// Created by nuaa on 23-7-2.
//
#include "recognize/image_processing.h"
//用作对原图像进行处理，讲处理后的图像送入神经网络识别端。可保存样本图片，记得修改初始计数值
void image_processing::Picture_process(image_processing &image) {
    Mat img;

//    for(int _row=0; _row < img_gray.rows; _row++){
//        for(int _col=0; _col < img_gray.cols; _col++){
//            if(img_gray.at<uchar>(_row,_col) < otsuThreshold(img_gray))
//                img_gray.at<uchar>(_row,_col) = 0;
//        }
//    }
//    // 低通滤波
//    static float thresh_ = threshold;
//    if (::isnan(threshold) || ::isinf(threshold)) {
//        threshold = thresh_;
//    } else {
//        threshold = a * thresh_ + (1 - a) * threshold;
//    }
//
//    if (threshold > maxThresh) {
//        threshold = maxThresh;
//    }
//    img_gray *= threshold;
//        NUM_bgr = img_gray.clone();
        // 保存装甲板图片用于训练神经网络
//        static unsigned int noooo = 121653;
//        cv::imwrite(std::string("/home/nuaa/shootttt/") + std::to_string(noooo) + ".png", NUM_bgr);
//        noooo++;
    image.typeCall = modelManager.infer_async(img);
}