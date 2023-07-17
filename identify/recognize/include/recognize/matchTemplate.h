//
// Created by nuaa on 23-7-17.
//

#ifndef SRC_MATCHTEMPLATE_H
#define SRC_MATCHTEMPLATE_H
#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

//金字塔层级
#define DOWN_LEVEL 3
//容许度
#define TOLERANCE_MAX 0.98

//针对于多目标，多角度
struct MatchResult
{
    float Score;
    cv::RotatedRect ROI;

    MatchResult() {};
    MatchResult(float _score, cv::RotatedRect _roi) :Score(_score), ROI(_roi) {};
};

class EasyTemplate
{
public:
    EasyTemplate();
    ~EasyTemplate();
public:
    //创建模板图像,
    //Img:模板图片；pryDown：是否使用低采样：低采样会加快检测、降低分数
    bool Mark(cv::Mat &Img,bool pryDown =false);

    //进行模板匹配，要求图像无缩放(或者和模板同步缩放)
    //Img:待检测图片；roi:最佳匹配矩形；score：匹配分数
    bool Match(cv::Mat &Img, cv::Rect &roi, float &score);

    //多角度模板匹配，在函数内部改角度步长
    //Img:待检测图片；roi:最佳匹配矩形；score：匹配分数
    bool Match(cv::Mat &Img, cv::RotatedRect &roi, float &score);

    //多角度，多模板  模板匹配
    //Img:待检测图片；results:最佳匹配矩形集合
    //tolerance:容许度
    bool Match(cv::Mat &Img, std::vector<MatchResult> &results,double tolerance= TOLERANCE_MAX);
private:
    bool gPryDownUsed;//是否使用低采样
    cv::Mat gTemplate;//模板图像，单通道
};
#endif //SRC_MATCHTEMPLATE_H
