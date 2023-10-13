#ifndef ROTATION_H
#define ROTATION_H

#include"ros/ros.h"
#include <opencv2/opencv.hpp>
#include <list>

class turn
{
public:
    cv::Mat lastColor, lastDepth;
    bool rotating;
    int count;
    double thres;
    double lastMSE;
    int maxCount;

    double getMean(cv::Mat &in);
    double getMSE(cv::Mat &color, cv::Mat &depth);
    int getStep(cv::Mat color, cv::Mat depth);

    turn()
    {
        rotating = false;
        count = 0;
        thres = 150;
        lastMSE = -1;
        maxCount = 30;
    }
};

class turn2
{
public:
    cv::Mat lastColor, lastDepth;
    bool rotating;
    int count;
    double thres;
    double lastMSE;
    int maxCount;
    int maxFrame;
    std::list<double> MSE;
    ros::Time begin;
    ros::NodeHandle *node;
    int ss;
    ros::Timer timer;

    double getMean(cv::Mat &in);
    double getMSE(cv::Mat &color, cv::Mat &depth);
    int getStep(cv::Mat color, cv::Mat depth, int step);
    void updateBegin();
    
    turn2()
    {
        rotating = false;
        count = 0;
        maxFrame = 30;
        thres = 25;
        lastMSE = -1;
        maxCount = 10;
        for(int i = 0; i < maxCount; i++)
        {
            MSE.push_back(0);
        }
    }
};

#endif