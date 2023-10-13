#include "../include/ui/rotation_recongnition.hpp"
#include <iostream>
#include <numeric>

using namespace cv;

// 获取矩阵平均值
double turn::getMean(Mat &in)
{
    double sum = 0.0;
	int rowNumber = in.rows;
	int colNumber = in.cols * in.channels();
	for (int i = 0; i < rowNumber;i++)
	{
		uchar* data = in.ptr<uchar>(i);
		for (int j = 0; j < colNumber; j++)
		{
			sum = data[j] + sum;
		}
	}
	return sum / (in.rows * in.cols);
}

// 获取两帧均方误差
double turn::getMSE(Mat &color, Mat &depth)
{
    if (lastColor.empty() || lastDepth.empty())
    {
        std::cout<<"no last img"<<std::endl;
        lastColor = color;
        lastDepth = depth;
        return 0;
    }
    double re;
    Mat tmp;
    absdiff(lastColor, color, tmp);
    re = getMean(tmp);
    std::cout<<"re is"<<re<<std::endl;
    lastColor = color;
    absdiff(lastDepth, depth, tmp);
    re += getMean(tmp) * 3;
    lastDepth = depth;
    lastMSE = re;
    std::cout << lastMSE<<std::endl;
    return re;
}

// 旋转是否结束判定，并转换为当前朝向台编号返回
int turn::getStep(Mat color, Mat depth)
{
    ROS_WARN("getStep");
    bool isR = getMSE(color, depth) > thres;
    if(!rotating && isR && count < maxCount)
    {
        if(count < 0)
            count = 0;
        count++;
    }
    if(rotating && !isR && count > -maxCount)
    {
        if(count > 0)
            count = 0;
        count--;
    }
    if(count >= maxCount)
    {
        rotating = true;
        count = 0;
        return 1;
    }
    if(count <= -maxCount)
    {
        rotating = false;
        count = 0;
        return 1;
    }
    std::cout<<"rotate count"<<count<<std::endl;
    return 0;
}

// 获取矩阵平均值
double turn2::getMean(Mat &in)
{
    double sum = 0.0;
	int rowNumber = in.rows;
	int colNumber = in.cols * in.channels();
	for (int i = 0; i < rowNumber;i++)
	{
		uchar* data = in.ptr<uchar>(i);
		for (int j = 0; j < colNumber; j++)
		{
			sum = data[j] + sum;
		}
	}
	return sum / (in.rows * in.cols);
}

// 获取两帧均方误差
double turn2::getMSE(Mat &color, Mat &depth)
{
    if (lastColor.empty() || lastDepth.empty())
    {
        std::cout<<"no last img"<<std::endl;
        lastColor = color;
        lastDepth = depth;
        return 0;
    }
    double re;
    Mat tmp;
    absdiff(lastColor, color, tmp);
    re = getMean(tmp);
    std::cout<<"re is"<<re<<std::endl;
    lastColor = color;
    absdiff(lastDepth, depth, tmp);
    re += getMean(tmp) * 3;
    lastDepth = depth;
    lastMSE = re;
    std::cout << lastMSE<<std::endl;
    return re;
}

// 旋转是否结束判定，并转换为当前朝向台编号返回
int turn2::getStep(Mat color, Mat depth, int step)
{
    ROS_WARN("getStep");
    MSE.push_back(getMSE(color, depth));
    MSE.erase(MSE.begin());
    double res = std::accumulate(MSE.begin(), MSE.end(), 0);
    if ((ros::Time::now() - begin).toSec() > 6 && rotating)
    {
        rotating = false;
        return 2;
    }
    if ((ros::Time::now() - begin).toSec() > 10 && !rotating)
    {
        rotating = true;
        begin = ros::Time::now();
        return 3;
    }
    if (res > maxCount * thres && !rotating)
    {
        rotating = true;
        begin = ros::Time::now();
        return 1;
    }
    if (res < maxCount * thres * 0.5 && rotating && (ros::Time::now() - begin).toSec() > 2)
    {
        rotating = false;
        return 1;
    }
    return 0;
}

void turn2::updateBegin()
{
    begin = ros::Time::now();
}
