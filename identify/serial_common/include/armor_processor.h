#ifndef ARMOR_PROCESSOR_H
#define ARMOR_PROCESSOR_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include "iomanip"
#include <stdlib.h>

#include <string>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>
#include <opencv2/opencv.hpp>
#include "serial_common.h"
#include <algorithm>
#include <numeric>
#include <serial_common/gimbalPRY.h>

using namespace cv;

enum class AimStatus
{
    MOVING,
    ROTATING,
    DAFU
};

class ArmorProcessor
{
public:
    int deviceID;
    bool ifShow;
    bool singleSht;

    float gimbalYaw, gimbalPitch, gimbalRoll;
    int speed = 26;
    int cnt = 0;
    int zcnt = 0;
    int switcnt = 0;

    Mat camCoord;
    Mat camRotMat;
    Mat camRotVec;
    Mat cam2world;
    Mat worldCoord;
    Mat worldRotMat;
    Mat KFresult;
    Mat origin;

    Mat preWrdCoord;
    Mat coordiShow;
    Mat PCAdata;
    vector<float> dataX, dataY, dataZ;
    Mat shootTgt;
    float distance2cam = 0;
    float dis;
    float disAcum = 0;
    int dir;
    bool newArmor, armorHitted;
    ros::Time curTime,lastTime;

    AimStatus status = AimStatus::MOVING;
    bool recgUpdate = false;
    bool newTgt = true;
    bool getTgt = false;
    bool ready = false;
    bool zready = false;
    

    unsigned char mode = 0x12;

    const unsigned char mode_normal=0x02; //自瞄
    const unsigned char mode_s_windMill=0x03;  // 小符
    const unsigned char mode_top=0x05; //陀螺
    const unsigned char mode_b_windMill=0x04; //大符

    KalmanFilter* KF;
    Mat measurement;
    float spdx, spdy, spdz;
    float KFGain=0.2;

	short lastx=0;

    Mat LastInputCoodi, CurrentInputPixel, PixelRecord;
    int jump_cnt, lost_cnt;

    
    
    ArmorProcessor(bool ifShow=false, int deviceID=0);
    ~ArmorProcessor();
    void recAndProcess(gimbal_info& gimbal, aim_info& target,ros::Publisher& pub_);
    bool modeUpdate(gimbal_info& gimbal);
    bool infoUpdate(gimbal_info& gimbal,ros::Publisher& pub_);
    void ifReset(Mat InputCoordi,float InterframeError,int FilterLength);
    float disBetweenPoints(Mat& A, Mat& B);
    float disInPlain(Mat& A, Mat& B);

};





#endif
