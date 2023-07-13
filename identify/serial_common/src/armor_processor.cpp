#include "armor_processor.h"

ArmorProcessor:: ArmorProcessor(bool _ifShow, int _deviceID)
{
    deviceID = _deviceID;
    ifShow = _ifShow;
    camCoord = Mat::zeros(3,1,CV_32FC1);
    preWrdCoord = Mat::zeros(3,1,CV_32FC1);
    PCAdata = Mat::zeros(400,2,CV_32FC1);
    PixelRecord = Mat::zeros(3,1,CV_32FC1);
    origin = Mat::zeros(3,1,CV_32FC1);
    camRotVec = Mat_<float>(3,1);
    camRotMat = Mat_<float>(3,3);
    cam2world = Mat_<float>(3,3);
    worldCoord = Mat_<float>(3,1);
    worldRotMat = Mat_<float>(3,3);
    KFresult = Mat_<float>(3,1);
    shootTgt = Mat_<float>(3,1);
    dataX.resize(600);
    dataY.resize(600);
    dataZ.resize(10);

    // //KalmanFilter initialize 还没写完
	// KF=new KalmanFilter(6,3,0);
    // setIdentity(KF->transitionMatrix);
	// KF->transitionMatrix.at<float>(0,3) = 0.002;
    // KF->transitionMatrix.at<float>(1,4) = 0.002;
    // KF->transitionMatrix.at<float>(2,5) = 0.002;
	// //initialize as identity matix
	// setIdentity(KF->measurementMatrix);
	// setIdentity(KF->processNoiseCov, Scalar::all(100));
	// setIdentity(KF->measurementNoiseCov, Scalar::all(0.1));
	// setIdentity(KF->errorCovPost, Scalar::all(1));

    coordiShow = Mat::zeros(500,500,CV_8UC3);
    lastTime = ros::Time::now();
}

ArmorProcessor:: ~ArmorProcessor()
{
    if(KF != nullptr)
    {
        delete KF;
        KF = nullptr;
    }
}

void ArmorProcessor::recAndProcess(gimbal_info& gimbal, aim_info& target,ros::Publisher& pub_)
{
    infoUpdate(gimbal,pub_);
}

void ArmorProcessor::ifReset(Mat InputCoordi, float InterframeError, int FilterLength)
{
	

	// float PixelLength=GetPixelLength(LastInputPixel,CurrentInputPixel);
	// cout<<PixelLength<<endl;

	float PixelLength=disBetweenPoints(PixelRecord, InputCoordi);
  	cout<<"PixelLength= "<<PixelLength<<endl<<endl;

	if(PixelLength > InterframeError)
	{
		jump_cnt++;
		if(jump_cnt >= FilterLength)
		{
			jump_cnt = 0;
			newTgt = true;
            InputCoordi.copyTo(PixelRecord);
		}
		else
		{
			recgUpdate = false;
		}
	}
	else
	{
		jump_cnt=0;
		InputCoordi.copyTo(PixelRecord);
	}

    InputCoordi.copyTo(LastInputCoodi);
}

bool ArmorProcessor::modeUpdate(gimbal_info& gimbal)
{

    if(mode != (unsigned char)gimbal.cmd_id)
    {
    	cout<<"---------------mode change----------------"<<endl;
        newTgt = true;
        mode = (unsigned char)gimbal.cmd_id; 
        unsigned char MKType=mode & 0xF0; //装甲模式
        unsigned char WKmode=mode & 0x0F; //工作模式

        //加入发送
        if(WKmode == mode_normal)
        {
            status = AimStatus::MOVING;
            return true;
        }
        else if(WKmode == mode_top)
        {
            status = AimStatus::ROTATING;
            return true;
        }
        else if(WKmode == mode_b_windMill || WKmode == mode_s_windMill)
        {
            status = AimStatus::DAFU;
            return true;
        }
        else
            return false;
    }
    return true;
}

bool ArmorProcessor::infoUpdate(gimbal_info& gimbal,ros::Publisher& pub_)
{
    unsigned char header=0xBB;
    if(gimbal.header1!=header)
         return false;
         
    unsigned char len=16;
    if(gimbal.length!=len)
         return false;

  uint8_t* crc_caculate = (uint8_t*)&gimbal;
  if(gimbal.crc!=cal_crc_table(crc_caculate,15))
    return false;
         
         
    if(!modeUpdate(gimbal))
        return false;
    if(true)
    {
      serial_common::gimbalPRY g_p;
      g_p.pitch = gimbal.pitch;
      g_p.roll = gimbal.roll;
      g_p.yaw = gimbal.yaw;
      g_p.time = ros::Time::now();
      pub_.publish(g_p);
        gimbalYaw = gimbal.yaw * M_PI/180;
        gimbalPitch = gimbal.pitch * M_PI/180;
        gimbalRoll = gimbal.roll * M_PI/180;
        ROS_INFO_STREAM("serialRead "<<gimbalPitch<<" "<<gimbalRoll<<" "<<gimbalYaw<<endl);
        cam2world.at<float>(0,0) = cos(gimbalPitch) * cos(gimbalYaw);
        cam2world.at<float>(0,1) = cos(gimbalYaw) * sin(gimbalPitch) * sin(gimbalRoll)
                                - sin(gimbalYaw) * sin(gimbalRoll);
        cam2world.at<float>(0,2) = cos(gimbalYaw) * sin(gimbalPitch) * cos(gimbalRoll)
                                + sin(gimbalYaw) * sin(gimbalRoll);
        cam2world.at<float>(1,0) = cos(gimbalPitch) * sin(gimbalYaw);
        cam2world.at<float>(1,1) = sin(gimbalYaw) * sin(gimbalPitch) * sin(gimbalRoll)
                                + cos(gimbalYaw) * cos(gimbalRoll);
        cam2world.at<float>(1,2) = sin(gimbalYaw) * sin(gimbalPitch) * cos(gimbalRoll)
                                - cos(gimbalYaw) * sin(gimbalRoll);
        cam2world.at<float>(2,0) = -sin(gimbalPitch);
        cam2world.at<float>(2,1) = sin(gimbalRoll) * cos(gimbalPitch);
        cam2world.at<float>(2,2) = cos(gimbalRoll) * cos(gimbalPitch);
    }
    return true;
}
float ArmorProcessor::disBetweenPoints(Mat& A, Mat& B)
{
    return sqrt(pow((A.at<float>(0,0) - B.at<float>(0,0)),2)
                + pow((A.at<float>(1,0) - B.at<float>(1,0)),2)
                + pow((A.at<float>(2,0) - B.at<float>(2,0)),2));
}

float ArmorProcessor::disInPlain(Mat& A, Mat& B)
{
    return sqrt(pow((A.at<float>(0,0) - B.at<float>(0,0)),2)
                + pow((A.at<float>(1,0) - B.at<float>(1,0)),2));
}
