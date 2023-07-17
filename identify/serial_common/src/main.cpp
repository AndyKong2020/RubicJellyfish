#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include "iomanip"
#include <stdlib.h>
#include <serial_common/serialWrite.h>
#include <serial_common/HP.h>
#include <serial_common/RobotHP.h>
#include <serial_common/gimbal.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <vector>
#include "serial_common.h"
#include "armor_processor.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


using namespace std;

SerialCommon ser;

bool ifShow;

#ifdef SENTRY
int idUp=0;
int idDown=1;
ArmorProcessor *armorProUp = nullptr;
ArmorProcessor *armorProDown = nullptr;
#else
ArmorProcessor* armorPro = nullptr;
#endif
std_msgs::String write_rate;
ros::Publisher serial_rate_pub;
int lost_cnt;

//接受原始数据
uint8_t aRxBuffer[1] = {0};

//原始血量存储
vector<uint8_t> HPdata;
serial_common::HP hpdata_msg;
serial_common::RobotHP rHP_msg;
ros::Publisher HP_pub;

//模式切换
ros::Publisher mode_pub;
std_msgs::Int8 mode;
gimbal_info gimbalAng;




//void msg2Processor(ArmorProcessor& processor, const serial_common::serialWrite::ConstPtr& msg)
//{
//  if(msg->x != 30000 && msg->x != 20000)
//  {
//    //从相机坐标系转换到C板坐标系
//    processor.camCoord.at<float>(0,0) = msg->z / 1000.0;
//    processor.camCoord.at<float>(1,0) = msg->x / 1000.0;
//    processor.camCoord.at<float>(2,0) = msg->y / 1000.0;
//    lost_cnt=0;
//
//    processor.getTgt = true;
//  }
//  else if(msg->x == 30000)
//  {
//  	lost_cnt++;
//    processor.getTgt = false;
//    if(lost_cnt>600)
//    	processor.newTgt = true;
//  }
//  else
//  {
//    processor.getTgt = false;
//  }
//  processor.coordiConvent(shootTgt);
//  ser.serSend<aim_info>(shootTgt);
//}

#ifdef SENTRY
void write_callback_up(const serial_common::serialWrite::ConstPtr& msg)
{
  msg2Processor(armorProUp, msg);
}

void write_callback_down(const serial_common::serialWrite::ConstPtr& msg)
{
  msg2Processor(armorProDown, msg);
}
#else
void write_callback()
{
    aim_info shootTgt;
  vector<float> data;
  shootTgt.pitch = 1;

    cal_sum(&shootTgt);
    ser.serSend<aim_info>(shootTgt);
}
#endif

void receive_process(std::string &read_buffer)
{
  if(read_buffer[0]!=0xAA &&  read_buffer[1]!=0x02 &&  read_buffer[2]!=0x04 )
  {
    return;
  }
  int patch_num=(int)read_buffer[3];

}

int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_common_node");
    //声明节点句柄
    ros::NodeHandle nh;
    //订阅主题，并配置回调函数

    nh.getParam("/ifshow",ifShow);

    //ros::Subscriber write_sub = nh.subscribe("/down/write", 1, write_callback);
//    ros::Publisher img_pub_up = nh.advertise<sensor_msgs::Image>("/up/img_top", 1);
//    ros::Publisher img_pub_down = nh.advertise<sensor_msgs::Image>("/down/img_top", 1);
//    armorPro=new ArmorProcessor(ifShow);
    //ros::Subscriber write_sub = nh.subscribe("/write_to_Cboard", 1, write_callback);
//    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/img_top", 1);
//    #endif
//
//    mode_pub = nh.advertise<std_msgs::Int8>("/serial/read", 1);
//    HP_pub = nh.advertise<serial_common::HP>("/serial/HP", 1);
//    ros::Publisher PRY_pub = nh.advertise<serial_common::gimbalPRY>("/serial/gimbalPRY",1);
//
//
//    serial_rate_pub = nh.advertise<std_msgs::String>("/serial/write_rate", 1);
//    write_rate.data="1";
//
//    ros::Publisher status_pub = nh.advertise<serial_common::gimbal>("/robot_status", 1);

    //设置串口属性，并打开串口

    if(!ser.init())
      ros::shutdown();

    while(ros::ok())
    {
	   // serial_rate_pub.publish(write_rate);
      //ser.serRead<gimbal_info>(gimbalAng);//
      while(1) {
          write_callback();
          sleep(1);
      }
//      #ifdef SENTRY
//      if(gimbalAng.id == idUp)
//      {
//        armorProUp->recAndProcess(gimbalAng, shootTgt);
//      }
//      else
//      {
//        armorProDown->recAndProcess(gimbalAng, shootTgt);
//      }
//      #else
//      armorPro->recAndProcess(gimbalAng, shootTgt,PRY_pub);
//      mode.data=armorPro->mode;
//      mode_pub.publish(mode);
//      #endif



      ros::spinOnce();

    }


}
