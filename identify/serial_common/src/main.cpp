#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include "iomanip"
#include <stdlib.h>
#include <scheduler/pose_mode.h>
#include <scheduler/velocity_mode.h>
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
void t265poswrite_callback(const scheduler::pose_mode::ConstPtr& msg)
{
    t265_pos pos;
    pos.self_x = (int)msg->self_x*1000;
    pos.self_y = (int)msg->self_y*1000;
    pos.self_z = (int)msg->self_z*1000;
    pos.self_roll = (int)msg->self_roll*1000;
    pos.self_pitch = (int)msg->self_pitch*1000;
    pos.self_yaw = (int)msg->self_yaw*1000;
    pos.target_x = (int)msg->target_x*1000;
    pos.target_y = (int)msg->target_y*1000;
    pos.target_z = (int)msg->target_z*1000;
    pos.target_roll = (int)msg->target_roll*1000;
    pos.target_pitch = (int)msg->target_pitch*1000;
    pos.target_yaw = (int)msg->target_yaw*1000;
    cal_sum(&pos);
    ser.serSend<t265_pos>(pos,pos.length);
}
void t265velocitywrite_callback(const scheduler::velocity_mode::ConstPtr& msg)
{
    t265_velocity velocity;
    velocity.self_vx = (int)msg->self_vx*1000;
    velocity.self_vy = (int)msg->self_vy*1000;
    velocity.self_vz = (int)msg->self_vz*1000;
    velocity.self_wroll = (int)msg->self_wroll*1000;
    velocity.self_wpitch = (int)msg->self_wpitch*1000;
    velocity.self_wyaw = (int)msg->self_wyaw*1000;
    velocity.target_vx = (int)msg->target_vx*1000;
    velocity.target_vy = (int)msg->target_vy*1000;
    velocity.target_vz = (int)msg->target_vz*1000;
    velocity.target_wroll = (int)msg->target_wroll*1000;
    velocity.target_wpitch = (int)msg->target_wpitch*1000;
    velocity.target_wyaw = (int)msg->target_wyaw*1000;
    cal_sum(&velocity);
    ser.serSend<t265_velocity>(velocity,velocity.length);
}
void imagewrite_callback(const serial_common::serialWrite::ConstPtr& msg)
{
  image_target shootTgt;
  vector<float> data;
  shootTgt.mode = 2;
  int length = 16;

  shootTgt.length = length;
  cal_sum(&shootTgt);
  ser.serSend<image_target>(shootTgt,length);
}

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

    ros::Subscriber t265pos_sub = nh.subscribe("/t265/pos",1,t265poswrite_callback);
    ros::Subscriber t265velocity_sub = nh.subscribe("/t265/velocity",1,t265velocitywrite_callback);
    ros::Subscriber image_sub = nh.subscribe("/image/write", 1, imagewrite_callback);
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
//      while(1) {
//          write_callback();
//          //sleep(1);
//      }
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
