#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include "iomanip"
#include <stdlib.h>
#include "scheduler/pose_mode.h"
#include "serial_common/plot_test.h"
#include "serial_common/gimbal.h"
#include "recognize/image.h"
#include <string>
#include <iostream>
#include <stdio.h>
#include <vector>
#include "serial_common.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


using namespace std;
SerialCommon ser;
ros::Publisher plot_z;
ros::Publisher imu_show;
bool ifShow;

std_msgs::Int8 mode;
gimbal_info gimbalAng;
serial_common::gimbal  _imu;
serial_common::plot_test test;

void t265poswrite_callback(const scheduler::pose_mode::ConstPtr& msg)
{
    t265_pos pos;
    pos.self_x = msg->self_x;
    pos.self_y = msg->self_y;
    pos.self_z = msg->self_z;
    pos.self_roll = msg->self_roll;
    pos.self_pitch = msg->self_pitch;
    pos.self_yaw = msg->self_yaw;
    pos.target_x = msg->target_x;
    pos.target_y = msg->target_y;
    pos.target_z = msg->target_z;
    pos.target_roll = msg->target_roll;
    pos.target_pitch = msg->target_pitch;
    pos.target_yaw = msg->target_yaw;
    pos.self_vx = msg->self_vx;
    pos.self_vy = msg->self_vy;
    pos.self_vz = msg->self_vz;
    pos.self_wroll = msg->self_wroll;
    pos.self_wpitch = msg->self_wpitch;
    pos.self_wyaw = msg->self_wyaw;
    pos.target_vx = msg->target_vx;
    pos.target_vy = msg->target_vy;
    pos.target_vz = msg->target_vz;
    pos.target_wroll = msg->target_wroll;
    pos.target_wpitch = msg->target_wpitch;
    pos.target_wyaw = msg->target_wyaw;
    cout<< "z:"<<pos.self_z<<endl;
    test.x = pos.self_x;
    test.y = pos.self_y;
    test.z = pos.self_z;
    test.vx = pos.target_x;
    test.vy = pos.target_y;
    test.vz = pos.target_z;
    plot_z.publish(test);
    cal_sum(&pos);
    ser.serSend<t265_pos>(pos,pos.length);
}

//void imagewrite_callback(const recognize::image::ConstPtr& msg)
//{
//  image_target shootTgt;
//  vector<float> data;
//  shootTgt.mode = 2;
//  int length = 16;
//
//  shootTgt.length = length;
//  cal_sum(&shootTgt);
//  ser.serSend<image_target>(shootTgt,length);
//}

int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_common_node");
    //声明节点句柄
    ros::NodeHandle nh;
    //订阅主题，并配置回调函数

    nh.getParam("/ifshow",ifShow);

    plot_z = nh.advertise<serial_common::plot_test>("/plot_z", 1);
    imu_show = nh.advertise<serial_common::gimbal>("/imu_show", 1);
    ros::Subscriber t265pos_sub = nh.subscribe("/t265/pos",1,t265poswrite_callback);
    //ros::Subscriber image_sub = nh.subscribe("/image/write", 1, imagewrite_callback);

    //设置串口属性，并打开串口

    if(!ser.init())
      ros::shutdown();

    ros::Rate loop_rate(400);
    while(ros::ok())
    {
        ser.serRead<gimbal_info>(gimbalAng);//
        uint8_t _sumcheck = gimbalAng.sumcheck,_addcheck = gimbalAng.addcheck;
        cal_sum(&gimbalAng);
        if(gimbalAng.sumcheck == _sumcheck && gimbalAng.addcheck == _addcheck){
            _imu.x = gimbalAng.x;
            _imu.y = gimbalAng.y;
            _imu.z = gimbalAng.z;
            _imu.vx = gimbalAng.vx;
            _imu.vy = gimbalAng.vy;
            _imu.vz = gimbalAng.vz;
            _imu.ax = gimbalAng.vx;
            _imu.ay = gimbalAng.vy;
            _imu.az = gimbalAng.vz;
            _imu.quaw = gimbalAng.quaw;
            _imu.quax = gimbalAng.quax;
            _imu.quay = gimbalAng.quay;
            _imu.quaz = gimbalAng.quaz;
            imu_show.publish(_imu);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


}
