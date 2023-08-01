#ifndef SERIAL_C_H
#define SERIAL_C_H

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

using namespace std;
typedef struct
{
    float self_z = 0;
}__attribute__((packed)) plot_test;
typedef struct
{
    uint8_t header1 = 0xAA;
    uint8_t d_addr = 0xFF;
    uint8_t id = 0x00;
    uint8_t length = 103;
    uint8_t mode = 1;       //(T265_pos -> mode=1  T265_velocity -> mode=2  image -> mode=3 )
    float self_x = 0; //5+12+2
    float self_y = 0;
    float self_z = 0;
    float self_roll = 0;
    float self_pitch = 0;
    float self_yaw = 0;
    float target_x = 0;
    float target_y = 0;
    float target_z = 0;
    float target_roll = 0;
    float target_pitch = 0;
    float target_yaw = 0;
    float self_vx = 0;
    float self_vy = 0;
    float self_vz = 0;
    float self_wroll = 0;
    float self_wpitch = 0;
    float self_wyaw = 0;
    float target_vx = 0;
    float target_vy = 0;
    float target_vz = 0;
    float target_wroll = 0;
    float target_wpitch = 0;
    float target_wyaw = 0;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
}__attribute__((packed)) t265_pos;

typedef struct
{
    uint8_t header1 = 0xAA;
    uint8_t d_addr = 0xFF;
    uint8_t id = 0x00;
    uint8_t length = 16;
    uint8_t mode = 3;       //(T265_pos -> mode=1  T265_velocity -> mode=2  image -> mode=3 )
    uint8_t type = 0;
    float x = 0;
    float y = 0;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
}__attribute__((packed)) image_target;

typedef struct
{
    uint8_t header1 = 0xAA;
    uint8_t d_addr = 0xFF;
    uint8_t id = 0x00;
    uint8_t length = 32;
    int16_t x = 0; //4+2*13+2 = 32
    int16_t y = 0;
    int16_t z = 0;
    int16_t vx = 0;
    int16_t vy = 0;
    int16_t vz = 0;
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t quaw = 0;
    int16_t quax = 0;
    int16_t quay = 0;
    int16_t quaz = 0;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
}__attribute__((packed)) gimbal_info;

class SerialCommon
{
public:
    serial::Serial serial;//串口对象
    std::string buffer;
    

    bool init();
    unsigned char addCheckSum(unsigned char InputBytes[], unsigned char data_lenth);
    template <typename T> 
    void serSend(T& info,int len);
    template <typename T> 
    void serRead(T& gimbalInfo);

    

};

template <typename T> 
void SerialCommon::serRead(T& gimbalInfo)
{
    buffer.clear();
    serial.read(buffer,32);
    //serial.readline(buffer, 12,"\r\n");
    serial.flushInput();
    memset(&gimbalInfo, 0, sizeof(gimbalInfo));
    gimbalInfo=*(T*)buffer.c_str();
}

template <typename T> 
void SerialCommon::serSend(T& info, int len)
{
    serial.write((uint8_t*)&info , len);   //发送串口数据
}

inline void cal_sum(image_target* data){
    char *p = (char *)data;
    for(int i = 0; i < data->length-2; i++){
        data->sumcheck += p[i];
        data->addcheck += data->sumcheck;
    }
}
inline void cal_sum(t265_pos * data){
    char *p = (char *)data;
    for(int i = 0; i < data->length-2; i++){
        data->sumcheck += p[i];
        data->addcheck += data->sumcheck;
    }
}
inline void cal_sum(gimbal_info * data){
    char *p = (char *)data;
    uint8_t sum = 0,add = 0;
    for(int i = 0; i < data->length-2; i++){
        sum += p[i];
        add += sum;
    }
    data->sumcheck = sum;
    data->addcheck = add;
}

#endif
