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
    uint8_t header1 = 0xAA;
    uint8_t d_addr = 0xFF;
    uint8_t id = 0x00;
    uint8_t length = 19;
    uint8_t mode = 1;       //(T265_pos -> mode=1  T265_velocity -> mode=2  image -> mode=3 )
    int self_x = 0; //5+12+2
    int self_y = 0;
    int self_z = 0;
    int self_roll = 0;
    int self_pitch = 0;
    int self_yaw = 0;
    int target_x = 0;
    int target_y = 0;
    int target_z = 0;
    int target_roll = 0;
    int target_pitch = 0;
    int target_yaw = 0;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
}__attribute__((packed)) t265_pos;

typedef struct
{
    uint8_t header1 = 0xAA;
    uint8_t d_addr = 0xFF;
    uint8_t id = 0x00;
    uint8_t length = 19;
    uint8_t mode = 2;       //(T265_pos -> mode=1  T265_velocity -> mode=2  image -> mode=3 )
    int self_vx = 0;
    int self_vy = 0;
    int self_vz = 0;
    int self_wroll = 0;
    int self_wpitch = 0;
    int self_wyaw = 0;
    int target_vx = 0;
    int target_vy = 0;
    int target_vz = 0;
    int target_wroll = 0;
    int target_wpitch = 0;
    int target_wyaw = 0;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
}__attribute__((packed)) t265_velocity;

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
    uint8_t header1;
    uint8_t d_addr;
    uint8_t cmd_id;
    uint8_t length;
    float pitch;//弧度制
    float roll;
    float yaw;//-180～180
    uint8_t crc;
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
    
    serial.readline(buffer, 32,"\r\n");
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
inline void cal_sum(t265_velocity * data){
    char *p = (char *)data;
    for(int i = 0; i < data->length-2; i++){
        data->sumcheck += p[i];
        data->addcheck += data->sumcheck;
    }
}

#endif
