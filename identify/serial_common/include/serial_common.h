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


static const uint8_t crc_table[] =
        {
                0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97, 0xb9, 0x88, 0xdb, 0xea, 0x7d, 0x4c, 0x1f, 0x2e,
                0x43, 0x72, 0x21, 0x10, 0x87, 0xb6, 0xe5, 0xd4, 0xfa, 0xcb, 0x98, 0xa9, 0x3e, 0x0f, 0x5c, 0x6d,
                0x86, 0xb7, 0xe4, 0xd5, 0x42, 0x73, 0x20, 0x11, 0x3f, 0x0e, 0x5d, 0x6c, 0xfb, 0xca, 0x99, 0xa8,
                0xc5, 0xf4, 0xa7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7c, 0x4d, 0x1e, 0x2f, 0xb8, 0x89, 0xda, 0xeb,
                0x3d, 0x0c, 0x5f, 0x6e, 0xf9, 0xc8, 0x9b, 0xaa, 0x84, 0xb5, 0xe6, 0xd7, 0x40, 0x71, 0x22, 0x13,
                0x7e, 0x4f, 0x1c, 0x2d, 0xba, 0x8b, 0xd8, 0xe9, 0xc7, 0xf6, 0xa5, 0x94, 0x03, 0x32, 0x61, 0x50,
                0xbb, 0x8a, 0xd9, 0xe8, 0x7f, 0x4e, 0x1d, 0x2c, 0x02, 0x33, 0x60, 0x51, 0xc6, 0xf7, 0xa4, 0x95,
                0xf8, 0xc9, 0x9a, 0xab, 0x3c, 0x0d, 0x5e, 0x6f, 0x41, 0x70, 0x23, 0x12, 0x85, 0xb4, 0xe7, 0xd6,
                0x7a, 0x4b, 0x18, 0x29, 0xbe, 0x8f, 0xdc, 0xed, 0xc3, 0xf2, 0xa1, 0x90, 0x07, 0x36, 0x65, 0x54,
                0x39, 0x08, 0x5b, 0x6a, 0xfd, 0xcc, 0x9f, 0xae, 0x80, 0xb1, 0xe2, 0xd3, 0x44, 0x75, 0x26, 0x17,
                0xfc, 0xcd, 0x9e, 0xaf, 0x38, 0x09, 0x5a, 0x6b, 0x45, 0x74, 0x27, 0x16, 0x81, 0xb0, 0xe3, 0xd2,
                0xbf, 0x8e, 0xdd, 0xec, 0x7b, 0x4a, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xc2, 0xf3, 0xa0, 0x91,
                0x47, 0x76, 0x25, 0x14, 0x83, 0xb2, 0xe1, 0xd0, 0xfe, 0xcf, 0x9c, 0xad, 0x3a, 0x0b, 0x58, 0x69,
                0x04, 0x35, 0x66, 0x57, 0xc0, 0xf1, 0xa2, 0x93, 0xbd, 0x8c, 0xdf, 0xee, 0x79, 0x48, 0x1b, 0x2a,
                0xc1, 0xf0, 0xa3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1a, 0x2b, 0xbc, 0x8d, 0xde, 0xef,
                0x82, 0xb3, 0xe0, 0xd1, 0x46, 0x77, 0x24, 0x15, 0x3b, 0x0a, 0x59, 0x68, 0xff, 0xce, 0x9d, 0xac
        };//生成表对应多项式：0x31


inline uint8_t cal_crc_table(uint8_t *ptr, uint8_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc = crc_table[crc ^ *ptr++];
  }
  return crc;
}

#ifdef SENTRY
#define TX_DATA_LEN 18
#else
#define TX_DATA_LEN 19
#endif


using namespace std;

typedef struct
{
    uint8_t header1 = 0xAA;
    uint8_t d_addr = 0xFF;
    uint8_t id = 0x00;
    uint8_t length = TX_DATA_LEN;
    float pitch = 1.1;//角度制
    float roll = 0;
    float yaw = 0;//-180～180
    uint8_t if_track = 0;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
}__attribute__((packed)) aim_info;

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
    void serSend(T& a);
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
void SerialCommon::serSend(T& aim_info)
{
    serial.write((uint8_t*)&aim_info, TX_DATA_LEN);   //发送串口数据
}

inline void cal_sum(aim_info* data){
    char *p = (char *)data;
    for(int i = 0; i < data->length-2; i++){
        data->sumcheck += p[i];
        data->addcheck += data->sumcheck;
    }
}

#endif
