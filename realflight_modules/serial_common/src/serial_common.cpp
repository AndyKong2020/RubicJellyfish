#include "serial_common.h"
bool SerialCommon::init()
{
      serial.setPort("/dev/ttyUSB0");
      serial.setBaudrate(460800);
      serial::Timeout to = serial::Timeout::simpleTimeout(50);
      serial.setTimeout(to);
      serial.open();
      cout<<"SYSTEM USB NOT DETECTED"<<endl;
      if(!serial.isOpen())
      {
          serial.setPort("/dev/ttyUSB1");
          serial.open();
      }
    //检测串口是否已经打开，并给出提示信息
    if(serial.isOpen())
    {
        cout<<"Serial Port initialized"<<endl;
        return true;
    }
    else
    {
        cout<<"Unable to open port "<<endl;
        return false;
    }
}


unsigned char SerialCommon::addCheckSum(unsigned char InputBytes[], unsigned char data_lenth) 
{
    unsigned char byte_crc = 0;
    for (unsigned char i = 0; i < data_lenth; i++) 
    {
        byte_crc += InputBytes[i];
    }
    return byte_crc;
}
