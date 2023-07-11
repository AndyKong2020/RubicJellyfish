//
// Created by robin on 23-2-15.
//
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include "serial_robot/serialPro.h"
#include "serial_robot/robotComm.h"
#include "serial_robot/aim.h"
#include "serial_robot/exchange.h"
#include "serial_robot/gimbalPRY.h"
#include "serial_robot/spinning_control.h"
#include "serial_robot/dialSwitch.h"

ros::Publisher odomPub;
ros::Publisher imuPub;
ros::Publisher mode_change;
ros::Publisher gimbal;
ros::Publisher chassis_angle_Pub;
ros::Publisher dialSwitch_Pub;

robot::RobotSerial serial;

float recommend_pitch = 0.0;

message_data autoaim{
        float yaw, pitch, w_yaw, w_pitch;
        uint8_t target_rate, target_number;
};

message_data spin_speed{
        int16_t spinning_speed;
};

message_data exchangestation{
        float_t x, y, z, q_x, q_y, q_z, q_w;
};

message_data imu{
        float_t ax, ay, az;
        float_t wx, wy, wz;
        float_t qx, qy, qz, qw;
};

message_data chassis{
        int16_t vx, vy, wz;
};

message_data odometry{
        int x, y, yaw;
        int vx, vy, wz;
};

message_data kalman{
        float pitch, roll, yaw;
        uint8_t bullet;
};

message_data dialSwitch{
    uint8_t flag,is3big,is4big,is5big;
};

chassis c_rc{
        .vx = (int16_t) 0,
        .vy = (int16_t) 0,
        .wz = (int16_t) 0
};

void spin_speed_callback(const serial_robot::spinning_control &msg) {
    spin_speed spin_t{
            .spinning_speed = (int16_t) (msg.spinning_speed),
    };
    serial.write(0x85, spin_t);
}

void logic_pitch_callback(const std_msgs::Float32::ConstPtr &msg) {
    recommend_pitch = msg->data;
}

void aimcallback(const serial_robot::aim &msg) {
    autoaim aim{
            .yaw = (float) (msg.yaw),
            .pitch = (float) (msg.pitch),
            .w_yaw = (float) (msg.w_yaw),
            .w_pitch = (float) (msg.w_pitch),
            .target_rate = (uint8_t) (msg.target_rate),
            .target_number = (uint8_t) (msg.target_number),
    };
    if (aim.target_number == 0) {
        aim.pitch = recommend_pitch;
    }
    std::cout << aim.yaw << aim.pitch << aim.target_rate << std::endl;
    serial.write(0x81, aim);
}

void exchangecallback(const geometry_msgs::PoseStamped &msg) {
    exchangestation _exchange{};
    _exchange.x = (float) msg.pose.position.x;
    _exchange.y = (float) msg.pose.position.y;
    _exchange.z = (float) msg.pose.position.z;
    _exchange.q_x = (float) msg.pose.orientation.x;
    _exchange.q_y = (float) msg.pose.orientation.y;
    _exchange.q_z = (float) msg.pose.orientation.z;
    _exchange.q_w = (float) msg.pose.orientation.w;
    serial.write(0x84, _exchange);
}

void imucallback(const imu &msg) {
    sensor_msgs::Imu _imu;
    _imu.angular_velocity.x = msg.wx;
    _imu.angular_velocity.y = msg.wy;
    _imu.angular_velocity.z = msg.wz;
    _imu.linear_acceleration.x = msg.ax;
    _imu.linear_acceleration.y = msg.ay;
    _imu.linear_acceleration.z = msg.az;
    _imu.orientation.x = msg.qx;
    _imu.orientation.y = msg.qy;
    _imu.orientation.z = msg.qz;
    _imu.orientation.w = msg.qw;
    imuPub.publish(_imu);
}

void chassisCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    chassis c{
            .vx = (int16_t) (msg->linear.y * 2008) + c_rc.vy,
            .vy = (int16_t) (msg->linear.x * 2008) + c_rc.vx,
            .wz = (int16_t) (msg->angular.z * -1507) + c_rc.wz
//            .wz = (int16_t) 0 + c_rc.wz //测试导航不跟头使用
    };
    serial.write(0x82, c);
//    std::cout << "speed:" << c.vx << " " << c.vy << " " << c.wz << std::endl;
}

void Key_RC_Callback(const geometry_msgs::Twist::ConstPtr &msg) {//遥控
    c_rc.vx = (int16_t) (msg->linear.x * 2008);
    c_rc.vy = (int16_t) (msg->linear.y * 2008);
    c_rc.wz = (int16_t) (msg->angular.z * -1507);
//    c_rc.wz = (int16_t) 0;
    serial.write(0x82, c_rc);
}

void odomCallback(const odometry &msg) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = msg.x / 1000.;
    odom.pose.pose.position.y = msg.y / 1000.;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = msg.vx / 1000.;
    odom.twist.twist.linear.y = msg.vy / 1000.;
    odom.twist.twist.angular.z = msg.wz / 1000.;
    odomPub.publish(odom);
}

void gimbalCallback(const kalman &msg) {
    serial_robot::gimbalPRY _gimbal;
    _gimbal.pitch = msg.pitch;
    _gimbal.roll = msg.roll;
    _gimbal.yaw = msg.yaw;
    _gimbal.bullet = msg.bullet;
//    std::cout<<_gimbal.yaw<<_gimbal.pitch<<_gimbal.roll<< std::endl;
    gimbal.publish(_gimbal);
}

void chassis_angle_callback(const float &msg) {
    std_msgs::Float32 chassis_angle_msg;
    chassis_angle_msg.data = msg;
    chassis_angle_Pub.publish(chassis_angle_msg);
}

void modeCallback(const uint8_t &msg) {
    std_msgs::UInt8 mode;
    mode.data = msg;
    mode_change.publish(mode);
}
void switchCallback(const dialSwitch &msg){
    serial_robot::dialSwitch _switch;
    _switch.flag = msg.flag; // turn on --> 2  turn off --> 1
    if(_switch.flag == 2){
    _switch.is3big = msg.is3big;
    _switch.is4big = msg.is4big;
    _switch.is5big = msg.is5big;
    dialSwitch_Pub.publish(_switch);
    }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_robot");
    ros::NodeHandle nh("~");

    std::string serial_name;

    nh.param<std::string>("/serial_name", serial_name, "/dev/ttyUSB0");
    std::cout << "Using serial port: " << serial_name << std::endl;
    serial = std::move(robot::RobotSerial(serial_name, 115200));

    ros::Subscriber chassis_con = nh.subscribe("/cmd_vel", 1, chassisCallback);
    ros::Subscriber chassis_key_RC = nh.subscribe("/cmd_vel_RC", 1, Key_RC_Callback);
    ros::Subscriber auto_aim = nh.subscribe("/robot/auto_aim", 1, aimcallback);
    ros::Subscriber logic_pitch = nh.subscribe("/robot/logic_recommend_angle", 1, logic_pitch_callback);
    ros::Subscriber spinning_speeder = nh.subscribe("/robot/spnning_speed", 1, spin_speed_callback);
    ros::Subscriber exchange_sub = nh.subscribe("/robot/exchange", 1, exchangecallback);

    serial.registerErrorHandle([](int label, const std::string &text) {  //日志记录
        robot::RobotSerial::error _label;
        _label = (robot::RobotSerial::error) label;
        std::stringstream _str;
        for (auto c: text) {
            _str << std::setw(2) << std::hex << (int) *(uint8_t *) &c << " ";
        }
        _str << std::endl;
        switch (_label) {
            case robot::RobotSerial::lengthNotMatch:
                ROS_ERROR_STREAM("lengthNotMatch");
                ROS_ERROR_STREAM(_str.str());
            case robot::RobotSerial::rxLessThanLength:
                ROS_WARN_STREAM("rxLessThanLength");
                break;
            case robot::RobotSerial::crcError:
                ROS_ERROR_STREAM("crcError");
                ROS_ERROR_STREAM(_str.str());
                break;
            default:
                return;
        }
    });

    odomPub = nh.advertise<nav_msgs::Odometry>("/robot/odom", 5);
    imuPub = nh.advertise<sensor_msgs::Imu>("/robot/imu", 5);
//    exchangePub = nh.advertise<geometry_msgs::PoseStamped>("/robot/exchange", 5);
    mode_change = nh.advertise<std_msgs::UInt8>("/robot/mode", 5);
    chassis_angle_Pub = nh.advertise<std_msgs::Float32>("/robot/chassis_angle", 5);
    gimbal = nh.advertise<serial_robot::gimbalPRY>("/robot/gimbalPRY", 5);
    dialSwitch_Pub = nh.advertise<serial_robot::dialSwitch>("/robot/dialSwitch", 5);

    serial.registerCallback(0x11, imucallback);
    serial.registerCallback(0x12, odomCallback);
    serial.registerCallback(0x14, gimbalCallback);
    serial.registerCallback(0x15, modeCallback);
    serial.registerCallback(0x16, chassis_angle_callback);
    serial.registerCallback(0x17,switchCallback);

    serial.spin(true);
    ros::spin();
}
