//
// Created by andykong on 23-7-11.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/UInt8.h"
#include "scheduler/pose_mode.h"
#include "scheduler/velocity_mode.h"
#include "scheduler/Drone.h"
#include "scheduler/Task.h"
#include "serial_common/gimbal.h"
#include "recognize/image.h"

ros::Publisher pose_mode_pub;
ros::Publisher velocity_mode_pub;
ros::Publisher task_id_pub;
ros::Publisher target_pub;
ros::Publisher device_pub;

scheduler::pose_mode pose;

std_msgs::UInt8 current_task_id;

DronePose target_pose;
DronePose fire_pose;
ImageTarget img_target;
MessageToCar message_to_car = {0, 0, 0};
cv::Point2i frame_size;
bool is_send_fire_position_flag = false;
uint8_t mission = 2;
DeviceType device_type = DeviceType::NA;
static int tof_error_count;

TakeOffTask *take_off_task00 = nullptr;
RouteTask *route_task01 = nullptr;
PointTask *point_task02 = nullptr;
RouteTask *route_task03 = nullptr;
LandTask *land_task04 = nullptr;


DronePose take_off_point00;
DronePose route_point00, route_point01, route_point02, route_point03, route_point04, route_point05, route_point06;
DronePose stay_point00, stay_point01, stay_point02, stay_point03, stay_point04, stay_point05;
DronePose land_point00;
double take_off_height00;
Eigen::Vector3d point_true_value00;


void sendTaskId(const int & task_id)
{
    current_task_id.data = task_id;
    task_id_pub.publish(current_task_id);
}

void useDevice(const DeviceType & _device_type)
{
    std_msgs::UInt8 device;
    device.data = (uint8_t)_device_type;
    device_pub.publish(device);
}

void sendPosition(scheduler::pose_mode &_pose){

    _pose.self_x = drone.getPosition().x();
    _pose.self_y = drone.getPosition().y();
    _pose.self_z = drone.getHeight();
    _pose.self_roll = drone.getAngularOrientation().x();
    _pose.self_pitch = drone.getAngularOrientation().y();
    _pose.self_yaw = drone.getAngularOrientation().z();

    _pose.target_x = (float)target_pose.position.x();
    _pose.target_y = (float)target_pose.position.y();
    _pose.target_z = (float)target_pose.position.z();
    _pose.target_roll = (float)target_pose.angular_orientation.x();
    _pose.target_pitch = (float)target_pose.angular_orientation.y();
    _pose.target_yaw = (float)target_pose.angular_orientation.z();

    _pose.self_vx = drone.getVelocity().x();
    _pose.self_vy = drone.getVelocity().y();
    _pose.self_vz = drone.getVelocity().z();
    _pose.self_wroll = drone.getAngularVelocity().x();
    _pose.self_wpitch = drone.getAngularVelocity().y();
    _pose.self_wyaw = drone.getAngularVelocity().z();

    _pose.target_vx = (float)message_to_car.x;
    _pose.target_vy = (float)message_to_car.y;
    _pose.target_vz = message_to_car.point_id;
    _pose.target_wroll = (float)drone.getMileage();
    _pose.target_wpitch = 0;
    _pose.target_wyaw = 0;
    pose_mode_pub.publish(_pose);
    useDevice(device_type);
}
//void sendVelocity(scheduler::velocity_mode &velocity){
//
//
//
//    velocity_mode_pub.publish(velocity);
//}

void t265Callback(const nav_msgs::Odometry::ConstPtr & msg) {
    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond qtn_orientation(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    drone.setPosition(position);
    drone.setQtnOrientation(qtn_orientation);
    Eigen::Vector3d linear_velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Vector3d angular_velocity(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    drone.setVelocity(linear_velocity);
    drone.setAngularVelocity(angular_velocity);
    drone.setMileage();
}

void setParams(){

    frame_size.x = 640;
    frame_size.y = 480;


    take_off_point00.position = Eigen::Vector3d(0, 0, 0);
    take_off_point00.angular_orientation = Eigen::Vector3d(0, 0, 0);
    take_off_height00 = 1.8;
    take_off_task00 -> setTakeOffPoint(take_off_point00);
    take_off_task00 -> setTakeOffHeight(take_off_height00);
    stay_point00.position = Eigen::Vector3d(0, 0, 1.8);
    stay_point00.angular_orientation = Eigen::Vector3d(0, 0, 0);

    route_point00.position = Eigen::Vector3d(-2.7, 0.3, 1.8);
    route_point00.angular_orientation = Eigen::Vector3d(0, 0, 0);
    route_point01.position = Eigen::Vector3d(-2.5, 3.4, 1.8);
    route_point01.angular_orientation = Eigen::Vector3d(0, 0, 0);
    route_point02.position = Eigen::Vector3d(-1.1, 3.4, 1.8);
    route_point02.angular_orientation = Eigen::Vector3d(0, 0, 0);
    route_point03.position = Eigen::Vector3d(-1.1, 1.1, 1.8);
    route_point03.angular_orientation = Eigen::Vector3d(0, 0, 0);
    route_point04.position = Eigen::Vector3d(0, 0, 1.8);
    route_point04.angular_orientation = Eigen::Vector3d(0, 0, 0);
    route_task01 -> addToRouteList(route_point00);
    route_task01 -> addToRouteList(route_point01);
    route_task01 -> addToRouteList(route_point02);
    route_task01 -> addToRouteList(route_point03);
    route_task01 -> addToRouteList(route_point04);

    land_point00.position = Eigen::Vector3d(0, 0, -1.0);
    land_point00.angular_orientation = Eigen::Vector3d(0, 0, 0);
    land_task04 -> setLandPoint(land_point00);

    point_task02 ->setFrameSize(frame_size);
    point_task02 ->ifSetAccumulativeError(false);

    point_true_value00[0] = 0;
    point_true_value00[1] = 0;
    point_task02 ->setFrameSize(frame_size);
    point_task02 ->setTrueValue(point_true_value00);
    point_task02 ->ifSetAccumulativeError(false);


}
void imuCallback(const serial_common::gimbalConstPtr &msg) {   //change drone_control imu to camera imu
//    Eigen::Quaterniond quaternion(msg->quaw,msg->qua,msg->quay,msg->quaz);
//    Eigen::Vector3d eulerAngle=quaternion.matrix().eulerAngles(0,1,2);
//    drone_control.roll = eulerAngle.x();
//    drone_control.pitch = eulerAngle.y();
    static double last_h;

    if ((msg ->z - last_h > 0.1 || msg ->z - last_h < -0.1) && tof_error_count < 10) {
        tof_error_count ++;
        return;
    }
    tof_error_count = 0;
    last_h = msg->z;

    drone.setHeight(msg->z);
}
void imgCallback(const recognize::imageConstPtr &msg) {   //change drone_control imu to camera imu
    img_target.target_point.x = msg->x;
    img_target.target_point.y = msg->y;
    img_target.depth = msg->depth;
    img_target.is_detect = msg->mode;

}

void missionCallback(const std_msgs::UInt8 &msg) {
    mission = msg.data;
}

void stay(const DronePose & _pose, const double & time){
    target_pose.position = _pose.position;
    target_pose.angular_orientation = _pose.angular_orientation;
    ros::Rate rate(200);
    double start_time = ros::Time::now().toSec();
    while(ros::Time::now().toSec() - start_time < time){
        sendPosition(pose);
        ros::spinOnce();
        rate.sleep();
    }
}

void runTask(const double & stay_time, Task * task){
    ros::Rate control_rate(200);
    ROS_WARN("Task%d Start", task -> getTaskId());
    while (!task->isTaskFinished()){
        sendTaskId(task -> getTaskId());
        target_pose = task -> runTask();
        sendPosition(pose);
        task -> printLog();
        ros::spinOnce();
        control_rate.sleep();
    }
    ROS_WARN("Task%d Finished", task -> getTaskId());
    stay(task -> getStayPoint(), stay_time);
}

void runTask(const double & stay_time, PointTask * task, const ImageTarget & _image_target){
    ros::Rate control_rate(200);
    ROS_WARN("Task%d Start", task -> getTaskId());
    while (!task->isTaskFinished()){
    //while (1){
        sendTaskId(task -> getTaskId());
        task -> getMessage(_image_target);
        if (img_target.is_detect){
            target_pose = task -> runTask();
        }
        else{
            task -> setTargetPose(drone.getPose());
            stay(drone.getPose(), 3);
            if(!img_target.is_detect){
                target_pose = task -> runTask();
                break;
            }else{
                continue;
            }
        }
        geometry_msgs::PoseStamped point;
        point.header.frame_id = "t265_odom_frame";
        point.pose.position.x = target_pose.position.x();
        point.pose.position.y = target_pose.position.y();
        point.pose.position.z = target_pose.position.z();
        target_pub.publish(point);
        sendPosition(pose);
        task -> printLog();
        ros::spinOnce();
        control_rate.sleep();
    }
    ROS_WARN("Task%d Finished", task -> getTaskId());
    stay(task -> getStayPoint(), stay_time);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "scheduler");
    ros::NodeHandle nh;
    scheduler::velocity_mode velocity;
    drone.init();
    target_pose.position = Eigen::Vector3d::Zero();
    target_pose.angular_orientation = Eigen::Vector3d::Zero();

    take_off_task00 = new TakeOffTask(0);
    route_task01 = new RouteTask(1);
    point_task02 = new PointTask(2);
    point_task02 = new PointTask(3);
    land_task04 = new LandTask(4);

    pose_mode_pub = nh.advertise<scheduler::pose_mode>("/t265/pos", 1);
    task_id_pub = nh.advertise<std_msgs::UInt8>("/task_id", 1);
    device_pub = nh.advertise<std_msgs::UInt8>("/use_device", 1);
    target_pub = nh.advertise<geometry_msgs::PoseStamped>("/target", 10);
    ros::Subscriber imu = nh.subscribe("/imu_show",10,imuCallback);
    ros::Subscriber img = nh.subscribe("/image/write",10,imgCallback);
    ros::Subscriber t265_sub = nh.subscribe("/t265/odom/sample", 10, t265Callback);
    ros::Subscriber mission_sub = nh.subscribe("/switch_mode", 10, missionCallback);

    setParams();

    ros::Rate loop_rate(200);
    ros::Rate control_rate(200);
    while (ros::ok()) {
        device_type = DeviceType::NA;
        sendTaskId(0);
        mission = 1;
        if (mission == 1){
            ROS_WARN("RUNNING MISSION 1");
            runTask(2, take_off_task00);
            runTask(2, route_task01);
            runTask(0, point_task02, img_target);
            runTask(2, route_task03);
            runTask(0, land_task04);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
