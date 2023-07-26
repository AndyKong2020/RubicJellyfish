//
// Created by andykong on 23-7-11.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "scheduler/pose_mode.h"
#include "scheduler/velocity_mode.h"
#include "scheduler/Drone.h"

ros::Publisher pose_mode_pub;
ros::Publisher velocity_mode_pub;

void sendPosition(scheduler::pose_mode &pose){

    pose.self_x = drone.getPosition().x();
    pose.self_y = drone.getPosition().y();
    pose.self_z = drone.getPosition().z();
    pose.self_roll = drone.getAngularOrientation().x();
    pose.self_pitch = drone.getAngularOrientation().y();
    pose.self_yaw = drone.getAngularOrientation().z();

    pose.target_x = 1;
    pose.target_y = 1;
    pose.target_z = 1;
    pose.target_roll = 1;
    pose.target_pitch = 1;
    pose.target_yaw = 1;

    pose.self_vx = drone.getVelocity().x();
    pose.self_vy = drone.getVelocity().y();
    pose.self_vz = drone.getVelocity().z();
    pose.self_wroll = drone.getAngularVelocity().x();
    pose.self_wpitch = drone.getAngularVelocity().y();
    pose.self_wyaw = drone.getAngularVelocity().z();

    pose.target_vx = 1;
    pose.target_vy = 1;
    pose.target_vz = 1;
    pose.target_wroll = 1;
    pose.target_wpitch = 1;
    pose.target_wyaw = 1;
    pose_mode_pub.publish(pose);
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
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "scheduler");
    ros::NodeHandle nh;
    scheduler::pose_mode pose;
    scheduler::velocity_mode velocity;
    drone.init();

    pose_mode_pub = nh.advertise<scheduler::pose_mode>("/t265/pos", 1);
    //velocity_mode_pub = nh.advertise<scheduler::velocity_mode>("/t265/velocity", 1);

    ros::Subscriber t265_sub = nh.subscribe("/camera/odom/sample", 10, t265Callback);
    ros::Rate loop_rate(200);
    while (ros::ok()) {
        //std::cout << drone.getAngularOrientation() << std::endl;

        sendPosition(pose);
        //sendVelocity(velocity);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
