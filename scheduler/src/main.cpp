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
#include "scheduler/Task.h"

ros::Publisher pose_mode_pub;
ros::Publisher velocity_mode_pub;

scheduler::pose_mode pose;

DronePose target_pose;
RouteTask *route_task01 = nullptr;

DronePose route_point00;


void sendPosition(scheduler::pose_mode &_pose){

    _pose.self_x = drone.getPosition().x();
    _pose.self_y = drone.getPosition().y();
    _pose.self_z = drone.getPosition().z();
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

    _pose.target_vx = 1;
    _pose.target_vy = 1;
    _pose.target_vz = 1;
    _pose.target_wroll = 1;
    _pose.target_wpitch = 1;
    _pose.target_wyaw = 1;
    pose_mode_pub.publish(_pose);
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

    sendPosition(pose);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "scheduler");
    ros::NodeHandle nh;
    scheduler::velocity_mode velocity;
    drone.init();
    target_pose.position = Eigen::Vector3d::Zero();
    target_pose.angular_orientation = Eigen::Vector3d::Zero();
    route_task01 = new RouteTask(1);
    route_point00.position = Eigen::Vector3d(1, 0, 0);
    route_point00.angular_orientation = Eigen::Vector3d(0, 0, 0);
    route_task01 -> addToRouteList(route_point00);
    pose_mode_pub = nh.advertise<scheduler::pose_mode>("/t265/pos", 1);
    //velocity_mode_pub = nh.advertise<scheduler::velocity_mode>("/t265/velocity", 1);

    ros::Subscriber t265_sub = nh.subscribe("/camera/odom/sample", 10, t265Callback);
    ros::Rate loop_rate(200);
    while (ros::ok()) {
        //std::cout << drone.getAngularOrientation() << std::endl;

        //sendVelocity(velocity);
        while (!route_task01->isRouteFinished()){
            target_pose = route_task01 -> runTask();
            std::cout << target_pose.position << std::endl;
            drone.setPosition(target_pose.position);

        }

        std::cout << "route finished" << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
