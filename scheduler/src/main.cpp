//
// Created by andykong on 23-7-11.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#include "scheduler/Drone.h"

ros::Publisher pose_pub;


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
  drone.init();
  ros::Subscriber t265_sub = nh.subscribe("/camera/odom/sample", 10, t265Callback);
  while (ros::ok()) {
      std::cout << drone.getAngularOrientation() << std::endl;
      ros::spinOnce();
  }
  return 0;
}
