//
// Created by andykong on 23-7-11.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "scheduler");
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = 1.0;
    odom.pose.pose.position.y = 2.0;
    odom.pose.pose.position.z = 3.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;
    odom_pub.publish(odom);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
