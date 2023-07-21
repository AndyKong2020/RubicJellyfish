//
// Created by nuaa on 23-6-28.
//
#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "zbar.h"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <string>
#include "recognize/image_processing.h"
#include "recognize/matchTemplate.h"
#include "recognize/yolox.h"
#include "recognize/yolov5.h"
#include "recognize/image.h"
using namespace std;
using namespace cv;
using namespace zbar;


Mat img_show;
ros::Publisher image_pub;
image_processing _img;
vector<decodedObject> decode_rec;
EasyTemplate _temp;
Mat test1 = imread("/home/nuaa/wolf.jpg");
Mat test2 = imread("/home/nuaa/wolf_big.jpg");


void Image_cb(const sensor_msgs::ImageConstPtr &msg) {
    recognize::image _image;
    ros::Time start = ros::Time::now();
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        img_show = cv_ptr ->image;
        cv::imshow("the image",img_show);
        waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
//    decode(img_show,decode_rec); // recognize the code
//    decode_rec.clear(); // don't forget to clear the vector
    _img.image_threshold(img_show);
    std::cout << "Identify Latency: " << (ros::Time::now() - start).toSec() << "s" << std::endl;
    image_pub.publish(_image);
}

void Depth_cb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr Dest ;
    Dest = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth_pic = Dest->image;
    ushort d = depth_pic.at<ushort>(_img.point);           //读取深度值，数据类型为ushort单位为ｍｍ
    float d_value = float(d)/1000 ;      //强制转换
    cout<< "Value of depth_pic's pixel= "<<d_value<<endl;    //读取深度值
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "recognize");
    ros::NodeHandle n;
    ros::Subscriber resultsSub = n.subscribe("/camera/color/image_raw", 1, &Image_cb);
    ros::Subscriber DepthSub = n.subscribe("/camera/depth/image_rect_raw", 1, &Depth_cb);
    image_pub = n.advertise<recognize::image>("/image/write", 1);

    ros::Time start = ros::Time::now();

//the test of match_template

//    Rect _roi;
//    float _i = 0.8;
//    Mat display = test2.clone();
//    _temp.Mark(test1, true);
//    _temp.Match(test2, _roi,_i);
//    Point center={_roi.x+test1.cols/2,_roi.y+test1.rows/2};
//    cv::circle(display,center,50,Scalar(200, 255, 200),6);
//    imshow("model",test1);
//    imshow("show",display);

    //yolov5_identify(img_show);
    std::cout << "Identify Latency: " << (ros::Time::now() - start).toSec() << "s" << std::endl;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}