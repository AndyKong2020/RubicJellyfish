/*************************************************************************
  > File Name: video_pub.cpp
  > Author: CYZ
  > Mail:
  > Function: subscribe output img from windmill node and show them
 ************************************************************************/

#include<iostream>
using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

Mat img_show, img_depth, img_ir, coordinate, img_num, img_raw, img_top;
string dbg_img_path;
int false_idx=0;
bool ifrecord=false;
string rcd_path_;

string num2str(double i)

{
    stringstream ss;
    ss << i;
    return ss.str();
}

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_depth_sub_;
    image_transport::Subscriber image_ir_sub;
    image_transport::Subscriber coordinate_sub;
    image_transport::Subscriber top_sub;
    image_transport::Subscriber image_num_sub_;

    ros::Subscriber image_raw_sub_;
public:
    ImageConverter()
            : it_(nh_)
    {
        //nh_.getParam("/is_record", ifrecord);

        if(false)
        {
            //image_raw_sub_ = nh_.subscribe("/MVCamera/image_raw", 1, &ImageConverter::Image_cb, this);
        }


        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::Image_cb, this);
        image_depth_sub_=it_.subscribe("/camera/depth/image_rect_raw", 1, &ImageConverter::depth_imageCb, this);
        //image_ir_sub=it_.subscribe("/camera/ir/image_raw", 1, &ImageConverter::ir_imageCb, this);
        image_num_sub_=it_.subscribe("armor_detector/num_image",1, &ImageConverter::num_imageCb,this);
        coordinate_sub=it_.subscribe("armor_detector/coordinate", 1, &ImageConverter::coordinate_imageCb, this);
        top_sub=it_.subscribe("img_top", 1, &ImageConverter::top_imageCb, this);

        nh_.getParam("/dbg_img_path",dbg_img_path);
        nh_.getParam("/rcd_path", rcd_path_);

    }
    void ir_imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr3;
            cv_ptr3 = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
            img_ir = cv_ptr3 ->image;
            cv::imshow("the image",img_ir);
            waitKey(1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void top_imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            img_top = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow("top", img_top);
    }


    void num_imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            img_num = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow("num binary",img_num);
    }

    void coordinate_imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            coordinate = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow("coordinate", coordinate);
    }

    void Image_cb(const sensor_msgs::ImageConstPtr& msg)
    {

        try
        {
            cv_bridge::CvImagePtr cv_ptr1;
            cv_ptr1 = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
            img_show = cv_ptr1 ->image;
            cv::imshow("the image",img_show);
            waitKey(1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

    }

    void Image_raw_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        if(false)
        {
            try
            {
                img_raw = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }

    }

    void depth_imageCb(const sensor_msgs::ImageConstPtr& msg)
    {

        try
        {
            cv_bridge::CvImagePtr cv_ptr2;
            cv_ptr2 = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
            img_depth = cv_ptr2 ->image;
            cv::imshow("the depth_image",img_depth);
            waitKey(1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
};
int main(int argc, char **argv)
{
    ros::init(argc,argv,"img_displayer");
    ros::NodeHandle nh;
    ImageConverter ic;
    ros::spin();
    return 0;
}
