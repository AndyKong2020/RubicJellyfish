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
#include <std_msgs/UInt8.h>
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
image_processing depth_img;
vector<decodedObject> decode_rec;
image_processing _img;
ov::CompiledModel final_model;
uint8_t task_id, last_task_id = 0;
bool fire_task_flag = false;
float d_value = 0;
cv::Rect img_res;
float d_res;
uint8_t last_mode = 0;

void Image_cb(const sensor_msgs::ImageConstPtr &msg) {
    recognize::image _image;
    RotatedRect final_box;
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

    final_box = _img.image_threshold(img_show);
    //_img.tool_tohsv(img_show);


/*                     //Yolov5
    std::vector<Detection> yolov5_res = yolov5_identify(img_show,final_model);
    if(yolov5_res.size()>1 && yolov5_res.size()!=0){
        float max_con = 0;
        int max_id = 0;
        for(int i = 0; i > yolov5_res.size(); i++){
            if(yolov5_res[i].confidence>max_con){
                max_con = yolov5_res[i].confidence;
                max_id = i;
            }
        }
        final_box = yolov5_res[max_id].box;
    }else if(yolov5_res.size()==1){
        final_box = yolov5_res[0].box;
    }
*/
    std::cout << "xxx: " << final_box.center.x << std::endl;
    std::cout << "yyy: " << final_box.center.y << std::endl;
    //std::cout << "Identify Latency: " << (ros::Time::now() - start).toSec() << "s" << std::endl;

    uint8_t num = 0;
    if((last_mode = 0)){
        num = 50;
    } else if((last_mode = 1)){
        num = 0;
    }
    if(_img.image_check(final_box,_img.minsize,_img.maxsize,task_id,num,img_res)){
        d_res = d_value;
        _image.mode = 1;
    }else{
        _image.mode = 0;
    }

    _image.x = img_res.x;
    _image.y = img_res.y;
    _image.depth = d_res;
    depth_img.target.x = img_res.x;
    depth_img.target.y = img_res.y;
    cout<< "Value of depth_pic's pixel= "<<_image.depth<<endl;
    last_mode = _image.mode;

    if(_image.x == 0 || _image.y  == 0 || _image.depth == 0){
        _image.mode = 0;
    }
    image_pub.publish(_image);
    std::cout << "Identify Latency: " << (ros::Time::now() - start).toSec() << "s" << std::endl;
}

void Depth_cb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr Dest ;
    Dest = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth_pic = Dest->image;
    ushort d = depth_pic.at<ushort>(depth_img.target);           //读取深度值，数据类型为ushort单位为ｍｍ
    d_value = float(d)/1000 ;      //强制转换
    //cout<< "Value of depth_pic's pixel= "<<d_value<<endl;
}

void Task_cb(const std_msgs::UInt8 &msg) {
    task_id = msg.data;
    if((task_id == 1 && last_task_id == 2) || task_id == 14){
        fire_task_flag = true;
    }
    //fire_task_flag = true;
    if (!fire_task_flag) {
        _img.l1 = 0;
        _img.l2 = 128;
        _img.l3 = 128;
        _img.h1 = 75;
        _img.h2 = 255;
        _img.h3 = 255;
        _img.minsize = 10;
        _img.maxsize = 1800;
        _img.element = 7;
        _img.kind = 0;
    }else{
        _img.l1 = 14;
        _img.l2 = 0;
        _img.l3 = 0;
        _img.h1 = 255;
        _img.h2 = 255;
        _img.h3 = 90;
        _img.minsize = 500;
        _img.maxsize = 10000;
        _img.element = 10;
        _img.kind = 1;
    }
    last_task_id = task_id;

}
int main(int argc, char **argv) {
    ros::init(argc, argv, "recognize");
    ros::NodeHandle n;
    ros::Subscriber resultsSub = n.subscribe("/d435/color/image_raw", 20, &Image_cb);
    ros::Subscriber DepthSub = n.subscribe("/d435/aligned_depth_to_color/image_raw", 20, &Depth_cb);
    image_pub = n.advertise<recognize::image>("/image/write", 20);
    ros::Subscriber TaskSub = n.subscribe("/task_id", 20, &Task_cb);
    //final_model = yolo_init("/home/robin/yolov5/runs/train/exp2/weights/best.xml");
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}