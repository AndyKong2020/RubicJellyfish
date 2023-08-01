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
image_processing depth_img;
vector<decodedObject> decode_rec;
image_processing _img;
Mat test1 = imread("/home/robin/1.jpg");
Mat test2 = imread("/home/nuaa/wolf_big.jpg");
ov::CompiledModel final_model;
float d_value = 0;
cv::Rect img_res;
float d_res;
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

/*             //barcode and qrcode

    //_img.decode(img_show,decode_rec); // recognize the code
    //decode_rec.clear(); // don't forget to clear the vector

*/

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
    if(final_box.center.x != 0 && final_box.center.y != 0 && final_box.boundingRect().area() >= 2000) {
        img_res.x = final_box.center.x;
        img_res.y = final_box.center.y;
        d_res = d_value;
    }
    _image.mode = 1;
    _image.type = 1;
    _image.x = img_res.x;
    _image.y = img_res.y;
    _image.depth = d_res;
    depth_img.target.x = img_res.x;
    depth_img.target.y = img_res.y;
    cout<< "Value of depth_pic's pixel= "<<_image.depth<<endl;

    if(_image.x !=0 && _image.y != 0 && _image.depth != 0)
        image_pub.publish(_image);
    std::cout << "Identify Latency: " << (ros::Time::now() - start).toSec() << "s" << std::endl;
}

void Depth_cb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr Dest ;
    Dest = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth_pic = Dest->image;
    ushort d = depth_pic.at<ushort>(depth_img.target);           //读取深度值，数据类型为ushort单位为ｍｍ
    d_value = float(d)/10 ;      //强制转换
    //cout<< "Value of depth_pic's pixel= "<<d_value<<endl;

}
int main(int argc, char **argv) {
    ros::init(argc, argv, "recognize");
    ros::NodeHandle n;
    ros::Subscriber resultsSub = n.subscribe("/d435/color/image_raw", 20, &Image_cb);
    ros::Subscriber DepthSub = n.subscribe("/d435/aligned_depth_to_color/image_raw", 20, &Depth_cb);
    image_pub = n.advertise<recognize::image>("/image/write", 20);

    final_model = yolo_init("/home/robin/yolov5/runs/train/exp2/weights/best.xml");
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
    //decode_dis(test1);
    //_img.decode(test1,decode_rec);
    //std::cout << "Identify Latency: " << (ros::Time::now() - start).toSec() << "s" << std::endl;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}