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
#include "recognize/yolox.h"
#include "recognize/yolov5.h"

using namespace std;
using namespace cv;
using namespace zbar;

typedef struct
{
    string data;
    string type;
    vector<Point> location;
} decodedObject;
Mat img_show;
image_processing _img;
vector<decodedObject> decode_rec;


// 查找和解码条形码和二维码
void decode(Mat &im, vector<decodedObject>&decodedObjects)
{
    im = imread("/home/nuaa/2.jpg");
    // Create zbar scanner
    zbar::ImageScanner scanner;
    // 创建 zbar 扫描仪
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    // 将图像转换为灰度
    Mat imGray;
    cvtColor(im, imGray,CV_BGR2GRAY);
    // 将图像数据包装在 zbar 图像中
    zbar::Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);
    // 扫描条形码和二维码的图像
    int n = scanner.scan(image);
    // 打印结果
    for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
    {
        decodedObject obj;
        obj.type = symbol->get_type_name();
        obj.data = symbol->get_data();
        // 打印 type 和 data
        cout << "Type : " << obj.type << endl;
        cout << "Data : " << obj.data << endl << endl;
        // 获取位置
        for(int i = 0; i< symbol->get_location_size(); i++)
        {
            obj.location.emplace_back(symbol->get_location_x(i),symbol->get_location_y(i));
        }
        decodedObjects.push_back(obj);
    }
    // 显示条码和二维码位置
    // 遍历所有解码对象
    for(int i = 0; i < decodedObjects.size(); i++)
    {
        vector<Point> points = decodedObjects[i].location;
        vector<Point> hull;
        // 如果点不形成四边形，请找到凸包
        if(points.size() > 4)
            convexHull(points, hull);
        else
            hull = points;
        // 凸包中的点数
        int n = hull.size();

        for(int j = 0; j < n; j++)
        {
            line(im, hull[j], hull[ (j+1) % n], Scalar(255,0,0), 3);
        }
    }
    // 显示结果
    imshow("Results", im);
    //waitKey(0);
}


void Image_cb(const sensor_msgs::ImageConstPtr &msg) {
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
    decode(img_show,decode_rec); // recognize the code
    decode_rec.clear(); // don't forget to clear the vector
    std::cout << "Identify Latency: " << (ros::Time::now() - start).toSec() << "s" << std::endl;

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
    ros::Subscriber DepthSub = n.subscribe("/camera/depth/image_rect_raw", 1, Depth_cb);

    ros::Time start = ros::Time::now();
    //yolov5_identify(img_show);
    std::cout << "Identify Latency: " << (ros::Time::now() - start).toSec() << "s" << std::endl;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}