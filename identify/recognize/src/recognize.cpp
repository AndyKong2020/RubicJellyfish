//
// Created by nuaa on 23-6-28.
//
#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include ""
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <string>


#include "recognize/yolox.h"
#include "recognize/yolov5.h"


using namespace std;
using namespace cv;
//using namespace zbar;
Mat img_show;
//void BarCode(cv::Mat _image){
//    cv::Mat imgGray;
//
//    //转化为灰度图，方便运算与识别
//
//    cvtColor(_image, imgGray, COLOR_RGB2GRAY);
//
//    // 高斯平滑滤波
//    Mat imgGus;
//    GaussianBlur(imgGray, imgGus, Size(3,3), 0);
//
//    //4.求得水平和垂直方向灰度图像的梯度差,使用Sobel算子
//    Mat imageX16S, imageY16S;
//    Sobel(imgGus, imageX16S, CV_16S, 1, 0, 3, 1, 0, 4);
//    Sobel(imgGus, imageY16S, CV_16S, 0, 1, 3, 1, 0, 4);
//    convertScaleAbs(imageX16S, imageSobelX, 1, 0);
//    convertScaleAbs(imageY16S, imageSobelY, 1, 0);
//    imageSobelOut=imageSobelX-imageSobelY;
//
//
//    //5.均值滤波，消除高频噪声
//    Mat imgBlur;
//    blur(imageSobelOut, imgBlur, Size(3,3));
//
//
//    //6.二值化
//    Mat imgThreshold;
//    threshold(imgBlur,imgThreshold,80,255,THRESH_BINARY);
//
//    //闭运算，填充条码间隙
//    Mat element = getStructuringElement(0,Size(7,7));//闭运算需要的参数
//    morphologyEx(imgThreshold,imgThreshold,MORPH_CLOSE,elementmor);
//
//    // 膨胀，根据目标条形码的大小，需要多次膨胀操作，具体次数随情况变化
//    Mat  elementdli=getStructuringElement(0,Size(30,7));
//    dilate(imgThreshold,imgThreshold,elementdli);
//    dilate(imgThreshold,imgThreshold,elementdli);
//    dilate(imgThreshold,imgThreshold,elementdli);
//    dilate(imgThreshold,imgThreshold,elementdli);
//    dilate(imgThreshold,imgThreshold,elementdli);
//
//    vector<vector<Point>> contours;
//    vector<Vec4i> hiera;
//    findContours(imageSobleOutThreshold,contours,hiera,RETR_EXTERNAL,CHAIN_APPROX_NONE);
//    Mat scanimg[contours.size()];
//    for(int i=0;i<contours.size();i++)//contours is the contour of my image
//    {
//        Rect rect=boundingRect((Mat)contours[i]);//rect is a coordinate,it's has 4
//        //element
//        rectangle(image,rect,Scalar(255),2);
//        cout<<"a"<<endl;
//        scanimg[i] = image(rect);//image accoding to the rect to cut the pictrue
//        //process come to here ,things we need do to pictrue is
//        cvtColor(scanimg[i], scanimg[i], COLOR_RGB2GRAY);
//        string o = num2str(i);
//        imshow(o, scanimg[i]);
//        //already done,the next step is Identify barcode
//
//        ImageScanner scanner;
//        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
//        int width = scanimg[i].cols;
//        int height = scanimg[i].rows;
//        uchar *raw = (uchar *)scanimg[i].data;
//        Image imageZbar(width, height, "Y800", raw, width * height);
//        scanner.scan(imageZbar); //扫描条码
//        Image::SymbolIterator symbol = imageZbar.symbol_begin();
//        if(imageZbar.symbol_begin()==imageZbar.symbol_end())
//        {
//            cout<<"查询条码失败，图片清晰度不足！"<<endl;
//        }
//        for(;symbol != imageZbar.symbol_end();++symbol)
//        {
//            cout<<"类型："<<endl<<symbol->get_type_name()<<endl<<endl;
//            cout<<"条码："<<endl<<symbol->get_data()<<endl<<endl;
//        }
//    }
//}
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

    std::cout << "Identify Latency: " << (ros::Time::now() - start).toSec() << "s" << std::endl;

}
int main(int argc, char **argv) {
    ros::init(argc, argv, "recognize");
    ros::NodeHandle n;
    ros::Subscriber resultsSub = n.subscribe("/camera/color/image_raw", 1, &Image_cb);
    ros::Time start = ros::Time::now();
    yolov5_identify(img_show);
    std::cout << "Identify Latency: " << (ros::Time::now() - start).toSec() << "s" << std::endl;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}