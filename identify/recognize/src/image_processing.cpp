//
// Created by nuaa on 23-7-2.
//
#include "recognize/image_processing.h"

const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}
int image_processing::tool_tohsv(const Mat& Img){
    namedWindow(window_capture_name);
    namedWindow(window_detection_name);
    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    Mat frame, frame_HSV, frame_threshold;
    while (true) {
        frame = Img;
        if(frame.empty())
        {
            break;
        }
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        // Show the frames
        imshow(window_capture_name, frame);
        imshow(window_detection_name, frame_threshold);
        char key = (char) waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }
    }
    return 0;
}
int image_processing::image_threshold(const Mat& srcImg){
    //	srcImg原图
    Mat midImg,frame_threshold,dilation_dst;
    Mat dstImg = srcImg.clone();

    // Convert from BGR to HSV colorspace
    cvtColor(srcImg, midImg, COLOR_BGR2HSV);
    // Detect the object based on HSV Range Values
    inRange(midImg, Scalar(0, 157, 0), Scalar(180, 255, 255), frame_threshold);
    //	灰度化
    //cvtColor(frame_threshold, midImg,COLOR_BGR2GRAY);     //灰度图
    //	中值滤波
    medianBlur(frame_threshold, midImg, 9);               //滤波后
    //	二值化
    //adaptiveThreshold(midImg, midImg, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 1);
    //threshold(midImg, midImg, 190, 255, 0);
    namedWindow("【二值图】", WINDOW_NORMAL);
    imshow("【二值图】", midImg);
    //	形态学滤波，开运算
    int element_size =10;
    int s = element_size * 2 + 1;
    Mat structureElement = getStructuringElement(MORPH_RECT, Size(s, s), Point(-1, -1));
    dilate(midImg, midImg, structureElement, Point(-1, -1), 1);
    namedWindow("【P】", WINDOW_NORMAL);
    imshow("【P】", midImg);
//    Mat element = getStructuringElement(MORPH_RECT,  Size( 2*0 + 1,
//                                                           2*0+1 ),
//                                        Point( 0, 0 ));
//    dilate( frame_threshold, dilation_dst, element );
//    imshow( "Dilation Demo", dilation_dst );
//    //morphologyEx(frame_threshold, midImg, MORPH_OPEN, element);
//    namedWindow("【开运算后】", WINDOW_NORMAL);
//    imshow("【开运算后】", midImg);
    vector<vector<Point>> contours;
    findContours(midImg, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    Mat midImg1 = Mat::zeros(midImg.rows, midImg.cols, CV_8UC3);
    for (int i = 0; i < contours.size(); i++)
    {
        Scalar color(255, 255, 255);
        drawContours(midImg1, contours, i, color, 2);
    }
    namedWindow("【轮廓图】", WINDOW_NORMAL);
    imshow("【轮廓图】", midImg1);


    for (int i = 0; i < contours.size(); i++)
    {
        //每个轮廓
        vector<Point> points = contours[i];
        //对给定的2D点集，寻找最小面积的包围矩形
        RotatedRect box = minAreaRect(Mat(points));
        Point2f vertex[4];
        //将box 中存储的4 个顶点的坐标 存储到vertex[0]~vertex[3]中去
        box.points(vertex);

        //打印中心点位置及外接矩形角度
        cout << "中心点位置（第" << i << "条轮廓）：" << box.center << endl;
        cout << "外接矩形角度（第" << i << "条轮廓）：" << box.angle << endl;

        //绘制出最小面积的包围矩形
        line(dstImg, vertex[0], vertex[1], Scalar(200, 255, 200), 3, LINE_AA);
        line(dstImg, vertex[1], vertex[2], Scalar(200, 255, 200), 3, LINE_AA);
        line(dstImg, vertex[2], vertex[3], Scalar(200, 255, 200), 3, LINE_AA);
        line(dstImg, vertex[3], vertex[0], Scalar(200, 255, 200), 3, LINE_AA);

        //绘制中心的光标，为了容易理解，此处为手动计算中心点，也可以直接使用 box.center
        Point center, l, r, u, d;
        center.x = (vertex[0].x + vertex[2].x) / 2.0;
        center.y = (vertex[0].y + vertex[2].y) / 2.0;
        l.x = center.x - 10;
        l.y = center.y;
        r.x = center.x + 10;
        r.y = center.y;
        u.x = center.x;
        u.y = center.y - 10;
        d.x = center.x;
        d.y = center.y + 10;
        line(dstImg, l, r, Scalar(200, 255, 200), 2, LINE_AA);
        line(dstImg, u, d, Scalar(200, 255, 200), 2, LINE_AA);
    }
    namedWindow("【最小包围矩形及获取中心点】", WINDOW_NORMAL);
    imshow("【最小包围矩形及获取中心点】", dstImg);
}
//用作对原图像进行处理，讲处理后的图像送入神经网络识别端。可保存样本图片，记得修改初始计数值
void image_processing::Picture_process(image_processing &image) {
    Mat img;

//    for(int _row=0; _row < img_gray.rows; _row++){
//        for(int _col=0; _col < img_gray.cols; _col++){
//            if(img_gray.at<uchar>(_row,_col) < otsuThreshold(img_gray))
//                img_gray.at<uchar>(_row,_col) = 0;
//        }
//    }
//    // 低通滤波
//    static float thresh_ = threshold;
//    if (::isnan(threshold) || ::isinf(threshold)) {
//        threshold = thresh_;
//    } else {
//        threshold = a * thresh_ + (1 - a) * threshold;
//    }
//
//    if (threshold > maxThresh) {
//        threshold = maxThresh;
//    }
//    img_gray *= threshold;
//        NUM_bgr = img_gray.clone();
        // 保存装甲板图片用于训练神经网络
//        static unsigned int noooo = 121653;
//        cv::imwrite(std::string("/home/nuaa/shootttt/") + std::to_string(noooo) + ".png", NUM_bgr);
//        noooo++;
    image.typeCall = modelManager.infer_async(img);
}

