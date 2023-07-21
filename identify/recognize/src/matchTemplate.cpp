//
// Created by nuaa on 23-7-17.
//
#include "recognize/matchTemplate.h"

EasyTemplate::EasyTemplate()
{
    gPryDownUsed = false;
}


EasyTemplate::~EasyTemplate()
{

}

bool EasyTemplate::Mark(cv::Mat &Img, bool pryDown)
{
    //确保有图像
    if (Img.empty())
        return false;

    //确保输入的是单通道图像
    if (Img.channels() != 1)
        cv::cvtColor(Img, gTemplate, cv::COLOR_BGR2GRAY);
    else
        gTemplate = Img.clone();

    //是否使用降低采样
    gPryDownUsed = pryDown;

    //返回结果
    return true;
}


bool EasyTemplate::Match(cv::Mat &Img, cv::Rect &roi, float &score)
{
    score = 0.0;
    roi = cv::Rect(-1, -1, 0, 0);
    //确保有图像
    if (Img.empty() || gTemplate.empty())
        return false;

    //模板尺寸小于被识别图像
    if (Img.cols < gTemplate.cols || Img.rows < gTemplate.rows)
        return false;

    //确保输入的是单通道图像
    cv::Mat GrayImg;
    if (Img.channels() != 1)
        cv::cvtColor(Img, GrayImg, cv::COLOR_BGR2GRAY);
    else
        GrayImg = Img.clone();

    //用于直方图计算
    cv::Mat ScoreImg = GrayImg.clone();
    //用于模板匹配计算
    cv::Mat TempImg = gTemplate.clone();
    //使用降低采样
    if (gPryDownUsed)
    {
        for (int i = 0; i < DOWN_LEVEL; i++)
        {
            cv::pyrDown(GrayImg, GrayImg, cv::Size(GrayImg.cols / 2, GrayImg.rows / 2));
            cv::pyrDown(TempImg, TempImg, cv::Size(TempImg.cols / 2, TempImg.rows / 2));
        }
    }

    //结果图像,要求32f
    int rows = GrayImg.rows - TempImg.rows + 1;  //结果图像的行数：W-w+1，W：原图像的行数，w：模板图像的行数
    int cols = GrayImg.cols - TempImg.cols + 1;  //结果图像的列数：H-h+1，H：原图像的列数，h：模板图像的列数
    //创建结果图像
    cv::Mat ResultImg(rows, cols, CV_32FC1);
    //开始模板匹配
    //TM_CCOEFF_NORMED：标准相关性系数匹配，数字越大越好
    //TM_SQDIFF：平方差匹配，约接近0越好
    cv::matchTemplate(GrayImg, TempImg, ResultImg, cv::TM_CCORR_NORMED);
    //最大值
    double maxValue;
    cv::Point maxLocation;
    ///获取最大、最小像素值及其位置
    minMaxLoc(ResultImg, 0, &maxValue, 0, &maxLocation);

    //剔除
    if (maxValue < TOLERANCE_MAX)
        return false;

    //获取区域
    roi.x = maxLocation.x;
    roi.y = maxLocation.y;
    roi.width = TempImg.cols;
    roi.height = TempImg.rows;

    //使用降低采样
    if (gPryDownUsed)
    {
        roi.x = roi.x*pow(2, DOWN_LEVEL);
        roi.y = roi.y*pow(2, DOWN_LEVEL);
        roi.width = roi.width*pow(2, DOWN_LEVEL);
        roi.height = roi.height*pow(2, DOWN_LEVEL);
    }

    //这里使用直方图做相似度处理
    //直方图比较
    const int channels[1] = { 0 }; //通道索引
    float inRanges[2] = { 0,255 };  //像素范围
    const float* ranges[1] = { inRanges };//像素灰度级范围
    const int bins[1] = { 256 };   //直方图的维度

    cv::Mat hist1, hist2;
    //计算直方图并且归一化
    cv::calcHist(&gTemplate, 1, channels, cv::Mat(), hist1, 1, bins, ranges);
    cv::normalize(hist1, hist1, 1, 0, cv::NORM_L1);
    //计算直方图并且归一化
    cv::Mat _img = ScoreImg(roi).clone();
    cv::calcHist(&_img, 1, channels, cv::Mat(), hist2, 1, bins, ranges);
    cv::normalize(hist2, hist2, 1, 0, cv::NORM_L1);
    //匹配计算相似度
    float similarity =cv::compareHist(hist1,hist2,cv::HISTCMP_INTERSECT);
    //权值计算
    score = similarity;
    //返回结果
    return true;
}

//图像旋转一定角度，
//坏处：轮廓边缘增强了，需要处理边缘
cv::Mat ImageRotate(cv::Mat Img, double Angle)
{
    cv::Mat Rotated;
    double alpha = -Angle * CV_PI / 180.0;//convert angle to radian format

    cv::Point2f srcP[3];
    cv::Point2f dstP[3];

    srcP[0] = cv::Point2f(0, Img.rows);
    srcP[1] = cv::Point2f(Img.cols, 0);
    srcP[2] = cv::Point2f(Img.cols, Img.rows);

    //rotate the pixels
    for (int i = 0; i<3; i++)
        dstP[i] = cv::Point2f(srcP[i].x*cos(alpha) - srcP[i].y*sin(alpha), srcP[i].y*cos(alpha) + srcP[i].x*sin(alpha));

    double minx, miny, maxx, maxy;
    minx = std::min(std::min(std::min(dstP[0].x, dstP[1].x), dstP[2].x), float(0.0));
    miny = std::min(std::min(std::min(dstP[0].y, dstP[1].y), dstP[2].y), float(0.0));
    maxx = std::max(std::max(std::max(dstP[0].x, dstP[1].x), dstP[2].x), float(0.0));
    maxy = std::max(std::max(std::max(dstP[0].y, dstP[1].y), dstP[2].y), float(0.0));

    int w = maxx - minx;
    int h = maxy - miny;

    //translation
    for (int i = 0; i < 3; i++)
    {
        if (minx < 0)
            dstP[i].x -= minx;
        if (miny < 0)
            dstP[i].y -= miny;
    }
    //旋转图像
    cv::Mat warpMat = cv::getAffineTransform(srcP, dstP);
    cv::warpAffine(Img, Rotated, warpMat, cv::Size(w, h),cv::INTER_NEAREST,cv::BORDER_CONSTANT,cv::Scalar(0));
    return Rotated;
}

//旋转图像，边缘处理
void UpdateImgAngle(const cv::Mat& src, cv::Mat& mask, float angle, cv::Mat& dst, cv::Mat &dstmask, cv::RotatedRect &rotate)
{
    cv::Mat updated;
    updated = ImageRotate(src, angle);
    dstmask = ImageRotate(mask, angle);
    rotate = cv::RotatedRect(cv::Point(updated.cols/2, updated.rows/2),cv::Size(src.cols, src.rows), -angle);
    //处理一下边缘
    cv::bitwise_and(updated, updated, dst, dstmask);
}

bool EasyTemplate::Match(cv::Mat & Img, cv::RotatedRect & roi, float & score)
{
    score = 0.0;
    roi = cv::RotatedRect(cv::Point(-1,-1),cv::Size(0,0),0);
    //确保有图像
    if (Img.empty() || gTemplate.empty())
        return false;

    //模板尺寸小于被识别图像
    if (Img.cols < gTemplate.cols || Img.rows < gTemplate.rows)
        return false;

    //确保输入的是单通道图像
    cv::Mat GrayImg;
    if (Img.channels() != 1)
        cv::cvtColor(Img, GrayImg, cv::COLOR_BGR2GRAY);
    else
        GrayImg = Img.clone();

    //用于直方图计算
    cv::Mat ScoreImg = GrayImg.clone();
    //创建 mask 图像
    cv::Mat gTemplateMask(gTemplate.rows, gTemplate.cols,CV_8UC1,cv::Scalar::all(255));
    cv::Mat TempImg = gTemplate.clone();
    cv::Mat TempMaskImg = gTemplateMask.clone();
    //使用降低采样
    if (gPryDownUsed)
    {
        for (int i = 0; i < DOWN_LEVEL; i++)
        {
            cv::pyrDown(GrayImg, GrayImg, cv::Size(GrayImg.cols / 2, GrayImg.rows / 2));
            cv::pyrDown(TempImg, TempImg, cv::Size(TempImg.cols / 2, TempImg.rows / 2));
            cv::pyrDown(TempMaskImg, TempMaskImg, cv::Size(TempMaskImg.cols / 2, TempMaskImg.rows / 2));
        }
    }
    //步长(每一次检测后增加的角度)
    const int angle_step = 2;
    const int match_size = 360 / angle_step;
    //一共需要匹配match_size次
    float Angle_Max = 0;
    double Score_Max = 0.0f;
    cv::Size Size_Max;
    cv::Point Location_Max;
    for (int i = 0; i < match_size; i++)
    {
        //改变图像
        cv::RotatedRect rotate;
        cv::Mat nTemplate, nTemplateMask;
        UpdateImgAngle(TempImg, TempMaskImg,i*angle_step, nTemplate, nTemplateMask, rotate);

        //模板尺寸小于被识别图像
        if (GrayImg.cols < TempImg.cols || GrayImg.rows < TempImg.rows)
            continue;

        //
        //结果图像,要求32f
        int rows = GrayImg.rows - nTemplate.rows + 1;  //结果图像的行数：W-w+1，W：原图像的行数，w：模板图像的行数
        int cols = GrayImg.cols - nTemplate.cols + 1;  //结果图像的列数：H-h+1，H：原图像的列数，h：模板图像的列数
        //创建结果图像
        cv::Mat ResultImg(rows, cols, CV_32FC1);
        //开始模板匹配
        //TM_CCOEFF_NORMED：标准相关性系数匹配，数字越大越好
        //TM_SQDIFF：平方差匹配，约接近0越好
        cv::matchTemplate(GrayImg, nTemplate, ResultImg, cv::TM_CCORR_NORMED, nTemplateMask);
        //最大值
        double maxValue = 0.0f;;
        cv::Point maxLocation(-1,-1);
        ///获取最大、最小像素值及其位置
        cv::minMaxLoc(ResultImg, 0, &maxValue, 0, &maxLocation);

        //剔除较小的
        if (maxValue < TOLERANCE_MAX)
            continue;
        //
        if (maxValue > Score_Max)
        {
            //更新信息
            Score_Max = maxValue;
            //尺寸、角度、最佳匹配点
            Size_Max = nTemplate.size();
            Angle_Max = i*angle_step;
            Location_Max = maxLocation;
        }
    }
    cv::Size tempSize(gTemplate.cols, gTemplate.rows);
    //获取ROI
    cv::Rect matchedROI(Location_Max.x, Location_Max.y, Size_Max.width, Size_Max.height);
    //使用降低采样
    if (gPryDownUsed)
    {
        matchedROI.x = matchedROI.x*pow(2, DOWN_LEVEL);
        matchedROI.y = matchedROI.y*pow(2, DOWN_LEVEL);
        matchedROI.width = matchedROI.width*pow(2, DOWN_LEVEL);
        matchedROI.height = matchedROI.height*pow(2, DOWN_LEVEL);
    }
    //根据ROI中心点，创建旋转矩形
    roi = cv::RotatedRect(cv::Point(matchedROI.x + matchedROI.width / 2, matchedROI.y + matchedROI.height / 2), tempSize, -Angle_Max);

    //这里使用直方图做相似度处理
    //直方图比较
    const int channels[1] = { 0 }; //通道索引
    float inRanges[2] = { 0,255 };  //像素范围
    const float* ranges[1] = { inRanges };//像素灰度级范围
    const int bins[1] = { 256 };   //直方图的维度

    cv::Mat nTemplate, nTemplateMask;
    cv::RotatedRect rot;
    UpdateImgAngle(gTemplate, gTemplateMask, Angle_Max, nTemplate, nTemplateMask, rot);

    cv::Mat hist1, hist2;
    //计算直方图并且归一化
    cv::calcHist(&nTemplate, 1, channels, nTemplateMask, hist1, 1, bins, ranges);
    cv::normalize(hist1, hist1, 1, 0, cv::NORM_L1);
    //计算直方图并且归一化
    cv::Mat _img2 = ScoreImg(matchedROI).clone();
    cv::calcHist(&_img2, 1, channels, cv::Mat(), hist2, 1, bins, ranges);
    cv::normalize(hist2, hist2, 1, 0, cv::NORM_L1);
    //匹配计算相似度
    float similarity = cv::compareHist(hist1, hist2, cv::HISTCMP_INTERSECT);
    //权值计算
    score = similarity;
    //返回结果
    return true;
}

void getNextMaxLoc(cv::Mat & result, cv::Point preMaxLocation, cv::Size sizeTemplate, double& dMaxValue, cv::Point &maxLocation)
{
    //以当前点为中心
    int iStartX = std::max(preMaxLocation.x - sizeTemplate.width,0);
    int iStartY = std::max(preMaxLocation.y - sizeTemplate.height,0);
    //涂黑
    cv::rectangle(result, cv::Rect(iStartX, iStartY, 2 * sizeTemplate.width , 2 * sizeTemplate.height), cv::Scalar::all(-1), -1);
    //得到下一个最大值
    cv::minMaxLoc(result, 0, &dMaxValue, 0, &maxLocation);
}

bool EasyTemplate::Match(cv::Mat & Img, std::vector<MatchResult> &results, double tolerance)
{
    results.clear();
    //确保有图像
    if (Img.empty() || gTemplate.empty())
        return false;

    //模板尺寸小于被识别图像
    if (Img.cols < gTemplate.cols || Img.rows < gTemplate.rows)
        return false;

    //确保输入的是单通道图像
    cv::Mat GrayImg;
    if (Img.channels() != 1)
        cv::cvtColor(Img, GrayImg, cv::COLOR_BGR2GRAY);
    else
        GrayImg = Img.clone();

    //用于直方图计算
    cv::Mat ScoreImg = GrayImg.clone();
    //创建 mask 图像
    cv::Mat gTemplateMask(gTemplate.rows, gTemplate.cols, CV_8UC1, cv::Scalar::all(255));
    cv::Mat TempImg = gTemplate.clone();
    cv::Mat TempMaskImg = gTemplateMask.clone();
    //使用降低采样
    if (gPryDownUsed)
    {
        for (int i = 0; i < DOWN_LEVEL; i++)
        {
            cv::pyrDown(GrayImg, GrayImg, cv::Size(GrayImg.cols / 2, GrayImg.rows / 2));
            cv::pyrDown(TempImg, TempImg, cv::Size(TempImg.cols / 2, TempImg.rows / 2));
            cv::pyrDown(TempMaskImg, TempMaskImg, cv::Size(TempMaskImg.cols / 2, TempMaskImg.rows / 2));
        }
    }
    //步长(每一次检测后增加的角度)
    const int angle_step = 2;
    const int match_size = 360 / angle_step;
    //一共需要匹配match_size次
    for (int i = 0; i < match_size; i++)
    {
        //改变图像
        cv::RotatedRect rotate;
        cv::Mat nTemplate, nTemplateMask;
        UpdateImgAngle(TempImg, TempMaskImg, i*angle_step, nTemplate, nTemplateMask, rotate);

        //模板尺寸小于被识别图像
        if (GrayImg.cols < TempImg.cols || GrayImg.rows < TempImg.rows)
            continue;
        //
        //结果图像,要求32f
        int rows = GrayImg.rows - nTemplate.rows + 1;  //结果图像的行数：W-w+1，W：原图像的行数，w：模板图像的行数
        int cols = GrayImg.cols - nTemplate.cols + 1;  //结果图像的列数：H-h+1，H：原图像的列数，h：模板图像的列数
        //创建结果图像
        cv::Mat ResultImg(rows, cols, CV_32FC1);
        //开始模板匹配
        //TM_CCOEFF_NORMED  TM_CCORR_NORMED：数字越大越好
        //TM_SQDIFF：平方差匹配，约接近0越好
        cv::matchTemplate(GrayImg, nTemplate, ResultImg, cv::TM_CCORR_NORMED, nTemplateMask);
        //最大值
        double maxValue = 0.0f;;
        cv::Point maxLocation(-1, -1);
        ///获取最大、最小像素值及其位置
        cv::minMaxLoc(ResultImg, 0, &maxValue, 0, &maxLocation);
        //剔除较小的
        if (maxValue < tolerance)
            continue;
        //
        while (true)
        {
            cv::Rect matchedROI(maxLocation.x, maxLocation.y, nTemplate.cols, nTemplate.rows);
            //使用降低采样
            if (gPryDownUsed)
            {
                matchedROI.x = matchedROI.x*pow(2, DOWN_LEVEL);
                matchedROI.y = matchedROI.y*pow(2, DOWN_LEVEL);
                matchedROI.width = matchedROI.width*pow(2, DOWN_LEVEL);
                matchedROI.height = matchedROI.height*pow(2, DOWN_LEVEL);
            }
            //
            //根据ROI中心点，创建旋转矩形
            cv::RotatedRect roi = cv::RotatedRect(cv::Point(matchedROI.x + matchedROI.width / 2, matchedROI.y + matchedROI.height / 2), gTemplate.size(), -i*angle_step);
            //剔除遮挡的
            bool isOverlap = false;
            for(MatchResult match:results)
            {
                cv::Rect temp=match.ROI.boundingRect() & roi.boundingRect();
                if (temp.area() != 0)
                {
                    isOverlap = true;
                    break;
                }
            }
            //找到好的
            if (!isOverlap)
            {
                MatchResult re(maxValue, roi);
                results.push_back(re);
            }
            //获取下一个
            getNextMaxLoc(ResultImg, maxLocation, nTemplate.size(), maxValue, maxLocation);
            if (maxValue < tolerance)
                break;
        }
    }

    return true;
}