/**
 * @file /include/ui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ui_QNODE_HPP_
#define ui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN

#include <ros/ros.h>

#endif

#include "ui/rotation_recongnition.hpp"
#include "rc_msgs/results.h"
#include "rc_msgs/calibrateResult.h"
#include "rc_msgs/stepConfig.h"
#include "std_msgs/Bool.h"
#include <string>
#include <QThread>
#include <vector>
#include <QStringListModel>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <mutex>
#include <std_msgs/UInt8.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace ui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT

    public:
        int step_traffic;
        int window_step = 0;
    cv::Mat colorImg, depthImg;
        turn2 rotate;        // 修改此处切换旋转判定方案，
                            // turn为连续判定，turn2为MSE和判定
        std::mutex imageMtx;
        dynamic_reconfigure::Client<rc_msgs::stepConfig> client;
        rc_msgs::stepConfig config;

        QNode(int argc, char **argv);

        virtual ~QNode();

        bool init();

        void run();

        void rawImageCallback(const sensor_msgs::ImageConstPtr &msg);

        void rawImageDepthCallback(const sensor_msgs::ImageConstPtr &msg);

        void resultImageCallback(const rc_msgs::resultsConstPtr &msg);

        void beatCallback(const std_msgs::Bool::ConstPtr &msg);

        void nnBeatCallback(const std_msgs::Bool::ConstPtr &msg);

        void deskCallback(const rc_msgs::calibrateResult::ConstPtr &msg);

        void callback(const rc_msgs::stepConfig &config);

        void stepCallback(const std_msgs::UInt8::ConstPtr &msg);

        /*********************
        ** Logging
        **********************/
        enum LogLevel {
            Debug,
            Info,
            Warn,
            Error,
            Fatal
        };

        QStringListModel *loggingModel() { return &logging_model; }

        void log(const LogLevel &level, const std::string &msg);

        void startIdentify(int start_mode);

    Q_SIGNALS:

        void loggingUpdated();

        void rosShutdown();

        void complete();

        void updateStatus(int, bool *);

    private:
        int init_argc;
        char **init_argv;

        //ros::Publisher stepPublisher;
        ros::Publisher indentifyControler;
        ros::Publisher car_start;
        QStringListModel logging_model;
        std::vector<cv::Point> lastDesk;
        cv::Mat rotateImg, rawImage;


};

}  // namespace ui

#endif /* ui_QNODE_HPP_ */
