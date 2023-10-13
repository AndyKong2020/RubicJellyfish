#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "ui/qnode.hpp"
#include <std_msgs/UInt8.h>
#include "rc_msgs/results.h"
#include "rc_msgs/stepConfig.h"


namespace ui {

    int step = 0;        // 存储当前step
    bool engineInit = false;
    bool status[3] = {false, false, false};        // 存储当前三个节点连接状态，顺序为mv、nn、main
    bool stf[3] = {false, false, false};        //发送用status
    std::string mode{"mode"};


// qt多线程初始化
    QNode::QNode(int argc, char **argv) :
            init_argc(argc),
            init_argv(argv),
            client("/scheduler", boost::bind(&QNode::callback, this, _1)) {
        qRegisterMetaType<bool>("bool");
    }

// qt多线程销毁，结束ros相关服务
    QNode::~QNode() {
        if (ros::isStarted()) {
            ros::shutdown();
            ros::waitForShutdown();
        }
        wait();
    }

    void QNode::callback(const rc_msgs::stepConfig &config) {
        step = config.step;
        mode = config.mode;
        if (step == 8) {
            Q_EMIT complete();        // 释放UI中锁定资源
            log(Info, std::string("ifend  true: ") + std::to_string(step));
            std_msgs::Bool identify;
            identify.data = false;
            indentifyControler.publish(identify);
            // 之后在这里加上识别完成后显示结果的东西
        } else if (step == 2 || step == 5) {
            rotate.updateBegin();
            log(Info, std::string("ifend  step: ") + std::to_string(step));
            std_msgs::Bool identify;
            identify.data = false;
            indentifyControler.publish(identify);
        }
    }

// 初始化ros服务
    bool QNode::init() {
        rotateImg = cv::imread(std::string(RESOURCE_PATH) + "rotate.png");
        //ros::init(init_argc, init_argv, "ui_node");
        if (!ros::master::check()) {
            return false;
        }
        start();
        ROS_WARN("-------------------------init done-----------------------");
        return true;
    }

// 运行ros服务
    void QNode::run() {
        ros::start();
        ros::NodeHandle n;
        rotate.node = &n;

        // topic相关
        ros::Subscriber resultImageSub = n.subscribe("/rcnn_results", 1,
                                                       &ui::QNode::resultImageCallback, this);
        ros::Subscriber rawImageSub = n.subscribe("/yolo_detect", 1,
                                                       &ui::QNode::rawImageCallback, this);
        ros::Subscriber rawImageDepthSub = n.subscribe("/camera/rgb/image_raw", 1,
                                                       &ui::QNode::rawImageDepthCallback, this);
        ros::Subscriber beatSub = n.subscribe("/main_beat", 1,
                                                       &ui::QNode::beatCallback, this);
        ros::Subscriber nnBeatSub = n.subscribe("/nn_beat", 1,
                                                       &ui::QNode::nnBeatCallback, this);
        ros::Subscriber deskSub = n.subscribe("/calibrateResult", 1,
                                                       &ui::QNode::deskCallback, this);
        ros::Subscriber stepSub = n.subscribe("/step", 1,
                                                           &ui::QNode::stepCallback, this);
        indentifyControler = n.advertise<std_msgs::Bool>("/isIdentify", 1);
        car_start = n.advertise<std_msgs::UInt8>("/car_start", 1);

        ros::Rate loop_rate(30);
        int count = 0;
        int stt = 0;
        int end = 8;

        // 是否在启动后就开始识别
        std_msgs::Bool identify;
        identify.data = true;
        indentifyControler.publish(identify);

        while (ros::ok()) {
            // 处理发送step
            if (step != stt && step != end) {
                stt = step;
                rotate.ss = step;

                log(Info, std::string("Now step: ") +
                          std::to_string(config.step));
                std::string ll;
                switch (step) {
                    case -1:
                        ll = "Error happen!!!";
                        break;
                    case 0:
                        ll = "Waiting";
                        break;
                    case 1:
                        ll = "Identifying desk";
                        break;
                    case 2:
                        ll = "Waiting for turning";
                        break;
                    case 3:
                        ll = "Turning";
                        break;
                    case 4:
                        ll = "Identifying desk";
                        break;
                    case 5:
                        ll = "Waiting for turning";
                        break;
                    case 6:
                        ll = "Turning";
                        break;
                    case 7:
                        ll = "Identifying turntable";
                        break;
                    case 8:
                        ll = "Complete";
                        break;
                }
                log(Info, ll);
            }

            // 更新节点连接状态(1Hz)
            count++;
            if (count >= 30) {
                //log(Info, std::to_string(status[0]));
                stf[0] = status[0];
                stf[1] = status[1];
                stf[2] = status[2];
                Q_EMIT updateStatus(step, stf);
                count = 0;
                status[0] = false;
                status[1] = false;
                status[2] = false;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown();
    }

// 启动识别函数，根据round选择决定起始step
    void QNode::startIdentify(int start_mode) {
        //mode = _mode;
        int mode = 0;
        switch (start_mode) {
            case 1:
                mode = 1;
                break;
            case 2:
                mode = 2;
                break;
            case 3:
                mode = 3;
                break;
            case 4:
                mode = 4;
                break;
        }

        //std_msgs::Bool identify;
        //identify.data = true;
        //indentifyControler.publish(identify);

        //config.step = step;
        //config.mode = mode;
        //client.setConfiguration(config);
        std_msgs::UInt8 _mode;
        _mode.data = mode;
        car_start.publish(_mode);
        ROS_INFO("Now:  step: %d    ,mode: %s",
                 config.step, config.mode.c_str());
    }

// 日志记录函数
    void QNode::log(const LogLevel &level, const std::string &msg) {
        if (logging_model.rowCount() > 200) {
            logging_model.removeRows(0, logging_model.rowCount());;
        }
        logging_model.insertRows(logging_model.rowCount(), 1);
        std::stringstream logging_model_msg;
        if (!ros::master::check()) {
            logging_model_msg << "[FATAL] " << "ROS server not connect!";
        } else {
            switch (level) {
                case (Debug) : {
                    ROS_DEBUG_STREAM(msg);
                    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                    break;
                }
                case (Info) : {
                    ROS_INFO_STREAM(msg);
                    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                    break;
                }
                case (Warn) : {
                    ROS_WARN_STREAM(msg);
                    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                    break;
                }
                case (Error) : {
                    ROS_ERROR_STREAM(msg);
                    logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                    break;
                }
                case (Fatal) : {
                    ROS_FATAL_STREAM(msg);
                    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                    break;
                }
            }
        }
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount() - 1), new_row);
        Q_EMIT loggingUpdated(); // 视角自动回到底端
    }

    void QNode::resultImageCallback(const rc_msgs::resultsConstPtr &msg) {
        if (step != 0 && step != 8 && step != 2 && step != 5 && step != 3 && step != 6) {
            imageMtx.lock();
            colorImg = cv_bridge::toCvShare(msg->color, msg, "bgr8")->image.clone();
            rawImage = colorImg.clone();
            if (!lastDesk.empty()) {
                cv::polylines(colorImg, lastDesk, 1, cv::Scalar(0xff, 0xcc, 0x66));
            }
            imageMtx.unlock();
        }
    }

    void QNode::rawImageDepthCallback(const sensor_msgs::ImageConstPtr &msg) {
        try {
            //depthImg = cv_bridge::toCvShare(msg, "mono8")->image.clone();
            depthImg = cv::imread("/home/robin/road.jpg");
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

// 获取原始图像回调
    void QNode::rawImageCallback(const sensor_msgs::ImageConstPtr &msg) {
        status[0] = true;

        //log(Warn, std::string("Test!!!!!!!!!"));
        try {
            //对colorImg进行修改即可修改ui显示结果
//            if (step == 0 || step == 8) {
//                colorImg = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
//            } else if (step == 2 || step == 5 || step == 3 || step == 6) {
//                colorImg = rotateImg;
//            }
            colorImg = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if(step == 42){
            log(Warn, std::string("!!!Slow Dowm!!!"));
        }
//        if (step == 2 || step == 3 || step == 5 || step == 6) {
//            int t = rotate.getStep(colorImg, depthImg, step);
//            if (t == 2 && (step == 3 || step == 6)) {
//                step++;
//                log(Warn, std::string("Rotating time out!!!"));
//                config.step = step;
//                client.setConfiguration(config);
//            } else if (t == 3 && (step == 2 || step == 5)) {
//                step++;
//                log(Warn, std::string("Waiting for rotating time out!!!"));
//                config.step = step;
//                client.setConfiguration(config);
//            } else if (t == 1) {
//                if (step == 3 || step == 6) {
//                    std_msgs::Bool identify;
//                    identify.data = true;
//                    indentifyControler.publish(identify);
//                }
//                step++;
//                config.step = step;
//                client.setConfiguration(config);
//            }
//            //log(Info,std::string("Now MSE: ")+std::to_string(rotate.lastMSE));
//        }
    }

    void QNode::nnBeatCallback(const std_msgs::Bool::ConstPtr &msg) {
        status[1] = true;
        if (!engineInit) {
            engineInit = true;
            log(Warn, "Engine connected");
        }
    }
    void QNode::stepCallback(const std_msgs::UInt8::ConstPtr &msg) {
        step = msg->data;
        window_step = step;
    }
// main节点心跳包回调
    void QNode::beatCallback(const std_msgs::Bool::ConstPtr &msg) {
        status[2] = true;
    }

    void QNode::deskCallback(const rc_msgs::calibrateResult::ConstPtr &msg) {
        imageMtx.lock();
        lastDesk.clear();
        lastDesk.emplace_back(msg->data[1].x, msg->data[1].y);
        lastDesk.emplace_back(msg->data[0].x, msg->data[0].y);
        lastDesk.emplace_back(msg->data[2].x, msg->data[2].y);
        lastDesk.emplace_back(msg->data[3].x, msg->data[3].y);
        colorImg = rawImage.clone();
        cv::polylines(colorImg, lastDesk, 1, cv::Scalar(0xff, 0xcc, 0x66));
        imageMtx.unlock();
    }
}  // namespace ui
