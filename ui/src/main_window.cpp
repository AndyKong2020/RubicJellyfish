#include <QtWidgets>
#include <QApplication>
#include <QMessageBox>
#include <iostream>
#include <QMetaType>
#include "../include/ui/main_window.hpp"
#include "../include/ui/qnode.hpp"

namespace ui {

    using namespace Qt;

// 主窗口初始化
    MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
            : QMainWindow(parent), qnode(argc, argv) {
        // 注册信号类型
        qRegisterMetaType<cv::Mat>("cv::Mat");
        qRegisterMetaType<QVector<int>>("QVector<int>");
        qRegisterMetaType<bool *>("bool*");

        timer = new QTimer;
        timer->setInterval(40);

        // 绘制UI，设置图标
        ui.setupUi(this);
        ui.color_image->setAlignment(Qt::AlignCenter);
        ui.depth_image->setAlignment(Qt::AlignCenter);
        QString iconPath = QString::fromStdString(std::string(RESOURCE_PATH) + "icon.png");
        setWindowIcon(QIcon(iconPath));

        // 连接槽函数
        QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
        QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
        QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
        QObject::connect(&qnode, SIGNAL(complete()), this, SLOT(release()));
        QObject::connect(&qnode, SIGNAL(updateStatus(int, bool * )), this, SLOT(updateStatusShow(int, bool * )));
        QObject::connect(timer, SIGNAL(timeout()), this, SLOT(TimeOutUpdateSlot()));

        // 设置日志流
        ui.view_logging->setModel(qnode.loggingModel());

        // 初始化ros线程
        qnode.init();
        timer->start();
    }

    MainWindow::~MainWindow() = default;

/*****************************************************************************
** 槽函数
*****************************************************************************/

    void MainWindow::TimeOutUpdateSlot() {
        if (!qnode.colorImg.empty()) {
            colorImageDisplay(qnode.colorImg);
        }
        if (!qnode.depthImg.empty()) {
            depthImageDisplay(qnode.depthImg);
            //colorImageDisplay(qnode.depthImg);
        }
    }

// 开始按钮
    void MainWindow::on_button_connect_clicked(bool check) {
        if (ui.identifyRoundOne->isChecked() || ui.identifyRoundTwo->isChecked() || ui.measureRoundOne->isChecked() ||
            ui.measureRoundTwo->isChecked()) {
            ui.identifyRoundOne->setEnabled(false);
            ui.identifyRoundTwo->setEnabled(false);
            ui.measureRoundOne->setEnabled(false);
            ui.measureRoundTwo->setEnabled(false);
            ui.button_connect->setEnabled(false);
            if (ui.identifyRoundOne->isChecked()) {
                qnode.log(QNode::Info, "Patrol Mode Start!");
                qnode.startIdentify(1);
            } else if (ui.identifyRoundTwo->isChecked()) {
                qnode.log(QNode::Info, "Rescue Mode start!");
                qnode.startIdentify(2);
            } else if (ui.measureRoundOne->isChecked()) {
                qnode.log(QNode::Info, "Identify Picture Mode start!");
                qnode.startIdentify(3);
            } else if (ui.measureRoundTwo->isChecked()) {
                qnode.log(QNode::Info, "Free Mode start!");
                qnode.startIdentify(4);
            }
        } else {
            qnode.log(ui::QNode::Error, "Please select mode");
        }

    }

// 释放运行锁
    void MainWindow::release() {
        ui.identifyRoundOne->setEnabled(true);
        ui.identifyRoundTwo->setEnabled(true);
        ui.measureRoundOne->setEnabled(true);
        ui.measureRoundTwo->setEnabled(true);
        ui.button_connect->setEnabled(true);
    }

// 彩色图像显示
    void MainWindow::colorImageDisplay(cv::Mat &raw) {
        cv::Mat temp;
        QImage img;
        qnode.imageMtx.lock();
        cv::cvtColor(raw, temp, CV_BGR2RGB);
        img = QImage((const unsigned char *) (temp.data),
                     temp.cols, temp.rows,
                     temp.cols * temp.channels(),
                     QImage::Format_RGB888);
        qnode.imageMtx.unlock();
        img = img.scaled(ui.color_image->size(), Qt::KeepAspectRatio);
        ui.color_image->setPixmap(QPixmap::fromImage(img));
        if(qnode.window_step == 42){
            ui.status_runing->setText("Slow Down");
        }
    }

// 深度图像显示
    void MainWindow::depthImageDisplay(cv::Mat &raw) {
        cv::Mat temp;
        QImage img;
        qnode.imageMtx.lock();
        cv::cvtColor(raw, temp, CV_BGR2RGB);
        img = QImage((const unsigned char *) (temp.data),
                     temp.cols, temp.rows,
                     temp.cols * temp.channels(),
                     QImage::Format_RGB888);
        qnode.imageMtx.unlock();
        img = img.scaled(ui.depth_image->size(), Qt::KeepAspectRatio);
        ui.depth_image->setPixmap(QPixmap::fromImage(img));

//
//        img = img.scaled(ui.color_image->size(), Qt::KeepAspectRatio);
//        ui.color_image->setPixmap(QPixmap::fromImage(img));
    }

// 日志自动滚至底端
    void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
    }

// 退出按钮
    void MainWindow::closeEvent(QCloseEvent *event) {
        QMainWindow::closeEvent(event);
    }

// 根据心跳包等数据更新各节点状态及运行状态信息
    void MainWindow::updateStatusShow(int step, bool *sss) {
        switch (step) {
            case -1:
                ui.status_runing->setText("Error happen!!!");
                break;
            case 0:
                ui.status_runing->setText("Running");
                break;
            case 1:
                ui.status_runing->setText("Turning Right");
                break;
            case 2:
                ui.status_runing->setText("Turning Left");
                break;
            case 3:
                ui.status_runing->setText("Stop");
                break;
            case 4:
                ui.status_runing->setText("Slow Down");
                break;
            case 5:
                ui.status_runing->setText("Speed Up");
                break;
            case 6:
                ui.status_runing->setText("Turning");
                break;
            case 7:
                ui.status_runing->setText("Identifying turntable");
                break;
            case 8:
                ui.status_runing->setText("Complete");
                break;
        }
        if (sss[0]) {
            ui.status_mvdriver->setText("mv_driver connected");
        } else {
            ui.status_mvdriver->setText("mv_driver not connect");
        }
        if (sss[1]) {
            ui.status_nn->setText("nn connected");
        } else {
            ui.status_nn->setText("nn not connect");
        }
        if (sss[2]) {
            ui.status_main->setText("scheduler connected");
        } else {
            ui.status_main->setText("scheduler not connect");
        }
    }

}  // 命名空间 ui 结束


