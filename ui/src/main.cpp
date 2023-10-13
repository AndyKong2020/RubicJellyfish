/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets>
#include <QApplication>
#include "../include/ui/main_window.hpp"
#include <std_msgs/UInt8.h>

/*****************************************************************************
** Main
*****************************************************************************/

    int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle n;
    ui::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
