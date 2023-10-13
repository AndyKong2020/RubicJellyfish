/**
 * @file /include/ui/main_window.hpp
 *
 * @brief Qt based gui for ui.
 *
 * @date November 2010
 **/
#ifndef ui_MAIN_WINDOW_H
#define ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QTimer>


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
  void colorImageDisplay(cv::Mat&);
  void depthImageDisplay(cv::Mat&);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_button_connect_clicked(bool check );
  void on_button_quit_clicked() {close();}

  /******************************************
  ** Manual connections
  *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically
  void release();
  void updateStatusShow(int, bool*);
  void TimeOutUpdateSlot();

private:
	Ui::MainWindowDesign ui{};
	QNode qnode;
  QTimer *timer;
};

}  // namespace ui

#endif // ui_MAIN_WINDOW_H
