//
// Created by andykong on 23-7-11.
//
#ifndef SRC_TASKMANAGER_H
#define SRC_TASKMANAGER_H
#include <vector>
#include <Eigen/Core>
#include "ros/ros.h"

class TaskManager
{
public:
    TaskManager();
    void addToRouteList(const Eigen::Matrix<double, 3, 2> & route_point);
    Eigen::Matrix<double, 3, 2> nextRoutePoint();
    void nextTask();
    Eigen::Matrix<double, 3, 2> setCurrentRoutePoint(const Eigen::Matrix<double, 3, 2> & self_pose);
    int getCurrentRouteIndex();
    int getCurrentTask();
    void reset();
private:
    std::vector<Eigen::Matrix<double, 3, 2>> route_list;
    int route_index;
    int task_index;
};
#endif //SRC_TASKMANAGER_H
