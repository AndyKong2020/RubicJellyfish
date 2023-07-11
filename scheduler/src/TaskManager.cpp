//
// Created by andykong on 23-7-11.
//
#include "scheduler/TaskManager.h"


TaskManager::TaskManager()
{
    route_index = 0;
    task_index = 0;
    ros::param::
}

void TaskManager::addToRouteList(const Eigen::Matrix<double, 3, 2> & route_point)
{
    route_list.push_back(route_point);
}

Eigen::Matrix<double, 3, 2> TaskManager::nextRoutePoint()
{
    route_index++;
    return route_list[route_index];
}

void TaskManager::nextTask()
{
    task_index++;
}

Eigen::Matrix<double, 3, 2> TaskManager::setCurrentRoutePoint(const Eigen::Matrix<double, 3, 2> & self_pose)
{

}

int TaskManager::getCurrentRouteIndex()
{
    return route_index;
}

int TaskManager::getCurrentTask()
{
    return task_index;
}

void TaskManager::reset()
{
    route_index = 0;
    task_index = 0;
    route_list.clear();
}