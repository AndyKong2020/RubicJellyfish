//
// Created by andykong on 2023/7/14.
//
#include "scheduler/Task.h"

Task::Task(const int & task_id, const Eigen::Matrix<double, 3, 2> & pose_start, const Eigen::Matrix<double, 3, 2> & pose_end) {
    this->task_id = task_id;
    this->pose_start = pose_start;
    this->pose_end = pose_end;
}

int Task::getTaskId() const {
    return task_id;
}


void RouteTask::addToRouteList(const Eigen::Matrix<double, 3, 2> & route_point)
{
    route_list.push_back(route_point);
}

Eigen::Matrix<double, 3, 2> RouteTask::nextRoutePoint()
{
    route_index++;
    return route_list[route_index];
}

inline bool position_match(const Eigen::Matrix<double, 3, 2> & a, const Eigen::Matrix<double, 3, 2> & b)
{
    //计算两点距离
    double distance = sqrt(pow(a(0, 0) - b(0, 0), 2) + pow(a(0, 1) - b(0, 1), 2));
    return distance < 0.05;
}

inline bool orientation_match(const Eigen::Matrix<double, 3, 2> & a, const Eigen::Matrix<double, 3, 2> & b)
{
    //计算两点角度差
    double angle = angles::shortest_angular_distance(a(0, 2), b(0, 2));
    return abs(angle) < 0.1;
}

inline bool pose_match(const Eigen::Matrix<double, 3, 2> & a, const Eigen::Matrix<double, 3, 2> & b)
{
    return position_match(a, b) && orientation_match(a, b);
}

inline double position_distance(const Eigen::Matrix<double, 3, 2> & a, const Eigen::Matrix<double, 3, 2> & b)
{
    return sqrt(pow(a(0, 0) - b(0, 0), 2) + pow(a(0, 1) - b(0, 1), 2));
}

inline double orientation_distance(const Eigen::Matrix<double, 3, 2> & a, const Eigen::Matrix<double, 3, 2> & b)
{
    return abs(angles::shortest_angular_distance(a(0, 2), b(0, 2)));
}

Eigen::Matrix<double, 3, 2> RouteTask::findCurrentRoutePoint(const Eigen::Matrix<double, 3, 2> & self_pose)
{
//找到距离最近的点
    int min_index = 0;
    double min_distance = 1000;//magic number, don't change
    for (int i = 0; i < route_list.size(); i++)
    {
        double distance = position_distance(self_pose, route_list[i]);
        if (distance < min_distance)
        {
            min_distance = distance;
            min_index = i;
        }
    }

    route_index = min_index;
    return route_list[min_index];
}

int RouteTask::getCurrentRouteIndex() const
{
    return route_index;
}

Eigen::Matrix<double, 3, 2> RouteTask::getCurrentRoutePoint() const
{
    return route_list[route_index];
}

int RouteTask::getRouteListSize() const
{
    return (int)route_list.size();
}

Eigen::Matrix<double, 3, 2> RouteTask::runTask() {
    route_index = 0;
    while(route_index < route_list.size())
    {
        if (pose_match(drone.getPose(), route_list[route_index]))
        {
            return nextRoutePoint();
        }
        else
        {
            return route_list[route_index];
        }
    }
}

