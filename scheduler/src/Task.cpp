//
// Created by andykong on 2023/7/14.
//
#include "scheduler/Task.h"

Task::Task(const int & task_id) {
    this->task_id = task_id;
}

int Task::getTaskId() const {
    return task_id;
}


void RouteTask::addToRouteList(const DronePose & route_point)
{
    route_list.push_back(route_point);
}

DronePose RouteTask::nextRoutePoint()
{
    route_index++;
    return route_list[route_index];
}

inline bool position_match(const DronePose & a, const DronePose & b)
{
    //计算两点距离
    double distance = sqrt(pow(a.position.x() - b.position.x(), 2) + pow(a.position.y() - b.position.y(), 2));
    return distance < 0.05;
}

inline bool orientation_match(const DronePose & a, const DronePose & b)
{
    //计算两点角度差
    double angle = angles::shortest_angular_distance(a.angular_orientation.z(), b.angular_orientation.z());
    return abs(angle) < 0.1;
}

inline bool pose_match(const DronePose & a, const DronePose & b)
{
    return position_match(a, b) && orientation_match(a, b);
}

inline double position_distance(const DronePose & a, const DronePose & b)
{
    return sqrt(pow(a.position.x() - b.position.x(), 2) + pow(a.position.y() - b.position.y(), 2));
}

inline double orientation_distance(const DronePose & a, const DronePose & b)
{
    return abs(angles::shortest_angular_distance(a.angular_orientation.z(), b.angular_orientation.z()));
}

DronePose RouteTask::findCurrentRoutePoint(const DronePose & self_pose)
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

DronePose RouteTask::getCurrentRoutePoint() const
{
    return route_list[route_index];
}

int RouteTask::getRouteListSize() const
{
    return (int)route_list.size();
}

DronePose RouteTask::runTask() {
    route_index = 0;
    while(route_index < route_list.size() - 1)
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

RouteTask::RouteTask(const int &task_id) : Task(task_id) {
    route_index = 0;
}


PointTask::PointTask(const int &task_id) : Task(task_id) {

}

DronePose PointTask::runTask() {
}
