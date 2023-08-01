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

void Task::printLog() const {
    ROS_INFO("Task %d is running", task_id);
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

void RouteTask::printLog() const {
    ROS_INFO("RouteTask %d: , approaching to point NO_%d, x:%f, y:%f", task_id, route_index, route_list[route_index].position.x(), route_list[route_index].position.y());
}

DronePose RouteTask::runTask() {
    while(route_index < route_list.size())
    {
        if (pose_match(drone.getPose(), route_list[route_index]))
        {
            ROS_WARN("RouteTask %d: Arrived at route point %d", task_id, route_index);
            return nextRoutePoint();
        }
        else
        {
            return route_list[route_index];
        }
    }
    ROS_WARN("RouteTask %d: Route finished", task_id);
}

RouteTask::RouteTask(const int &task_id) : Task(task_id) {
    route_index = 0;
}

bool RouteTask::isTaskFinished() const {
    if (route_index == route_list.size())
    {
        return true;
    }
    else
    {
        return false;
    }
}

DronePose RouteTask::getStayPoint() {
    return route_list[route_list.size() - 1];
}


PointTask::PointTask(const int &task_id) : Task(task_id) {
    image_error.x = 0;
    image_error.y = 0;
}

cv::Point2f  PointTask::ImageTask(const imageTarget& img_target) {
    const int width = 640;
    const int height = 480;
    image_error.x = height/2 - img_target.img.y;
    image_error.y = width/2 - img_target.img.x;
    float s = sqrtf(image_error.x*image_error.x+image_error.y*image_error.y);
    image_error.x = image_error.x / s;
    image_error.y = image_error.y / s;
    return image_error;
}
bool PointTask::isPointOver(const imageTarget& img_target)  {
    PointTask::ImageTask(img_target);
    DronePose drone_target;
    drone_target.position.x()=drone_target.position.x()+image_error.x;
    drone_target.position.y()=drone_target.position.y()+image_error.y;
    if (position_match(drone.getPose(),drone_target))
    {
        return true;
    }
    else
    {
        return false;
    }
}
DronePose PointTask::runTask() {
}

TakeOffTask::TakeOffTask(const int &task_id) : Task(task_id) {
    take_off_point_on_land.position = Eigen::Vector3d::Zero();
    take_off_point_on_land.angular_orientation = Eigen::Vector3d::Zero();
    take_off_point_in_air.position = Eigen::Vector3d::Zero();
    take_off_point_in_air.angular_orientation = Eigen::Vector3d::Zero();
    take_off_height = 0;
}

void TakeOffTask::setTakeOffPoint(const DronePose &take_off_point) {
    take_off_point_on_land = take_off_point;
}

void TakeOffTask::setTakeOffHeight(const double &height) {
    take_off_height = height;
    take_off_point_in_air = take_off_point_on_land;
    take_off_point_in_air.position.z() += height;
}

void TakeOffTask::printLog() const {
    ROS_INFO("TakeOffTask %d: , take off height: %f", task_id, take_off_height);
}

inline bool height_match(const DronePose & a, const DronePose & b)
{
    return abs(a.position.z() - b.position.z()) < 0.02;
}

DronePose TakeOffTask::runTask() {
    if (height_match(drone.getPose(), take_off_point_in_air))
    {
        ROS_WARN("TakeOffTask %d: Arrived at take off point", task_id);
        return take_off_point_in_air;
    }
    else
    {
        return take_off_point_in_air;
    }
}

bool TakeOffTask::isTaskFinished() const {
    if (height_match(drone.getPose(), take_off_point_in_air))
    {
        return true;
    }
    else
    {
        return false;
    }
}


double TakeOffTask::getTakeOffHeight() const {
    return take_off_height;
}

DronePose TakeOffTask::getStayPoint() {
    return take_off_point_in_air;
}


LandTask::LandTask(const int &task_id) : Task(task_id) {

}

bool LandTask::isTaskFinished() const{
    if (drone.getPose().position.z() < -0.2)
    {
        return true;
    }
    else
    {
        return false;
    }
}

DronePose LandTask::runTask() {
    if (drone.getPose().position.z() < -0.2)
    {
        ROS_WARN("LandTask %d: Arrived at land point", task_id);
        return land_point;
    }
    else
    {
        return land_point;
    }
}

void LandTask::setLandPoint(const DronePose &_land_point) {
    land_point = _land_point;

}

void LandTask::printLog() const {
    ROS_INFO("LandTask %d: , land point: x:%f, y:%f, height:%f", task_id, land_point.position.x(), land_point.position.y(), land_point.position.z());
}

DronePose LandTask::getStayPoint() {
    return land_point;
}
