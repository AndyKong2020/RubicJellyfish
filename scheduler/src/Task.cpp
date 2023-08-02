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


PointTask::PointTask(const int & task_id) : Task(task_id) {
    tgt_error = cv::Point2d(0, 0);
    tgt_plane_distance = 0;
    setIntrinsicMatrix(385.5, 321.7, 385.5, 237.5);
}


//DronePose PointTask::runTask() {
//    tgt_plane_distance = sqrt(img_target.depth * img_target.depth - drone.getHeight() * drone.getHeight());
//    image_error.x = (double)frame_size.y/2 - (double)img_target.target_point.y;
//    image_error.y = (double)frame_size.x/2 - (double)img_target.target_point.x;
//    double proportion = tgt_plane_distance / sqrt(pow(image_error.x, 2) + pow(image_error.y, 2));
//    tgt_error.x = image_error.x * proportion;
//    tgt_error.y = image_error.y * proportion;
//    tgt_pose.position.x() = drone.getPose().position.x() + image_error.x;
//    tgt_pose.position.y() = drone.getPose().position.x() + image_error.y;
//    return tgt_pose;
//}

//DronePose PointTask::runTask() {
//    double theta_x = atan((img_target.target_point.x - cx)/fx);
//    double theta_y = atan((img_target.target_point.y - cy)/fy);
//    tgt_error.x = drone.getPosition().z() * tan(-theta_y - drone.getPose().angular_orientation.x());
//    tgt_error.y = drone.getPosition().z() * tan(theta_x + drone.getPose().angular_orientation.y());
//    tgt_pose.position.x() = drone.getPose().position.x() + tgt_error.x;
//    tgt_pose.position.y() = drone.getPose().position.x() + tgt_error.y;
//    return tgt_pose;
//}

DronePose PointTask::runTask() {
    Eigen::Vector3d pixel_tgt;
    pixel_tgt << img_target.target_point.x, img_target.target_point.y, 1;
    Eigen::Vector3d camera_tgt;
    camera_tgt << img_target.depth* (inverse_intrinsic_matrix * pixel_tgt);
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(drone.getPose().angular_orientation.z() - M_PI / 2, Eigen::Vector3d::UnitZ())
                      * Eigen::AngleAxisd(drone.getPose().angular_orientation.y(), Eigen::Vector3d::UnitY())
                      * Eigen::AngleAxisd(drone.getPose().angular_orientation.x() + M_2_PI, Eigen::Vector3d::UnitX());
    Eigen::Vector3d translation_vector;
    translation_vector << drone.getPose().position.x(), drone.getPose().position.y(), drone.getPose().position.z();
    Eigen::Vector3d world_tgt;
    world_tgt = rotation_matrix * camera_tgt + translation_vector;
    tgt_pose.position.x() = world_tgt.x();
    tgt_pose.position.y() = world_tgt.y();
    tgt_error.x = tgt_pose.position.x() - drone.getPose().position.x();
    tgt_error.y = tgt_pose.position.y() - drone.getPose().position.y();
    return tgt_pose;
}

void PointTask::printLog() const {
    ROS_INFO("approaching to point, x:%f, y:%f", tgt_pose.position.x(), tgt_pose.position.y());
}



bool PointTask::isTaskFinished() const {
    if (tgt_error.x == 0 && tgt_error.y == 0)
    {
        ROS_WARN("PointTask Not Init Yet");
        return false;
    }
    if (position_match(drone.getPose(),tgt_pose))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void PointTask::setFrameSize(const cv::Point2i &_frame_size) {
    frame_size = _frame_size;

}

void PointTask::getMessage(const ImageTarget &_img_target) {
    img_target = _img_target;
}

DronePose PointTask::getStayPoint() {
    setAccumulativeError();
    return tgt_pose;
}

void PointTask::setTrueValue(const Eigen::Vector3d &_true_value) {
    true_value = _true_value;
}

void PointTask::setAccumulativeError() {
    true_value[0] += tgt_error.x;
    true_value[1] += tgt_error.y;
    drone.setAccumulativeError(true_value);
    ROS_WARN("accumulative error x:%f, y:%f", drone.getAccumulativeError().x(), drone.getAccumulativeError().y());
}

void PointTask::setIntrinsicMatrix(const double &_fx, const double &_fy, const double &_cx, const double &_cy) {
    fx = _fx;
    fy = _fy;
    cx = _cx;
    cy = _cy;
    intrinsic_matrix << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    inverse_intrinsic_matrix << intrinsic_matrix.inverse();
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
