//
// Created by andykong on 2023/7/14.
//

#ifndef SRC_TASK_H
#define SRC_TASK_H
#include <vector>
#include <Eigen/Core>
#include <ros/ros.h>
#include <chrono>
#include "angles/angles.h"
#include "Drone.h"

class Task
{
public:
    explicit Task(const int & task_id);
    int getTaskId() const;
    virtual DronePose runTask() = 0;
protected:
    int task_id;
};

class RouteTask : public Task
{
public:
    explicit RouteTask(const int & task_id);
    void addToRouteList(const DronePose & route_point);
    DronePose nextRoutePoint();
    DronePose findCurrentRoutePoint(const DronePose & self_pose);
    int getCurrentRouteIndex() const;
    DronePose getCurrentRoutePoint() const;
    int getRouteListSize() const;
    bool isRouteFinished() const;
    DronePose runTask() override;

private:
    int route_index;
    std::vector<DronePose> route_list;

};

class PointTask : public Task
{
public:
    explicit PointTask(const int & task_id);
    cv::Point2f ImageTask(const imageTarget& img_target);
    DronePose runTask() override;
    bool isPointOver() const;
    cv::Point2f image_error;
    cv::Point2f error_fix;
private:
    Eigen::Vector3d accumulative_error;
    DronePose point;

};

class TakeOffTask : public Task
{
public:
    explicit TakeOffTask(const int & task_id);
    void setTakeOffPoint(const DronePose & take_off_point);
    void setTakeOffHeight(const double & height);
    bool isTakeOffFinished() const;
    double getTakeOffHeight() const;
    DronePose runTask() override;
private:
    DronePose take_off_point_on_land;
    double take_off_height;
    DronePose take_off_point_in_air;
};
#endif //SRC_TASK_H
