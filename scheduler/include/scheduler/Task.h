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
    virtual bool isTaskFinished() const = 0;
    virtual void printLog() const;
    virtual DronePose getStayPoint() = 0;
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
    void printLog() const override;
    bool isTaskFinished() const override;
    DronePose getStayPoint() override;
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
    bool isPointOver(const imageTarget& img_target) ;
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
    bool isTaskFinished() const override;
    double getTakeOffHeight() const;
    void printLog() const override;
    DronePose getStayPoint() override;
    DronePose runTask() override;
private:
    DronePose take_off_point_on_land;
    double take_off_height;
    DronePose take_off_point_in_air;

};

class LandTask : public Task
{
public:
    explicit LandTask(const int & task_id);
    bool isTaskFinished() const override;
    void setLandPoint(const DronePose & land_point);
    void printLog() const override;
    DronePose getStayPoint() override;
    DronePose runTask() override;
private:
    DronePose land_point;
};
#endif //SRC_TASK_H
