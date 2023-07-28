//
// Created by andykong on 2023/7/14.
//

#ifndef SRC_TASK_H
#define SRC_TASK_H
#include <vector>
#include <Eigen/Core>
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
    DronePose runTask() override;
private:
    Eigen::Vector3d accumulative_error;
    DronePose point;

};
#endif //SRC_TASK_H
