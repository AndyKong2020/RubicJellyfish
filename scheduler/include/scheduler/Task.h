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
    Task(const int & task_id, const Eigen::Matrix<double, 3, 2> & pose_start, const Eigen::Matrix<double, 3, 2> & pose_end);
    int getTaskId() const;
    virtual Eigen::Matrix<double, 3, 2> runTask() = 0;
private:
    int task_id;
    Eigen::Matrix<double, 3, 2> pose_start, pose_end;
};

class RouteTask : public Task
{
public:
    void addToRouteList(const Eigen::Matrix<double, 3, 2> & route_point);
    Eigen::Matrix<double, 3, 2> nextRoutePoint();
    Eigen::Matrix<double, 3, 2> findCurrentRoutePoint(const Eigen::Matrix<double, 3, 2> & self_pose);
    int getCurrentRouteIndex() const;
    Eigen::Matrix<double, 3, 2> getCurrentRoutePoint() const;
    int getRouteListSize() const;
    Eigen::Matrix<double, 3, 2> runTask() override;

private:
    int route_index;
    std::vector<Eigen::Matrix<double, 3, 2>> route_list;

};
#endif //SRC_TASK_H
