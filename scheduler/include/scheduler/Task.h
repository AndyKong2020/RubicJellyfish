//
// Created by andykong on 2023/7/14.
//

#ifndef SRC_TASK_H
#define SRC_TASK_H
#include <vector>
#include <Eigen/Core>

class Task
{
public:
    Task(const int & task_id, const Eigen::Matrix<double, 3, 2> & pose_start, const Eigen::Matrix<double, 3, 2> & pose_end);
    int getTaskId() const;
    virtual void runTask() = 0;
private:
    int task_id;
    Eigen::Matrix<double, 3, 2> pose_start, pose_end;
};

class RouteTask : public Task
{
public:
    void addToRouteList(const Eigen::Matrix<double, 3, 2> & route_point);
    int getRouteIndex() const;
    Eigen::Matrix<double, 3, 2> nextRoutePoint();
    Eigen::Matrix<double, 3, 2> setCurrentRoutePoint(const Eigen::Matrix<double, 3, 2> & self_pose);
    int getCurrentRouteIndex();

private:
    int route_index;
    std::vector<Eigen::Matrix<double, 3, 2>> route_list;

};
#endif //SRC_TASK_H
