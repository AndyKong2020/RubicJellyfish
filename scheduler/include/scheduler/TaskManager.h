//
// Created by andykong on 23-7-11.
//
#ifndef SRC_TASKMANAGER_H
#define SRC_TASKMANAGER_H
#include <vector>
#include <Eigen/Core>
#include "ros/ros.h"
#include <scheduler/Task.h>
#include <scheduler/Drone.h>

class TaskManager
{
public:
    TaskManager();
    int nextTask();
    int getCurrentTaskId() const;
    void reset();
private:
    int task_index;
    std::vector<Task*> task_list;

};
#endif //SRC_TASKMANAGER_H
