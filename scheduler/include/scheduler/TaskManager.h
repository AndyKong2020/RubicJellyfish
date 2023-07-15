//
// Created by andykong on 23-7-11.
//
#ifndef SRC_TASKMANAGER_H
#define SRC_TASKMANAGER_H
#include <vector>
#include <Eigen/Core>
#include "ros/ros.h"

class TaskManager
{
public:
    TaskManager();
    void nextTask();
    int getCurrentTask() const;
    void reset();
private:
    int task_index;

};
#endif //SRC_TASKMANAGER_H
