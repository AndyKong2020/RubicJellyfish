//
// Created by andykong on 2023/7/14.
//
#include "scheduler/Task.h"

Task::Task(const int & task_id, const Eigen::Matrix<double, 3, 2> & pose_start, const Eigen::Matrix<double, 3, 2> & pose_end) {
    this->task_id = task_id;
    this->pose_start = pose_start;
    this->pose_end = pose_end;
}

int Task::getTaskId() const {
    return task_id;
}

