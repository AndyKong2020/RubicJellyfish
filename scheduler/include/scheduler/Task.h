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
private:
    int task_id;
    Eigen::Matrix<double, 3, 2> pose_start, pose_end;
};
#endif //SRC_TASK_H
