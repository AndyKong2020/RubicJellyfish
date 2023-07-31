//
// Created by andykong on 23-7-11.
//
#include <scheduler/TaskManager.h>


TaskManager::TaskManager()
{
    task_index = 0;
}

int TaskManager::nextTask()
{
    task_index++;
    return task_index;
}

int TaskManager::getCurrentTaskId() const
{
    return task_index;
}

void TaskManager::reset()
{
    task_index = 0;
}