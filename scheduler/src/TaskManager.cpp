//
// Created by andykong on 23-7-11.
//
#include <scheduler/TaskManager.h>


TaskManager::TaskManager()
{
    task_index = 0;
}

void TaskManager::nextTask()
{
    task_index++;
}

int TaskManager::getCurrentTask() const
{
    return task_index;
}

void TaskManager::reset()
{
    task_index = 0;
}