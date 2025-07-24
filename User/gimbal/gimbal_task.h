#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"

// 任务队列大小
#define GIMBAL_TASK_QUEUE_SIZE 10

// 任务结构体
typedef struct
{
  float azimuth_deg;
  float zenith_deg;
} GimbalTask_t;

// 函数声明
void gimbal_task_add(float azimuth_deg, float zenith_deg);
void gimbal_task_handler(void);

#endif
