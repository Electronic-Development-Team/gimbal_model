#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"

// 任务队列大小
#define GIMBAL_TASK_QUEUE_SIZE 10

// 电机最大和最小转速 (RPM)
#define MAX_MOTOR_SPEED_RPM 40.0f
#define MIN_MOTOR_SPEED_RPM 1.0f

// 任务结构体
typedef struct
{
  float azimuth_deg;   // 方位角
  float elevation_deg; // 俯仰角
} GimbalTask_t;

// 添加任务
void gimbal_task_add(float azimuth_deg, float elevation_deg);
void gimbal_task_handler(void);

#endif
