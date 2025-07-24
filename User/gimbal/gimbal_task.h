#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"

// ������д�С
#define GIMBAL_TASK_QUEUE_SIZE 10

// ����ṹ��
typedef struct
{
  float azimuth_deg;
  float zenith_deg;
} GimbalTask_t;

// ��������
void gimbal_task_add(float azimuth_deg, float zenith_deg);
void gimbal_task_handler(void);

#endif
