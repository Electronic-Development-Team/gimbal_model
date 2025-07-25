#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"

// ������д�С
#define GIMBAL_TASK_QUEUE_SIZE 10

// ���������Сת�� (RPM)
#define MAX_MOTOR_SPEED_RPM 40.0f
#define MIN_MOTOR_SPEED_RPM 1.0f

// ����ṹ��
typedef struct
{
  float azimuth_deg;   // ��λ��
  float elevation_deg; // ������
} GimbalTask_t;

// �������
void gimbal_task_add(float azimuth_deg, float elevation_deg);
void gimbal_task_handler(void);

#endif
