#include "gimbal_task.h"
#include "step_motor_port.h"
#include <string.h>
#include <math.h>

// 任务队列和指针
static GimbalTask_t gimbal_task_queue[GIMBAL_TASK_QUEUE_SIZE];
static volatile uint8_t queue_head = 0;
static volatile uint8_t queue_tail = 0;

// 云台当前角度状态
static float current_azimuth_deg = 0.0f;
static float current_elevation_deg = 0.0f;

// 状态机状态
typedef enum
{
  GIMBAL_TASK_IDLE,
  GIMBAL_TASK_SETUP_MOTOR1,
  GIMBAL_TASK_SETUP_MOTOR2,
  GIMBAL_TASK_TRIGGER_SYNC,
  GIMBAL_TASK_WAITING
} GimbalTaskState_t;

static GimbalTaskState_t gimbal_task_state = GIMBAL_TASK_IDLE;
static uint32_t delay_end_time = 0;

// 用于在状态间传递计算出的速度和延时
static uint16_t s_azimuth_rpm, s_elevation_rpm;
static uint32_t s_completion_ms;

/**
 * @brief 向任务队列中添加一个新的云台指向任务
 *
 * @param azimuth_deg 方位角
 * @param elevation_deg 俯仰角
 */
void gimbal_task_add(float azimuth_deg, float elevation_deg)
{
  uint8_t next_head = (queue_head + 1) % GIMBAL_TASK_QUEUE_SIZE;
  if (next_head == queue_tail)
  {
    // 队列已满，可以选择覆盖或者报错
    return;
  }
  gimbal_task_queue[queue_head].azimuth_deg = azimuth_deg;
  gimbal_task_queue[queue_head].elevation_deg = elevation_deg;
  queue_head = next_head;
}

/**
 * @brief 云台任务处理器，应在主循环中定时调用
 */
void gimbal_task_handler(void)
{
  // 获取当前任务，但不移出队列
  GimbalTask_t *current_task = &gimbal_task_queue[queue_tail];

  switch (gimbal_task_state)
  {
  case GIMBAL_TASK_IDLE:
    // 如果队列中有任务，则开始新任务
    if (queue_head != queue_tail)
    {
      // 1. 计算角度差和最大差值
      float delta_azimuth = current_task->azimuth_deg - current_azimuth_deg;
      float delta_elevation = current_task->elevation_deg - current_elevation_deg;
      float max_delta = fmaxf(fabsf(delta_azimuth), fabsf(delta_elevation));

      if (max_delta < 0.01f)
      {
        // 几乎没有移动，直接完成任务
        queue_tail = (queue_tail + 1) % GIMBAL_TASK_QUEUE_SIZE;
        break;
      }

      // 2. 计算运动时间和各电机速度
      float duration_s = max_delta / (MAX_MOTOR_SPEED_RPM * 360.0f / 60.0f);
      s_completion_ms = (uint32_t)(duration_s * 1000.0f);

      s_azimuth_rpm = (uint16_t)roundf(MAX_MOTOR_SPEED_RPM * fabsf(delta_azimuth) / max_delta);
      s_elevation_rpm = (uint16_t)roundf(MAX_MOTOR_SPEED_RPM * fabsf(delta_elevation) / max_delta);

      // 保证有移动时速度不为0
      if (s_azimuth_rpm < MIN_MOTOR_SPEED_RPM && fabsf(delta_azimuth) > 0.01f)
      {
        s_azimuth_rpm = MIN_MOTOR_SPEED_RPM;
      }
      if (s_elevation_rpm < MIN_MOTOR_SPEED_RPM && fabsf(delta_elevation) > 0.01f)
      {
        s_elevation_rpm = MIN_MOTOR_SPEED_RPM;
      }

      gimbal_task_state = GIMBAL_TASK_SETUP_MOTOR1;
    }
    break;

  case GIMBAL_TASK_SETUP_MOTOR1:
    Step_Motor_1_angle_Control(current_task->azimuth_deg, s_azimuth_rpm, true);
    delay_end_time = HAL_GetTick() + SEND_DELAY_TIME; // SEND_DELAY_TIMEms延时确保指令发送
    gimbal_task_state = GIMBAL_TASK_SETUP_MOTOR2;
    break;

  case GIMBAL_TASK_SETUP_MOTOR2:
    if (HAL_GetTick() >= delay_end_time)
    {
      Step_Motor_2_angle_Control(current_task->elevation_deg, s_elevation_rpm, true);
      delay_end_time = HAL_GetTick() + SEND_DELAY_TIME; // SEND_DELAY_TIMEms延时
      gimbal_task_state = GIMBAL_TASK_TRIGGER_SYNC;
    }
    break;

  case GIMBAL_TASK_TRIGGER_SYNC:
    if (HAL_GetTick() >= delay_end_time)
    {
      Step_Motor_Sync_Start();
      // 更新当前位置为目标位置
      current_azimuth_deg = current_task->azimuth_deg;
      current_elevation_deg = current_task->elevation_deg;
      // 设置任务完成的等待时间
      delay_end_time = HAL_GetTick() + s_completion_ms;
      gimbal_task_state = GIMBAL_TASK_WAITING;
    }
    break;

  case GIMBAL_TASK_WAITING:
    // 等待电机运动结束
    if (HAL_GetTick() >= delay_end_time)
    {
      // 任务完成，移出队列
      queue_tail = (queue_tail + 1) % GIMBAL_TASK_QUEUE_SIZE;
      gimbal_task_state = GIMBAL_TASK_IDLE;
    }
    break;
  }
}
