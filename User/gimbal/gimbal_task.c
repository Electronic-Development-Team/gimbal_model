#include "gimbal_task.h"
#include "step_motor_port.h"
#include <string.h>

// 任务队列和指针
static GimbalTask_t gimbal_task_queue[GIMBAL_TASK_QUEUE_SIZE];
static volatile uint8_t queue_head = 0;
static volatile uint8_t queue_tail = 0;

// 状态机状态
typedef enum
{
  GIMBAL_TASK_IDLE,
  GIMBAL_TASK_SEND_MOTOR1,
  GIMBAL_TASK_WAIT_DELAY,
  GIMBAL_TASK_SEND_MOTOR2
} GimbalTaskState_t;

static GimbalTaskState_t gimbal_task_state = GIMBAL_TASK_IDLE;
static uint32_t delay_end_time = 0;

/**
 * @brief 向队列中添加一个新的云台控制任务
 *
 * @param azimuth_deg 方位角
 * @param zenith_deg 天顶角
 */
void gimbal_task_add(float azimuth_deg, float zenith_deg)
{
  uint8_t next_head = (queue_head + 1) % GIMBAL_TASK_QUEUE_SIZE;
  if (next_head == queue_tail)
  {
    // 队列已满，可以选择丢弃或报告错误
    return;
  }
  gimbal_task_queue[queue_head].azimuth_deg = azimuth_deg;
  gimbal_task_queue[queue_head].zenith_deg = zenith_deg;
  queue_head = next_head;
}

/**
 * @brief 云台任务处理器（应在主循环或定时器中调用）
 */
void gimbal_task_handler(void)
{
  // 如果队列为空且状态机空闲，则无事可做
  if (queue_head == queue_tail && gimbal_task_state == GIMBAL_TASK_IDLE)
  {
    return;
  }

  // 如果状态机空闲，但队列中有任务，则启动新任务
  if (gimbal_task_state == GIMBAL_TASK_IDLE)
  {
    gimbal_task_state = GIMBAL_TASK_SEND_MOTOR1;
  }

  // 获取当前任务，但不弹出
  GimbalTask_t *current_task = &gimbal_task_queue[queue_tail];

  // 状态机执行
  if (gimbal_task_state == GIMBAL_TASK_SEND_MOTOR1)
  {
    if (Step_Motor_1_angle_Control(current_task->azimuth_deg) == HAL_OK)
    {
      // 发送成功，状态机推进到等待延时
      gimbal_task_state = GIMBAL_TASK_WAIT_DELAY;
      delay_end_time = HAL_GetTick() + 1; // 设置1ms延时
    }
  }
  else if (gimbal_task_state == GIMBAL_TASK_WAIT_DELAY)
  {
    // 等待1ms延时结束
    if (HAL_GetTick() >= delay_end_time)
    {
      gimbal_task_state = GIMBAL_TASK_SEND_MOTOR2;
    }
  }
  else if (gimbal_task_state == GIMBAL_TASK_SEND_MOTOR2)
  {
    if (Step_Motor_2_angle_Control(current_task->zenith_deg) == HAL_OK)
    {
      // 电机2也发送成功，此任务完成
      gimbal_task_state = GIMBAL_TASK_IDLE;
      // 弹出队列
      queue_tail = (queue_tail + 1) % GIMBAL_TASK_QUEUE_SIZE;
    }
  }
}
