#include "gimbal_task.h"
#include "step_motor_port.h"
#include <string.h>
#include <math.h>

// ������к�ָ��
static GimbalTask_t gimbal_task_queue[GIMBAL_TASK_QUEUE_SIZE];
static volatile uint8_t queue_head = 0;
static volatile uint8_t queue_tail = 0;

// ��̨��ǰ�Ƕ�״̬
static float current_azimuth_deg = 0.0f;
static float current_elevation_deg = 0.0f;

// ״̬��״̬
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

// ������״̬�䴫�ݼ�������ٶȺ���ʱ
static uint16_t s_azimuth_rpm, s_elevation_rpm;
static uint32_t s_completion_ms;

/**
 * @brief ��������������һ���µ���ָ̨������
 *
 * @param azimuth_deg ��λ��
 * @param elevation_deg ������
 */
void gimbal_task_add(float azimuth_deg, float elevation_deg)
{
  uint8_t next_head = (queue_head + 1) % GIMBAL_TASK_QUEUE_SIZE;
  if (next_head == queue_tail)
  {
    // ��������������ѡ�񸲸ǻ��߱���
    return;
  }
  gimbal_task_queue[queue_head].azimuth_deg = azimuth_deg;
  gimbal_task_queue[queue_head].elevation_deg = elevation_deg;
  queue_head = next_head;
}

/**
 * @brief ��̨����������Ӧ����ѭ���ж�ʱ����
 */
void gimbal_task_handler(void)
{
  // ��ȡ��ǰ���񣬵����Ƴ�����
  GimbalTask_t *current_task = &gimbal_task_queue[queue_tail];

  switch (gimbal_task_state)
  {
  case GIMBAL_TASK_IDLE:
    // �����������������ʼ������
    if (queue_head != queue_tail)
    {
      // 1. ����ǶȲ������ֵ
      float delta_azimuth = current_task->azimuth_deg - current_azimuth_deg;
      float delta_elevation = current_task->elevation_deg - current_elevation_deg;
      float max_delta = fmaxf(fabsf(delta_azimuth), fabsf(delta_elevation));

      if (max_delta < 0.01f)
      {
        // ����û���ƶ���ֱ���������
        queue_tail = (queue_tail + 1) % GIMBAL_TASK_QUEUE_SIZE;
        break;
      }

      // 2. �����˶�ʱ��͸�����ٶ�
      float duration_s = max_delta / (MAX_MOTOR_SPEED_RPM * 360.0f / 60.0f);
      s_completion_ms = (uint32_t)(duration_s * 1000.0f);

      s_azimuth_rpm = (uint16_t)roundf(MAX_MOTOR_SPEED_RPM * fabsf(delta_azimuth) / max_delta);
      s_elevation_rpm = (uint16_t)roundf(MAX_MOTOR_SPEED_RPM * fabsf(delta_elevation) / max_delta);

      // ��֤���ƶ�ʱ�ٶȲ�Ϊ0
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
    delay_end_time = HAL_GetTick() + SEND_DELAY_TIME; // SEND_DELAY_TIMEms��ʱȷ��ָ���
    gimbal_task_state = GIMBAL_TASK_SETUP_MOTOR2;
    break;

  case GIMBAL_TASK_SETUP_MOTOR2:
    if (HAL_GetTick() >= delay_end_time)
    {
      Step_Motor_2_angle_Control(current_task->elevation_deg, s_elevation_rpm, true);
      delay_end_time = HAL_GetTick() + SEND_DELAY_TIME; // SEND_DELAY_TIMEms��ʱ
      gimbal_task_state = GIMBAL_TASK_TRIGGER_SYNC;
    }
    break;

  case GIMBAL_TASK_TRIGGER_SYNC:
    if (HAL_GetTick() >= delay_end_time)
    {
      Step_Motor_Sync_Start();
      // ���µ�ǰλ��ΪĿ��λ��
      current_azimuth_deg = current_task->azimuth_deg;
      current_elevation_deg = current_task->elevation_deg;
      // ����������ɵĵȴ�ʱ��
      delay_end_time = HAL_GetTick() + s_completion_ms;
      gimbal_task_state = GIMBAL_TASK_WAITING;
    }
    break;

  case GIMBAL_TASK_WAITING:
    // �ȴ�����˶�����
    if (HAL_GetTick() >= delay_end_time)
    {
      // ������ɣ��Ƴ�����
      queue_tail = (queue_tail + 1) % GIMBAL_TASK_QUEUE_SIZE;
      gimbal_task_state = GIMBAL_TASK_IDLE;
    }
    break;
  }
}
