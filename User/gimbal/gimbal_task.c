#include "gimbal_task.h"
#include "step_motor_port.h"
#include <string.h>

// ������к�ָ��
static GimbalTask_t gimbal_task_queue[GIMBAL_TASK_QUEUE_SIZE];
static volatile uint8_t queue_head = 0;
static volatile uint8_t queue_tail = 0;

// ״̬��״̬
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
 * @brief ����������һ���µ���̨��������
 *
 * @param azimuth_deg ��λ��
 * @param zenith_deg �춥��
 */
void gimbal_task_add(float azimuth_deg, float zenith_deg)
{
  uint8_t next_head = (queue_head + 1) % GIMBAL_TASK_QUEUE_SIZE;
  if (next_head == queue_tail)
  {
    // ��������������ѡ�����򱨸����
    return;
  }
  gimbal_task_queue[queue_head].azimuth_deg = azimuth_deg;
  gimbal_task_queue[queue_head].zenith_deg = zenith_deg;
  queue_head = next_head;
}

/**
 * @brief ��̨����������Ӧ����ѭ����ʱ���е��ã�
 */
void gimbal_task_handler(void)
{
  // �������Ϊ����״̬�����У������¿���
  if (queue_head == queue_tail && gimbal_task_state == GIMBAL_TASK_IDLE)
  {
    return;
  }

  // ���״̬�����У���������������������������
  if (gimbal_task_state == GIMBAL_TASK_IDLE)
  {
    gimbal_task_state = GIMBAL_TASK_SEND_MOTOR1;
  }

  // ��ȡ��ǰ���񣬵�������
  GimbalTask_t *current_task = &gimbal_task_queue[queue_tail];

  // ״̬��ִ��
  if (gimbal_task_state == GIMBAL_TASK_SEND_MOTOR1)
  {
    if (Step_Motor_1_angle_Control(current_task->azimuth_deg) == HAL_OK)
    {
      // ���ͳɹ���״̬���ƽ����ȴ���ʱ
      gimbal_task_state = GIMBAL_TASK_WAIT_DELAY;
      delay_end_time = HAL_GetTick() + 1; // ����1ms��ʱ
    }
  }
  else if (gimbal_task_state == GIMBAL_TASK_WAIT_DELAY)
  {
    // �ȴ�1ms��ʱ����
    if (HAL_GetTick() >= delay_end_time)
    {
      gimbal_task_state = GIMBAL_TASK_SEND_MOTOR2;
    }
  }
  else if (gimbal_task_state == GIMBAL_TASK_SEND_MOTOR2)
  {
    if (Step_Motor_2_angle_Control(current_task->zenith_deg) == HAL_OK)
    {
      // ���2Ҳ���ͳɹ������������
      gimbal_task_state = GIMBAL_TASK_IDLE;
      // ��������
      queue_tail = (queue_tail + 1) % GIMBAL_TASK_QUEUE_SIZE;
    }
  }
}
