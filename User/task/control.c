/**
 * @file control.c
 * @brief ��������
 * @note:����һ��Ӳ��ϵͳ,�ҵĳ���ʹ���������ŷ������Ϊ��������,����ĵ�ַ�ֱ�Ϊ0x01��0x02
 * @version 0.2
 * @date 2025-07-16
 * @copyright Copyright (c) 2025
 */
#include "control.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "Emm_V5.h"
#include "gray_detection.h"
#include "car_config.h"

volatile uint8_t usart2_rx_buffer = 9; // ���ջ�����
volatile uint8_t crossroads = 0;       // ��ʮ��·�ڵĴ���
volatile uint16_t room_num = 0;        // �������
volatile uint16_t room_target = 0;     // Ŀ�겡��

// --- UART RX Frame ---
#define RX_BUFFER_SIZE 4
static uint8_t rx_frame_buffer[RX_BUFFER_SIZE];
static uint8_t rx_frame_index = 0;
static volatile uint8_t new_frame_received = 0;

// --- ����ٶȿ���״̬�� ---
typedef enum
{
  MOTOR_TASK_IDLE,       // ����״̬��û������
  MOTOR_TASK_SEND_LEFT,  // �ȴ����������ٶ�ָ��
  MOTOR_TASK_WAIT_DELAY, // �����η���֮��ȴ�1ms
  MOTOR_TASK_SEND_RIGHT, // �ȴ����������ٶ�ָ��
} MotorTaskState_t;

typedef struct
{
  int16_t left_speed;
  int16_t right_speed;
  uint8_t acc;
} MotorSpeedTask_t;

#define MOTOR_TASK_QUEUE_SIZE 5
static MotorSpeedTask_t motor_task_queue[MOTOR_TASK_QUEUE_SIZE];
static volatile uint8_t queue_head = 0;
static volatile uint8_t queue_tail = 0;

static volatile MotorTaskState_t motor_task_state = MOTOR_TASK_IDLE;
static uint32_t delay_end_time = 0;

/**
 * @brief ���һ���µĵ���ٶȿ������񵽶���
 * @param left_speed �����ٶ�
 * @param right_speed �ҵ���ٶ�
 * @param acc ���ٶ�
 * @return 0: �ɹ�, -1: ��������
 */
int8_t set_motor_speed(int16_t left_speed, int16_t right_speed, uint8_t acc)
{
  uint8_t next_head = (queue_head + 1) % MOTOR_TASK_QUEUE_SIZE;
  if (next_head == queue_tail)
  {
    return -1; // ��������
  }
  motor_task_queue[queue_head].left_speed = left_speed;
  motor_task_queue[queue_head].right_speed = right_speed;
  motor_task_queue[queue_head].acc = acc;
  queue_head = next_head;
  return 0;
}

/**
 * @brief ����ٶ�����������Ӧ����ѭ����ʱ���е��ã�
 */
void motor_speed_task_handler(void)
{
  // �������Ϊ����״̬�����У������¿���
  if (queue_head == queue_tail && motor_task_state == MOTOR_TASK_IDLE)
  {
    return;
  }

  // ���״̬�����У���������������������������
  if (motor_task_state == MOTOR_TASK_IDLE)
  {
    motor_task_state = MOTOR_TASK_SEND_LEFT;
  }

  // ��ȡ��ǰ���񣬵�������
  MotorSpeedTask_t *current_task = &motor_task_queue[queue_tail];

  // ״̬��ִ��
  if (motor_task_state == MOTOR_TASK_SEND_LEFT)
  {
    int16_t speed = current_task->left_speed;
    uint8_t dir = (speed > 0);
    if (Emm_V5_Vel_Control(0x01, dir, abs(speed), current_task->acc, 0) == HAL_OK)
    {
      // ���ͳɹ���״̬���ƽ����ȴ���ʱ
      motor_task_state = MOTOR_TASK_WAIT_DELAY;
      delay_end_time = HAL_GetTick() + 1; // ����1ms��ʱ
    }
  }
  else if (motor_task_state == MOTOR_TASK_WAIT_DELAY)
  {
    // �ȴ�1ms��ʱ����
    if (HAL_GetTick() >= delay_end_time)
    {
      motor_task_state = MOTOR_TASK_SEND_RIGHT;
    }
  }
  else if (motor_task_state == MOTOR_TASK_SEND_RIGHT)
  {
    // ��������ܽ������״̬��˵����ʱ�ѵ�
    int16_t speed = current_task->right_speed;
    uint8_t dir = (speed > 0);
    if (Emm_V5_Vel_Control(0x02, dir, abs(speed), current_task->acc, 0) == HAL_OK)
    {
      // ����Ҳ���ͳɹ������������
      motor_task_state = MOTOR_TASK_IDLE;
      // ��������
      queue_tail = (queue_tail + 1) % MOTOR_TASK_QUEUE_SIZE;
    }
  }
}

/**
 * @brief UART������ɻص�����
 * @note ���������Ҫ�� stm32f4xx_it.c �� `HAL_UART_TxCpltCallback` �б�����
 */
void control_uart_tx_cplt_callback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == Emm_V5_HUART.Instance)
  {
    // Ŀǰ���ǵ�״̬���ƽ��߼���ͬ���ģ�
    // ����handler�з��ͳɹ��������ƽ�״̬��
    // DMA�ķ�����������ζ�����ǿ��������ﴦ������ӵ��첽�߼���
    // ��Ϊ�˼򻯣���ǰʵ��������handler����ѯ��
  }
}

/**
 * @brief ���������������С������ѭ��
 * @return ��������ѭ��,�ͷ���1,���򷵻�0
 */
uint8_t line_following_task(void)
{
  // 1. ���´���������
  grey_sensor_Read();

  // 2. ����ת��ֵ
  int turn_value = Calculate_Turn_Value();

  if (turn_value == INT16_MIN)
  {
    // û�м�⵽�ߣ������ѹ���
    // ���ԣ�ԭ��ͣ��
    set_motor_speed(0, 0, DEFAULT_ACCELERATION);
  }
  else if (turn_value == INT16_MAX)
  {
    set_motor_speed(CAR_BASE_SPEED, CAR_BASE_SPEED, DEFAULT_ACCELERATION);
  }
  else
  {
    // ����ת��ֵ�����������ٶ�
    // turn_value > 0 ��ʾ������ߣ���Ҫ��ת�������������ֿ�
    // turn_value < 0 ��ʾ�����ұߣ���Ҫ��ת�����ֿ죬������
    int16_t left_speed = CAR_BASE_SPEED - turn_value;
    int16_t right_speed = CAR_BASE_SPEED + turn_value;

    // ���ٶȽ����޷�
    if (left_speed > CAR_MAX_SPEED)
      left_speed = CAR_MAX_SPEED;
    if (left_speed < 0)
      left_speed = 0;
    if (right_speed > CAR_MAX_SPEED)
      right_speed = CAR_MAX_SPEED;
    if (right_speed < 0)
      right_speed = 0;

    set_motor_speed(left_speed, right_speed, DEFAULT_ACCELERATION); // ʹ��һ��Ĭ�ϼ��ٶ�
  }

  return 0; // ѭ��δ���
}

/**
 * @brief Open loop steering control function
 *
 * @param steerTime С����ת��ʱ��(ms),��ʱ����תΪ��,˳ʱ����תΪ��.
 * @param Rotate_Speed ת��ת��ֵ(��λ:ת/����)
 * @param Rotate_Speed_Base ����ת��ֵ(��λ:ת/����)
 * @return uint8_t 1:��� 0:С������ת����
 */
uint8_t open_loop_steering_control(int16_t steerTime, int16_t Rotate_Speed, int16_t Rotate_Speed_Base)
{
  typedef enum
  {
    STEER_IDLE,
    STEERING
  } SteeringState_t;
  static SteeringState_t steering_state = STEER_IDLE;
  static uint32_t turn_end_time = 0;

  if (steering_state == STEER_IDLE)
  {
    if (steerTime == 0)
    {
      return 1; // Not told to do anything, so we are "done"
    }

    // New command received while idle, start turning.
    int16_t left_speed, right_speed;

    if (steerTime > 0)
    { // Counter-clockwise turn (left)
      left_speed = Rotate_Speed_Base - Rotate_Speed;
      right_speed = Rotate_Speed_Base + Rotate_Speed;
    }
    else
    { // Clockwise turn (right)
      left_speed = Rotate_Speed_Base + Rotate_Speed;
      right_speed = Rotate_Speed_Base - Rotate_Speed;
    }

    set_motor_speed(left_speed, right_speed, DEFAULT_ACCELERATION);
    turn_end_time = HAL_GetTick() + abs(steerTime);
    steering_state = STEERING;
    return 0; // Turn initiated, not complete yet.
  }
  else // steering_state == STEERING
  {
    // A turn is in progress, check if it's time to stop.
    // Ignore new steerTime/speed values until the current turn is complete.
    if (HAL_GetTick() >= turn_end_time)
    {
      set_motor_speed(0, 0, DEFAULT_ACCELERATION); // Stop motors
      steering_state = STEER_IDLE;                 // Reset state for next command
      return 1;                                    // Turn completed
    }
    return 0; // Still turning
  }
}

/**
 * @brief ���ڽ����жϴ����������ڹ�������֡
 * @param rx_data �Ӵ��ڽ��յ��ĵ��ֽ�����
 */
void uart_rx_handler(uint8_t rx_data)
{
  if (rx_frame_index == 0 && rx_data == 0x55) // ֡ͷ
  {
    rx_frame_buffer[rx_frame_index++] = rx_data;
  }
  else if (rx_frame_index > 0 && rx_frame_index < RX_BUFFER_SIZE - 1)
  {
    rx_frame_buffer[rx_frame_index++] = rx_data;
  }
  else if (rx_frame_index == RX_BUFFER_SIZE - 1 && rx_data == 0x6B) // ֡β
  {
    rx_frame_buffer[rx_frame_index] = rx_data;
    new_frame_received = 1;
    rx_frame_index = 0;
  }
  else
  {
    rx_frame_index = 0; // ��Ч���ݣ�����
  }
}

/**
 * @brief �Ӿ����մ��ڳ�ʼ��
 * @param �ޣ����øģ�д����
 *
 */
void visual_reception_init()
{
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&usart2_rx_buffer, 1);
}

/**
 * @brief �����Ӿ�ģ��ͨ�����ڷ��͹�������������֡
 * @note  �ú���Ӧ����ѭ���б���ѯ���á�
 */
void visual_process_command(void)
{
  if (!new_frame_received)
  {
    return; // û�������ݣ�ֱ�ӷ���
  }

  // ��ʼ����������֡
  uint8_t data1 = rx_frame_buffer[1];
  uint8_t data2 = rx_frame_buffer[2];

  // �Ӹ�4λ�͵�4λ�н�������
  uint8_t num1 = (data1 >> 4) & 0x0F;
  uint8_t num2 = data1 & 0x0F;
  uint8_t num3 = (data2 >> 4) & 0x0F;
  uint8_t num4 = data2 & 0x0F;

  // ����ʶ�𵽵����ָ���������Ŀ��򲼾�
  // ���򣺵������ָ���Ŀ�꣬������ָ��²���
  if (num1 != 0 && num2 == 0 && num3 == 0 && num4 == 0) // ����1: ʶ��1������
  {
    room_target = num1;
  }
  else if (num1 != 0 && num2 != 0 && num3 == 0 && num4 == 0) // ����2: ʶ��2������
  {
    room_num = num1 * 10 + num2; // ���磺34
  }
  else if (num1 != 0 && num2 != 0 && num3 != 0 && num4 == 0) // ����3: ʶ��3������
  {
    room_num = num1 * 100 + num2 * 10 + num3; // ���磺567
  }
  else if (num1 != 0 && num2 != 0 && num3 != 0 && num4 != 0) // ����4: ʶ��4������
  {
    room_num = num1 * 1000 + num2 * 100 + num3 * 10 + num4; // ���磺5678
  }

  // ������ϣ������־λ��׼��������һ֡
  new_frame_received = 0;
}
