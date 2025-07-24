#include "step_motor_port.h"
#include "main.h"
#include "Emm_V5.h"
#include <math.h>

#define PULSES_PER_REVOLUTION 3200
#define MOTOR_1_ADDR 0x01
#define MOTOR_2_ADDR 0x02
#define MOTOR_SPEED_RPM 40
#define MOTOR_ACCEL 240



/**
 * @brief ���Ʒ�λ�ǵĵ��λ�ÿ��ƽӿ�
 * @param angle λ��(��λ:��)(ȡֵ-90~90��x��Ϊ�ο���xoyƽ������ʱ��Ϊ��)
 *
 */
HAL_StatusTypeDef Step_Motor_1_angle_Control(float angle)
{
  float delta_angle = angle;

  if (fabsf(delta_angle) < 0.01f)
  {
    return HAL_OK;
  }

  long pulses = (long)roundf(delta_angle / 360.0f * PULSES_PER_REVOLUTION);
  uint8_t dir = (pulses >= 0) ? 1 : 0;
  uint32_t pulses_abs = ABS(pulses);

  return Emm_V5_Pos_Control(MOTOR_1_ADDR, dir, MOTOR_SPEED_RPM, MOTOR_ACCEL, pulses_abs, true, false);
}

/**
 * @brief ���Ƽ���(�춥��)�ĵ��λ�ÿ��ƽӿ�
 * @param angle ��z��ļн�(��λ:��)(ȡֵ0~180��)
 *
 */
HAL_StatusTypeDef Step_Motor_2_angle_Control(float angle)
{
  float motor_angle = angle - 90.0f;
  float delta_angle = motor_angle;

  if (fabsf(delta_angle) < 0.01f)
  {
    return HAL_OK;
  }

  long pulses = (long)roundf(delta_angle / 360.0f * PULSES_PER_REVOLUTION);
  uint8_t dir = (pulses >= 0) ? 1 : 0;
  uint32_t pulses_abs = ABS(pulses);

  return Emm_V5_Pos_Control(MOTOR_2_ADDR, dir, MOTOR_SPEED_RPM, MOTOR_ACCEL, pulses_abs, true, false);
}
