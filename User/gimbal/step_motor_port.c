#include "step_motor_port.h"
#include "main.h"
#include "Emm_V5.h"
#include <math.h>

#define PULSES_PER_REVOLUTION 3200
#define MOTOR_1_ADDR 0x01
#define MOTOR_2_ADDR 0x02
#define MOTOR_ACCEL 240

/**
 * @brief ���Ʒ�λ�ǵĵ��λ�ÿ��ƽӿ�
 * @param angle Ŀ��λ��(��λ:��)(ȡֵ-90~90��x��Ϊ�ο���xoyƽ������ʱ��Ϊ��)
 * @param speed_rpm ���ת�� (RPM)
 */
HAL_StatusTypeDef Step_Motor_1_angle_Control(float angle, uint16_t speed_rpm)
{
  long pulses = (long)roundf(angle / 360.0f * PULSES_PER_REVOLUTION);
  uint8_t dir = (pulses >= 0) ? 1 : 0;
  uint32_t pulses_abs = ABS(pulses);

  // Emm_V5_Pos_Control���һ������false��ʾ��ʹ��ͬ��ģʽ
  return Emm_V5_Pos_Control(MOTOR_1_ADDR, dir, speed_rpm, MOTOR_ACCEL, pulses_abs, true, false);
}

/**
 * @brief ���Ƹ����ǵĵ��λ�ÿ��ƽӿ�
 * @param angle Ŀ�긩����(��λ:��)(ȡֵ-90~90, xoyƽ��Ϊ0��, z������Ϊ+90��)
 * @param speed_rpm ���ת�� (RPM)
 */
HAL_StatusTypeDef Step_Motor_2_angle_Control(float angle, uint16_t speed_rpm)
{
  long pulses = (long)roundf(angle / 360.0f * PULSES_PER_REVOLUTION);
  uint8_t dir = (pulses >= 0) ? 1 : 0;
  uint32_t pulses_abs = ABS(pulses);

  // Emm_V5_Pos_Control���һ������false��ʾ��ʹ��ͬ��ģʽ
  return Emm_V5_Pos_Control(MOTOR_2_ADDR, dir, speed_rpm, MOTOR_ACCEL, pulses_abs, true, false);
}
