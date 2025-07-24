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
 * @brief 控制方位角的电机位置控制接口
 * @param angle 位置(单位:度)(取值-90~90以x轴为参考在xoy平面内逆时针为正)
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
 * @brief 控制级角(天顶角)的电机位置控制接口
 * @param angle 与z轴的夹角(单位:度)(取值0~180度)
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
