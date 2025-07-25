#include "step_motor_port.h"
#include "main.h"
#include "Emm_V5.h"
#include <math.h>

#define PULSES_PER_REVOLUTION 3200
#define MOTOR_1_ADDR 0x01
#define MOTOR_2_ADDR 0x02
#define MOTOR_ACCEL 240

/**
 * @brief 控制方位角的电机位置控制接口
 * @param angle 目标位置(单位:度)(取值-90~90以x轴为参考在xoy平面内逆时针为正)
 * @param speed_rpm 电机转速 (RPM)
 */
HAL_StatusTypeDef Step_Motor_1_angle_Control(float angle, uint16_t speed_rpm)
{
  long pulses = (long)roundf(angle / 360.0f * PULSES_PER_REVOLUTION);
  uint8_t dir = (pulses >= 0) ? 1 : 0;
  uint32_t pulses_abs = ABS(pulses);

  // Emm_V5_Pos_Control最后一个参数false表示不使用同步模式
  return Emm_V5_Pos_Control(MOTOR_1_ADDR, dir, speed_rpm, MOTOR_ACCEL, pulses_abs, true, false);
}

/**
 * @brief 控制俯仰角的电机位置控制接口
 * @param angle 目标俯仰角(单位:度)(取值-90~90, xoy平面为0度, z轴正向为+90度)
 * @param speed_rpm 电机转速 (RPM)
 */
HAL_StatusTypeDef Step_Motor_2_angle_Control(float angle, uint16_t speed_rpm)
{
  long pulses = (long)roundf(angle / 360.0f * PULSES_PER_REVOLUTION);
  uint8_t dir = (pulses >= 0) ? 1 : 0;
  uint32_t pulses_abs = ABS(pulses);

  // Emm_V5_Pos_Control最后一个参数false表示不使用同步模式
  return Emm_V5_Pos_Control(MOTOR_2_ADDR, dir, speed_rpm, MOTOR_ACCEL, pulses_abs, true, false);
}
