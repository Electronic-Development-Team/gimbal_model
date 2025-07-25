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
 * @param sync 是否为同步模式的一部分
 */
HAL_StatusTypeDef Step_Motor_1_angle_Control(float angle, uint16_t speed_rpm, bool sync)
{
  long pulses = (long)roundf(angle / 360.0f * PULSES_PER_REVOLUTION);
  uint8_t dir = (pulses >= 0) ? 1 : 0;
  uint32_t pulses_abs = ABS(pulses);

  return Emm_V5_Pos_Control(MOTOR_1_ADDR, dir, speed_rpm, MOTOR_ACCEL, pulses_abs, true, sync);
}

/**
 * @brief 控制俯仰角的电机位置控制接口
 * @param angle 目标俯仰角(单位:度)(取值-90~90, xoy平面为0度, z轴正向为+90度)
 * @param speed_rpm 电机转速 (RPM)
 * @param sync 是否为同步模式的一部分
 */
HAL_StatusTypeDef Step_Motor_2_angle_Control(float angle, uint16_t speed_rpm, bool sync)
{
  long pulses = (long)roundf(angle / 360.0f * PULSES_PER_REVOLUTION);
  uint8_t dir = (pulses >= 0) ? 1 : 0;
  uint32_t pulses_abs = ABS(pulses);

  return Emm_V5_Pos_Control(MOTOR_2_ADDR, dir, speed_rpm, MOTOR_ACCEL, pulses_abs, true, sync);
}

/**
 * @brief 触发已设置为同步模式的电机同时运动
 */
void Step_Motor_Sync_Start(void)
{
  // 使用任一电机地址来触发所有已设置为同步模式的电机
  Emm_V5_Synchronous_motion(MOTOR_1_ADDR);
}
