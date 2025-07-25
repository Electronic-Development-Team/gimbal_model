#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "main.h"
#include <stdbool.h>

/**
 * @brief 控制方位角的电机位置控制接口
 * @param angle 位置(单位:度)(取值-90~90以x轴为参考在xoy平面内逆时针为正)
 * @param speed_rpm 电机转速 (RPM)
 */
HAL_StatusTypeDef Step_Motor_1_angle_Control(float angle, uint16_t speed_rpm);

/**
 * @brief 控制俯仰角的电机位置控制接口
 * @param angle 俯仰角(单位:度)(取值-90~90, xoy平面为0度, z轴正向为+90度)
 * @param speed_rpm 电机转速 (RPM)
 */
HAL_StatusTypeDef Step_Motor_2_angle_Control(float angle, uint16_t speed_rpm);

#endif
