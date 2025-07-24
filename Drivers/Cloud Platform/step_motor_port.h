#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

/**
 * @brief 控制方位角的电机位置控制接口
 * @param angle 位置(单位:度)(取值-90~90以x轴为参考在xoy平面内逆时针为正)
 *
 */
void Step_Motor_1_angle_Control(float angle);

/**
 * @brief 控制级角(天顶角)的电机位置控制接口
 * @param angle 与z轴的夹角(单位:度)(取值0~180度)
 *
 */
void Step_Motor_2_angle_Control(float angle);

#endif
