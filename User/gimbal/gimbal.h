#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

/**
 * @brief 控制云台指向三维空间中的一个点
 *
 * 该函数接收一个三维坐标(x, y, z)，计算出对应的方位角和俯仰角,
 * 然后通过底层接口控制云台对准该点。
 *
 * @param x 目标点的x坐标
 * @param y 目标点的y坐标
 * @param z 目标点的z坐标
 */
void gimbal_Control(float x, float y, float z);
void test_gimbal(void);

#endif
