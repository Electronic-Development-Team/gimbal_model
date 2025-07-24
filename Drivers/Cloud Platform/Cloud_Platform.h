#ifndef CLOUD_PLATFORM_H
#define CLOUD_PLATFORM_H

/**
 * @brief 根据三维空间坐标控制云台指向
 *
 * 该函数接收一个点的(x, y, z)坐标, 计算出对应的方位角和天顶角,
 * 并调用底层电机接口来驱动云台对准该点。
 *
 * @param x 目标点的x坐标
 * @param y 目标点的y坐标
 * @param z 目标点的z坐标
 */
void Cloud_Platform_Control(float x, float y, float z);

#endif
