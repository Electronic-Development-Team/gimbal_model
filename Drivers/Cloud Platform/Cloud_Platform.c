#include "Cloud_Platform.h"
#include "step_motor_port.h"
#include <math.h>

// 为不提供 M_PI 宏的编译器定义 PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 根据三维空间坐标控制云台指向
 *
 * @param x 目标点的x坐标
 * @param y 目标点的y坐标
 * @param z 目标点的z坐标
 */
void Cloud_Platform_Control(float x, float y, float z)
{
  // 1. 计算方位角 (Azimuth angle)
  // 向量在xoy平面的投影与x轴正向的夹角
  // 使用 atan2f(y, x) 计算出的弧度在(-PI, PI]范围, 转换为度是(-180, 180]
  // 当x>0时(题目限定的卦限), 角度范围是(-90, 90), 符合电机1的要求
  float azimuth_rad = atan2f(y, x);
  float azimuth_deg = azimuth_rad * 180.0f / M_PI;

  // 2. 计算天顶角 (Zenith angle)
  // 向量与z轴正向的夹角
  // 首先计算向量的模长
  float vector_length = sqrtf(x * x + y * y + z * z);
  float zenith_deg = 0.0f;

  // 避免当坐标为(0,0,0)时除以零
  if (vector_length > 0.0f)
  {
    // acosf 的结果范围是 [0, PI], 转换为度是 [0, 180], 符合电机2的要求
    float zenith_rad = acosf(z / vector_length);
    zenith_deg = zenith_rad * 180.0f / M_PI;
  }

  // 3. 调用底层电机控制接口
  Step_Motor_1_angle_Control(azimuth_deg);
  Step_Motor_2_angle_Control(zenith_deg);
}
