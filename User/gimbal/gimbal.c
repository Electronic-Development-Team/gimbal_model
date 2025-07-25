#include "gimbal.h"
#include "gimbal_task.h"
#include "main.h"
#include <math.h>

// 为不提供 M_PI 宏的编译器定义 PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 在等待时处理云台任务的延迟函数
 * @param ms 延迟的毫秒数
 */
static void gimbal_delay_ms(uint32_t ms)
{
  uint32_t start_tick = HAL_GetTick();
  while (HAL_GetTick() - start_tick < ms)
  {
    gimbal_task_handler();
  }
}

/**
 * @brief 根据三维空间坐标控制云台指向
 *
 * @param x 目标点的x坐标
 * @param y 目标点的y坐标
 * @param z 目标点的z坐标
 */
void gimbal_Control(float x, float y, float z)
{
  // 1. 计算方位角 (Azimuth angle)
  // 向量在xoy平面的投影与x轴正向的夹角
  // 使用 atan2f(y, x) 返回值的范围是(-PI, PI], 转换为角度(-180, 180]
  // 当x>0时(即目标在右半平面), 角度范围为(-90, 90), 符合电机1要求
  float azimuth_rad = atan2f(y, x);
  float azimuth_deg = azimuth_rad * 180.0f / M_PI;

  // 2. 计算俯仰角 (Elevation angle)
  // 向量与xoy平面的夹角
  float vector_length = sqrtf(x * x + y * y + z * z);
  float elevation_deg = 0.0f;

  // 避免当向量为(0,0,0)时除零
  if (vector_length > 0.0f)
  {
    // asinf 的结果范围是 [-PI/2, PI/2], 转换为角度 [-90, 90]
    // z轴正向为+90度, xoy平面为0度, z轴负向为-90度, 符合电机2要求
    float elevation_rad = asinf(z / vector_length);
    elevation_deg = elevation_rad * 180.0f / M_PI;
  }

  // 3. 将计算的角度添加到任务队列
  gimbal_task_add(azimuth_deg, elevation_deg);
}

void test_gimbal(void)
{
  while (1)
  {
    // 指向第1卦限 (+, +, +)
    gimbal_Control(10, 3.5, 50);
    gimbal_delay_ms(1000);

    // 指向第4卦限 (+, -, +)
    gimbal_Control(10, -4.5, 20);
    gimbal_delay_ms(1000);

    // 指向第5卦限 (+, +, -)
    gimbal_Control(10, 1.5, -5);
    gimbal_delay_ms(1000);

    // 指向第8卦限 (+, -, -)
    gimbal_Control(10, -7, -15);
    gimbal_delay_ms(1000);
  }
}
