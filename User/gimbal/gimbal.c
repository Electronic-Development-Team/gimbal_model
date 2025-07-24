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

  // 3. 将计算出的角度添加到任务队列
  gimbal_task_add(azimuth_deg, zenith_deg);
}

void test_gimbal(void)
{
  while (1)
  {
    // 指向第1卦限 (+, +, +)
    gimbal_Control(10, 10, 10);
    gimbal_delay_ms(3000);

    // 指向第4卦限 (+, -, +)
    gimbal_Control(10, -10, 10);
    gimbal_delay_ms(3000);

    // 指向第5卦限 (+, +, -)
    gimbal_Control(10, 10, -10);
    gimbal_delay_ms(3000);

    // 指向第8卦限 (+, -, -)
    gimbal_Control(10, -10, -10);
    gimbal_delay_ms(3000);
  }
}
