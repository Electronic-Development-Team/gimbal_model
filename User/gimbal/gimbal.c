#include "gimbal.h"
#include "gimbal_task.h"
#include "main.h"
#include <math.h>

// Ϊ���ṩ M_PI ��ı��������� PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief �ڵȴ�ʱ������̨������ӳٺ���
 * @param ms �ӳٵĺ�����
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
 * @brief ������ά�ռ����������ָ̨��
 *
 * @param x Ŀ����x����
 * @param y Ŀ����y����
 * @param z Ŀ����z����
 */
void gimbal_Control(float x, float y, float z)
{
  // 1. ���㷽λ�� (Azimuth angle)
  // ������xoyƽ���ͶӰ��x������ļн�
  // ʹ�� atan2f(y, x) ������Ļ�����(-PI, PI]��Χ, ת��Ϊ����(-180, 180]
  // ��x>0ʱ(��Ŀ�޶�������), �Ƕȷ�Χ��(-90, 90), ���ϵ��1��Ҫ��
  float azimuth_rad = atan2f(y, x);
  float azimuth_deg = azimuth_rad * 180.0f / M_PI;

  // 2. �����춥�� (Zenith angle)
  // ������z������ļн�
  // ���ȼ���������ģ��
  float vector_length = sqrtf(x * x + y * y + z * z);
  float zenith_deg = 0.0f;

  // ���⵱����Ϊ(0,0,0)ʱ������
  if (vector_length > 0.0f)
  {
    // acosf �Ľ����Χ�� [0, PI], ת��Ϊ���� [0, 180], ���ϵ��2��Ҫ��
    float zenith_rad = acosf(z / vector_length);
    zenith_deg = zenith_rad * 180.0f / M_PI;
  }

  // 3. ��������ĽǶ���ӵ��������
  gimbal_task_add(azimuth_deg, zenith_deg);
}

void test_gimbal(void)
{
  while (1)
  {
    // ָ���1���� (+, +, +)
    gimbal_Control(10, 10, 10);
    gimbal_delay_ms(3000);

    // ָ���4���� (+, -, +)
    gimbal_Control(10, -10, 10);
    gimbal_delay_ms(3000);

    // ָ���5���� (+, +, -)
    gimbal_Control(10, 10, -10);
    gimbal_delay_ms(3000);

    // ָ���8���� (+, -, -)
    gimbal_Control(10, -10, -10);
    gimbal_delay_ms(3000);
  }
}
