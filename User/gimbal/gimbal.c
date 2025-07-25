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
  // ʹ�� atan2f(y, x) ����ֵ�ķ�Χ��(-PI, PI], ת��Ϊ�Ƕ�(-180, 180]
  // ��x>0ʱ(��Ŀ�����Ұ�ƽ��), �Ƕȷ�ΧΪ(-90, 90), ���ϵ��1Ҫ��
  float azimuth_rad = atan2f(y, x);
  float azimuth_deg = azimuth_rad * 180.0f / M_PI;

  // 2. ���㸩���� (Elevation angle)
  // ������xoyƽ��ļн�
  float vector_length = sqrtf(x * x + y * y + z * z);
  float elevation_deg = 0.0f;

  // ���⵱����Ϊ(0,0,0)ʱ����
  if (vector_length > 0.0f)
  {
    // asinf �Ľ����Χ�� [-PI/2, PI/2], ת��Ϊ�Ƕ� [-90, 90]
    // z������Ϊ+90��, xoyƽ��Ϊ0��, z�Ḻ��Ϊ-90��, ���ϵ��2Ҫ��
    float elevation_rad = asinf(z / vector_length);
    elevation_deg = elevation_rad * 180.0f / M_PI;
  }

  // 3. ������ĽǶ���ӵ��������
  gimbal_task_add(azimuth_deg, elevation_deg);
}

void test_gimbal(void)
{
  while (1)
  {
    // ָ���1���� (+, +, +)
    gimbal_Control(10, 3.5, 50);
    gimbal_delay_ms(1000);

    // ָ���4���� (+, -, +)
    gimbal_Control(10, -4.5, 20);
    gimbal_delay_ms(1000);

    // ָ���5���� (+, +, -)
    gimbal_Control(10, 1.5, -5);
    gimbal_delay_ms(1000);

    // ָ���8���� (+, -, -)
    gimbal_Control(10, -7, -15);
    gimbal_delay_ms(1000);
  }
}
