#include "main.h"
#include "control.h"
#include "task.h"
#include "test.h"
#include "gray_detection.h"
#include "tim.h"
#include "car_config.h"
/*
  �˴�˵���������򣺰˸���������ѭ��ֱ���䵽���Ӧ����ִ�������Լ�������㣺

  ��1���Ӿ�ֻ������ ��Ҫ��ѭ���ж�+д��
    ��Ҫ�����������Ӿ������������ߵ���Ӧ������ִ�У����ƣ������ԭ·����
    ԭ·�����߼����Խ���ջ�洢��ԭ����¼��ʱ·��Ȼ�󷵻ص�ջ������һ������
  ��2��ȫʹ���Ӿ���Ϊ�����߼���ѭ��
    ��Ҫ�������Ӿ�ʶ�����֣�Ȼ���ָ�������ʮ��·�ڸ�������ת��ת����ֱ��

  �Ҿ�����ѭ�����Ӿ���һ����Ӻ��ʣ��Ӿ������ж����֣�Ҳ�����жϵ�ǰ·���Ƿ�Ϊʮ��·��
  ����Ҳ��д���������жϵĻ���ʱ��д����Ŀ϶���ʮ��·���ж��߼���ѭ�����Ӿ�һ���ã��Ӿ����������ܷ�ɶ��
  ���ڽ���Ҳ���и��ˣ���д�㶫������������Щ������Լ�ѭ��Ҳ�������Ŀ����߼��������߼�Эͬ����Ҳ�鷳

  ���������˵�������ѭ��д�������Ч�ʱȽϸ��Ҿ��á�
 */

/**
 * @brief  ��ʱ����������ص�����
 * @param  htim: ��ʱ�����
 * @retval ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM12)
  {
// TODO: ��ʱ����������ص�����
#if TEST_IN_INTERRUPT == 1
    test_in_interrupt();
#endif
  }
}

/**
 * @brief ͨ��״̬��ʵ��С����ʻ��X��"����"(X=1,2,3,4,5,6,7,8)
 * @param �������(1,2,3,4,5,6,7,8)
 * @retval 1:��� 0:������
 */
uint8_t Car_To_Room(void)
{
  if (room_target >= 1 && room_target <= 2)
  {
    return Car_To_Room_1_2(room_target);
  }
  else if (room_target >= 3 && room_target <= 4)
  {
    return Car_To_Room_3_4(room_target);
  }
  else if (room_target >= 5 && room_target <= 8)
  {
    return Car_To_Room_5_8(room_target);
  }

  // ���������Ч�Ĳ�����ţ���ֹͣ
  set_motor_speed(0, 0, DEFAULT_ACCELERATION);
  return 0;
}

/**
 * @brief ͨ��״̬��ʵ��С����ʻ��1,2��"����"
 * @note:��ͼ���ܣ�1,2�Ų����ֱ��ڵ�һ��·�ڵ���������,С����Ҫ��ҩ������,��ѭ��ֱ�е���
 * ��һ��·��,Ȼ����ת����ת,����1,2�Ų���,Ȼ��ȴ�ȡҩ(���庯�������),ȡҩ��ɺ�,ͨ��
 * open_loop_steering_control��ת180��,Ȼ��ѭ��ֱ�е�·��,Ȼ����ת����ת,��ֱ�е���ҩ��,
 * �������
 * @param �������(1,2)
 * @retval 1:��� 0:������
 */
uint8_t Car_To_Room_1_2(uint8_t room_num)
{
  // ����״̬���ĸ���״̬
  typedef enum
  {
    CAR_STATE_INIT,               // ��ʼ״̬
    GOING_TO_CROSSING_1,          // ǰ����һ��·��
    TURNING_AT_CROSSING_1,        // �ڵ�һ��·��ת��
    GOING_TO_ROOM,                // ǰ������
    AT_ROOM,                      // ���ﲡ�����ȴ�ж��
    TURNING_180_AT_ROOM,          // �ڲ����ſ�180�ȵ�ͷ
    RETURNING_TO_CROSSING_1,      // ���ص�һ��·��
    TURNING_AT_CROSSING_1_RETURN, // ��·��ת��ҩ������
    RETURNING_TO_PHARMACY,        // ����ҩ��
    TASK_COMPLETE                 // �������
  } CarState_t;

  static CarState_t car_state = CAR_STATE_INIT;
  static uint8_t target_room_local = 0;

  // ���Ŀ�겡���ı䣬����1/2�ŷ���������״̬��
  if (room_num != target_room_local)
  {
    car_state = CAR_STATE_INIT;
    target_room_local = room_num;
    Car_To_Crossing(0); // ͨ������һ����ͬ��Ŀ��ֵ������Car_To_Crossing������״̬
  }

  // �������1�Ż�2�Ų�������ͣ��������
  if (room_num != 1 && room_num != 2)
  {
    set_motor_speed(0, 0, DEFAULT_ACCELERATION);
    return 0;
  }

  switch (car_state)
  {
  case CAR_STATE_INIT:
    car_state = GOING_TO_CROSSING_1;
    // ֱ�ӽ�����һ��״̬
  case GOING_TO_CROSSING_1:
    if (Car_To_Crossing(1)) // ��ʻ����һ��·��
    {
      car_state = TURNING_AT_CROSSING_1;
    }
    break;

  case TURNING_AT_CROSSING_1:
  {
    // 1�Ų�����ת��2�Ų�����ת��
    int16_t turn_duration_ms = (target_room_local == 1) ? TURN_90_DURATION_MS : -TURN_90_DURATION_MS;
    if (open_loop_steering_control(turn_duration_ms, TURN_90_SPEED, TURN_90_BASE_SPEED))
    {
      car_state = GOING_TO_ROOM; // ת����ɣ�������һ��״̬
    }
    break;
  }

  case GOING_TO_ROOM:
  {
    if (Car_To_Crossing(1)) // ��ʻ������
    {
      car_state = AT_ROOM;
    }
    break;
  }

  case AT_ROOM:
    // TODO: ������ɫָʾ��
    // TODO:�ȴ�ж��
    // TODO: Ϩ���ɫָʾ��
    car_state = TURNING_180_AT_ROOM;
    break;

  case TURNING_180_AT_ROOM:
    // 180�ȵ�ͷ
    if (open_loop_steering_control(TURN_180_DURATION_MS, TURN_180_SPEED, TURN_180_BASE_SPEED))
    {
      car_state = RETURNING_TO_CROSSING_1;
    }
    break;

  case RETURNING_TO_CROSSING_1:
    if (Car_To_Crossing(1)) // ������·��
    {
      car_state = TURNING_AT_CROSSING_1_RETURN;
    }
    break;

  case TURNING_AT_CROSSING_1_RETURN:
  {
    // ����ʱ����1�Ų���������Ҫ��ת����2�Ų���������Ҫ��ת
    int16_t turn_duration_ms = (target_room_local == 1) ? -TURN_90_DURATION_MS : TURN_90_DURATION_MS;
    if (open_loop_steering_control(turn_duration_ms, TURN_90_SPEED, TURN_90_BASE_SPEED))
    {
      car_state = RETURNING_TO_PHARMACY;
    }
    break;
  }

  case RETURNING_TO_PHARMACY:
    if (Car_To_Crossing(1)) // ����ҩ��
    {
      car_state = TASK_COMPLETE;
    }
    break;

  case TASK_COMPLETE:
    set_motor_speed(0, 0, DEFAULT_ACCELERATION); // ������ɣ�ͣ��
    // TODO: ������ɫָʾ��
    car_state = CAR_STATE_INIT; // ��λ״̬���Ա��´ε���
    target_room_local = 0;
    return 1; // �������
  }
  return 0;
}

/**
 * @brief ͨ��״̬��ʵ��С����ʻ��3,4��"����"
 * @note:��ͼ���ܣ�3,4�Ų����ֱ��ڵڶ���·�ڵ���������,С����Ҫ��ҩ������,��ѭ��ֱ�е���
 * �ڶ���·��,��ͨ����ȡroom_num���������ȡ���ֵ��Ų�(3,4�����ı�����Ҳ��ǹ̶�)
 * ���3,4�Ų����ڵڶ���·�ڵ����,��room_num��ֵΪ34,���3,4�Ų����ڵڶ���·�ڵ��Ҳ�,
 * ��room_num��ֵΪ43,���ݲ������Ȼ����ת����ת,����3,4�Ų���,Ȼ��ȴ�ȡҩ(��
 * �庯�������),ȡҩ��ɺ�,ͨ��open_loop_steering_control��ת180��,Ȼ��ѭ��ֱ�е�·��,
 * Ȼ����ת����ת,��ֱ��ͨ������·��,����ҩ��,�������
 * @param �������(3,4)
 * @retval 1:��� 0:������
 */
uint8_t Car_To_Room_3_4(uint8_t room_num)
{
  // ����״̬���ĸ���״̬
  typedef enum
  {
    CAR_STATE_INIT,               // ��ʼ״̬
    GOING_TO_CROSSING_2,          // ǰ���ڶ���·��
    TURNING_AT_CROSSING_2,        // �ڵڶ���·��ת��
    GOING_TO_ROOM,                // ǰ������
    AT_ROOM,                      // ���ﲡ�����ȴ�ж��
    TURNING_180_AT_ROOM,          // �ڲ����ſ�180�ȵ�ͷ
    RETURNING_TO_CROSSING_2,      // ���صڶ���·��
    TURNING_AT_CROSSING_2_RETURN, // ��·��ת��ҩ������
    RETURNING_TO_PHARMACY,        // ����ҩ��
    TASK_COMPLETE                 // �������
  } CarState_t;

  static CarState_t car_state = CAR_STATE_INIT;
  static uint8_t target_room_local = 0;
  static uint8_t room_layout_3_4 = 34; // ����Ĭ��3������4�����ҡ�34������3��4, 43������4��3

  // ���Ŀ�겡���ı䣬������״̬��
  if (room_num != target_room_local)
  {
    car_state = CAR_STATE_INIT;
    target_room_local = room_num;
    // ע�⣺3��4�ŷ����������ò�Ӧ���·�ڼ�������Ϊ��������������·�ڼ���
  }

  // �������3�Ż�4�Ų�������ͣ��������
  if (target_room_local != 3 && target_room_local != 4)
  {
    set_motor_speed(0, 0, DEFAULT_ACCELERATION);
    return 0;
  }

  switch (car_state)
  {
  case CAR_STATE_INIT:
    car_state = GOING_TO_CROSSING_2;
    // intentional fall-through

  case GOING_TO_CROSSING_2:
    if (Car_To_Crossing(2)) // ��ʻ���ڶ���·��
    {
      car_state = TURNING_AT_CROSSING_2;
    }
    break;

  case TURNING_AT_CROSSING_2:
  {
    // ����ȫ�ֱ��� room_num (���Ӿ�����)����������
    if (room_num == 34 || room_num == 43)
    {
      room_layout_3_4 = room_num;
    }
    // 3������(34)��Ŀ��3����ת�� 3������(43)��Ŀ��3����ת
    // 4������(34)��Ŀ��4����ת�� 4������(43)��Ŀ��4����ת
    int16_t turn_duration_ms = TURN_90_DURATION_MS; // Ĭ����ת
    uint8_t is_left_turn = (room_layout_3_4 == 34 && target_room_local == 3) || (room_layout_3_4 == 43 && target_room_local == 4);

    if (!is_left_turn)
    {
      turn_duration_ms = -TURN_90_DURATION_MS; // ��ת
    }

    if (open_loop_steering_control(turn_duration_ms, TURN_90_SPEED, TURN_90_BASE_SPEED))
    {
      car_state = GOING_TO_ROOM;
    }
    break;
  }

  case GOING_TO_ROOM:
    if (Car_To_Crossing(1)) // ��·����ʻ�������ſ�
    {
      car_state = AT_ROOM;
    }
    break;

  case AT_ROOM:
    // TODO: ������ɫָʾ��
    // TODO: �ȴ�ж��
    // TODO: Ϩ���ɫָʾ��
    car_state = TURNING_180_AT_ROOM;
    break;

  case TURNING_180_AT_ROOM:
    if (open_loop_steering_control(TURN_180_DURATION_MS, TURN_180_SPEED, TURN_180_BASE_SPEED))
    {
      car_state = RETURNING_TO_CROSSING_2;
    }
    break;

  case RETURNING_TO_CROSSING_2:
    if (Car_To_Crossing(1)) // �Ӳ������ص�·��
    {
      car_state = TURNING_AT_CROSSING_2_RETURN;
    }
    break;

  case TURNING_AT_CROSSING_2_RETURN:
  {
    // ����ʱת�����෴
    int16_t turn_duration_ms = -TURN_90_DURATION_MS; // Ĭ����ת
    uint8_t is_left_turn = (room_layout_3_4 == 34 && target_room_local == 3) || (room_layout_3_4 == 43 && target_room_local == 4);

    if (!is_left_turn)
    {
      turn_duration_ms = TURN_90_DURATION_MS; // ��ת
    }

    if (open_loop_steering_control(turn_duration_ms, TURN_90_SPEED, TURN_90_BASE_SPEED))
    {
      car_state = RETURNING_TO_PHARMACY;
    }
    break;
  }

  case RETURNING_TO_PHARMACY:
    if (Car_To_Crossing(2)) // �ӵڶ���·�ڷ���ҩ��
    {
      car_state = TASK_COMPLETE;
    }
    break;

  case TASK_COMPLETE:
    set_motor_speed(0, 0, DEFAULT_ACCELERATION);
    // TODO: ������ɫָʾ��
    target_room_local = 0; // ����Ŀ�꣬׼���´�����
    car_state = CAR_STATE_INIT;
    return 1; // �������
  }

  return 0; // ���������
}

/**
 * @brief ͨ��״̬��ʵ��С����ʻ��5,6,7,8��"����"
 * @note ��ͼ���ܣ�5,6,7,8�Ų���λ�ڵ�����ʮ��·�ڵ����ࡣ
 *       �ú��������Ӿ�ϵͳ�ڵ������·��ǰ����ȫ�ֱ��� `room_num` Ϊһ����λ����
 *       ���� `5678`����ʾ�����5��6�Ų������Ҳ���7��8�Ų�����
 *       ������������ͣ���ȴ��Ӿ�ʶ��
 * @param room_num Ŀ�겡�����(5,6,7,8)
 * @retval 1:������� 0:������
 */
uint8_t Car_To_Room_5_8(uint8_t room_num)
{
  // ����״̬���ĸ���״̬
  typedef enum
  {
    CAR_STATE_INIT,          // ��ʼ״̬
    GOING_TO_CROSSING_3,     // ǰ��������·��
    TURNING_AT_CROSSING_3,   // �ڵ�����·��ת��
    GOING_TO_T_JUNCTION,     // ǰ��T��·��
    MAKING_FINAL_TURN,       // ��T��·��������ת��
    GOING_TO_ROOM_FINAL,     // ǰ�����ղ���
    AT_ROOM,                 // ���ﲡ�����ȴ�ж��
    TURNING_180_AT_ROOM,     // �ڲ����ſ�180�ȵ�ͷ
    RETURNING_TO_T_JUNCTION, // ����T��·��
    MAKING_RETURN_TURN_1,    // ��T��·��ת�򣬷��ص���·��
    RETURNING_TO_CROSSING_3, // ���ص�����·��
    MAKING_RETURN_TURN_2,    // �ڵ���·��ת�򣬷���ҩ��
    RETURNING_TO_PHARMACY,   // ����ҩ��
    TASK_COMPLETE            // �������
  } CarState_t;

  static CarState_t car_state = CAR_STATE_INIT;
  static uint8_t target_room_local = 0;
  static uint16_t layout_at_crossing_3 = 0; // ���浽�����·��ʱ�Ĳ���
  static uint8_t is_initial_turn_left = 0;  // ��¼�ڵ���·�ڵ�ת����

  // ���Ŀ�겡���ı䣬������״̬��
  if (room_num != target_room_local)
  {
    car_state = CAR_STATE_INIT;
    target_room_local = room_num;
    Car_To_Crossing(0); // ����·�ڼ�����
  }

  // �������5,6,7,8�Ų�������ͣ��������
  if (target_room_local < 5 || target_room_local > 8)
  {
    set_motor_speed(0, 0, DEFAULT_ACCELERATION);
    return 0;
  }

  switch (car_state)
  {
  case CAR_STATE_INIT:
    car_state = GOING_TO_CROSSING_3;
    // intentional fall-through

  case GOING_TO_CROSSING_3:
    if (Car_To_Crossing(3))
    {
      // ����ʱ��ʹ���Ӿ�ϵͳ��;�и��µ�ȫ�ֲ��ֱ���
      layout_at_crossing_3 = room_num;
      car_state = TURNING_AT_CROSSING_3;
    }
    break;

  case TURNING_AT_CROSSING_3:
  {
    // �ж�Ŀ����������������Ҳ�����
    uint16_t left_rooms = layout_at_crossing_3 / 100;
    is_initial_turn_left = (target_room_local == (left_rooms / 10) || target_room_local == (left_rooms % 10));

    int16_t turn_duration_ms = is_initial_turn_left ? TURN_90_DURATION_MS : -TURN_90_DURATION_MS; // ��ת����ת
    if (open_loop_steering_control(turn_duration_ms, TURN_90_SPEED, TURN_90_BASE_SPEED))
    {
      car_state = GOING_TO_T_JUNCTION;
    }
    break;
  }

  case GOING_TO_T_JUNCTION:
    if (Car_To_Crossing(1)) // ����T��·��
    {
      car_state = MAKING_FINAL_TURN;
    }
    break;

  case MAKING_FINAL_TURN:
  {
    uint16_t area_layout = is_initial_turn_left ? (layout_at_crossing_3 / 100) : (layout_at_crossing_3 % 100);
    uint8_t is_final_turn_left = (target_room_local == (area_layout / 10));

    int16_t turn_duration_ms = is_final_turn_left ? TURN_90_DURATION_MS : -TURN_90_DURATION_MS; // ��ת����ת
    if (open_loop_steering_control(turn_duration_ms, TURN_90_SPEED, TURN_90_BASE_SPEED))
    {
      car_state = GOING_TO_ROOM_FINAL;
    }
    break;
  }

  case GOING_TO_ROOM_FINAL:
    if (Car_To_Crossing(1)) // ���ﲡ���ſ�
    {
      car_state = AT_ROOM;
    }
    break;

  case AT_ROOM:
    // TODO: ������ɫָʾ��, �ȴ�ж��
    car_state = TURNING_180_AT_ROOM;
    break;

  case TURNING_180_AT_ROOM:
    if (open_loop_steering_control(TURN_180_DURATION_MS, TURN_180_SPEED, TURN_180_BASE_SPEED)) // 180�ȵ�ͷ
    {
      car_state = RETURNING_TO_T_JUNCTION;
    }
    break;

  case RETURNING_TO_T_JUNCTION:
    if (Car_To_Crossing(1))
    {
      car_state = MAKING_RETURN_TURN_1;
    }
    break;

  case MAKING_RETURN_TURN_1:
  {
    uint16_t area_layout = is_initial_turn_left ? (layout_at_crossing_3 / 100) : (layout_at_crossing_3 % 100);
    uint8_t is_final_turn_left = (target_room_local == (area_layout / 10));
    int16_t turn_duration_ms = is_final_turn_left ? -TURN_90_DURATION_MS : TURN_90_DURATION_MS; // ����ʱת���෴
    if (open_loop_steering_control(turn_duration_ms, TURN_90_SPEED, TURN_90_BASE_SPEED))
    {
      car_state = RETURNING_TO_CROSSING_3;
    }
    break;
  }

  case RETURNING_TO_CROSSING_3:
    if (Car_To_Crossing(1))
    {
      car_state = MAKING_RETURN_TURN_2;
    }
    break;

  case MAKING_RETURN_TURN_2:
  {
    int16_t turn_duration_ms = is_initial_turn_left ? -TURN_90_DURATION_MS : TURN_90_DURATION_MS; // ����ʱת���෴
    if (open_loop_steering_control(turn_duration_ms, TURN_90_SPEED, TURN_90_BASE_SPEED))
    {
      car_state = RETURNING_TO_PHARMACY;
    }
    break;
  }

  case RETURNING_TO_PHARMACY:
    if (Car_To_Crossing(3)) // ����ҩ��
    {
      car_state = TASK_COMPLETE;
    }
    break;

  case TASK_COMPLETE:
    set_motor_speed(0, 0, DEFAULT_ACCELERATION);
    // TODO: ������ɫָʾ��
    target_room_local = 0; // ����Ŀ�꣬׼���´�����
    car_state = CAR_STATE_INIT;
    return 1; // �������
  }

  return 0; // ���������
}

/**
 * @brief ����С����ʻ����X��ʮ��·��(X=1,2,3)
 * @note �ú���Ϊ�����ú�������������ɲ�����1�����ڲ�״̬���Զ����ã�
 *       �´�ʹ����ͬ��������ʱ������ִ������
 * @param crossing_num Ŀ��ʮ��·�ڱ��(��1��ʼ����)������0��ʹ�������ֿ��в�ͣ����
 * @retval 1:����Ŀ��·�ڲ�ͣ�� 0:��������л��ڿ���״̬
 */
uint8_t Car_To_Crossing(uint8_t crossing_num)
{
  typedef enum
  {
    STATE_IDLE,   // ����״̬
    STATE_RUNNING // ����״̬
  } State_t;

  static State_t state = STATE_IDLE;
  static uint8_t crossings_found = 0;           // ���ҵ���·������
  static uint8_t is_on_crossing = 0;            // �Ƿ���·����
  static uint32_t last_crossing_enter_time = 0; // �ϴν���·�ڵ�ʱ��
  static uint8_t target_crossing_num = 0;       // Ŀ��·�ڱ��

  if (state == STATE_IDLE)
  {
    if (crossing_num > 0)
    {
      // �յ��µ�����ָ��
      state = STATE_RUNNING;
      target_crossing_num = crossing_num;
      crossings_found = 0;
      is_on_crossing = 0;
      last_crossing_enter_time = 0; // ����ʱ����������������һ��·��
    }
    else
    {
      // crossing_numΪ0����Ч�����ֿ���״̬��ȷ��С��ֹͣ
      set_motor_speed(0, 0, DEFAULT_ACCELERATION);
      return 0;
    }
  }

  // �����������״̬��
  uint32_t now = HAL_GetTick();

  // --- ·�ڼ���߼� ---
  if (Calculate_Turn_Value() == INT16_MAX)
  { // ��⵽·�ڴ������ź�
    if (!is_on_crossing)
    { // ��ʾ�ս���·������
      is_on_crossing = 1;
      // ������ֻ�е������ϴν���·����һ��ʱ����ʱ�ż���
      if (last_crossing_enter_time == 0 || now - last_crossing_enter_time > CROSSING_DEBOUNCE_MS)
      {
        crossings_found++;
        last_crossing_enter_time = now;
      }
    }
  }
  else
  {
    // ���ټ�⵽·����
    is_on_crossing = 0;
  }

  // --- ��������ж��߼� ---
  if (crossings_found >= target_crossing_num && is_on_crossing)
  {
    // ����Ŀ��λ�ã�ͣ��
    set_motor_speed(0, 0, DEFAULT_ACCELERATION);
    // Ϊ�´���������״̬
    state = STATE_IDLE;
    return 1; // ������ɣ�
  }

  // --- �˶������߼� ---
  line_following_task();
  return 0; // ���������
}
