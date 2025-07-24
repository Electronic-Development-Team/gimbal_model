/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include "my_menu.h"
#include "Emm_V5.h"
#include "test.h"
#include "gray_detection.h"
#include "control.h"
#include "task.h"
#include "car_config.h"

void main_test(void)
{
    line_following_task();
}

void before_main_test(void)
{
    while (1)
    {
        /* code */
    }
}

void test_in_interrupt(void)
{
#if TEST_IN_INTERRUPT == 1
#if TEST_TASK == 1
    // �������״̬
    typedef enum
    {
        TEST_STATE_RUNNING, // ����������
        TEST_STATE_DELAY,   // ������ɣ��ӳٵȴ�
    } TestState_t;

    static TestState_t test_state = TEST_STATE_RUNNING;
#if TASK_RUN_ONLY_ONCE == 0
    static uint32_t delay_end_time = 0;
#endif

    switch (test_state)
    {
    case TEST_STATE_RUNNING:
    {
        uint8_t task_finished = 0;
        // ִ��ѡ���Ĳ�������
#if TEST_CAR_TO_CROSSING == 1
        task_finished = Car_To_Crossing(TEST_CROSSING_NUM);
#elif TEST_CAR_TO_ROOM == 1
        room_target = TEST_ROOM_NUM; // ����ȫ��Ŀ��
        task_finished = Car_To_Room();
#elif TEST_OPEN_LOOP_STEERING == 1
        task_finished = open_loop_steering_control(TEST_OPEN_LOOP_STEERING_TIME, TEST_OPEN_LOOP_STEERING_SPEED, TEST_OPEN_LOOP_STEERING_SPEED_BASE);
#endif

        // ���������ɣ�������ӳ�״̬
        if (task_finished)
        {
            test_state = TEST_STATE_DELAY;
#if TASK_RUN_ONLY_ONCE == 0
            delay_end_time = HAL_GetTick() + (TEST_REPEAT_DELAY_S * 1000);
#endif
        }
        break;
    }
    case TEST_STATE_DELAY:
        // �ȴ��ӳ�ʱ�����
#if TASK_RUN_ONLY_ONCE == 0
        if (HAL_GetTick() >= delay_end_time)
        {
            test_state = TEST_STATE_RUNNING; // �ӳٽ�����׼�����¿�ʼ����
        }
#endif
        break;
    }
#elif TEST_DRIVER == 1
#if TEST_DRIVER_EMM_V5 == 1
    // ����Emm_V5����,��ת1s,��ת1s,���ѭ��,��Ҫ�������ķ�ʽʵ��
    typedef enum
    {
        MOTOR_TEST_FORWARD, // ��ת״̬
        MOTOR_TEST_REVERSE, // ��ת״̬
        MOTOR_TEST_WAIT     // ��ʱ�ȴ�״̬
    } MotorTestState_t;

    static MotorTestState_t motor_test_state = MOTOR_TEST_FORWARD;
    static MotorTestState_t next_motor_state = MOTOR_TEST_REVERSE; // ��¼��һ��Ҫ���������״̬
    static uint32_t wait_end_time = 0;

    switch (motor_test_state)
    {
    case MOTOR_TEST_FORWARD:
        // ������תָ��
        if (Emm_V5_Vel_Control(TEST_MOTOR_ADDR, 1, TEST_MOTOR_SPEED, DEFAULT_ACCELERATION, 0) == HAL_OK)
        {
            motor_test_state = MOTOR_TEST_WAIT;
            next_motor_state = MOTOR_TEST_REVERSE; // ��һ����ת
            wait_end_time = HAL_GetTick() + 1000;
        }
        break;

    case MOTOR_TEST_REVERSE:
        // ���ͷ�תָ��
        if (Emm_V5_Vel_Control(TEST_MOTOR_ADDR, 0, TEST_MOTOR_SPEED, DEFAULT_ACCELERATION, 0) == HAL_OK)
        {
            motor_test_state = MOTOR_TEST_WAIT;
            next_motor_state = MOTOR_TEST_FORWARD; // ��һ����ת
            wait_end_time = HAL_GetTick() + 1000;
        }
        break;

    case MOTOR_TEST_WAIT:
        // �ȴ�1�����
        if (HAL_GetTick() >= wait_end_time)
        {
            motor_test_state = next_motor_state; // ������һ������״̬
        }
        break;
    }
#elif TEST_CONTROL_SET_MOTOR_SPEED == 1
    // ����set_motor_speed����, �������������ת1s, ��ת1s, ���ѭ��
    typedef enum
    {
        SMS_TEST_FORWARD, // ��ת״̬
        SMS_TEST_REVERSE, // ��ת״̬
        SMS_TEST_WAIT     // ��ʱ�ȴ�״̬
    } SMSTestState_t;

    static SMSTestState_t sms_test_state = SMS_TEST_FORWARD;
    static SMSTestState_t next_sms_state = SMS_TEST_REVERSE;
    static uint32_t wait_end_time = 0;

    switch (sms_test_state)
    {
    case SMS_TEST_FORWARD:
        // ͨ��set_motor_speed���������������ת
        if (set_motor_speed(TEST_MOTOR_SPEED, TEST_MOTOR_SPEED, DEFAULT_ACCELERATION) == 0)
        {
            // ����ɹ���ӵ����У�����ȴ�״̬
            sms_test_state = SMS_TEST_WAIT;
            next_sms_state = SMS_TEST_REVERSE; // ��һ����ת
            wait_end_time = HAL_GetTick() + 1000;
        }
        break;

    case SMS_TEST_REVERSE:
        // ͨ��set_motor_speed���������������ת
        if (set_motor_speed(-TEST_MOTOR_SPEED, -TEST_MOTOR_SPEED, DEFAULT_ACCELERATION) == 0)
        {
            // ����ɹ���ӵ����У�����ȴ�״̬
            sms_test_state = SMS_TEST_WAIT;
            next_sms_state = SMS_TEST_FORWARD; // ��һ����ת
            wait_end_time = HAL_GetTick() + 1000;
        }
        break;

    case SMS_TEST_WAIT:
        // �ȴ�1�����
        if (HAL_GetTick() >= wait_end_time)
        {
            sms_test_state = next_sms_state; // ������һ������״̬
        }
        break;
    }
#endif
#endif
#endif
}
