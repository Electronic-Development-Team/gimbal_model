#ifndef CAR_CONFIG
#define CAR_CONFIG

#define TEST_IN_MAIN 0
#define TEST_IN_INTERRUPT 1 // ���ж��н��в���
#define TEST_BEFORE_MAIN 0

#define TEST_TASK 1                    // �������
#define TEST_DRIVER 0                  // ��������
#define TASK_RUN_ONLY_ONCE 1           // �����Ƿ�ֻ����һ��
#define TEST_DRIVER_EMM_V5 0           // Emm_V5����ֱ�Ӳ���
#define TEST_CONTROL_SET_MOTOR_SPEED 1 //  set_motor_speed ��������

#define TEST_MOTOR_ADDR 0x01 // Ҫ���Եĵ����ַ
#define TEST_MOTOR_SPEED 200 // ���������еĵ���ٶ� (RPM)

#define TEST_CAR_TO_ROOM 1
#define TEST_ROOM_NUM 1
#define TEST_CAR_TO_CROSSING 0
#define TEST_CROSSING_NUM 2
#define TEST_OPEN_LOOP_STEERING 0
#define TEST_OPEN_LOOP_STEERING_TIME 1000
#define TEST_OPEN_LOOP_STEERING_SPEED 40
#define TEST_OPEN_LOOP_STEERING_SPEED_BASE 70

#define TEST_REPEAT_DELAY_S 5 // ���β�������֮����ӳ�ʱ�䣨�룩

// Motion parameters
#define CAR_BASE_SPEED 50        // ����ѭ���ٶ� (��λ:ת/����)
#define CAR_MAX_SPEED 100        // ���ѭ���ٶ� (��λ:ת/����)
#define DEFAULT_ACCELERATION 230 // Ĭ�ϵ�����ٶ� (0-255)

// Steering control parameters
#define TURN_90_DURATION_MS 500 // 90��ת������ʱ�� (ms)
#define TURN_90_SPEED 50        // 90��ת��ʱ�ĵ������ (RPM)
#define TURN_90_BASE_SPEED 30   // 90��ת��ʱ�Ļ����ٶ� (RPM)

#define TURN_180_DURATION_MS 1000 // 180��ת������ʱ�� (ms)
#define TURN_180_SPEED 70         // 180��ת��ʱ�ĵ������ (RPM)
#define TURN_180_BASE_SPEED 0     // 180��ת��ʱ�Ļ����ٶ� (0��ʾԭ��ת��)

#define SCALE_FACTOR 6 // ����ϵ��

// State machine timing parameters
#define CROSSING_DEBOUNCE_MS 1000     // ʶ��Ϊ��·�ڵķ���ʱ�� (ms)
#define LEAVING_CROSSING_DELAY_MS 500 // ȷ���뿪·�ڵ��ӳ�ʱ�� (ms)

// Gray sensor parameters
#define CONSECUTIVE_GRAY_SENSORS 6 // �����Ҷȴ���������

#endif
