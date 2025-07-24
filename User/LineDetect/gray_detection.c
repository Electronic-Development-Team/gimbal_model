#include "gray_detection.h"
#include "car_config.h"

_gray_state gray_state;

float gray_status[2] = {0}, gray_status_backup[2][20] = {0}; // �Ҷȴ�����״̬����ʷֵ
uint32_t gray_status_worse = 0;                              // �Ҷȹ��쳣״̬������
u16 scaleFactor = SCALE_FACTOR;                               // ����ϵ��

/***************************************************
������: int Calculate_Turn_Value(void)
˵��:	���ݻҶȴ�������ֵ����ת��ֵ
���:	��
����:	ת��ֵ����תΪ������תΪ����
��ע:	��⵽����ʱgray_state.gray.bitXΪ1������Ϊ0
             gray_state.gray.bit1Ϊ��ͷ���Ҳ�Ĵ�������ֵ
             gray_state.gray.bit12Ϊ����ഫ������ֵ
****************************************************/
int Calculate_Turn_Value(void)
{
    // ��������״̬��Ȩ�ط��������У�����ѭ������
    const uint8_t sensors[12] = {
        gray_state.gray.bit1, gray_state.gray.bit2, gray_state.gray.bit3,
        gray_state.gray.bit4, gray_state.gray.bit5, gray_state.gray.bit6,
        gray_state.gray.bit7, gray_state.gray.bit8, gray_state.gray.bit9,
        gray_state.gray.bit10, gray_state.gray.bit11, gray_state.gray.bit12};
    const int weights[12] = {-16, -11, -8, -5, -3, -1, 1, 3, 5, 8, 11, 16};

    int weighted_sum = 0;
    int active_sensors = 0;
    int consecutive_count = 0;
    int max_consecutive = 0;

    // ��һ��ѭ���м���������Ҫ��ֵ
    for (int i = 0; i < 12; i++)
    {
        if (sensors[i])
        {
            weighted_sum += weights[i];
            active_sensors++;
            consecutive_count++;
        }
        else
        {
            if (consecutive_count > max_consecutive)
            {
                max_consecutive = consecutive_count;
            }
            consecutive_count = 0; // ������������
        }
    }

    // ѭ���������ٴμ���Բ����β����������
    if (consecutive_count > max_consecutive)
    {
        max_consecutive = consecutive_count;
    }

    // �µ��ж��߼���ֻ�е���⵽CONSECUTIVE_GRAY_SENSORS������� **����** �Ĵ�����ʱ������Ϊ��ʮ��·��
    if (max_consecutive >= CONSECUTIVE_GRAY_SENSORS)
    {
        return INT16_MAX; // ����һ������ֵ����ʾС���Ѿ�����ʮ��·������
    }

    // ���û�д�������⵽���ߣ�����һ����Сֵ
    if (active_sensors == 0)
    {
        return INT16_MIN; // ��ʾû����Ч��ת��ֵ
    }

    // �����Ȩƽ��ֵ
    float position = (float)weighted_sum / active_sensors;

    // ת��ΪPWMֵ������ʹ��һ������ϵ���Ŵ�Ч��
    // ���Ը���ʵ������������ϵ��
    int turn_pwm = (int)(position * scaleFactor); // ����ϵ��20���Ը���ʵ�ʿ���Ч������

    // ��PWMֵ�����޷�����ֹ�����С
    if (turn_pwm > turn_PWM_Limit)
        turn_pwm = turn_PWM_Limit;
    if (turn_pwm < -turn_PWM_Limit)
        turn_pwm = -turn_PWM_Limit;

    return turn_pwm; // ����ת��ֵ����תΪ������תΪ��
}

/***************************************************
������: void gpio_input_check_channel_12_linewidth_10mm(void)
˵��:	12·�Ҷȹ�gpio���
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void gpio_input_check_channel_12_linewidth_10mm(void)
{
    for (uint16_t i = 19; i > 0; i--)
    {
        gray_status_backup[0][i] = gray_status_backup[0][i - 1];
    }
    gray_status_backup[0][0] = gray_status[0];
    switch (gray_state.state)
    {
    case 0x0001:
        gray_status[0] = -11;
        gray_status_worse /= 2;
        break; // 000000000001b
    case 0x0003:
        gray_status[0] = -10;
        gray_status_worse /= 2;
        break; // 000000000011b
    case 0x0002:
        gray_status[0] = -9;
        gray_status_worse /= 2;
        break; // 000000000010b
    case 0x0006:
        gray_status[0] = -8;
        gray_status_worse /= 2;
        break; // 000000000110b
    case 0x0004:
        gray_status[0] = -7;
        gray_status_worse /= 2;
        break; // 000000000100b
    case 0x000C:
        gray_status[0] = -6;
        gray_status_worse /= 2;
        break; // 000000001100b
    case 0x0008:
        gray_status[0] = -5;
        gray_status_worse /= 2;
        break; // 000000001000b
    case 0x0018:
        gray_status[0] = -4;
        gray_status_worse /= 2;
        break; // 000000011000b
    case 0x0010:
        gray_status[0] = -3;
        gray_status_worse /= 2;
        break; // 000000010000b
    case 0x0030:
        gray_status[0] = -2;
        gray_status_worse /= 2;
        break; // 000000110000b
    case 0x0020:
        gray_status[0] = -1;
        gray_status_worse /= 2;
        break; // 000000100000b
    case 0x0060:
        gray_status[0] = 0;
        gray_status_worse /= 2;
        break; // 000001100000b
    case 0x0040:
        gray_status[0] = 1;
        gray_status_worse /= 2;
        break; // 000001000000b
    case 0x00C0:
        gray_status[0] = 2;
        gray_status_worse /= 2;
        break; // 000011000000b
    case 0x0080:
        gray_status[0] = 3;
        gray_status_worse /= 2;
        break; // 000010000000b
    case 0x0180:
        gray_status[0] = 4;
        gray_status_worse /= 2;
        break; // 000110000000b
    case 0x0100:
        gray_status[0] = 5;
        gray_status_worse /= 2;
        break; // 000100000000b
    case 0x0300:
        gray_status[0] = 6;
        gray_status_worse /= 2;
        break; // 001100000000b
    case 0x0200:
        gray_status[0] = 7;
        gray_status_worse /= 2;
        break; // 001000000000b
    case 0x0600:
        gray_status[0] = 8;
        gray_status_worse /= 2;
        break; // 011000000000b
    case 0x0400:
        gray_status[0] = 9;
        gray_status_worse /= 2;
        break; // 010000000000b
    case 0x0C00:
        gray_status[0] = 10;
        gray_status_worse /= 2;
        break; // 110000000000b
    case 0x0800:
        gray_status[0] = 11;
        gray_status_worse /= 2;
        break; // 100000000000b
    case 0x0000:
        gray_status[0] = gray_status_backup[0][0];
        gray_status_worse++;
        break; // 00000000b
    default:   // ����������������ж�
    {
        gray_status[0] = gray_status_backup[0][0];
        gray_status_worse++;
    }
    }

    static uint16_t tmp_cnt = 0;
    switch (gray_state.state) // ֹͣ�߼��
    {
    case 0x0030:
        tmp_cnt++;
        break; // 000000110000b
    case 0x0020:
        tmp_cnt++;
        break; // 000000100000b
    case 0x0060:
        tmp_cnt++;
        break; // 000001100000b
    case 0x0040:
        tmp_cnt++;
        break; // 000001000000b
    case 0x00C0:
        tmp_cnt++;
        break;   // 000011000000b
    case 0x00F0: // 000011110000b
    {
        if (tmp_cnt >= 10)
        {
            tmp_cnt = 0;
        }
    }
    break;
    }
}

void gpio_input_check_channel_12_linewidth_20mm(void)
{
    for (uint16_t i = 19; i > 0; i--)
    {
        gray_status_backup[1][i] = gray_status_backup[1][i - 1];
    }
    gray_status_backup[1][0] = gray_status[1];
    switch (gray_state.state)
    {
    case 0x0001:
        gray_status[1] = -11;
        break; // 000000000001b
    case 0x0003:
        gray_status[1] = -10;
        break; // 000000000011b
    case 0x0002:
        gray_status[1] = -9;
        break; // 000000000010b
    case 0x0007:
        gray_status[1] = -9;
        break; // 000000000111b
    case 0x0006:
        gray_status[1] = -8;
        break; // 000000000110b
    case 0x0004:
        gray_status[1] = -7;
        break; // 000000000100b
    case 0x000E:
        gray_status[1] = -7;
        break; // 000000001110b
    case 0x000C:
        gray_status[1] = -6;
        break; // 000000001100b
    case 0x0008:
        gray_status[1] = -5;
        break; // 000000001000b
    case 0x001C:
        gray_status[1] = -5;
        break; // 000000011100b
    case 0x0018:
        gray_status[1] = -4;
        break; // 000000011000b
    case 0x0010:
        gray_status[1] = -3;
        break; // 000000010000b
    case 0x0038:
        gray_status[1] = -3;
        break; // 000000111000b
    case 0x0030:
        gray_status[1] = -2;
        break; // 000000110000b
    case 0x0020:
        gray_status[1] = -1;
        break; // 000000100000b
    case 0x0070:
        gray_status[1] = -1;
        break; // 000001110000b
    case 0x0060:
        gray_status[1] = 0;
        break; // 000001100000b
    case 0x0040:
        gray_status[1] = 1;
        break; // 000001000000b
    case 0x00E0:
        gray_status[1] = 1;
        break; // 000011100000b
    case 0x00C0:
        gray_status[1] = 2;
        break; // 000011000000b
    case 0x0080:
        gray_status[1] = 3;
        break; // 000010000000b
    case 0x01C0:
        gray_status[1] = 3;
        break; // 000111000000b
    case 0x0180:
        gray_status[1] = 4;
        break; // 000110000000b
    case 0x0100:
        gray_status[1] = 5;
        break; // 000100000000b
    case 0x0380:
        gray_status[1] = 5;
        break; // 001110000000b
    case 0x0300:
        gray_status[1] = 6;
        break; // 001100000000b
    case 0x0200:
        gray_status[1] = 7;
        break; // 001000000000b
    case 0x0700:
        gray_status[1] = 7;
        break; // 011100000000b
    case 0x0600:
        gray_status[1] = 8;
        break; // 011000000000b
    case 0x0400:
        gray_status[1] = 9;
        break; // 010000000000b
    case 0x0E00:
        gray_status[1] = 9;
        break; // 111000000000b
    case 0x0C00:
        gray_status[1] = 10;
        break; // 110000000000b
    case 0x0800:
        gray_status[1] = 11;
        break; // 100000000000b
    case 0x0000:
    default: // ����������������ж�
    {
        gray_status[1] = 0;
    }
    }
}
/**
 * @brief ���»Ҷȴ���������
 * @note ͨ��I2C��ȡ12·�Ҷȴ���������
 * ����gray_state.gray.bit1~gray_state.gray.bit12��ֵ
 * ����gray_state.gray.bit1Ϊ��ͷ���Ҳ�Ĵ�������ֵ,
 * gray_state.gray.bit12Ϊ����ഫ������ֵ
 *
 */
void grey_sensor_Read(void)
{
    gray_state.state = pca9555_read_bit12(0x20 << 1); // ͨ��I2C��ʽ��ȡ12·�Ҷȴ���������
    gpio_input_check_channel_12_linewidth_10mm();     // ���10mm�����ߵ�ƫ����ȡ
    gpio_input_check_channel_12_linewidth_20mm();     // ���20mm�����ߵ�ƫ����ȡ
}

void grey_sensor_Init(void)
{
    i2c_CheckDevice(0x40); // �Ҷȴ�����IIC��ʼ��
}

void grey_sensorData_print(void)
{
    // �������ӡ
    printf("%d%d%d%d%d%d%d%d%d%d%d%d \t %d \t %f \t %f\n",
           gray_state.gray.bit12,
           gray_state.gray.bit11,
           gray_state.gray.bit10,
           gray_state.gray.bit9,
           gray_state.gray.bit8,
           gray_state.gray.bit7,
           gray_state.gray.bit6,
           gray_state.gray.bit5,
           gray_state.gray.bit4,
           gray_state.gray.bit3,
           gray_state.gray.bit2,
           gray_state.gray.bit1,
           gray_state.state, gray_status[0], gray_status[1]);
}

/**
 * @brief ���ָ������ĻҶȴ������Ƿ��м�⵽����
 * @param start_graySensor ��ʼ�Ҷȴ���������(��1��ʼ)
 * @param end_graySensor �����Ҷȴ���������
 * @return int ���ؼ�⵽���ߵĴ�����ֵ����INT16_MIN��ʾû�м�⵽����
 */
int IsCertainGraySenorsAcrived(uint8_t start_graySensor, uint8_t end_graySensor)
{
    // ȷ����������������Ч��Χ��
    if (start_graySensor < 1)
        start_graySensor = 1;
    if (end_graySensor > 12)
        end_graySensor = 12;

    // ��鴫�����Ķ�Ӧλ
    for (uint8_t i = start_graySensor; i <= end_graySensor; i++)
    {
        // ����gray_state.gray.bit1��Ӧstate�����λ��������Ҫ�����Ӧ��λ
        // ������1��Ӧ������λ0��������2��Ӧ������λ1���Դ�����
        if (gray_state.state & (1 << (i - 1)))
        {
            printf("������%d��⵽����\n", i);
            return i;
        }
    }

    return INT16_MIN; // δ��⵽����
}
