#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "usart.h"
#include <stdbool.h>

/**********************************************************
���ܣ�
����һ�����ڿ���Emm_V5��������Ŀ��ļ�����Ҫ���ڿ��Ƶ�����ٶȡ�λ�á�����Ȳ�����
��������һ��ͨ���ŷ�ϵͳ���ƵĲ���������Ծ�ȷ��ͨ���ջ��ķ�ʽ���Ƶ����λ��,�ٶ�
����,���ҿ��Խ��ж��ͬ�����ơ�
**********************************************************/

#define Emm_V5_HUART huart3 // ʹ�ô���3���Ƶ��
#define ABS(x) ((x) > 0 ? (x) : -(x))

typedef enum
{
    S_VER = 0,    /* ��ȡ�̼��汾�Ͷ�Ӧ��Ӳ���汾 */
    S_RL = 1,     /* ��ȡ��ȡ���������� */
    S_PID = 2,    /* ��ȡPID���� */
    S_VBUS = 3,   /* ��ȡ���ߵ�ѹ */
    S_CPHA = 5,   /* ��ȡ����� */
    S_ENCL = 7,   /* ��ȡ�������Ի�У׼��ı�����ֵ */
    S_TPOS = 8,   /* ��ȡ���Ŀ��λ�ýǶ� */
    S_VEL = 9,    /* ��ȡ���ʵʱת�� */
    S_CPOS = 10,  /* ��ȡ���ʵʱλ�ýǶ� */
    S_PERR = 11,  /* ��ȡ���λ�����Ƕ� */
    S_FLAG = 13,  /* ��ȡʹ��/��λ/��ת״̬��־λ */
    S_Conf = 14,  /* ��ȡ�������� */
    S_State = 15, /* ��ȡϵͳ״̬���� */
    S_ORG = 16,   /* ��ȡ���ڻ���/����ʧ��״̬��־λ */
} SysParams_t;

/**********************************************************
*** ע�⣺ÿ�������Ĳ����ľ���˵��������Ķ�Ӧ������ע��˵��
**********************************************************/

/**
 * @brief    ����ǰλ������
 * @param    addr  �������ַ
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);

/**
 * @brief    �����ת����
 * @param    addr  �������ַ
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Reset_Clog_Pro(uint8_t addr);

/**
 * @brief    ��ȡϵͳ����
 * @param    addr  �������ַ
 * @param    s     ��ϵͳ��������
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);

/**
 * @brief    �޸Ŀ���/�ջ�����ģʽ
 * @param    addr     �������ַ
 * @param    svF      ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
 * @param    ctrl_mode������ģʽ����Ӧ��Ļ�ϵ�P_Pul�˵�����0�ǹر������������ţ�1�ǿ���ģʽ��2�Ǳջ�ģʽ��3����En�˿ڸ���Ϊ��Ȧ��λ�����������ţ�Dir�˿ڸ���Ϊ��λ����ߵ�ƽ����
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);

/**
 * @brief    ʹ���źſ���
 * @param    addr  �������ַ
 * @param    state ��ʹ��״̬     ��trueΪʹ�ܵ����falseΪ�رյ��
 * @param    snF   �����ͬ����־ ��falseΪ�����ã�trueΪ����
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_En_Control(uint8_t addr, bool state, bool snF);

/**
 * @brief    �ٶ�ģʽ
 * @param    addr�������ַ
 * @param    dir ������       ��0ΪCW������ֵΪCCW
 * @param    vel ���ٶ�       ����Χ0 - 5000RPM
 * @param    acc �����ٶ�     ����Χ0 - 255��ע�⣺0��ֱ������
 * (���ٶ���245���ϵĵ���Ż�õ��ȽϺ��ʵļ��ٶ�,245���µ�ʱ�������ٶȻ�ǳ���,������㹫ʽ����:
 * ���ٶȵ�λΪ0��ʾ��ʹ�����߼Ӽ��٣�ֱ�Ӱ����趨���ٶ����С����߼Ӽ���ʱ����㹫ʽ��
 * t2 - t1 = (256 - acc) * 50(us)��Vt2 = Vt1 + 1(RPM))
 * @param    snF �����ͬ����־��falseΪ�����ã�trueΪ����
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);

/**
 * @brief    λ��ģʽ
 * @param    addr�������ַ
 * @param    dir ������        ��0ΪCW������ֵΪCCW
 * @param    vel ���ٶ�(RPM)   ����Χ0 - 5000RPM
 * @param    acc �����ٶ�      ����Χ0 - 255��ע�⣺0��ֱ������
 * @param    clk ��������      ����Χ0- (2^32 - 1)��
 * @param    raF ����λ/���Ա�־��falseΪ����˶���trueΪ����ֵ�˶�
 * @param    snF �����ͬ����־ ��falseΪ�����ã�trueΪ����
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);

/**
 * @brief    ����ֹͣ�����п���ģʽ��ͨ�ã�
 * @param    addr  �������ַ
 * @param    snF   �����ͬ����־��falseΪ�����ã�trueΪ����
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Stop_Now(uint8_t addr, bool snF);

/**
 * @brief    ���ͬ���˶�
 * @param    addr  �������ַ
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Synchronous_motion(uint8_t addr);

/**
 * @brief    ���õ�Ȧ��������λ��
 * @param    addr  �������ַ
 * @param    svF   ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Origin_Set_O(uint8_t addr, bool svF);

/**
 * @brief    �޸Ļ������
 * @param    addr  �������ַ
 * @param    svF   ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
 * @param    o_mode ������ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
 * @param    o_dir  �����㷽��0ΪCW������ֵΪCCW
 * @param    o_vel  �������ٶȣ���λ��RPM��ת/���ӣ�
 * @param    o_tm   �����㳬ʱʱ�䣬��λ������
 * @param    sl_vel ������λ��ײ������ת�٣���λ��RPM��ת/���ӣ�
 * @param    sl_ma  ������λ��ײ�������������λ��Ma��������
 * @param    sl_ms  ������λ��ײ������ʱ�䣬��λ��Ms�����룩
 * @param    potF   ���ϵ��Զ��������㣬falseΪ��ʹ�ܣ�trueΪʹ��
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);

/**
 * @brief    ��������
 * @param    addr   �������ַ
 * @param    o_mode ������ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
 * @param    snF   �����ͬ����־��falseΪ�����ã�trueΪ����
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);

/**
 * @brief    ǿ���жϲ��˳�����
 * @param    addr  �������ַ
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Origin_Interrupt(uint8_t addr);

#endif
