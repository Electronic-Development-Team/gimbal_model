#include "Emm_V5.h"
#include <stdbool.h>

/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

extern UART_HandleTypeDef Emm_V5_HUART;
// ����һ����̬�ķ��ͻ���������ȷ��DMA��ȫ����
static uint8_t Emm_V5_TX_Buffer[32];

/**
 * @brief    ����ǰλ������
 * @param    addr  �������ַ
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr; // ��ַ
  Emm_V5_TX_Buffer[1] = 0x0A; // ������
  Emm_V5_TX_Buffer[2] = 0x6D; // ������
  Emm_V5_TX_Buffer[3] = 0x6B; // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 4);
}

/**
 * @brief    �����ת����
 * @param    addr  �������ַ
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr; // ��ַ
  Emm_V5_TX_Buffer[1] = 0x0E; // ������
  Emm_V5_TX_Buffer[2] = 0x52; // ������
  Emm_V5_TX_Buffer[3] = 0x6B; // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 4);
}

/**
 * @brief    ��ȡϵͳ����
 * @param    addr  �������ַ
 * @param    s     ��ϵͳ��������
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;

  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[i] = addr;
  ++i; // ��ַ

  switch (s) // ������
  {
  case S_VER:
    Emm_V5_TX_Buffer[i] = 0x1F;
    ++i;
    break;
  case S_RL:
    Emm_V5_TX_Buffer[i] = 0x20;
    ++i;
    break;
  case S_PID:
    Emm_V5_TX_Buffer[i] = 0x21;
    ++i;
    break;
  case S_VBUS:
    Emm_V5_TX_Buffer[i] = 0x24;
    ++i;
    break;
  case S_CPHA:
    Emm_V5_TX_Buffer[i] = 0x27;
    ++i;
    break;
  case S_ENCL:
    Emm_V5_TX_Buffer[i] = 0x31;
    ++i;
    break;
  case S_TPOS:
    Emm_V5_TX_Buffer[i] = 0x33;
    ++i;
    break;
  case S_VEL:
    Emm_V5_TX_Buffer[i] = 0x35;
    ++i;
    break;
  case S_CPOS:
    Emm_V5_TX_Buffer[i] = 0x36;
    ++i;
    break;
  case S_PERR:
    Emm_V5_TX_Buffer[i] = 0x37;
    ++i;
    break;
  case S_FLAG:
    Emm_V5_TX_Buffer[i] = 0x3A;
    ++i;
    break;
  case S_ORG:
    Emm_V5_TX_Buffer[i] = 0x3B;
    ++i;
    break;
  case S_Conf:
    Emm_V5_TX_Buffer[i] = 0x42;
    ++i;
    Emm_V5_TX_Buffer[i] = 0x6C;
    ++i;
    break;
  case S_State:
    Emm_V5_TX_Buffer[i] = 0x43;
    ++i;
    Emm_V5_TX_Buffer[i] = 0x7A;
    ++i;
    break;
  default:
    return HAL_ERROR; // ��Ч�Ĳ�������
  }

  Emm_V5_TX_Buffer[i] = 0x6B;
  ++i; // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, i);
}

/**
 * @brief    �޸Ŀ���/�ջ�����ģʽ
 * @param    addr     �������ַ
 * @param    svF      ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
 * @param    ctrl_mode������ģʽ����Ӧ��Ļ�ϵ�P_Pul�˵�����0�ǹر������������ţ�1�ǿ���ģʽ��2�Ǳջ�ģʽ��3����En�˿ڸ���Ϊ��Ȧ��λ�����������ţ�Dir�˿ڸ���Ϊ��λ����ߵ�ƽ����
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr;      // ��ַ
  Emm_V5_TX_Buffer[1] = 0x46;      // ������
  Emm_V5_TX_Buffer[2] = 0x69;      // ������
  Emm_V5_TX_Buffer[3] = svF;       // �Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  Emm_V5_TX_Buffer[4] = ctrl_mode; // ����ģʽ����Ӧ��Ļ�ϵ�P_Pul�˵�����0�ǹر������������ţ�1�ǿ���ģʽ��2�Ǳջ�ģʽ��3����En�˿ڸ���Ϊ��Ȧ��λ�����������ţ�Dir�˿ڸ���Ϊ��λ����ߵ�ƽ����
  Emm_V5_TX_Buffer[5] = 0x6B;      // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 6);
}

/**
 * @brief    ʹ���źſ���
 * @param    addr  �������ַ
 * @param    state ��ʹ��״̬     ��trueΪʹ�ܵ����falseΪ�رյ��
 * @param    snF   �����ͬ����־ ��falseΪ�����ã�trueΪ����
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr;           // ��ַ
  Emm_V5_TX_Buffer[1] = 0xF3;           // ������
  Emm_V5_TX_Buffer[2] = 0xAB;           // ������
  Emm_V5_TX_Buffer[3] = (uint8_t)state; // ʹ��״̬
  Emm_V5_TX_Buffer[4] = snF;            // ���ͬ���˶���־
  Emm_V5_TX_Buffer[5] = 0x6B;           // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 6);
}

/**
 * @brief    �ٶ�ģʽ
 * @param    addr�������ַ
 * @param    dir ������       ��0ΪCW������ֵΪCCW
 * @param    vel ���ٶ�       ����Χ0 - 5000RPM
 * @param    acc �����ٶ�     ����Χ0 - 255��ע�⣺0��ֱ������
 * @param    snF �����ͬ����־��falseΪ�����ã�trueΪ����
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr;                // ��ַ
  Emm_V5_TX_Buffer[1] = 0xF6;                // ������
  Emm_V5_TX_Buffer[2] = dir;                 // ����
  Emm_V5_TX_Buffer[3] = (uint8_t)(vel >> 8); // �ٶ�(RPM)��8λ�ֽ�
  Emm_V5_TX_Buffer[4] = (uint8_t)(vel >> 0); // �ٶ�(RPM)��8λ�ֽ�
  Emm_V5_TX_Buffer[5] = acc;                 // ���ٶȣ�ע�⣺0��ֱ������
  Emm_V5_TX_Buffer[6] = snF;                 // ���ͬ���˶���־
  Emm_V5_TX_Buffer[7] = 0x6B;                // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 8);
}

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
HAL_StatusTypeDef Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr;                 // ��ַ
  Emm_V5_TX_Buffer[1] = 0xFD;                 // ������
  Emm_V5_TX_Buffer[2] = dir;                  // ����
  Emm_V5_TX_Buffer[3] = (uint8_t)(vel >> 8);  // �ٶ�(RPM)��8λ�ֽ�
  Emm_V5_TX_Buffer[4] = (uint8_t)(vel >> 0);  // �ٶ�(RPM)��8λ�ֽ�
  Emm_V5_TX_Buffer[5] = acc;                  // ���ٶȣ�ע�⣺0��ֱ������
  Emm_V5_TX_Buffer[6] = (uint8_t)(clk >> 24); // ������(bit24 - bit31)
  Emm_V5_TX_Buffer[7] = (uint8_t)(clk >> 16); // ������(bit16 - bit23)
  Emm_V5_TX_Buffer[8] = (uint8_t)(clk >> 8);  // ������(bit8  - bit15)
  Emm_V5_TX_Buffer[9] = (uint8_t)(clk >> 0);  // ������(bit0  - bit7 )
  Emm_V5_TX_Buffer[10] = raF;                 // ��λ/���Ա�־
  Emm_V5_TX_Buffer[11] = snF;                 // ���ͬ���˶���־
  Emm_V5_TX_Buffer[12] = 0x6B;                // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 13);
}

/**
 * @brief    ����ֹͣ�����п���ģʽ��ͨ�ã�
 * @param    addr  �������ַ
 * @param    snF   �����ͬ����־��falseΪ�����ã�trueΪ����
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr; // ��ַ
  Emm_V5_TX_Buffer[1] = 0xFE; // ������
  Emm_V5_TX_Buffer[2] = 0x98; // ������
  Emm_V5_TX_Buffer[3] = snF;  // ���ͬ���˶���־
  Emm_V5_TX_Buffer[4] = 0x6B; // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 5);
}

/**
 * @brief    ���ͬ���˶�
 * @param    addr  �������ַ
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Synchronous_motion(uint8_t addr)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr; // ��ַ
  Emm_V5_TX_Buffer[1] = 0xFF; // ������
  Emm_V5_TX_Buffer[2] = 0x66; // ������
  Emm_V5_TX_Buffer[3] = 0x6B; // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 4);
}

/**
 * @brief    ���õ�Ȧ��������λ��
 * @param    addr  �������ַ
 * @param    svF   ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr; // ��ַ
  Emm_V5_TX_Buffer[1] = 0x93; // ������
  Emm_V5_TX_Buffer[2] = 0x88; // ������
  Emm_V5_TX_Buffer[3] = svF;  // �Ƿ�洢��־
  Emm_V5_TX_Buffer[4] = 0x6B; // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 5);
}

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
HAL_StatusTypeDef Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr;                    // ��ַ
  Emm_V5_TX_Buffer[1] = 0x4C;                    // ������
  Emm_V5_TX_Buffer[2] = 0xAE;                    // ������
  Emm_V5_TX_Buffer[3] = svF;                     // �Ƿ�洢��־
  Emm_V5_TX_Buffer[4] = o_mode;                  // ����ģʽ
  Emm_V5_TX_Buffer[5] = o_dir;                   // ���㷽��
  Emm_V5_TX_Buffer[6] = (uint8_t)(o_vel >> 8);   // �����ٶ�(RPM)��8λ�ֽ�
  Emm_V5_TX_Buffer[7] = (uint8_t)(o_vel >> 0);   // �����ٶ�(RPM)��8λ�ֽ�
  Emm_V5_TX_Buffer[8] = (uint8_t)(o_tm >> 24);   // ���㳬ʱʱ��(bit24 - bit31)
  Emm_V5_TX_Buffer[9] = (uint8_t)(o_tm >> 16);   // ���㳬ʱʱ��(bit16 - bit23)
  Emm_V5_TX_Buffer[10] = (uint8_t)(o_tm >> 8);   // ���㳬ʱʱ��(bit8  - bit15)
  Emm_V5_TX_Buffer[11] = (uint8_t)(o_tm >> 0);   // ���㳬ʱʱ��(bit0  - bit7 )
  Emm_V5_TX_Buffer[12] = (uint8_t)(sl_vel >> 8); // ����λ��ײ������ת��(RPM)��8λ�ֽ�
  Emm_V5_TX_Buffer[13] = (uint8_t)(sl_vel >> 0); // ����λ��ײ������ת��(RPM)��8λ�ֽ�
  Emm_V5_TX_Buffer[14] = (uint8_t)(sl_ma >> 8);  // ����λ��ײ���������(Ma)��8λ�ֽ�
  Emm_V5_TX_Buffer[15] = (uint8_t)(sl_ma >> 0);  // ����λ��ײ���������(Ma)��8λ�ֽ�
  Emm_V5_TX_Buffer[16] = (uint8_t)(sl_ms >> 8);  // ����λ��ײ������ʱ��(Ms)��8λ�ֽ�
  Emm_V5_TX_Buffer[17] = (uint8_t)(sl_ms >> 0);  // ����λ��ײ������ʱ��(Ms)��8λ�ֽ�
  Emm_V5_TX_Buffer[18] = potF;                   // �ϵ��Զ���������
  Emm_V5_TX_Buffer[19] = 0x6B;                   // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 20);
}

/**
 * @brief    ��������
 * @param    addr   �������ַ
 * @param    o_mode ������ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
 * @param    snF   �����ͬ����־��falseΪ�����ã�trueΪ����
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr;   // ��ַ
  Emm_V5_TX_Buffer[1] = 0x9A;   // ������
  Emm_V5_TX_Buffer[2] = o_mode; // ����ģʽ
  Emm_V5_TX_Buffer[3] = snF;    // ���ͬ���˶���־
  Emm_V5_TX_Buffer[4] = 0x6B;   // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 5);
}

/**
 * @brief    ǿ���жϲ��˳�����
 * @param    addr  �������ַ
 * @retval   HAL_StatusTypeDef��HAL״̬(HAL_OK, HAL_ERROR, HAL_BUSY)
 */
HAL_StatusTypeDef Emm_V5_Origin_Interrupt(uint8_t addr)
{
  // ��鴮���Ƿ���æ
  if (Emm_V5_HUART.gState != HAL_UART_STATE_READY)
    return HAL_BUSY;

  // װ������
  Emm_V5_TX_Buffer[0] = addr; // ��ַ
  Emm_V5_TX_Buffer[1] = 0x9C; // ������
  Emm_V5_TX_Buffer[2] = 0x48; // ������
  Emm_V5_TX_Buffer[3] = 0x6B; // У���ֽ�

  // ��������
  return HAL_UART_Transmit_DMA(&Emm_V5_HUART, Emm_V5_TX_Buffer, 4);
}
