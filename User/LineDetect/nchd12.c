#include "soft_i2c.h"
#include "nchd12.h"

uint8_t flag = 0;
uint16_t pcf8575_read_bit12(uint8_t slave_num)
{
	uint8_t hdata, ldata;
	uint16_t bit12;
	i2c_Start();
	i2c_SendByte(slave_num | HOST_READ_COMMAND);
	uint8_t ack = i2c_WaitAck();
	flag = ack;
	ldata = i2c_ReadByte(1);
	hdata = i2c_ReadByte(0);
	i2c_Stop();
	bit12 = (uint16_t)(hdata << 8 | ldata) & 0x0fff;
	return bit12;
}

uint16_t pca9555_read_bit12(uint8_t slave_num)
{
	uint8_t hdata, ldata;
	uint16_t bit12;
	i2c_Start();
	i2c_SendByte(slave_num); //	д��ӻ���ַ
	i2c_WaitAck();
	i2c_SendByte(INPUT_PORT_REGISTER0); // д��Ҫ��ȡ�ļĴ�����ַ
	i2c_WaitAck();
	i2c_Start();																 /* ��ʼ�������� */
	i2c_SendByte(slave_num | HOST_READ_COMMAND); // ���ʹӻ���ַ������Ϊ��ȡ
	i2c_WaitAck();
	ldata = i2c_ReadByte(1);
	hdata = i2c_ReadByte(0);
	i2c_Stop();
	bit12 = (uint16_t)(hdata << 8 | ldata) & 0x0fff;
	return bit12;
}
