#ifndef __NCHD12_H
#define __NCHD12_H
#include "main.h"
/************************************************************PCA9555 ����***********************************************************/
#define SUCCESS 0
#define ERROR 1

#define SLAVE_ADDR0 0x40

#define HOST_WRITE_COMMAND 0x00
#define HOST_READ_COMMAND 0x01

#define INPUT_PORT_REGISTER0 0x00              /* ����˿ڼĴ���0������IO00-IO07 */
#define INPUT_PORT_REGISTER1 0x01              /* ����˿ڼĴ���1������IO10-IO17 */
#define OUTPUT_PORT_REGISTER0 0x02             /* ����˿ڼĴ���0������IO00-IO07 */
#define OUTPUT_PORT_REGISTER1 0x03             /* ����˿ڼĴ���1������IO10-IO17 */
#define POLARITY_INVERSION_PORT_REGISTER0 0x04 /* ���Է�ת�˿ڼĴ���0������IO00-IO07 */
#define POLARITY_INVERSION_PORT_REGISTER1 0x05 /* ���Է�ת�˿ڼĴ���1������IO10-IO17 */
#define CONFIG_PORT_REGISTER0 0x06             /* ����˿ڼĴ���0������IO00-IO07 */
#define CONFIG_PORT_REGISTER1 0x07             /* ����˿ڼĴ���1������IO10-IO17 */

#define GPIO_PORT0 0
#define GPIO_PORT1 1

#define GPIO_0 0x01
#define GPIO_1 0x02
#define GPIO_2 0x04
#define GPIO_3 0x08
#define GPIO_4 0x10
#define GPIO_5 0x20
#define GPIO_6 0x40
#define GPIO_7 0x80

uint16_t pcf8575_read_bit12(uint8_t slave_num);
uint16_t pca9555_read_bit12(uint8_t slave_num);

#endif
