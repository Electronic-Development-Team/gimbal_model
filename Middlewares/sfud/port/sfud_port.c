/*
 * This file is part of the Serial Flash Universal Driver Library.
 *
 * Copyright (c) 2016, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2016-04-23
 */

#include "main.h"
#include "spi.h"
#include <sfud.h>
#include <stdarg.h>
#include <stdio.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>

typedef struct
{
    SPI_HandleTypeDef *spix;
    GPIO_TypeDef *cs_gpiox;
    uint16_t cs_gpio_pin;
} spi_user_data, *spi_user_data_t;

static char log_buf[256];

void sfud_log_debug(const char *file, const long line, const char *format, ...);

static void spi_lock(const sfud_spi *spi)
{
    __disable_irq();
}

static void spi_unlock(const sfud_spi *spi)
{
    __enable_irq();
}

/**
 * SPI write data then read data
 */
static sfud_err spi_write_read(const sfud_spi *spi, const uint8_t *write_buf, size_t write_size, uint8_t *read_buf,
                               size_t read_size)
{
    sfud_err result = SFUD_SUCCESS;
    spi_user_data_t spi_dev = (spi_user_data_t)spi->user_data;

    if (write_size)
    {
        SFUD_ASSERT(write_buf);
    }
    if (read_size)
    {
        SFUD_ASSERT(read_buf);
    }

    HAL_GPIO_WritePin(spi_dev->cs_gpiox, spi_dev->cs_gpio_pin, GPIO_PIN_RESET);

    if (write_size != 0)
    {
        if (HAL_OK != HAL_SPI_Transmit(spi_dev->spix, (uint8_t *)write_buf, write_size, 1000))
            result = SFUD_ERR_TIMEOUT;
    }

    if (read_size != 0)
    {
        if (HAL_OK != HAL_SPI_Receive(spi_dev->spix, (uint8_t *)read_buf, read_size, 1000))
            result = SFUD_ERR_TIMEOUT;
    }

    HAL_GPIO_WritePin(spi_dev->cs_gpiox, spi_dev->cs_gpio_pin, GPIO_PIN_SET);

    return result;
}

/* about 100 microsecond delay */
static void retry_delay_100us(void)
{
    uint32_t cnt = 10000;
    while (cnt--)
        __NOP();
}

static spi_user_data spi1 = {.spix = &hspi2, .cs_gpiox = SPI2_CS_GPIO_Port, .cs_gpio_pin = SPI2_CS_Pin};
sfud_err sfud_spi_port_init(sfud_flash *flash)
{
    sfud_err result = SFUD_SUCCESS;

    switch (flash->index)
    {
    case SFUD_W25_DEVICE_INDEX:
    {
        /* 同步 Flash 移植所需的接口及数据 */
        flash->spi.wr = spi_write_read;
        flash->spi.lock = spi_lock;
        flash->spi.unlock = spi_unlock;
        flash->spi.user_data = &spi1;
        /* about 100 microsecond delay */
        flash->retry.delay = retry_delay_100us;
        /* adout 60 seconds timeout */
        flash->retry.times = 60 * 10000;

        break;
    }
    }

    return result;
}

/**
 * This function is print debug info.
 *
 * @param file the file which has call this function
 * @param line the line number which has call this function
 * @param format output format
 * @param ... args
 */
void sfud_log_debug(const char *file, const long line, const char *format, ...)
{
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    printf("[SFUD](%s:%ld) ", file, line);
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\r\n", log_buf);
    va_end(args);
}

/**
 * This function is print routine info.
 *
 * @param format output format
 * @param ... args
 */
void sfud_log_info(const char *format, ...)
{
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    printf("[SFUD]");
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\r\n", log_buf);
    va_end(args);
}
