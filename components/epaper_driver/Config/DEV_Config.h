/*****************************************************************************
* | File      	:   DEV_Config.h
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master 
*                and enhance portability
*----------------
* |	This version:   V2.0
* | Date        :   2018-10-30
* | Info        :
* 1.add:
*   UBYTE\UWORD\UDOUBLE
* 2.Change:
*   EPD_RST -> EPD_RST_PIN
*   EPD_DC -> EPD_DC_PIN
*   EPD_CS -> EPD_CS_PIN
*   EPD_BUSY -> EPD_BUSY_PIN
* 3.Remote:
*   EPD_RST_1\EPD_RST_0
*   EPD_DC_1\EPD_DC_0
*   EPD_CS_1\EPD_CS_0
*   EPD_BUSY_1\EPD_BUSY_0
* 3.add:
*   #define DEV_Digital_Write(_pin, _value) bcm2835_gpio_write(_pin, _value)
*   #define DEV_Digital_Read(_pin) bcm2835_gpio_lev(_pin)
*   #define DEV_SPI_WriteByte(__value) bcm2835_spi_transfer(__value)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#include <stdint.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/task.h"

// CSB（CS）：从机片选信号，低电平有效，为低电平的时候，芯片使能。
// SCL（SCK/SCLK）：串行时钟信号。
// D/C（DC）：数据/命令控制信号，低电平时写入命令（Command）；高电平时写入数据（Data/parameter）。
// SDA（DIN）：串行数据信号。
// 时序：CPHL=0，CPOL=0，即 SPI 模式0。

// To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
// but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16

extern spi_device_handle_t spi_EPD;

#define LCD_HOST SPI2_HOST

/**
 * e-Paper GPIO
**/
// ESP_IDF
#define PIN_NUM_MISO -1 //unused
#define PIN_NUM_MOSI 3
#define PIN_NUM_CLK 2
#define PIN_NUM_CS 7
#define PIN_NUM_DC 6
#define PIN_NUM_RST 10
#define PIN_NUM_BUSY 11
// PlatformIO
// #define EPD_SCK_PIN 2
// #define EPD_MOSI_PIN 3
// #define EPD_CS_PIN 7
// #define EPD_RST_PIN 10
// #define EPD_DC_PIN 6
// #define EPD_BUSY_PIN 11

#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0

/**
 * data
**/
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/**
 * GPIO read and write
**/
#define DEV_Digital_Write(_pin, _value) gpio_set_level(_pin, _value == 0? GPIO_PIN_RESET:GPIO_PIN_SET)
#define DEV_Digital_Read(_pin) gpio_get_level(_pin)

/**
 * delay x ms
**/
#define DEV_Delay_ms(__xms) vTaskDelay(pdMS_TO_TICKS(__xms));

void DEV_SPI_WriteByte(UBYTE value);
void DEV_SPI_Write_nByte(UBYTE *value, UDOUBLE len);

int DEV_Module_Init(void);
void DEV_Module_Exit(void);
void EPD_GPIO_init();
void EPD_SPI_init();

#endif
