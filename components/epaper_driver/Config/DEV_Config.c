/*****************************************************************************
* | File      	:   DEV_Config.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master
*                and enhance portability
*----------------
* |	This version:   V2.0
* | Date        :   2018-10-30
* | Info        :
# ******************************************************************************
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
#include "DEV_Config.h"
// #include "stm32f1xx_hal_spi.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <string.h>

// This function is called (in irq context!) just before a transmission starts. It will
// set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

// Initialize non-SPI GPIOs
void EPD_GPIO_init()
{
    // Initialize non-SPI GPIOs
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL << PIN_NUM_DC) | (1ULL << PIN_NUM_RST) | (1ULL << PIN_NUM_CS));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = false;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = ((1ULL << PIN_NUM_BUSY));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = false;
    gpio_config(&io_conf);

    // Reset the display
    gpio_set_level(PIN_NUM_DC, 0);
    gpio_set_level(PIN_NUM_RST, 0);
    gpio_set_level(PIN_NUM_CS, 0);
    gpio_set_level(PIN_NUM_BUSY, 0);
}

spi_device_handle_t spi_EPD;

// Initialize SPI GPIOs
void EPD_SPI_init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PARALLEL_LINES * 320 * 2 + 8};
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // Clock out at 10 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = PIN_NUM_CS,         // CS pin
        .queue_size = 7,                    // We want to be able to queue 7 transactions at a time
        // .pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi_EPD);
    ESP_ERROR_CHECK(ret);
}

void DEV_SPI_WriteByte(UBYTE value)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = 8;             // Command is 8 bits
    t.tx_buffer = &value;     // The data is the cmd itself
    // t.user = (void *)1;                         // D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi_EPD, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

void DEV_SPI_Write_nByte(UBYTE *value, UDOUBLE len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0)
        return;                                 // no need to send anything
    memset(&t, 0, sizeof(t));                   // Zero out the transaction
    t.length = len * 8;                         // Len is in bytes, transaction length is in bits.
    t.tx_buffer = value;                        // Data
    t.user = (void *)1;                         // D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi_EPD, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

int DEV_Module_Init(void)
{
    gpio_set_level(PIN_NUM_DC, 0);
    gpio_set_level(PIN_NUM_CS, 0);
    gpio_set_level(PIN_NUM_RST, 1);
    return 0;
}

void DEV_Module_Exit(void)
{
    gpio_set_level(PIN_NUM_DC, 0);
    gpio_set_level(PIN_NUM_CS, 0);
    gpio_set_level(PIN_NUM_RST, 0);
}
