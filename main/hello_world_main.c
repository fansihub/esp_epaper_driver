/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

#include "DEV_Config.h"
#include "EPD_Test.h"
#include "EPD_2in9.h"

static const char *TAG = "epaper example";

UBYTE *BlackImage;

void app_main(void)
{
    while (1)
    {
        ESP_LOGI(TAG, "epaper example start");
        EPD_test();

        while (1)
        {
            ESP_LOGI(TAG, "epaper example end");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
    }
}
