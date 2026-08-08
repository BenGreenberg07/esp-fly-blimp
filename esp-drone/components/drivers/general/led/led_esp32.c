/**
*
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * led.c - LED handing functions
 */
#include <stdbool.h>

/*FreeRtos includes*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"
#include "led.h"
#include "stm32_legacy.h"

#define BENCH_TAG "LED_BENCH"

static unsigned int led_pin[] = {
    [LED_BLUE] = LED_GPIO_BLUE,
    [LED_RED]   = LED_GPIO_RED,
    [LED_GREEN] = LED_GPIO_GREEN,
};
static int led_polarity[] = {
    [LED_BLUE] = LED_POL_BLUE,
    [LED_RED]   = LED_POL_RED,
    [LED_GREEN] = LED_POL_GREEN,
};

static bool isInit = false;

//Initialize the green led pin as output
void ledInit()
{
    int i;

    if (isInit) {
        return;
    }

    for (i = 0; i < LED_NUM; i++) {
        gpio_config_t io_conf = {
            //bit mask of the pins that you want to set,e.g.GPIO18/19
            .pin_bit_mask = (1ULL << led_pin[i]),
            //disable pull-down mode
            .pull_down_en = 0,
            //disable pull-up mode
            .pull_up_en = 0,
            //set as output mode
            .mode = GPIO_MODE_OUTPUT,
        };
        //configure GPIO with the given settings
        gpio_config(&io_conf);
        ledSet(i, 0);
    }

    isInit = true;
}

bool ledTest(void)
{
    ledSet(LED_GREEN, 1);
    ledSet(LED_RED, 0);
    vTaskDelay(M2T(250));
    ledSet(LED_GREEN, 0);
    ledSet(LED_RED, 1);
    vTaskDelay(M2T(250));
    // LED test end
    ledClearAll();
    ledSet(LED_BLUE, 1);

    return isInit;
}

void ledClearAll(void)
{
    int i;

    for (i = 0; i < LED_NUM; i++) {
        //Turn off the LED:s
        ledSet(i, 0);
    }
}

void ledSetAll(void)
{
    int i;

    for (i = 0; i < LED_NUM; i++) {
        //Turn on the LED:s
        ledSet(i, 1);
    }
}
void ledSet(led_t led, bool value)
{
    if (led > LED_NUM || led == LED_NUM) {
        return;
    }

    if (led_polarity[led] == LED_POL_NEG) {
        value = !value;
    }

    if (value) {
        gpio_set_level(led_pin[led], 1);
    } else {
        gpio_set_level(led_pin[led], 0);
    }
}

// Standalone bench-test: bypasses all flight code, reads single-char commands
// off the USB serial console, and drives one LED combo at a time so you can
// watch the physical board and see what actually lights up. Never returns.
void ledBenchTest(void)
{
    ledInit();
    ledClearAll();

    // Plain stdin/fgetc doesn't actually receive typed input over idf.py
    // monitor unless the UART driver is installed and VFS is told to read
    // through it (a classic ESP-IDF gotcha) -- without this, every fgetc()
    // call below just returns EOF forever and nothing you type ever arrives.
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    printf("\n==== LED BENCH TEST ====\n");
    printf("Commands (type a letter/digit + Enter in the serial monitor):\n");
    printf("  0 = BLUE only  (GPIO %d)\n", led_pin[LED_BLUE]);
    printf("  1 = RED only   (GPIO %d)\n", led_pin[LED_RED]);
    printf("  2 = GREEN only (GPIO %d)\n", led_pin[LED_GREEN]);
    printf("  a = ALL on\n");
    printf("  x = ALL off\n");
    printf("Starting state: ALL OFF.\n");
    ESP_LOGI(BENCH_TAG, "ready, pins BLUE=%d RED=%d GREEN=%d",
             led_pin[LED_BLUE], led_pin[LED_RED], led_pin[LED_GREEN]);

    while (1) {
        int c = fgetc(stdin);
        if (c == EOF || c == '\n' || c == '\r') {
            vTaskDelay(M2T(20));
            continue;
        }
        ledClearAll();
        switch (c) {
        case '0':
            ledSet(LED_BLUE, 1);
            ESP_LOGI(BENCH_TAG, "BLUE only -> GPIO %d HIGH", led_pin[LED_BLUE]);
            break;
        case '1':
            ledSet(LED_RED, 1);
            ESP_LOGI(BENCH_TAG, "RED only -> GPIO %d HIGH", led_pin[LED_RED]);
            break;
        case '2':
            ledSet(LED_GREEN, 1);
            ESP_LOGI(BENCH_TAG, "GREEN only -> GPIO %d HIGH", led_pin[LED_GREEN]);
            break;
        case 'a':
        case 'A':
            ledSetAll();
            ESP_LOGI(BENCH_TAG, "ALL on -> GPIO %d,%d,%d HIGH",
                     led_pin[LED_BLUE], led_pin[LED_RED], led_pin[LED_GREEN]);
            break;
        case 'x':
        case 'X':
            ESP_LOGI(BENCH_TAG, "ALL off");
            break;
        default:
            ESP_LOGW(BENCH_TAG, "unknown command '%c' (use 0/1/2/a/x)", c);
            break;
        }
    }
}

