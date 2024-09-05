#pragma once

#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

const gpio_num_t MATRIX_ROW_PINS[] = {
    GPIO_NUM_15,
    GPIO_NUM_16,
    GPIO_NUM_17,
    GPIO_NUM_18,
    GPIO_NUM_8};
const gpio_num_t MATRIX_COL_PINS[] = {
    GPIO_NUM_1,
    GPIO_NUM_2,
    GPIO_NUM_4,
    GPIO_NUM_5,
    GPIO_NUM_6,
    GPIO_NUM_7};

#define MATRIX_ROWS sizeof(MATRIX_ROW_PINS) / sizeof(gpio_num_t)
#define MATRIX_COLS sizeof(MATRIX_COL_PINS) / sizeof(gpio_num_t)

#define DEBOUNCE 4000

// #define DEVICE_ROLE ROLE_PRIMARY
// #define DEVICE_ROLE ROLE_LEFT
#define DEVICE_ROLE ROLE_RIGHT

#define DISPLAY_SUPPORT false
#define DISPLAY_CS_PIN GPIO_NUM_10
#define DISPLAY_MOSI_PIN GPIO_NUM_11
#define DISPLAY_SCLK_PIN GPIO_NUM_12
#define DISPLAY_BUSY_PIN GPIO_NUM_3
#define DISPLAY_RST_PIN GPIO_NUM_13
#define DISPLAY_DC_PIN GPIO_NUM_9
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 296

#define STATUS_LED_GPIO 48
// #define STATUS_LED_GPIO 8

#endif
