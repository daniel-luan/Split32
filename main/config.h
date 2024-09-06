#pragma once

#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

#define DEVICE_ROLE ROLE_PRIMARY
// #define DEVICE_ROLE ROLE_LEFT
// #define DEVICE_ROLE ROLE_RIGHT

// ========== Common Shared Config ==========
// #define STATUS_LED_GPIO 48
#define STATUS_LED_GPIO 47
// #define STATUS_LED_GPIO 8

enum DeviceRole
{
    ROLE_UNKNOWN = -1,
    ROLE_PRIMARY = 0,
    ROLE_LEFT,
    ROLE_RIGHT,
    ROLE_COUNT
};

// ========== Primary Device Config ==========
#define PRIMARY_MATRIX_ROWS SECONDARY_MATRIX_ROWS
#define PRIMARY_MATRIX_COLS SECONDARY_MATRIX_COLS * 2
#define PING_INTERVAL 3000000
#define EXPECTED_PEERS 2

// ========== Secondary Device Configs ==========

#define SLEEP_US 1 * 60 * 1000 * 1000
// #define SLEEP_US 5 * 1000 * 1000

#define DEBOUNCE 4000
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

#define SECONDARY_MATRIX_ROWS sizeof(MATRIX_ROW_PINS) / sizeof(gpio_num_t)
#define SECONDARY_MATRIX_COLS sizeof(MATRIX_COL_PINS) / sizeof(gpio_num_t)

typedef struct
{
    int rowOffset;
    int colOffset;
} DeviceMatrixOffsets;

extern const DeviceMatrixOffsets deviceOffsets[];

#define DEFINE_DEVICE_OFFSETS                               \
    const DeviceMatrixOffsets deviceOffsets[ROLE_COUNT] = { \
        [ROLE_PRIMARY] = {0, 0},                            \
        [ROLE_LEFT] = {0, 0},                               \
        [ROLE_RIGHT] = {0, SECONDARY_MATRIX_COLS},          \
    };

// ========== Display Config ==========
#define DISPLAY_SUPPORT false
#define DISPLAY_CS_PIN GPIO_NUM_10
#define DISPLAY_MOSI_PIN GPIO_NUM_11
#define DISPLAY_SCLK_PIN GPIO_NUM_12
#define DISPLAY_BUSY_PIN GPIO_NUM_3
#define DISPLAY_RST_PIN GPIO_NUM_13
#define DISPLAY_DC_PIN GPIO_NUM_9
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 296

#endif
