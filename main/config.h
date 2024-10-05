#pragma once

#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

// ========== Device Role ==========
enum DeviceRole
{
    ROLE_PRIMARY = 0,
    ROLE_LEFT,
    ROLE_RIGHT,
    ROLE_COUNT
};

// #define DEVICE_ROLE ROLE_PRIMARY
// #define DEVICE_ROLE ROLE_LEFT
#define DEVICE_ROLE ROLE_RIGHT

// ========== Status LED Config ==========
#define STATUS_LED_GPIO 48
// #define STATUS_LED_GPIO 47
// #define STATUS_LED_GPIO 8

// ========== Primary Device Config ==========
#define PRIMARY_MATRIX_ROWS SECONDARY_MATRIX_ROWS
#define PRIMARY_MATRIX_COLS SECONDARY_MATRIX_COLS * 2
#define PING_INTERVAL 3 * 1000 * 1000
#define EXPECTED_PEERS 2

#define PRIMARY_UART_PORT_NUM UART_NUM_2
#define PRIMARY_UART_BAUD_RATE 115200
#define PRIMARY_UART_TX_PIN 4
#define PRIMARY_UART_RX_PIN 3

// ========== Secondary Device Configs ==========

#define DEEP_SLEEP_DELAY_US 3 * 60 * 1000 * 1000
// #define DEEP_SLEEP_DELAY_US 10 * 1000 * 1000

#define DEBOUNCE 4000

const gpio_num_t MATRIX_ROW_PINS[] = {
    GPIO_NUM_1,
    GPIO_NUM_2,
    GPIO_NUM_5,
    GPIO_NUM_6,
    GPIO_NUM_7,
};

// Right
const gpio_num_t MATRIX_COL_PINS[] = {
    GPIO_NUM_8,
    GPIO_NUM_9,
    GPIO_NUM_10,
    GPIO_NUM_11,
    GPIO_NUM_12,
    GPIO_NUM_13,
};

// Left
// const gpio_num_t MATRIX_COL_PINS[] = {
//     GPIO_NUM_13,
//     GPIO_NUM_12,
//     GPIO_NUM_11,
//     GPIO_NUM_10,
//     GPIO_NUM_9,
//     GPIO_NUM_8,
// };


#define SECONDARY_MATRIX_ROWS 5
#define SECONDARY_MATRIX_COLS 6

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
#define DISPLAY_CS_PIN GPIO_NUM_16
#define DISPLAY_MOSI_PIN GPIO_NUM_17
#define DISPLAY_SCLK_PIN GPIO_NUM_18
#define DISPLAY_BUSY_PIN GPIO_NUM_14
#define DISPLAY_RST_PIN GPIO_NUM_21
#define DISPLAY_DC_PIN GPIO_NUM_15
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 296

#endif
