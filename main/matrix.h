#pragma once

#ifndef MATRIX_H
#define MATRIX_H

#include <cstring>
#include <vector>

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"

#include "config.h"

typedef struct
{
    uint8_t row;
    uint8_t col;
    uint8_t state;
} key_event_t;

class Matrix
{
    void gpioMatrixInit();

    void rtcPinDeinit(gpio_num_t pin);
    void rtcMatrixDeinit(void);
    void rtcMatrixInit(void);

    uint32_t DEBOUNCE_MATRIX[SECONDARY_MATRIX_ROWS][SECONDARY_MATRIX_COLS] = {0};

    uint8_t MATRIX_STATE[SECONDARY_MATRIX_ROWS][SECONDARY_MATRIX_COLS] = {0};
    uint8_t PREV_MATRIX_STATE[SECONDARY_MATRIX_ROWS][SECONDARY_MATRIX_COLS] = {0};

    void updateMatrixState(uint8_t row, uint8_t col, uint8_t curState, uint64_t currentTime);
    void debounceKey(uint8_t row, uint8_t col, uint64_t currentTime);
    void scanColumn(uint8_t col, uint64_t currentTime);

public:
    Matrix();

    QueueHandle_t keyEventQueue;
    void scanMatrix();
    void sleep();

    uint32_t lastKeyPress;
};

#endif