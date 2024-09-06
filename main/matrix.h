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
    void gpio_matrix_init();
    void rtc_pin_deinit(gpio_num_t pin);
    void rtc_matrix_deinit(void);
    void rtc_matrix_init(void);

    uint32_t DEBOUNCE_MATRIX[SECONDARY_MATRIX_ROWS][SECONDARY_MATRIX_COLS] = {0};

    uint8_t MATRIX_STATE[SECONDARY_MATRIX_ROWS][SECONDARY_MATRIX_COLS] = {0};
    uint8_t PREV_MATRIX_STATE[SECONDARY_MATRIX_ROWS][SECONDARY_MATRIX_COLS] = {0};

    void update_matrix_state(uint8_t row, uint8_t col, uint8_t curState);
    void debounce_key(uint8_t row, uint8_t col, uint64_t currentTime);
    void scan_column(uint8_t col, uint64_t currentTime);

public:
    Matrix();

    QueueHandle_t key_event_queue;
    void scan_matrix();
    void sleep();
};

#endif