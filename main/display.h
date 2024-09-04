#pragma once

#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdlib.h>
#include <cstring>

#include "hal/spi_types.h"

#include "esp_log.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>

#include "freertos/FreeRTOS.h"

static const char *LOG_TAG = "DISPLAY";

class Display
{
    spi_device_handle_t spi;

    gpio_num_t CS_PIN = GPIO_NUM_10;
    gpio_num_t MOSI_PIN = GPIO_NUM_11;
    gpio_num_t SCLK_PIN = GPIO_NUM_12;
    gpio_num_t BUSY_PIN = GPIO_NUM_3;
    gpio_num_t RST_PIN = GPIO_NUM_13;
    gpio_num_t DC_PIN = GPIO_NUM_9;

    int WIDTH = 128;
    int HEIGHT = 296;

    bool _power_is_on = false;

    void init_spi();
    void init_gpio();

    void wait_while_busy(const char *name);
    void write_command(const uint8_t cmd);

    void write_data(const uint8_t *data, int len);

    void write_data(const uint8_t data)
    {
        write_data(&data, 1);
    }

    void set_partial_ram_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    void init_display();

    void update_part();

    void init_part()
    {
        power_on();
        init_display();
    }

public:
    Display(
        gpio_num_t CS_PIN,
        gpio_num_t MOSI_PIN,
        gpio_num_t SCLK_PIN,
        gpio_num_t BUSY_PIN,
        gpio_num_t RST_PIN,
        gpio_num_t DC_PIN,
        int WIDTH,
        int HEIGHT) : CS_PIN(CS_PIN),
                      MOSI_PIN(MOSI_PIN),
                      SCLK_PIN(SCLK_PIN),
                      BUSY_PIN(BUSY_PIN),
                      RST_PIN(RST_PIN),
                      DC_PIN(DC_PIN),
                      WIDTH(WIDTH),
                      HEIGHT(HEIGHT)
    {
        init_gpio();
        init_spi();
    }

    void power_on();

    void power_off();

    void fill_screen(uint8_t bw = 0xFF, uint8_t red = 0x00);

    void refresh();
    void write_display(const uint8_t bw[4736], const uint8_t red[4736]);
};

#endif