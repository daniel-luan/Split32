#pragma once

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "led_strip.h"

#define STATUS_LED_GPIO 48

enum class StatusColor
{
    Red,
    Green,
    Blue,
    Yellow,
    Cyan,
    Magenta,
    White,
    Black
};

class STATUS_LED
{
public:
    static STATUS_LED &get()
    {
        static STATUS_LED instance; // Guaranteed to be destroyed.
                                    // Instantiated on first use.
        return instance;
    }

private:
    STATUS_LED()
    {
        led_strip_config_t strip_config{
            .strip_gpio_num = STATUS_LED_GPIO,        // The GPIO that connected to the STATUS_LED strip's data line
            .max_leds = 1,                            // The number of STATUS_LEDs in the strip,
            .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your STATUS_LED strip
            .led_model = LED_MODEL_WS2812,            // STATUS_LED strip model
            .flags = {.invert_out = false},           // whether to invert the output signal
        };

        led_strip_rmt_config_t rmt_config{
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 10 * 1000 * 1000,
            .mem_block_symbols = 0,
            .flags = {.with_dma = false}};

        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

        /* Set all STATUS_LED off to clear all pixels */
        led_strip_clear(led_strip);
    }

public:
    STATUS_LED(STATUS_LED const &) = delete;
    void operator=(STATUS_LED const &) = delete;

    void off()
    {
        led_strip_clear(led_strip);
    }

    void set(uint32_t r, uint32_t g, uint32_t b)
    {
        led_strip_set_pixel(led_strip, 0, r, g, b);
        led_strip_refresh(led_strip);
    }

    void set(StatusColor color)
    {
        switch (color)
        {
        case StatusColor::Red:
            set(16, 0, 0);
            break;
        case StatusColor::Green:
            set(0, 16, 0);
            break;
        case StatusColor::Blue:
            set(0, 0, 16);
            break;
        case StatusColor::Yellow:
            set(16, 16, 0);
            break;
        case StatusColor::Cyan:
            set(0, 16, 16);
            break;
        case StatusColor::Magenta:
            set(16, 0, 16);
            break;
        case StatusColor::White:
            set(16, 16, 16);
            break;
        case StatusColor::Black:
            led_strip_clear(led_strip);
            break;
        }
    }

private:
    led_strip_handle_t led_strip;
};

#endif