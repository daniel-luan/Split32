#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "status_led.h"

static const char *TAG = "SPLIT32";

extern "C" void app_main(void)
{
    while (1)
    {
        STATUS_LED::get().set(StatusColor::White);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        STATUS_LED::get().set(StatusColor::Red);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        STATUS_LED::get().set(StatusColor::Green);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        STATUS_LED::get().set(StatusColor::Blue);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        STATUS_LED::get().set(StatusColor::Magenta);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        STATUS_LED::get().set(StatusColor::Black);
    }
}
