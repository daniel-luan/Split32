#include <stdio.h>
#include <stdlib.h>

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "config.h"
#include "status_led.h"
#include "primary.h"
#include "secondary.h"

static const char *TAG = "SPLIT32";

void init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void init_wifi()
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
}

extern "C" void app_main(void)
{

    STATUS_LED::get().set(StatusColor::White);

    ESP_LOGI(TAG, "Init NVS");
    init_nvs();

    STATUS_LED::get().set(StatusColor::Red);

    ESP_LOGI(TAG, "Init Wifi");
    init_wifi();

    STATUS_LED::get().set(StatusColor::Blue);

    ESP_LOGI(TAG, "Starting app_main");

    if constexpr (DEVICE_ROLE == ROLE_PRIMARY)
    {
        Primary &primary = Primary::get();
        primary.init();
        xTaskCreate(primary.get().process_recv_task, "recv_task", 8192, NULL, 4, NULL);
    }
    else
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        Secondary &secondary = Secondary::get();
        secondary.init();
        xTaskCreate(secondary.get().process_recv_task, "recv_task", 8192, NULL, 4, NULL);

        STATUS_LED::get().set(StatusColor::Green);

        secondary.registerWithPrimary();

        STATUS_LED::get().set(StatusColor::Magenta);

        uint8_t matrix[MATRIX_ROWS][MATRIX_COLS] = {0};

        for (int i = 0; i < MATRIX_ROWS; i++)
        {
            for (int j = 0; j < MATRIX_COLS; j++)
            {
                matrix[i][j] = i + j;
            }
        }

        while (1)
        {
            secondary.sendMatrixToPrimary(matrix);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

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
