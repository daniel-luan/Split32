#include "matrix.h"

static const char *GPIO_TAG = "MATRIX";

Matrix::Matrix()
{
    key_event_queue = xQueueCreate(64, sizeof(key_event_t));

    if (key_event_queue == NULL)
    {
        ESP_LOGE(GPIO_TAG, "Failed to create key_event_queue");
    }

    rtc_matrix_deinit();
    gpio_matrix_init();
}

void Matrix::gpio_matrix_init()
{
    gpio_config_t io_conf = {};

    // Initializing cols as ouput
    memset(&io_conf, 0, sizeof(gpio_config_t));
    io_conf.mode = GPIO_MODE_OUTPUT;
    for (uint8_t col = 0; col < SECONDARY_MATRIX_COLS; col++)
    {
        io_conf.pin_bit_mask |= (1ULL << MATRIX_COL_PINS[col]);
    }
    gpio_config(&io_conf);

    // Initializing rows as input with pull-down
    memset(&io_conf, 0, sizeof(gpio_config_t));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    for (uint8_t row = 0; row < SECONDARY_MATRIX_ROWS; row++)
    {
        io_conf.pin_bit_mask |= (1ULL << MATRIX_ROW_PINS[row]);
    }
    gpio_config(&io_conf);
}

void Matrix::rtc_pin_deinit(gpio_num_t pin)
{
    if (rtc_gpio_is_valid_gpio(pin) == 1)
    {
        rtc_gpio_set_level(pin, 0);
        rtc_gpio_set_direction(pin, RTC_GPIO_MODE_DISABLED);
        gpio_reset_pin(pin);
    }
}

void Matrix::rtc_matrix_deinit(void)
{
    for (uint8_t col = 0; col < SECONDARY_MATRIX_COLS; col++)
    {
        rtc_pin_deinit(MATRIX_COL_PINS[col]);
    }

    for (uint8_t row = 0; row < SECONDARY_MATRIX_ROWS; row++)
    {
        rtc_pin_deinit(MATRIX_ROW_PINS[row]);
    }
}

void Matrix::rtc_matrix_init(void)
{

    for (uint8_t col = 0; col < SECONDARY_MATRIX_COLS; col++)
    {

        if (rtc_gpio_is_valid_gpio(MATRIX_COL_PINS[col]) == 1)
        {
            rtc_gpio_init((MATRIX_COL_PINS[col]));
            rtc_gpio_set_direction(MATRIX_COL_PINS[col],
                                   RTC_GPIO_MODE_INPUT_OUTPUT);
            rtc_gpio_set_level(MATRIX_COL_PINS[col], 1);
        }
    }

    uint64_t rtc_mask = 0;
    for (uint8_t row = 0; row < SECONDARY_MATRIX_ROWS; row++)
    {
        if (rtc_gpio_is_valid_gpio(MATRIX_ROW_PINS[row]) == 1)
        {
            rtc_gpio_init((MATRIX_ROW_PINS[row]));
            rtc_gpio_set_direction(MATRIX_ROW_PINS[row],
                                   RTC_GPIO_MODE_INPUT_OUTPUT);
            rtc_gpio_set_drive_capability(MATRIX_ROW_PINS[row],
                                          GPIO_DRIVE_CAP_0);
            rtc_gpio_set_level(MATRIX_ROW_PINS[row], 0);

            rtc_gpio_wakeup_enable(MATRIX_ROW_PINS[row], GPIO_INTR_HIGH_LEVEL);

            rtc_mask |= 1LLU << MATRIX_ROW_PINS[row];
        }
    }

    esp_sleep_enable_ext1_wakeup(rtc_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
}

// Function to update the matrix state after debouncing
void Matrix::update_matrix_state(uint8_t row, uint8_t col, uint8_t currentState)
{
    if (MATRIX_STATE[row][col] != currentState)
    {
        // ESP_EARLY_LOGI(GPIO_TAG, "%d %d - %d", row, col, currentState);
        MATRIX_STATE[row][col] = currentState;

        key_event_t event = {
            .row = row,
            .col = col,
            .state = currentState};

        if (key_event_queue != NULL)
        {
            if (xQueueSend(key_event_queue, &event, portMAX_DELAY) != pdPASS)
            {
                ESP_LOGE(GPIO_TAG, "Failed to send key event to queue");
            }
        }
    }
}

// Function to handle debouncing for a single key
void Matrix::debounce_key(uint8_t row, uint8_t col, uint64_t currentTime)
{
    uint8_t currentState = gpio_get_level(MATRIX_ROW_PINS[row]);

    if (PREV_MATRIX_STATE[row][col] != currentState)
    {
        DEBOUNCE_MATRIX[row][col] = currentTime;
    }

    if ((currentTime - DEBOUNCE_MATRIX[row][col]) > DEBOUNCE)
    {
        update_matrix_state(row, col, currentState);
    }

    PREV_MATRIX_STATE[row][col] = currentState;
}

// Function to scan a single column
void Matrix::scan_column(uint8_t col, uint64_t currentTime)
{
    gpio_set_level(MATRIX_COL_PINS[col], 1);

    for (uint8_t row = 0; row < SECONDARY_MATRIX_ROWS; row++)
    {
        debounce_key(row, col, currentTime);
    }

    gpio_set_level(MATRIX_COL_PINS[col], 0);
}

void Matrix::scan_matrix()
{
    uint64_t currentTime = esp_timer_get_time();

    for (uint8_t col = 0; col < SECONDARY_MATRIX_COLS; col++)
    {
        scan_column(col, currentTime);
    }
}

void Matrix::sleep()
{
    // rtc_matrix_init();
    // esp_sleep_enable_ext1_wakeup_io();
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    // esp_deep_sleep_start();
}