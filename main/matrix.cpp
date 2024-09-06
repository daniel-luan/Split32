#include "matrix.h"

static const char *GPIO_TAG = "MATRIX";

Matrix::Matrix() : lastKeyPress(esp_timer_get_time())
{
    keyEventQueue = xQueueCreate(64, sizeof(key_event_t));

    if (keyEventQueue == NULL)
    {
        ESP_LOGE(GPIO_TAG, "Failed to create keyEventQueue");
    }

    rtcMatrixDeinit();
    gpioMatrixInit();
}

void Matrix::gpioMatrixInit()
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

// Function to update the matrix state after debouncing
void Matrix::updateMatrixState(uint8_t row, uint8_t col, uint8_t currentState, uint64_t currentTime)
{
    if (currentState)
    {
        lastKeyPress = currentTime;
    }

    if (MATRIX_STATE[row][col] != currentState)
    {
        MATRIX_STATE[row][col] = currentState;

        key_event_t event = {
            .row = row,
            .col = col,
            .state = currentState};

        if (keyEventQueue != NULL)
        {
            if (xQueueSend(keyEventQueue, &event, portMAX_DELAY) != pdPASS)
            {
                ESP_LOGE(GPIO_TAG, "Failed to send key event to queue");
            }
        }
    }
}

// Function to handle debouncing for a single key
void Matrix::debounceKey(uint8_t row, uint8_t col, uint64_t currentTime)
{
    uint8_t currentState = gpio_get_level(MATRIX_ROW_PINS[row]);

    if (PREV_MATRIX_STATE[row][col] != currentState)
    {
        DEBOUNCE_MATRIX[row][col] = currentTime;
    }

    if ((currentTime - DEBOUNCE_MATRIX[row][col]) > DEBOUNCE)
    {
        updateMatrixState(row, col, currentState, currentTime);
    }

    PREV_MATRIX_STATE[row][col] = currentState;
}

// Function to scan a single column
void Matrix::scanColumn(uint8_t col, uint64_t currentTime)
{
    gpio_set_level(MATRIX_COL_PINS[col], 1);

    for (uint8_t row = 0; row < SECONDARY_MATRIX_ROWS; row++)
    {
        debounceKey(row, col, currentTime);
    }

    gpio_set_level(MATRIX_COL_PINS[col], 0);
}

void Matrix::scanMatrix()
{
    uint64_t currentTime = esp_timer_get_time();

    for (uint8_t col = 0; col < SECONDARY_MATRIX_COLS; col++)
    {
        scanColumn(col, currentTime);
    }
}

void Matrix::rtcPinDeinit(gpio_num_t pin)
{
    if (rtc_gpio_is_valid_gpio(pin) == 1)
    {
        rtc_gpio_set_level(pin, 0);
        rtc_gpio_set_direction(pin, RTC_GPIO_MODE_DISABLED);
        rtc_gpio_hold_dis(pin);
        gpio_reset_pin(pin);
    }
}

void Matrix::rtcMatrixDeinit(void)
{
    for (uint8_t col = 0; col < SECONDARY_MATRIX_COLS; col++)
    {
        rtcPinDeinit(MATRIX_COL_PINS[col]);
    }

    for (uint8_t row = 0; row < SECONDARY_MATRIX_ROWS; row++)
    {
        rtcPinDeinit(MATRIX_ROW_PINS[row]);
    }
}

void Matrix::rtcMatrixInit(void)
{

    for (uint8_t col = 0; col < SECONDARY_MATRIX_COLS; col++)
    {

        if (rtc_gpio_is_valid_gpio(MATRIX_COL_PINS[col]) == 1)
        {
            rtc_gpio_init((MATRIX_COL_PINS[col]));
            rtc_gpio_set_direction(MATRIX_COL_PINS[col],
                                   RTC_GPIO_MODE_OUTPUT_ONLY);
            rtc_gpio_set_level(MATRIX_COL_PINS[col], 1);

            rtc_gpio_hold_en(MATRIX_COL_PINS[col]);
        }
    }

    uint64_t rtc_mask = 0;
    for (uint8_t row = 0; row < SECONDARY_MATRIX_ROWS; row++)
    {
        if (rtc_gpio_is_valid_gpio(MATRIX_ROW_PINS[row]) == 1)
        {
            rtc_gpio_init((MATRIX_ROW_PINS[row]));
            rtc_gpio_set_direction(MATRIX_ROW_PINS[row],
                                   RTC_GPIO_MODE_INPUT_ONLY);
            rtc_gpio_set_drive_capability(MATRIX_ROW_PINS[row],
                                          GPIO_DRIVE_CAP_0);
            rtc_gpio_set_level(MATRIX_ROW_PINS[row], 0);

            rtc_gpio_pullup_dis(MATRIX_ROW_PINS[row]);
            rtc_gpio_pulldown_en(MATRIX_ROW_PINS[row]);

            rtc_gpio_hold_en(MATRIX_ROW_PINS[row]);

            rtc_gpio_wakeup_enable(MATRIX_ROW_PINS[row], GPIO_INTR_HIGH_LEVEL);

            rtc_mask |= 1LLU << MATRIX_ROW_PINS[row];
        }
    }

    esp_sleep_enable_ext1_wakeup(rtc_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
}

void Matrix::sleep()
{
    rtcMatrixInit();
    esp_deep_sleep_start();
}