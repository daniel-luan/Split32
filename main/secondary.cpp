
#include "secondary.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "status_led.h"

const char *SECONDARY_TAG = "SecondaryTransport";

Secondary::Secondary()
{
    recv_queue = xQueueCreate(32, sizeof(QueueItem));
    state = INIT;
}

void Secondary::init()
{
    assert(state == INIT);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(Secondary::sendCallback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(Secondary::recvCallback));

    esp_now_peer_info_t broadcast_peer = {};
    memset(broadcast_peer.peer_addr, 0xFF, 6); // Broadcast address
    broadcast_peer.channel = 0;
    broadcast_peer.ifidx = WIFI_IF_STA;
    broadcast_peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));
    state = REGISTERING;
}

void Secondary::sendCallback(const uint8_t *mac_addr, esp_now_send_status_t status)
{

    if (status == ESP_NOW_SEND_SUCCESS)
    {
    }
    else
    {
        ESP_LOGW(SECONDARY_TAG, "Failed to send message");
        get().state = REGISTERING;
    }
}

void Secondary::recvCallback(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    Packet msg;
    memcpy(&msg, data, data_len);

    QueueItem item;
    memcpy(item.mac_addr, esp_now_info->src_addr, 6);
    memcpy(&item.message, data, data_len);
    xQueueSend(get().recv_queue, &item, portMAX_DELAY);
}

void Secondary::espnowProcessRecvTask(void *p)
{

    QueueItem item;
    for (;;)
    {

        if (xQueueReceive(get().recv_queue, &item, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGD(SECONDARY_TAG, "Got item %d from " MACSTR, item.message.header.packetType, MAC2STR(item.mac_addr));

            if (item.message.header.packetType == PACKET_TYPE_ACK)
            {
                // Only expect this message type in registering phase
                if (get().state != REGISTERING)
                {
                    continue;
                }

                ESP_LOGI(SECONDARY_TAG, "New Primary Address " MACSTR, MAC2STR(item.mac_addr));

                if (memcmp(get().primary_address, item.mac_addr, 6) != 0)
                {

                    memcpy(get().primary_address, item.mac_addr, 6);

                    esp_now_peer_info_t broadcast_peer = {};
                    memcpy(broadcast_peer.peer_addr, item.mac_addr, 6);
                    broadcast_peer.channel = 0;
                    broadcast_peer.ifidx = WIFI_IF_STA;
                    broadcast_peer.encrypt = false;
                    ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));
                }

                get().state = RUNNING;
            }
            else if (item.message.header.packetType == PACKET_TYPE_INFO_REQ)
            {
                ESP_LOGD(SECONDARY_TAG, "Sending PACKET_TYPE_REGISTRATION");
                Packet message = {PACKET_TYPE_REGISTRATION, DEVICE_ROLE, RegistrationPacket{}};
                ESP_ERROR_CHECK(esp_now_send(item.mac_addr, (uint8_t *)&message, sizeof(message))); // NULL for broadcast
            }
        }
    }
}

void Secondary::keyEventTask(void *pvParameters)
{
    keyEventTask_params_t *params = (keyEventTask_params_t *)pvParameters;
    key_event_t event;

    while (1)
    {
        // Wait for an event from the queue
        if (xQueueReceive(params->keyEventQueue, &event, portMAX_DELAY) == pdPASS)
        {
            // Process the key event (e.g., log it or handle the key press)
            ESP_LOGD(SECONDARY_TAG, "Key event - Row: %d, Col: %d, State: %d", event.row, event.col, event.state);
            params->secondary->sendKeyEventToPrimary(event);
        }
    }
}

void Secondary::registerWithPrimary()
{
    if (esp_timer_get_time() - lastRegistrationTime < 1000000)
    {
        return;
    }

    lastRegistrationTime = esp_timer_get_time();
    const uint8_t destination_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    ESP_LOGD(SECONDARY_TAG, "Sending PACKET_TYPE_REGISTRATION");

    Packet message = {PACKET_TYPE_REGISTRATION, DEVICE_ROLE, RegistrationPacket{}};
    ESP_ERROR_CHECK(esp_now_send(destination_mac, (uint8_t *)&message, sizeof(message))); // NULL for broadcast
}

void Secondary::sendKeyEventToPrimary(key_event_t key_event)
{
    assert(state == RUNNING);

    ESP_LOGD(SECONDARY_TAG, "Sending PACKET_TYPE_MATRIX");
    MatrixPacket matrixPacket;

    memcpy(&matrixPacket.keyEvent, &key_event, sizeof(key_event_t));
    Packet message = {PACKET_TYPE_MATRIX, DEVICE_ROLE, matrixPacket};
    ESP_ERROR_CHECK(esp_now_send(primary_address, (uint8_t *)&message, sizeof(message)));
}

void Secondary::run()
{

    Matrix m;

    State lastState = UNKNOWN;

    while (true)
    {
        auto currentTime = esp_timer_get_time();
        auto currentState = state;

        // On state transition
        if (lastState != currentState)
        {
            ESP_LOGI(SECONDARY_TAG, "Entering State %d from %d", currentState, lastState);

            if (currentState == INIT)
            {
                init();
            }
            else if (currentState == REGISTERING)
            {
                STATUS_LED::get().set(StatusColor::Blue);
                if (recvTaskHandle == NULL)
                {
                    xTaskCreate(Secondary::espnowProcessRecvTask, "recv_task", 8192, NULL, 4, &recvTaskHandle);
                }
            }
            else if (currentState == RUNNING)
            {

                STATUS_LED::get().set(StatusColor::Green);

                if (keyEventTaskHandle == NULL)
                {
                    keyEventTask_params_t params = {
                        .keyEventQueue = m.keyEventQueue,
                        .secondary = this};

                    xTaskCreate(Secondary::keyEventTask, "keyEventTask", 8192, (void *)&params, 4, &keyEventTaskHandle);
                }
            }

            lastState = currentState;

            lastEventTime = currentTime;
        }

        // Run code for current state
        if (currentState == INIT)
        {
        }
        else if (currentState == REGISTERING)
        {
            registerWithPrimary();
        }
        else if (currentState == RUNNING)
        {
            m.scanMatrix();
        }

        if (currentTime - std::max(lastEventTime, m.lastKeyPress) > DEEP_SLEEP_DELAY_US)
        {
            ESP_LOGE(SECONDARY_TAG, "Sleeping");

            STATUS_LED::get().off();
            m.sleep();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}