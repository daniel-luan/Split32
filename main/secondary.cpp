
#include "secondary.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "status_led.h"

const char *Secondary::tag = "SecondaryTransport";

Secondary::Secondary()
{
    recv_queue = xQueueCreate(32, sizeof(QueueItem));
    state = INIT;
}

void Secondary::init()
{
    assert(state == INIT);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(get().sendCallback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(get().recvCallback));

    esp_now_peer_info_t broadcast_peer = {};
    memset(broadcast_peer.peer_addr, 0xFF, 6); // Broadcast address
    broadcast_peer.channel = 0;
    broadcast_peer.ifidx = WIFI_IF_STA;
    broadcast_peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));

    state = INITIALIZED;
}

void Secondary::sendCallback(const uint8_t *mac_addr, esp_now_send_status_t status)
{

    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(tag, "Message sent successfully");
    }
    else
    {
        ESP_LOGI(tag, "Failed to send message");
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

void Secondary::espnow_process_recv_task(void *p)
{

    QueueItem item;
    for (;;)
    {
        if (xQueueReceive(get().recv_queue, &item, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        ESP_LOGI(tag, "Got item %d from " MACSTR, item.message.header.packetType, MAC2STR(item.mac_addr));

        if (item.message.header.packetType == PacketType::PACKET_TYPE_ACK)
        {

            memcpy(get().primary_address, item.mac_addr, 6);

            esp_now_peer_info_t broadcast_peer = {};
            memcpy(broadcast_peer.peer_addr, item.mac_addr, 6);
            broadcast_peer.channel = 0;
            broadcast_peer.ifidx = WIFI_IF_STA;
            broadcast_peer.encrypt = false;
            ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));

            get().state = RUNNING;
            ESP_LOGI(tag, "State: RUNNING");
        }
    }
}

void Secondary::key_event_task(void *pvParameters)
{
    key_event_task_params_t *params = (key_event_task_params_t *)pvParameters;
    key_event_t event;

    while (1)
    {
        // Wait for an event from the queue
        if (xQueueReceive(params->key_event_queue, &event, portMAX_DELAY) == pdPASS)
        {
            // Process the key event (e.g., log it or handle the key press)
            ESP_LOGI(tag, "Key event - Row: %d, Col: %d, State: %d", event.row, event.col, event.state);
            params->secondary->sendKeyEventToPrimary(event);
        }
    }
}

void Secondary::registerWithPrimary()
{
    const uint8_t destination_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    ESP_LOGI(tag, "Sending PACKET_TYPE_REGISTRATION");
    Packet message = {PACKET_TYPE_REGISTRATION, DEVICE_ROLE};
    ESP_ERROR_CHECK(esp_now_send(destination_mac, (uint8_t *)&message, sizeof(message))); // NULL for broadcast
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void Secondary::sendKeyEventToPrimary(key_event_t key_event)
{
    assert(state == RUNNING);

    ESP_LOGI(tag, "Sending PACKET_TYPE_MATRIX");
    MatrixPacket matrixPacket;

    memcpy(&matrixPacket.keyEvent, &key_event, sizeof(key_event_t));
    Packet message = {PACKET_TYPE_MATRIX, DEVICE_ROLE, matrixPacket};
    ESP_ERROR_CHECK(esp_now_send(primary_address, (uint8_t *)&message, sizeof(message)));
}

void Secondary::run()
{

    STATUS_LED::get().set(StatusColor::Green);

    Matrix m;

    State lastState = UNKNOWN;

    while (true)
    {

        State currentState = state;

        // On state transition
        if (lastState != currentState)
        {
            ESP_LOGI(tag, "Entering State %d", currentState);

            if (currentState == INIT)
            {
                init();
            }
            else if (currentState == INITIALIZED)
            {
                state = REGISTERING;
            }
            else if (currentState == REGISTERING)
            {
                xTaskCreate(Secondary::espnow_process_recv_task, "recv_task", 8192, NULL, 4, NULL);
            }
            else if (currentState == RUNNING)
            {
                key_event_task_params_t params = {
                    .key_event_queue = m.key_event_queue,
                    .secondary = this};

                xTaskCreate(Secondary::key_event_task, "key_event_task", 8192, (void *)&params, 4, NULL);
            }
        }

        lastState = currentState;

        // Run code for current state
        if (currentState == INIT)
        {
        }
        else if (currentState == INITIALIZED)
        {
        }
        else if (currentState == REGISTERING)
        {
            registerWithPrimary();
        }
        else if (currentState == RUNNING)
        {
            m.scan_matrix();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}