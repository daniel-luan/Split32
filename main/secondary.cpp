
#include "secondary.h"
#include "esp_log.h"
#include "esp_mac.h"

const char *Secondary::tag = "SecondaryTransport";

Secondary::Secondary()
{
    recv_queue = xQueueCreate(10, sizeof(QueueItem));
    state = INIT;
    ESP_LOGI(tag, "State: INIT");
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
}

void Secondary::sendCallback(const uint8_t *mac_addr, esp_now_send_status_t status)
{

    if (status == ESP_NOW_SEND_SUCCESS)
    {
        // std::cout << "Message sent successfully\n";
    }
    else
    {
        // std::cout << "Failed to send message\n";
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

void Secondary::queue_process_task(void *p)
{

    QueueItem item;
    for (;;)
    {
        if (xQueueReceive(get().recv_queue, &item, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        ESP_LOGI(tag, "Got item %d from " MACSTR, item.message.packetType, MAC2STR(item.mac_addr));

        if (item.message.packetType == PacketType::MSG_TYPE_ACK)
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

void Secondary::registerWithPrimary()
{
    state = REGISTERING;
    ESP_LOGI(tag, "State: REGISTERING");

    while (state == Secondary::State::REGISTERING)
    {
        const uint8_t destination_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        ESP_LOGI(tag, "Sending MSG_TYPE_REGISTRATION");
        Packet message = {MSG_TYPE_REGISTRATION};
        ESP_ERROR_CHECK(esp_now_send(destination_mac, (uint8_t *)&message, sizeof(message))); // NULL for broadcast
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void Secondary::sendDataToPrimary()
{
    assert(state == RUNNING);

    ESP_LOGI(tag, "Sending MSG_TYPE_MATRIX");
    Packet message = {MSG_TYPE_MATRIX};
    ESP_ERROR_CHECK(esp_now_send(primary_address, (uint8_t *)&message, sizeof(message))); // NULL for broadcast
}