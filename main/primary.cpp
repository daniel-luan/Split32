#include "primary.h"

#include "esp_log.h"
#include "esp_mac.h"

const char *PRIMARY_TAG = "PrimaryTransport";

Primary::Primary()
{
    recv_queue = xQueueCreate(10, sizeof(QueueItem));
    state = INIT;
}

void Primary::init()
{
    assert(state == INIT);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(Primary::sendCallback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(Primary::recvCallback));

    state = INITIALIZED;
}

void Primary::sendCallback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    const auto addr = std::to_array(*reinterpret_cast<const uint8_t(*)[6]>(mac_addr));

    auto it = get().peerMap.find(addr);

    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(PRIMARY_TAG, "Message sent successfully");

        if (it != get().peerMap.end())
        {
            it->second.sendSuccesful = true;
            it->second.lastPacketTime = esp_timer_get_time();
        }
    }
    else
    {
        ESP_LOGI(PRIMARY_TAG, "Failed to send message");

        if (it != get().peerMap.end())
        {
            it->second.sendSuccesful = false;
        }
    }
}

void Primary::recvCallback(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    Packet msg;
    memcpy(&msg, data, data_len);

    QueueItem item;
    memcpy(item.mac_addr, esp_now_info->src_addr, 6);
    memcpy(&item.message, data, data_len);
    xQueueSend(get().recv_queue, &item, portMAX_DELAY);
}

void Primary::espnowProcessRecvTask(void *p)
{
    QueueItem item;
    for (;;)
    {
        if (xQueueReceive(get().recv_queue, &item, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        ESP_LOGI(PRIMARY_TAG, "Got item %d from " MACSTR, item.message.header.packetType, MAC2STR(item.mac_addr));

        const auto mac_addr = std::to_array(item.mac_addr);

        get().updatePeerInfo(mac_addr, item.message.header.splitSide);

        if (item.message.header.packetType == PacketType::PACKET_TYPE_REGISTRATION)
        {
            ESP_LOGI(PRIMARY_TAG, "Sending PACKET_TYPE_ACK to " MACSTR, MAC2STR(item.mac_addr));
            Packet message = {PACKET_TYPE_ACK, ROLE_PRIMARY};
            ESP_ERROR_CHECK(esp_now_send(item.mac_addr, (uint8_t *)&message, sizeof(message))); // NULL for broadcast
        }
        else if (item.message.header.packetType == PacketType::PACKET_TYPE_MATRIX)
        {
            auto event = std::get<MatrixPacket>(item.message.payload);

            auto row = event.keyEvent.row;
            auto col = event.keyEvent.col;
            if (item.message.header.splitSide == ROLE_LEFT)
            {
                row += LEFT_MATRIX_ROW_OFFSET;
                col += LEFT_MATRIX_COL_OFFSET;
            }
            else if (item.message.header.splitSide == ROLE_RIGHT)
            {
                row += RIGHT_MATRIX_ROW_OFFSET;
                col += RIGHT_MATRIX_COL_OFFSET;
            }

            ESP_LOGI(PRIMARY_TAG, "Key event - Row: %d, Col: %d, State: %d", row, col, event.keyEvent.state);

            get().MATRIX_STATE[row][col] = event.keyEvent.state;
        }
    }
}

void Primary::updatePeerInfo(MacAddress addr, DeviceRole role)
{
    auto inserted = peerMap.insert({addr, {}});

    if (inserted.second)
    {
        ESP_LOGI(PRIMARY_TAG, "New Peer Registering " MACSTR "Role: %d", MAC2STR(addr), role);

        esp_now_peer_info_t broadcast_peer = {};
        memcpy(broadcast_peer.peer_addr, addr.data(), 6);
        broadcast_peer.channel = 0;
        broadcast_peer.ifidx = WIFI_IF_STA;
        broadcast_peer.encrypt = false;
        ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));
    }

    if (role != ROLE_UNKNOWN)
    {
        inserted.first->second.role = role;
    }

    inserted.first->second.lastPacketTime = esp_timer_get_time();
    inserted.first->second.sendSuccesful = true;
}

void Primary::checkPeerAlive()
{
    auto currentTime = esp_timer_get_time();

    auto it = peerMap.begin();
    while (it != peerMap.end())
    {
        if (!it->second.sendSuccesful)
        {
            ESP_LOGI(PRIMARY_TAG, "Send failed to Peer " MACSTR, MAC2STR(it->first.data()));
            esp_now_del_peer(it->first.data());

            onPeerDisconnect(it->second.role);
            it = peerMap.erase(it);
        }
        else if (currentTime - it->second.lastPacketTime > PING_INTERVAL)
        {
            ESP_LOGI(PRIMARY_TAG, "Sending PACKET_TYPE_PING to " MACSTR " to check alive", MAC2STR(it->first.data()));
            it->second.lastPacketTime = currentTime;
            Packet message = {PACKET_TYPE_PING, ROLE_PRIMARY};
            ESP_ERROR_CHECK(esp_now_send(it->first.data(), (uint8_t *)&message, sizeof(message)));
            ++it;
        }
        else
        {
            ++it;
        }
    }
}

void Primary::run()
{

    State lastState = UNKNOWN;

    while (true)
    {
        State currentState = state;
        // On state transition
        if (lastState != currentState)
        {
            ESP_LOGI(PRIMARY_TAG, "Entering State %d", currentState);

            if (state == INIT)
            {
                init();
            }
            else if (state == INITIALIZED)
            {
                state = RUNNING;
            }
            else if (state == RUNNING)
            {
                xTaskCreate(Primary::espnowProcessRecvTask, "recv_task", 8192, NULL, 4, NULL);
            }
            lastState = currentState;
        }

        if (state == INIT)
        {
        }
        else if (state == INITIALIZED)
        {
        }
        else if (state == RUNNING)
        {
            checkPeerAlive();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void Primary::onPeerDisconnect(DeviceRole role)
{
    ESP_LOGI(PRIMARY_TAG, "Clearing peer matrix for side %d", role);

    // Clear peer matrix on the primary
    int row_offset = 0;
    int col_offset = 0;
    if (role == ROLE_LEFT)
    {
        row_offset += LEFT_MATRIX_ROW_OFFSET;
        col_offset += LEFT_MATRIX_COL_OFFSET;
    }
    else if (role == ROLE_RIGHT)
    {
        row_offset += RIGHT_MATRIX_ROW_OFFSET;
        col_offset += RIGHT_MATRIX_COL_OFFSET;
    }

    for (int row = row_offset; row < SECONDARY_MATRIX_ROWS + row_offset; row++)
    {
        for (int col = col_offset; col < SECONDARY_MATRIX_COLS + col_offset; col++)
        {
            MATRIX_STATE[row][col] = 0;
        }
    }
}
