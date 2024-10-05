#include "primary.h"

#include "esp_log.h"
#include "esp_mac.h"

#include "driver/uart.h"
#include "status_led.h"

const char *PRIMARY_TAG = "PrimaryTransport";

Primary::Primary()
{
    recv_queue = xQueueCreate(10, sizeof(QueueItem));
    state = INIT;
}

void Primary::init()
{
    assert(state == INIT);

    // Init esp-now
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(Primary::sendCallback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(Primary::recvCallback));

    // Init uart
    uart_config_t uart_config{
        .baud_rate = PRIMARY_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };

    ESP_ERROR_CHECK(uart_driver_install(PRIMARY_UART_PORT_NUM, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PRIMARY_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PRIMARY_UART_PORT_NUM, PRIMARY_UART_TX_PIN, PRIMARY_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    state = INITIALIZED;
}

void Primary::sendCallback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    const auto addr = std::to_array(*reinterpret_cast<const uint8_t(*)[6]>(mac_addr));

    auto it = get().peerMap.find(addr);

    if (status == ESP_NOW_SEND_SUCCESS)
    {
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
    ESP_LOGI(PRIMARY_TAG, "Listening for packets");
    QueueItem item;

    uint8_t key_event_data[3];
    key_event_data[0] = 0x12;

    for (;;)
    {

        if (xQueueReceive(get().recv_queue, &item, portMAX_DELAY) == pdPASS)
        {

            ESP_LOGI(PRIMARY_TAG, "Got item %d from " MACSTR, item.message.header.packetType, MAC2STR(item.mac_addr));

            const auto mac_addr = std::to_array(item.mac_addr);

            auto peerResult = get().updatePeerInfo(mac_addr, item.message.header.deviceRole);
            if (peerResult == PeerUpdateResult::PEER_LIST_FULL)
            {
                continue;
            }
            else if (peerResult == PeerUpdateResult::PEER_ADDED && item.message.header.packetType != PACKET_TYPE_REGISTRATION)
            {
                ESP_LOGI(PRIMARY_TAG, "Sending PACKET_TYPE_INFO_REQ to " MACSTR, MAC2STR(item.mac_addr));
                Packet message = {PACKET_TYPE_INFO_REQ, ROLE_PRIMARY};
                ESP_ERROR_CHECK(esp_now_send(item.mac_addr, (uint8_t *)&message, sizeof(message))); // NULL for broadcast
            }

            if (item.message.header.packetType == PACKET_TYPE_REGISTRATION)
            {
                ESP_LOGI(PRIMARY_TAG, "Sending PACKET_TYPE_ACK to " MACSTR, MAC2STR(item.mac_addr));
                Packet message = {PACKET_TYPE_ACK, ROLE_PRIMARY};
                ESP_ERROR_CHECK(esp_now_send(item.mac_addr, (uint8_t *)&message, sizeof(message))); // NULL for broadcast
            }
            else if (item.message.header.packetType == PACKET_TYPE_MATRIX)
            {
                auto event = std::get<MatrixPacket>(item.message.payload);

                // Calculate key event offset
                event.keyEvent.row += deviceOffsets[item.message.header.deviceRole].rowOffset;
                event.keyEvent.col += deviceOffsets[item.message.header.deviceRole].colOffset;

                ESP_LOGI(PRIMARY_TAG, "Key event - Row: %d, Col: %d, State: %d", event.keyEvent.row, event.keyEvent.col, event.keyEvent.state);

                // Update primary matrix state
                get().MATRIX_STATE[event.keyEvent.row][event.keyEvent.col] = event.keyEvent.state;

                // Send key event via uart
                get().key_event_count++;
                key_event_data[1] = event.keyEvent.col;
                key_event_data[2] = (event.keyEvent.row << 1) | (event.keyEvent.state & 0x01);
                uart_write_bytes(PRIMARY_UART_PORT_NUM, (const char *)key_event_data, 3);
            }
        }
    }
}

Primary::PeerUpdateResult Primary::updatePeerInfo(MacAddress addr, DeviceRole role)
{

    auto it = peerMap.find(addr);

    if (it == peerMap.end())
    {
        if (peerMap.size() + 1 > EXPECTED_PEERS)
        {
            ESP_LOGE(PRIMARY_TAG, "Max expected number of %d peers reached, ignoring message", EXPECTED_PEERS);
            return PeerUpdateResult::PEER_LIST_FULL;
        }

        ESP_LOGI(PRIMARY_TAG, "New Peer Registering " MACSTR " Role: %d", MAC2STR(addr), role);

        esp_now_peer_info_t broadcast_peer = {};
        memcpy(broadcast_peer.peer_addr, addr.data(), 6);
        broadcast_peer.channel = 0;
        broadcast_peer.ifidx = WIFI_IF_STA;
        broadcast_peer.encrypt = false;
        ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));

        peerMap[addr] = {.role = role, .lastPacketTime = esp_timer_get_time(), .sendSuccesful = true};

        return PeerUpdateResult::PEER_ADDED;
    }
    else
    {
        it->second.lastPacketTime = esp_timer_get_time();
        it->second.sendSuccesful = true;

        return PeerUpdateResult::PEER_EXISTING;
    }
}

void Primary::checkPeersConnected()
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
            ESP_LOGI(PRIMARY_TAG, "Sending PACKET_TYPE_PING to " MACSTR, MAC2STR(it->first.data()));
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

matrix_row_t Primary::packRows(int row)
{
    matrix_row_t packed_row = 0;
    for (size_t col = 0; col < PRIMARY_MATRIX_COLS; ++col)
    {
        if (MATRIX_STATE[row][col] != 0)
        {
            packed_row |= (1 << col); // Set the corresponding bit for each pressed key
        }
    }

    return packed_row;
}

void Primary::packMatrix(uint8_t packed_matrix[])
{
    // start at byte 1 to account for the type header
    size_t index = 1;

    // Pack matrix row data
    for (size_t row = 0; row < PRIMARY_MATRIX_ROWS; ++row)
    {
        matrix_row_t packed_row = packRows(row); // Your existing packRows function

        // Pack each row into the byte array
        size_t row_size = sizeof(matrix_row_t);
        for (size_t i = 0; i < row_size; ++i)
        {
            packed_matrix[index++] = (packed_row >> (i * 8)) & 0xFF;
        }
    }
    // Add the end byte
    packed_matrix[index++] = 0xFF; // Assuming 0xFF is the end byte
}

void Primary::uartRevcTask(void *p)
{
    uint8_t data[2] = {0};
    size_t length = 0;
    size_t read = 0;

    size_t packed_full_matrix_size = (PRIMARY_MATRIX_ROWS * sizeof(matrix_row_t)) + 2;
    uint8_t packed_full_matrix[packed_full_matrix_size] = {0};
    packed_full_matrix[0] = 0x11;

    while (true)
    {
        ESP_ERROR_CHECK(uart_get_buffered_data_len(PRIMARY_UART_PORT_NUM, (size_t *)&length));

        // Wait and continue if there is no data to read
        if (length == 0)
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        // Read type byte
        read = uart_read_bytes(PRIMARY_UART_PORT_NUM, data, 1, 10 / portTICK_PERIOD_MS);
        assert(read == 1);

        if (data[0] == 0x01)
        // Full matrix request
        {
            get().matrix_request_count++;
            get().packMatrix(packed_full_matrix);
            uart_write_bytes(PRIMARY_UART_PORT_NUM, (const char *)packed_full_matrix, (PRIMARY_MATRIX_ROWS * sizeof(matrix_row_t)) + 2);
        }
        else if (data[0] == 0x02)
        {
            // Led update
            read = uart_read_bytes(PRIMARY_UART_PORT_NUM, data, 1, 10 / portTICK_PERIOD_MS);
            get().led_update_count++;
            ESP_LOGI(PRIMARY_TAG, "LED Update: %d", data[0]);
        }
        else if (data[0] == 0x03)
        {
            // Layer update
            read = uart_read_bytes(PRIMARY_UART_PORT_NUM, data, 1, 10 / portTICK_PERIOD_MS);
            get().layer_update_count++;
            ESP_LOGI(PRIMARY_TAG, "Layer Update: %d", data[0]);
        }
        else
        {
            ESP_LOGW(PRIMARY_TAG, "Unknown command: %d", data[0]);
        }
    }
}

void Primary::run()
{

    State lastState = UNKNOWN;

    int64_t lastLogTime = 0;

    while (true)
    {
        State currentState = state;
        // On state transition
        if (lastState != currentState)
        {
            ESP_LOGI(PRIMARY_TAG, "Entering State %d from %d", currentState, lastState);

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
                STATUS_LED::get().set(StatusColor::Green);

                if (recvTaskHandle == NULL)
                {
                    xTaskCreate(Primary::espnowProcessRecvTask, "recv_task", 8192, NULL, 4, &recvTaskHandle);
                }

                if (uartRevcTaskHandle == NULL)
                {
                    xTaskCreatePinnedToCore(Primary::uartRevcTask, "uart_task", 8192, NULL, 4, &uartRevcTaskHandle, 1);
                }
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
            checkPeersConnected();

            if (esp_timer_get_time() - lastLogTime > 1000000)
            {
                ESP_LOGI(PRIMARY_TAG, "Matrix: %llu LED: %llu Layer: %llu Key Events Sent: %llu", matrix_request_count, led_update_count, layer_update_count, key_event_count);
                lastLogTime = esp_timer_get_time();
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void Primary::onPeerDisconnect(DeviceRole role)
{
    ESP_LOGI(PRIMARY_TAG, "Clearing peer matrix for side %d", role);

    // Clear peer matrix on the primary
    int rowOffset = deviceOffsets[role].rowOffset;
    int colOffset = deviceOffsets[role].colOffset;

    for (int row = rowOffset; row < SECONDARY_MATRIX_ROWS + rowOffset; row++)
    {
        for (int col = colOffset; col < SECONDARY_MATRIX_COLS + colOffset; col++)
        {
            MATRIX_STATE[row][col] = 0;
        }
    }
}
