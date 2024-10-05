#pragma once

#ifndef PRIMARY_H
#define PRIMARY_H

#include <stdio.h>
#include <string.h>
#include <vector>
#include <array>
#include <map>

#include "esp_log.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "espnow_transport.h"

#if (PRIMARY_MATRIX_COLS <= 8)
typedef uint8_t matrix_row_t;
#elif (PRIMARY_MATRIX_COLS <= 16)
typedef uint16_t matrix_row_t;
#elif (PRIMARY_MATRIX_COLS <= 32)
typedef uint32_t matrix_row_t;
#else
#error "MATRIX_COLS: invalid value"
#endif

class Primary
{
    struct SecondayPeer
    {
        DeviceRole role;
        int64_t lastPacketTime;
        bool sendSuccesful = true;
    };

    using MacAddress = std::array<uint8_t, 6U>;
    std::map<MacAddress, SecondayPeer> peerMap;

    struct QueueItem
    {
        uint8_t mac_addr[6];
        Packet message;
    };

    QueueHandle_t recv_queue;

    static void sendCallback(const uint8_t *mac_addr, esp_now_send_status_t status);
    static void recvCallback(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);

    enum State
    {
        UNKNOWN = -1,
        INIT = 0,
        INITIALIZED,
        RUNNING,
    };

    State state;

    void init();

    static void espnowProcessRecvTask(void *p);
    TaskHandle_t recvTaskHandle = NULL;

    enum PeerUpdateResult
    {
        PEER_LIST_FULL,
        PEER_ADDED,
        PEER_EXISTING,
    };

    PeerUpdateResult updatePeerInfo(MacAddress addr, DeviceRole role);

    void checkPeersConnected();

    uint8_t MATRIX_STATE[PRIMARY_MATRIX_ROWS][PRIMARY_MATRIX_COLS] = {0};

    void onPeerDisconnect(DeviceRole role);

    matrix_row_t packRows(int row);
    uint8_t packed_matrix[PRIMARY_MATRIX_ROWS * sizeof(matrix_row_t) + 1] = {0};
    void packMatrix();

    uint64_t matrix_request_count = 0;
    uint64_t led_update_count = 0;

    static void uartTask(void *p);
    TaskHandle_t uartTaskHandle = NULL;

public:
    Primary(Primary const &) = delete;
    void operator=(Primary const &) = delete;

    Primary();

    static Primary &get()
    {
        static Primary instance;
        return instance;
    }

    void run();
};

#endif