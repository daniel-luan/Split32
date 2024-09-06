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

class Primary
{
    struct SecondayPeer
    {
        DeviceRole role;
        uint64_t lastPacketTime;
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

    void updatePeerInfo(MacAddress addr, DeviceRole role);

    void checkPeerAlive();

    uint8_t MATRIX_STATE[PRIMARY_MATRIX_ROWS][PRIMARY_MATRIX_COLS] = {0};

    void onPeerDisconnect(DeviceRole role);

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