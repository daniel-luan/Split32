#pragma once

#ifndef PRIMARY_H
#define PRIMARY_H

#include <stdio.h>
#include <string.h>
#include <vector>
#include <array>

#include "esp_log.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "espnow_transport.h"

class Primary
{
    static const char *tag;

    struct MacAddress
    {
        uint8_t addr[6];
    };

    std::vector<MacAddress> peer_list;

    bool is_peer_in_list(const uint8_t mac_addr[6])
    {
        for (const auto &mac : peer_list)
        {
            if (memcmp(mac.addr, mac_addr, 6) == 0)
            {
                return true; // MAC address found in the list
            }
        }
        return false; // MAC address not found in the list
    }

    struct QueueItem
    {
        uint8_t mac_addr[6];
        Packet message;
    };

    QueueHandle_t recv_queue;

    static void sendCallback(const uint8_t *mac_addr, esp_now_send_status_t status);

    static void recvCallback(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);

public:
    Primary(Primary const &) = delete;
    void operator=(Primary const &) = delete;

    Primary();

    static Primary &get()
    {
        static Primary instance;
        return instance;
    }

    enum State
    {
        INIT = 0,
        INITIALIZED,
        RUNNING,
    };

    State state;

    void init();

    static void queue_process_task(void *p);

    void registerSecondary(uint8_t mac_addr[6]);
};

#endif