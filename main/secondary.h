#pragma once

#ifndef SECONDARY_H
#define SECONDARY_H

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "espnow_transport.h"

class Secondary
{
    uint8_t primary_address[6];

    static const char *tag;

    struct QueueItem
    {
        uint8_t mac_addr[6];
        Packet message;
    };

    QueueHandle_t recv_queue;

    static void sendCallback(const uint8_t *mac_addr, esp_now_send_status_t status);

    static void recvCallback(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);

public:
    Secondary(Secondary const &) = delete;
    void operator=(Secondary const &) = delete;

    Secondary();

    static Secondary &get()
    {
        static Secondary instance;
        return instance;
    }

    enum State
    {
        UNKNOWN = -1,
        INIT = 0,
        INITIALIZED,
        REGISTERING,
        RUNNING,
    };

    State state;

    void init();
    void run();

    static void espnow_process_recv_task(void *p);

    typedef struct
    {
        QueueHandle_t key_event_queue;
        Secondary *secondary;
    } key_event_task_params_t;
    static void key_event_task(void *pvParameters);

    void registerWithPrimary();

    void sendKeyEventToPrimary(key_event_t key_event);
};

#endif