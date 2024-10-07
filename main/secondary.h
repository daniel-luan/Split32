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
    uint8_t primary_address[6] = {0};

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
        REGISTERING,
        RUNNING,
    };

    State state;

    void init();

    static void espnowProcessRecvTask(void *p);
    TaskHandle_t recvTaskHandle = NULL;

    typedef struct
    {
        QueueHandle_t keyEventQueue;
        Secondary *secondary;
    } keyEventTask_params_t;
    static void keyEventTask(void *pvParameters);
    TaskHandle_t keyEventTaskHandle = NULL;

    int64_t lastRegistrationTime = -1000000;
    void registerWithPrimary();

    void sendKeyEventToPrimary(key_event_t key_event);

    uint32_t lastEventTime = 0;

public:
    Secondary(Secondary const &) = delete;
    void operator=(Secondary const &) = delete;

    Secondary();

    static Secondary &get()
    {
        static Secondary instance;
        return instance;
    }

    void run();
};

#endif