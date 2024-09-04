#pragma once

#ifndef ESPNOW_TRANSPORT_H
#define ESPNOW_TRANSPORT_H

#include <cstdint>
#include <variant>

#include "config.h"

enum PacketType
{
    PACKET_TYPE_REGISTRATION = 0,
    PACKET_TYPE_ACK,
    PACKET_TYPE_MATRIX,
    PACKET_TYPE_INFO,
};

enum DeviceRole
{
    ROLE_PRIMARY = 0,
    ROLE_LEFT,
    ROLE_RIGHT,
};

typedef struct __attribute__((packed))
{
    PacketType packetType;
    DeviceRole splitSide;
} Header;

typedef struct __attribute__((packed))
{
    uint8_t MATRIX_STATE[MATRIX_ROWS][MATRIX_COLS] = {0};
} MatrixPacket;

typedef struct __attribute__((packed))
{
    float battery;
} InfoPacket;

typedef struct
{
    Header header;
    std::variant<MatrixPacket, InfoPacket> payload = {};
} Packet;

#endif