#pragma once

#ifndef ESPNOW_TRANSPORT_H
#define ESPNOW_TRANSPORT_H

#include <cstdint>
#include <variant>

#include "config.h"
#include "matrix.h"

enum PacketType
{
    PACKET_TYPE_REGISTRATION = 0,
    PACKET_TYPE_ACK,
    PACKET_TYPE_PING,
    PACKET_TYPE_MATRIX,
    PACKET_TYPE_INFO_REQ,
};


typedef struct __attribute__((packed))
{
    PacketType packetType;
    DeviceRole deviceRole;
} Header;


typedef struct __attribute__((packed))
{
} RegistrationPacket;


typedef struct __attribute__((packed))
{
    key_event_t keyEvent;
} MatrixPacket;

typedef struct __attribute__((packed))
{
    float battery;
} InfoPacket;

typedef struct
{
    Header header;
    std::variant<MatrixPacket, InfoPacket, RegistrationPacket> payload = {};
} Packet;

#endif