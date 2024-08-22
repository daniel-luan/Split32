#pragma once

#ifndef ESPNOW_TRANSPORT_H
#define ESPNOW_TRANSPORT_H

#include <cstdint>

enum PacketType
{
    MSG_TYPE_REGISTRATION = 0,
    MSG_TYPE_ACK,
    MSG_TYPE_MATRIX,
};

typedef struct __attribute__((packed))
{
        
} MatrixPacket;

typedef union
{
    MatrixPacket another;
} PayloadUnion;

typedef struct __attribute__((packed))
{
    PacketType packetType;
    PayloadUnion payload;
} Packet;

#endif