#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#define MAX_STR_LEN   6

// 16 bytes long
typedef struct
{
 char packetType;   // 1 byte
 char command;    // 1 byte
 uint8_t data[MAX_STR_LEN]; // 6 bytes
 uint32_t params[2];   // 4 * 2 bytes
} TPacket;


#endif
