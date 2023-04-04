#ifndef SERIALIZE
#define SERIALIZE

#include <stdlib.h>

#define MAGIC_NUMBER 0xFCFD
#define MAX_DATA_SIZE 16 // TPacket is 16 bytes long
#define PACKET_SIZE  sizeof(TComms)

// 20 bytes long
typedef struct
{
 uint16_t magic;     // 2 byte
 uint8_t dataSize;    // 1 byte
 uint8_t buffer[MAX_DATA_SIZE]; // 16 bytes
 uint8_t checksum;    // 1 byte
} TComms;

typedef enum
{
 PACKET_OK = 0,
 PACKET_BAD = 1,
 PACKET_CHECKSUM_BAD = 2,
 PACKET_INCOMPLETE = 3,
 PACKET_COMPLETE = 4
} TResult;

int serialize(char *buffer, void *dataStructure, size_t size);
TResult deserialize(const char *buffer, int len, void *output);

#endif
