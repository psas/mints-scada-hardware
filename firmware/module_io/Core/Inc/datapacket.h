#ifndef DATAPACKET_H
#define DATAPACKET_H

#include <stdint.h>

/*
ID bits
[10]  reply 0=to id, 1=from id
[9]   error
[8]   reserved
[7:0] Device ID

self.reply is the reply bit
self.err   is the error bit
self.rsvd  in the reserved bits from the ID
self.id    is the address of the message
self.num   is the sequence number
self.cmd   is the command
self.data  is the array of 6 bytes of data 
*/

typedef struct BigLittleData_t {
    uint32_t big;
    uint16_t little;
} BigLittleData;

typedef struct CanData_t {
    // The sequence number of the packets
    uint8_t seq;
    // The command the packet is about
    uint8_t cmd;
    // Arguments or data for the packet
        uint8_t args[6];
    // union {
    //     BigLittleData bld;
    // };
} CanData;

typedef struct DataPacket_t {
    // The ID of the device the packet is related to
    uint8_t id;
    // If the packet is an error or not
    uint8_t err;
    // If the packet is a reply or not
    uint8_t reply;
    // Reserved data bit. Doesn't matter what it is.
    uint8_t reserved;
    // The size of the packet. This includes the SEQ and CMD bytes. Max 8.
    uint16_t datasize;
    // Note: data must be aligned with the middle of a word, otherwise BigLittleData access will fail.
    
    // The data of the packet.
    CanData data;

} DataPacket;

#define DATAPACKET_RESVD_BIT 8
#define DATAPACKET_ERROR_BIT 9
#define DATAPACKET_REPLY_BIT 10

/* Prints a datapacket to USB serial */
void printDataPacket(DataPacket* pkt);

#define DATAPACKET_READ_SUCCESS 0
#define DATAPACKET_READ_NOTHING 1
#define DATAPACKET_READ_ERROR 2
#define DATAPACKET_READ_TOOSMALL 3
/* Get a DataPacket from CAN if one is ready */
int readDataPacketFromCan(DataPacket* dest);

#define DATAPACKET_WRITE_SUCCESS 0
#define DATAPACKET_WRITE_ERROR 2
/* Send a DataPacket over CAN */
int writeDatapacketToCan(DataPacket* pkt);


#endif