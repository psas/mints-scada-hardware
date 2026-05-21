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

typedef struct __attribute__((packed)) CanData_t {
    // The sequence number of the packets
    uint8_t seq;
    // The command the packet is about
    uint8_t cmd;
    // Arguments or data for the packet
    union {
        uint8_t bytes[6];
        struct __attribute__((packed)) {
            uint32_t squish;
            uint16_t squishjr;
       };
    };
} CanData;

typedef struct DataFrame_t {
    // The ID of the device the packet is related to
    uint8_t id;
    // If the packet is an error or not
    uint8_t err;
    // If the packet is a reply or not
    uint8_t reply;
    // Reserved data bit. Doesn't matter what it is.
    uint8_t reserved;
    // The data of the packet.
    CanData data;
    // The size of the packet. This includes the SEQ and CMD bytes.
    uint8_t datasize;

} DataFrame;

#define DATAFRAME_RESVD_BIT 8
#define DATAFRAME_ERROR_BIT 9
#define DATAFRAME_REPLY_BIT 10

#define DATAFRAME_READ_SUCCESS 0
#define DATAFRAME_READ_NOTHING 1
#define DATAFRAME_READ_ERROR 2
#define DATAFRAME_READ_TOOSMALL 3
int readDataFrameFromCan(DataFrame *dest);

#define DATAFRAME_WRITE_SUCCESS 0
#define DATAFRAME_WRITE_ERROR 2
int writeDataframeToCan(DataFrame *frame);

void copyUID(uint8_t *dest, uint8_t bytes, uint8_t offset);
void get_compressUID(uint8_t *dest);
int processFrame(DataFrame *frame, uint8_t baseAddress);
void getCanMessages(void);
int calc_baseAddress(void);
uint32_t start_CANBus(void);
