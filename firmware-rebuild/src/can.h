#include <stdint.h>
typedef struct CanData_t {
    // The sequence number of the packets
    uint8_t seq;
    uint8_t cmd;
    // Arguments or data for the packet
    uint8_t bytes[6];
} CanData;

typedef struct DataPacket_t {
    uint8_t id;
    uint8_t err;
    uint8_t reply;
    uint8_t reserved;
    uint16_t datasize;
    // Note: data must be aligned with the middle of a word, otherwise BigLittleData access will fail.
    CanData data;
} DataPacket;

typedef enum {
  DATAPACKET_READ_SUCCESS,
  DATAPACKET_READ_NOTHING,
  DATAPACKET_READ_ERROR,
  DATAPACKET_READ_TOOSMALL
} datapacket_read_status;

typedef enum {
  DATAPACKET_SEND_SUCCESS,
  DATAPACKET_SEND_ERROR,
} datapacket_send_status;

