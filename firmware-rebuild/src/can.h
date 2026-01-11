#include "stm32f0xx_hal_can.h"
#include <stdint.h>

// typedef struct CanData_t {
//   uint8_t seq;      // The sequence number of the frames
//   uint8_t cmd;      //
//   uint8_t bytes[7]; // Arguments or data for the frame
// } CanData;

typedef struct DataFrame_t {
  uint32_t ide;        // type of identifier for the message
  uint32_t id;
  uint32_t frame_type; // remote or data or error frame specifier
  uint32_t datasize;   // length of the data
  uint8_t data[7];     // contents of the data
} DataFrame;

typedef enum {
  DATAFRAME_READ_SUCCESS,
  DATAFRAME_READ_NOTHING,
  DATAFRAME_READ_ERROR,
  DATAFRAME_READ_TOOSMALL
} dataframe_read_status;

typedef enum {
  DATAFRAME_SEND_SUCCESS,
  DATAFRAME_SEND_ERROR,
} dataframe_send_status;
