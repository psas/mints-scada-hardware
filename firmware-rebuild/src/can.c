#include "can.h"
#include "board_cfg.h"
#include "configuration.h"
#include "init.h"
#include "main.h"
#include "mcp346x.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_gpio.h"
#include "uprintf.h"
#include <stdint.h>
#include <string.h>

// Copies several bytes of UID to somewhere
// Copies 0 < bytes <= 12 bytes to dest
// Starts at 0 <= offset < 12 bytes into the UID
// If bytes of offset would give illegal locations, they are modified to not.

void copyUID(uint8_t *dest, uint8_t bytes, uint8_t offset) {
  if (offset > 11) {
    offset = 11;
  }
  if (bytes + offset > 12) {
    bytes = 12 - offset;
  }
  if (bytes < 1) {
    bytes = 1;
  }
  memcpy(dest, (uint8_t *)(UID_BASE + offset), bytes);
}

// Reads the UID from the memory base address then,
// Writes a compressed 6 byte version of the UID of the MCU to a given memory
// location Used to distinguish each unique MCU on the bus

void get_compressUID(uint8_t *dest) {
  uint8_t *uid = (uint8_t *)UID_BASE;

  for (int i = 0; i < 6; i++) {
    dest[i] = (uid[i] + uid[i + 6]) & 0xFF;
  }
}

int readDataFrameFromCan(DataFrame *dest) {
  int num = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
  if (num > 0) {
    CAN_RxHeaderTypeDef RxHeader;
    HAL_StatusTypeDef s = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader,
                                               (uint8_t *)&(dest->data));
    dest->err = (RxHeader.StdId >> DATAFRAME_ERROR_BIT) & 1;
    dest->reserved = (RxHeader.StdId >> DATAFRAME_RESVD_BIT) & 1;
    dest->reply = (RxHeader.StdId >> DATAFRAME_REPLY_BIT) & 1;
    dest->id = RxHeader.StdId & 0xFF;
    dest->datasize = RxHeader.DLC;

    if (s != HAL_OK) {
      return DATAFRAME_READ_ERROR;
    } else {
      if (dest->datasize < 2) {
        return DATAFRAME_READ_TOOSMALL;
      }
      return DATAFRAME_READ_SUCCESS;
    }
  } else {
    return DATAFRAME_READ_NOTHING;
  }
}

int writeDataframeToCan(DataFrame *frame) {
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox = 0xDEADBEEF;

  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = (frame->reply << DATAFRAME_REPLY_BIT) |
                   (frame->err << DATAFRAME_ERROR_BIT) |
                   (frame->reserved << DATAFRAME_RESVD_BIT) | frame->id;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = frame->datasize;
  HAL_StatusTypeDef s = HAL_CAN_AddTxMessage(
      &hcan, &TxHeader, (const uint8_t *)&(frame->data), &TxMailbox);
  if (s != HAL_OK) {
    return DATAFRAME_WRITE_ERROR;
  }
  return DATAFRAME_WRITE_SUCCESS;
}

int processFrame(DataFrame *frame, int baseAddress) {
  uint8_t subid = (frame->id & 0xF) & ~(BASE_ADDR_OFFSET);
  uprintf("Base address: %08x\r\n", baseAddress);
  if (frame->err) {
    if (frame->data.cmd == BUSCMD_CLAIM_ID) {
      uint8_t tempid[6];
      get_compressUID(tempid);
      if (memcmp(tempid, frame->data.bytes, 6)) {
        onFatalError();
        uprintf("Another device has the same ID\r\n");
      }
    }
    return 1;
  }
  if (frame->reply) {
    onFatalError();
    frame->err = 0;
    frame->reply = 1;
    writeDataframeToCan(frame);
    return 0;
  }
  switch (frame->data.cmd) {
  case BUSCMD_CLAIM_ID:
    uint8_t tempid[6];
    get_compressUID(tempid);
    memcpy(tempid, frame->data.bytes, 6);
    frame->err = 1;
    frame->reply = 1;
    writeDataframeToCan(frame);
    break;
  case BUSCMD_READ_ID_LOW: {
    frame->reply = 1;
    frame->datasize = 8;
    copyUID(frame->data.bytes, 6, 0);
  } break;
  case BUSCMD_READ_ID_HIGH: {
    frame->reply = 1;
    frame->datasize = 8;
    copyUID(frame->data.bytes, 6, 6);
    writeDataframeToCan(frame);
  } break;
#ifdef CONFIG_OUTPUTS
  case BUSCMD_READ_VALUE: {
    frame->datasize = 8;
    frame->reply = 1;
    memset(HAL_GPIO_ReadPin(outputPorts[subid], outputPins[subid]) == GPIO_PIN_SET), frame->data.bytes, frame->datasize)
    writeDataframeToCan(frame);
  } break;
  case BUSCMD_WRITE_VALUE: {
    uint8_t data[4];
    // Toggle state of GPIO pins based on received data from CAN
    HAL_GPIO_WritePin(outputPorts[subid], outputPins[subid],
                      frame->squish.value ? GPIO_PIN_SET : GPIO_PIN_RESET)
  } break;
#endif
#ifdef CONFIG_ADC
  case BUSCMD_READ_VALUE: {
    for (int i = 0; i < 7; i += 2) {
      MCP346x_analogRead(&adc, (uint8_t)i, (uint8_t)(i + 1), GAIN_1,
                         frame->data.bytes);
      frame->id = 0x55;
      frame->datasize = 8;
      frame->reply = 1;
      frame->err = 0;
      frame->reserved = 0;
      frame->data.bytes[4] = (uint8_t)i;
      frame->data.bytes[5] = (uint8_t)i;
      writeDataframeToCan(frame);
      uprintf("Channel REF, 0x%02x%02x%02x%02x%02x%02x\r\n",
              frame->data.bytes[0], frame->data.bytes[1], frame->data.bytes[2],
              frame->data.bytes[3], frame->data.bytes[4], frame->data.bytes[5]);
    }
  } break;
#endif
  }
  return 0;
}

void getCanMessages(uint8_t baseAddress) {
  while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)) {
    uprintf("hi");
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    struct DataFrame_t frame = {0};
    switch (readDataFrameFromCan(&frame)) {
    case DATAFRAME_READ_ERROR:
      uprintf("failed read\r\n");
      break;
    case DATAFRAME_READ_SUCCESS:
      uprintf("successful read\r\n");
      break;
    case DATAFRAME_READ_NOTHING:
      uprintf("no value read from packet\r\n");
      break;
    case DATAFRAME_READ_TOOSMALL:
      uprintf("error: data payload < 2 bytes\r\n");
      break;
    }

    if ((frame.id & 0xF0) == (baseAddress & 0xF0)) {
      processFrame(&frame, baseAddress);
    } else {
      uprintf("Message was not for me\r\n");
    }
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
}
