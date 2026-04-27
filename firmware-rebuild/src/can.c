#include "can.h"
#include "board_cfg.h"
#include "configuration.h"
#include "init.h"
#include "main.h"
#include "mcp346x.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_gpio.h"
#include "uprintf.h"
#include <stdint.h>
#include <string.h>

int readDataPacketFromCan(DataPacket *dest) {
  int num = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
  if (num > 0) {
    // Place to temporarily store data
    CAN_RxHeaderTypeDef RxHeader;
    HAL_StatusTypeDef s = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader,
                                               (uint8_t *)&(dest->data));
    dest->err = (RxHeader.StdId >> DATAPACKET_ERROR_BIT) & 1;
    dest->reserved = (RxHeader.StdId >> DATAPACKET_RESVD_BIT) & 1;
    dest->reply = (RxHeader.StdId >> DATAPACKET_REPLY_BIT) & 1;
    dest->id = RxHeader.StdId & 0xFF;
    dest->datasize = RxHeader.DLC;

    if (s != HAL_OK) {
      return DATAPACKET_READ_ERROR;
    } else {
      if (dest->datasize < 2) {
        return DATAPACKET_READ_TOOSMALL;
      }
      return DATAPACKET_READ_SUCCESS;
    }
  } else {
    return DATAPACKET_READ_NOTHING;
  }
}

int writeDatapacketToCan(DataPacket *pkt) {
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox = 0xDEADBEEF;

  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = (pkt->reply << DATAPACKET_REPLY_BIT) |
                   (pkt->err << DATAPACKET_ERROR_BIT) |
                   (pkt->reserved << DATAPACKET_RESVD_BIT) | pkt->id;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = pkt->datasize;
  HAL_StatusTypeDef s = HAL_CAN_AddTxMessage(
      &hcan, &TxHeader, (const uint8_t *)&(pkt->data), &TxMailbox);
  if (s != HAL_OK) {
    return DATAPACKET_WRITE_ERROR;
  }
  return DATAPACKET_WRITE_SUCCESS;
}

/**
 * Copies several bytes of UID to somewhere
 * Copies 0 < bytes <= 12 bytes to dest
 * Starts at 0 <= offset < 12 bytes into the UID
 * If bytes of offset would give illegal locations, they are modified to not.
 */

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

/**
 * Writes a compressed version of the UID to a given memory location.
 * Always writes 6 bytes
 */
void compressUID(uint8_t *dest) {
  uint8_t *uid = (uint8_t *)UID_BASE;

  for (int i = 0; i < 6; i++) {
    dest[i] = (uid[i] + uid[i + 6]) & 0xFF;
  }
}

int processPacket(DataPacket *pk) {
  //uint8_t subid = (pk->id & 0xF) - BASE_ADDR_OFFSET;

  if (pk->err) {
    // If soneone else said they have my ID, that's bad and we need to stop.
    if (pk->data.cmd == BUSCMD_CLAIM_ID) {
      uint8_t tempid[6];
      compressUID(tempid);
      if (memcmp(tempid, pk->data.bytes, 6)) {
        onFatalError();
      }
    }
    return 1;
  }
  // If the packet was a reply, then someone else is using my ID.
  // This is not allowed, so send an error reply

  // If the packet is a reply, it was probably just sent by us, so ignore it.
  if (pk->reply) {
    // onFatalError();
    // pk->err = 0; // Don't set the error bit
    // pk->reply = 1;
    // writeDatapacketToCan(pk);
    // return 2;
    return 0;
  }
  if (pk->datasize < 2) {
    return 3;
  }
  switch (pk->data.cmd) {
  case BUSCMD_CLAIM_ID: {
    uint8_t tempid[6];
    compressUID(tempid);
    int f = 0;
    for (int i = 0; i < 6; i++) {
      f &= tempid[i] != pk->data.bytes[i];
    }
    if (f) {
      pk->err = 1;
      pk->reply = 1;
      writeDatapacketToCan(pk);
    } else {
    }
  } break;
  case BUSCMD_READ_ID_LOW: {
    pk->reply = 1;
    pk->datasize = 8;
    copyUID(pk->data.bytes, 6, 0);
  } break;
  case BUSCMD_READ_ID_HIGH: {
    pk->reply = 1;
    pk->datasize = 8;
    copyUID(pk->data.bytes, 6, 6);
    writeDatapacketToCan(pk);
  } break;
#ifdef CONFIG_OUTPUTS
  case BUSCMD_READ_VALUE: {
    uint32_t val = HAL_GPIO_ReadPin(outputPorts[subid], outputPins[subid]);
    pk->datasize = 8;
    // BigLittleData* bld = BIGLITTLEDATA(pk);
    BIGLITTLEDATA(pk)->big = val;
    // bld->big = val;
    pk->reply = 1;
    writeDatapacketToCan(pk);
  } break;
  case BUSCMD_WRITE_VALUE: {
    HAL_GPIO_WritePin(outputPorts[subid], outputPins[subid],
                      BIGLITTLEDATA(pk)->big);
  } break;
#endif
#ifdef CONFIG_I2C
  case BUSCMD_READ_VALUE: {
    uint32_t val = HAL_GPIO_ReadPin(outputPorts[subid], outputPins[subid]);
    pk->datasize = 8;
    // BigLittleData* bld = BIGLITTLEDATA(pk);
    BIGLITTLEDATA(pk)->big = val;
    // bld->big = val;
    pk->reply = 1;
    writeDatapacketToCan(pk);
  } break;
  case BUSCMD_WRITE_VALUE: {
    HAL_GPIO_WritePin(outputPorts[subid], outputPins[subid],
                      BIGLITTLEDATA(pk)->big);
  } break;
#endif
#ifdef CONFIG_ADC
  case BUSCMD_READ_VALUE: {
    int i = 0;
    for(i>=0; i <4;i++) {
      MCP346x_analogRead(&adc, (uint8_t)i, (uint8_t)i, GAIN_1, pk->data.bytes);
      pk->id = 0x55;
      pk->datasize = 8;
      pk->reply = 1;
      pk->err = 0;
      pk->reserved = 0;
      pk->data.bytes[4] = (uint8_t)i; // these are redundant, I think?
      pk->data.bytes[5] = (uint8_t)i; // but I need to be sure the rest of the bytes are 0x00
      writeDatapacketToCan(pk);
    }
  } break;
#endif
  default: {
  }
  }
  return 0;
}

void getCanMessages(void) {
  int baseAddress = calc_baseAddress();
  while (!fatal && HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    struct DataPacket_t pk = {0};
    for (int i = 0; i < 6; i++) {
      pk.data.bytes[i] = 0;
    }
    pk.datasize = 0;
    int rs = readDataPacketFromCan(&pk);
    if (rs != DATAPACKET_READ_SUCCESS) {
      uprintf("failed read");
      return;
    }
    if (pk.id == 0x08) {
      processPacket(&pk);
    }else {
      uprintf("not4me");
    }

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
}
