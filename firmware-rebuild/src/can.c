#include "can.h"
#include "board_cfg.h"
#include "configuration.h"
#include "init.h"
#include "main.h"
#include "mcp346x.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_gpio.h"
#include "usb.h"
#include <stdint.h>
#include <string.h>

void printDataPacket(DataPacket *pkt) {
  uprintf("%c%d %c%02X #%02x !%02x", (pkt->err) ? 'E' : '.', pkt->reserved & 1,
          (pkt->reply) ? '<' : '>', pkt->id, pkt->data.seq, pkt->data.cmd);
  for (int i = 0; i < pkt->datasize - 2; i++) {
    uprintf(" %02X", pkt->data.bytes[i]);
  }
  // print(f"{'E' if self.err == 1 else '.'}{self.rsvd:01b} {'<' if self.reply
  // else '>'}{self.id:02X} #{self.seq:02X} !{self.cmd:02x}: {'
  // '.join([f'{b:02X}' for b in self.data])}")
}

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
      uprintf("RX Error %d", s);
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

  // uprintf("State: %d ", hcan.State);
  HAL_StatusTypeDef s = HAL_CAN_AddTxMessage(
      &hcan, &TxHeader, (const uint8_t *)&(pkt->data), &TxMailbox);
  if (s != HAL_OK) {
    uprintf("TX Error %d", s);
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
  int baseAddress = calc_baseAddress();
  uint8_t subid = (pk->id & 0xF) - BASE_ADDR_OFFSET;
  uprintf("#%d ", subid);

  // If the packet was an error, check if it's one we care about
  if (pk->err) {
    uprintf("That was an error message!");
    // If soneone else said they have my ID, that's bad and we need to stop.
    if (pk->data.cmd == BUSCMD_CLAIM_ID) {
      uint8_t tempid[6];
      compressUID(tempid);
      if (memcmp(tempid, pk->data.bytes, 6)) {
        onFatalError();
        uprintf("\n[FATAL] Someone else already had my ID %02x\n", baseAddress);
      }
    }
    return 1;
  }
  // If the packet was a reply, then someone else is using my ID.
  // This is not allowed, so send an error reply

  // If the packet is a reply, it was probably just sent by us, so ignore it.
  if (pk->reply) {
    // onFatalError();
    // uprintf("[FATAL] Someone else with my ID sent a reply");
    // pk->err = 0; // Don't set the error bit
    // pk->reply = 1;
    // writeDatapacketToCan(pk);
    // return 2;
    uprintf("That was a reply.");
    return 0;
  }
  if (pk->datasize < 2) {
    uprintf("Command too short!");
    return 3;
  }
  uprintf("Exec time! ");
  switch (pk->data.cmd) {
  case BUSCMD_CLAIM_ID: {
    uprintf("Id claim commnd!");
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
      uprintf("It was me");
    }
  } break;
  case BUSCMD_READ_ID_LOW: {
    uprintf("UID low read");
    pk->reply = 1;
    pk->datasize = 8;
    copyUID(pk->data.bytes, 6, 0);
  } break;
  case BUSCMD_READ_ID_HIGH: {
    uprintf("UID low high");
    pk->reply = 1;
    pk->datasize = 8;
    copyUID(pk->data.bytes, 6, 6);
    writeDatapacketToCan(pk);
  } break;
#ifdef CONFIG_OUTPUTS
  case BUSCMD_READ_VALUE: {
    uprintf("Read value command");
    uint32_t val = HAL_GPIO_ReadPin(outputPorts[subid], outputPins[subid]);
    pk->datasize = 8;
    // BigLittleData* bld = BIGLITTLEDATA(pk);
    BIGLITTLEDATA(pk)->big = val;
    // bld->big = val;
    pk->reply = 1;
    uprintf("\nSending reply");
    printDataPacket(pk);
    writeDatapacketToCan(pk);
  } break;
  case BUSCMD_WRITE_VALUE: {
    uprintf("Write value command");
    HAL_GPIO_WritePin(outputPorts[subid], outputPins[subid],
                      BIGLITTLEDATA(pk)->big);
  } break;
#endif
#ifdef CONFIG_ADC
  case BUSCMD_READ_VALUE: {
    uprintf("Read value command. %d %d", subid << 1, (subid << 1) + 1);
    // uint32_t val = MCP346x_analogRead(&extadc, MUX_AVDD, MUX_AGND, GAIN_1);
    uint32_t val =
        MCP346x_analogRead(&adc, subid << 1, (subid << 1) + 1, GAIN_1);
    pk->datasize = 8;
    // BigLittleData* bld = BIGLITTLEDATA(pk);
    BIGLITTLEDATA(pk)->big = val;
    // bld->big = val;
    pk->reply = 1;
    uprintf("\nSending reply ");
    printDataPacket(pk);
    writeDatapacketToCan(pk);
  } break;
#endif
  default: {
    uprintf("Unknown command");
  }
  }
  return 0;
}

void getCanMessages(void) {
  int baseAddress = calc_baseAddress();
  while (!fatal && HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    uprintf("got message! ");
    struct DataPacket_t pk = {0};
    for (int i = 0; i < 6; i++) {
      pk.data.bytes[i] = 0;
    }
    pk.datasize = 0;
    int rs = readDataPacketFromCan(&pk);
    if (rs != DATAPACKET_READ_SUCCESS) {
      uprintf("packet read fail! %d\n", rs);
      return;
    }
    printDataPacket(&pk);

    uint8_t bid = pk.id & 0xf0;
    uprintf(" bid:%2x", bid);

    if ((pk.id & (0xf0 | SUB_ADDR_MASK)) == baseAddress) {
      uprintf(" 4me!");

      processPacket(&pk);
    }

    uprintf("\n");
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
}
