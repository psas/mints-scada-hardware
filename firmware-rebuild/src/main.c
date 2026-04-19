#include "main.h"
#include "board_cfg.h"
#include "can.h"
#include "configuration.h"
#include "init.h"
#include "main.h"
#include "mcp346x.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "uprintf.h"
#include <stdint.h>
#include <stdio.h>

void initPeripherials(void) {
  HAL_Init();
  SystemClock_Config();
  initGPIO();
  initCAN();
  initSPI();
  initUART();
}

extern CAN_FilterTypeDef sFilterConfig;
static uint8_t baseAddress = 0;
// static int count = 0;
int fatal = 0;

void onFatalError(void) {
  fatal = 1;
  // Do things here to set things to a safe state.
  // For outputs, this means all off.
  // For inputs, this means nothing.
}

int calc_baseAddress(void) {
  // Read base ID
  baseAddress |= HAL_GPIO_ReadPin(ADDR_GPIO_PORT, ADDR1_Pin) << 4;
  baseAddress |= HAL_GPIO_ReadPin(ADDR_GPIO_PORT, ADDR2_Pin) << 5;
  baseAddress |= HAL_GPIO_ReadPin(ADDR_GPIO_PORT, ADDR4_Pin) << 6;
  baseAddress |= HAL_GPIO_ReadPin(ADDR_GPIO_PORT, ADDR8_Pin) << 7;
  baseAddress += BASE_ADDR_OFFSET;
  return baseAddress;
}

/* Does everything. Is wrapped by main so that the program will halt if this
 * ever returns. */
void doEverything(void) {
  // Set up a filter. Hopefully it just grabs everything
  CAN_FilterTypeDef sFilterConfig = {
      .FilterFIFOAssignment = CAN_FILTER_FIFO0, // set fifo assignment
      .FilterIdHigh = baseAddress << 5,
      .FilterIdLow = 0,
      .FilterMaskIdHigh = 0xF0 << 5,
      .FilterMaskIdLow = 0,
      .FilterScale = CAN_FILTERSCALE_32BIT, // set filter scale
      .FilterMode = CAN_FILTERMODE_IDMASK,
      .FilterActivation = ENABLE,
  };

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {

    Error_Handler();
    return;
  }

  // Setup ID claim command
  struct DataPacket_t iddp = {
      .id = baseAddress,
      .err = 0, // Not an error packet
      .reserved = 0,
      .reply = 0,
      .data.seq = 0, // Don't care what it is
      .data.cmd = BUSCMD_CLAIM_ID,
      .datasize = 8, // Include the sequence number and command in this count
  };
  compressUID(iddp.data.bytes);
  // Send ID claim command
  uint32_t freeTX = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
  if (writeDatapacketToCan(&iddp) != HAL_OK) {
    Error_Handler();
    return;
  }

  uint8_t subid = (baseAddress & 0xF) - BASE_ADDR_OFFSET;
  // Wait for ID claim command to be sent
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < freeTX) {
    while (1) {
      getCanMessages();
    }
  }
}

int main(void) {
  initPeripherials();
  doEverything();
  while (1); // Halt if main ever exits
}
