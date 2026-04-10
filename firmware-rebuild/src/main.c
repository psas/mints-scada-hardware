#include "main.h"
#include <stdio.h>
#include "board_cfg.h"
#include "can.h"
#include "configuration.h"
#include "init.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"

void initPeripherials(void) {
  HAL_Init();
  SystemClock_Config();
  initGPIO();
  initCAN();
}

extern CAN_FilterTypeDef sFilterConfig;
static uint8_t baseAddress = 0;
static int count = 0;
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
      .FilterIdHigh = baseAddress << 5,   // the ID that the filter looks for (switch this for the other microcontroller)
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
      .reply = 0,                  // Not a reply
      .data.seq = 0,               // Don't care what it is
      .data.cmd = BUSCMD_CLAIM_ID, // Sets the ID to claim
      .datasize = 8, // Include the sequence number and command in this count
  };
  compressUID(iddp.data.bytes);
  // Send ID claim command
  uint32_t freeTX = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
  if (writeDatapacketToCan(&iddp) != HAL_OK) {
    Error_Handler();
    return;
  }
  // Wait for ID claim command to be sent
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < freeTX){
    while (1) {
      getCanMessages();
      DataPacket dp;
      // dp.id = 0x75;
      // dp.id = count & 0xFF;
      dp.id = baseAddress | 0x5;
      dp.err = 0;                        // Not an error packet
      dp.reserved = 0;                   // Set to 0 for easier debugging
      dp.reply = 0;                      // Not a reply
      dp.data.seq = count & 0xFF;        // Increment sequence number each time
      dp.data.cmd = BUSCMD_READ_ID_HIGH; // Sets the ID to claim
      dp.data.bytes[0] = 0x00;
      dp.data.bytes[1] = 0x01;
      dp.data.bytes[2] = 0x02;
      dp.data.bytes[3] = 0x03;
      dp.datasize = 6; // Include the sequence number and command in this count

      uint32_t endtime = HAL_GetTick() + 500;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      writeDatapacketToCan(&dp);
      HAL_Delay(50);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      while (endtime > HAL_GetTick()) {
        HAL_Delay(1);
      }
    }
  }
}

int main(void) {
  // Initialize peripheral libraries
  initPeripherials();
  doEverything();
  while (1); // Halt if main ever exits
}
