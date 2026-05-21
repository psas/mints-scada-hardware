#include "main.h"
#include "board_cfg.h"
#include "can.h"
#include "configuration.h"
#include "init.h"
#include "mcp346x.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "uprintf.h"
#include <stdint.h>
#include <stdio.h>

void testSPI(void) {
  int count = 0;
  DataFrame frame;
  for (int i = 0; i < 4; i++) {
    MCP346x_analogRead(&adc, (uint8_t)i, (uint8_t)i, GAIN_1, frame.data.bytes);
    frame.id = 0x66;
    frame.datasize = 8;
    frame.reply = 0;
    frame.err = 0;
    frame.reserved = 0;
    frame.data.bytes[4] = (uint8_t)i;
    frame.data.bytes[5] = (uint8_t)i;
    uprintf("Channel %d, 0x%02x%02x%02x%02x%02x%02x\r\n", i, frame.data.bytes[0],
            frame.data.bytes[1], frame.data.bytes[2], frame.data.bytes[3],
            frame.data.bytes[4], frame.data.bytes[5]);
    writeDataframeToCan(&frame);
  }
  uprintf("Count=%d\r\n", count);
  count++;
  MCP346x_analogRead(&adc, MUX_REFP, MUX_REFN, GAIN_1, frame.data.bytes);
  frame.id = 0x88;
  frame.datasize = 8;
  frame.reply = 0;
  frame.err = 0;
  frame.reserved = 0;
  frame.data.bytes[4] = 0x88;
  frame.data.bytes[5] = 0x88;
  uprintf("Channel REF, 0x%02x%02x%02x%02x%02x%02x\r\n", frame.data.bytes[0],
          frame.data.bytes[1], frame.data.bytes[2], frame.data.bytes[3],
          frame.data.bytes[4], frame.data.bytes[5]);
  writeDataframeToCan(&frame);
  HAL_Delay(255);
}

void initPeripherials(void) {
  HAL_Init();
  SystemClock_Config();
  initGPIO();
  initCAN();
  initSPI();
  initUART();
#ifdef CONFIG_ADC
  MCP346x_Init(&hspi2);
#endif
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
  uint8_t baseAddress = calc_baseAddress();
  CAN_FilterTypeDef sFilterConfig = {
      .FilterFIFOAssignment = CAN_FILTER_FIFO0, // set fifo assignment
      .FilterIdHigh = baseAddress << 4,
      .FilterIdLow = 0,
      .FilterMaskIdHigh = 0xF0 << 5,
      .FilterMaskIdLow = 0,
      .FilterScale = CAN_FILTERSCALE_32BIT, // set filter scale
      .FilterMode = CAN_FILTERMODE_IDMASK,
      .FilterActivation = ENABLE,
  };

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
    uprintf("Error on HAL init");
  }

  struct DataFrame_t iddp = {
      .id = baseAddress,
      .err = 0,
      .reserved = 0,
      .reply = 0,
      .data.seq = 0,
      .data.cmd = BUSCMD_CLAIM_ID,
      .datasize = 8,
  };
  get_compressUID(iddp.data.bytes);
  uint32_t init_freeTX = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);

  if (writeDataframeToCan(&iddp) != HAL_OK) {
    Error_Handler();
  }
  DataFrame frame = {0};
  uint8_t subid = (baseAddress & 0xF) - BASE_ADDR_OFFSET;
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < init_freeTX) {
    while (1) {
      // getCanMessages();
      // testSPI();
      MCP346x_analogRead(&adc, MUX_REFP, MUX_REFN, GAIN_1, frame.data.bytes);
      frame.id = 0x66;
      frame.datasize = 8;
      frame.reply = 0;
      frame.err = 0;
      frame.reserved = 0;
      frame.data.bytes[4] = 0x66;
      frame.data.bytes[5] = 0x66;
      writeDataframeToCan(&frame);
      HAL_Delay(255);
    }
  }
  uprintf("Error main program exited");
}

int main(void) {
  initPeripherials();
  doEverything();
  while (1); // Halt if main ever exits
}
