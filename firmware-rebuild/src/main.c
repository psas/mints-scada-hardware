#include "main.h"
#include "board_cfg.h"
#include "can.h"
#include "configuration.h"
#include "init.h"
#include "mcp346x.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "uprintf.h"
#include <stdint.h>
#include <stdio.h>

void testSPI(void) {
  DataFrame frame;
  for (int i = 0; i < 7; i+=2) {
    MCP346x_analogRead(&adc, (uint8_t)i, (uint8_t)(i+1), GAIN_1, frame.data.bytes);
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
static int fatal = 0;
void onFatalError(void) {
  fatal = 1;
  // Do things here to set things to a safe state.
  // For outputs, this means all off.
  // For inputs, this means nothing.
}
void checkFatal(void) {
  fatal = 1;

}
uint8_t calc_baseAddress(void) {
  // Read base ID
  uint8_t baseAddress = 0;
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
  uprintf("baseAddress = 0x%02x\r\n", baseAddress);
  CAN_FilterTypeDef sFilterConfig = {
      .FilterFIFOAssignment = CAN_FILTER_FIFO0,
      .FilterIdHigh = baseAddress << 5,
      .FilterIdLow = 0x0,
      .FilterMaskIdHigh = (0xF & ~(BASE_ADDR_OFFSET)) << 5, // see Figure 319 in stm32f0x ref manual for bit alignment
      .FilterMaskIdLow = 0x0,
      .FilterScale = CAN_FILTERSCALE_32BIT,
      .FilterMode = CAN_FILTERMODE_IDMASK,
      .FilterActivation = ENABLE,
  };

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
    uprintf("Error on CAN filter init");
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
  uint8_t subid = (frame.id & 0xF) & ~(BASE_ADDR_OFFSET);
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < init_freeTX) {
    while (1) {
      getCanMessages(baseAddress);
    }
  }
  uprintf("Error main program exited");
}

int main(void) {
  initPeripherials();
  doEverything();

  // TODO: set safe state
  // watchdog later

  // drop to assembly startup file halt
}
