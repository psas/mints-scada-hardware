#include "main.h"
#include "board_cfg.h"
#include "configuration.h"
#include "can.h"
#include "init.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_gpio.h"
#include "usb.h"

void initPeripherials(void) {
  HAL_Init();
  // Set up clocks
  SystemClock_Config();
  initUSB();
  initGPIO();
  initADC();
  initCAN();
#ifdef CONFIG_I2C
  initI2C1();
#endif
#ifdef CONFIG_ADC
  initSPI();
#endif
}

extern CAN_FilterTypeDef sFilterConfig;
uint8_t baseAddress = 0;

// Represents if a fatal error has occurred

#ifdef LOOPBACK
static int count = 0;
#endif

#ifdef CONFIG_OUTPUTS
GPIO_TypeDef *outputPorts[] = {LED_GPIO_Port,  OUT1_GPIO_Port, OUT2_GPIO_Port,
                               OUT3_GPIO_Port, OUT4_GPIO_Port, OUT5_GPIO_Port,
                               OUT6_GPIO_Port, OUT7_GPIO_Port};
uint16_t outputPins[] = {LED_Pin,  OUT1_Pin, OUT2_Pin, OUT3_Pin,
                         OUT4_Pin, OUT5_Pin, OUT6_Pin, OUT7_Pin};
#endif

#ifdef CONFIG_ADC
#include "mcp346x.h"
MCP346x extadc;
#endif

int fatal = 0;

void onFatalError(void) {
  fatal = 1;
  // Do things here to set things to a safe state.
  // For outputs, this means all off.
  // For inputs, this means nothing.
}

uint32_t random(int bits) {
  uint32_t rsp = 0;
  for (int i = 0; i < bits; i++) {
    HAL_ADC_Start(&hadc);
    while (HAL_ADC_PollForConversion(&hadc, 1000)) {
      // do a little waiting
    }
    rsp <<= 1;
    rsp |= (HAL_ADC_GetValue(&hadc) & 1);
  }
  return rsp;
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

  // Initialize ADC
#ifdef CONFIG_ADC
  extadc = MCP346x_Init();
#endif

  // Wait a random amount of time to ensure that if two devices try to start
  // with the same ID, one will have a change to get started and reply to the
  // other alerting them of the issue.
  HAL_Delay(random(6));

  // Read base ID

  uprintf("My base address is 0x%02x\n", baseAddress);

  // Set up the CAN filters
  // Set up a filter. Hopefully it just grabs everything
  CAN_FilterTypeDef sFilterConfig = {
      .FilterFIFOAssignment = CAN_FILTER_FIFO0, // set fifo assignment
      .FilterIdHigh = baseAddress
                      << 5, // the ID that the filter looks for (switch this for
                            // the other microcontroller)
      .FilterIdLow = 0,
      .FilterMaskIdHigh = 0xF0 << 5,
      .FilterMaskIdLow = 0,
      .FilterScale = CAN_FILTERSCALE_32BIT, // set filter scale
      .FilterMode = CAN_FILTERMODE_IDMASK,
      .FilterActivation = ENABLE,
  };

  HAL_StatusTypeDef canSetupStatus = HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
  if (canSetupStatus != HAL_OK) {
    uprintf("CAN filter init Error %d\n", canSetupStatus);
    onFatalError();
    return;
  } else {
    uprintf("CAN filter initalized\n");
  }
  // Start CAN and alert if it failed
  canSetupStatus = HAL_CAN_Start(&hcan);
  if (canSetupStatus != HAL_OK) {
    uprintf("CAN start error %d\n", canSetupStatus);
    onFatalError();
    return;
  } else {
    uprintf("CAN started\n");
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
  int freeTX = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
  int ts = writeDatapacketToCan(&iddp);
  if (ts != HAL_OK) {
    uprintf("\n[FATAL] Could not send CAN address claim packet.");
    onFatalError();
    return;
  }
  uprintf("Sent ID query ");
  printDataPacket(&iddp);
  uprintf("\n");
  // Wait for ID claim command to be sent
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < freeTX);

  while (1) {
    getCanMessages();
#ifdef LOOPBACK
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
    uprintf("0x%x Test TX %4d", random(4), count++);
    int ts = writeDatapacketToCan(&dp);
    uprintf("[%d]: ", ts);
    printDataPacket(&dp);
    uprintf("\n");
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    while (endtime > HAL_GetTick()) {
      // HAL_Delay(1);
    }
#endif
  }
}

int main(void) {
  // Initialize peripherial libraries
  initPeripherials();

  // Wait a moment for USB to connect
  // Might disable for production
  HAL_Delay(2000);
  uprintf("Hello\n");

  doEverything();
  while (1); // Halt if main ever exits
}
