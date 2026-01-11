#include "init.h"
#include "board_cfg.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "usb.h"

uint8_t baseAddress = 0;
CAN_HandleTypeDef hcan;

// #TODO: Better error handling
// #TODO Use #ifdef or make scripts to distinguish between configs
int initGen(void) {
  SystemClock_Config();
  HAL_Init();
  initADDR_GPIO();
#ifdef CONFIG_ADC
  if (detectBuild() == ADC_conf) {
    initSPI();
  } else {
    return (0);
  }
#endif
#ifdef CONFIG_I2C
  if (detectBuild() == I2C_conf &&) {
    initI2C();
  } else {
    return (0);
  }
#endif
#ifdef CONFIG_ValveCtl
  if (detectBuild() == ValveCtl_conf &&) {
    initValveCtl();
  } else {
    return (0);
  }
#endif
  return 0;
}

int initADDR_GPIO(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_Init = {0};
  GPIO_Init.Pin = ADDR4_Pin | ADDR1_Pin | ADDR8_Pin | ADDR2_Pin;
  GPIO_Init.Mode = GPIO_MODE_INPUT;
  GPIO_Init.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_Init);
  return 0;
}

void init_CAN(void) {
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }
}

void initCAN_RXTX(CAN_HandleTypeDef *canHandle) {
  // PB8 & PB9 have alternate functions as CAN_RX and CAN_TX
  if (canHandle->Instance == CAN) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_Init = {0};
    GPIO_Init.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Init.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);
  }
}

void initSPI(void) {
  SPI_HandleTypeDef hspi2;
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
}

void initSPI_GPIO(SPI_HandleTypeDef *spiHandle) {
  GPIO_InitTypeDef GPIO_Init = {0};
  if (spiHandle->Instance == SPI2) {
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_Init.Pin = SPI_SCK | SPI_MISO | SPI_MOSI;
    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Init.Alternate = GPIO_AF0_SPI2;
    HAL_GPIO_Init(SPI_PORT, &GPIO_Init);
  }
}

// int initI2C(){
// }
// int initValveCtl(){
//  if (good){
//     return 1;
//   }
//   if (notgood){
//     return 0
//   }
// }

Build detectBuild(void) {

  // detect upper hex number
  // read (0x0 or 0x1) left shift, OR with baseAddress
  baseAddress |= HAL_GPIO_ReadPin(GPIOA, ADDR4_Pin) << 4;
  baseAddress |= HAL_GPIO_ReadPin(GPIOA, ADDR1_Pin) << 5;
  baseAddress |= HAL_GPIO_ReadPin(GPIOA, ADDR8_Pin) << 6;
  baseAddress |= HAL_GPIO_ReadPin(GPIOA, ADDR2_Pin) << 7;
  baseAddress += BASE_ADDR_OFFSET; // am not 100% on the offset here

  uprintf("My base address is 0x%02x\n", baseAddress);

  if (baseAddress < 0x20 && baseAddress > 0x0F) {
    return (ADC_conf);
  }
  if (baseAddress < 0x30 && baseAddress > 0x1F) {
    return (I2C_conf);
  }
  if (baseAddress < 0x50 && baseAddress > 0x3F) {
    return (ValveCtl_conf);
  } else {
    return (Error_conf);
  }
}
