#include "init.h"
#include "board_cfg.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "usb.h"

uint8_t baseAddress = 0;
// #TODO Use #ifdef or make scripts to distinguish between configs

int initGen(void) {
  SystemClock_Config();
  HAL_Init();
  initADDR_GPIO();
#ifdef CONFIG_ADC
  if (detectBuild() == ADC_conf) {
    initSPI();
  } else {
    return 0;
  }
#endif
#ifdef CONFIG_I2C
  if (detectBuild() == I2C_conf &&) {
    initI2C();
  } else {
    return 0;
  }
#endif
#ifdef CONFIG_ValveCtl
  if (detectBuild() == ValveCtl_conf &&) {
    initValveCtl();
  } else {
    return 0;
  }
#endif
  return 0;
}

int initADDR_GPIO(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_Init = {
      .Pin = ADDR4_Pin | ADDR1_Pin | ADDR8_Pin | ADDR2_Pin,
      .Mode = GPIO_MODE_INPUT,
      .Pull = GPIO_PULLDOWN,
  };
  HAL_GPIO_Init(GPIOA, &GPIO_Init);
  return 0;
}

void initCAN(void) {
  CAN_HandleTypeDef hcan = {
      .Instance = CAN,
      .Init =
          {
              .Mode = CAN_MODE_NORMAL,
              .Prescaler = 8,
              .SyncJumpWidth = CAN_SJW_1TQ,
              .TimeSeg1 = CAN_BS1_2TQ,
              .TimeSeg2 = CAN_BS2_3TQ,
              .TimeTriggeredMode = DISABLE,
              .AutoBusOff = DISABLE,
              .AutoWakeUp = DISABLE,
              .AutoRetransmission = ENABLE,
              .ReceiveFifoLocked = DISABLE,
              .TransmitFifoPriority = DISABLE,
          },
  };
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

    GPIO_InitTypeDef GPIO_Init = {
        .Pin = GPIO_PIN_8 | GPIO_PIN_9,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = GPIO_AF4_CAN,
    };
    HAL_GPIO_Init(GPIOB, &GPIO_Init);
  }
}

void initSPI(void) {
  SPI_HandleTypeDef hspi2 = {
    .Instance = SPI2,
    .Init = {
      .Mode = SPI_MODE_MASTER,
      .Direction = SPI_DIRECTION_2LINES,
      .DataSize = SPI_DATASIZE_8BIT,
      .CLKPolarity = SPI_POLARITY_LOW,
      .CLKPhase = SPI_PHASE_1EDGE,
      .NSS = SPI_NSS_SOFT,
      .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8,
      .FirstBit = SPI_FIRSTBIT_MSB,
      .TIMode = SPI_TIMODE_DISABLE,
      .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
      .CRCPolynomial = 7,
      .CRCLength = SPI_CRC_LENGTH_DATASIZE,
      .NSSPMode = SPI_NSS_PULSE_ENABLE,
    }
  };
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
}

void initSPI_GPIO(SPI_HandleTypeDef *spiHandle) {
  if (spiHandle->Instance == SPI2) {
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_Init = {
        .Pin = SPI_SCK | SPI_MISO | SPI_MOSI,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = GPIO_AF0_SPI2,
    };
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
