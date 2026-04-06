#include "init.h"
#include "board_cfg.h"
#include "main.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_can.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_spi.h"

GPIO_InitTypeDef GPIO_InitStruct = {0};

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
SPI_HandleTypeDef hspi2 = {.Instance = SPI2,
                           .Init = {
                               .Mode = SPI_MODE_MASTER,
                               .Direction = SPI_DIRECTION_2LINES,
                               .DataSize = SPI_DATASIZE_16BIT,
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
                           }};

I2C_HandleTypeDef hi2c1 = {
    .Instance = I2C1,
    .Init =
        {
            .Timing = 0x2000090E,
            .OwnAddress1 = 0,
            .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
            .DualAddressMode = I2C_DUALADDRESS_DISABLE,
            .OwnAddress2 = 0,
            .OwnAddress2Masks = I2C_OA2_NOMASK,
            .GeneralCallMode = I2C_GENERALCALL_DISABLE,
            .NoStretchMode = I2C_NOSTRETCH_DISABLE,
        },
};

ADC_HandleTypeDef hadc = {
    .Instance = ADC1,
    .Init =
        {
            .ClockPrescaler = ADC_CLOCK_ASYNC_DIV1,
            .Resolution = ADC_RESOLUTION_12B,
            .DataAlign = ADC_DATAALIGN_RIGHT,
            .ScanConvMode = ADC_SCAN_DIRECTION_FORWARD,
            .EOCSelection = ADC_EOC_SINGLE_CONV,
            .LowPowerAutoWait = DISABLE,
            .LowPowerAutoPowerOff = DISABLE,
            .ContinuousConvMode = DISABLE,
            .DiscontinuousConvMode = DISABLE,
            .ExternalTrigConv = ADC_SOFTWARE_START,
            .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
            .DMAContinuousRequests = DISABLE,
            .Overrun = ADC_OVR_DATA_PRESERVED,
        },
};

ADC_ChannelConfTypeDef sConfig = {
    .Channel = ADC_CHANNEL_2,
    .Rank = ADC_RANK_CHANNEL_NUMBER,
    .SamplingTime = ADC_SAMPLETIME_1CYCLE_5,
    .Channel = ADC_CHANNEL_TEMPSENSOR,
};
void initGPIO(void) {

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB,
                    OUT0_Pin | OUT1_Pin | OUT2_Pin | OUT3_Pin | OUT4_Pin |
                        OUT5_Pin | OUT6_Pin | OUT7_Pin,
                    GPIO_PIN_RESET);

  HAL_GPIO_WritePin(CTRL_GPIO_Port, CTRL_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
                           .Pin = CONFIG0_Pin | CONFIG1_Pin,
                           .Mode = GPIO_MODE_ANALOG,
                           .Pull = GPIO_NOPULL,
                       });

  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
                           .Pin = ADDR4_Pin | ADDR1_Pin | ADDR8_Pin | ADDR2_Pin,
                           .Mode = GPIO_MODE_INPUT,
                           .Pull = GPIO_PULLDOWN,
                       });

  HAL_GPIO_Init(GPIOB,
                &(GPIO_InitTypeDef){
                    .Pin = OUT0_Pin | OUT1_Pin | OUT2_Pin | CTRL_Pin |
                           OUT3_Pin | OUT4_Pin | OUT5_Pin | OUT6_Pin | OUT7_Pin,
                    .Mode = GPIO_MODE_OUTPUT_PP,
                    .Pull = GPIO_NOPULL,
                    .Speed = GPIO_SPEED_FREQ_LOW,
                });

  HAL_GPIO_Init(LED_GPIO_Port, &(GPIO_InitTypeDef){
                                   .Pin = LED_Pin,
                                   .Mode = GPIO_MODE_OUTPUT_PP,
                                   .Pull = GPIO_NOPULL,
                                   .Speed = GPIO_SPEED_FREQ_LOW,
                               });
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle) {

  // PB8 & PB9 have alternate functions as CAN_RX and CAN_TX
  if (canHandle->Instance == CAN) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
                             .Pin = GPIO_PIN_8 | GPIO_PIN_9,
                             .Mode = GPIO_MODE_AF_PP,
                             .Pull = GPIO_NOPULL,
                             .Speed = GPIO_SPEED_FREQ_HIGH,
                             .Alternate = GPIO_AF4_CAN,
                         });

    /* CAN interrupt Init */
    HAL_NVIC_SetPriority(CEC_CAN_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle) {
  if (canHandle->Instance == CAN) {
    __HAL_RCC_CAN1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

    HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);
  }
}

void initCAN(void) {
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    __asm("bkpt");
  }
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    __asm("bkpt");
  }

  __asm("bkpt");
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void initSPI_GPIO(SPI_HandleTypeDef *spiHandle) {
  if (spiHandle->Instance == SPI2) {
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_Init(SPI_PORT, &(GPIO_InitTypeDef){
                                .Pin = SPI_SCK | SPI_MISO | SPI_MOSI,
                                .Mode = GPIO_MODE_AF_PP,
                                .Pull = GPIO_NOPULL,
                                .Speed = GPIO_SPEED_FREQ_HIGH,
                                .Alternate = GPIO_AF0_SPI2,
                            });
  }
}

void initSPI(void) {
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    __asm("bkpt");
  }
  initSPI_GPIO(&hspi2);
}

void initI2C1(void) {
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    __asm("bkpt");
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    __asm("bkpt");
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    __asm("bkpt");
  }
}

void initI2C1_HAL(I2C_HandleTypeDef *i2cHandle) {

  if (i2cHandle->Instance == I2C1) {

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
                             .Pin = GPIO_PIN_10 | GPIO_PIN_11,
                             .Mode = GPIO_MODE_AF_OD,
                             .Pull = GPIO_NOPULL,
                             .Speed = GPIO_SPEED_FREQ_HIGH,
                             .Alternate = GPIO_AF1_I2C1,
                         });
    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

void DeInitI2C1_HAL(I2C_HandleTypeDef *i2cHandle) {

  if (i2cHandle->Instance == I2C1) {
    __HAL_RCC_I2C1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
  }
}

void initADC(void) {
  if (HAL_ADC_Init(&hadc) != HAL_OK) {
    __asm("bkpt");
  }

  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
    __asm("bkpt");
  }

  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
    __asm("bkpt");
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle) {

  if (adcHandle->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_Init(ADC_RAND_GPIO_Port, &(GPIO_InitTypeDef){
                                          .Pin = ADC_RAND_Pin,
                                          .Mode = GPIO_MODE_ANALOG,
                                          .Pull = GPIO_NOPULL,
                                      });
  }
}
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle) {

  if (adcHandle->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_DISABLE();
    HAL_GPIO_DeInit(ADC_RAND_GPIO_Port, ADC_RAND_Pin);
  }
}
