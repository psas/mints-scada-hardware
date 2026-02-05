#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_spi_ex.h"

extern SPI_HandleTypeDef hspi2;
extern CAN_HandleTypeDef hcan;
extern ADC_HandleTypeDef hadc;
extern I2C_HandleTypeDef hi2c1;

void initCAN(void);
void initGPIO(void);
void initI2C1(void);
void initI2C1_HAL(I2C_HandleTypeDef *i2cHandle);
void initSPI(void);
void initSPI_GPIO(SPI_HandleTypeDef *spiHandle);
void initADC(void);
