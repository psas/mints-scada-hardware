#include "Legacy/stm32_hal_legacy.h"
#include "board_cfg.h"
#include "can.h"
#include "configuration.h"
#include "init.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_i2c.h"
#include "stm32f0xx_hal_i2c_ex.h"
#include "uprintf.h"
#include <stdint.h>
#include <stdio.h>

//from hal manual
HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter);
HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter);
HAL_I2CEx_DisableWakeUp();
HAL_I2CEx_DisableFastModePlus();

HAL_I2C_Master_Transmit();
HAL_I2C_Master_Receive();

readCTRLReg(){

}

// 3 bit addressing, 0-7
selectBus(){

}

