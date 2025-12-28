#include "init.h"
#include "stm32f0xx_hal.h"
#include "board_cfg.h"
#include "main.h"
#include "usb.h"

uint8_t baseAddress = 0;

// #TODO: Better error handling
// #TODO Use #ifdef or make scripts to distinguish between configs
int initGen(void){
  SystemClock_Config();
  HAL_Init();
  #ifdef CONFIG_ADC
  if (detectBuild() == ADC_conf) {
    initSPI(); 
  }else{
    return (0);
  }
  #endif
  #ifdef CONFIG_I2C
  if (detectBuild() == I2C_conf &&) {
    initI2C(); 
  }else{
    return (0);
  }
  #endif
  #ifdef CONFIG_ValveCtl
  if (detectBuild() == ValveCtl_conf &&) {
    initValveCtl();  
  }else{
    return (0);
  }
  #endif
  return 0;
}

int initADDR_GPIO(void){
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_Init = {0};
  GPIO_Init.Pin = ADDR4_Pin|ADDR1_Pin|ADDR8_Pin|ADDR2_Pin;
  GPIO_Init.Mode = GPIO_MODE_INPUT;
  GPIO_Init.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_Init);
  return 0;
}
// int initCAN(){
// }
// int initSPI(){
// }
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


Build detectBuild(void){

  // detect upper hex number
  // read (0x0 or 0x1) left shift, OR with baseAddress
  baseAddress |= HAL_GPIO_ReadPin(GPIOA, ADDR4_Pin) << 4;
  baseAddress |= HAL_GPIO_ReadPin(GPIOA, ADDR1_Pin) << 5;
  baseAddress |= HAL_GPIO_ReadPin(GPIOA, ADDR8_Pin) << 6;
  baseAddress |= HAL_GPIO_ReadPin(GPIOA, ADDR2_Pin) << 7;
  baseAddress += BASE_ADDR_OFFSET; // am not 100% on the offset here

  uprintf("My base address is 0x%02x\n", baseAddress);

  if (baseAddress < 0x20 && baseAddress > 0x0F){
     return (ADC_conf);
  } 
  if (baseAddress < 0x30 && baseAddress > 0x1F){
     return (I2C_conf);
  }
  if (baseAddress < 0x50 && baseAddress > 0x3F){
     return (ValveCtl_conf);
  }
  else{
     return (Error_conf);
  }
}

