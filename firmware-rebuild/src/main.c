#include "usb.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "board_cfg.h"
#include "stm32f0xx_hal_gpio.h"

void flashLED(){
  while (1) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(300000);
  }
}

int main(){
  SystemClock_Config();
  HAL_Init();
   __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_Init = {0};
  GPIO_Init.Pin = ADDR4_Pin|ADDR8_Pin;
  GPIO_Init.Mode = GPIO_MODE_INPUT;
  GPIO_Init.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_Init);
  flashLED(); 
  return 0;
}
