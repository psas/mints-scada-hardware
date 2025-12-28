#include "main.h"
#include "board_cfg.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "usb.h"
#include "usbd_cdc_if.h"

static void initGPIO(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  initUSB();
  initGPIO();
  return 0;
}

void initGPIO(void) {
  GPIO_InitTypeDef GPIO_Init = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_Init.Pin = LED_Pin;
  GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_Init);
}
