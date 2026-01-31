#include "main.h"
#include "board_cfg.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "usb.h"

//TODO: Figure out how to manage the different configs on compile time

static void initGPIO(void);

int main(void) {
  HAL_Init();
  initGPIO();

  while (1) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(300);
    uprintf("Hi I am alive");
    HAL_Delay(100);
  }
  return 0;
}

void initGPIO(void) {
  GPIO_InitTypeDef GPIO_Init = {
      .Pin = LED_Pin,
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_LOW,
  };
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_Init);
}
