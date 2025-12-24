#include "usb.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "stm32f0xx_hal.h"

void Error_Handler(void) {
  uprintf("An error has occurred! The program will halt now.\n");
  HAL_Delay(15);
  __disable_irq();
  while (1) {
  }
}
