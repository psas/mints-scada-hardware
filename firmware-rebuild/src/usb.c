#include "init.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f0xx_hal.h"
#include "usb.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "usbd_def.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

void Error_Handler(void) {
  uprintf("An error has occurred! The program will halt now.\n");
  HAL_Delay(15);
  __disable_irq();
  while (1) {
  }
}

USBD_HandleTypeDef hUsbDeviceFS;
void initUSB(void) {
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) !=
      USBD_OK) {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
    Error_Handler();
  }
}

uint8_t txbuff[MAX_USB_PRINT_LENGTH] = {0};
// usb serial port printf
// converts newline into newline and carriage return
// max message length is defined by MAX_USB_PRINT_LENGTH
void uprintf(const char *format, ...) {

  // Format the string to the given format
  va_list arg;
  va_start(arg, format);
  vsprintf((char *)txbuff, format, arg);
  va_end(arg);

  // Measure the length of the string and count newlines
  char *c = (char *)txbuff;
  int l = 0;
  int n = 0;
  while (*c != '\0' && l + n < MAX_USB_PRINT_LENGTH - 1) {
    if (*c == '\n') {
      n++;
    }
    l++;
    c++;
  }
  int txl = l + n;
  // Add carage returns next to newlines going backwards through the string to
  // not use a second buffer
  uint8_t *w = &txbuff[l + n - 1];
  c--;
  while (w >= txbuff) {
    *w-- = (uint8_t)*c;
    if (*c-- == '\n') {
      *w-- = '\r';
      n--;
    }
    // Break out if we've found the last newline since the rest of the message
    // is already in place
    if (n <= 0) {
      break;
    }
  }
  // Print the message
  uprint(txbuff, txl + 1);
  // I don't know why this delay is need, but things broke without it, so here
  // it is. Turns out it was to give the USB peripheral a moment to send the
  // message before the program crashed
  HAL_Delay(1);
}

void uprint(uint8_t *str, const int length) {
  uint8_t status = USBD_BUSY;
  while (status != USBD_OK) {
    // Not sure why this is length-1, but it prevents random chars from
    // appearing at the start of the sent message
    status = CDC_Transmit_FS(str, length - 1);
  }
}
