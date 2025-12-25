#ifndef UPRINTF_H
#define UPRINTF_H

#define MAX_USB_PRINT_LENGTH 96
#include <stdint.h>

void Error_Handler(void);
void uprintf(const char *format, ...);
void uprint(uint8_t* str, int length);

#endif
