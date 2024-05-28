#include "loop.h"
#include <stdint.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "unistd.h"
#include <stdarg.h>
#include <stdio.h>

int count = 0;

#define MAX_USB_PRINT_LENGTH 64
uint8_t txbuff[MAX_USB_PRINT_LENGTH] = {0};
/*
 * Like printf, but to print on the USB Serial port.
 * Also converts newlines into a newline & a carage return
 * Max message length is defined by MAX_USB_PRINT_LENGTH
 */
void usbPrintf(const char *format, ...) {

    // Format the string to the given format
    va_list arg;
    va_start(arg, format);
    vsprintf(txbuff, format, arg);
    va_end(arg);

    // Measure the length of the string and count newlines
    char* c = txbuff;
    int l = 0;
    int n = 0;
    while(*c != '\0' && l+n < MAX_USB_PRINT_LENGTH-1) {
        if(*c == '\n') {
            n++;
        }
        l++;
        c++;
    }
    // Add carage returns next to newlines going backwards through the string to not use a second buffer
    uint8_t* w = &txbuff[l+n-1];
    c--;
    while(w >= txbuff) {
        if(*c == '\n') {
            *w-- = '\r';
            n--;
        }
        *w-- = (uint8_t) *c--;
        // Break out if we've found the last newline since the rest of the message is already in place
        if(n <= 0) {
            break;
        }
    }
    // Print the message
    CDC_Transmit_FS(txbuff, l+n);
    // I don't know why this delay is need, but things broke without it, so here it is.
    HAL_Delay(1);
}

char data[32];
int first = 1;
// Called repeatedly while running 
void loop() {
    if(first) {
        first = 0;
        HAL_Delay(2000);
        CDC_Transmit_FS((uint8_t*) "HELLO\n\r", 7);
    }
    HAL_Delay(500);
    //   HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    usbPrintf("Hello World from USB CDC%d\n", count++);
}