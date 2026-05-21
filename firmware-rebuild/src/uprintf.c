#include "uprintf.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "init.h"
#include "main.h"

uint8_t txbuff[MAX_USB_PRINT_LENGTH] = {0};
/*
 * Like printf, but to print on the USB Serial port.
 * Also converts newlines into a newline & a carage return
 * Max message length is defined by MAX_USB_PRINT_LENGTH
 */
void uprintf(const char *format, ...) {

    // Format the string to the given format
    va_list arg;
    va_start(arg, format);
    vsprintf((char*) txbuff, format, arg);
    va_end(arg);

    // Measure the length of the string and count newlines
    char* c = (char*) txbuff;
    int l = 0;
    int n = 0;
    while(*c != '\0' && l+n < MAX_USB_PRINT_LENGTH-1) {
        if(*c == '\n') {
            n++;
        }
        l++;
        c++;
    }
    int txl = l+n;
    // Add carage returns next to newlines going backwards through the string to not use a second buffer
    uint8_t* w = &txbuff[l+n-1];
    c--;
    while(w >= txbuff) {
        *w-- = (uint8_t) *c;
        if(*c-- == '\n') {
            *w-- = '\r';
            n--;
        }
        // Break out if we've found the last newline since the rest of the message is already in place
        if(n <= 0) {
            break;
        }
    }
    HAL_UART_Transmit(&huart1, txbuff, txl, 1100);
}
