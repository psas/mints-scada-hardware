#ifndef INTERFACE_MODULE_H
#define INTERFACE_MODULE_H

#define PACKET_ERROR_ERROR 1
#define PACKET_SIZE_ERROR 2
#define PACKET_ID_ERROR 3

#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"
#include "stm32f0xx_hal_adc.h"

extern CAN_HandleTypeDef hcan;
extern ADC_HandleTypeDef hadc;

int main(void);
void getCanMessages(void);

#endif