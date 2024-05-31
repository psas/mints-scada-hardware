#ifndef LOOP_H
#define LOOP_H

#include "main.h"

void setup(void);
void loop(void);

extern CAN_HandleTypeDef hcan;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;

#endif