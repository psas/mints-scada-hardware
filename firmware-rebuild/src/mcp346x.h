#include "stm32f0xx_hal_spi.h"

typedef struct MCP346x_t {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* CS_port; // CS is active low
    uint16_t CS_pin;
    GPIO_TypeDef* EN_port;  // EN is active high
    uint16_t EN_pin;
    uint8_t reg[18];
    double ref;
} MCP346x;

