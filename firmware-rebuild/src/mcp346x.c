#include "mcp346x.h"
#include "stm32f0xx_hal_gpio.h"

struct MCP346x_t mcpADC {
  .hspi = &hspi2;
  .CS_port = CTRL_GPIO_Port;
  .CS_pin = CTRL_Pin;
  .EN_port = 0;
  .EN_pin = 0;
  .ref = 7.354736328125e-05; // 3.4 / (15900 * 3)
}

void enable_pins(void){
  HAL_GPIO_WritePin(mcpADC.CS_port, mcpADC.CS_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(mcpADC.EN_port, mcpADC.EN_pin, GPIO_PIN_SET);
}
void disable_pins(void) {
  HAL_GPIO_WritePin(mcpADC.CS_port, mcpADC.CS_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(mcpADC.EN_port, mcpADC.EN_pin, GPIO_PIN_RESET);
}

