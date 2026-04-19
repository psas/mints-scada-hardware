#include "mcp346x.h"
#include "board_cfg.h"
#include "init.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_spi.c"
#include "stm32f0xx_hal_spi.h"
#include "uprintf.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

MCP346x adc = {
   .ref = 7.354736328125e-05, // 3.4 / (15900 * 3)
   .cs_port = CTRL_GPIO_Port,
   .cs_pin = CTRL_Pin,
};

MCP346x MCP346x_Init(SPI_HandleTypeDef *spi) {
   adc.hspi = spi;

  __HAL_SPI_ENABLE(&hspi2);

  uint8_t cmds[] = {
      /* CFG0 */ CONFIG0_VREF_INT | CLK_SEL_INTERNAL | BIAS_OFF | MODE_STANDBY,
      /* CFG1 */ PRESCALER_1 | OSR_32,
      /* CFG2 */ BOOST_1 | GAIN_1 << 3 | MUX_ZERO_OFF | 0b11,
      /* CFG3 */ CONV_MODE_CONTINUOUS | DATA_FORMAT_32 | CRC_FORMAT_16 |
          CRC_ON_READ_DISABLED | DIGITAL_OFFSET_CALIB_DISABLED |
          GAIN_CALIB_DISABLED,
      /* IRQ  */
      IRQ_PIN_MODE_IRQ | IRQ_PIN_INACTIVE_LOGIC | FAST_COMMANDS_ENABLED |
          CONVERSION_START_INTERRUPT_DISABLED,
      /* MUX  */ 0x01,
      /* SCAN */ 0, 0, 0,
      /* TIMR */ 0, 0, 0,
      /* OFST */ 0x00, 0x00, 0x3E,
      /* GAIN */ 0x7D, 0xEE, 0x80};
  HAL_Delay(1);
  MCP346x_writeRegs(&adc, REG_CONFIG0, cmds, 18);

  cmds[3] = 4;

  return adc;
}

void enableSPI(const MCP346x *adc) {
  HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET);
}

void disableSPI(const MCP346x *adc) {
  HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET);
}

uint8_t MCP346x_sendCmd(const MCP346x *adc, const uint8_t fastcmd) {
  // 2 bit device address, 4 bit register address, 2 bit command
  uint8_t cmd = 0b01 << 6 | fastcmd << 2 | CMD_TYPE_FAST;
  uint8_t status;
  enableSPI(adc);

  HAL_SPI_TransmitReceive(&hspi2, &cmd, &status, 1, 100);

  disableSPI(adc);
  return status;
}

uint8_t MCP346x_readRegs(const MCP346x *adc, const uint8_t reg, uint8_t *result,
                         const int count) {
  // 2 bit device address, 4 bit register address, 2 bit command
  enableSPI(adc);

  // Read the desired number of result bytes
  uint8_t tosend[count + 1];
  tosend[0] = PACK_COMMAND(reg, CMD_TYPE_READY_MANY);
  uint8_t reply[count + 1];
  HAL_SPI_TransmitReceive(&hspi2, tosend, reply, count + 1, 100);
  memcpy(result, reply + 1, count);

  disableSPI(adc);
  // return status;
  return reply[0];
}

// Return reply[1]
uint8_t MCP346x_readReg(const MCP346x *adc, const uint8_t reg) {
  uint8_t reply;
  MCP346x_readRegs(adc, reg, &reply, 1);
  return reply;
}

uint8_t MCP346x_writeRegs(MCP346x *adc, const uint8_t reg,
                          const uint8_t *values, const int count) {
  // Pack the values into a single SPI task
  uint8_t tosend[count + 1];
  // 2 bit device address, 4 bit register address, 2 bit command
  tosend[0] = PACK_COMMAND(reg, CMD_TYPE_WRITE_MANY);

  for (int i = 0; i < count; i++) {
    tosend[i + 1] = values[i];
    adc->reg[reg + i] = values[i];
  }
  uint8_t reply[count + 1];

  enableSPI(adc);
  HAL_Delay(1);

  HAL_SPI_TransmitReceive(&hspi2, tosend, reply, count + 1, 100);

  HAL_Delay(1);
  disableSPI(adc);

  return reply[0];
}

uint8_t MCP346x_writeReg(MCP346x *adc, const uint8_t reg, uint8_t value) {
  // uint8_t vals[] = {value};
  uint8_t vals[] = {PACK_COMMAND(reg, CMD_TYPE_WRITE_MANY), value};
  uint8_t reply[2];
  enableSPI(adc);

  HAL_SPI_TransmitReceive(&hspi2, vals, reply, 2, 100);
  disableSPI(adc);
  return reply[0];
}

/**
 * Sets up the ADC and starts a reading.
 * You can find the channel IDs in mcp346x.h
 *
 * @param adc  the ADC to read from
 * @param vp   the 4bit ID of the positive channel
 * @param vn   the 4bit ID of the negative channel
 * @param gain the 3bit ID of the gain
 */
void MCP346x_startADC(MCP346x *adc, const uint8_t vp, const uint8_t vn,
                      const uint8_t gain) {
  MCP346x_setValue(adc, REG_CONFIG2, gain << 3, GAIN_MASK);
  MCP346x_writeReg(adc, REG_MUX, vp << 4 | vn);
  MCP346x_sendCmd(adc, CMD_FAST_GO);
}

/**
 * Gets the last reading started by MCP346x_startADC.
 * May crash your program if you haven't started a reading since there is no
 * timeout Also may not work if you've already read the value
 *
 * @param adc the ADC to read from
 * @return The raw read value
 */
int32_t MCP346x_readADC(const MCP346x *adc) {
  uint8_t buff[4];
  uint8_t status = 0xFF;
  while (status & STATUS_DATA_READY) {
    status = MCP346x_readRegs(adc, REG_ADCDATA, buff, sizeof(buff));
  }
  return (int32_t)((buff[0] << 24) | (buff[1] << 16) | (buff[2] << 8) |
                   buff[3]);
}

/**
 * Sets up the ADC, starts a reading, and gets the result.
 * You can find the channel IDs in mcp346x.h
 *
 * @param adc  the ADC to read from
 * @param vp   the 4bit ID of the positive channel
 * @param vn   the 4bit ID of the negative channel
 * @param gain the 3bit ID of the gain
 * @return the raw ADC reading
 */
int32_t MCP346x_analogRead(MCP346x *adc, const uint8_t vp, const uint8_t vn,
                           const uint8_t gain) {
  MCP346x_startADC(adc, vp, vn, gain << 3);
  return MCP346x_readADC(adc);
}

double MCP346x_convertVoltage(const MCP346x *adc, int32_t reading,
                              uint8_t gain) {
  double gainf = 1.0 / 3.0;
  if (gain) {
    gainf = (double)(1 << (gain - 1));
  }
  return ((double)reading * adc->ref) / gainf;
}

void MCP346x_setValue(MCP346x *adc, const uint8_t reg, const uint8_t value,
                      const uint8_t mask) {
  uint8_t val = adc->reg[reg];
  if (val & mask != value) {
    uint8_t old = val;
    val &= ~mask;
    val |= value;
    MCP346x_writeReg(adc, reg, val);
  }
}

void MCP346x_printRegs(const MCP346x *adc) {
  uint8_t reply[31];
  MCP346x_readRegs(adc, 0, reply, 31);
  // char buff[(16 * 3) + 3] = {0};
  char buff[(16 * 3) + 1] = {0};
  for (int i = 0; i < 29; i++) {
    snprintf(&buff[3 * (i % 16)], 4, "%02X ", reply[i]);
    if (i % 16 == 15) {
      uprintf("%s\n", buff);
    }
  }
  uprintf("%s\n", buff);
}

void bits(uint8_t num, int offset, int count, char *buff) {
  num <<= 8 - count - offset;
  for (int i = 0; i < count; i++) {
    *buff++ = ((num >> 7) & 1) + '0';
    num <<= 1;
  }
  *buff = 0;
}

void MCP346x_printInfo(const MCP346x *adc) {
  uint8_t reply[31];
  MCP346x_readRegs(adc, 0, reply, 31);
  char buff[8];
  uprintf("ADC Result: 0x%08x\n",
          reply[0] << 24 | reply[1] << 16 | reply[2] << 8 | reply[3]);
  bits(reply[4], 7, 1, buff);
  uprintf("Vref Int: %s\n", buff);
  bits(reply[4], 6, 1, buff);
  uprintf("Shutdown: %s\n", buff);
  bits(reply[4], 4, 2, buff);
  uprintf("CLK_SEL: %s\n", buff);
  bits(reply[4], 2, 2, buff);
  uprintf("CS_SEL: %s\n", buff);
  bits(reply[4], 0, 2, buff);
  uprintf("ADC_MODE: %s\n", buff);
  bits(reply[5], 6, 2, buff);
  uprintf("Prescaler: %s\n", buff);
  bits(reply[5], 2, 4, buff);
  uprintf("Oversampling: %s\n", buff);
  bits(reply[6], 6, 2, buff);
  uprintf("BOOST: %s\n", buff);
  bits(reply[6], 3, 3, buff);
  uprintf("GAIN: %s\n", buff);
  bits(reply[6], 2, 1, buff);
  uprintf("AZ_MUX: %s\n", buff);
  bits(reply[7], 6, 2, buff);
  uprintf("CONV_MODE: %s\n", buff);
  bits(reply[7], 4, 2, buff);
  uprintf("DATA_FORMAT: %s\n", buff);
  bits(reply[7], 3, 1, buff);
  uprintf("CRC_FORMAT: %s\n", buff);
  bits(reply[7], 2, 1, buff);
  uprintf("EN_CRCCOM: %s\n", buff);
  bits(reply[7], 1, 1, buff);
  uprintf("EN_OFFCAL: %s\n", buff);
  bits(reply[7], 0, 1, buff);
  uprintf("EN_GAINCAL: %s\n", buff);
}
// 7.23us start
// 5.88us with basic optimizations
