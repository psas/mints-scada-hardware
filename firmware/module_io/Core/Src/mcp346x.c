//
// Created by benjamin on 5/25/25.
//
#include "mcp346x.h"

#include <stdio.h>

#include "uprintf.h"

MCP346x MCP346x_Init(SPI_HandleTypeDef* spi, GPIO_TypeDef* cs_port, uint16_t cs_pin, GPIO_TypeDef* en_port, uint16_t en_pin) {
    MCP346x adc;
    adc.hspi = spi;
    __HAL_SPI_ENABLE(spi);
    adc.cs_port = cs_port;
    adc.cs_pin = cs_pin;
    adc.en_port = en_port;
    adc.en_pin = en_pin;
    adc.ref = 7.354736328125e-05; // 3.4 / (15900 * 3)

    uint8_t firststatus = MCP346x_sendCmd(&adc, CMD_FAST_RESET);
    // uprintf("%02x\n", firststatus);

    uint8_t cmds[] = {
        /* CFG0 */ CONFIG0_AWAKE | CLK_SEL_INTERNAL | BIAS_OFF | MODE_STANDBY,
        /* CFG1 */ PRESCALER_1 | OSR_32,
        /* CFG2 */ BOOST_1 | GAIN_1<<3 | MUX_ZERO_OFF | 0b11,
        /* CFG3 */ CONV_MODE_CONTINUOUS | DATA_FORMAT_16 | CRC_FORMAT_16
        | CRC_ON_READ_DISABLED | DIGITAL_OFFSET_CALIB_DISABLED | GAIN_CALIB_DISABLED,
        /* IRQ  */
        IRQ_PIN_MODE_IRQ | IRQ_PIN_INACTIVE_LOGIC | FAST_COMMANDS_ENABLED | CONVERSION_START_INTERRUPT_DISABLED,
        /* MUX  */ 0x01,
        /* SCAN */ 0, 0, 0,
        /* TIMR */ 0, 0, 0,
        /* OFST */ 0x00, 0x00, 0x3E,
        /* GAIN */ 0x7D, 0xEE, 0x80
    };
    MCP346x_writeRegs(&adc, REG_CONFIG0, cmds, 18);

    cmds[3] = 4;

    return adc;
}

void enable(const MCP346x* adc) {
    if (adc->en_pin) {
        HAL_GPIO_WritePin(adc->en_port, adc->en_pin, GPIO_PIN_SET);
    }
    if (adc->cs_pin) {
        HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET);
    }
}

void disable(const MCP346x* adc) {
    if (adc->cs_pin) {
        HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET);
    }
    if (adc->en_pin) {
        HAL_GPIO_WritePin(adc->en_port, adc->en_pin, GPIO_PIN_RESET);
    }
}

uint8_t MCP346x_sendCmd(const MCP346x* adc, const uint8_t fastcmd) {
    // 2 bit device address, 4 bit register address, 2 bit command
    uint8_t cmd = 0b01 << 6 | fastcmd << 2 | CMD_TYPE_FAST;
    // uprintf("Sending %02x\n", cmd);
    uint8_t status;

    // Select the chip to begin the transaction
    enable(adc);

    // Send the read command
    HAL_GPIO_WritePin(GP1_GPIO_Port, GP1_Pin, GPIO_PIN_SET);
    // HAL_SPI_TransmitReceive(&hspi1, &cmd, &status, 1, 100);
    SPI_TransmitReceive(&hspi1, &cmd, &status, 1, 100);
    HAL_GPIO_WritePin(GP1_GPIO_Port, GP1_Pin, GPIO_PIN_RESET);

    // Unselect the chip to end the transaction
    disable(adc);

    return status;
}

uint8_t MCP346x_readRegs(const MCP346x* adc, const uint8_t reg, uint8_t* result, const int count) {
    // 2 bit device address, 4 bit register address, 2 bit command

    // Select the chip to begin the transaction
    enable(adc);


    // 19.73 to start

    // Send the read command
    HAL_GPIO_WritePin(GP3_GPIO_Port, GP3_Pin, GPIO_PIN_SET);
    // uint8_t cmd = 0b01 << 6 | reg << 2 | CMD_TYPE_READY_MANY;
    // uint8_t status;
    // SPI_TransmitReceive(&hspi1, &cmd, &status, 1, 100);
    // HAL_SPI_Receive(&hspi1, result, count, 1000);

    uint8_t tosend[count+1];
    tosend[0] = PACK_COMMAND(reg, CMD_TYPE_READY_MANY);
    uint8_t reply[count+1];
    HAL_SPI_TransmitReceive(&hspi1, tosend, reply, count+1, 100);
    for (int i = 0; i < count; i++) {
        result[i] = reply[i+1];
    }
    // Read the desired number of result bytes
    HAL_GPIO_WritePin(GP3_GPIO_Port, GP3_Pin, GPIO_PIN_RESET);

    // Unselect the chip to end the transaction
    disable(adc);

    // return status;
    return reply[0];
}

uint8_t MCP346x_readReg(const MCP346x* adc, const uint8_t reg) {
    uint8_t reply;
    MCP346x_readRegs(adc, reg, &reply, 1);
    return reply;
}


uint8_t MCP346x_writeRegs(MCP346x* adc, const uint8_t reg, const uint8_t* values, const int count) {
    // Pack the values into a single SPI task
    uint8_t tosend[count+1];
    // 2 bit device address, 4 bit register address, 2 bit command
    tosend[0] = PACK_COMMAND(reg, CMD_TYPE_WRITE_MANY);

    // Copy over values and cache changes
    for (int i = 0; i < count; i++) {
        tosend[i+1] = values[i];
        adc->reg[reg + i] = values[i];
    }

    // Spot to store the reply
    uint8_t reply[count+1];

    // Select the chip to begin the transaction
    enable(adc);

    // Send the command and values
    SPI_TransmitReceive(&hspi1, tosend, reply, count+1, 100);

    // Unselect the chip to end the transaction
    disable(adc);

    // return status;
    return reply[0];
}

uint8_t MCP346x_writeReg(MCP346x* adc, const uint8_t reg, uint8_t value) {
    // 17.2 us to begin, 440 total
    // 11.65 us switching to optomized SPI, 391 total
    // 10 us optomized, 377 total
    HAL_GPIO_WritePin(GP2_GPIO_Port, GP2_Pin, GPIO_PIN_SET);
    // uint8_t vals[] = {value};
    uint8_t vals[] = {PACK_COMMAND(reg, CMD_TYPE_WRITE_MANY), value};
    uint8_t reply[2];
    enable(adc);
    SPI_TransmitReceive(&hspi1, vals, reply, 2, 100);
    disable(adc);
    // uint8_t reply = MCP346x_writeRegs(adc, reg, vals, 1);
    HAL_GPIO_WritePin(GP2_GPIO_Port, GP2_Pin, GPIO_PIN_RESET);
    // return reply;
    return reply[0];
}

void MCP346x_startADC(MCP346x* adc, const uint8_t vp, const uint8_t vs, const uint8_t gain) {
    MCP346x_setValue(adc, REG_CONFIG2, gain<<3, GAIN_MASK);
    MCP346x_writeReg(adc, REG_MUX, vp << 4 | vs);
    MCP346x_sendCmd(adc, CMD_FAST_GO);
}

int16_t MCP346x_readADC(const MCP346x* adc) {
    uint8_t buff[2];
    uint8_t status = 0xFF;
    while (status & STATUS_DATA_READY) {
        status = MCP346x_readRegs(adc, REG_ADCDATA, buff, 2);
    }
    return (int16_t)((buff[0] << 8) | buff[1]);
}

int16_t MCP346x_analogRead(MCP346x* adc, const uint8_t vp, const uint8_t vs, const uint8_t gain) {
    MCP346x_startADC(adc, vp, vs, gain<<3);
    return MCP346x_readADC(adc);
}

double MCP346x_convertVoltage(const MCP346x* adc, int16_t reading, uint8_t gain) {
    double gainf = 1.0 / 3.0;
    if (gain) {
        gainf = (double)(1 << (gain - 1));
    }
    return ((double)reading * adc->ref) / gainf;
}

void MCP346x_setValue(MCP346x* adc, const uint8_t reg, const uint8_t value, const uint8_t mask) {
    uint8_t val = adc->reg[reg];
    if (val & mask != value) {
        uint8_t old = val;
        val &= ~mask;
        val |= value;
        MCP346x_writeReg(adc, reg, val);
    }
}

void MCP346x_printRegs(const MCP346x* adc) {
    uint8_t reply[29];
    MCP346x_readRegs(adc, 0, reply, 34);
    // char buff[(16 * 3) + 3] = {0};
    char buff[(16 * 3) + 1] = {0};
    for (int i = 0; i < 29; i++) {
        snprintf(&buff[3 * (i % 16)], 4, "%02X ", reply[i]);
        if (i % 16 == 15) {
            // buff[(16 * 3)] = '\n';
            // buff[(16 * 3)+1] = '\n';
            // buff[(16 * 3)+2] = '\0';
            // uprint(buff, (16 * 3) + 3);
            uprintf("%s\n", buff);
        }
    }
    // buff[(16 * 3)] = '\n';
    // buff[(16 * 3)+1] = '\n';
    // buff[(16 * 3)+2] = '\0';
    // uprint(buff, (16 * 3) + 3);
    uprintf("%s\n", buff);
}

void bits(uint8_t num, int offset, int count, char* buff) {
    num <<= 8 - count - offset;
    for (int i = 0; i < count; i++) {
        *buff++ = ((num >> 7) & 1) + '0';
        num <<= 1;
    }
    *buff = 0;
}

void MCP346x_printInfo(const MCP346x* adc) {
    uint8_t reply[29];
    MCP346x_readRegs(adc, 0, reply, 34);
    char* buff[4];
    uprintf("ADC Result: 0x%04x\n", reply[0] << 8 | reply[1]);
    bits(reply[2], 6, 2, buff);
    uprintf("Shutdown: %s\n", buff);
    bits(reply[2], 4, 2, buff);
    uprintf("CLK_SEL: %s\n", buff);
    bits(reply[2], 2, 2, buff);
    uprintf("CS_SEL: %s\n", buff);
    bits(reply[2], 0, 2, buff);
    uprintf("ADC_MODE: %s\n", buff);
    bits(reply[3], 6, 2, buff);
    uprintf("Prescaler: %s\n", buff);
    bits(reply[3], 2, 4, buff);
    uprintf("Oversampling: %s\n", buff);
    bits(reply[4], 6, 2, buff);
    uprintf("BOOST: %s\n", buff);
    bits(reply[4], 3, 3, buff);
    uprintf("GAIN: %s\n", buff);
    bits(reply[4], 2, 1, buff);
    uprintf("AZ_MUX: %s\n", buff);
    bits(reply[5], 6, 2, buff);
    uprintf("CONV_MODE: %s\n", buff);
    bits(reply[5], 4, 2, buff);
    uprintf("DATA_FORMAT: %s\n", buff);
    bits(reply[5], 3, 1, buff);
    uprintf("CRC_FORMAT: %s\n", buff);
    bits(reply[5], 2, 1, buff);
    uprintf("EN_CRCCOM: %s\n", buff);
    bits(reply[5], 1, 1, buff);
    uprintf("EN_OFFCAL: %s\n", buff);
    bits(reply[5], 0, 1, buff);
    uprintf("EN_GAINCAL: %s\n", buff);
}

// 7.23us start
// 5.88us with basic optimizations

/**
  * @brief  Transmit and Receive an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent and received
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
int SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size,
                        uint32_t Timeout) {
    uint16_t initial_TxXferCount;
    uint16_t initial_RxXferCount;
    uint32_t tmp_mode;
    HAL_SPI_StateTypeDef tmp_state;
    uint32_t tickstart;

    /* Variable used to alternate Rx and Tx during transfer */
    uint32_t txallowed = 1U;
    HAL_StatusTypeDef errorcode = HAL_OK;

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    /* Init temporary variables */
    tmp_state = hspi->State;
    tmp_mode = hspi->Init.Mode;
    initial_TxXferCount = Size;
    initial_RxXferCount = Size;

    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if (hspi->State != HAL_SPI_STATE_BUSY_RX) {
        hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    }

    /* Set the transaction information */
    hspi->ErrorCode = HAL_SPI_ERROR_NONE;
    hspi->pRxBuffPtr = (uint8_t*)pRxData;
    hspi->RxXferCount = Size;
    hspi->RxXferSize = Size;
    hspi->pTxBuffPtr = (uint8_t*)pTxData;
    hspi->TxXferCount = Size;
    hspi->TxXferSize = Size;

    /*Init field not used in handle to zero */
    hspi->RxISR = NULL;
    hspi->TxISR = NULL;

    /* Set the Rx Fifo threshold */
    SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);

    if (initial_TxXferCount == 0x01U) {
        if (hspi->TxXferCount > 1U) {
            hspi->Instance->DR = *((uint16_t*)hspi->pTxBuffPtr);
            hspi->pTxBuffPtr += sizeof(uint16_t);
            hspi->TxXferCount -= 2U;
        } else {
            *(__IO uint8_t*)&hspi->Instance->DR = (*hspi->pTxBuffPtr);
            hspi->pTxBuffPtr++;
            hspi->TxXferCount--;
        }
    }
    while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U)) {
        /* Check TXE flag */
        if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) && (hspi->TxXferCount > 0U) && (txallowed == 1U)) {
            if (hspi->TxXferCount > 1U) {
                hspi->Instance->DR = *((uint16_t*)hspi->pTxBuffPtr);
                hspi->pTxBuffPtr += sizeof(uint16_t);
                hspi->TxXferCount -= 2U;
            } else {
                *(__IO uint8_t*)&hspi->Instance->DR = (*hspi->pTxBuffPtr);
                hspi->pTxBuffPtr++;
                hspi->TxXferCount--;
            }
            /* Next Data is a reception (Rx). Tx not allowed */
            txallowed = 0U;
        }

        /* Wait until RXNE flag is reset */
        if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) && (hspi->RxXferCount > 0U)) {
            if (hspi->RxXferCount > 1U) {
                *((uint16_t*)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DR;
                hspi->pRxBuffPtr += sizeof(uint16_t);
                hspi->RxXferCount -= 2U;
                if (hspi->RxXferCount <= 1U) {
                    /* Set RX Fifo threshold before to switch on 8 bit data size */
                    SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
                }
            } else {
                (*(uint8_t*)hspi->pRxBuffPtr) = *(__IO uint8_t*)&hspi->Instance->DR;
                hspi->pRxBuffPtr++;
                hspi->RxXferCount--;
            }
            /* Next Data is a Transmission (Tx). Tx is allowed */
            txallowed = 1U;
        }
    }

    hspi->State = HAL_SPI_STATE_READY;
    // __HAL_UNLOCK(hspi);
    return errorcode;
}

// HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
// {
//   uint32_t tickstart;
//   HAL_StatusTypeDef errorcode = HAL_OK;
//
//   if ((hspi->Init.DataSize > SPI_DATASIZE_8BIT) || ((hspi->Init.DataSize <= SPI_DATASIZE_8BIT) && (Size > 1U)))
//   {
//     /* in this case, 16-bit access is performed on Data
//        So, check Data is 16-bit aligned address */
//     assert_param(IS_SPI_16BIT_ALIGNED_ADDRESS(pData));
//   }
//
//   if ((hspi->Init.Mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES))
//   {
//     hspi->State = HAL_SPI_STATE_BUSY_RX;
//     /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
//     return HAL_SPI_TransmitReceive(hspi, pData, pData, Size, Timeout);
//   }
//
//   /* Process Locked */
//   __HAL_LOCK(hspi);
//
//   /* Init tickstart for timeout management*/
//   tickstart = HAL_GetTick();
//
//   if (hspi->State != HAL_SPI_STATE_READY)
//   {
//     errorcode = HAL_BUSY;
//     goto error;
//   }
//
//   if ((pData == NULL) || (Size == 0U))
//   {
//     errorcode = HAL_ERROR;
//     goto error;
//   }
//
//   /* Set the transaction information */
//   hspi->State       = HAL_SPI_STATE_BUSY_RX;
//   hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
//   hspi->pRxBuffPtr  = (uint8_t *)pData;
//   hspi->RxXferSize  = Size;
//   hspi->RxXferCount = Size;
//
//   /*Init field not used in handle to zero */
//   hspi->pTxBuffPtr  = (uint8_t *)NULL;
//   hspi->TxXferSize  = 0U;
//   hspi->TxXferCount = 0U;
//   hspi->RxISR       = NULL;
//   hspi->TxISR       = NULL;
//
// #if (USE_SPI_CRC != 0U)
//   /* Reset CRC Calculation */
//   if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
//   {
//     SPI_RESET_CRC(hspi);
//     /* this is done to handle the CRCNEXT before the latest data */
//     hspi->RxXferCount--;
//   }
// #endif /* USE_SPI_CRC */
//
//   /* Set the Rx Fifo threshold */
//   if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
//   {
//     /* Set RX Fifo threshold according the reception data length: 16bit */
//     CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
//   }
//   else
//   {
//     /* Set RX Fifo threshold according the reception data length: 8bit */
//     SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
//   }
//
//   /* Configure communication direction: 1Line */
//   if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
//   {
//     SPI_1LINE_RX(hspi);
//   }
//
//   /* Check if the SPI is already enabled */
//   if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
//   {
//     /* Enable SPI peripheral */
//     __HAL_SPI_ENABLE(hspi);
//   }
//
//   /* Receive data in 8 Bit mode */
//   if (hspi->Init.DataSize <= SPI_DATASIZE_8BIT)
//   {
//     /* Transfer loop */
//     while (hspi->RxXferCount > 0U)
//     {
//       /* Check the RXNE flag */
//       if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
//       {
//         /* read the received data */
//         (* (uint8_t *)hspi->pRxBuffPtr) = *(__IO uint8_t *)&hspi->Instance->DR;
//         hspi->pRxBuffPtr += sizeof(uint8_t);
//         hspi->RxXferCount--;
//       }
//       else
//       {
//         /* Timeout management */
//         if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
//         {
//           errorcode = HAL_TIMEOUT;
//           goto error;
//         }
//       }
//     }
//   }
//   else
//   {
//     /* Transfer loop */
//     while (hspi->RxXferCount > 0U)
//     {
//       /* Check the RXNE flag */
//       if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
//       {
//         *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DR;
//         hspi->pRxBuffPtr += sizeof(uint16_t);
//         hspi->RxXferCount--;
//       }
//       else
//       {
//         /* Timeout management */
//         if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
//         {
//           errorcode = HAL_TIMEOUT;
//           goto error;
//         }
//       }
//     }
//   }
//
// #if (USE_SPI_CRC != 0U)
//   /* Handle the CRC Transmission */
//   if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
//   {
//     /* freeze the CRC before the latest data */
//     SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
//
//     /* Read the latest data */
//     if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, Timeout, tickstart) != HAL_OK)
//     {
//       /* the latest data has not been received */
//       errorcode = HAL_TIMEOUT;
//       goto error;
//     }
//
//     /* Receive last data in 16 Bit mode */
//     if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
//     {
//       *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DR;
//     }
//     /* Receive last data in 8 Bit mode */
//     else
//     {
//       (*(uint8_t *)hspi->pRxBuffPtr) = *(__IO uint8_t *)&hspi->Instance->DR;
//     }
//
//     /* Wait the CRC data */
//     if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, Timeout, tickstart) != HAL_OK)
//     {
//       SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
//       errorcode = HAL_TIMEOUT;
//       goto error;
//     }
//
//     /* Read CRC to Flush DR and RXNE flag */
//     if (hspi->Init.DataSize == SPI_DATASIZE_16BIT)
//     {
//       /* Read 16bit CRC */
//       READ_REG(hspi->Instance->DR);
//     }
//     else
//     {
//       /* Read 8bit CRC */
//       READ_REG(*(__IO uint8_t *)&hspi->Instance->DR);
//
//       if ((hspi->Init.DataSize == SPI_DATASIZE_8BIT) && (hspi->Init.CRCLength == SPI_CRC_LENGTH_16BIT))
//       {
//         if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, Timeout, tickstart) != HAL_OK)
//         {
//           /* Error on the CRC reception */
//           SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
//           errorcode = HAL_TIMEOUT;
//           goto error;
//         }
//         /* Read 8bit CRC again in case of 16bit CRC in 8bit Data mode */
//         READ_REG(*(__IO uint8_t *)&hspi->Instance->DR);
//       }
//     }
//   }
// #endif /* USE_SPI_CRC */
//
//   /* Check the end of the transaction */
//   if (SPI_EndRxTransaction(hspi, Timeout, tickstart) != HAL_OK)
//   {
//     hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
//   }
//
// #if (USE_SPI_CRC != 0U)
//   /* Check if CRC error occurred */
//   if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
//   {
//     SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
//     __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
//   }
// #endif /* USE_SPI_CRC */
//
//   if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
//   {
//     errorcode = HAL_ERROR;
//   }
//
// error :
//   hspi->State = HAL_SPI_STATE_READY;
//   __HAL_UNLOCK(hspi);
//   return errorcode;
// }