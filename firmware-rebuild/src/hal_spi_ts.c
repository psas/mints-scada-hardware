#include "board_cfg.h"
#include "init.h"
#include "mcp346x.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_spi.c"
#include "stm32f0xx_hal_spi.h"
#include "uprintf.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                          uint8_t *pTxData, uint8_t *pRxData,
                                          uint16_t Size, uint32_t Timeout) {

  /* Variable used to alternate Rx and Tx during transfer */
  uint32_t txallowed = 1U;
  HAL_StatusTypeDef errorcode = HAL_OK;

  /* Check Direction parameter */
  assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

  /* Process Locked */
  __HAL_LOCK(hspi);

  /* Init tickstart for timeout management*/
  uint32_t tickstart = HAL_GetTick();

  /* Init temporary variables */
  HAL_SPI_StateTypeDef tmp_state = hspi->State;
  uint32_t tmp_mode = hspi->Init.Mode;
  uint16_t initial_TxXferCount = Size;

  if (!((tmp_state == HAL_SPI_STATE_READY) ||
        ((tmp_mode == SPI_MODE_MASTER) &&
         (hspi->Init.Direction == SPI_DIRECTION_2LINES) &&
         (tmp_state == HAL_SPI_STATE_BUSY_RX)))) {
    errorcode = HAL_BUSY;
    goto error;
  }

  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U)) {
    errorcode = HAL_ERROR;
    goto error;
  }

  /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
  if (hspi->State != HAL_SPI_STATE_BUSY_RX) {
    hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
  }

  /* Set the transaction information */
  hspi->ErrorCode = HAL_SPI_ERROR_NONE;
  hspi->pRxBuffPtr = (uint8_t *)pRxData;
  hspi->RxXferCount = Size;
  hspi->RxXferSize = Size;
  hspi->pTxBuffPtr = (uint8_t *)pTxData;
  hspi->TxXferCount = Size;
  hspi->TxXferSize = Size;

  /*Init field not used in handle to zero */
  hspi->RxISR = NULL;
  hspi->TxISR = NULL;

  /* Set the Rx Fifo threshold */
  if (hspi->Init.DataSize > SPI_DATASIZE_8BIT) {
    /* Set fiforxthreshold according the reception data length: 16bit */
    CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
  } else {
    /* Set fiforxthreshold according the reception data length: 8bit */
    SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
  }

  /* Check if the SPI is already enabled */
  if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) {
    /* Enable SPI peripheral */
    __HAL_SPI_ENABLE(hspi);
  }

  /* Transmit and Receive data in 16 Bit mode */
  if (hspi->Init.DataSize > SPI_DATASIZE_8BIT) {
    if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U)) {
      hspi->Instance->DR = *((uint16_t *)hspi->pTxBuffPtr);
      hspi->pTxBuffPtr += sizeof(uint16_t);
      hspi->TxXferCount--;
    }
    while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U)) {
      /* Check TXE flag */
      if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) &&
          (hspi->TxXferCount > 0U) && (txallowed == 1U)) {
        hspi->Instance->DR = *((uint16_t *)hspi->pTxBuffPtr);
        hspi->pTxBuffPtr += sizeof(uint16_t);
        hspi->TxXferCount--;
        /* Next Data is a reception (Rx). Tx not allowed */
        txallowed = 0U;
      }

      /* Check RXNE flag */
      if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) &&
          (hspi->RxXferCount > 0U)) {
        *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DR;
        hspi->pRxBuffPtr += sizeof(uint16_t);
        hspi->RxXferCount--;
        /* Next Data is a Transmission (Tx). Tx is allowed */
        txallowed = 1U;
      }
      if (((HAL_GetTick() - tickstart) >= Timeout) &&
          (Timeout != HAL_MAX_DELAY)) {
        errorcode = HAL_TIMEOUT;
        hspi->State = HAL_SPI_STATE_READY;
        goto error;
      }
    }
  }
  /* Transmit and Receive data in 8 Bit mode */
  else {
    if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U)) {
      *((__IO uint8_t *)&hspi->Instance->DR) = (*hspi->pTxBuffPtr);
      hspi->pTxBuffPtr += sizeof(uint8_t);
      hspi->TxXferCount--;
    }
    while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U)) {
      /* Check TXE flag */
      if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) &&
          (hspi->TxXferCount > 0U) && (txallowed == 1U)) {
        *(__IO uint8_t *)&hspi->Instance->DR = (*hspi->pTxBuffPtr);
        hspi->pTxBuffPtr++;
        hspi->TxXferCount--;
        /* Next Data is a reception (Rx). Tx not allowed */
        txallowed = 0U;
      }

      /* Wait until RXNE flag is reset */
      if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) &&
          (hspi->RxXferCount > 0U)) {
        (*(uint8_t *)hspi->pRxBuffPtr) = *(__IO uint8_t *)&hspi->Instance->DR;
        hspi->pRxBuffPtr++;
        hspi->RxXferCount--;
        /* Next Data is a Transmission (Tx). Tx is allowed */
        txallowed = 1U;
      }
      if ((((HAL_GetTick() - tickstart) >= Timeout) &&
           ((Timeout != HAL_MAX_DELAY))) ||
          (Timeout == 0U)) {
        errorcode = HAL_TIMEOUT;
        hspi->State = HAL_SPI_STATE_READY;
        goto error;
      }
    }
  }

  /* Check the end of the transaction */
  if (SPI_EndRxTxTransaction(hspi, Timeout, tickstart) != HAL_OK) {
    errorcode = HAL_ERROR;
    hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
  }

  if (hspi->ErrorCode != HAL_SPI_ERROR_NONE) {
    errorcode = HAL_ERROR;
  } else {
    hspi->State = HAL_SPI_STATE_READY;
  }

error:
  __HAL_UNLOCK(hspi);
  return errorcode;
}
