#include "can.h"
#include "main.h"
#include "stm32f0xx_hal_can.h"
#include "usb.h"

CAN_HandleTypeDef hcan;

dataframe_read_status readCAN_MSG(DataFrame *RxData) {
  CAN_RxHeaderTypeDef RxHeader;
  while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
    RxData->ide = RxHeader.IDE;
    RxData->id = RxHeader.StdId; // should this be assigned to StdId or ExtId based on Rxdata.ide?
    RxData->frame_type = RxHeader.RTR;
    RxData->datasize = RxHeader.DLC;
  }

  if (RxData != HAL_OK) {
    uprintf("RX Error %d", RxData);
    return DATAFRAME_READ_ERROR;
  }
  if (RxData->datasize < 2) {
    return DATAFRAME_READ_TOOSMALL;
  }

  return DATAFRAME_READ_SUCCESS;
  if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) <= 0) {
    return DATAFRAME_READ_NOTHING;
  }
}

dataframe_send_status sendCAN_MSG(DataFrame *TxData) {
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox = 0xDEADBEEF;

  TxHeader.IDE = TxData->ide;
  TxHeader.StdId = TxData->id;
  TxHeader.RTR = TxData->frame_type;
  TxHeader.DLC = TxData->datasize;

  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, (const uint8_t *)&(TxData->data),
                           &TxMailbox) != HAL_OK) {
    uprintf("TX Error");
    return DATAFRAME_SEND_ERROR;
  }
  return DATAFRAME_SEND_SUCCESS;
}

// TODO: Figure out why Ben did these bitwise operations
// dest->err = (RxHeader.StdId >> DATAFRAME_ERROR_BIT) & 1;
// dest->reserved = (RxHeader.StdId >> DATAFRAME_RESVD_BIT) & 1;
// dest->reply = (RxHeader.StdId >> DATAFRAME_REPLY_BIT) & 1;
// dest->id = RxHeader.StdId & 0xFF;
// dest->datasize = RxHeader.DLC;
// TODO: Set a filter to only look for messages from the host device(comp)
