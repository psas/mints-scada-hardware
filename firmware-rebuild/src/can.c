#include "can.h"
#include "board_cfg.h"
#include "main.h"
#include "stm32f0xx_hal_can.h"
#include "usb.h"
#include <stdint.h>

uint8_t module_id = 0xFFFF1000;

CAN_HandleTypeDef hcan;

dataframe_read_status readCAN_MSG(DataFrame *RxData) {
  CAN_RxHeaderTypeDef RxHeader = {0};
  int fifo_lvl = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
  if (fifo_lvl > 0) {
    HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(
        &hcan, CAN_RX_FIFO0, &RxHeader, (uint8_t *)&(RxData->data));
    RxData->ide = RxHeader.IDE;
    RxData->id = RxHeader.StdId;
    RxData->frametype = RxHeader.RTR;
    RxData->datasize = RxHeader.DLC;

    if (status != HAL_OK) {
      uprintf("RX Error %d", RxData);
      return DATAFRAME_READ_ERROR;
    }
    if (RxData->datasize < 2) {
      uprintf("Data size too small");
      uprintf("State: %d ", hcan.State);
      return DATAFRAME_READ_TOOSMALL;
    }
    uprintf("State: %d ", hcan.State);
    return DATAFRAME_READ_SUCCESS;
  } else {
    return DATAFRAME_READ_NOTHING;
  }
}

dataframe_send_status sendCAN_MSG(DataFrame *TxData) {
  uint32_t TxMailbox = 0xDEADBEEF;
  // uprintf("State: %d ", hcan.State);
  CAN_TxHeaderTypeDef TxHeader = {.IDE = TxData->ide,
                                  .StdId = TxData->id,
                                  .RTR = TxData->frame_type,
                                  .DLC = TxData->datasize};
  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, (const uint8_t *)&(TxData->data),
                           &TxMailbox) != HAL_OK) {
    uprintf("HAL Error on TX");
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

// int processFrame(struct DataFrame_t *can_msg) {
// }

int getCAN_MSG(void) {
  // while there is a message
  while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    struct DataFrame_t can_msg = {0};
    int readmsg = readCAN_MSG(&can_msg);
    switch (readmsg) {
    case DATAFRAME_READ_SUCCESS:
      if (can_msg.id != module_id) {
        return DATAFRAME_READ_WRONG_ID;
      } else {
        //processFrame(&can_msg);
        return DATAFRAME_READ_SUCCESS;
      }
    case DATAFRAME_READ_ERROR:
      return DATAFRAME_READ_ERROR;
    case DATAFRAME_READ_NOTHING:
      return DATAFRAME_READ_NOTHING;
    case DATAFRAME_READ_TOOSMALL:
      return DATAFRAME_READ_TOOSMALL;
    }
  }
}
