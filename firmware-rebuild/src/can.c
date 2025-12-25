#include "usb.h"
#include "main.h"
#include "can.h"
#include "stm32f0xx_hal_can.h"

// why don't we define this ??
CAN_HandleTypeDef hcan; 
// = {
//   hcan.id = 0x28,
// };

static void initCAN(void){
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }
}

void sendCAN_MSG(DataPacket* TxData){
  CAN_TxHeaderTypeDef TxHeader;
}

datapacket_read_status readCAN_MSG(DataPacket* RxData){
  CAN_RxHeaderTypeDef RxHeader;
  DataPacket datapkt = {0}; 
  while(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0){
    while(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0 ,&RxHeader, (uint8_t*) &RxData->data) == HAL_OK){
      RxData->id = 
      RxData->err = 
      RxData->reply = 
      RxData->reserved = 
      RxData->datasize = 
    }
    return (DATAPACKET_READ_ERROR);
  }
}

// TODO: Figure out why Ben did these bitwise operations
        // dest->err = (RxHeader.StdId >> DATAPACKET_ERROR_BIT) & 1;
        // dest->reserved = (RxHeader.StdId >> DATAPACKET_RESVD_BIT) & 1;
        // dest->reply = (RxHeader.StdId >> DATAPACKET_REPLY_BIT) & 1;
        // dest->id = RxHeader.StdId & 0xFF;
        // dest->datasize = RxHeader.DLC;
//TODO: Set a filter to only look for messages from the host device(comp)
