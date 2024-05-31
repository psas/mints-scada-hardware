// Header for this file
#include "loop.h"
// uint types
#include <stdint.h>
// USB serial printing
#include "uprintf.h"
// Other USB things
#include "usb_device.h"
#include "usbd_cdc_if.h"
// Data packets from CAN
#include "datapacket.h"

// There's an hcan variable somewhere. Compiler - Please find it and let me use it.
extern CAN_HandleTypeDef hcan;

void sendDemoCanMessage() {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox = 0xDEADBEEF;

    // Set up a demo message
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = 0x555;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = 0x00;
    TxData[1] = 0x01;
    TxData[2] = 0x02;
    TxData[3] = 0x03;
    TxData[4] = 0x04;
    TxData[5] = 0x05;
    TxData[6] = 0x06;
    TxData[7] = 0x07;

    // uprintf("State: %d ", hcan.State);
    HAL_StatusTypeDef s = HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    if(s != HAL_OK) {
        uprintf("TX Error %d", s);
    }

    // Wait a moment for the message to actually transmit
    HAL_Delay(1);
    // Receive the packet
    uprintf("RX ");
    int num = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
    uprintf("Size: %1d ", num);
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    s = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
    // uprintf("Status: %08x ", HAL_CAN_GetState(&hcan));
    if(s != HAL_OK) {
        uprintf("RX Error %d", s);
    } else {
        uprintf("ID: %03x ", RxHeader.StdId);
        uprintf("Length: %1d ", RxHeader.DLC);
        uprintf("Data: [");
        for(int i = 0; i < RxHeader.DLC; i++) {
            if(i!=0) {
                uprintf(",");
            }
            uprintf("%02x", RxData[i]);
        }
        uprintf("]");
    }

    uprintf("\n");


}

// Run once at the beginning
void setup() {
    // Demo data packet

}

//   HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

int count = 0;
// Called repeatedly while running 
void loop() {
    // Do stuff on light off

    DataPacket dp;
    dp.id = 0x5A;
    dp.err = 0;
    dp.reply = 1;
    dp.data.seq = 0x47;
    dp.data.cmd = 0x28;
    dp.data.args[0] = 0x00;
    dp.data.args[1] = 0x01;
    dp.data.args[2] = 0x02;
    dp.data.args[3] = 0x03;
    dp.datasize = 6;

    HAL_Delay(250);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    uprintf("Test %4d", count++);
    // sendDemoCanMessage();
    int ts = writeDatapacketToCan(&dp);
    uprintf("[%d]: ", ts);
    printDataPacket(&dp);

    // Wait and turn light on
    HAL_Delay(250);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    DataPacket rdp;
    rdp.id = 0;
    rdp.err = 0;
    rdp.reply = 0;
    rdp.data.seq = 0;
    rdp.data.cmd = 0;
    for(int i = 0; i < 6; i++) {
        rdp.data.args[i] = 0;
    }
    rdp.datasize = 0;
    int rs = readDataPacketFromCan(&rdp);
    uprintf(" Reply [%d]: ", rs);
    printDataPacket(&rdp);
    uprintf(" FIFO:%d", HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0));
    uprintf("\n");


}