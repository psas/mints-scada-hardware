// Hardware setup functions config
// USB setup functions
// #include "usb_device.h"
// DataPackets
extern "C" {
    #include "stm32f0xx_hal.h"
    #include "stm32f0xx_hal_can.h"
    #include "main.h"
    #include "uprintf.h"
}

#include "datapacket.h"

#include "loop.h"

static int count = 0;
extern CAN_FilterTypeDef sFilterConfig;
uint8_t baseAddress = 0;

extern "C" {
void onCanMessage(void) {
    uprintf(" Got message! ");
    if(!HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)) {
        uprintf("Mailbox empty.\n");
        return;
    }
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
    if(rs != DATAPACKET_READ_SUCCESS) {
        uprintf("Packet read fail! %d\n", rs);
        return;
    }
    uprintf(" Reply [%d]: ", rs);
    printDataPacket(&rdp);
    uprintf(" FIFO:%d", HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0));

    uint8_t bid = rdp.id & 0xF0;
    uprintf(" BID:%2x", bid);

    if(bid == baseAddress) {
        uprintf(" 4Me!");

        uint8_t subid = rdp.id & 0xF;
        uprintf("#%d", subid);

        if(rdp.err) {
            uprintf("Error message! Undefined!");
        } else {
            if(rdp.reply) {
                uprintf("Warning! This packet looks like a reply from someone else on the same address!");
            }
        }
    }

    uprintf("\n");
}
}

int main(void) {

    // Initialize libraries
    initHardware();

    // Wait a moment for USB to connect
    HAL_Delay(2000);
    uprintf("Hello\n");

    // Read base ID
    // baseAddress |= HAL_GPIO_ReadPin(ADDR1_GPIO_Port, ADDR1_Pin) << 4;
    // baseAddress |= HAL_GPIO_ReadPin(ADDR2_GPIO_Port, ADDR2_Pin) << 5;
    // baseAddress |= HAL_GPIO_ReadPin(ADDR4_GPIO_Port, ADDR4_Pin) << 6;
    // baseAddress |= HAL_GPIO_ReadPin(ADDR8_GPIO_Port, ADDR8_Pin) << 7;

    baseAddress = 0x70;
    uprintf("My address is 0x%02x\n", baseAddress);

    // Set up the CAN filters
    // Set up a filter. Hopefully it just grabs everything
    sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
    sFilterConfig.FilterIdHigh=baseAddress<<5; //the ID that the filter looks for (switch this for the other microcontroller)
    sFilterConfig.FilterIdLow=0;
    sFilterConfig.FilterMaskIdHigh=0xF0<<5;
    sFilterConfig.FilterMaskIdLow=0;
    sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterActivation=ENABLE;

    HAL_StatusTypeDef canSetupStatus = HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
    if(canSetupStatus != HAL_OK) {
      uprintf("CAN filter init Error %d\n", canSetupStatus);
    } else {
      uprintf("CAN filter initalized\n", canSetupStatus);
    }

    // Start CAN and alert if it failed
    canSetupStatus = HAL_CAN_Start(&hcan);
    if(canSetupStatus != HAL_OK) {
        uprintf("CAN start error %d\n", canSetupStatus);
    } else {
        uprintf("CAN started\n");
    }

    while(true) {
#ifdef LOOPBACK
        DataPacket dp;
        // dp.id = 0x75;
        // dp.id = count & 0xFF;
        dp.id = baseAddress | 0x5;
        dp.err = 0;
        dp.reply = 1;
        dp.reserved = 0;
        dp.data.seq = 0x47;
        dp.data.cmd = 0x28;
        dp.data.args[0] = 0x00;
        dp.data.args[1] = 0x01;
        dp.data.args[2] = 0x02;
        dp.data.args[3] = 0x03;
        dp.datasize = 6;

        HAL_Delay(250);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        uprintf("Test TX %4d", count++);
        // sendDemoCanMessage();
        int ts = writeDatapacketToCan(&dp);
        uprintf("[%d]: ", ts);
        printDataPacket(&dp);

        // Wait and turn light on
        HAL_Delay(250);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        // uprintf("\n");
        onCanMessage();

        // if(count == 0x100) {
        //     while(true) {

        //     }
        // }
#endif
    }

}