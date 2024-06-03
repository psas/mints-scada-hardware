#include "main.h"
#include "stm32f0xx_hal.h"
#include "uprintf.h"
#include "datapacket.h"
// USB printf
// Get stuff from main, mostly CAN stuff
extern CAN_HandleTypeDef hcan;

void printDataPacket(DataPacket* pkt) {
    uprintf("%c%d %c%02X #%02x !%02x", (pkt->err)?'E':'.', pkt->reserved&1, (pkt->reply)?'<':'>', pkt->id, pkt->data.seq, pkt->data.cmd);
    for(int i = 0; i < pkt->datasize-2; i++) {
        uprintf(" %02X", pkt->data.args[i]);
    }
    // print(f"{'E' if self.err == 1 else '.'}{self.rsvd:01b} {'<' if self.reply else '>'}{self.id:02X} #{self.seq:02X} !{self.cmd:02x}: {' '.join([f'{b:02X}' for b in self.data])}")

}

int readDataPacketFromCan(DataPacket* dest) {
    int num = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
    if(num > 0) {
        // Place to temporarily store data
        CAN_RxHeaderTypeDef RxHeader;
        HAL_StatusTypeDef s = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, (uint8_t*) &(dest->data));
        dest->err = (RxHeader.StdId >> DATAPACKET_ERROR_BIT) & 1;
        dest->reserved = (RxHeader.StdId >> DATAPACKET_RESVD_BIT) & 1;
        dest->reply = (RxHeader.StdId >> DATAPACKET_REPLY_BIT) & 1;
        dest->id = RxHeader.StdId & 0xFF;
        dest->datasize = RxHeader.DLC;

        if(s != HAL_OK) {
            uprintf("RX Error %d", s);
            return DATAPACKET_READ_ERROR;
        } else {
            if(dest->datasize < 2) {
                return DATAPACKET_READ_TOOSMALL;
            }
            return DATAPACKET_READ_SUCCESS;
        }
    } else {
        return DATAPACKET_READ_NOTHING;
    }
}

int writeDatapacketToCan(DataPacket* pkt) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox = 0xDEADBEEF;

    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = (pkt->reply << DATAPACKET_REPLY_BIT) | (pkt->err << DATAPACKET_ERROR_BIT) | (pkt->reserved << DATAPACKET_RESVD_BIT) | pkt->id;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = pkt->datasize;

    // uprintf("State: %d ", hcan.State);
    HAL_StatusTypeDef s = HAL_CAN_AddTxMessage(&hcan, &TxHeader, (const uint8_t*) &(pkt->data), &TxMailbox);
    if(s != HAL_OK) {
        uprintf("TX Error %d", s);
        return DATAPACKET_WRITE_ERROR;
    }
    return DATAPACKET_WRITE_SUCCESS;
}