#include "loop.h"
#include <stdint.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
// #include "string.h"
// #include "unistd.h"
// #include <stdio.h>
#include "uprintf.h"

int count = 0;

extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef pHeader; //declare a specific header for message transmittions
// extern CAN_RxHeaderTypeDef pRxHeader; //declare header for message reception
// extern uint32_t TxMailbox;
// extern uint8_t a,r; //declare byte to be transmitted //declare a receive byte
extern CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure

void setupCanFilters() {
    pHeader.DLC=1; //give message size of 1 byte
	pHeader.IDE=CAN_ID_STD; //set identifier to standard
	pHeader.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
	pHeader.StdId=0x244; //define a standard identifier, used for message identification by filters (switch this for the other microcontroller)
	
	//filter one (stack light blink)
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh=0x245<<5; //the ID that the filter looks for (switch this for the other microcontroller)
	sFilterConfig.FilterIdLow=0;
	sFilterConfig.FilterMaskIdHigh=0;
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=ENABLE;
	
    HAL_StatusTypeDef s = HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
    if(s != HAL_OK) {
        uprintf("CAN filter init Error %d\n", s);
    } else {
        uprintf("CAN filter initalized\n", s);
    }
	 //configure CAN filter
	
	// HAL_CAN_Start(&hcan); //start CAN
	// HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
}

void sendDemoCanMessage() {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox = 0xDEADBEEF;

    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = 0xAAA;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;

    TxData[0] = 50;  
    // TxData[1] = 0xAA;

    // uprintf("State: %d ", hcan.State);
    HAL_StatusTypeDef s = HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(1);
    // uprintf("[%08x]", hcan.Instance->ESR);
    // uprintf("[%08x]", hcan.Instance->MSR);
    // uprintf("Free: %d ", HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
    // uprintf("Mailbox: %08x ", TxMailbox);
    // uprintf("Status: %08x ", HAL_CAN_GetState(&hcan));
    if(s != HAL_OK) {
        uprintf("TX Error %d", s);
    }

    uprintf(" RX ");
    int num = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
    uprintf("Size: %1d ", num);
    HAL_Delay(1);
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    s = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
    uprintf("Status: %08x ", HAL_CAN_GetState(&hcan));
    if(s != HAL_OK) {
        uprintf("RX Error %d", s);
    } else {

    }

    uprintf("\n");


}

// typedef enum 
// {
//   HAL_OK       = 0x00U,
//   HAL_ERROR    = 0x01U,
//   HAL_BUSY     = 0x02U,
//   HAL_TIMEOUT  = 0x03U
// } HAL_StatusTypeDef;

// Run once at the beginning
void setup() {

    HAL_Delay(2000);
    CDC_Transmit_FS((uint8_t*) "HELLO\r\n", 7);

    setupCanFilters();

    HAL_StatusTypeDef s = HAL_CAN_Start(&hcan);
    if(s != HAL_OK) {
        uprintf("CAN start error %d\n", s);
    } else {
        uprintf("CAN started\n");
    }
}

char data[32];
int first = 1;
// Called repeatedly while running 
void loop() {
    HAL_Delay(250);
    //   HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    uprintf("Test %4d ", count++);
    sendDemoCanMessage();
}