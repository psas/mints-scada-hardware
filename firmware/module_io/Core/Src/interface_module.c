// Hardware setup functions config
// USB setup functions
// #include "usb_device.h"
// DataPackets
#include "interface_module.h"

#include "stm32f042x6.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"
#include "stm32f0xx_hal_adc.h"
#include "main.h"
#include "uprintf.h"
#include "buscommands.h"
#include "configuration.h"

#include "datapacket.h"

// #include "loop.h"

/** TODO
 * Add checkbox to software in order to quickly remove sensors from the autopoller
 * Add message queue to firmware so that long tasks don't block future messages.
 */

extern CAN_FilterTypeDef sFilterConfig;
uint8_t baseAddress = 0;

// Represents if a fatal error has occurred
int fatal = 0;

#ifdef LOOPBACK
static int count = 0;
#endif

#ifdef CONFIG_OUTPUTS
GPIO_TypeDef* outputPorts[] = {LED_GPIO_Port, OUT1_GPIO_Port, OUT2_GPIO_Port, OUT3_GPIO_Port, OUT4_GPIO_Port, OUT5_GPIO_Port, OUT6_GPIO_Port, OUT7_GPIO_Port};
uint16_t outputPins[] = {LED_Pin, OUT1_Pin, OUT2_Pin, OUT3_Pin, OUT4_Pin, OUT5_Pin, OUT6_Pin, OUT7_Pin};
#endif

#ifdef CONFIG_ADC
#include "mcp346x.h"
MCP346x extadc;
#endif

void onFatalError(void) {
    fatal = 1;
    // Do things here to set things to a safe state.
    // For outputs, this means all off.
    // For inputs, this means nothing.
}

/**
 * Copies several bytes of UID to somewhere
 * Copies 0 < bytes <= 12 bytes to dest
 * Starts at 0 <= offset < 12 bytes into the UID
 * If bytes of offset would give illegal locations, they are modified to not.
*/
void copyUID(uint8_t* dest, uint8_t bytes, uint8_t offset) {
    if(offset > 11) {
        offset = 11;
    }
    if(bytes + offset > 12) {
        bytes = 12 - offset;
    }
    if(bytes < 1) {
        bytes = 1;
    }

    // UID_BASE
    uint8_t* rdp = (uint8_t*) (UID_BASE + offset);
    for(int i = 0; i < bytes; i++) {
        *dest++ = *rdp++;
    }
}

/**
 * Writes a compressed version of the UID to a given memory location.
 * Always writes 6 bytes
*/
void compressUID(uint8_t* dest) {
    uint8_t* uid = (uint8_t*) UID_BASE;

    for(int i = 0; i < 6; i++) {
        dest[i] = (uid[i] + uid[i+6]) & 0xFF;
    }
}

int processPacket(DataPacket* pk) {
    uint8_t subid = (pk->id & 0xF) - BASE_ADDR_OFFSET;
    uprintf("#%d ", subid);

    // If the packet was an error, check if it's one we care about
    if(pk->err) {
        uprintf("That was an error message!");
        // If soneone else said they have my ID, that's bad and we need to stop.
        if(pk->data.cmd == BUSCMD_CLAIM_ID) {
            uint8_t tempid[6];
            compressUID(tempid);
            int f = 0;
            for(int i = 0; i < 6; i++) {
                f &= tempid[i] != pk->data.bytes[i]; 
            }
            if(f) {
                onFatalError();
                uprintf("\n[FATAL] Someone else already had my ID %02x\n", baseAddress);
            }
        }
        return 1;
    }
    // If the packet was a reply, then someone else is using my ID.
    // This is not allowed, so send an error reply

    // If the packet is a reply, it was probably just sent by us, so ignore it.
    if(pk->reply) {
        // onFatalError();
        // uprintf("[FATAL] Someone else with my ID sent a reply");
        // pk->err = 0; // Don't set the error bit 
        // pk->reply = 1;
        // writeDatapacketToCan(pk);
        // return 2;
        uprintf("That was a reply.");
        return 0;
    }
    if(pk->datasize < 2) {
        uprintf("Command too short!");
        return 3;
    }
    uprintf("Exec time! ");
    switch(pk->data.cmd) {
    case BUSCMD_CLAIM_ID: {
        uprintf("Id claim commnd!");
        uint8_t tempid[6];
        compressUID(tempid);
        int f = 0;
        for(int i = 0; i < 6; i++) {
            f &= tempid[i] != pk->data.bytes[i]; 
        }
        if(f) {
            pk->err = 1;
            pk->reply = 1;
            writeDatapacketToCan(pk);
        } else {
            uprintf("It was me");
        }
    } break;
    case BUSCMD_READ_ID_LOW: {
        uprintf("UID low read");
        pk->reply = 1;
        pk->datasize = 8;
        copyUID(pk->data.bytes, 6, 0);
    } break;
    case BUSCMD_READ_ID_HIGH: {
        uprintf("UID low high");
        pk->reply = 1;
        pk->datasize = 8;
        copyUID(pk->data.bytes, 6, 6);
        writeDatapacketToCan(pk);
    } break;
#ifdef CONFIG_OUTPUTS
    case BUSCMD_READ_VALUE: {
        uprintf("Read value command");
        uint32_t val = HAL_GPIO_ReadPin(outputPorts[subid], outputPins[subid]);
        pk->datasize = 8;
        // BigLittleData* bld = BIGLITTLEDATA(pk);
        BIGLITTLEDATA(pk)->big = val;
        // bld->big = val;
        pk->reply = 1;
        uprintf("\nSending reply ");
        printDataPacket(pk);
        writeDatapacketToCan(pk);
    } break;
    case BUSCMD_WRITE_VALUE: {
        uprintf("Write value command");
        HAL_GPIO_WritePin(outputPorts[subid], outputPins[subid], BIGLITTLEDATA(pk)->big);
    } break;
#endif
#ifdef CONFIG_ADC
    case BUSCMD_READ_VALUE: {
        uprintf("Read value command. %d %d", subid << 1, (subid << 1) + 1);
        // uint32_t val = MCP346x_analogRead(&extadc, MUX_AVDD, MUX_AGND, GAIN_1);
        uint32_t val = MCP346x_analogRead(&extadc, subid << 1, subid << 1 + 1, GAIN_1);
        pk->datasize = 8;
        // BigLittleData* bld = BIGLITTLEDATA(pk);
        BIGLITTLEDATA(pk)->big = val;
        // bld->big = val;
        pk->reply = 1;
        uprintf("\nSending reply ");
        printDataPacket(pk);
        writeDatapacketToCan(pk);
    } break;
#endif
    default: {
        uprintf("Unknown command");
    }
    }
    return 0;
}

void getCanMessages(void) {
    while(!fatal && HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        uprintf("Got message! ");
        DataPacket pk;
        pk.id = 0;
        pk.err = 0;
        pk.reply = 0;
        pk.data.seq = 0;
        pk.data.cmd = 0;
        for(int i = 0; i < 6; i++) {
            pk.data.bytes[i] = 0;
        }
        pk.datasize = 0;
        int rs = readDataPacketFromCan(&pk);
        if(rs != DATAPACKET_READ_SUCCESS) {
            uprintf("Packet read fail! %d\n", rs);
            return;
        }
        printDataPacket(&pk);

        uint8_t bid = pk.id & 0xF0;
        uprintf(" BID:%2x", bid);

        if((pk.id & (0xF0 | SUB_ADDR_MASK)) == baseAddress) {
            uprintf(" 4Me!");

            processPacket(&pk);
        }

        uprintf("\n");
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }
}

uint32_t random(int bits) {
    uint32_t rsp = 0;
    for(int i = 0; i < bits; i++) {
        HAL_ADC_Start(&hadc);
        while(HAL_ADC_PollForConversion(&hadc, 1000)) {
            // do a little waiting
        }
        rsp <<= 1;
        rsp |= (HAL_ADC_GetValue(&hadc) & 1);
    }
    return rsp;
}

/* Does everything. Is wrapped by main so that the program will halt if this ever returns. */
void doEverything(void) {
    
    // Initialize peripherial libraries
    initPeripherials();

    // Wait a moment for USB to connect
    // Might disable for production
    HAL_Delay(2000);
    uprintf("Hello\n");

    // Initialize ADC
#ifdef CONFIG_ADC
    extadc = MCP346x_Init(&hspi2, CTRL_GPIO_Port, CTRL_Pin, 0, 0);
#endif

    // Wait a random amount of time to ensure that if two devices try to start with the same ID,
    // one will have a change to get started and reply to the other alerting them of the issue.
    HAL_Delay(random(6));

    // Read base ID
    baseAddress |= HAL_GPIO_ReadPin(ADDR1_GPIO_Port, ADDR1_Pin) << 4;
    baseAddress |= HAL_GPIO_ReadPin(ADDR2_GPIO_Port, ADDR2_Pin) << 5;
    baseAddress |= HAL_GPIO_ReadPin(ADDR4_GPIO_Port, ADDR4_Pin) << 6;
    baseAddress |= HAL_GPIO_ReadPin(ADDR8_GPIO_Port, ADDR8_Pin) << 7;
    baseAddress += BASE_ADDR_OFFSET;

    uprintf("My base address is 0x%02x\n", baseAddress);

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
      onFatalError();
      return;
    } else {
      uprintf("CAN filter initalized\n");
    }
    // Start CAN and alert if it failed
    canSetupStatus = HAL_CAN_Start(&hcan);
    if(canSetupStatus != HAL_OK) {
        uprintf("CAN start error %d\n", canSetupStatus);
        onFatalError();
        return;
    } else {
        uprintf("CAN started\n");
    }

    // Setup ID claim command
    DataPacket iddp;
    iddp.id = baseAddress;
    iddp.err = 0; // Not an error packet
    iddp.reserved = 0;
    iddp.reply = 0; // Not a reply
    iddp.data.seq = 0; // Don't care what it is
    iddp.data.cmd = BUSCMD_CLAIM_ID; // Sets the ID to claim
    compressUID(iddp.data.bytes);
    iddp.datasize = 8; // Include the sequence number and command in this count
    // Send ID claim command
    int freeTX = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    int ts = writeDatapacketToCan(&iddp);
    if(ts != HAL_OK) {
        uprintf("\n[FATAL] Could not send CAN address claim packet.");
        onFatalError();
        return;
    }
    uprintf("Sent ID query ");
    printDataPacket(&iddp);
    uprintf("\n");
    // Wait for ID claim command to be sent
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < freeTX);

    while(1) {
        getCanMessages();
#ifdef LOOPBACK
        DataPacket dp;
        // dp.id = 0x75;
        // dp.id = count & 0xFF;
        dp.id = baseAddress | 0x5;
        dp.err = 0; // Not an error packet
        dp.reserved = 0; // Set to 0 for easier debugging
        dp.reply = 0; // Not a reply
        dp.data.seq = count & 0xFF; // Increment sequence number each time
        dp.data.cmd = BUSCMD_READ_ID_HIGH; // Sets the ID to claim
        dp.data.args[0] = 0x00;
        dp.data.args[1] = 0x01;
        dp.data.args[2] = 0x02;
        dp.data.args[3] = 0x03;
        dp.datasize = 6; // Include the sequence number and command in this count

        uint32_t endtime = HAL_GetTick() + 500;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        uprintf("0x%x Test TX %4d", random(4), count++);
        int ts = writeDatapacketToCan(&dp);
        uprintf("[%d]: ", ts);
        printDataPacket(&dp);
        uprintf("\n");
        HAL_Delay(50);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        while(endtime > HAL_GetTick()) {
            // HAL_Delay(1);
        }
#endif
        
    }

}

int main(void) {
    doEverything();
    while(1); // Halt if main ever exits
}