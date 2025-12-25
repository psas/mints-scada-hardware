#project name
PROJECT = firmware-rebuild

#cpu arch/type
CPU = -mcpu=cortex-m0 -mthumb
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
SZ = $(PREFIX)size
BUILD_DIR := ./build
SRC_DIR := ./src

C_SOURCES := ./cfg/board_cfg.c\
./cfg/usbd_cdc_if.c\
./cfg/usbd_desc.c\
./cfg/usbd_conf.c\
./src/usb.c\
./src/main.c\
./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_usb.c\
./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c\
./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c\
./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd.c\
./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd_ex.c\
./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c\
./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c\
./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c\
./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c\
./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c\
./stm32libs/STM32CubeF0/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c\
./stm32libs/syscalls.c\
./stm32libs/sysmem.c\
./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c\
./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c\

ASM_SOURCES := ./stm32libs/STM32CubeF0/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f042x6.s

BOARD_CFG := ./cfg/
STM_LIBS := ./stm32libs/ 
HAL_INC := ./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Inc/
HAL_SRC := ./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Src/
CMSIS_INC := ./stm32libs/STM32CubeF0/Drivers/CMSIS/Include/
CMSIS_DEVICE_LIBS := ./stm32libs/STM32CubeF0/Drivers/CMSIS/Device/ST/STM32F0xx/Include/
USB_INC := ./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/
USB_DEVICE_LIBS := ./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Src/
CDC_USB_INC := ./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/
CDC_USB_DEVICE_LIBS := ./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/
LINK_SCRIPT := ./stm32libs/STM32F042C6Tx_FLASH.ld

INCLUDE_DIR := $(SRC_DIR)\
			 $(BOARD_CFG)\
			 $(STM_LIBS)\
			 $(HAL_INC)\
			 $(HAL_SRC)\
			 $(CMSIS_INC)\
			 $(CMSIS_DEVICE_LIBS)\
			 $(USB_INC)\
			 $(USB_DEVICE_LIBS)\
			 $(CDC_USB_INC)\
			 $(CDC_USB_DEVICE_LIBS)\

INCLUDE_FLAGS += $(addprefix -I,$(INCLUDE_DIR))
WERROR_FLAGS = -Wimplicit-function-declaration -Wall -Wextra #enable all the warnings+more
# gc-sectionss here helps get rid region overflow errors in flash 
# what does map do here?
LDFLAGS = -T$(LINK_SCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
CFLAGS = -specs=nano.specs $(CPU) $(INCLUDE_FLAGS) $(WERROR_FLAGS) -g -gdwarf-2 -fdata-sections -ffunction-sections -MMD -MP -MF"$(@:%.o=%.d)"
OBJECTS += $(patsubst %.c,$(BUILD_DIR)/%.o,$(notdir $(C_SOURCES)))
DEPS := $(OBJECTS:.o=.d)
TARGET := main
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
VPATH := $(sort $(dir $(C_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	cp $(BUILD_DIR)/$(TARGET).bin $(TARGET).bin

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)
#
# $(TARGET): $(OBJECTS)
# 	$(CC) $^ -o $@ -T$(LINK_SCRIPT) $(INCLUDE_FLAGS)

# -MMD: build dependency list of header files as .d, 
# -MP: Adds a dummy rule for each header file listed in the dependency file other wise it becomes stale
