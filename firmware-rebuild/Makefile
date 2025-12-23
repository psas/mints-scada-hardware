#project name
PROJECT = firmware-rebuild

#cpu arch/type
CPU = -mcpu=cortex-m0 -mthumb

#TODO: setup build directory (.o, .elf, .d, and etc)
# and set up references to the necessary sources (e.g syscalls.c)
BUILD_DIR := ./build
SRC_DIR := ./src

BOARD_CFG := ./cfg/
STM_LIBS := ./stm32libs/ 
HAL_INC := ./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Inc/
HAL_SRC := ./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Src/
CMSIS_INC := ./stm32libs/STM32CubeF0/Drivers/CMSIS/Include
CMSIS_DEVICE_LIBS := ./stm32libs/STM32CubeF0/Drivers/CMSIS/Device/ST/STM32F0xx/Include/
USB_INC := ./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/
USB_DEVICE_LIBS := ./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Src/
CDC_USB_INC := ./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/
CDC_USB_DEVICE_LIBS := ./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/
ASM_STARTUP := ./stm32libs/STM32CubeF0/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f042x6.s

INCLUDE_DIR = $(BOARD_CFG)\
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
WERROR_FLAGS = -Wimplicit-function-declaration
CFLAGS = $(CPU) $(INCLUDE_FLAGS) $(WERROR_FLAGS)

main: 
	arm-none-eabi-gcc $(SRC_DIR)/main.c $(CFLAGS)


