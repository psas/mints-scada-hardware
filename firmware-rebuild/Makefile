#project name
PROJECT = firmware-rebuild

#cpu arch/type
CPU = -mcpu=cortex-m0 -mthumb

#TODO: setup build directory (.o, .elf, .d, and etc)
BUILD_DIR := ./build
SRC_DIR := ./src

BOARD_CFG := ./cfg/
STM_LIBS := ./stm32libs/ 
HAL_INC := ./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Inc/
HAL_SRC := ./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Src/
CMSIS_INC := ./stm32libs/STM32CubeF0/Drivers/CMSIS/Include
CMSIS_DEVICE_LIBS := ./stm32libs/STM32CubeF0/Drivers/CMSIS/Device/ST/STM32F0xx/Include/
ASM_STARTUP := ./stm32libs/STM32CubeF0/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f042x6.s

INCLUDE_DIR = $(BOARD_CFG)\
			 $(STM_LIBS)\
			 $(HAL_INC)\
			 $(HAL_SRC)\
			 $(CMSIS_INC)\
			 $(CMSIS_DEVICE_LIBS)\

INCLUDE_FLAGS += $(addprefix -I,$(INCLUDE_DIR))
CFLAGS = $(CPU) $(INCLUDE_FLAGS)

main: 
	arm-none-eabi-gcc $(SRC_DIR)/main.c $(CFLAGS)


