#project name
PROJECT = firmware-rebuild

#cpu arch/type
CPU = cortex-m0

BUILD_DIR := ./build
SRC_DIR := ./src

#ASM_STARTUP = startup_stm32f042x6.s

HAL_INC := ./stm32libs/stm32f0xx-hal-driver/Inc/
HAL_SRC := ./stm32libs/stm32f0xx-hal-driver/Src/
STM_LIBS := ./stm32libs/ 
BOARD_CFG := ./cfg/

FILE = $(SRC_DIR)/main.c

#ASM_STARTUP = startup_stm32f042x6.s

INCLUDE_DIR = $(BOARD_CFG)\
			 $(HAL_INC)\
			 $(HAL_SRC)\
			 $(STM_LIBS)
			 # $(CMSIS_SRC)\
			 

INCLUDE_FLAGS += $(addprefix -I,$(INCLUDE_DIR))
CFLAGS = -mcpu=$(CPU) $(INCLUDE_FLAGS)

edit: 
	arm-none-eabi-gcc $(FILE) $(CFLAGS)


