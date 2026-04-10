# Microcontroller Information
Microcontroller Sub Family: STM32F042X6
Exact Microcontroller: STM32F042C6,
Footprint: LQFP48

# Build Instructions

In order to build a binary we use GNU Make.
While in the `./firmware-rebuild/` directory:

* Run `make` to build the binary and elf file

~* Run `make flash` to flash the binary onto the microcontroller using DFU~ (USB was removed)

* Run `make write` to flash the elf onto the microcontroller using openocd then immediately close the remote server

* Run `make debug` to flash the elf onto the microcontroller using openocd to keep the server open

# Externally Sourced Files

Board Config Template for `./cfg/stm32f0xx_hal_conf.h`  comes from
`./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_conf_template.h`

Board Config `./cfg/stm32f0xx_hal_conf.h` comes from
`./stm32libs/STM32CubeF0/Drivers/CMSIS/Device/ST/STM32F0xx/Include/stm32f042x6.h`

