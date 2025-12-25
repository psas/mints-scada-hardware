Firmware Rebuild

Microcontroller Sub Family: STM32F042X6
Exact Microcontroller: STM32F042C6, 
Footprint: LQFP

Board Config Template for ./cfg/stm32f0xx_hal_conf.h  comes from
./stm32libs/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_conf_template.h

Board Config ./cfg/stm32f0xx_hal_conf.h comes from
./stm32libs/STM32CubeF0/Drivers/CMSIS/Device/ST/STM32F0xx/Include/stm32f042x6.h

Board Config Template for ./cfg/usbd_conf.c  comes from
./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_conf_template.c

Board Config ./cfg/usbd_conf.h comes from
./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_conf_template.h

Board Config Template for ./cfg/usbd_cdc_if.h comes from
./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/usbd_cdc_if_template.h

Board Config Template for ./cfg/usbd_cdc_if.c comes from
./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc_if_template.c

Board Config Template for ./cfg/usbd_desc.c comes from
./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_desc_template.c

Board Config Template for ./cfg/usbd_desc.h comes from
./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_desc_template.h

>[IMPORTANT]
> I changed some code in the following submodules libraries locally in order to fix some build errors.
> The changes could be wrong as I am not 100% sure about them.
> Generally seems like STM32 is not keeping up with their published code on
> github.

In `./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c`
```
 #include "usbd_cdc.h"
 #include "usbd_ctlreq.h"
+#include "stm32f0xx_hal_pcd.h"
```

In `./stm32libs/STM32CubeF0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_core.h`
```
+#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
```
