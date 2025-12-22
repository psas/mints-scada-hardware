typedef enum {
  ADC,
  I2C,
  ValveCtl,
} Build;

// initGen will init everything will need to be initialized no matter the build
int initGen(){
  //TODO: initSPI
  //TODO: initGPIO
  
  detectBuild();
}

GPIO_PinState addr1 = HAL_GPIO_ReadPin(*ADDR1_GPIO_Port, ADDR1_Pin);
GPIO_PinState addr2 = HAL_GPIO_ReadPin(*ADDR2_GPIO_Port, ADDR2_Pin);
GPIO_PinState addr4 = HAL_GPIO_ReadPin(*ADDR4_GPIO_Port, ADDR4_Pin); 
GPIO_PinState addr8 = HAL_GPIO_ReadPin(*ADDR8_GPIO_Port, ADDR8_Pin); 

int initCAN(){
  if (good){
    return 1;
  }
  if (notgood){
    return 0
  }
}
int initI2C(){
  if (good){
    return 1;
  }
  if (notgood){
    return 0
  }
}
int initADC(){
 if (good){
    return 1;
  }
  if (notgood){
    return 0
  }
}
int initValveCtl(){
 if (good){
    return 1;
  }
  if (notgood){
    return 0
  }
}

Build detectBuild(){
// TODO: Error detection for multiple high pins
  if (addr1 = GPIO_PIN_SET){
    return (ADC);
  }
  if (addr2 = GPIO_PIN_SET){
    return (I2C);
  }
  if (addr4 = GPIO_PIN_SET){
    return (ValveCtl);
  }
  return Build;
}
