typedef enum {
  ADC_conf,
  I2C_conf,
  ValveCtl_conf,
  Error_conf,
} Build;

void initGen(void);
void initADDR_GPIO(void);
void initCAN(void);
