// CAN loopback mode. Use ONLY for firmware development
// #define LOOPBACK

// Configuration: Select exactly one. Selecting multiple may lead to issues
#define CONFIG_OUTPUTS
// #define CONFIG_ADC
// #define CONFIG_I2C

// Check that exactly 1 output configuration has been selected
#if !(defined(CONFIG_OUTPUTS) || (defined(CONFIG_ADC) || defined(CONFIG_I2C))) \
    || (defined(CONFIG_OUTPUTS) && (defined(CONFIG_ADC) || defined(CONFIG_I2C))) \
    || (defined(CONFIG_ADC) && defined(CONFIG_I2C))
    #error("You must specify exactly 1 of CONFIG_OUTPUTS, CONFIG_ADC, or CONFIG_I2C")
#endif