#ifndef CONFIGURATION_H
#define CONFIGURATION_H
// CAN loopback mode. Use ONLY for firmware development
// #define LOOPBACK

// Configuration: Select exactly one. Selecting multiple may lead to issues
// #define CONFIG_OUTPUTS
#define CONFIG_ADC
// #define CONFIG_I2C

// Check that exactly 1 output configuration has been selected
#if !(defined(CONFIG_OUTPUTS) || (defined(CONFIG_ADC) || defined(CONFIG_I2C))) \
    || (defined(CONFIG_OUTPUTS) && (defined(CONFIG_ADC) || defined(CONFIG_I2C))) \
    || (defined(CONFIG_ADC) && defined(CONFIG_I2C))
    #error("You must specify exactly 1 of CONFIG_OUTPUTS, CONFIG_ADC, or CONFIG_I2C")
#endif

// BASE_ADDR_OFFSET gets added to the base address read from the rotary encoder
// SUB_ADDR_MASK masks the sub address bits that must match the address in order to be counted as a match

#ifdef CONFIG_OUTPUTS
#define BASE_ADDR_OFFSET 0
#define SUB_ADDR_MASK 0x8
#endif
#ifdef CONFIG_ADC
#define BASE_ADDR_OFFSET 8
#define SUB_ADDR_MASK 0xC
#endif
#ifdef CONFIG_I2C
#define BASE_ADDR_OFFSET 12
#define SUB_ADDR_MASK 0xC
#endif
#endif // CONFIGURATION_H
