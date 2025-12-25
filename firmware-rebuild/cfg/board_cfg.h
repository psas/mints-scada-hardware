// // Configuration: Select exactly one. Selecting multiple may lead to issues
// #define CONFIG_OUTPUTS
// // #define CONFIG_ADC
// // #define CONFIG_I2C
//
// // Check that exactly 1 output configuration has been selected
// #if !(defined(CONFIG_OUTPUTS) || (defined(CONFIG_ADC) || defined(CONFIG_I2C))) \
//     || (defined(CONFIG_OUTPUTS) && (defined(CONFIG_ADC) || defined(CONFIG_I2C))) \
//     || (defined(CONFIG_ADC) && defined(CONFIG_I2C))
//     #error("You must specify exactly 1 of CONFIG_OUTPUTS, CONFIG_ADC, or CONFIG_I2C")
// #endif
//
// #ifdef CONFIG_OUTPUTS
// #define BASE_ADDR_OFFSET 0
// #endif
// #ifdef CONFIG_ADC
// #define BASE_ADDR_OFFSET 8
// #endif
// #ifdef CONFIG_I2C
// #define BASE_ADDR_OFFSET 12
// #endif
// testing use

void SystemClock_Config(void);
#define BASE_ADDR_OFFSET 12

// Pin definitions //

//PA is in GPIOA & PB is in GPIOB
//See PCB layout as to why they are ordered in this manner

#define ADDR4_Pin GPIO_PIN_4
#define ADDR1_Pin GPIO_PIN_5
#define ADDR8_Pin GPIO_PIN_6
#define ADDR2_Pin GPIO_PIN_7
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA
