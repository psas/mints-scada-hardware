void SystemClock_Config(void);
void Error_Handler(void);

// Pin definitions //

// PA is in GPIOA & PB is in GPIOB
// See PCB layout as to why they are ordered in this manner

#define ADDR4_Pin GPIO_PIN_4
#define ADDR1_Pin GPIO_PIN_5
#define ADDR8_Pin GPIO_PIN_6
#define ADDR2_Pin GPIO_PIN_7
#define ADDR_GPIO_PORT GPIOA

#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA

#define SPI_PORT GPIOB
#define SPI_CS GPIO_PIN_12
#define SPI_SCK GPIO_PIN_13
#define SPI_MISO GPIO_PIN_14
#define SPI_MOSI GPIO_PIN_15

#define CONFIG0_Pin GPIO_PIN_0
#define CONFIG0_GPIO_Port GPIOA
#define CONFIG1_Pin GPIO_PIN_1
#define CONFIG1_GPIO_Port GPIOA

#define ADC_RAND_Pin GPIO_PIN_2
#define ADC_RAND_GPIO_Port GPIOA

#define OUT0_Pin GPIO_PIN_0
#define OUT0_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_1
#define OUT1_GPIO_Port GPIOB
#define OUT2_Pin GPIO_PIN_2
#define OUT2_GPIO_Port GPIOB

#define CTRL_Pin GPIO_PIN_12
#define CTRL_GPIO_Port GPIOB

#define OUT3_Pin GPIO_PIN_3
#define OUT3_GPIO_Port GPIOB
#define OUT4_Pin GPIO_PIN_4
#define OUT4_GPIO_Port GPIOB
#define OUT5_Pin GPIO_PIN_5
#define OUT5_GPIO_Port GPIOB
#define OUT6_Pin GPIO_PIN_6
#define OUT6_GPIO_Port GPIOB
#define OUT7_Pin GPIO_PIN_7
#define OUT7_GPIO_Port GPIOB

#define USART2_CTS GPIO_PIN_0
#define USART2_RTS GPIO_PIN_1
#define USART2_TX GPIO_PIN_2
#define USART2_RX GPIO_PIN_3
#define USART2_CK GPIO_PIN_4
#define USART2_GPIO_PORT GPIOA
