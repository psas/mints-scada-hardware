/*
* Library to communicate with the Microchip MCP3461/2/4 Two/Four/Eight-Channel,
* 153.6 ksps, Low Noise, 16-Bit Delta-Sigma ADC
* https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP3461-2-4-Two-Four-Eight-Channel-153.6-ksps-Low-Noise-16-Bit-Delta-Sigma-ADC-Data-Sheet-20006180D.pdf
*/

#ifndef MCP346X_H
#define MCP346X_H

#include "spi.h"

typedef struct MCP346x_t {
    SPI_HandleTypeDef* hspi;
    // CS is active low
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    // EN is active high
    GPIO_TypeDef* en_port;
    uint16_t en_pin;
    uint8_t reg[18];
    double ref;
} MCP346x;

MCP346x MCP346x_Init(SPI_HandleTypeDef *spi, GPIO_TypeDef* cs_port, uint16_t cs_pin, GPIO_TypeDef* en_port, uint16_t en_pin);

uint8_t MCP346x_sendCmd(const MCP346x* adc, const uint8_t fastcmd);
uint8_t MCP346x_readReg(const MCP346x* adc, const uint8_t reg);
uint8_t MCP346x_readRegs(const MCP346x* adc, const uint8_t reg, uint8_t* result, const int count);
uint8_t MCP346x_writeReg(MCP346x* adc, const uint8_t reg, uint8_t value);
uint8_t MCP346x_writeRegs(MCP346x* adc, const uint8_t reg, const uint8_t* values, const int count);
void MCP346x_setValue(MCP346x* adc, const uint8_t reg, const uint8_t value, const uint8_t mask);

void MCP346x_startADC(MCP346x* adc, const uint8_t vp, const uint8_t vs, const uint8_t gain);
int16_t MCP346x_readADC(const MCP346x* adc);
int16_t MCP346x_analogRead(MCP346x* adc, const uint8_t vp, const uint8_t vs, const uint8_t gain);
double MCP346x_convertVoltage(const MCP346x* adc, int16_t reading, uint8_t gain);
void MCP346x_printRegs(const MCP346x* adc);
void MCP346x_printInfo(const MCP346x* adc);

int SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size,
                        uint32_t Timeout);

#define PACK_COMMAND(reg, cmd) 0b01 << 6 | reg << 2 | cmd

/** Fast commands, take no args */
#define CMD_TYPE_FAST 0b00
/** Read a single address */
#define CMD_TYPE_READ 0b01
/** Write to sequential addresses */
#define CMD_TYPE_WRITE_MANY 0b10
/** Read from sequential addresses */
#define CMD_TYPE_READY_MANY 0b11

/** ADC Conversion Start/Restart Fast Command (Overwrites ADC_MODE[1:0] = 11) */
#define CMD_FAST_GO 0b1010
/** ADC Standby Mode Fast Command (Overwrites ADC_MODE[1:0] = 10) */
#define CMD_FAST_STANDBY 0b1011
/** ADC Shutdown Mode Fast Command (Overwrites ADC_MODE[1:0] = 00) */
#define CMD_FAST_SHUTDOWN 0b1100
/** Full Shutdown Mode Fast Command (Overwrites CONFIG0[7:0] = 0x00) */
#define CMD_FAST_FULL_SHUTDOWN 0b1101
/** Device Full Reset Fast Command (Resets Whole Register Map to Default Value) */
#define CMD_FAST_RESET 0b1110

/** Latest A/D conversion data output value  */
#define REG_ADCDATA 0x0
/** ADC Operating mode, Master Clock mode and Input Bias Current Source mode */
#define REG_CONFIG0 0x1
/** Prescale and OSR settings */
#define REG_CONFIG1 0x2
/** ADC boost and gain settings, auto-zeroing settings for analog multiplexer, voltage reference and ADC */
#define REG_CONFIG2 0x3
/** Conversion mode, data and CRC format settings */
#define REG_CONFIG3 0x4
/** IRQ Status bits and IRQ mode settings, enable for Fast commands and for conversion start pulse */
#define REG_IRQ 0x5
/** Analog multiplexer input selection */
#define REG_MUX 0x6
/** SCAN mode settings */
#define REG_SCAN 0x7
/** Delay value for TIMER between each SCAN cycle */
#define REG_TIMER 0x8
/** ADC digital offset calibration value */
#define REG_OFFSETCAL 0x9
/** ADC digital gain calibration value */
#define REG_GAINCAL 0xA
// 0xB and 0xC reserved
/** Password value for SPI Write mode locking */
#define REG_LOCK 0xD
// 0xE reserved
/** CRC checksum for the device configuration */
#define REG_CRCCFG 0xF

/*
 * CONFIG0
 */
/** CONFIG0 Full Shutdown */
#define CONFIG0_FULL_SHUTDOWN 0b00 << 6
/** CONFIG0 Awake */
#define CONFIG0_AWAKE 0b11 << 6

/** CONFIG0 Using and outputting internal clock */
#define CLK_SEL_INTERNAL_OUT 0b11 << 4
/** CONFIG0 Using internal clock, not outputting */
#define CLK_SEL_INTERNAL  0b10 << 4
// 0x01 is also external
/** CONFIG0 Using external clock (default) */
#define CLK_SEL_EXTERNAL 0b00 << 4
/** CONFIG0 Clock source mask */
#define CLK_SEL_MASK 0b11 << 4

/** CONFIG0 15uA bias current */
#define BIAS_150 0b11 << 2
/** CONFIG0 3.7uA bias current */
#define BIAS_37 0b10 << 2
/** CONFIG0 0.9uA bias current */
#define BIAS_90 0b01 << 2
/** CONFIG0 Bias current off (default) */
#define BIAS_OFF 0b00 << 2
/** CONFIG0 Bias mask */
#define BIAS_MASK 0b11 << 2;

/** CONFIG0 ADC Conversion mode */
#define MODE_CONVERSION 0b11
/** CONFIG0 ADC Standby mode */
#define MODE_STANDBY 0b10
// 0b01 is also shutdown
/** CONFIG0 ADC Shutdown mode (default) */
#define MODE_SHUTDOWN 0b00
/** CONFIG0 mode mask */
#define MODE_MASK 0b11;

/*
 * CONFIG1
 */
/** CONFIG1 AMCLK = MCLK/8 */
#define PRESCALER_8 0b11 << 6
/** CONFIG1 AMCLK = MCLK/4 */
#define PRESCALER_4 0b10 << 6
/** CONFIG1 AMCLK = MCLK/2 */
#define PRESCALER_2 0b01 << 6
/** CONFIG1 AMCLK = MCLK (default) */
#define PRESCALER_1 0b00 << 6
/** CONFIG1 prescaler mask */
#define PRESCALER_MASK 0b11 << 6;

/** CONFIG1 98304 oversampling ratio */
#define OSR_98304 0b1111 << 2
/** CONFIG1 81920 oversampling ratio */
#define OSR_81920 0b1110 << 2
/** CONFIG1 49152 oversampling ratio */
#define OSR_49152 0b1101 << 2
/** CONFIG1 40960 oversampling ratio */
#define OSR_40960 0b1100 << 2
/** CONFIG1 24576 oversampling ratio */
#define OSR_24576 0b1011 << 2
/** CONFIG1 20480 oversampling ratio */
#define OSR_20480 0b1010 << 2
/** CONFIG1 16384 oversampling ratio */
#define OSR_16384 0b1001 << 2
/** CONFIG1 8192 oversampling ratio */
#define OSR_8192 0b1000 << 2
/** CONFIG1 4096 oversampling ratio */
#define OSR_4096 0b0111 << 2
/** CONFIG1 2048 oversampling ratio */
#define OSR_2048 0b0110 << 2
/** CONFIG1 1024 oversampling ratio */
#define OSR_1024 0b0101 << 2
/** CONFIG1 512 oversampling ratio */
#define OSR_512 0b0100 << 2
/** CONFIG1 256 oversampling ratio (default) */
#define OSR_256 0b0011 << 2
/** CONFIG1 128 oversampling ratio */
#define OSR_128 0b0010 << 2
/** CONFIG1 64 oversampling ratio */
#define OSR_64 0b0001 << 2
/** CONFIG1 32 oversampling ratio */
#define OSR_32 0b0000 << 2
/** CONFIG1 oversampling mask */
#define OSR_MASK 0b1111 << 2

/*
 * CONFIG2
 */
/** CONFIG2 ADC channel has current x 2 */
#define BOOST_2 0b11 << 6
/** CONFIG2 ADC channel has current x 1 (default) */
#define BOOST_1 0b10 << 6
/** CONFIG2 ADC channel has current x 0.66 */
#define BOOST_066 0b01 << 6
/** CONFIG2 ADC channel has current x 0.5 */
#define BOOST_05 0b00 << 6
/** CONFIG2 ADC boost mask */
#define BOOST_MASK 0b11 << 6

/** CONFIG2 Gain is x64 (x16 analog, x4 digital) */
#define GAIN_64 0b111
/** CONFIG2 Gain is x32 (x16 analog, x2 digital) */
#define GAIN_32 0b110
/** CONFIG2 Gain is x16 */
#define GAIN_16 0b101
/** CONFIG2 Gain is x8 */
#define GAIN_8 0b100
/** CONFIG2 Gain is x4 */
#define GAIN_4 0b011
/** CONFIG2 Gain is x2 */
#define GAIN_2 0b010
/** CONFIG2 Gain is x1 (default) */
#define GAIN_1 0b001
/** CONFIG2 Gain is x1/3 */
#define GAIN_03 0b000
/** CONFIG2 Gain mask */
#define GAIN_MASK 0b111 << 3

/** CONFIG2 ADC auto-zeroing algorithm is enabled */
#define MUX_ZERO_ON 0b1 << 2
/** CONFIG2 Analog input multiplexer auto-zeroing algorithm is disabled (default) */
#define MUX_ZERO_OFF 0b0 << 2
/** CONFIG2 Analog mux auto-zero mask */
#define MUX_MASK 0b11 << 2

/*
 * CONFIG3
 */
/** CONFIG3 Continuous Conversion mode */
#define CONV_MODE_CONTINUOUS 0b11 << 6
/** CONFIG3 One-shot conversion, sets ADC_MODE[1:0] to ‘10’ (Standby) at the end of the conversion */
#define CONV_MODE_ONESHOT_STANDBY 0b10 << 6
/** CONFIG3 One-shot conversion, sets ADC_MODE[1:0] to ‘0x’ (ADC Shutdown) at the end of the conversion (default)*/
#define CONV_MODE_ONESHOT_SHUTDOWN 0b00 << 6
/** CONFIG3 Conversion mode mask */
#define  CONV_MODE_MASK 0b11 << 6

/** CONFIG3 32-bit (17-bit right justified data plus Channel ID): CHID[3:0] plus SGN extension (12 bits) plus 16-bit
 *  ADC data; allows overrange with the SGN extension */
#define DATA_FORMAT_32_CHID 0b11 << 5
/** CONFIG3 32-bit (17-bit right justified data): SGN extension (16-bit) plus 16-bit ADC data; allows overrange with
 *  the SGN extension */
#define DATA_FORMAT_32 0b10 << 5
/** CONFIG3 32-bit (16-bit left justified data): 16-bit ADC data plus 0x0000 (16-bit); does not allow overrange (ADC
 *  code locked to 0xFFFF or 0x8000 */
#define DATA_FORMAT_32_PACK 0b01 << 5
/** CONFIG3 16-bit (default ADC coding): 16-bit ADC data; does not allow overrange (ADC code locked to 0xFFFF
 *  or 0x8000) (default) */
#define DATA_FORMAT_16 0b00 << 5
/** CONFIG3 Data format mask */
#define DATA_FORMAT_MASK 0b11 << 5

/** CONFIG3 CRC-16 followed by 16 zeros (32-bit format) */
#define CRC_FORMAT_32 0b1 << 3
/** CONFIG3 CRC-16 only (16-bit format) (default) */
#define CRC_FORMAT_16 0b0 << 3
/** CONFIG3 CRC Format Mask */
#define CRC_FORMAT_MASK 0b11 << 3

/** CONFIG3 CRC on communication enabled */
#define CRC_ON_READ_ENABLED 0b1 << 2
/** CONFIG3 CRC on communication disabled (default) */
#define CRC_ON_READ_DISABLED 0b0 << 2
/** CONFIG3 CRC on read mask */
#define CRC_ON_READ_MASK 0b11 << 2

/** CONFIG3 Digital Offset Calibration enabled */
#define DIGITAL_OFFSET_CALIB_ENABLED 0b1 << 1
/** CONFIG3 Digital Offset Calibration disabled (default) */
#define DIGITAL_OFFSET_CALIB_DISABLED 0b0 << 1
/** CONFIG3 Digital Offset Calibration Mask */
#define DIGITAL_OFFSET_CALIB_MASK 0b11 << 1

/** CONFIG3 Gain Calibration enabled */
#define GAIN_CALIB_ENABLED 0b1
/** CONFIG3 Gain Calibration disabled (default) */
#define GAIN_CALIB_DISABLED 0b0
/** CONFIG3 Gain calib mask */
#define GAIN_CALIB_MASK 0b11

/*
 * IRQ
 */
/** IRQ Data Ready Status Flag; active low */
#define IRQ_DATA_READY 0b1 << 6
/** IRQ CRC Error Status Flag Bit for Internal Registers; active low */
#define IRQ_CFC_ERROR 0b1 << 5
/** IRQ POR status flag; active low */
#define IRQ_POR 0v1 << 4

/** IRQ MDAT output is selected; only POR and CRC interrupts can be present on this pin and take priority
over the MDAT output */
#define IRQ_PIN_MODE_MDAT 0b1 << 3
/** IRQ IRQ output is selected; all interrupts can appear on the IRQ/MDAT pin; active low (default) */
#define IRQ_PIN_MODE_IRQ 0b0 << 3
/** IRQ IRQ output selection mask */
#define IRQ_PIN_MODE_MASK 0b1 << 3

/** IRQ The Inactive state of the IRQ pin is logic high (does not require a pull-up resistor to DV DD) */
#define IRQ_PIN_INACTIVE_LOGIC 0b1 << 2
/** IRQ The Inactive state of the IRQ pin is High-Z (requires a pull-up resistor to DV DD) (default) */
#define IRQ_PIN_INACTIVE_HIGHZ 0b0 << 2
/** IRQ Inactivate state mask */
#define IRQ_PIN_INACTIVE_MASK 0b1 << 2

/** IRQ Fast commands are enabled (default) */
#define FAST_COMMANDS_ENABLED 0b1 << 1
/** IRQ Fast commands are disabled */
#define FAST_COMMANDS_DISABLED 0b0 << 1
/** IRQ Fast Command Mask */
#define FAST_COMMANDS_MASK 0b1 << 1

/** IRQ Conversion Start Interrupt Output enabled (default) */
#define CONVERSION_START_INTERRUPT_ENABLED 0b1
/** IRQ Conversion Start Interrupt Output disabled */
#define CONVERSION_START_INTERRUPT_DISABLED 0b0
/** IRQ Conversion start interrupt mask */
#define CONVERSION_START_INTERRUPT_MASK 0b1

/*
 * MUX
 */
/** MUX from Internal VCM */
#define MUX_VCM 0b1111
/** MUX from Internal Temperature Sensor Diode M (TEMP Diode M) */
#define MUX_TEMP_M 0b1110
/** MUX from Internal Temperature Sensor Diode P (TEMP Diode P) */
#define MUX_TEMP_P 0b1101
/** MUX from REFIN- */
#define MUX_REFN 0b1100
/** MUX from REFIN+ */
#define MUX_REFP 0b1011
/** MUX from AVDD */
#define MUX_AVDD 0b1001
/** MUX from AGND */
#define MUX_AGND 0b1000
/** MUX from CH7 */
#define MUX_CH7 0b0111
/** MUX from CH6 */
#define MUX_CH6 0b0110
/** MUX from CH5 */
#define MUX_CH5 0b0101
/** MUX from CH4 */
#define MUX_CH4 0b0100
/** MUX from CH3 */
#define MUX_CH3 0b0011
/** MUX from CH2 */
#define MUX_CH2 0b0010
/** MUX from CH1 */
#define MUX_CH1 0b0001
/** MUX from CH0 (default) */
#define MUX_CH0 0b0000

/*
 * SCAN
 */
/** SCAN Scan conversion delay 512 * DMCLK */
#define SCAN_DELAY_512 0b111 << 21
/** SCAN Scan conversion delay 256 * DMCLK */
#define SCAN_DELAY_256 0b110 << 21
/** SCAN Scan conversion delay 128 * DMCLK */
#define SCAN_DELAY_128 0b101 << 21
/** SCAN Scan conversion delay 64 * DMCLK */
#define SCAN_DELAY_64  0b100 << 21
/** SCAN Scan conversion delay 32 * DMCLK */
#define SCAN_DELAY_32  0b011 << 21
/** SCAN Scan conversion delay 16 * DMCLK */
#define SCAN_DELAY_16  0b010 << 21
/** SCAN Scan conversion delay 8 * DMCLK */
#define SCAN_DELAY_8   0b001 << 21
/** SCAN Scan conversion delay 0: No Delay (default) */
#define SCAN_DELAY_0   0b000 << 21
/** SCAN Scan delay mask */
#define SCAN_DELAY_MASK 0b111 << 21

/** SCAN Enable scan on OFFSET mux=0x88 */
#define SCAN_ENABLE_VCM 0b1 << 14
/** SCAN Enable scan on VCM mux=0xF8,  gain=1 */
#define SCAN_ENABLE_AVDD 0b1 << 13
/** SCAN Enable scan on AVDD mux=0x98, gain=0.33 */
#define SCAN_ENABLE_TEMP 0b1 << 12
/** SCAN Enable scan on TEMP mux=0xDE, gain=1 */
#define SCAN_ENABLE_DIFF_CHD 0b1 << 11
/** SCAN Enable scan on Differential Channel D (CH6-CH7) mux=0x67 */
#define SCAN_ENABLE_DIFF_CHC 0b1 << 10
/** SCAN Enable scan on Differential Channel C (CH4-CH5) mux=0x45 */
#define SCAN_ENABLE_DIFF_CHB 0b1 << 9
/** SCAN Enable scan on Differential Channel B (CH2-CH3) mux=0x23 */
#define SCAN_ENABLE_SING_CHA 0b1 << 8
/** SCAN Enable scan on Differential Channel A (CH0-CH1) mux=0x01 */
#define SCAN_ENABLE_SING_CH7 0b1 << 7
/** SCAN Enable scan on Single-Ended Channel CH7 mux=0x78 */
#define SCAN_ENABLE_SING_CH6 0b1 << 6
/** SCAN Enable scan on Single-Ended Channel CH6 mux=0x68 */
#define SCAN_ENABLE_SING_CH5 0b1 << 5
/** SCAN Enable scan on Single-Ended Channel CH5 mux=0x58 */
#define SCAN_ENABLE_SING_CH4 0b1 << 4
/** SCAN Enable scan on Single-Ended Channel CH4 mux=0x48 */
#define SCAN_ENABLE_SING_CH3 0b1 << 3
/** SCAN Enable scan on Single-Ended Channel CH3 mux=0x38 */
#define SCAN_ENABLE_SING_CH2 0b1 << 2
/** SCAN Enable scan on Single-Ended Channel CH2 mux=0x28 */
#define SCAN_ENABLE_SING_CH1 0b1 << 1
/** SCAN Enable scan on Single-Ended Channel CH1 mux=0x18 */
#define SCAN_ENABLE_SING_CH0 0b1 << 0

#define STATUS_DATA_READY 0b1 << 2
#define STATUS_CFC_ERROR 0b1 << 1
#define STATUS_POR 0b1 << 0

#endif //MCP346X_H
