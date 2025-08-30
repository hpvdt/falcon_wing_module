#ifndef ADS131M03_HEADER
#define ADS131M03_HEADER

#include <stdbool.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"

enum ADSRegisterAddress {
  ADS_REG_ID            = 0x00,
  ADS_REG_STATUS        = 0x01,
  ADS_REG_MODE          = 0x02,
  ADS_REG_CLOCK         = 0x03,
  ADS_REG_GAIN          = 0x04,
  ADS_REG_CFG           = 0x06,
  ADS_REG_THRSHLD_MSB   = 0x07,
  ADS_REG_THRSHLD_LSB   = 0x08,
  ADS_REG_CH0_CFG       = 0x09,
  ADS_REG_CH0_OCAL_MSB  = 0x0A,
  ADS_REG_CH0_OCAL_LSB  = 0x0B,
  ADS_REG_CH0_GCAL_MSB  = 0x0C,
  ADS_REG_CH0_GCAL_LSB  = 0x0D,
  ADS_REG_CH1_CFG       = 0x0E,
  ADS_REG_CH1_OCAL_MSB  = 0x0F,
  ADS_REG_CH1_OCAL_LSB  = 0x10,
  ADS_REG_CH1_GCAL_MSB  = 0x11,
  ADS_REG_CH1_GCAL_LSB  = 0x12,
  ADS_REG_CH2_CFG       = 0x13,
  ADS_REG_CH2_OCAL_MSB  = 0x14,
  ADS_REG_CH2_OCAL_LSB  = 0x15,
  ADS_REG_CH2_GCAL_MSB  = 0x16,
  ADS_REG_CH2_GCAL_LSB  = 0x17,
  ADS_REG_REGMAP_CRC    = 0x3E
};

enum ADSCommand{
  ADS_CMD_NULL      = 0x0000,
  ADS_CMD_RESET     = 0x0011,
  ADS_CMD_STANDBY   = 0x0022,
  ADS_CMD_WAKEUP    = 0x0033,
  ADS_CMD_LOCK      = 0x0555,
  ADS_CMD_UNLOCK    = 0x0666,
  ADS_CMD_RREG      = 0xA000,
  ADS_CMD_WREG      = 0x6000
};

enum ADSWordFormat{
  ADS_WORD_SHORT    = 0b00,
  ADS_WORD_NORMAL   = 0b01,
  ADS_WORD_ZERO_PAD = 0b10,
  ADS_WORD_SIGN_PAD = 0b11
};

struct ADSConfig {
	SPI_HandleTypeDef* bus;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
	GPIO_TypeDef *reset_port;
	uint16_t reset_pin;
	GPIO_TypeDef *data_ready_port;
	uint16_t data_ready_pin;
};

enum ADSDataReadyActiveFormat {
	ADS_DRDY_ACTIVE_FRMT_LOW 	= 0,
	ADS_DRDY_ACTIVE_FRMT_PULSE 	= 1,
};

enum ADSDataReadyIdleFormat {
	ADS_DRDY_IDLE_FRMT_HIGH 	= 0,
	ADS_DRDY_IDLE_FRMT_HIZ	 	= 1,
};

enum ADSDataReadySource {
	ADS_DRDY_SRC_LAGGING = 0,
	ADS_DRDY_SRC_ANY = 1,
	ADS_DRDY_SRC_LEADING = 2,
};

enum ADSCRCType {
	ADS_CRC_TYPE_16_BIT_CCITT 	= 0, // x^16 + x^12 + x^5 + 1
	ADS_CRC_TYPE_16_BIT_ANSI	= 1, // x^16 + x^15 + x^2 + 1
};

struct ADSModeRegister {
	enum ADSDataReadyActiveFormat data_ready_available_format : 1;
	enum ADSDataReadyIdleFormat data_ready_unavailable_format : 1;
	enum ADSDataReadySource data_ready_source : 2;
	bool spi_timeout_enabled : 1;
	uint8_t RESERVE_0_A : 3; // Always write 0
	enum ADSWordFormat word_format : 2;
	bool reset : 1; // Write a zero to clear bit in status register
	enum ADSCRCType crc_type : 1;
	bool spi_crc_enable : 1;
	bool register_map_crc_enable : 1;
	uint8_t RESERVE_0_B : 2; // Write zero
};

enum ADSPowerMode {
	ADS_POWER_VERY_LOW = 0,
	ADS_POWER_LOW = 1,
	ADS_POWER_HIGH_RES = 2,
};

enum ADSOSRRatio {
	ADS_OSR_RATIO_128 = 0,
	ADS_OSR_RATIO_256 = 1,
	ADS_OSR_RATIO_512 = 2,
	ADS_OSR_RATIO_1024 = 3,
	ADS_OSR_RATIO_2048 = 4,
	ADS_OSR_RATIO_4096 = 5,
	ADS_OSR_RATIO_8192 = 6,
	ADS_OSR_RATIO_16256 = 7
};

struct ADSClockRegister {
	enum ADSPowerMode power_mode : 2;
	enum ADSOSRRatio osr : 3;
	bool turbo_mode_osr_64 : 1; // Engage OSR of 64, overrides OSR if enabled
	uint8_t RESERVED_0_A : 2; // Write 0
	bool enable_channel_0 : 1;
	bool enable_channel_1 : 1;
	bool enable_channel_2 : 1;
	uint8_t RESERVED_0_B : 5; // Write 0
};

enum ADSPGAGain {
	ADS_PGA_GAIN_1 = 0,
	ADS_PGA_GAIN_2 = 1,
	ADS_PGA_GAIN_4 = 2,
	ADS_PGA_GAIN_8 = 3,
	ADS_PGA_GAIN_16 = 4,
	ADS_PGA_GAIN_32 = 5,
	ADS_PGA_GAIN_64 = 6,
	ADS_PGA_GAIN_128 = 7,
};

struct ADSGainRegister {
	enum ADSPGAGain channel_0_gain : 3;
	uint8_t RESERVE_0_A : 1; // Write 0
	enum ADSPGAGain channel_1_gain : 3;
	uint8_t RESERVE_0_B : 1; // Write 0
	enum ADSPGAGain channel_2_gain : 3;
	uint8_t RESERVE_0_C : 5; // Write 0
};

void ads_begin(struct ADSConfig config);
void ads_reset();

uint16_t ads_command(uint16_t command);
void ads_read_channels(int32_t* destination);
void ads_update_channels();

#endif
