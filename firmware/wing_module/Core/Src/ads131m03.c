#include "ads131m03.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"
#include <stdint.h>
#include <stdbool.h>
#include "wing_module_config.h"


static const int ADS_TIMEOUT_MS = 10;
static struct ADSConfig _config;

volatile static struct ADSModeRegister _ads_mode = {
	.crc_type = ADS_CRC_TYPE_16_BIT_CCITT,
	.data_ready_available_format = ADS_DRDY_ACTIVE_FRMT_LOW,
	.data_ready_source = ADS_DRDY_SRC_LAGGING,
	.data_ready_unavailable_format = ADS_DRDY_IDLE_FRMT_HIGH,
	.register_map_crc_enable = false,
	.reset = 0,
	.spi_crc_enable = false,
	.spi_timeout_enabled = true,
	.word_format = ADS_WORD_NORMAL,
	.RESERVE_0_A = 0,
	.RESERVE_0_B = 0,
};

static struct ADSClockRegister _ads_clock = {
	.power_mode = ADS_POWER_HIGH_RES,
	.osr = ADS_OSR_RATIO_1024,
	.turbo_mode_osr_64 = 0,
	.enable_channel_0 = true,
	.enable_channel_1 = true,
	.enable_channel_2 = true,
	.RESERVED_0_A = 0,
	.RESERVED_0_B = 0,
};

static struct ADSGainRegister _ads_gain = {
	.channel_0_gain = ADS_PGA_GAIN_1,
	.channel_1_gain = ADS_PGA_GAIN_1,
	.channel_2_gain = ADS_PGA_GAIN_1,
	.RESERVE_0_A = 0,
	.RESERVE_0_B = 0,
	.RESERVE_0_C = 0,
};

volatile static int32_t current_reading[3] = {0,0,0};

struct CyclicalBuffer {
	int32_t samples[256];
	int32_t sum; // Must be updated with each sample
	uint_fast8_t current_index;
	uint_fast16_t buffer_length; // Needs to potentially hold 256
	int32_t zero_point;
	float scale;
};

static struct CyclicalBuffer torsion_buffer;
static struct CyclicalBuffer strain_buffer;


void add_sample(uint32_t sample, struct CyclicalBuffer* buffer) {
	int32_t previous_sample = buffer->samples[buffer->current_index];
	int32_t delta = sample - previous_sample;

	buffer->samples[buffer->current_index] = sample;
	buffer->sum = buffer->sum + delta;
	buffer->current_index++;

	if (buffer->current_index >= buffer->buffer_length) buffer->current_index = 0;
}

void clear_buffer(struct CyclicalBuffer* buffer) {
	// Does not affect the buffer length, scale, or zero
	for (int i = 0; i < 256; i++) buffer->samples[i] = 0;
	buffer->current_index = 0;
	buffer->sum = 0;
}


/**
 * @brief Writes a given number of registers from the ADS131M03 chip
 *
 * @param reg Start register for write operation
 * @param numRegs Number of individual registers to write to
 * @param source Source buffer for new register data
 */
void ads_write_reg(enum ADSRegisterAddress reg, uint8_t num_regs, uint16_t *source);

/**
 * @brief Reads a given number of registers from the ADS131M03 chip
 *
 * @param reg Start register for read operation
 * @param numRegs Number of individual registers to read
 * @param destination Destination buffer for received data
 */
void ads_read_reg(enum ADSRegisterAddress reg, uint8_t num_regs, uint16_t *destination);
void ads_update_config_regs();

/**
 * @brief Transfers a word with the ADS131M03
 *
 * @param value Value to pass into ADS131M03
 * @param isADCReading Are we reading an ADC reading (defaults to false)
 * @return Response from ADS131M03 as 32 bit integer
 */
int32_t ads_transfer_single_word(uint16_t value, bool is_adc_reading);

uint16_t ads_calculate_crc(enum ADSCommand command, uint16_t payload[], size_t length) {
	if (!_ads_mode.spi_crc_enable) return 0;

	(void) command;
	(void) payload;
	(void) length;

	if (_ads_mode.crc_type == ADS_CRC_TYPE_16_BIT_CCITT) {
		// x^16 + x^12 + x^5 + 1
	}
	else {
		// 16-bit ANSI calculation
		// x^16 + x^15 + x^2 + 1
	}

	return 0;
}


void ads_begin(struct ADSConfig config) {
	_config = config;

	strain_buffer.buffer_length = 256;
	strain_buffer.scale = 1.0;
	strain_buffer.zero_point = 0;
	clear_buffer(&strain_buffer);
	torsion_buffer.buffer_length = 256;
	torsion_buffer.scale = 1.0;
	torsion_buffer.zero_point = 0;
	clear_buffer(&torsion_buffer);

	ads_reset();

	ads_update_config_regs();
}

void ads_update_config_regs() {
	uint16_t buffer[3];
	// Since the main registers are sequential we can send them in a single burst
	buffer[0] = *(uint16_t*) &_ads_mode;
	buffer[1] = *(uint16_t*) &_ads_clock;
	buffer[2] = *(uint16_t*) &_ads_gain;
	ads_write_reg(ADS_REG_MODE, 3, buffer);

	// Clear the two most recent measurements
	ads_command(ADS_CMD_NULL);
	ads_command(ADS_CMD_NULL);
}


void ads_reset() {
	HAL_GPIO_WritePin(_config.reset_port, _config.reset_pin, GPIO_PIN_RESET);
	HAL_Delay(5); // Needs to be longer than 2048 pulses of MCLK
	HAL_GPIO_WritePin(_config.reset_port, _config.reset_pin, GPIO_PIN_SET);
}

uint16_t ads_command(uint16_t command) {
	uint16_t response = 0;
	uint16_t calculated_crc = ads_calculate_crc(command, NULL, 0);

	HAL_GPIO_WritePin(_config.cs_port, _config.cs_pin, GPIO_PIN_RESET);
	ads_transfer_single_word(command, false);
	// Update the channel data with the most recent data available on each channel
	// Not applicable when issuing a reset command
	if (command != ADS_CMD_RESET) {
		current_reading[0] = ads_transfer_single_word(calculated_crc, true);
		current_reading[1] = ads_transfer_single_word(0, true);
		current_reading[2] = ads_transfer_single_word(0, true);
	}
	else {
		ads_transfer_single_word(calculated_crc, false);
		ads_transfer_single_word(0, false);
		ads_transfer_single_word(0, false);
	}
	ads_transfer_single_word(0, false); // Not bothering with CRC for returns from ADS131 (yet)

	HAL_GPIO_WritePin(_config.cs_port, _config.cs_pin, GPIO_PIN_SET);

	return response;
}

/**
 * @brief Updates channel readings for ADS131M03 to object
 * 
 */
void ads_update_channels() {
	bool data_ready = HAL_GPIO_ReadPin(_config.data_ready_port, _config.data_ready_pin) == GPIO_PIN_RESET;
	if (!data_ready) return; // Data not ready

	// Channels are sent in response to any short command, so using NULL to get them
	ads_command(ADS_CMD_NULL);
	add_sample(current_reading[ADS_CHANNEL_STRAIN], &strain_buffer);
	add_sample(current_reading[ADS_CHANNEL_TORSION], &torsion_buffer);
}


void ads_write_reg(enum ADSRegisterAddress reg, uint8_t num_regs, uint16_t *source) {
	uint16_t write_command_to_issue = ADS_CMD_WREG | (reg << 7) | (num_regs - 1);

	/* Writing to registers is a bit messy

	Writing is kind of like a normal operation where the ADC values are returned with a CRC,
	so at least 4 words are exchanged in addition to the command.

	However depending on the amount of writing done there may not be enough registers to write
	to complete the reads so some transfers need to be done with no data to complete.

	Conversely if there are for than four registers write, it will extend beyond the returned
	data so that needs to be handled as well.

	Appended to any transfer into the ADC is a CRC word, which is currently not implemented
	nor expected by the ADC by default.
	*/

	uint8_t reg_sent = 0; // Used to keep track of registers sent between stages
	uint16_t calculated_crc = ads_calculate_crc(reg, source, num_regs);

	HAL_GPIO_WritePin(_config.cs_port, _config.cs_pin, GPIO_PIN_RESET);

	ads_transfer_single_word(write_command_to_issue, false);

	// Read ADCs
	for (; reg_sent < 3; reg_sent++) {
		// Send register values if available, CRC on trailing if needed
		if (reg_sent < num_regs) current_reading[reg_sent] = ads_transfer_single_word(source[reg_sent], true);
		else if (reg_sent == num_regs) current_reading[reg_sent] = ads_transfer_single_word(calculated_crc, true);
		else current_reading[reg_sent] = ads_transfer_single_word(0, true);
	}

	// Read in CRC from ADC
	uint16_t received_crc = 0;
	if (reg_sent < num_regs) received_crc = ads_transfer_single_word(source[reg_sent], false);
	else if (reg_sent == num_regs) received_crc = ads_transfer_single_word(calculated_crc, false);
	else received_crc = ads_transfer_single_word(0, false);
	reg_sent++;

	(void) received_crc; // Do nothing with the CRC currently

	// Send remaining registers. Data from ADS131M03 is invalid at this point
	for (; reg_sent < num_regs; reg_sent++) ads_transfer_single_word(source[reg_sent], false);

	// Append CRC if needed to exchanges longer than the standard message
	if ((_ads_mode.spi_crc_enable) && (num_regs > 4)) ads_transfer_single_word(calculated_crc, false);

	HAL_GPIO_WritePin(_config.cs_port, _config.cs_pin, GPIO_PIN_SET);
}


void ads_read_reg(enum ADSRegisterAddress reg, uint8_t num_regs, uint16_t *destination) {
  
	uint16_t readCommandToIssue = ADS_CMD_RREG | (reg << 7) | (num_regs - 1);

	ads_command(readCommandToIssue);

	// Read in the register(s)
	if (num_regs == 1) *destination = ads_command(ADS_CMD_NULL); // For a single register the register is returned as a response to the subsequent command
	else {
		// For multi-register reads, they are read in as a series appended with CRC

		HAL_GPIO_WritePin(_config.cs_port, _config.cs_pin, GPIO_PIN_RESET);

		uint16_t calculated_crc = ads_calculate_crc(ADS_CMD_NULL, NULL, 0);

		ads_transfer_single_word(ADS_CMD_NULL, false); // First response is an acknowledgement, discarded

		// Transfer CRC in exchange for first register
		destination[0] = ads_transfer_single_word(calculated_crc, false);
		for (uint8_t i = 1; i < num_regs; i++) destination[i] = ads_transfer_single_word(0, false);

		ads_transfer_single_word(0, false); // Discard CRC from ADS131M03

		HAL_GPIO_WritePin(_config.cs_port, _config.cs_pin, GPIO_PIN_SET);
	}
}


int32_t ads_transfer_single_word(uint16_t value, bool is_adc_reading) {
	uint8_t tx_buffer[4] = {(value >> 8) & 0xFF,  (value >> 0) & 0xFF, 0, 0};
	uint8_t rx_buffer[4] = {0};

	const uint_fast8_t bytes_per_word = 3;
	/* Disabled non-standard word code
	 * This was done to speed up the transfer process since in this design there's no need to change the width
	 */

//	switch (_ads_mode.word_format) {
//	case ADS_WORD_NORMAL:
//		bytes_per_word = 3;
//		break;
//	case ADS_WORD_SHORT:
//		bytes_per_word = 2;
//		break;
//	case ADS_WORD_ZERO_PAD:
//	case ADS_WORD_SIGN_PAD:
//		bytes_per_word = 4;
//		break;
//	}

	HAL_SPI_TransmitReceive(_config.bus, tx_buffer, rx_buffer, bytes_per_word, ADS_TIMEOUT_MS);

	int32_t response = 0;
	if (!is_adc_reading) { // If not an ADC reading then the first two bytes exchanged are the value and response
		response |= rx_buffer[0] << 8;
		response |= rx_buffer[1] << 0;

		return (response);
	}

	// Read in ADC readings may require sign extension using Two's compliment logic

//	switch (_ads_mode.word_format) {
//	case ADS_WORD_NORMAL:
//	case ADS_WORD_ZERO_PAD:
//		response |= rx_buffer[0] << 16;
//		response |= rx_buffer[1] <<  8;
//		response |= rx_buffer[2] <<  0;
//		// Extend sign if needed (1 at 24th bit)
//		if (response & 0x00800000) response |= 0xFF000000;
//		break;
//	case ADS_WORD_SHORT:
//		response |= rx_buffer[0] << 8;
//		response |= rx_buffer[1] << 0;
//		// Extend sign if needed (1 at 16th bit)
//		if (response & 0x00008000) response |= 0xFFFF0000;
//		break;
//	case ADS_WORD_SIGN_PAD:
//		response |= rx_buffer[0] << 24;
//		response |= rx_buffer[1] << 16;
//		response |= rx_buffer[2] <<  8;
//		response |= rx_buffer[3] <<  0;
//
//		// Sign already extended in SIGN_PAD
//		break;
//	}

	response |= rx_buffer[0] << 16;
	response |= rx_buffer[1] <<  8;
	response |= rx_buffer[2] <<  0;
	// Extend sign if needed (1 at 24th bit)
	if (response & 0x00800000) response |= 0xFF000000;

	return (response);
}

float ads_read_channel(enum ADSChannel channel) {
	struct CyclicalBuffer* buffer_of_interest = NULL;
	switch (channel) {
	case ADS_CHANNEL_TORSION:
		buffer_of_interest = &torsion_buffer;
		break;
	case ADS_CHANNEL_STRAIN:
		buffer_of_interest = &strain_buffer;
		break;
	default:
		return (0);
	}

	int32_t temp = buffer_of_interest->sum;
	temp = temp / buffer_of_interest->buffer_length;
	temp = temp - buffer_of_interest->zero_point;

	return (buffer_of_interest->scale * (float)temp);
}

void ads_configure_channel(enum ADSChannel channel, struct ADSChannelProcessingConfig config) {
	struct CyclicalBuffer* buffer_of_interest = NULL;

	switch (channel) {
	case ADS_CHANNEL_STRAIN:
		buffer_of_interest = &strain_buffer;

		_ads_gain.channel_0_gain = config.gain;
		_ads_clock.osr = config.osr;
		break;
	case ADS_CHANNEL_TORSION:
		buffer_of_interest = &torsion_buffer;

		_ads_gain.channel_1_gain = config.gain;
		_ads_clock.osr = config.osr;
		break;
	default:
		return;
	}

	clear_buffer(buffer_of_interest);
	buffer_of_interest->buffer_length = config.buffer_length;
	buffer_of_interest->zero_point = config.zero_point;
	buffer_of_interest->scale = config.scaling_factor;

	ads_update_config_regs();
}
