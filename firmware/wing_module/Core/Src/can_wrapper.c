#include <stdbool.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"
#include "can_wrapper.h"

static uint8_t bits_to_dlc(uint8_t bits) {
	uint8_t quotient = bits / 8;
	if (quotient >= 8) return (8); // Clamp it to max

	uint8_t remainder = bits % 8;
	if (remainder == 0) return (quotient);
	return (quotient + 1); // Round up if needed
}


static void get_unsigned_int(uint8_t* buffer, uint8_t* current_bit, uint8_t bit_length, uint32_t* destination) {
	if (*current_bit >= 64) return;

	uint8_t end_bit = *current_bit + bit_length;
	uint8_t start_byte = bits_to_dlc(*current_bit);
	uint8_t end_byte = bits_to_dlc(end_bit - 1);

	uint32_t collected_data = 0;

	for (uint_fast8_t i = 0; i <= (end_byte - start_byte); i++) {
		uint8_t bit_of_interest = *current_bit & 0x7;
		uint8_t temp = buffer[i + start_byte];

		const uint_fast8_t BITS_AVAILABLE_IN_THIS_BYTE = 8 - bit_of_interest;
		uint_fast8_t bits_to_mask_out = bit_length;
		if (bits_to_mask_out > BITS_AVAILABLE_IN_THIS_BYTE) bits_to_mask_out = BITS_AVAILABLE_IN_THIS_BYTE;

		temp = temp >> bit_of_interest;
		const uint8_t MASK = 0xFF >> (8 - bits_to_mask_out);
		temp = temp & MASK;

		collected_data = collected_data | (temp << (8 * i));

		bit_length = bit_length - bits_to_mask_out;
		*current_bit = *current_bit + bits_to_mask_out;
	}

	*destination = collected_data;
}

static void get_bool(uint8_t* buffer, uint8_t* current_bit, uint8_t bit_length, bool* destination) {
	uint32_t aquired_val = 0;
	get_unsigned_int(buffer, current_bit, bit_length, &aquired_val);
	*destination = aquired_val == true;
}

static void get_signed_int(uint8_t* buffer, uint8_t* current_bit, uint8_t bit_length, int32_t* destination) {
	uint32_t acquired_val = 0;
	get_unsigned_int(buffer, current_bit, bit_length, &acquired_val);

	// Sign extend if needed
	int32_t sign_extended = *(int32_t*) &acquired_val;
	if ((1 << (bit_length - 1)) & sign_extended) {
		sign_extended = sign_extended | (UINT32_MAX << bit_length);
	}

	*destination = sign_extended;
}

static void get_float(uint8_t* buffer, uint8_t* current_bit, uint8_t bit_length, float* destination) {
	if (bit_length != 32) return; // Should only be used for complete floats (32 - bit)
	get_unsigned_int(buffer, current_bit, bit_length, (uint32_t*) destination);
}

static void put_unsigned_int(uint8_t* buffer, uint8_t* current_bit, uint8_t bit_length, uint32_t val) {
	if (*current_bit >= 64) return;

	// NOTE: This function assumes that the buffer has been cleared beforehand

	uint8_t end_bit = *current_bit + bit_length;
	uint8_t start_byte = bits_to_dlc(*current_bit);
	uint8_t end_byte = bits_to_dlc(end_bit - 1);

	for (uint_fast8_t i = start_byte; i <= end_byte; i++) {
		uint8_t bit_of_interest = *current_bit & 0x7;
		uint8_t temp = val & 0xFF;

		const uint_fast8_t BITS_POSSIBLE_TO_WRITE = bit_length;
		uint_fast8_t bits_to_mask_out = bit_length;
		if (bits_to_mask_out > BITS_POSSIBLE_TO_WRITE) bits_to_mask_out = BITS_POSSIBLE_TO_WRITE;

		const uint8_t MASK = 0xFF >> (8 - bits_to_mask_out);
		temp = temp & MASK; // Ensure we don't write value beyond our designated slot
		temp = temp << bit_of_interest;

		buffer[i] = buffer[i] | temp;

		bit_length = bit_length - bits_to_mask_out;
		*current_bit = *current_bit + bits_to_mask_out;

		val = val >> bits_to_mask_out; // Shift down value for next byte
	}
}

static void put_bool(uint8_t* buffer, uint8_t* current_bit, uint8_t bit_length, bool val) {
	put_unsigned_int(buffer, current_bit, bit_length, val);
}

static void put_signed_int(uint8_t* buffer, uint8_t* current_bit, uint8_t bit_length, int32_t val) {
	put_unsigned_int(buffer, current_bit, bit_length, *(int32_t*) &val);
}

static void put_float(uint8_t* buffer, uint8_t* current_bit, uint8_t bit_length, float val) {
	if (bit_length != 32) return; // Should only be used for complete floats (32 - bit)
	put_unsigned_int(buffer, current_bit, bit_length, *(int32_t*) &val);
}

uint8_t can_pack_light_command(uint8_t* destination_buffer, struct CANLightCommand msg) {
	uint8_t current_bit = 0;

	put_unsigned_int(destination_buffer, &current_bit, 8, msg.pulse_count);
	put_unsigned_int(destination_buffer, &current_bit, 8, msg.heart_beat_max_duty);
	put_unsigned_int(destination_buffer, &current_bit, 8, msg.heart_beat_min_duty);
	put_unsigned_int(destination_buffer, &current_bit, 16, msg.pulse_period_on_ms);
	put_unsigned_int(destination_buffer, &current_bit, 16, msg.pulse_period_off_ms);

	return (bits_to_dlc(current_bit - 1));
}

uint8_t can_pack_surface_command(uint8_t* destination_buffer, struct CANSurfaceCommand msg) {
	uint8_t current_bit = 0;

	put_signed_int(destination_buffer, &current_bit, 16, msg.port_angle_thousandths);
	put_signed_int(destination_buffer, &current_bit, 16, msg.starboard_angle_thousandths);
	put_signed_int(destination_buffer, &current_bit, 16, msg.elevator_angle_thousandths);
	put_signed_int(destination_buffer, &current_bit, 16, msg.rudder_angle_thousandths);

	return (bits_to_dlc(current_bit - 1));
}

uint8_t can_pack_load_broadcast(uint8_t* destination_buffer, struct CANLoadBroadcast msg) {
	uint8_t current_bit = 0;

	put_float(destination_buffer, &current_bit, 32, msg.strain_reading);
	put_float(destination_buffer, &current_bit, 32, msg.torsion_reading);

	return (bits_to_dlc(current_bit - 1));
}

uint8_t can_pack_surface_broadcast(uint8_t* destination_buffer, struct CANSurfaceBroadcast msg) {
	uint8_t current_bit = 0;

	put_signed_int(destination_buffer, &current_bit, 15, msg.reading_thousandths);
	put_bool(destination_buffer, &current_bit, 1, msg.surface_not_following);

	return (bits_to_dlc(current_bit - 1));
}

uint8_t can_pack_lidar_broadcast(uint8_t* destination_buffer, struct CANLidarBroadcast msg) {
	uint8_t current_bit = 0;

	put_unsigned_int(destination_buffer, &current_bit, 15, msg.distance_mm);
	put_bool(destination_buffer, &current_bit, 1, msg.alarm);

	return (bits_to_dlc(current_bit - 1));
}

uint8_t can_pack_node_general_config_command(uint8_t* destination_buffer, struct CANNodeGeneralConfigCommand msg) {
	uint8_t current_bit = 0;

	put_bool(destination_buffer, &current_bit, 1, msg.driving_servo);
	put_bool(destination_buffer, &current_bit, 1, msg.measuring_strain);
	put_bool(destination_buffer, &current_bit, 1, msg.measuring_torsion);
	put_bool(destination_buffer, &current_bit, 1, msg.measuring_lidar);
	put_bool(destination_buffer, &current_bit, 1, msg.operating_indicator);
	put_unsigned_int(destination_buffer, &current_bit, 11, msg.update_period_ms);

	return (bits_to_dlc(current_bit - 1));
}

uint8_t can_pack_servo_config_command(uint8_t* destination_buffer, struct CANServoConfigCommand msg) {
	uint8_t current_bit = 0;

	put_unsigned_int(destination_buffer, &current_bit, 2, msg.surface);
	put_unsigned_int(destination_buffer, &current_bit, 2, msg.scheme);
	put_unsigned_int(destination_buffer, &current_bit, 4, msg.misalignment_alarm_sec);
	put_unsigned_int(destination_buffer, &current_bit, 12, msg.potentiometer_top);
	put_unsigned_int(destination_buffer, &current_bit, 12, msg.potentiometer_bottom);
	put_unsigned_int(destination_buffer, &current_bit, 12, msg.servo_top_us);
	put_unsigned_int(destination_buffer, &current_bit, 12, msg.servo_bottom_us);
	put_unsigned_int(destination_buffer, &current_bit, 8, msg.position_tolerance_half_percent);

	return (bits_to_dlc(current_bit - 1));
}

uint8_t can_pack_strain_gauge_config_command(uint8_t* destination_buffer, struct CANStrainGaugeConfigCommand msg) {
	uint8_t current_bit = 0;

	put_unsigned_int(destination_buffer, &current_bit, 3, msg.osr);
	put_unsigned_int(destination_buffer, &current_bit, 3, msg.gain);
	put_unsigned_int(destination_buffer, &current_bit, 2, msg.buffer_depth_64_samples);
	put_signed_int(destination_buffer, &current_bit, 24, msg.zero_point);
	put_float(destination_buffer, &current_bit, 32, msg.scaling_factor);

	return (bits_to_dlc(current_bit - 1));
}

uint8_t can_pack_lidar_config_command(uint8_t* destination_buffer, struct CANLidarConfigCommand msg) {
	uint8_t current_bit = 0;

	put_unsigned_int(destination_buffer, &current_bit, 8, msg.buffer_depth);
	put_unsigned_int(destination_buffer, &current_bit, 16, msg.alarm_level);

	return (bits_to_dlc(current_bit - 1));
}

uint8_t can_pack_indicator_config_command(uint8_t* destination_buffer, struct CANIndicatorConfigCommand msg) {
	uint8_t current_bit = 0;

	put_bool(destination_buffer, &current_bit, 1, msg.disable_buzzer);
	put_unsigned_int(destination_buffer, &current_bit, 7, msg.invert_led);
	put_unsigned_int(destination_buffer, &current_bit, 16, msg.buzzer_period_ms);

	return (bits_to_dlc(current_bit - 1));
}







void can_unpack_light_command(uint8_t* source_buffer, struct CANLightCommand* msg) {
	uint8_t current_bit = 0;

	get_unsigned_int(source_buffer, &current_bit, 8, (uint32_t*)&msg->pulse_count);
	get_unsigned_int(source_buffer, &current_bit, 8, (uint32_t*)&msg->heart_beat_max_duty);
	get_unsigned_int(source_buffer, &current_bit, 8, (uint32_t*)&msg->heart_beat_min_duty);
	get_unsigned_int(source_buffer, &current_bit, 16, (uint32_t*)&msg->pulse_period_on_ms);
	get_unsigned_int(source_buffer, &current_bit, 16, (uint32_t*)&msg->pulse_period_off_ms);
}

void can_unpack_surface_command(uint8_t* source_buffer, struct CANSurfaceCommand* msg) {
	uint8_t current_bit = 0;

	get_signed_int(source_buffer, &current_bit, 16, (int32_t*)&msg->port_angle_thousandths);
	get_signed_int(source_buffer, &current_bit, 16, (int32_t*)&msg->starboard_angle_thousandths);
	get_signed_int(source_buffer, &current_bit, 16, (int32_t*)&msg->elevator_angle_thousandths);
	get_signed_int(source_buffer, &current_bit, 16, (int32_t*)&msg->rudder_angle_thousandths);
}

void can_unpack_load_broadcast(uint8_t* source_buffer, struct CANLoadBroadcast* msg) {
	uint8_t current_bit = 0;

	get_float(source_buffer, &current_bit, 32, &msg->strain_reading);
	get_float(source_buffer, &current_bit, 32, &msg->torsion_reading);
}

void can_unpack_surface_broadcast(uint8_t* source_buffer, struct CANSurfaceBroadcast* msg) {
	uint8_t current_bit = 0;

	get_signed_int(source_buffer, &current_bit, 15, (int32_t*)&msg->reading_thousandths);
	get_bool(source_buffer, &current_bit, 1, &msg->surface_not_following);
}

void can_unpack_lidar_broadcast(uint8_t* source_buffer, struct CANLidarBroadcast* msg) {
	uint8_t current_bit = 0;

	get_unsigned_int(source_buffer, &current_bit, 15, (uint32_t*)&msg->distance_mm);
	get_bool(source_buffer, &current_bit, 1, &msg->alarm);
}

void can_unpack_node_general_config_command(uint8_t* source_buffer, struct CANNodeGeneralConfigCommand* msg) {
	uint8_t current_bit = 0;

	get_bool(source_buffer, &current_bit, 1, &msg->driving_servo);
	get_bool(source_buffer, &current_bit, 1, &msg->measuring_strain);
	get_bool(source_buffer, &current_bit, 1, &msg->measuring_torsion);
	get_bool(source_buffer, &current_bit, 1, &msg->measuring_lidar);
	get_bool(source_buffer, &current_bit, 1, &msg->operating_indicator);
	get_unsigned_int(source_buffer, &current_bit, 11, (uint32_t*)&msg->update_period_ms);
}

void can_unpack_servo_config_command(uint8_t* source_buffer, struct CANServoConfigCommand* msg) {
	uint8_t current_bit = 0;

	get_unsigned_int(source_buffer, &current_bit, 2, (uint32_t*)&msg->surface);
	get_unsigned_int(source_buffer, &current_bit, 2, (uint32_t*)&msg->scheme);
	get_unsigned_int(source_buffer, &current_bit, 4, (uint32_t*)&msg->misalignment_alarm_sec);
	get_unsigned_int(source_buffer, &current_bit, 12, (uint32_t*)&msg->potentiometer_top);
	get_unsigned_int(source_buffer, &current_bit, 12, (uint32_t*)&msg->potentiometer_bottom);
	get_unsigned_int(source_buffer, &current_bit, 12, (uint32_t*)&msg->servo_top_us);
	get_unsigned_int(source_buffer, &current_bit, 12, (uint32_t*)&msg->servo_bottom_us);
	get_unsigned_int(source_buffer, &current_bit, 8, (uint32_t*)&msg->position_tolerance_half_percent);
}

void can_unpack_strain_gauge_config_command(uint8_t* source_buffer, struct CANStrainGaugeConfigCommand* msg) {
	uint8_t current_bit = 0;

	get_unsigned_int(source_buffer, &current_bit, 3, (uint32_t*)&msg->osr);
	get_unsigned_int(source_buffer, &current_bit, 3, (uint32_t*)&msg->gain);
	get_unsigned_int(source_buffer, &current_bit, 2, (uint32_t*)&msg->buffer_depth_64_samples);
	get_signed_int(source_buffer, &current_bit, 24, (int32_t*)&msg->zero_point);
	get_float(source_buffer, &current_bit, 32, &msg->scaling_factor);
}

void can_unpack_lidar_config_command(uint8_t* source_buffer, struct CANLidarConfigCommand* msg) {
	uint8_t current_bit = 0;

	get_unsigned_int(source_buffer, &current_bit, 8, (uint32_t*)&msg->buffer_depth);
	get_unsigned_int(source_buffer, &current_bit, 16, (uint32_t*)&msg->alarm_level);
}

void can_unpack_indicator_config_command(uint8_t* source_buffer, struct CANIndicatorConfigCommand* msg) {
	uint8_t current_bit = 0;

	get_bool(source_buffer, &current_bit, 1, &msg->disable_buzzer);
	get_unsigned_int(source_buffer, &current_bit, 7, (uint32_t*)&msg->invert_led);
	get_unsigned_int(source_buffer, &current_bit, 16, (uint32_t*)&msg->buzzer_period_ms);
}
