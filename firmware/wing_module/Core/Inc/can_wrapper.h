/*
 * can_wrapper.h
 *
 *  Created on: Aug 25, 2025
 *      Author: savo
 */

#ifndef INC_FALCON_CAN_H_
#define INC_FALCON_CAN_H_

#include <stdint.h>

// Mask defines bits that MUST match
#define CAN_CONFIG_ID_BASE				0x400
#define CAN_CONFIG_MASK					0x71F
#define CAN_CONFIG_BIT_START			5
#define CAN_COMMAND_ANGLE_ID_BASE		0x020
#define CAN_COMMAND_LIGHT_ID_BASE		0x040
#define CAN_READING_ANGLE_ID_BASE		0x200
#define CAN_READING_ANGLE_MASK			0x7FC
#define CAN_NODE_ADDRESS_MASK			0x7E0

#define CAN_COMMAND_LIGHT_FILTER_BANK 	0
#define CAN_COMMAND_ANGLE_FILTER_BANK 	1
#define CAN_CONFIG_FILTER_BANK 			2
#define CAN_READING_ANGLE_BANK 			3

// These might need to be verified after programming, used for checking acceptance filter
#define CAN_COMMAND_LIGHT_FILTER_NUMBER 0
#define CAN_COMMAND_ANGLE_FILTER_NUMBER 2
#define CAN_CONFIG_FILTER_NUMBER 		0
#define CAN_READING_ANGLE_NUMBER 		1

#define CAN_IGNORE_EXTENDED_ID_HIGH 	0x0000
#define CAN_IGNORE_EXTENDED_ID_LOW 		0x0000
#define CAN_IGNORE_EXTENDED_MASK_HIGH 	0x0000
#define CAN_IGNORE_EXTENDED_MASK_LOW 	0x0004

#define CAN_CONFIG_FIFO 				CAN_FILTER_FIFO0
#define CAN_COMMAND_FIFO 				CAN_FILTER_FIFO1

#define CAN_BROADCAST_ANGLE_ID			0x200
#define CAN_BROADCAST_LOAD_ID_BASE		0x220
#define CAN_BROADCAST_LIDAR_ID_BASE		0x240

enum ConfigMessageID {
	CONFIG_MESSAGE_GENERAL 		= 0,
	CONFIG_MESSAGE_SERVO 		= 1,
	CONFIG_MESSAGE_TORSION 		= 2,
	CONFIG_MESSAGE_STRAIN 		= 3,
	CONFIG_MESSAGE_INDICATOR 	= 4,
	CONFIG_MESSAGE_LIDAR 		= 5,
};

struct CANLoadBroadcast {
	float strain_reading;
	float torsion_reading;
};

struct CANLightCommand {
	uint8_t pulse_count;
	uint8_t heart_beat_max_duty;
	uint8_t heart_beat_min_duty;
	uint16_t pulse_period_on_ms;
	uint16_t pulse_period_off_ms;
};

struct CANSurfaceCommand {
	// Store the angles normalized (-1 to 1) in thousandths
	int16_t port_angle_thousandths;
	int16_t starboard_angle_thousandths;
	int16_t elevator_angle_thousandths;
	int16_t rudder_angle_thousandths;
};

struct CANSurfaceBroadcast {
	int16_t reading_thousandths; // 15 bits Value between -1000 and 1000 inclusive representing surface reading
	bool surface_not_following;
};

struct CANNodeGeneralConfigCommand {
	bool driving_servo;
	bool measuring_torsion;
	bool measuring_strain;
	bool measuring_lidar;
	bool operating_indicator;
	uint16_t update_period_ms; // 11 bits
};

enum ControlSurface {
	CONTROL_SURFACE_PORT = 0,
	CONTROL_SURFACE_STARBOARD = 1,
	CONTROL_SURFACE_ELEVATOR = 2,
	CONTROL_SURFACE_RUDDER = 3,
};

enum ControlScheme {
	CONTROL_SCHEME_OPEN = 0,
	CONTROL_SCHEME_BANG_BANG = 1,
};

struct CANServoConfigCommand {
	uint8_t surface; // 2 bits
	uint8_t scheme; // 2 bits
	uint8_t misalignment_alarm_sec ; // 4 bits, Time to tolerate an out of tolerance location (0 to disable)
	uint16_t potentiometer_top; // 12 bits
	uint16_t potentiometer_bottom; // 12 bits
	uint16_t servo_top_us; // 12 bits
	uint16_t servo_bottom_us; // 12 bits
	uint8_t position_tolerance_half_percent;
};

struct CANStrainGaugeConfigCommand {
	uint8_t osr; // 3 bits
	uint8_t gain; // 3 bits
	uint8_t buffer_depth_64_samples; // 2-bits, Allows some control of the buffer size
	int32_t zero_point;
	float scaling_factor; // What to multiply the average by to get final value
};

struct CANLidarConfigCommand {
	uint8_t buffer_depth;
	uint16_t alarm_level;
};

struct CANLidarBroadcast {
	uint16_t distance_mm;
	bool alarm;
};

struct CANIndicatorConfigCommand {
	bool disable_buzzer;
	uint8_t invert_led; // 7 bit
	uint16_t buzzer_period_ms; // Duration of buzzer period if a surface is not in place (0 for constant buzzing)
};

uint8_t can_pack_light_command(uint8_t* destination_buffer, struct CANLightCommand msg);
uint8_t can_pack_surface_command(uint8_t* destination_buffer, struct CANSurfaceCommand msg);
uint8_t can_pack_load_broadcast(uint8_t* destination_buffer, struct CANLoadBroadcast msg);
uint8_t can_pack_surface_broadcast(uint8_t* destination_buffer, struct CANSurfaceBroadcast msg);
uint8_t can_pack_lidar_broadcast(uint8_t* destination_buffer, struct CANLidarBroadcast msg);
uint8_t can_pack_node_general_config_command(uint8_t* destination_buffer, struct CANNodeGeneralConfigCommand msg);
uint8_t can_pack_servo_config_command(uint8_t* destination_buffer, struct CANServoConfigCommand msg);
uint8_t can_pack_strain_gauge_config_command(uint8_t* destination_buffer, struct CANStrainGaugeConfigCommand msg);
uint8_t can_pack_lidar_config_command(uint8_t* destination_buffer, struct CANLidarConfigCommand msg);
uint8_t can_pack_indicator_config_command(uint8_t* destination_buffer, struct CANIndicatorConfigCommand msg);

void can_unpack_light_command(uint8_t* source_buffer, struct CANLightCommand* msg);
void can_unpack_surface_command(uint8_t* source_buffer, struct CANSurfaceCommand* msg);
void can_unpack_load_broadcast(uint8_t* source_buffer, struct CANLoadBroadcast* msg);
void can_unpack_surface_broadcast(uint8_t* source_buffer, struct CANSurfaceBroadcast* msg);
void can_unpack_lidar_broadcast(uint8_t* source_buffer, struct CANLidarBroadcast* msg);
void can_unpack_node_general_config_command(uint8_t* source_buffer, struct CANNodeGeneralConfigCommand* msg);
void can_unpack_servo_config_command(uint8_t* source_buffer, struct CANServoConfigCommand* msg);
void can_unpack_strain_gauge_config_command(uint8_t* source_buffer, struct CANStrainGaugeConfigCommand* msg);
void can_unpack_lidar_config_command(uint8_t* source_buffer, struct CANLidarConfigCommand* msg);
void can_unpack_indicator_config_command(uint8_t* source_buffer, struct CANIndicatorConfigCommand* msg);

#endif /* INC_FALCON_CAN_H_ */
