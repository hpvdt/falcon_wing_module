/*
 * can_wrapper.h
 *
 *  Created on: Aug 25, 2025
 *      Author: savo
 */

#ifndef INC_FALCON_CAN_H_
#define INC_FALCON_CAN_H_

#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "wing_module_config.h"

enum ConfigMessageID {
	CONFIG_MESSAGE_GENERAL 		= 0,
	CONFIG_MESSAGE_SERVO 		= 1,
	CONFIG_MESSAGE_TORSION 		= 2,
	CONFIG_MESSAGE_STRAIN 		= 3,
	CONFIG_MESSAGE_INDICATOR 	= 4,
	CONFIG_MESSAGE_LIDAR 		= 5,
};

struct WingLoadBroadcast {
	float strain_reading;
	float torsion_reading;
};

struct WingLightCommand {
	uint8_t pulse_count : 8;
	uint8_t heart_beat_max_duty : 8;
	uint8_t heart_beat_min_duty : 8;
	uint16_t pulse_period_on_ms : 16;
	uint16_t pulse_period_off_ms : 16;
};

// Mask defines bits that MUST match
#define CAN_CONFIG_ID_BASE				0x600
#define CAN_CONFIG_MASK					0x71F
#define CAN_COMMAND_ANGLE_ID_BASE		0x020
#define CAN_COMMAND_LIGHT_ID_BASE		0x040
#define CAN_READING_ANGLE_ID_BASE		0x200
#define CAN_READING_ANGLE_MASK			0x7FC

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

#define CAN_BROADCAST_ANGLE_ID_BASE		0x200
#define CAN_BROADCAST_LOAD_ID_BASE		0x220
#define CAN_BROADCAST_LIDAR_ID_BASE		0x240

void can_update_filters(struct WingModuleConfig* config, CAN_HandleTypeDef* can);

#endif /* INC_FALCON_CAN_H_ */
