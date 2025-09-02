/*
 * wing_module_config.h
 *
 *  Created on: Aug 24, 2025
 *      Author: savo
 */

#ifndef INC_WING_MODULE_CONFIG_H_
#define INC_WING_MODULE_CONFIG_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"
#include <stdbool.h>
#include <stdint.h>
#include "can_wrapper.h"

// Trying to keep all these configs to 8 bytes or less so they can be transmitted as single CAN frames

struct WingModuleConfig {
	uint8_t node_id; // Determined from solder jumpers on PCB
	uint8_t configuration_needed; // Masked bitwise for which mode still needs configuration data

	struct CANNodeGeneralConfigCommand general;
	struct CANServoConfigCommand servo;
	struct CANStrainGaugeConfigCommand strain;
	struct CANStrainGaugeConfigCommand torsion;
	struct CANLidarConfigCommand lidar;
	struct CANIndicatorConfigCommand indicator;
};

void can_update_node_filters(struct WingModuleConfig* config, CAN_HandleTypeDef* can);

#endif /* INC_WING_MODULE_CONFIG_H_ */
