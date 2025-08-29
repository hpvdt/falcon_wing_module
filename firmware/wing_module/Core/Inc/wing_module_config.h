/*
 * wing_module_config.h
 *
 *  Created on: Aug 24, 2025
 *      Author: savo
 */

#ifndef INC_WING_MODULE_CONFIG_H_
#define INC_WING_MODULE_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

// Trying to keep all these configs to 8 bytes or less so they can be transmitted as single CAN frames

struct WingModuleGeneralConfig {
	bool driving_servo : 1;
	bool measuring_torsion : 1;
	bool measuring_strain : 1;
	bool measure_lidar : 1;
	bool operating_indicator : 1;
	uint8_t RESERVE : 3;
	uint16_t update_period_ms : 16;
};

enum ControlSurface {
	CONTROL_SURFACE_PORT = 0,
	CONTROL_SURFACE_STARBOARD = 1,
	CONTROL_SURFACE_ELEVATOR = 2,
	CONTROL_SURFACE_RUDDER = 3,
};

struct WingModuleServoConfig {
	enum ControlSurface surface : 2;
	uint8_t misalignment_alarm_sec : 6; // Time to tolerate an out of tolerance location
	uint16_t potentiometer_top : 12;
	uint16_t potentiometer_bottom : 12;
	uint16_t servo_top : 12;
	uint16_t servo_bottom : 12;
	uint8_t position_tolerance_half_percent : 8;
};

struct WingModuleStrainGaugeConfig {
	uint8_t buffer_depth : 8;
	int8_t osr : 3;
	uint8_t gain : 3;
	uint8_t RESERVE : 2;
	int32_t zero_point : 24;
	int32_t division_factor : 24; // What to divide the average by to get final value
};

struct WingModuleLidarConfig {
	uint8_t buffer_depth : 8;
};

struct WingModuleIndicatorConfig {
	bool disable_buzzer : 1;
	uint8_t invert_led : 7;
	uint16_t buzzer_period_ms : 16; // Duration of buzzer period if a surface is not in place (0 for constant buzzing)
};

struct WingModuleConfig {
	uint8_t node_id; // Determined from solder jumpers on PCB
	uint8_t configuration_needed; // Masked bitwise for which mode still needs configuration data

	// The centrally provided configurations
	struct WingModuleGeneralConfig general;
	struct WingModuleServoConfig servo;
	struct WingModuleStrainGaugeConfig strain;
	struct WingModuleStrainGaugeConfig torsion;
	struct WingModuleLidarConfig lidar;
	struct WingModuleIndicatorConfig indicator;
};

#endif /* INC_WING_MODULE_CONFIG_H_ */
