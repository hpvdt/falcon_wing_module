/*
 * surface_control.h
 *
 *  Created on: Aug 31, 2025
 *      Author: savo
 */

#ifndef INC_SURFACE_CONTROL_H_
#define INC_SURFACE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>
#include "can_wrapper.h"
#include "wing_module_config.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_tim.h"

struct SurfaceConfiguration {
	enum ControlScheme scheme;
	uint8_t misalignment_alarm_sec; // Time to tolerate an out of tolerance location
	uint16_t potentiometer_top;
	uint16_t potentiometer_bottom;
	uint16_t servo_top_us;
	uint16_t servo_bottom_us;
	float position_tolerance;

	ADC_HandleTypeDef* pot_adc;
	TIM_HandleTypeDef* servo_timer;
	uint32_t servo_timer_channel;

	uint16_t potentiometer_zero; // Calculated, no need to set
	int16_t potentiometer_half_range; // Calculated, no need to set
	uint16_t servo_us_zero; // Calculated, no need to set
	int16_t servo_us_half_range; // Calculated, no need to set
};

struct SurfaceCommand {
	float target; // Value between -1 and 1 inclusive to aim for
};

struct SurfaceBroadcast {
	float reading; // Value between -1 and 1 inclusive representing surface reading
	bool surface_not_following : 1;
};

void surface_start(struct SurfaceConfiguration surf_config);
void surface_stop();

void surface_control_loop();

void surface_update_command(struct SurfaceCommand command);

void surface_prepare_broadcast(struct SurfaceBroadcast* destination);

#endif /* INC_SURFACE_CONTROL_H_ */
