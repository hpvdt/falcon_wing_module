/*
 * surface_control.c
 *
 *  Created on: Aug 31, 2025
 *      Author: savo
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_tim.h"

#include "surface_control.h"

static volatile bool surface_unaligned = false;
static volatile struct SurfaceConfiguration surface_config = {
	.misalignment_alarm_sec = 5,
	.position_tolerance = 0.05,
	.potentiometer_bottom = 1000,
	.potentiometer_top = 3000,
	.potentiometer_half_range = 1000,
	.potentiometer_zero = 2000,
	.scheme = CONTROL_SCHEME_OPEN,
	.servo_bottom_us = 1500,
	.servo_top_us = 2500,
};

static volatile struct SurfaceCommand current_command = {
	.target = 0.0,
};


static float surface_read_angle() {
	uint16_t reading = HAL_ADC_GetValue(surface_config.pot_adc);

	reading = reading - surface_config.potentiometer_zero;
	float normalized_result = (float)reading / (float)surface_config.potentiometer_half_range;

	// TODO: potentially add some linearization to accommodate non-linear motion

	return (normalized_result);
}

static uint16_t surface_angle_to_servo_us(float normalized_angle) {
	float temp = normalized_angle * surface_config.servo_us_half_range;
	temp = temp + surface_config.servo_us_zero;

	// TODO: potentially add some linearization to accommodate non-linear motion

	uint16_t result = temp;
	return (result);
}

static HAL_StatusTypeDef surface_set_surface(uint16_t duty_us) {
	TIM_OC_InitTypeDef temp_oc_config= {0};
	temp_oc_config.OCMode = TIM_OCMODE_PWM1;
	temp_oc_config.Pulse = duty_us;
	temp_oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	temp_oc_config.OCFastMode = TIM_OCFAST_DISABLE;
	return (HAL_TIM_PWM_ConfigChannel(surface_config.servo_timer, &temp_oc_config, surface_config.servo_timer_channel));
}

void surface_update_command(struct SurfaceCommand command) {
	current_command.target = command.target;
}

void surface_stop() {
	HAL_TIM_PWM_Stop(surface_config.servo_timer, surface_config.servo_timer_channel);
}

void surface_start(struct SurfaceConfiguration surf_config) {
	surface_config = surf_config;

	/* Precalculating some variables
	 * This helps speed up other portion of codes, especially those potentially called in interrupts
	 * I am purposely not taking absolutes so we can have inverted scales, for example:
	 *
	 * Servo "bottom" is actually the higher end of the duty due to assembly
	 */
	surface_config.servo_us_half_range = (surface_config.servo_top_us - surface_config.servo_bottom_us) / 2;
	surface_config.servo_us_zero = surface_config.servo_bottom_us + surface_config.servo_us_half_range;
	surface_config.potentiometer_half_range = (surface_config.potentiometer_top - surface_config.potentiometer_bottom) / 2;
	surface_config.potentiometer_zero = surface_config.potentiometer_bottom + surface_config.potentiometer_half_range;

	// TODO: Start PWM and attach control loop interrupt?
	HAL_TIM_PWM_Start(surface_config.servo_timer, surface_config.servo_timer_channel);
}


void surface_prepare_broadcast(struct SurfaceBroadcast* destination) {
	destination->reading = surface_read_angle();
	destination->surface_not_following = surface_unaligned;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim == surface_config.servo_timer) surface_control_loop();
}

void surface_control_loop() {
	float current_angle = surface_read_angle(); // Read once at the start since it won't physically change quick enough

	switch (surface_config.scheme) {
	case CONTROL_SCHEME_BANG_BANG:
		// TODO: Implement. Currently fall-through to open control
	case CONTROL_SCHEME_OPEN:
		surface_set_surface(surface_angle_to_servo_us(current_command.target));
		break;
	default:
		surface_set_surface(surface_config.servo_us_zero); // Fail-safe
		break;
	}

	float delta = fabs(current_command.target - current_angle);
	bool in_position = delta < surface_config.position_tolerance;

	static uint32_t start_of_misalignment_tick = 0;

	if (in_position) {
		start_of_misalignment_tick = 0;
		surface_unaligned = false;
	}
	else {
		const uint32_t CURRENT_TICK = HAL_GetTick();

		if (start_of_misalignment_tick == 0) {
			start_of_misalignment_tick = CURRENT_TICK + surface_config.misalignment_alarm_sec * 1000;
		}

		surface_unaligned = CURRENT_TICK > start_of_misalignment_tick;
	}
}
