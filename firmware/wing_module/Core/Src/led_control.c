#include <stdbool.h>
#include <stdint.h>

#include "led_control.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

static const uint16_t ERROR_ON_DUTY = UINT16_MAX;
static const uint16_t ERROR_OFF_DUTY = 0;
static const uint16_t HEARTBEAT_DUTY_INCREMENT = 10;
static const uint16_t HEARTBEAT_INCREMENT_PERIOD_MS = 5;

void operate_led(struct LEDControl* led_config) {
	// Error light is always just binary
	uint16_t error_duty = ERROR_ON_DUTY;
	if (!led_config->error_light_on) error_duty = ERROR_OFF_DUTY;

	TIM_OC_InitTypeDef pwm_config = {0};
	pwm_config.Pulse = error_duty;
	pwm_config.OCMode = TIM_OCMODE_PWM1;
	pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	pwm_config.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(led_config->error_timer, &pwm_config, led_config->error_channel);

	static uint32_t next_change_ms = 0;
	static int32_t current_duty = 0; // Slightly over sized to catch underflows on uint16_t
	static int16_t duty_increment = HEARTBEAT_DUTY_INCREMENT;
	static bool current_status_led_state = false;
	const uint32_t CURRENT_TICK = HAL_GetTick();

	if (led_config->new_pulse_data) {
		led_config->new_pulse_data = false;

		next_change_ms = CURRENT_TICK;
		current_status_led_state = false;
		led_config->pulse_count++; // Accommodate the immediate decrement that follows
	}

	if (next_change_ms > CURRENT_TICK) return; // Skip unneeded updates

	if (led_config->pulse_count > 0) { // Check if pulses are needed
		if (current_status_led_state) {
			current_duty = UINT16_MAX;
			next_change_ms = CURRENT_TICK + led_config->pulse_period_on_ms;
		}
		else {
			current_duty = 0;
			next_change_ms = CURRENT_TICK + led_config->pulse_period_off_ms;
			led_config->pulse_count--;
		}
	}
	else { // Heart beat
		current_duty += duty_increment;

		if (current_duty < led_config->heart_beat_min_duty) {
			current_duty = led_config->heart_beat_min_duty;
			duty_increment = HEARTBEAT_DUTY_INCREMENT;
		}
		else if (current_duty > led_config->heart_beat_max_duty) {
			current_duty = led_config->heart_beat_max_duty;
			duty_increment = -HEARTBEAT_DUTY_INCREMENT;
		}

		next_change_ms = CURRENT_TICK + HEARTBEAT_INCREMENT_PERIOD_MS;
	}

	pwm_config.Pulse = current_duty;
	HAL_TIM_PWM_ConfigChannel(led_config->status_timer, &pwm_config, led_config->status_channel);
}
