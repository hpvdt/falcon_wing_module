#ifndef INC_LED_CONTROL_H_
#define INC_LED_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

struct LEDControl {
	bool error_light_on;

	bool new_pulse_data;
	uint8_t pulse_count;
	uint16_t pulse_period_on_ms;
	uint16_t pulse_period_off_ms;

	uint16_t heart_beat_max_duty;
	uint16_t heart_beat_min_duty;

	TIM_HandleTypeDef* error_timer;
	uint32_t error_channel;
	TIM_HandleTypeDef* status_timer;
	uint32_t status_channel;
};

void operate_led(struct LEDControl* led_config);

#endif /* INC_LED_CONTROL_H_ */
