#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"
#include <stdbool.h>
#include <stdint.h>
#include "can_wrapper.h"
#include "wing_module_config.h"

void can_update_node_filters(struct WingModuleConfig* config, CAN_HandleTypeDef* can) {
	uint16_t id_filter = config->node_id | CAN_CONFIG_ID_BASE;
	id_filter = (id_filter << 5) | CAN_IGNORE_EXTENDED_ID_HIGH;

	uint16_t mask_filter = CAN_CONFIG_MASK;
	mask_filter = (mask_filter << 5) | CAN_IGNORE_EXTENDED_MASK_HIGH;

	HAL_CAN_Stop(can);

	CAN_FilterTypeDef filter_for_config = {
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterActivation = CAN_FILTER_ENABLE, 		// Always enabled
		.FilterFIFOAssignment = CAN_CONFIG_FIFO,
		.FilterBank = CAN_CONFIG_FILTER_BANK,
		.FilterIdHigh = id_filter,
		.FilterIdLow = CAN_IGNORE_EXTENDED_ID_LOW,
		.FilterMaskIdHigh = mask_filter,
		.FilterMaskIdLow = CAN_IGNORE_EXTENDED_MASK_LOW,
	};

	HAL_CAN_ConfigFilter(can, &filter_for_config);

	// For commands since there are only two we'll use direct ID checks

	id_filter = config->node_id | CAN_COMMAND_LIGHT_ID_BASE;
	id_filter = (id_filter << 5) | CAN_IGNORE_EXTENDED_ID_HIGH;

	CAN_FilterTypeDef filter_for_light = {
		.FilterMode = CAN_FILTERMODE_IDLIST,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterActivation = CAN_FILTER_ENABLE,		// Always enable
		.FilterFIFOAssignment = CAN_COMMAND_FIFO,
		.FilterBank = CAN_COMMAND_LIGHT_FILTER_BANK,
		.FilterIdHigh = id_filter,
		.FilterIdLow = CAN_IGNORE_EXTENDED_ID_LOW,
		.FilterMaskIdHigh = 0,
		.FilterMaskIdLow = 0,
	};

	HAL_CAN_ConfigFilter(can, &filter_for_light);

	id_filter = config->servo.surface | CAN_COMMAND_ANGLE_ID_BASE;
	id_filter = (id_filter << 5) | CAN_IGNORE_EXTENDED_ID_HIGH;

	CAN_FilterTypeDef filter_for_servo_angle = {
		.FilterMode = CAN_FILTERMODE_IDLIST,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterActivation = CAN_FILTER_DISABLE,		// Assume disabled
		.FilterFIFOAssignment = CAN_COMMAND_FIFO,
		.FilterBank = CAN_COMMAND_ANGLE_FILTER_BANK,
		.FilterIdHigh = id_filter,
		.FilterIdLow = CAN_IGNORE_EXTENDED_ID_LOW,
		.FilterMaskIdHigh = 0,
		.FilterMaskIdLow = 0,
	};
	if (config->general.driving_servo) filter_for_servo_angle.FilterActivation = CAN_FILTER_ENABLE;

	HAL_CAN_ConfigFilter(can, &filter_for_servo_angle);

	id_filter = (CAN_READING_ANGLE_ID_BASE << 5) | CAN_IGNORE_EXTENDED_ID_HIGH;
	mask_filter = (CAN_READING_ANGLE_MASK << 5) | CAN_IGNORE_EXTENDED_MASK_HIGH;

	CAN_FilterTypeDef filter_for_surface_angles = {
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterActivation = CAN_FILTER_DISABLE,
		.FilterFIFOAssignment = CAN_COMMAND_FIFO,
		.FilterBank = CAN_READING_ANGLE_BANK,
		.FilterIdHigh = id_filter,
		.FilterIdLow = CAN_IGNORE_EXTENDED_ID_LOW,
		.FilterMaskIdHigh = mask_filter,
		.FilterMaskIdLow = CAN_IGNORE_EXTENDED_MASK_LOW,
	};
	if (config->general.operating_indicator) filter_for_surface_angles.FilterActivation = CAN_FILTER_ENABLE;

	HAL_CAN_ConfigFilter(can, &filter_for_surface_angles);

	HAL_CAN_Start(can);
}
