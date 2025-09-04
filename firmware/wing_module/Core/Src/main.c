/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wing_module_config.h"
#include "can_wrapper.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ads131m03.h"
#include "led_control.h"
#include "surface_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
struct WingModuleConfig config;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Since we have two FIFOs, I'll use one for configuration messages and the other for commands to the module
void can_process_config_message(struct WingModuleConfig* config, CAN_HandleTypeDef* can) {
	int messages = HAL_CAN_GetRxFifoFillLevel(can, CAN_CONFIG_FIFO);


	for (int i = 0; i < messages; i++) {
		CAN_RxHeaderTypeDef header;
		uint8_t buffer[8];
		HAL_CAN_GetRxMessage(can, CAN_CONFIG_FIFO, &header, buffer);

		unsigned int config_type = (header.StdId >> 5) & 0x7;

		uint8_t reference_buffer[8];
		size_t length_to_consider = 0;

		switch (config_type) {
		case CONFIG_MESSAGE_GENERAL:
			length_to_consider = can_pack_node_general_config_command(reference_buffer, config->general);
			break;
		case CONFIG_MESSAGE_SERVO:
			length_to_consider = can_pack_servo_config_command(reference_buffer, config->servo);
			break;
		case CONFIG_MESSAGE_TORSION:
			length_to_consider = can_pack_strain_gauge_config_command(reference_buffer, config->torsion);
			break;
		case CONFIG_MESSAGE_STRAIN:
			length_to_consider = can_pack_strain_gauge_config_command(reference_buffer, config->strain);
			break;
		case CONFIG_MESSAGE_INDICATOR:
			length_to_consider = can_pack_indicator_config_command(reference_buffer, config->indicator);
			break;
		case CONFIG_MESSAGE_LIDAR:
			length_to_consider = can_pack_lidar_config_command(reference_buffer, config->lidar);
			break;
		default:
			continue; // Ignore unrecognized messages
		}

		bool matches_old = memcmp(reference_buffer, buffer, length_to_consider) == 0;
		bool need_this_config = (config->configuration_needed & (1 << config_type)) != 0;
		if (matches_old && !need_this_config) continue; // Skip updates if unnecessary repetition

		// Mask out the received configuration
		config->configuration_needed &= ~(1 << config_type);

		// Need to update CAN filters if the configuration changes
		switch (config_type) {
		case CONFIG_MESSAGE_GENERAL:
			can_unpack_node_general_config_command(buffer, &config->general);

			// Reset configuration
			config->configuration_needed = 0;
			if (config->general.driving_servo) 			config->configuration_needed |= 1 << CONFIG_MESSAGE_SERVO;
			if (config->general.measuring_torsion) 		config->configuration_needed |= 1 << CONFIG_MESSAGE_TORSION;
			if (config->general.measuring_strain) 		config->configuration_needed |= 1 << CONFIG_MESSAGE_STRAIN;
			if (config->general.operating_indicator) 	config->configuration_needed |= 1 << CONFIG_MESSAGE_INDICATOR;
			if (config->general.measuring_lidar) 		config->configuration_needed |= 1 << CONFIG_MESSAGE_LIDAR;
			surface_stop(); // Stop driving surface

			can_update_node_filters(config, can);
			break;
		case CONFIG_MESSAGE_SERVO:
			can_unpack_servo_config_command(buffer, &config->servo);

			struct SurfaceConfiguration servo_config = {
				.misalignment_alarm_sec = config->servo.misalignment_alarm_sec,
				.position_tolerance = config->servo.position_tolerance_half_percent * 0.005,

				.pot_adc = &hadc1,
				.potentiometer_bottom = config->servo.potentiometer_bottom,
				.potentiometer_top = config->servo.potentiometer_top,

				.scheme = config->servo.scheme,

				.servo_bottom_us = config->servo.servo_bottom_us,
				.servo_top_us = config->servo.servo_top_us,
				.servo_timer = &htim3,
				.servo_timer_channel = TIM_CHANNEL_2,
			};

			surface_start(servo_config);
			break;
		case CONFIG_MESSAGE_TORSION:
			can_unpack_strain_gauge_config_command(buffer, &config->torsion);

			struct ADSChannelProcessingConfig temp_torsion = {
				.buffer_length = 64 * (config->torsion.buffer_depth_64_samples + 1),
				.gain = config->torsion.gain,
				.osr = config->torsion.osr,
				.zero_point = config->torsion.zero_point,
				.scaling_factor = config->torsion.scaling_factor,
			};

			ads_configure_channel(ADS_CHANNEL_TORSION, temp_torsion);
			break;
		case CONFIG_MESSAGE_STRAIN:
			can_unpack_strain_gauge_config_command(buffer, &config->strain);

			struct ADSChannelProcessingConfig temp_strain = {
				.buffer_length = 64 * (config->strain.buffer_depth_64_samples + 1),
				.gain = config->strain.gain,
				.osr = config->strain.osr,
				.zero_point = config->strain.zero_point,
				.scaling_factor = config->strain.scaling_factor,
			};

			ads_configure_channel(ADS_CHANNEL_STRAIN, temp_strain);
			break;
		case CONFIG_MESSAGE_INDICATOR:
			can_unpack_indicator_config_command(buffer, &config->indicator);
			break;
		case CONFIG_MESSAGE_LIDAR:
			can_unpack_lidar_config_command(buffer, &config->lidar);
			break;
		}
	}
}

void can_process_control_message(struct WingModuleConfig* config, CAN_HandleTypeDef* can) {

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	config.node_id = 0;
	const bool ADDRESS_ACTIVE = true;
	if (HAL_GPIO_ReadPin(ADDR_0_GPIO_Port, ADDR_0_Pin) == ADDRESS_ACTIVE) config.node_id += 1;
	if (HAL_GPIO_ReadPin(ADDR_1_GPIO_Port, ADDR_1_Pin) == ADDRESS_ACTIVE) config.node_id += 2;
	if (HAL_GPIO_ReadPin(ADDR_2_GPIO_Port, ADDR_2_Pin) == ADDRESS_ACTIVE) config.node_id += 4;
	if (HAL_GPIO_ReadPin(ADDR_3_GPIO_Port, ADDR_3_Pin) == ADDRESS_ACTIVE) config.node_id += 8;
	if (HAL_GPIO_ReadPin(ADDR_4_GPIO_Port, ADDR_4_Pin) == ADDRESS_ACTIVE) config.node_id += 16;
	printf("Recorded node ID as %d\r\n", config.node_id);

	config.configuration_needed = (1 << CONFIG_MESSAGE_GENERAL); // Mark need for main configuration
	// Perhaps in the future we can read in previous configuration from flash here

	can_update_node_filters(&config, &hcan);

	struct ADSConfig ads_config = {
		.bus = &hspi1,
		.cs_pin = ADS_CS_Pin,
		.cs_port = ADS_CS_GPIO_Port,
		.reset_pin = ADS_RESET_Pin,
		.reset_port = ADS_RESET_GPIO_Port,
		.data_ready_pin = ADS_DATA_RDY_Pin,
		.data_ready_port = ADS_DATA_RDY_GPIO_Port,
	};
	ads_begin(ads_config);

	struct LEDControl led = {
		.error_light_on = config.configuration_needed != 0,
		.heart_beat_max_duty = UINT8_MAX,
		.heart_beat_min_duty = 0,
		.new_pulse_data = true,
		.pulse_count = 3, // Run some flashes to start
		.pulse_period_on_ms = 1000,
		.pulse_period_off_ms = 300,

		.error_timer = &htim2,
		.error_channel = LED_RED_CHANNEL,
		.status_timer = &htim2,
		.status_channel = LED_WHITE_CHANNEL,
	};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		const uint32_t CURRENT_TICK = HAL_GetTick();

		ads_update_channels();
		surface_control_loop();

		static uint32_t tick_mark_broadcast_ms = 0;
		if (CURRENT_TICK > tick_mark_broadcast_ms) {
			tick_mark_broadcast_ms = CURRENT_TICK + config.general.update_period_ms;
			uint8_t tx_buffer[8];

			if (config.general.measuring_strain || config.general.measuring_torsion) {
				struct CANLoadBroadcast msg = {
					.strain_reading = 0,
					.torsion_reading = 0,
				};
				if (config.general.measuring_strain) msg.strain_reading = ads_read_channel(ADS_CHANNEL_STRAIN);
				if (config.general.measuring_torsion) msg.torsion_reading = ads_read_channel(ADS_CHANNEL_TORSION);
				can_pack_load_broadcast(tx_buffer, msg);

				printf("T: %.3f\tS: %.3f\r\n", msg.torsion_reading, msg.strain_reading);

				CAN_TxHeaderTypeDef strain_gauge_header = {
					.StdId = CAN_BROADCAST_LOAD_ID_BASE | config.node_id,
					.DLC = sizeof(struct CANLoadBroadcast),
					.ExtId = 0,
					.IDE = CAN_ID_STD,
					.RTR = CAN_RTR_DATA,
					.TransmitGlobalTime = DISABLE,
				};
				uint32_t mailbox;
				if (HAL_CAN_AddTxMessage(&hcan, &strain_gauge_header, tx_buffer, &mailbox) == HAL_OK) {
					printf("SENT STRAIN READINGS OVER CAN\r\n");
				}
			}

			if (config.general.driving_servo) {
				struct CANSurfaceBroadcast msg = {
					.reading_thousandths = 0,
					.surface_not_following = false,
				};
				surface_prepare_broadcast(&msg);
				can_pack_surface_broadcast(tx_buffer, msg);

				printf("Angle: %.3f\tAlarm: %d\r\n", msg.reading_thousandths / 1000.0, msg.surface_not_following);

				CAN_TxHeaderTypeDef angle_header = {
					.StdId = CAN_BROADCAST_ANGLE_ID | config.servo.surface,
					.DLC = sizeof(struct CANSurfaceBroadcast),
					.ExtId = 0,
					.IDE = CAN_ID_STD,
					.RTR = CAN_RTR_DATA,
					.TransmitGlobalTime = DISABLE,
				};
				uint32_t mailbox;
				if (HAL_CAN_AddTxMessage(&hcan, &angle_header, tx_buffer, &mailbox) == HAL_OK) {
					printf("SENT ANGLE OVER CAN\r\n");
				}
			}
		}

		while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_CONFIG_FIFO) > 0) {
			can_process_config_message(&config, &hcan);
		}

		while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_COMMAND_FIFO) > 0) {
			uint8_t command_buffer[8];
			CAN_RxHeaderTypeDef command_header;

			HAL_CAN_GetRxMessage(&hcan, CAN_COMMAND_FIFO, &command_header, command_buffer);
			printf("Received command via filter %ld\r\n", command_header.FilterMatchIndex);

			switch (command_header.FilterMatchIndex) {
			case CAN_COMMAND_LIGHT_FILTER_NUMBER:
				struct CANLightCommand light_command = {0};
				can_unpack_light_command(command_buffer, &light_command);

				led.pulse_count = light_command.pulse_count;
				led.pulse_period_off_ms = light_command.pulse_period_off_ms;
				led.pulse_period_on_ms = light_command.pulse_period_on_ms;
				led.heart_beat_max_duty = light_command.heart_beat_max_duty;
				led.heart_beat_min_duty = light_command.heart_beat_min_duty;
				led.new_pulse_data = true;
				break;
			case CAN_COMMAND_ANGLE_FILTER_BANK:
				struct CANSurfaceCommand surface_command;
				can_unpack_surface_command(command_buffer, &surface_command);

				surface_update_command(surface_command);
				break;
			default:
				break;
			}
		}

		led.error_light_on = config.configuration_needed != 0; // Illuminate error if lacking configuration
		// TODO: Add error lighting if the servo is failing to hit the commands?
		operate_led(&led);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1); // Recommended before operating
  HAL_Delay(1); // Some time for the calibration to occur
  HAL_ADC_Start(&hadc1);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 20;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = ENABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 127;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADS_RESET_Pin|ADS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ADS_RESET_Pin ADS_CS_Pin */
  GPIO_InitStruct.Pin = ADS_RESET_Pin|ADS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADS_DATA_RDY_Pin */
  GPIO_InitStruct.Pin = ADS_DATA_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADS_DATA_RDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADDR_2_Pin ADDR_1_Pin ADDR_3_Pin ADDR_4_Pin */
  GPIO_InitStruct.Pin = ADDR_2_Pin|ADDR_1_Pin|ADDR_3_Pin|ADDR_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_CLK_IN_Pin */
  GPIO_InitStruct.Pin = ADC_CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADDR_0_Pin */
  GPIO_InitStruct.Pin = ADDR_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ADDR_0_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
