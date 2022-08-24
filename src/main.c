/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <main.h>

#include <ws2815.h>

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

static ws2815_desc_t ws1;
static ws2815_desc_t ws2;

volatile uint32_t led_work = 0U;

#define SHAPE_LEN (7)

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} color_t;

/*color_t shape[SHAPE_LEN] = {{0, 0, 0},	 {16, 0, 0},  {64, 0, 0},  {128,
   0, 0}, {0, 192, 0}, {0, 0, 255}, {0, 192, 0}, {128, 0, 0},
			    {64, 0, 0},	 {16, 0, 0},  {0, 0, 0}};*/

color_t shape[SHAPE_LEN] = {{0, 0, 0},	  {64, 16, 0},	{128, 32, 0},
			    {255, 64, 0}, {128, 32, 0}, {64, 16, 0},
			    {0, 0, 0}};

/*color_t shape[SHAPE_LEN] = {{0, 0, 0},	 {0, 64, 0}, {0, 128, 0}, {0,
   255, 0}, {0, 128, 0}, {0, 64, 0}, {0, 0, 0}};*/

/**
 * @brief  The application entry point.
 * @retval int
 */
int
main(void)
{
	/* MCU Configuration */

	/* Reset of all peripherals, Initializes the Flash interface and the
	 * Systick.
	 */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_CAN_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();

	ws2815_init(&ws1, &htim1, TIM_CHANNEL_1);
	ws2815_init(&ws2, &htim1, TIM_CHANNEL_2);

	ws2815_set_brightness(&ws1, 255);
	ws2815_set_brightness(&ws2, 255);

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

	ws2815_clear(&ws1);
	ws2815_clear(&ws2);
	while (!ws2815_show(&ws1)) {
		/* do nothing */
	}
	while (!ws2815_show(&ws2)) {
		/* do nothing */
	}

	int c = -SHAPE_LEN;

	HAL_GPIO_WritePin(GPIOA, GP1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GP2_Pin, GPIO_PIN_SET);

	/* Infinite loop */
	while (1) {
		if (led_work >= 250U) {
			HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
			int i;
			for (i = 0; i < SHAPE_LEN; i++) {
				if ((c + i >= 0) && (c + i < (int)NUM_PIXELS)) {
					ws2815_setRGB(&ws2, (size_t)(c + i),
						      shape[i].r, shape[i].g,
						      shape[i].b);

					ws2815_setRGB(&ws1, (size_t)(c + i),
						      shape[i].r, shape[i].g,
						      shape[i].b);
				}
			}

			while (!ws2815_show(&ws1)) {
				/* do nothing */
			}
			HAL_Delay(2);
			while (!ws2815_show(&ws2)) {
				/* do nothing */
			}

			c++;
			if (c > (int)(NUM_PIXELS + SHAPE_LEN)) {
				c = -SHAPE_LEN;
			}

			HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);

			led_work = 0U;
		}
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void
SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified
	 * parameters in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
				      RCC_CLOCKTYPE_SYSCLK |
				      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) !=
	    HAL_OK) {
		Error_Handler();
	}

	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void
MX_CAN_Init(void)
{
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 16;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_TIM1_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 80 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) !=
	    HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) !=
	    HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) !=
	    HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) !=
	    HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_TIM3_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 7200 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) !=
	    HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void
HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
	if (htim_base->Instance == TIM1) {
		/* Peripheral clock enable */
		__HAL_RCC_TIM1_CLK_ENABLE();

		/* TIM1 DMA Init */
		/* TIM1_CH1 Init */
		ws1.dma_handle.Instance = DMA1_Channel2;
		ws1.dma_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
		ws1.dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
		ws1.dma_handle.Init.MemInc = DMA_MINC_ENABLE;
		ws1.dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		ws1.dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		ws1.dma_handle.Init.Mode = DMA_CIRCULAR;
		ws1.dma_handle.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		if (HAL_DMA_Init(&ws1.dma_handle) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(htim_base, hdma[TIM_DMA_ID_CC1], ws1.dma_handle);

		/* TIM1_CH2 Init */
		ws2.dma_handle.Instance = DMA1_Channel3;
		ws2.dma_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
		ws2.dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
		ws2.dma_handle.Init.MemInc = DMA_MINC_ENABLE;
		ws2.dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		ws2.dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		ws2.dma_handle.Init.Mode = DMA_CIRCULAR;
		ws2.dma_handle.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		if (HAL_DMA_Init(&ws2.dma_handle) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(htim_base, hdma[TIM_DMA_ID_CC2], ws2.dma_handle);
	} else if (htim_base->Instance == TIM3) {
		/* Peripheral clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();
		/* TIM3 interrupt Init */
		HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}
}

/**
 * @brief This function handles DMA1 channel2 global interrupt.
 */
void
DMA1_Channel2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&ws1.dma_handle);
}

/**
 * @brief This function handles DMA1 channel3 global interrupt.
 */
void
DMA1_Channel3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&ws2.dma_handle);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_USART1_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * Enable DMA controller clock
 */
static void
MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void
MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GP2_Pin | GP1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : GP2_Pin GP1_Pin */
	GPIO_InitStruct.Pin = GP2_Pin | GP1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LED1_Pin LED2_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
__attribute__((noreturn)) void
Error_Handler(void)
{
	/* User can add his own implementation to report the HAL error
	 * return state */
	__disable_irq();
	while (1) {
	}
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line
 * number where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void
assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name
	   and line number, ex: printf("Wrong parameters value: file %s
	   on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
