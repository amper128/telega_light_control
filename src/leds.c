/**
 * @file leds.c
 * @author Алексей Хохлов <root@amper.me>
 * @copyright WTFPL License
 * @date 2022
 * @brief Работа с светодиодными лентами
 */

#include <main.h>

#include <ws2815.h>

#include <leds.h>
#include <proto/vesc_proto.h>

#define GP1_Pin GPIO_PIN_12
#define GP1_GPIO_Port GPIOB
#define GP2_Pin GPIO_PIN_13
#define GP2_GPIO_Port GPIOB
#define GP3_Pin GPIO_PIN_14
#define GP3_GPIO_Port GPIOB
#define GP4_Pin GPIO_PIN_15
#define GP4_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_10
#define PWM3_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_11
#define PWM4_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB

#define SHAPE_LEN (7)

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} color_t;

static TIM_HandleTypeDef htim1;

typedef struct {
	ws2815_desc_t ws;
	color_t color;
	uint8_t period;
	leds_mode_t mode;
} leds_desc_t;

static void pwm_timer_init(void);
static void leds_gpio_init(void);

void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);

typedef void (*led_mode_funcion_t)(leds_desc_t *);

static void led_mode_fading(leds_desc_t *ld);
static void led_mode_static(leds_desc_t *ld);
static void led_mode_running_shape(leds_desc_t *ld);
static void led_mode_blinking(leds_desc_t *ld);
static void led_mode_flashing(leds_desc_t *ld);
static void led_mode_fso(leds_desc_t *ld);

static led_mode_funcion_t leds_modes[LEDS_MODE_MAX] = {
    [LEDS_MODE_FADING] = led_mode_fading,
    [LEDS_MODE_STATIC_COLOR] = led_mode_static,
    [LEDS_MODE_RUNNING_SHAPE] = led_mode_running_shape,
    [LEDS_MODE_BLINKING] = led_mode_blinking,
    [LEDS_MODE_FLASHING] = led_mode_flashing,
    [LEDS_MODE_POLICE] = led_mode_fso};

static leds_desc_t leds[4U] = {
    {.color = {64U, 32U, 0U}, .mode = LEDS_MODE_FADING, .period = 20U},
    {.color = {64U, 0U, 0U}, .mode = LEDS_MODE_STATIC_COLOR, .period = 20U},
    {.color = {64U, 32U, 0U}, .mode = LEDS_MODE_FADING, .period = 20U},
    {.color = {64U, 0U, 0U}, .mode = LEDS_MODE_STATIC_COLOR, .period = 20U}};

void
leds_init(void)
{
	/* таймер с PWM */
	pwm_timer_init();
	leds_gpio_init();

	leds[0U].ws.pixels = 46U;
	leds[1U].ws.pixels = 8U;
	leds[2U].ws.pixels = 8U;
	leds[3U].ws.pixels = 8U;

	ws2815_init(&leds[0U].ws, &htim1, TIM_CHANNEL_1);
	ws2815_init(&leds[1U].ws, &htim1, TIM_CHANNEL_2);
	ws2815_init(&leds[2U].ws, &htim1, TIM_CHANNEL_3);
	ws2815_init(&leds[3U].ws, &htim1, TIM_CHANNEL_4);

	ws2815_set_brightness(&leds[0U].ws, 255);
	ws2815_set_brightness(&leds[1U].ws, 255);
	ws2815_set_brightness(&leds[2U].ws, 255);
	ws2815_set_brightness(&leds[3U].ws, 255);

	ws2815_clear(&leds[0U].ws);
	ws2815_clear(&leds[1U].ws);

	HAL_GPIO_WritePin(GPIOA, GP1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GP2_Pin, GPIO_PIN_SET);

	while (!ws2815_show(&leds[0U].ws)) {
		/* do nothing */
	}
	while (!ws2815_show(&leds[1U].ws)) {
		/* do nothing */
	}
}

void
leds_msp_init(TIM_HandleTypeDef *htim_base)
{
	/* Peripheral clock enable */
	__HAL_RCC_TIM1_CLK_ENABLE();

	/* TIM1 DMA Init */
	/* TIM1_CH1 Init */
	leds[0U].ws.dma_handle.Instance = DMA1_Channel2;
	leds[0U].ws.dma_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
	leds[0U].ws.dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
	leds[0U].ws.dma_handle.Init.MemInc = DMA_MINC_ENABLE;
	leds[0U].ws.dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	leds[0U].ws.dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	leds[0U].ws.dma_handle.Init.Mode = DMA_CIRCULAR;
	leds[0U].ws.dma_handle.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	if (HAL_DMA_Init(&leds[0U].ws.dma_handle) != HAL_OK) {
		Error_Handler();
	}

	__HAL_LINKDMA(htim_base, hdma[TIM_DMA_ID_CC1], leds[0U].ws.dma_handle);

	/* TIM1_CH2 Init */
	leds[1U].ws.dma_handle.Instance = DMA1_Channel3;
	leds[1U].ws.dma_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
	leds[1U].ws.dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
	leds[1U].ws.dma_handle.Init.MemInc = DMA_MINC_ENABLE;
	leds[1U].ws.dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	leds[1U].ws.dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	leds[1U].ws.dma_handle.Init.Mode = DMA_CIRCULAR;
	leds[1U].ws.dma_handle.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	if (HAL_DMA_Init(&leds[1U].ws.dma_handle) != HAL_OK) {
		Error_Handler();
	}

	__HAL_LINKDMA(htim_base, hdma[TIM_DMA_ID_CC2], leds[1U].ws.dma_handle);
}

void leds_work(void);

void
change_leds_mode(uint8_t id, uint8_t mode)
{
	if (id > 1U) {
		return;
	}

	if (mode >= (uint8_t)LEDS_MODE_MAX) {
		return;
	}

	if (leds[id].mode != (leds_mode_t)mode) {
		leds[id].mode = (leds_mode_t)mode;

		ws2815_clear(&leds[id].ws);
	}
}

void
change_leds_color(uint8_t id, uint8_t r, uint8_t g, uint8_t b)
{
	if (id > 1U) {
		return;
	}

	leds[id].color.r = r;
	leds[id].color.g = g;
	leds[id].color.b = b;
}

void
change_leds_period(uint8_t id, uint8_t period)
{
	if (id > 1U) {
		return;
	}

	leds[id].period = period;
}

void
change_leds_brightness(uint8_t id, uint8_t brightness)
{
	if (id > 1U) {
		return;
	}

	ws2815_set_brightness(&leds[id].ws, brightness);
}

static void
led_mode_static(leds_desc_t *ld)
{
	size_t i;
	for (i = 0; i < ld->ws.pixels; i++) {
		ws2815_setRGB(&ld->ws, (size_t)(i), ld->color.r, ld->color.g,
			      ld->color.b);
	}

	while (!ws2815_show(&ld->ws)) {
		/* do nothing */
	}
}

static void
led_mode_running_shape(leds_desc_t *ld)
{
	static int shape_pos = -SHAPE_LEN;
	static const float shape_div[SHAPE_LEN] = {0.0f, 0.25f, 0.5f, 1.0f,
						   0.5f, 0.25f, 0.0f};

	int i;
	for (i = 0; i < SHAPE_LEN; i++) {
		if ((shape_pos + i >= 0) &&
		    (shape_pos + i < (int)ld->ws.pixels)) {
			color_t p;

			p.r = (uint8_t)(ld->color.r * shape_div[i]);
			p.g = (uint8_t)(ld->color.g * shape_div[i]);
			p.b = (uint8_t)(ld->color.b * shape_div[i]);

			ws2815_setRGB(&ld->ws, (size_t)(shape_pos + i), p.r,
				      p.g, p.b);
		}
	}

	shape_pos++;
	if (shape_pos > (int)(ld->ws.pixels + SHAPE_LEN)) {
		shape_pos = -SHAPE_LEN;
	}

	while (!ws2815_show(&ld->ws)) {
		/* do nothing */
	}
}

static void
led_mode_fading(leds_desc_t *ld)
{
	static int fade_step = 0;
	static float fade_value = 0.0f;

	if (fade_step == 0) {
		if (1.0f - fade_value <= 0.001f) {
			fade_step = 1;
		} else {
			fade_value += 0.02f;
		}
	} else {
		if (fade_value >= 0.001f) {
			fade_value -= 0.02f;
		} else {
			fade_step = 0;
		}
	}

	size_t i;
	for (i = 0; i < ld->ws.pixels; i++) {
		color_t p;

		p.r = (uint8_t)(ld->color.r * fade_value);
		p.g = (uint8_t)(ld->color.g * fade_value);
		p.b = (uint8_t)(ld->color.b * fade_value);

		ws2815_setRGB(&ld->ws, (size_t)(i), p.r, p.g, p.b);
	}

	while (!ws2815_show(&ld->ws)) {
		/* do nothing */
	}
}

static void
led_mode_blinking(leds_desc_t *ld)
{
	static size_t blink_step = 0;
	static int blink_value;

	if (blink_step < ld->period) {
		blink_step++;
	} else {
		blink_step = 0U;
		if (blink_value == 0) {
			blink_value = 1;
		} else {
			blink_value = 0;
		}
	}

	if (blink_value) {
		size_t i;
		for (i = 0; i < ld->ws.pixels; i++) {
			ws2815_setRGB(&ld->ws, (size_t)(i), ld->color.r,
				      ld->color.g, ld->color.b);
		}
	} else {
		ws2815_clear(&ld->ws);
	}

	while (!ws2815_show(&ld->ws)) {
		/* do nothing */
	}
}

static void
led_mode_flashing(leds_desc_t *ld)
{
	static int flash_step = 0;
#define SPEED 5

	switch (flash_step) {
	case (1 * SPEED):
		/* first flash */
	case (3 * SPEED): {
		size_t i;
		for (i = 0; i < ld->ws.pixels / 2U; i++) {
			ws2815_setRGB(&ld->ws, (size_t)(i), ld->color.r,
				      ld->color.g, ld->color.b);
		}

		break;
	}

	case (5 * SPEED):
	case (7 * SPEED): {
		size_t i;
		for (i = ld->ws.pixels / 2U; i < ld->ws.pixels; i++) {
			ws2815_setRGB(&ld->ws, (size_t)(i), ld->color.r,
				      ld->color.g, ld->color.b);
		}

		break;
	}

	case 0:
		/* first blank */
	case (2 * SPEED):
		/* second blank */
	case (4 * SPEED):

	case (6 * SPEED):
		ws2815_clear(&ld->ws);
		break;

	case (8 * SPEED):
		ws2815_clear(&ld->ws);
		flash_step = 0;
		break;

	default:
		break;
	}

	flash_step++;

	while (!ws2815_show(&ld->ws)) {
		/* do nothing */
	}
}

static void
led_mode_fso(leds_desc_t *ld)
{
	static int flash_step = 0;
#define FSO_SPEED (4)

	switch (flash_step) {
	case (1 * FSO_SPEED):
		/* first flash */
	case (3 * FSO_SPEED): {
		size_t i;
		for (i = 0; i < ld->ws.pixels / 2U; i++) {
			color_t p;

			p.r = 255U;
			p.g = 0U;
			p.b = 0U;

			ws2815_setRGB(&ld->ws, (size_t)(i), p.r, p.g, p.b);
		}

		break;
	}

	case (5 * FSO_SPEED):
	case (7 * FSO_SPEED): {
		size_t i;
		for (i = ld->ws.pixels / 2U; i < ld->ws.pixels; i++) {
			color_t p;

			p.r = 0U;
			p.g = 0U;
			p.b = 255U;

			ws2815_setRGB(&ld->ws, (size_t)(i), p.r, p.g, p.b);
		}

		break;
	}

	case 0:
		/* first blank */
	case (2 * FSO_SPEED):
		/* second blank */
	case (4 * FSO_SPEED):

	case (6 * FSO_SPEED):
		ws2815_clear(&ld->ws);
		break;

	case (8 * FSO_SPEED):
		ws2815_clear(&ld->ws);
		flash_step = 0;
		break;

	default:
		break;
	}

	flash_step++;

	while (!ws2815_show(&ld->ws)) {
		/* do nothing */
	}
}

void
leds_work(void)
{
	HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);

	size_t w;
	for (w = 0U; w < 2U; w++) {
		if (leds[w].mode >= LEDS_MODE_MAX) {
			continue;
		}
		leds_modes[leds[w].mode](&leds[w]);

		HAL_Delay(2U);
	}

	HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void
pwm_timer_init(void)
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
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) !=
	    HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) !=
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

static void
leds_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(
	    GPIOB, GP1_Pin | GP2_Pin | GP3_Pin | GP4_Pin | LED1_Pin | LED2_Pin,
	    GPIO_PIN_RESET);

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
 * @brief This function handles DMA1 channel2 global interrupt.
 */
void
DMA1_Channel2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&leds[0U].ws.dma_handle);
}

/**
 * @brief This function handles DMA1 channel3 global interrupt.
 */
void
DMA1_Channel3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&leds[1U].ws.dma_handle);
}

void
HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (htim->Instance == TIM1) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**TIM1 GPIO Configuration
		PA8     ------> TIM1_CH1
		PA9     ------> TIM1_CH2
		PA10     ------> TIM1_CH3
		PA11     ------> TIM1_CH4
		*/
		GPIO_InitStruct.Pin = PWM1_Pin | PWM2_Pin | PWM3_Pin | PWM4_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}
