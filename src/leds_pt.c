/**
 * @file leds.c
 * @author Алексей Хохлов <root@amper.me>
 * @copyright WTFPL License
 * @date 2022
 * @brief Работа с светодиодными лентами
 */

#include <main.h>

#include <leds.h>
#include <proto/vesc_proto.h>

#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define GP2_Pin GPIO_PIN_10
#define GP2_GPIO_Port GPIOA
#define GP1_Pin GPIO_PIN_11
#define GP1_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB

static TIM_HandleTypeDef htim1;

typedef struct {
	__IO uint32_t *tim_ccr;
	leds_mode_t mode;
	uint8_t brightness;
	uint8_t period;
	uint8_t __reserved[2U];
} leds_desc_t;

static void pwm_timer_init(void);
static void leds_gpio_init(void);

void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);

typedef void (*led_mode_funcion_t)(leds_desc_t *);

static void led_mode_fading(leds_desc_t *ld);
static void led_mode_static(leds_desc_t *ld);
static void led_mode_blinking(leds_desc_t *ld);
static void led_mode_flashing(leds_desc_t *ld);

static led_mode_funcion_t leds_modes[LEDS_MODE_MAX] = {
    [LEDS_MODE_FADING] = led_mode_fading,
    [LEDS_MODE_STATIC_COLOR] = led_mode_static,
    [LEDS_MODE_RUNNING_SHAPE] = led_mode_static,
    [LEDS_MODE_BLINKING] = led_mode_blinking,
    [LEDS_MODE_FLASHING] = led_mode_flashing,
    [LEDS_MODE_POLICE] = led_mode_flashing};

static leds_desc_t leds[2U] = {
    {.mode = LEDS_MODE_STATIC_COLOR, .period = 20U, .brightness = 32U},
    {.mode = LEDS_MODE_STATIC_COLOR, .period = 20U, .brightness = 32U}};

void
leds_init(void)
{
	/* таймер с PWM */
	pwm_timer_init();
	leds_gpio_init();

	HAL_GPIO_WritePin(GPIOA, GP1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GP2_Pin, GPIO_PIN_SET);

	leds[0U].tim_ccr = &htim1.Instance->CCR1;
	leds[1U].tim_ccr = &htim1.Instance->CCR2;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void
leds_msp_init(TIM_HandleTypeDef *htim_base)
{
	/* Peripheral clock enable */
	__HAL_RCC_TIM1_CLK_ENABLE();

	(void)htim_base;
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

	leds[id].mode = (leds_mode_t)mode;
}

void
change_leds_color(uint8_t id, uint8_t r, uint8_t g, uint8_t b)
{
	/* do nothing */
	(void)id;
	(void)r;
	(void)g;
	(void)b;
	return;
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

	leds[id].brightness = brightness;
}

static void
led_mode_static(leds_desc_t *ld)
{
	*ld->tim_ccr = ld->brightness;
	return;
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

	*ld->tim_ccr = (uint32_t)(ld->brightness * fade_value);
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
		*ld->tim_ccr = ld->brightness;
	} else {
		*ld->tim_ccr = 0U;
	}
}

static void
led_mode_flashing(leds_desc_t *ld)
{
	static int flash_step = 0;
#define SPEED 5

	switch (flash_step) {
	case (1 * SPEED):
	case (3 * SPEED):
	case (5 * SPEED):
	case (7 * SPEED): {
		*ld->tim_ccr = ld->brightness;

		break;
	}

	case 0:
	case (2 * SPEED):
	case (4 * SPEED):
	case (6 * SPEED):
		*ld->tim_ccr = 0U;
		break;

	case (8 * SPEED):
		*ld->tim_ccr = 0U;
		flash_step = 0;
		break;

	default:
		break;
	}

	flash_step++;
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
	htim1.Init.Prescaler = 50;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 255;
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

static void
leds_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

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

void
HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (htim->Instance == TIM1) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**TIM1 GPIO Configuration
		PA8     ------> TIM1_CH1
		PA9     ------> TIM1_CH2
		*/
		GPIO_InitStruct.Pin = PWM1_Pin | PWM2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}
