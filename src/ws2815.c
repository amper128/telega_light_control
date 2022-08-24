/**
 * @file ws2815.c
 * @author Алексей Хохлов <root@amper.me>
 * @copyright WTFPL License
 * @date 2022
 * @brief Управление лентой WS2815
 */

#include <ws2815.h>

#include <math.h>
#include <string.h>

static volatile uint8_t PWM_HI; ///< PWM Code HI Log.1 period
static volatile uint8_t PWM_LO; ///< PWM Code LO Log.1 period

static void HSV2RGB(uint8_t hue, uint8_t sat, uint8_t val, uint8_t *_r,
		    uint8_t *_g, uint8_t *_b);
// Callbacks
static void ws2815_TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
static void ws2815_TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma);

/**
 * @addtogroup Private_entities
 * @{ */

/**
 * @brief Convert color in HSV to RGB
 * @param[in] hue HUE (color) [0..255]
 * @param[in] sat Saturation  [0..255]
 * @param[in] val Value (brightness) [0..255]
 * @param[out] _r Pointer to RED component value
 * @param[out] _g Pointer to GREEN component value
 * @param[out] _b Pointer to BLUE component value
 */
static void
HSV2RGB(uint8_t hue, uint8_t sat, uint8_t val, uint8_t *_r, uint8_t *_g,
	uint8_t *_b)
{
	if (sat == 0) { // if white color
		*_r = *_g = *_b = val;
		return;
	}
	// Float is smoother but check for FPU (Floating point unit) in your MCU
	// Otherwise it will take longer time in the code
	// FPU is in: F3/L3 and greater
	// Src: https://github.com/Inseckto/HSV-to-RGB
	float h = (float)hue / 255;
	float s = (float)sat / 255;
	float v = (float)val / 255;

	int i = (int)floorf(h * 6);
	float f = h * 6 - (float)i;
	uint8_t p = (uint8_t)(v * (1 - s) * 255.0);
	uint8_t q = (uint8_t)(v * (1 - f * s) * 255.0);
	uint8_t t = (uint8_t)(v * (1 - (1 - f) * s) * 255.0);

	switch (i % 6) {
		// Src: https://stackoverflow.com/questions/3018313
		//    uint8_t reg = hue / 43;
		//    uint8_t rem = (hue - (reg * 43)) * 6;
		//    uint8_t p = (val * (255 - sat)) >> 8;
		//    uint8_t q = (val * (255 - ((sat * rem) >> 8))) >> 8;
		//    uint8_t t = (val * (255 - ((sat * (255 - rem)) >> 8))) >>
		//    8; switch (reg) {
	case 0:
		*_r = val, *_g = t, *_b = p;
		break;
	case 1:
		*_r = q, *_g = val, *_b = p;
		break;
	case 2:
		*_r = p, *_g = val, *_b = t;
		break;
	case 3:
		*_r = p, *_g = q, *_b = val;
		break;
	case 4:
		*_r = t, *_g = p, *_b = val;
		break;
	default:
		*_r = val, *_g = p, *_b = q;
		break;
	}
}

/**
 * @brief  TIM DMA Delay Pulse complete callback.
 * @param  hdma pointer to DMA handle.
 * @retval None
 */
static void
ws2815_TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma)
{
	TIM_HandleTypeDef *htim =
	    (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
	ws2815_desc_t *desc;
	desc = __containerof(hdma, ws2815_desc_t, dma_handle);
	// if wrong handlers
	if (hdma != &desc->dma_handle || htim != desc->htim) {
		return;
	}
	if (desc->BUF_COUNTER == 0) {
		return; // if no data to transmit - return
	}

	htim->Channel = desc->active_channel;
	if (hdma->Init.Mode == DMA_NORMAL) {
		TIM_CHANNEL_STATE_SET(htim, desc->tim_channel,
				      HAL_TIM_CHANNEL_STATE_READY);
	}

	// if data transfer
	if (desc->BUF_COUNTER < NUM_PIXELS) {
		// fill second part of buffer
		for (volatile uint8_t i = 0; i < 8; i++) {
			desc->PWM_BUF[i + 24] =
			    (((desc->RGB_BUF[3 * desc->BUF_COUNTER] << i) &
			      0x80) > 0)
				? PWM_HI
				: PWM_LO;
			desc->PWM_BUF[i + 32] =
			    (((desc->RGB_BUF[3 * desc->BUF_COUNTER + 1] << i) &
			      0x80) > 0)
				? PWM_HI
				: PWM_LO;
			desc->PWM_BUF[i + 40] =
			    (((desc->RGB_BUF[3 * desc->BUF_COUNTER + 2] << i) &
			      0x80) > 0)
				? PWM_HI
				: PWM_LO;
		}
		desc->BUF_COUNTER++;
	} else if (desc->BUF_COUNTER < NUM_PIXELS + 2) { // if RET transfer
		memset((uint32_t *)&desc->PWM_BUF[PWM_BUF_SIZE / 2], 0,
		       (PWM_BUF_SIZE / 2) * sizeof(uint32_t)); // second part
		desc->BUF_COUNTER++;
	} else { // if END of transfer
		desc->BUF_COUNTER = 0;
		// STOP DMA:
		__HAL_TIM_DISABLE_DMA(htim, desc->tim_dma_cc);
		(void)HAL_DMA_Abort_IT(hdma);

		if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET) {
			/* Disable the Main Output */
			__HAL_TIM_MOE_DISABLE(htim);
		}
		/* Disable the Peripheral */
		__HAL_TIM_DISABLE(htim);
		/* Set the TIM channel state */
		TIM_CHANNEL_STATE_SET(htim, desc->tim_channel,
				      HAL_TIM_CHANNEL_STATE_READY);
		desc->loc_state = WS2815_READY;
	}
	htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}

/**
 * @brief  TIM DMA Delay Pulse half complete callback.
 * @param  hdma pointer to DMA handle.
 * @retval None
 */
static void
ws2815_TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma)
{
	TIM_HandleTypeDef *htim =
	    (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
	ws2815_desc_t *desc;
	desc = __containerof(hdma, ws2815_desc_t, dma_handle);

	// if wrong handlers
	if (hdma != &desc->dma_handle || htim != desc->htim) {
		return;
	}
	if (desc->BUF_COUNTER == 0) {
		return; // if no data to transmit - return
	}
	// if data transfer
	if (desc->BUF_COUNTER < NUM_PIXELS) {
		// fill first part of buffer
		for (volatile uint8_t i = 0; i < 8; i++) {
			desc->PWM_BUF[i] =
			    (((desc->RGB_BUF[3 * desc->BUF_COUNTER] << i) &
			      0x80) > 0)
				? PWM_HI
				: PWM_LO;
			desc->PWM_BUF[i + 8] =
			    (((desc->RGB_BUF[3 * desc->BUF_COUNTER + 1] << i) &
			      0x80) > 0)
				? PWM_HI
				: PWM_LO;
			desc->PWM_BUF[i + 16] =
			    (((desc->RGB_BUF[3 * desc->BUF_COUNTER + 2] << i) &
			      0x80) > 0)
				? PWM_HI
				: PWM_LO;
		}
		desc->BUF_COUNTER++;
	} else if (desc->BUF_COUNTER < NUM_PIXELS + 2) { // if RET transfer
		memset((uint32_t *)desc->PWM_BUF, 0,
		       (PWM_BUF_SIZE / 2) * sizeof(uint32_t)); // first part
		desc->BUF_COUNTER++;
	}
}

/** @} */ // Private

/**
 * @brief Init timer & prescalers
 * @param none
 */
void
ws2815_init(ws2815_desc_t *desc, TIM_HandleTypeDef *tim_handle,
	    uint32_t tim_channel /*, DMA_HandleTypeDef *dma_handle*/)
{
	/* Auto-calculation! */
	uint32_t APBfq; // Clock freq
	/*#ifdef APB1
		APBfq = HAL_RCC_GetPCLK1Freq();
		APBfq *= (RCC->CFGR & RCC_CFGR_PPRE1) == 0 ? 1 : 2;
	#endif
	#ifdef APB2*/
	APBfq = HAL_RCC_GetPCLK2Freq();
	APBfq *= (RCC->CFGR & RCC_CFGR_PPRE2) == 0 ? 1 : 2;
	/*#endif*/

	desc->htim = tim_handle;
	desc->tim_channel = tim_channel;

	switch (desc->tim_channel) {
	default:
	case TIM_CHANNEL_1:
		desc->tim_dma_id = TIM_DMA_ID_CC1;
		desc->tim_dma_cc = TIM_DMA_CC1;
		desc->tim_ccr = &desc->htim->Instance->CCR1;
		desc->active_channel = HAL_TIM_ACTIVE_CHANNEL_1;
		break;

	case TIM_CHANNEL_2:
		desc->tim_dma_id = TIM_DMA_ID_CC2;
		desc->tim_dma_cc = TIM_DMA_CC2;
		desc->tim_ccr = &desc->htim->Instance->CCR2;
		desc->active_channel = HAL_TIM_ACTIVE_CHANNEL_2;
		break;

	case TIM_CHANNEL_3:
		desc->tim_dma_id = TIM_DMA_ID_CC3;
		desc->tim_dma_cc = TIM_DMA_CC3;
		desc->tim_ccr = &desc->htim->Instance->CCR3;
		desc->active_channel = HAL_TIM_ACTIVE_CHANNEL_3;
		break;

	case TIM_CHANNEL_4:
		desc->tim_dma_id = TIM_DMA_ID_CC4;
		desc->tim_dma_cc = TIM_DMA_CC4;
		desc->tim_ccr = &desc->htim->Instance->CCR4;
		desc->active_channel = HAL_TIM_ACTIVE_CHANNEL_4;
		break;
	}

	APBfq /= (uint32_t)(800 * 1000); // 800 KHz - 1.25us

	desc->htim->Instance->PSC = 0; // dummy hardcode now
	desc->htim->Instance->ARR =
	    (uint16_t)(APBfq - 1);     // set timer prescaler
	desc->htim->Instance->EGR = 1; // update timer registers

	PWM_HI = (uint8_t)((APBfq * 0.56) - 1U); // Log.1 - 56% - 0.70us
	PWM_LO = (uint8_t)((APBfq * 0.28) - 1U); // Log.0 - 28% - 0.35us

	//#if INV_SIGNAL
	//    TIM_POINTER->CCER |= TIM_CCER_CC2P; // set inv ch bit
	//#else
	//    TIM_POINTER->CCER &= ~TIM_CCER_CC2P;
	//#endif
	desc->loc_state = WS2815_READY; // Set Ready Flag
	TIM_CCxChannelCmd(desc->htim->Instance, desc->tim_channel,
			  TIM_CCx_ENABLE); // Enable GPIO to IDLE state
	HAL_Delay(1);			   // Make some delay
}

/**
 * @brief Fill ALL LEDs with (0,0,0)
 * @param none
 * @note Update strip after that
 */
void
ws2815_clear(ws2815_desc_t *desc)
{
	ws2815_fillRGB(desc, 0, 0, 0);
}

/**
 * @brief Set GLOBAL LED brightness
 * @param[in] br Brightness [0..255]
 */
void
ws2815_set_brightness(ws2815_desc_t *desc, uint8_t br)
{
	desc->brightness = br;
}

/**
 * @brief Set LED with RGB color by index
 * @param[in] i LED position
 * @param[in] r Red component   [0..255]
 * @param[in] g Green component [0..255]
 * @param[in] b Blue component  [0..255]
 */
void
ws2815_setRGB(ws2815_desc_t *desc, size_t i, uint8_t r, uint8_t g, uint8_t b)
{
	// overflow protection
	if (i >= NUM_PIXELS) {
		return;
	}

	// set brightness
	uint8_t rs = (uint8_t)((r * ((uint32_t)desc->brightness + 1U)) >> 8U);

#if USE_GAMMA_CORRECTION
	uint8_t gs =
	    (uint8_t)((g * 0xB0U * ((uint32_t)desc->brightness + 1U)) >> 16U);
	uint8_t bs =
	    (uint8_t)((b * 0xF0U * ((uint32_t)desc->brightness + 1U)) >> 16U);
#else
	uint8_t gs = g * ((uint16_t)desc->brightness + 1U) >> 8U;
	uint8_t bs = b * ((uint16_t)desc->brightness + 1U) >> 8U;
#endif

	desc->RGB_BUF[3 * i] = rs;     // subpixel 1
	desc->RGB_BUF[3 * i + 1] = gs; // subpixel 2
	desc->RGB_BUF[3 * i + 2] = bs; // subpixel 3
}

/**
 * @brief Set LED with HSV color by index
 * @param[in] i LED position
 * @param[in] hue HUE (color) [0..255]
 * @param[in] sat Saturation  [0..255]
 * @param[in] val Value (brightness) [0..255]
 */
void
ws2815_setHSV(ws2815_desc_t *desc, size_t i, uint8_t hue, uint8_t sat,
	      uint8_t val)
{
	uint8_t _r, _g, _b;		       // init buffer color
	HSV2RGB(hue, sat, val, &_r, &_g, &_b); // get RGB color
	ws2815_setRGB(desc, i, _r, _g, _b);    // set color
}

/**
 * @brief Fill ALL LEDs with RGB color
 * @param[in] r Red component   [0..255]
 * @param[in] g Green component [0..255]
 * @param[in] b Blue component  [0..255]
 */
void
ws2815_fillRGB(ws2815_desc_t *desc, uint8_t r, uint8_t g, uint8_t b)
{
	for (volatile size_t i = 0; i < NUM_PIXELS; i++) {
		ws2815_setRGB(desc, i, r, g, b);
	}
}

/**
 * @brief Fill ALL LEDs with HSV color
 * @param[in] hue HUE (color) [0..255]
 * @param[in] sat Saturation  [0..255]
 * @param[in] val Value (brightness) [0..255]
 */
void
ws2815_fillHSV(ws2815_desc_t *desc, uint8_t hue, uint8_t sat, uint8_t val)
{
	uint8_t _r, _g, _b;		       // init buffer color
	HSV2RGB(hue, sat, val, &_r, &_g, &_b); // get color once (!)
	ws2815_fillRGB(desc, _r, _g, _b);      // set color
}

/**
 * @brief Get current DMA status
 * @param none
 * @return #WS2815_STATE enum
 */
WS2815_STATE
ws2815_ready(ws2815_desc_t *desc)
{
	return desc->loc_state;
}

/**
 * @brief Update strip
 * @param none
 * @return #WS2815_STATE enum
 */
WS2815_STATE
ws2815_show(ws2815_desc_t *desc)
{
	desc->loc_state = WS2815_BUSY;
	if (desc->BUF_COUNTER != 0 ||
	    desc->dma_handle.State != HAL_DMA_STATE_READY) {
		return WS2815_BUSY;
	} else {
		for (volatile uint8_t i = 0; i < 8; i++) {
			// set first transfer from first values
			desc->PWM_BUF[i] =
			    (((desc->RGB_BUF[0] << i) & 0x80) > 0) ? PWM_HI
								   : PWM_LO;
			desc->PWM_BUF[i + 8] =
			    (((desc->RGB_BUF[1] << i) & 0x80) > 0) ? PWM_HI
								   : PWM_LO;
			desc->PWM_BUF[i + 16] =
			    (((desc->RGB_BUF[2] << i) & 0x80) > 0) ? PWM_HI
								   : PWM_LO;
			desc->PWM_BUF[i + 24] =
			    (((desc->RGB_BUF[3] << i) & 0x80) > 0) ? PWM_HI
								   : PWM_LO;
			desc->PWM_BUF[i + 32] =
			    (((desc->RGB_BUF[4] << i) & 0x80) > 0) ? PWM_HI
								   : PWM_LO;
			desc->PWM_BUF[i + 40] =
			    (((desc->RGB_BUF[5] << i) & 0x80) > 0) ? PWM_HI
								   : PWM_LO;
		}
		HAL_StatusTypeDef DMA_Send_Stat = HAL_ERROR;
		while (DMA_Send_Stat != HAL_OK) {
			if (TIM_CHANNEL_STATE_GET(desc->htim,
						  desc->tim_channel) ==
			    HAL_TIM_CHANNEL_STATE_BUSY) {
				DMA_Send_Stat = HAL_BUSY;
				continue;
			} else if (TIM_CHANNEL_STATE_GET(desc->htim,
							 desc->tim_channel) ==
				   HAL_TIM_CHANNEL_STATE_READY) {
				TIM_CHANNEL_STATE_SET(
				    desc->htim, desc->tim_channel,
				    HAL_TIM_CHANNEL_STATE_BUSY);
			} else {
				DMA_Send_Stat = HAL_ERROR;
				continue;
			}

			desc->dma_handle.XferCpltCallback =
			    ws2815_TIM_DMADelayPulseCplt;
			desc->dma_handle.XferHalfCpltCallback =
			    ws2815_TIM_DMADelayPulseHalfCplt;
			desc->dma_handle.XferErrorCallback = TIM_DMAError;
			if (HAL_DMA_Start_IT(
				&desc->dma_handle, (uint32_t)desc->PWM_BUF,
				(uint32_t)desc->tim_ccr,
				(uint16_t)PWM_BUF_SIZE) != HAL_OK) {
				DMA_Send_Stat = HAL_ERROR;
				continue;
			}
			__HAL_TIM_ENABLE_DMA(desc->htim, desc->tim_dma_cc);
			if (IS_TIM_BREAK_INSTANCE(desc->htim->Instance) !=
			    RESET)
				__HAL_TIM_MOE_ENABLE(desc->htim);
			if (IS_TIM_SLAVE_INSTANCE(desc->htim->Instance)) {
				uint32_t tmpsmcr =
				    desc->htim->Instance->SMCR & TIM_SMCR_SMS;
				if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
					__HAL_TIM_ENABLE(desc->htim);
			} else
				__HAL_TIM_ENABLE(desc->htim);
			DMA_Send_Stat = HAL_OK;
		}
		desc->BUF_COUNTER = 2;
		return WS2815_OK;
	}
}

/** @} */ // Driver
