

#pragma once

#include <stm32f1xx_hal.h>

/**
 * @addtogroup ARGB_Driver
 * @brief Addressable RGB LED Driver
 * @{
 * @addtogroup User_settings
 * @brief LED & Timer's settings
 * @{
 */

#define NUM_PIXELS 46U ///< Pixel quantity

#define USE_GAMMA_CORRECTION 1 ///< Gamma-correction should fix red&green

#define RGB_BUF_SIZE (3 * NUM_PIXELS) ///< Strip size in bytes
#define PWM_BUF_SIZE (3 * 8 * 2)      ///< Pack len * 8 bit * 2 LEDs

/// @}

/**
 * @addtogroup Global_entities
 * @brief All driver's methods
 * @{
 * @enum ARGB_STATE
 * @brief Driver's status enum
 */
typedef enum WS2815_STATE {
	WS2815_BUSY = 0,      ///< DMA Transfer in progress
	WS2815_READY = 1,     ///< DMA Ready to transfer
	WS2815_OK = 2,	      ///< Function execution success
	WS2815_PARAM_ERR = 3, ///< Error in input parameters
} WS2815_STATE;

typedef struct {
	TIM_HandleTypeDef *htim;
	volatile uint32_t tim_channel;
	HAL_TIM_ActiveChannel active_channel;
	DMA_HandleTypeDef dma_handle;
	volatile uint16_t tim_dma_id;
	volatile uint32_t tim_dma_cc;
	__IO uint32_t *tim_ccr;

	uint8_t brightness;
	volatile WS2815_STATE loc_state;

	volatile uint8_t RGB_BUF[RGB_BUF_SIZE];
	volatile uint32_t PWM_BUF[PWM_BUF_SIZE];
	volatile uint16_t BUF_COUNTER; /* PWM buffer iterator */
} ws2815_desc_t;

void ws2815_init(ws2815_desc_t *desc, TIM_HandleTypeDef *tim_handle, uint32_t tim_channel/*,
		 DMA_HandleTypeDef *dma_handle*/); // Initialization

void ws2815_clear(ws2815_desc_t *desc); // Clear strip

void ws2815_set_brightness(ws2815_desc_t *desc,
			   uint8_t br); // Set global brightness

void ws2815_setRGB(ws2815_desc_t *desc, size_t i, uint8_t r, uint8_t g,
		   uint8_t b); // Set single LED by RGB
void ws2815_setHSV(ws2815_desc_t *desc, size_t i, uint8_t hue, uint8_t sat,
		   uint8_t val); // Set single LED by HSV

void ws2815_fillRGB(ws2815_desc_t *desc, uint8_t r, uint8_t g,
		    uint8_t b); // Fill all strip with RGB color
void ws2815_fillHSV(ws2815_desc_t *desc, uint8_t hue, uint8_t sat,
		    uint8_t val); // Fill all strip with HSV color

WS2815_STATE ws2815_ready(ws2815_desc_t *desc); // Get DMA Ready state
WS2815_STATE ws2815_show(ws2815_desc_t *desc);	// Push data to the strip
