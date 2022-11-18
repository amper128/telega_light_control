/**
 * @file leds.h
 * @author Алексей Хохлов <root@amper.me>
 * @copyright WTFPL License
 * @date 2022
 * @brief Работа с светодиодными лентами
 */

#pragma once

#include <stdint.h>
#include <stm32f1xx_hal_tim.h>

void leds_init(void);

void leds_msp_init(TIM_HandleTypeDef *htim_base);

void leds_work(void);

void change_leds_mode(uint8_t id, uint8_t mode);

void change_leds_color(uint8_t id, uint8_t r, uint8_t g, uint8_t b);

void change_leds_period(uint8_t id, uint8_t period);

void change_leds_brightness(uint8_t id, uint8_t brightness);
