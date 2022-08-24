/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
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

#include "stm32f1xx_it.h"
#include "main.h"

extern DMA_HandleTypeDef hdma_tim1_ch1;
extern DMA_HandleTypeDef hdma_tim1_ch2;
extern TIM_HandleTypeDef htim3;

extern volatile uint32_t led_work;

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
__attribute__((noreturn)) void
NMI_Handler(void)
{
	while (1) {
		/* do nothing */
	}
}

/**
 * @brief This function handles Hard fault interrupt.
 */
__attribute__((noreturn)) void
HardFault_Handler(void)
{
	while (1) {
		/* do nothing */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
__attribute__((noreturn)) void
MemManage_Handler(void)
{
	while (1) {
		/* do nothing */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
__attribute__((noreturn)) void
BusFault_Handler(void)
{
	while (1) {
		/* do nothing */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
__attribute__((noreturn)) void
UsageFault_Handler(void)
{
	while (1) {
		/* do nothing */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void
SVC_Handler(void)
{
	/* do nothing */
}

/**
 * @brief This function handles Debug monitor.
 */
void
DebugMon_Handler(void)
{
	/* do nothing */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void
PendSV_Handler(void)
{
	/* do nothing */
}

/**
 * @brief This function handles System tick timer.
 */
void
SysTick_Handler(void)
{
	HAL_IncTick();
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles TIM3 global interrupt.
 */
void
TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim3);
	led_work++;
}
