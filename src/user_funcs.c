/*
 * user_funcs.c
 *
 *  Created on: 10 Dec 2015
 *      Author: alex
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "user_funcs.h"

/* Private typedef ---------------------------------------------------------*/
//TIM_OC_InitTypeDef sConfigOC;
//TIM_HandleTypeDef htim3;

/* User functions ------------------------------------------------------------*/
void TIM_PWM_Adjust(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *configOC, uint32_t duty_cycle, uint32_t channel)
{
	configOC->Pulse = duty_cycle;

	HAL_TIM_PWM_Stop(htim, channel);
	HAL_TIM_PWM_ConfigChannel(htim, configOC, channel);
	HAL_TIM_PWM_Start(htim, channel);

}

uint32_t uTicks
