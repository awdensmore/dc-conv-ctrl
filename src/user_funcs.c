/*
 * user_funcs.c
 *
 *  Created on: 10 Dec 2015
 *      Author: alex
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "user_funcs.h"

/* Private function prototypes -----------------------------------------------*/
void TIM3_PWM_Adjust(uint32_t pulse_prd, uint32_t channel);

/* Private typedef ---------------------------------------------------------*/
//TIM_OC_InitTypeDef sConfigOC;
//TIM_HandleTypeDef htim3;

/* User functions ------------------------------------------------------------*/
void TIM3_PWM_Adjust(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *configOC, uint32_t duty_cycle, uint32_t channel)
{

	HAL_TIM_PWM_Stop(&htim, channel);
	HAL_TIM_PWM_ConfigChannel(&htim, &configOC, channel);
	HAL_TIM_PWM_Start(&htim, channel);

}
