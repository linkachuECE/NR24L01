/*
 * File: SERVO.c
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "servo.h"

extern void Error_Handler(void);

void SERVO_Init(SERVO_HandleTypeDef* pDev)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_HandleTypeDef htim;
    uint32_t PSC_Value = 0;
    uint32_t ARR_Value = 0;

	/*--------[ Calculate The PSC & ARR Values To Maximize PWM Resolution ]-------*/

	/* Those Equations Sets The F_pwm = 50Hz & Maximizes The Resolution*/
	PSC_Value = (uint32_t) (pDev->TIM_CLK / 3276800.0);
	ARR_Value = (uint32_t) ((pDev->TIM_CLK / (50.0 * (PSC_Value + 1.0))) - 1.0);

	/*--------[ Configure The Servo PWM Timer Channel ]-------*/

	/*--[Check The Timer & Enable Its Clock]--*/
	if(pDev->TIM_Instance == TIM1)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();
	}
	else if(pDev->TIM_Instance == TIM2)
	{
		__HAL_RCC_TIM2_CLK_ENABLE();
	}
	else if(pDev->TIM_Instance == TIM3)
	{
		__HAL_RCC_TIM3_CLK_ENABLE();
	}
	else if(pDev->TIM_Instance == TIM4)
	{
		__HAL_RCC_TIM4_CLK_ENABLE();
	}

	htim.Instance = pDev->TIM_Instance;
	htim.Init.Prescaler = PSC_Value;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = ARR_Value;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if(HAL_TIM_Base_Init(&htim) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if(HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig) != HAL_OK)
	{
	    Error_Handler();
	}
	if(HAL_TIM_PWM_Init(&htim) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if(HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if(HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, pDev->PWM_TIM_CH) != HAL_OK)
	{
		Error_Handler();
	}

	/*--------[ Configure The Servo PWM GPIO Pin ]-------*/

	if(pDev->SERVO_GPIO == GPIOA)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}
	else if(pDev->SERVO_GPIO == GPIOB)
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	else if(pDev->SERVO_GPIO == GPIOC)
	{
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}
	else if(pDev->SERVO_GPIO == GPIOD)
	{
		__HAL_RCC_GPIOD_CLK_ENABLE();
	}
	else if(pDev->SERVO_GPIO == GPIOE)
	{
		__HAL_RCC_GPIOE_CLK_ENABLE();
	}
	GPIO_InitStruct.Pin = pDev->SERVO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(pDev->SERVO_GPIO, &GPIO_InitStruct);

	/*--------[ Calculate & Save The Servo Pulse Information ]-------*/

	pDev->iMinPeriod = (uint16_t) (ARR_Value * (pDev->fMinPulse / 20.0));
	pDev->iMaxPeriod = (uint16_t) (ARR_Value * (pDev->fMaxPulse / 20.0));

	/*--------[ Start The PWM Channel ]-------*/

	HAL_TIM_PWM_Start(&htim, pDev->PWM_TIM_CH);
}

/* Moves A Specific Motor To A Specific Degree That Can Be Float Number */
void SERVO_MoveTo(SERVO_HandleTypeDef* pDev, float fAngle)
{
	uint16_t iPulse = 0;

	iPulse = ((fAngle * (pDev->iMaxPeriod - pDev->iMinPeriod)) / 180.0) + pDev->iMinPeriod;

	*(pDev->TIM_CCRx) = iPulse;
}

/* Moves A Specific Motor With A Raw Pulse Width Value */
void SERVO_RawMove(SERVO_HandleTypeDef* pDev, uint16_t iPulse)
{
	if(iPulse <= pDev->iMaxPeriod && iPulse >= pDev->iMinPeriod)
	{
		*(pDev->TIM_CCRx) = iPulse;
	}
}

/* Gets The Maximum Pulse Width Value For A Specific Motor */
uint16_t SERVO_GetMaxPulse(SERVO_HandleTypeDef* pDev)
{
	return pDev->iMaxPeriod;
}


/* Gets The Minimum Pulse Width Value For A Specific Motor */
uint16_t SERVO_GetMinPulse(SERVO_HandleTypeDef* pDev)
{
	return pDev->iMinPeriod;
}


/* Move A Motor From 0 deg to 180 And Back to 0 again */
void SERVO_Sweep(SERVO_HandleTypeDef* pDev)
{
	uint8_t iAngle = 0;

	SERVO_MoveTo(pDev, 0);

	HAL_Delay(250);
	while(iAngle < 180)
	{
		SERVO_MoveTo(pDev, iAngle++);
		HAL_Delay(50);
	}
	HAL_Delay(250);
	while(iAngle > 0)
	{
		SERVO_MoveTo(pDev, iAngle++);
		HAL_Delay(50);
	}
}
