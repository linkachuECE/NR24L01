/*
 * File: SERVO.h
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#ifndef SERVO_H_
#define SERVO_H_

#define HAL_TIM_MODULE_ENABLED

#include "stm32f1xx_hal.h"


typedef struct
{
	GPIO_TypeDef * 	SERVO_GPIO;
	uint16_t       	SERVO_PIN;
	TIM_TypeDef*   	TIM_Instance;
	uint32_t*      	TIM_CCRx;
	uint32_t       	PWM_TIM_CH;
	uint32_t       	TIM_CLK;
	float          	fMinPulse;
	float          	fMaxPulse;
	uint16_t		iMinPeriod; // DO NOT SET, CALCULATED FROM fMinPulse
	uint16_t		iMaxPeriod; // DO NOT SET, CALCULATED FROM fMaxPulse
} SERVO_HandleTypeDef;

/*-----[ Prototypes For All Functions ]-----*/

void SERVO_Init(SERVO_HandleTypeDef* pDev);
void SERVO_MoveTo(SERVO_HandleTypeDef* pDev, float fAngle);
void SERVO_RawMove(SERVO_HandleTypeDef* pDev, uint16_t iPulse);
uint16_t SERVO_GetMaxPulse(SERVO_HandleTypeDef* pDev);
uint16_t SERVO_GetMinPulse(SERVO_HandleTypeDef* pDev);
void SERVO_Sweep(SERVO_HandleTypeDef* pDev);


#endif /* SERVO_H_ */
