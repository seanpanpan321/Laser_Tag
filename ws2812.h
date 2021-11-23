/*
 * ws2812.h
 *
 *  Created on: Nov 10, 2021
 *      Author: seanpanpan
 */

#ifndef SRC_WS2812_H_
#define SRC_WS2812_H_

#include "math.h"
#include "stm32f1xx_hal.h"

void Set_LED (int LEDnum, int Red, int Green, int Blue);

void Set_Brightness (int brightness);

void WS2812_Send (void);

void WS2812_Send_B (void);

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

void Set_LED_Color(int Red, int Green, int Blue);


#endif /* SRC_WS2812_H_ */
