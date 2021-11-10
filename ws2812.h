#ifndef WS2812_H
#define WS2812_H

#include "math.h"
#include "stm32f1xx_hal.h"

void Set_LED (int LEDnum, int Red, int Green, int Blue);

void Set_Brightness (int brightness);

void WS2812_Send (void);

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

void Set_LED_Color(int Red, int Green, int Blue);   

#endif
