/*
 * ledstrip.h
 *
 *  Created on: Nov 6, 2021
 *      Author: seanpanpan
 */

#include "ledstrip.h" 
#include "fireshot.h"
#include "lcd.h"
#include "ws2812.h"
UART_HandleTypeDef huart1;
void bulletIndicator(uint8_t bulletCount, uint8_t isReloading, uint8_t charge){
	Set_LED_Color (0,0,0);
	for (int i=bulletCount; i>0; i--)
	{
		if(i%2==0){
			if (i<=charge ){
				Set_LED(i*2-2, 0, 0, 100);
				Set_LED(i*2-1, 0, 0, 100);
			}
			else{
				Set_LED(i*2-2, 0, 100, 0);
				Set_LED(i*2-1, 0, 100, 0);
			}

		}
		else{
			if (i<=charge){
				Set_LED(i*2-2, 100, 0, 100);
				Set_LED(i*2-1, 100  , 0, 100);
			}
			else {
				Set_LED(i*2-2, 100, 100, 0);
				Set_LED(i*2-1, 100, 100, 0);
			}

		}

	}
	Set_Brightness(3);


	WS2812_Send_B();
}

void disableMode(uint8_t *lives){
	(*lives)--;
	//led yellow
	Set_LED_Color(100, 100, 0);
	Set_Brightness(20);
	WS2812_Send();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	if((*lives) == 0){
		LCD_Clear (0, 0, 240, 320, BACKGROUND);
		LCD_DrawString(80, 160, "GAME OVER");

		char gameover[] = "GAME OVER";
		HAL_UART_Transmit (&huart1, (uint8_t*)&gameover, sizeof(gameover), 100);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		return;
	}

	delay_ms(3000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}
