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

void targetShot(uint8_t isShot){

}

void bulletIndicator(uint8_t bulletCount, uint8_t isReloading){
	Set_LED_Color (0,0,0);
	for (int i=bulletCount; i>0; i--)
	{
		if(i%2==0){
			Set_LED(i*2-2, 0, 100, 0);
			Set_LED(i*2-1, 0, 100, 0);
		}
		else{
			Set_LED(i*2-2, 0, 0, 100);
			Set_LED(i*2-1, 0, 0, 100);
		}

	}
	Set_Brightness(3);


	WS2812_Send_B();
}

void disableMode(uint8_t *lives, uint8_t target1, uint8_t target2, uint8_t target3){
	(*lives)--;
	isShot(target1, target2, target3);

	//led red
	Set_LED_Color(255, 0, 0);
	Set_Brightness(20);
	WS2812_Send();

	if((*lives) == 0){
		LCD_Clear (0, 0, 240, 320, BACKGROUND);
		LCD_DrawString(80, 160, "GAME OVER");

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		return;
	}

	char buf[8];
	sprintf(buf,"%d" ,(*lives));
	LCD_DrawString(0, 250, "lives left: ");
	LCD_DrawString(100, 250, buf);

	delay_ms(3000);
}
