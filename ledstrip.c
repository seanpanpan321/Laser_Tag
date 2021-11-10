/*
 * ledstrip.h
 *
 *  Created on: Nov 6, 2021
 *      Author: seanpanpan
 */

#include "ledstrip.h" 
#include "fireshot.h"
#include "lcd.h"

void targetShot(uint8_t isShot){

}

void bulletIndicator(uint8_t bulletCount, uint8_t isReloading){


}

void disableMode(uint8_t *lives, uint8_t target1, uint8_t target2, uint8_t target3){
	(*lives)--;
	isShot(target1, target2, target3);

	//led red
	allRed();
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

	HAL_Delay(3000);
}

void allGreen(){
	  Set_LED(0, 0, 255, 0);
	  Set_LED(1, 0, 255, 0);
	  Set_LED(2, 0, 255, 0);
	  Set_LED(3, 0, 255, 0);
	  Set_LED(4, 0, 255, 0);
	  Set_LED(5, 0, 255, 0);
	  Set_LED(6, 0, 255, 0);
	  Set_LED(7, 0, 255, 0);
	  Set_LED(8, 0, 255, 0);
	  Set_LED(9, 0, 255, 0);
	  Set_LED(10, 0, 255, 0);
	  Set_LED(11, 0, 255, 0);
}

void allRed(){
	  Set_LED(0, 255, 0, 0);
	  Set_LED(1, 255, 0, 0);
	  Set_LED(2, 255, 0, 0);
	  Set_LED(3, 255, 0, 0);
	  Set_LED(4, 255, 0, 0);
	  Set_LED(5, 255, 0, 0);
	  Set_LED(6, 255, 0, 0);
	  Set_LED(7, 255, 0, 0);
	  Set_LED(8, 255, 0, 0);
	  Set_LED(9, 255, 0, 0);
	  Set_LED(10, 255, 0, 0);
	  Set_LED(11, 255, 0, 0);
}
