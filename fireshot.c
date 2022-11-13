/*
 * fireshot.h
 *
 *  Created on: Nov 6, 2021
 *      Author: seanpanpan
 */
#include "fireshot.h"
#include "stm32f1xx_hal.h"
TIM_HandleTypeDef htim1;
/***************************
 * Function name: delay_us
 * Function function: delay function, delay us
 * Input: i
 * Output: None
 ***************************/
void sendLow(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	delay_us(12);
}

void sendHigh(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	delay_us(12);
}

void burst(int length){
	while(length > 0){
		sendHigh();
		sendLow();
		length -= 24;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}

void sendZero(){
	burst(1500);
	delay_us(1000);
	burst(1500);
}

void sendOne(){
	burst(1500);
	delay_us(3000);
	burst(1500);
}

void fire(uint8_t triggerPressed, uint8_t *bulletCount, uint8_t team, uint8_t charge){
		  if((*bulletCount) == 0)//needs reload
			  return;

		  if(triggerPressed){
			  for(int i = 0; i<charge;i++){
			  //bullet to shoot depends on which team the player is on
			  if(team == 0){
				  for (int i=0;i<30;i++)
					  sendZero();
			  }
			  else if(team == 1){
				  for (int i=0;i<30;i++)
					  sendOne();
			  }
			  //turn on laser
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		  	  //turn on motor
		  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		  	  delay_ms(400);
			  //turn off laser and motor
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		  	  (*bulletCount)--;
		  	  delay_ms(100);
		  }
		  }
}


uint8_t reload(uint8_t curBulletCount){
	if(curBulletCount == 6)
		return 6;

	delay_ms(750);
	curBulletCount++;
	return curBulletCount;
}
