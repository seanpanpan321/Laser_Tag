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
 void delay_us(uint32_t i)
  {
      uint32_t temp;
     SysTick->LOAD=9*i;         //Set the reinstallation value, at 72MHZ
     SysTick->CTRL=0X01;         //Enable, reduce to zero is no action, use external clock source
     SysTick->VAL=0;                //Clear counter
     do
     {
         temp=SysTick->CTRL;           //Read the current countdown value
     }
     while((temp&0x01)&&(!(temp&(1<<16))));     //Waiting time arrives
     SysTick->CTRL=0;    //Turn off the counter
     SysTick->VAL=0;        //Clear counter
 }

//void delay_us(uint16_t us)
//{
//	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
//	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
//}
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
	burst(500);
	delay_us(500);
	burst(500);
}

void sendOne(){
	burst(500);
	delay_us(1500);
	burst(500);
}

void fire(uint8_t triggerPressed, uint8_t *bulletCount, uint8_t team){
		  if((*bulletCount) == 0){
			  //needs reload
			  return;
		  }

		  if(triggerPressed){
			  //bullet to shoot depends on which team the player is on
			  if(team == 0){
				  sendZero();
				  sendZero();
				  sendZero();
			  }
			  else if(team == 1){
				  sendOne();
				  sendOne();
				  sendOne();
			  }
			  //turn on laser
			  //TODO: change to npn transistor
			  LCD_DrawString(0, 180, "before laser");
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		  	  //turn on motor
		  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

//		  	  HAL_Delay(500);
		  	  for(int i = 0; i < 1000; i++){
		  		  delay_us(500);
		  	  }

			  //turn off laser and motor
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		  	  (*bulletCount)--;
		  	LCD_DrawString(0, 220, "at the end");
		  }
}

void isShot(uint8_t target1, uint8_t target2, uint8_t target3){
		  //is shot
		  if(target1 || target2 || target3){
			  //target x is shot
			  if(target1){
				  //led strip is red and turns back to green
			  }
			  else if(target2){

			  }
			  else if(target3){

			  }

			  //gun is disabled

			  //turn on buzzer
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		  }
		  //is not shot
		  else{
			  //led strip is green

			  //turn off buzzer
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		  }
}

uint8_t reload(uint8_t curBulletCount){
	if(curBulletCount == 6)
		return 6;

	for(int i = 0; i < 1000; i++){
		  delay_us(750);
	  }
	curBulletCount++;
	return curBulletCount;
}
