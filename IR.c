/*
 * IR.c
 *
 *  Created on: Nov 23, 2021
 *      Author: seanpanpan
 */
#include "IR.h"

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

uint16_t countHighTime(){
	uint16_t counter = 0;
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)){
		counter++;
		delay_us(1);
		if (counter > 5000)
			return 6000;
	}
	return counter;
}
