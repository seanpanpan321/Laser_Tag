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
	char buf[10];
	while(HAL_GPIO_ReadPin(IRSensor_GPIO_Port, IRSensor_Pin)){
		counter++;
		delay_us(1);
		if (counter > 5000)
			return 5000;
			//LCD_DrawString(0,300,"error");
	}
	return counter;
}

/*
void decode(uint16_t *data)
{
	char buf[50];
	*data = 0;
	uint16_t counter = 0;
	uint32_t countExit = 0;
	LCD_DrawString(0, 0, "inside decode");
	//check for the 4.5ms low time 4500us
	do{
			counter = 0;
			counter = countHighTime();
			countExit++;
			if (countExit > 10000){
				return;
				LCD_DrawString(0, 0, "returned");
			}		
	}while(counter < 100);
	
	sprintf(buf,"%d",counter);
	LCD_DrawString(200, 0, buf);
	
	if(counter < 2000 || counter > 6000)
		return;
	
	LCD_DrawString(0, 0+HEIGHT_EN_CHAR, "4.5ms detected");
	countExit =0;
	
	//state decoding
	for (int i=0; i<16; i++){
		//ignoring the small pulse within 12us
		do{
			counter = 0;
			counter = countHighTime();
			countExit++;
			if (countExit > 2000)
				return;
		}while(counter <50);
	
		// HighTime [300,900]   ->   0
		if (counter > 300 && counter < 900){
			data += 0<<(16-i);
			LCD_DrawString(0, 2*HEIGHT_EN_CHAR+HEIGHT_EN_CHAR *i, "0");
		}
			
		// HighTime [900,1500]  ->   1
		if (counter > 900 && counter < 1500){
			data += 1<<(16-i);
			LCD_DrawString(0, 2*HEIGHT_EN_CHAR+HEIGHT_EN_CHAR *i, "1");
		}
	}
}
*/
