#include "IR.h"

//testing the IR for High and Low


//Transmitting Code

/*This function turns off IR_LED and delays for 12us*/
void send_low(void) {
	HAL_GPIO_WritePin(GPIOB, IR_Pin, GPIO_PIN_RESET); //turn off IR_LED
	DWT_Delay(IR_DELAY);
}
/*This function turns on IR_LED and delays for 12us*/
void send_high(void) {
	HAL_GPIO_WritePin(GPIOB, IR_Pin, GPIO_PIN_SET); //turn on IR_LED
	DWT_Delay(IR_DELAY);
}
/*This function sends IR bursts for time specified, in microseconds*/
void burst(int length) {
	while (length > 0) {
		send_high();
		send_low();
		length -= IR_DELAY * 2;
	}
	HAL_GPIO_WritePin(GPIOB, IR_Pin, GPIO_PIN_RESET); //turn off IR_LED
}

void send_leading_code(void){
	burst(9000000);
	DWT_Delay(4500000);
}

/*This function sends the code specified.
1000us bursts are translated to logical 0s.
1500us bursts are translated to logical 1s.
This code and function is unique to each gun.*/
void send_code(void) { //sends 0b00101010 -> 42
	send_leading_code();
	burst(1500); DWT_Delay(500);
	burst(1500); DWT_Delay(500);
	burst(1000); DWT_Delay(500);
	burst(1000); DWT_Delay(500);
	burst(1000); DWT_Delay(500);
	burst(1500); DWT_Delay(500);
	burst(1500); DWT_Delay(500);
	burst(1500); DWT_Delay(500);
}
/*This function sends your code multiple times via the IR_LED.*/
void fire_blaster(uint8_t times) {
	for (uint8_t i = 0; i < times; i++) {
		send_code();
	}
}