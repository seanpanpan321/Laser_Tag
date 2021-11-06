/*
 * fireshot.h
 *
 *  Created on: Nov 6, 2021
 *      Author: seanpanpan
 */

#ifndef SRC_FIRESHOT_H_
#define SRC_FIRESHOT_H_

void fire(uint8_t triggerPressed, uint8_t *bulletCount){



		  if((*bulletCount) == 0){
			  //needs reload
			  return;
		  }

		  if(triggerPressed){
			  //turn on laser
		  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		  	  //turn on motor
		  	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		  	  HAL_Delay(500);

			  //turn off laser and motor
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		  	  (*bulletCount)--;
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

#endif /* SRC_FIRESHOT_H_ */
