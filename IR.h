/*
 * IR.h
 *
 *  Created on: Nov 23, 2021
 *      Author: seanpanpan
 */

#ifndef SRC_IR_H_
#define SRC_IR_H_

#include "main.h"
#include <stdint.h>
#include "stm32f1xx_hal_gpio.h"
#include "lcd.h"
void delay_us(uint32_t i);
uint16_t countHighTime();

#endif /* SRC_IR_H_ */
