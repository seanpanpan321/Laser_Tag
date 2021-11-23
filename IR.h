#ifndef IR_H
#define IR_H

#include "main.h"
#include <stdint.h>
#include "stm32f1xx_hal_gpio.h"
#include "lcd.h"
void delay_us(uint32_t i);
uint16_t countHighTime();


#endif