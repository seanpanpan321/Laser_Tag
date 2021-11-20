#ifndef IR_H
#define IR_H

#include "main.h"
#include "dwt_delay.h"
#include <stdint.h>
#include "stm32f1xx_hal_gpio.h"

#define IR_LED 0
#define IR_DELAY 12
#define BITS_PER_CODE 8
//TransmittingFunctions
void send_low();
void send_high();
void burst(int length);
void send_leading_code(void);
void send_code(void);
void fire_blaster(uint8_t times);


#endif