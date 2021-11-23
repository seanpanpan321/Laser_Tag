#ifndef INTERFACE_H
#define INTERFACE_H

#include "lcd.h"
#include "stdint.h"

//choosing a team at the begining of the game 
void teamInit(uint8_t team);

//draw a heart on the lcd at (x,y), with color = BLUE or color = RED
void drawHeart(uint16_t x, uint16_t y, uint16_t color);

//show the remaining life on the lcd
void lifeIndicate (uint8_t remainingLife);

//indicating the team with 2 retangular bar on the top and button of the lcd 
void teamIndicate (uint8_t team);

void display(uint8_t team, uint8_t remainingLife);




#endif