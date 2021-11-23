#include "interface.h"


void teamInit(uint8_t team){

}
	

//color = BLUE or color = RED
void drawHeart(uint16_t x, uint16_t y, uint16_t color){
	for(int i = 0; i < 128; i++){
		for(int n = 0; n < 8; n++){
			uint8_t nth_bit = heart[i] & (1<<(7-n)) ;
			if ((nth_bit>>(7-n) )== 1)
				LCD_DrawDot(x+i%4*8+n, y+i/4, color );
		}
	}
}

void lifeIndicate (uint8_t remainingLife){
	for (int i=0; i<remainingLife; i++){
		drawHeart(104,18+i*50 , RED);
	}
}

void teamIndicate (uint8_t team){
	uint16_t teamColor;
	if(team == 1)
		teamColor = RED;
	else 
		teamColor = BLUE;
	
	LCD_OpenWindow ( 0, 0, 80, 320 );
	LCD_FillColor ( 80 * 320, teamColor );		
	
	LCD_OpenWindow ( 240-80, 0, 80, 320 );
	LCD_FillColor ( 80 * 320, teamColor );
}
