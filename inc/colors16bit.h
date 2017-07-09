#include "device.h"               // Device header



#ifndef COLORS16BIT_H
#define COLORS16BIT_H

namespace colors16bit
{
	const uint16_t BLACK =	0x0000;
	const uint16_t RED 	 =  0xF800;
	const uint16_t BLUE =  0x001F;
	const uint16_t GREEN =	0x07E0;
	const uint16_t CYAN	 =  0x07FF;
	const uint16_t MAGENTA =0xF81F;
	const uint16_t YELLOW =	0xFFE0;
	const uint16_t WHITE =	0xFFFF;

	const uint16_t GRAY    =	 0x8410;
	const uint16_t SKY     =	 0x5d1C;
	const uint16_t ORANGE  =  	 0xFCA0;
	const uint16_t PINK    =	 0xF97F;
	const uint16_t BROWN   =	 0x8200;
	const uint16_t VIOLET  =	 0x9199;
	const uint16_t SILVER  =  	 0xA510;
	const uint16_t GOLD    =	 0xA508;
	const uint16_t BEGH    =	 0xF77B;
	const uint16_t NAVY    =	 0x000F;
	const uint16_t DARK_GREEN =	 0x03E0;
	const uint16_t DARK_CYAN  =	 0x03EF;
	const uint16_t MAROON     =	 0x7800;
	const uint16_t PURPLE     =  0x7BE0;//0x8010;
	const uint16_t LIGHT_GREY =  0xC618;
	const uint16_t DARK_GREY  =  0x7BEF;

	const uint8_t redShift = 11;
	const uint8_t greenShift = 5;
	const uint8_t blueShift = 0;
}


#endif
