#include "MKL26Z4.h"                // Device header
#include "tact.h"
#include "delay.h"
#include "ssd1289.h"
#include "shape.h"
#include "pixel.h"
#include "verline.h"
#include "horline.h"
#include "rectangle.h"
#include "colors16bit.h"
#include "image.h"
#include "list.h"
#include "mpic.h"

Tact frq;


Horline hLine1 (0, 50, colors16bit::WHITE, 190, 5);
Horline hLine2 (0, 70, colors16bit::WHITE, 190, 5);
Verline vline1 (100, 0, colors16bit::WHITE, 200, 5);

//main screen
MonoPicture tools (30, 228, colors16bit::GRAY, colors16bit::BLACK, bigImages::support, 12, 96);
MonoPicture outside (190, 108, colors16bit::GRAY, colors16bit::BLACK, bigImages::outside, 12, 96);
MonoPicture watch (190, 228, colors16bit::GRAY, colors16bit::BLACK, bigImages::alarm, 12, 96);
MonoPicture livingSmall (25, 113, colors16bit::GRAY, colors16bit::BLACK, smallImages::living, 5, 40);
MonoPicture bathSmall (90, 113, colors16bit::GRAY, colors16bit::BLACK, smallImages::bath, 5, 40);
MonoPicture diningSmall (25, 53, colors16bit::GRAY, colors16bit::BLACK, smallImages::dining, 5, 40);

MonoPicture home (227, 96, colors16bit::GRAY, colors16bit::BLACK, midlleImages::home, 9, 72);
MonoPicture back (227, 216, colors16bit::GRAY, colors16bit::BLACK, midlleImages::back, 9, 72);


//equipment screen
MonoPicture pump (14, 216, colors16bit::GRAY, colors16bit::BLACK, midlleImages::pump, 9, 72);
MonoPicture boiler (120, 216, colors16bit::GRAY, colors16bit::BLACK, midlleImages::level, 9, 72);
MonoPicture fan (14, 96, colors16bit::GRAY, colors16bit::BLACK, midlleImages::fan, 9, 72);
MonoPicture floor (120, 96, colors16bit::GRAY, colors16bit::BLACK, midlleImages::floor, 9, 72);

//rooms screen
MonoPicture livingBig (14, 216, colors16bit::GRAY, colors16bit::BLACK, midlleImages::living, 9, 72);
MonoPicture bathBig (120, 216, colors16bit::GRAY, colors16bit::BLACK, midlleImages::bath, 9, 72);
MonoPicture diningBig (14, 96, colors16bit::GRAY, colors16bit::BLACK, midlleImages::dining, 9, 72);




List mScreen;
List equipment;
List rooms;

extern "C" {
	void SysTick_Handler();
	void PIT_CH1_IRQHandler();
	void IRQ_IRQHandler();
}



enum class color: uint8_t {red, green, blue};

union rgb24
{
	uint32_t code;
	uint8_t color[3];
};


void mainScreenFon (Ssd1289 &);
void otherScreenFon (Ssd1289 &);

const uint16_t colors [] = {colors16bit::BLACK, colors16bit::RED, colors16bit::BLUE,  colors16bit::GREEN, colors16bit::CYAN, colors16bit::MAGENTA,  colors16bit::YELLOW, colors16bit::WHITE,
 colors16bit::GRAY, colors16bit::SKY, colors16bit::ORANGE,  colors16bit::PINK, colors16bit::BROWN,colors16bit::VIOLET, colors16bit::SILVER,
 colors16bit::GOLD,  colors16bit::BEGH, colors16bit::NAVY, colors16bit::DARK_GREEN, colors16bit::DARK_CYAN, colors16bit::MAROON,  colors16bit::PURPLE,
colors16bit::LIGHT_GREY,  colors16bit::DARK_GREY};

List * Screens [] = {&mScreen, &equipment, &rooms};

int main()
{
	//main screen
	mScreen.addLast(&tools);
	mScreen.addLast(&watch);
	mScreen.addLast(&outside);
	mScreen.addLast(&livingSmall);
	mScreen.addLast(&diningSmall);
	mScreen.addLast(&bathSmall);

	//equipment screen
	equipment.addLast(&pump);
	equipment.addLast(&boiler);
	equipment.addLast(&fan);
	equipment.addLast(&floor);

	//rooms screen
	rooms.addLast(&livingBig);
	rooms.addLast(&diningBig);
	rooms.addLast(&bathBig);

	Ssd1289 display;
	Shape::driver = &display;
	/*
	Ssd1289::sFont num;
	num.font = number;
	num.height = 55;
	num.width = 3;
	num.shift = 0;
	display.symbol (50,50,  colors16bit::BLACK, colors16bit::GRAY, 0, num);
*/
	while (1)
	{
		for (uint8_t i=0;i<3;++i)
		{
			if (!i)	mainScreenFon (display);
			else otherScreenFon(display);
			Screens [i]->iterate();
			delay_ms(1000);
		}
	}
}

void mainScreenFon (Ssd1289 & d)
{
	d.fillScreen(colors16bit::SILVER);

	d.verLine(160, 0, colors16bit::BLACK, 240, 2);
	d.horLine(0, 120, colors16bit::BLACK, 320, 2);

	//gradient
	d.rectangle(5,5, colors16bit::BLACK,150, 110, 1);
	d.rectangle(165,5, colors16bit::BLACK,150, 110, 1);
	d.rectangle(5,125, colors16bit::BLACK,150, 110, 1);
	d.rectangle(165,125, colors16bit::BLACK,150, 110, 1);
	d.horLine(6, 6, colors16bit::GRAY, 149, 109);
	d.horLine(166, 6, colors16bit::GRAY, 149, 109);
	d.horLine(6, 126, colors16bit::GRAY, 149, 109);
	d.horLine(166, 126, colors16bit::GRAY, 149, 109);
}

void otherScreenFon (Ssd1289 &d)
{
	d.fillScreen(colors16bit::SILVER);

	d.verLine(106, 0, colors16bit::BLACK, 240, 2);
	d.verLine(212, 0, colors16bit::BLACK, 240, 2);
	d.horLine(0, 120, colors16bit::BLACK, 320, 2);

	d.rectangle(5,5, colors16bit::BLACK,96, 110, 1);
	d.rectangle(111,5, colors16bit::BLACK,96, 110, 1);
	d.rectangle(218,5, colors16bit::BLACK,96, 110, 1);
	d.rectangle(5,125, colors16bit::BLACK,96, 110, 1);
	d.rectangle(111,125, colors16bit::BLACK,96, 110, 1);
	d.rectangle(218,125, colors16bit::BLACK,96, 110, 1);
	d.horLine(6, 6, colors16bit::GRAY, 95, 109);
	d.horLine(112, 6, colors16bit::GRAY, 95, 109);
	d.horLine(219, 6, colors16bit::GRAY, 95, 109);
	d.horLine(6, 126, colors16bit::GRAY, 95, 109);
	d.horLine(112, 126, colors16bit::GRAY, 95, 109);
	d.horLine(219, 126, colors16bit::GRAY, 95, 109);
	back.draw();
	home.draw();
}





