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
#include "font.h"
#include "list.h"
#include "mpic.h"
#include "shift.h"
#include "spi.h"
#include "xpt2046.h"
#include "buffer.h"
#include "nrf24l01.h"
#include "tbutton.h"

Tact frq;
Spi spi0 (Spi::SPI_N::SPI_0);
Ssd1289::sFont bNum;
Ssd1289::sFont small;
//Intrpt radioIrq (Gpio::Port::D, 1, Intrpt::mode::fallingEdge);

Buffer value;
Ssd1289 display;
Nrf24l01 radio (spi0);

void bin (uint16_t, uint16_t, uint8_t);

struct flags
{
  unsigned touch :1;
  unsigned mainScreen :1;
  unsigned subScreen :2;
}flag;

extern "C" {
	void SysTick_Handler();
	void PIT_CH1_IRQHandler();
	void IRQ_IRQHandler();
	void PORTC_PORTD_IRQHandler();
}

void PORTC_PORTD_IRQHandler()
{
	radio.clearFlag();
	uint8_t status = radio.readStatus();
	radio.writeRegister (STATUS, status);
	bin (150,180, status);
}


void bin (uint16_t, uint16_t, uint8_t);

int main()
{
	value.setFont(Array_dec);

	Shape::driver = &display;
	display.fillScreen(colors16bit::SILVER);
	small.font = smallTimesNewRomanRus::simbols;
	small.height = 8;
	small.width = 1;
	small.shift = 32;
	bNum.font = midleNumbers::number;
	bNum.height = 36;
	bNum.width = 2;
	bNum.shift = 0;

	/*Pin sck (Gpio::Port::E, 17, Gpio::mux::Alt2);
	Pin mosi (Gpio::Port::E, 18, Gpio::mux::Alt2);
	Pin miso (Gpio::Port::E, 19, Gpio::mux::Alt2);*/

	Pin radioIrq (Gpio::Port::D, 1, Gpio::PP::PullUp);

	display.string(10,200, colors16bit::SILVER,colors16bit::BLACK,  "CONFIG", small, 0);
	display.string(10,180, colors16bit::SILVER,colors16bit::BLACK,  "STATUS", small, 0);
	display.string(10,160, colors16bit::SILVER,colors16bit::BLACK,  "INPUT:", small, 0);

	//display.drawArr(10, 200, colors16bit::BLACK, colors16bit::SILVER, midlleTimesNewRomanRus::rus, 2,17);

	bin (70,200, radio.readRegister(CONFIG));
	bin (70,180, radio.readStatus());

	while (1)
	{
		for (uint8_t i=0;i<0xFF;++i)
		{
			radio.writeRegister(W_TX_PAYLOAD, i);
			radio.txState();
			while (radioIrq.state());
			uint8_t b = radio.readStatus();
			bin (70,180, b);
			radio.writeRegister(STATUS, b);
			delay_ms(1000);
		}
	}
}

void bin (uint16_t x, uint16_t y, uint8_t data)
{
	for (int8_t i=7;i>=0;--i, x+=8)
	{
		if (data & 1<<i)
		{
			display.string(x,y, colors16bit::SILVER, colors16bit::BLACK, "1", small, 1, 0);

		}
		else
		{
			display.string(x,y, colors16bit::SILVER,colors16bit::BLACK,  "0", small, 1, 0);
		}
	}
}


