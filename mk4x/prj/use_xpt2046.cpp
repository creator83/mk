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
#include "i2c.h"
#include "sht20.h"
#include "ds3231.h"

Tact frq;
Spi spi1 (Spi::SPI_N::SPI_1);
Xpt2046 touch (spi1);
Ssd1289::sFont bNum;
Buffer value;
Ssd1289 display;
Tgrid sense (touch, 3, 2);
Tbutton mainScreen (sense);

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
	void PORTA_IRQHandler();
}

void PORTA_IRQHandler()
{

}


int main()
{
	value.setFont(Array_dec);

	Shape::driver = &display;
	display.fillScreen(colors16bit::SILVER);

	Ssd1289::sFont mTimes;
	bNum.font = numbers::times36;
	bNum.height = 24;
	bNum.width = 16;
	bNum.shift = 0;
	mTimes.font = rusFont::times20;
	mTimes.width = 23;
	mTimes.height = 21;
	mTimes.shift = 192;



	spi1.setMode(Spi::Mode::software);
	Pin sck (Gpio::Port::E, 2, Gpio::mux::Alt2);
	Pin mosi (Gpio::Port::E, 1, Gpio::mux::Alt2);
	Pin miso (Gpio::Port::E, 3, Gpio::mux::Alt2);
	Pin sda (Gpio::Port::E, 18, Gpio::mux::Alt4);
	Pin scl (Gpio::Port::E, 19, Gpio::mux::Alt4);
	I2c i2c0 (I2c::nI2c::I2c0);
	Ds3231 clock (i2c0);
	Sht20 sensor (&i2c0);
	//__WFI();
	//asm ("wfe");

	while (1)
	{
		sda.togle();
		delay_ms(1000);
		//clock.readCalendar();
		//clock.read(0x01);
		/*sensor.readTemperature();
		sensor.readHummidity();
		value.parsDec16(sensor.getTemperature(), 3);
		display.string (50,50, colors16bit::GRAY,  colors16bit::BLACK, value.getElement(2),bNum, 3,0);
		value.parsDec16(sensor.getHummidity(), 3);
		display.string (50,150, colors16bit::GRAY,  colors16bit::BLACK, value.getElement(2),bNum, 3,0);
		delay_ms(300);*/

	}
}
