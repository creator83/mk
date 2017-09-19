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
#include "flashspi.h"
#include "dma.h"
#include "ds1307.h"
#include "i2c.h"

Tact frq;
Spi spi (Spi::SPI_N::SPI_0);
Ssd1289::sFont bNum;
Buffer value;
Ssd1289 display;
//Flash memry (spi, Gpio::Port::E, 16);
Pin sda (Gpio::Port::E, 18, Gpio::mux::Alt4);
Pin scl (Gpio::Port::E, 19, Gpio::mux::Alt4);
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
}

uint8_t dest [10];
uint8_t data [7];
uint8_t test [10] = {24,1,2,3,4,5,6,7,8,9};
int main()
{
	I2c driverI2c (I2c::nI2c::I2c0);
	driverI2c.wByte(0xD0, 0x0F, 0x08);
	driverI2c.read(data, 0xD0, 0, 7);
	driverI2c.rByte(0xD0, 0x0F);
	driverI2c.read(data, 0xD0, 0x0E, 2);

	value.setFont(Array_dec);

	Shape::driver = &display;
	display.fillScreen(colors16bit::SILVER);

	Ssd1289::sFont mTimes;
	bNum.font = bigNumbers::numbers;
	bNum.height = 55;
	bNum.width = 3;
	bNum.shift = 0;
	mTimes.font = midlleTimesNewRomanRus::rus;
	mTimes.width = 2;
	mTimes.height = 17;
	mTimes.shift = 0;
	Dma dma0 (Dma::dmaChannel::ch0);
	dma0.setDestination((uint32_t)dest);
	dma0.setSource((uint32_t)midlleTimesNewRomanRus::rus);
	dma0.setDsize(Dma::size::bit8);
	dma0.setSsize(Dma::size::bit8);
	dma0.setIncDestination(true);
	dma0.setIncSource(true);
	dma0.setLength(10);
	dma0.start();
	while (!dma0.flagDone());
	//display.symbol (50,50,  colors16bit::BLACK, colors16bit::GRAY);
	Dma dma1 (Dma::dmaChannel::ch1);
	dma1.setDestination((uint32_t)&SPI0->DL);
	dma1.setSource((uint32_t)test);
	dma1.setDsize(Dma::size::bit8);
	dma1.setSsize(Dma::size::bit8);
	dma1.setIncDestination(false);
	dma1.setIncSource(true);
	dma1.setLength(10);
	dma1.enableDmaMux(Dma::dmaMux::spi0Tx);


	spi.setMode(Spi::Mode::software);
	spi.setDivision(Spi::Division::div4);
	spi.setFrameSize(Spi::Size::bit8);
	Pin cs (Gpio::Port::E, 16);
	Pin sck (Gpio::Port::E, 17, Gpio::mux::Alt2);
	Pin mosi (Gpio::Port::E, 18, Gpio::mux::Alt2);
	Pin miso (Gpio::Port::E, 19, Gpio::mux::Alt2);
	cs.set();
	spi.setDma(dma1);
	spi.start();
	cs.clear();
	spi.enableDma(Spi::dma::transmit);
	while (!dma1.flagDone());
	while (!spi.flagSptef());
	cs.set();
	/*I2c i2c0 (I2c::nI2c::I2c0);
	Ds1307 time (i2c0);*/




	while (1)
	{

		/*time.read(ds1307reg::Seconds);
		delay_ms(100);
			delay_ms(100);
		dma1.clearFlags();
		dma1.setLength(10);
		while (!spi.flagSptef());
		spi.putDataDl(0x00);
		spi.enableDma(Spi::dma::transmit);
		//memry.readID();
		while (!dma1.flagDone());
		spi.disableDma(Spi::dma::transmit);
		dma1.setLength(10);
		dma1.clearFlags();
		delay_ms(100);*/


	}
}

