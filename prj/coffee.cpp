#include "device.h"
#include "state.h"
#include "pin.h"
#include "dma.h"
#include "spi.h"
#include "ili9341.h"
#include "colors16bit.h"
#include "shape.h"
#include "xpt2046.h"
#include "tbutton.h"
#include "tree.h"
#include "list.h"
#include "tgrid.h"

extern "C"{
    void DMA1_IRQHandler();
    void SPI0_IRQHandler();
}

//===settings hardware===//
State & frq = State::getInstance();
//set pin
Intrpt irqTouch (port::A, 1, Intrpt::mode::fallingEdge);
Pin csLcd (port::E, 16, Gpio::mux::Alt2);
Pin sck (port::E, 17, Gpio::mux::Alt2);
Pin mosi (port::E, 18, Gpio::mux::Alt2);
Pin miso (port::E, 19, Gpio::mux::Alt2);
Spi spiDisplay (numberSpi::SPI_0, Spi::ctarNumber::ctar1, Spi::csNumber::cs0);
Spi spiTouch (numberSpi::SPI_0, Spi::ctarNumber::ctar0, Spi::csNumber::cs1);

Ili9341 display(spiDisplay, port::E, 0, port::E, 1);
Xpt2046 touch (spiTouch, irqTouch);
//===Graphic part===//

Tgrid nineArea (touch,3,3);
Tgrid sixArea (touch,2,3);
Tgrid setArea (touch,6,4);


ListButton tMainScreen (nineArea);
List mScreen (&tMainScreen);
Tree menu (&mScreen);




/*uint16_t colors [5] = {colors16bit::GRAY, colors16bit::NAVY, colors16bit::DARK_GREEN, 
    colors16bit::BEGH, colors16bit::PURPLE};*/

int main()
{
    display.setResolution (Ili9341::resolution::res320x240);
    
	while (1){
	}
}

