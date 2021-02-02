#include "device.h"
#include "state.h"
#include "pin.h"
#include "dma.h"
#include "spi.h"
#include "ili9341.h"
#include "colors16bit.h"

State & frq = State::getInstance();
//set pin
Pin cs (port::E, 16, Gpio::mux::Alt2);
Pin sck (port::E, 17, Gpio::mux::Alt2);
Pin mosi (port::E, 18, Gpio::mux::Alt2);
Pin miso (port::E, 19, Gpio::mux::Alt2);
Spi spi0 (numberSpi::SPI_0, Spi::ctarNumber::ctar1, Spi::csNumber::cs0);
Dma m2spi (dmaChannel::ch1);
Dma m2m (dmaChannel::ch0);
Ili9341 display(spi0, m2spi, m2m, port::E, 0, port::E, 1);
uint16_t colors [10] = {colors16bit::GRAY, colors16bit::NAVY, colors16bit::DARK_GREEN, 
                        colors16bit::BEGH, colors16bit::PURPLE, colors16bit::BROWN, 
                        colors16bit::CYAN, colors16bit::MAGENTA, colors16bit::YELLOW,
                        colors16bit::BLACK};


extern "C"{
	void DMA1_IRQHandler();
    void SPI0_IRQHandler();
}

void DMA1_IRQHandler (){
    
}
void SPI0_IRQHandler(){
    --display.counter;
    if (display.counter){
        SPI0->SR |= SPI_SR_EOQF_MASK| SPI_SR_TCF_MASK;
        DMA0->SERQ = DMA_SERQ_SERQ(1);
    }
    else{
        SPI0->SR |= SPI_SR_EOQF_MASK| SPI_SR_TCF_MASK;
    }
    
}


int main()
{
    SPI0->RSER |= SPI_RSER_EOQF_RE_MASK;
    NVIC_EnableIRQ (SPI0_IRQn);
    DMA0->TCD[1].CSR |= DMA_CSR_INTMAJOR_MASK;
    display.setResolution (Ili9341::resolution::res320x240);
    display.fillScreen(colors16bit::BLACK);
    //display.fillScreenDma(colors16bit::BLUE);

	while (1){
        
        for (uint8_t i=0;i<5;++i){
            display.fillScreenDma(colors[i]);
        }
        for (uint8_t i=5;i<10;++i){
            display.fillScreen(colors[i]);
        }
	}
}

