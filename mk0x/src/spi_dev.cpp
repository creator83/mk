#include "spi_dev.h"

SPI_Type * SpiDev::spiAdr [1]={SPI0};

SpiDev::SpiDev(spiNumber s){
	nSpi = spiAdr[static_cast<uint8_t>(s)];
}

SPI_Type * SpiDev::getSpiDev (){
	return nSpi;
}
