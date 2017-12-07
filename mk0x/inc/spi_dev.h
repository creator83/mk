#include "device.h"                 // Device header


#ifndef SPI_DEV_H
#define SPI_DEV_H


class SpiDev
{
//variables
public:
	enum class spiNumber {SPI_0};

private:
	static SPI_Type * spiAdr [1];
	SPI_Type * nSpi;

//functions
public:
	SpiDev(spiNumber s=spiNumber::SPI_0);
	SPI_Type * getSpiDev();

private:
};



#endif

