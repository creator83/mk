#include "MKL17Z4.h"                 // Device header
#include "spi.h"
#include "delay.h"

//#define SPI8

#ifndef MAX6675_H
#define MAX6675_H

class max6675
{
//variables
public:
	char buffer_value[2];
	static char number [10];
private:
  spi spi_ics;
	

//functions
public:
	max6675 ();
	uint16_t readCelsius();
	void buffer (uint16_t t);
};
#endif
