#include "max6675.h"

char max6675::number [10] = {'0','1','2','3','4','5','6','7','8','9'};

max6675::max6675 ()
:spi_ics(spi::D, spi::div16, spi::neg, spi::second, spi::bit16)
{
}

uint16_t max6675::readCelsius()
{
	union
	{
		uint16_t full_val;
		uint8_t half_val[2];
	}val;
	while (!spi_ics.flag_sptef());
	spi_ics.put_data_dl (0x00);
	spi_ics.put_data_dh (0x00);
	while (!spi_ics.flag_sprf());
	val.half_val[0] = spi_ics.get_data_dl();
	val.half_val[1] = spi_ics.get_data_dh();

	if (val.full_val & 0x4)
	  {
    // uh oh, no thermocouple attached!
    return 0; 
    //return -100;
  }
	val.full_val *=10;
	val.full_val>>=5;
	return val.full_val;
}



void max6675::buffer (uint16_t t)
{
	char dec, ones;
	uint16_t temp = t;
	
	for (dec=0;temp>=10;++dec)temp -=10;

	for (ones=0;temp>=1;++ones)temp--;
	
	buffer_value [0]= number [dec];
	buffer_value [1]= number [ones];
}


