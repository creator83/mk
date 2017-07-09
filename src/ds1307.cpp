#include "ds1307.h"

Ds1307::Ds1307 (I2c &d)
:sda(ds1307def::sdaPort, ds1307def::sdaPin, ds1307def::sdaMux),
 sdl (ds1307def::sdlPort, ds1307def::sdlPin, ds1307def::sdlMux)
{
	driver = &d;
}

void Ds1307::write (uint8_t reg, uint8_t val)
{
	driver->wByte(selfAddress, reg, val);
}

uint8_t Ds1307::read (uint8_t reg)
{
	return driver->rByte (selfAddress, reg);
}

void Ds1307::readCalendar ()
{
	uint8_t * ptr = bcdData;
	driver->start ();
	while (!driver->flagBusy());
	driver->setAddress (selfAddress, I2c::directionBit::write);
	driver->waitAck();
	driver->putData(0x00);
	driver->waitAck();
	driver->restart();
	driver->setAddress (selfAddress, I2c::directionBit::read);
	driver->waitAck();
	driver->setMode (I2c::mode::receiver);
	driver->generateAck();
	for (uint8_t i=0;i<6;++i)
	{
		*ptr++ = driver->getData();
		driver->waitAck();
	}
	driver->generateNack ();
	*ptr = driver->getData();
	driver->waitAck();
	driver->stop ();
}

void Ds1307::writeCalendar ()
{
	uint8_t * ptr = bcdData;
	driver->start ();
	while (!driver->flagBusy());
	driver->setAddress (selfAddress, I2c::directionBit::write);
	driver->waitAck();
	driver->putData(0x00);
	driver->waitAck();
	for (uint8_t i=0;i<7;++i)
	{
		driver->putData(*ptr++);
		driver->waitAck();
	}
	driver->stop ();
	while (driver->flagBusy());
}

void Ds1307::stop ()
{
	driver->wByte (selfAddress, ds1307reg::Seconds, stateOff);
}

void Ds1307::start ()
{
	driver->wByte (selfAddress, ds1307reg::Seconds, stateOn);
}

void Ds1307::setSeconds (uint8_t val)
{
	driver->wByte (selfAddress, ds1307reg::Seconds, val);
}

void Ds1307::setMinutes (uint8_t val)
{
	driver->wByte (selfAddress, ds1307reg::Minutes, val);
}

void Ds1307::setHours (uint8_t val)
{
	driver->wByte (selfAddress, ds1307reg::Hours, val);
}

void Ds1307::setDay (uint8_t val)
{
	driver->wByte (selfAddress, ds1307reg::Day, val);
}

void Ds1307::setDate (uint8_t val)
{
	driver->wByte (selfAddress, ds1307reg::Date, val);
}

void Ds1307::setMonth (uint8_t val)
{
	driver->wByte (selfAddress, ds1307reg::Month, val);
}

void Ds1307::setYear (uint8_t val)
{
	driver->wByte (selfAddress, ds1307reg::Year, val);
}

uint8_t & Ds1307::getSeconds ()
{
	return bcdData [ds1307reg::Seconds];
}

uint8_t & Ds1307::getMinutes ()
{
	return bcdData [ds1307reg::Minutes];
}

uint8_t & Ds1307::getHours ()
{
	return bcdData [ds1307reg::Hours];
}

uint8_t & Ds1307::getDay ()
{
	return bcdData [ds1307reg::Day];
}

uint8_t & Ds1307::getDate ()
{
	return bcdData [ds1307reg::Date];
}

uint8_t & Ds1307::getMonth ()
{
	return bcdData [ds1307reg::Month];
}

uint8_t & Ds1307::getYear ()
{
	return bcdData [ds1307reg::Year];
}

void Ds1307::convert (convertMode)
{

}

void Ds1307::convertBcd ()
{
	//seconds
	  uint8_t temp = (bcdData[ds1307reg::Seconds]&valueMask::Dseconds)>>4;
	  decData[ds1307reg::Seconds] = temp*10;
	  temp = bcdData[ds1307reg::Seconds]&valueMask::seconds;
	  decData[ds1307reg::Seconds] += temp;

	  //minutes
	  temp = (bcdData[ds1307reg::Minutes]&valueMask::Dminutes)>>4;
	  decData[ds1307reg::Minutes] = temp*10;
	  temp = bcdData[ds1307reg::Minutes]&valueMask::minutes;
	  decData[ds1307reg::Minutes] += temp;

	  //hours
	  temp = (bcdData[ds1307reg::Hours]&valueMask::Dhours)>>4;
	  decData[ds1307reg::Hours] = temp*10;
	  temp = bcdData[ds1307reg::Hours]&valueMask::hours;
	  decData[ds1307reg::Hours] += temp;

	  //day
	  decData[ds1307reg::Day] = bcdData[ds1307reg::Day];

	  //date
	  temp = (bcdData[ds1307reg::Date]&valueMask::Ddate)>>4;
	  decData[ds1307reg::Date] = temp*10;
	  temp = bcdData[ds1307reg::Date]&valueMask::date;
	  decData[ds1307reg::Date] += temp;

	  //year
	  temp = (bcdData[ds1307reg::Year]&valueMask::Dyear)>>4;
	  decData[ds1307reg::Year] = temp*10;
	  temp = bcdData[ds1307reg::Year]&valueMask::year;
	  decData[ds1307reg::Year] += temp;
}

void Ds1307::convertDec ()
{
	//seconds
	  uint8_t temp = decData[ds1307reg::Seconds];
	  uint8_t result;
	  for (result =0;temp>=10;++result, temp-=10) ;
	  bcdData[ds1307reg::Seconds] = result << 4;
	  bcdData[ds1307reg::Seconds] |= temp%10;

	  //minutes
	  temp = decData[ds1307reg::Minutes];
	  for (result =0;temp>=10;++result, temp-=10) ;
	  bcdData[ds1307reg::Minutes] = result << 4;
	  bcdData[ds1307reg::Minutes] |= temp%10;

	  //hours
	  temp = decData[ds1307reg::Hours];
	  for (result =0;temp>=10;++result, temp-=10) ;
	  bcdData[ds1307reg::Hours] =  (result << 4);
	  bcdData[ds1307reg::Hours] |= temp%10;

	  //day
	  bcdData[ds1307reg::Day] = decData[ds1307reg::Day];

	  //date
	  temp = decData[ds1307reg::Date];
	  for (result =0;temp>=10;++result, temp-=10) ;
	  bcdData[ds1307reg::Date] = result << 4;
	  bcdData[ds1307reg::Date] |= temp%10;

	  //year
	  temp = decData[ds1307reg::Year];
	  for (result =0;temp>=10;++result, temp-=10) ;
	  bcdData[ds1307reg::Year] = result << 4;
	  bcdData[ds1307reg::Year] |= temp%10;
}
