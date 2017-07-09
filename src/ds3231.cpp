#include "Ds3231.h"

Ds3231::Ds3231(I2c &d, Pin &sda_, Pin & scl_)
{
	sda = &sda_;
	scl = &scl_;
	driver = &d;
}

void Ds3231::write (uint8_t reg, uint8_t val)
{
	driver->wByte(selfAddress, reg, val);
}

uint8_t Ds3231::read (uint8_t reg)
{
	return driver->rByte (selfAddress, reg);
}

void Ds3231::readCalendar ()
{
	uint8_t * ptr = bcdData;
	driver->start ();
	while (!driver->flagBusy());
	driver->setAddress (this->selfAddress, I2c::directionBit::write);
	if (driver->waitAck() == 0 ) return;
	driver->putData(0x00);
	if (driver->waitAck() == 0 ) return;
	driver->restart();
	driver->setAddress (selfAddress, I2c::directionBit::read);
	if (driver->waitAck() == 0 ) return;
	driver->setMode (I2c::mode::receiver);
	driver->generateAck();
	uint8_t dummy = driver->getData();
	driver->waitAck();
	for (uint8_t i=0;i<5;++i)
	{
		*ptr++ = driver->getData();
		driver->waitAck();
	}
	driver->generateNack ();
	*ptr++ = driver->getData();
	driver->waitAck();
	driver->stop ();
	*ptr = driver->getData();
	while (driver->flagBusy());
}

void Ds3231::writeCalendar ()
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

void Ds3231::stop ()
{
	uint8_t temp = driver->rByte (selfAddress, Ds3231reg::control);
	temp |= 0x80;
	driver->wByte (selfAddress, Ds3231reg::control, temp);
}

void Ds3231::start ()
{
	uint8_t temp = driver->rByte (selfAddress, Ds3231reg::control);
	temp &= ~ 0x80;
	driver->wByte (selfAddress, Ds3231reg::control, temp);
}

void Ds3231::setData ()
{
	convertDec ();
	writeCalendar ();
}

void Ds3231::setSeconds (uint8_t val)
{
	driver->wByte (selfAddress, Ds3231reg::Seconds, val);
}

void Ds3231::setMinutes (uint8_t val)
{
	driver->wByte (selfAddress, Ds3231reg::Minutes, val);
}

void Ds3231::setHours (uint8_t val)
{
	driver->wByte (selfAddress, Ds3231reg::Hours, val);
}

void Ds3231::setDay (uint8_t val)
{
	driver->wByte (selfAddress, Ds3231reg::Day, val);
}

void Ds3231::setDate (uint8_t val)
{
	driver->wByte (selfAddress, Ds3231reg::Date, val);
}

void Ds3231::setMonth (uint8_t val)
{
	driver->wByte (selfAddress, Ds3231reg::Month, val);
}

void Ds3231::setYear (uint8_t val)
{
	driver->wByte (selfAddress, Ds3231reg::Year, val);
}

uint8_t & Ds3231::getSeconds ()
{
	return bcdData [Ds3231reg::Seconds];
}

uint8_t & Ds3231::getMinutes ()
{
	return bcdData [Ds3231reg::Minutes];
}

uint8_t & Ds3231::getHours ()
{
	return bcdData [Ds3231reg::Hours];
}

uint8_t & Ds3231::getDay ()
{
	return bcdData [Ds3231reg::Day];
}

uint8_t & Ds3231::getDate ()
{
	return bcdData [Ds3231reg::Date];
}

uint8_t & Ds3231::getMonth ()
{
	return bcdData [Ds3231reg::Month];
}

uint8_t & Ds3231::getYear ()
{
	return bcdData [Ds3231reg::Year];
}

void Ds3231::convert (convertMode)
{

}

void Ds3231::convertBcd ()
{
	//seconds
	  uint8_t temp = (bcdData[Ds3231reg::Seconds]&valueMask::Dseconds)>>4;
	  decData[Ds3231reg::Seconds] = temp*10;
	  temp = bcdData[Ds3231reg::Seconds]&valueMask::seconds;
	  decData[Ds3231reg::Seconds] += temp;

	  //minutes
	  temp = (bcdData[Ds3231reg::Minutes]&valueMask::Dminutes)>>4;
	  decData[Ds3231reg::Minutes] = temp*10;
	  temp = bcdData[Ds3231reg::Minutes]&valueMask::minutes;
	  decData[Ds3231reg::Minutes] += temp;

	  //hours
	  temp = (bcdData[Ds3231reg::Hours]&valueMask::Dhours)>>4;
	  decData[Ds3231reg::Hours] = temp*10;
	  temp = bcdData[Ds3231reg::Hours]&valueMask::hours;
	  decData[Ds3231reg::Hours] += temp;

	  //day
	  decData[Ds3231reg::Day] = bcdData[Ds3231reg::Day];

	  //date
	  temp = (bcdData[Ds3231reg::Date]&valueMask::Ddate)>>4;
	  decData[Ds3231reg::Date] = temp*10;
	  temp = bcdData[Ds3231reg::Date]&valueMask::date;
	  decData[Ds3231reg::Date] += temp;

	  //year
	  temp = (bcdData[Ds3231reg::Year]&valueMask::Dyear)>>4;
	  decData[Ds3231reg::Year] = temp*10;
	  temp = bcdData[Ds3231reg::Year]&valueMask::year;
	  decData[Ds3231reg::Year] += temp;
}

void Ds3231::convertDec ()
{
	//seconds
	  uint8_t temp = decData[Ds3231reg::Seconds];
	  uint8_t result;
	  for (result =0;temp>=10;++result, temp-=10) ;
	  bcdData[Ds3231reg::Seconds] = result << 4;
	  bcdData[Ds3231reg::Seconds] |= temp%10;

	  //minutes
	  temp = decData[Ds3231reg::Minutes];
	  for (result =0;temp>=10;++result, temp-=10) ;
	  bcdData[Ds3231reg::Minutes] = result << 4;
	  bcdData[Ds3231reg::Minutes] |= temp%10;

	  //hours
	  temp = decData[Ds3231reg::Hours];
	  for (result =0;temp>=10;++result, temp-=10) ;
	  bcdData[Ds3231reg::Hours] =  (result << 4);
	  bcdData[Ds3231reg::Hours] |= temp%10;

	  //day
	  bcdData[Ds3231reg::Day] = decData[Ds3231reg::Day];

	  //date
	  temp = decData[Ds3231reg::Date];
	  for (result =0;temp>=10;++result, temp-=10) ;
	  bcdData[Ds3231reg::Date] = result << 4;
	  bcdData[Ds3231reg::Date] |= temp%10;

	  //year
	  temp = decData[Ds3231reg::Year];
	  for (result =0;temp>=10;++result, temp-=10) ;
	  bcdData[Ds3231reg::Year] = result << 4;
	  bcdData[Ds3231reg::Year] |= temp%10;
}
