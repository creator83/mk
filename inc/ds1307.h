#include "MKL26Z4.h"
#include "pin.h"
#include "i2c.h"
#include "delay.h"

#ifndef DS1307_H
#define DS1307_H

namespace ds1307def
{
	const Gpio::Port sdaPort = Gpio::Port::E;
	const uint8_t sdaPin = 25;
	const Gpio::mux sdaMux = Gpio::mux::Alt5;

	const Gpio::Port sdlPort = Gpio::Port::E;
	const uint8_t sdlPin = 24;
	const Gpio::mux sdlMux = Gpio::mux::Alt5;
}

namespace ds1307reg
{
const uint8_t Seconds = 0x00;
const uint8_t Minutes = 0x01;
const uint8_t Hours   = 0x02;
const uint8_t Day     = 0x03;
const uint8_t Date    = 0x04;
const uint8_t Month   = 0x05;
const uint8_t Year    = 0x06;
const uint8_t Control = 0x07;
}

namespace valueMask
{
  const uint8_t seconds = 0x0F;
  const uint8_t Dseconds = 0x70;
  const uint8_t minutes = 0x0F;
  const uint8_t Dminutes = 0xF0;
  const uint8_t hours   = 0x0F;
  const uint8_t Dhours   = 0x30;
  const uint8_t day     = 0x07;
  const uint8_t date    = 0x0F;
  const uint8_t Ddate    = 0x30;
  const uint8_t Dmonth    = 0x10;
  const uint8_t month    = 0x0F;
  const uint8_t year    = 0x0F;
  const uint8_t Dyear    = 0xF0;
}

const uint8_t selfAddress = 0xD0;
const uint8_t stateOff    = 0x80;
const uint8_t stateOn     = 0x00;

class Ds1307
{
public:
	enum class convertMode {bcdToDec, decToBcd};
private:
  I2c * driver;
  Pin sda, sdl;
  uint8_t bcdData [7];
  uint8_t decData [7];
public:
  Ds1307(I2c &);
  void write (uint8_t reg, uint8_t val);
  uint8_t read (uint8_t reg);
  void readCalendar ();
  void writeCalendar ();
  void stop ();
  void start ();

  void setSeconds (uint8_t);
  void setMinutes (uint8_t);
  void setHours (uint8_t);
  void setDay (uint8_t);
  void setDate (uint8_t);
  void setMonth (uint8_t);
  void setYear (uint8_t);

  uint8_t & getSeconds ();
  uint8_t & getMinutes ();
  uint8_t & getHours ();
  uint8_t & getDay ();
  uint8_t & getDate ();
  uint8_t & getMonth ();
  uint8_t & getYear ();
  void convert (convertMode);
private:
  void convertBcd ();
  void convertDec ();

};


#endif
