#include "device.h"
#include "pin.h"
#include "i2c.h"
#include "delay.h"

#ifndef DS3231_H
#define DS3231_H

namespace Ds3231reg
{
const uint8_t Seconds = 0x00;
const uint8_t Minutes = 0x01;
const uint8_t Hours   = 0x02;
const uint8_t Day     = 0x03;
const uint8_t Date    = 0x04;
const uint8_t Month   = 0x05;
const uint8_t Year    = 0x06;

const uint8_t alarm1Seconds = 0x07;
const uint8_t alarm1Minutes = 0x08;
const uint8_t alarm1Hours   = 0x09;
const uint8_t alarm1DayDate = 0x0A;

const uint8_t alarm2Minutes = 0x0B;
const uint8_t alarm2Hours   = 0x0C;
const uint8_t alarm2DayDate = 0x0D;

const uint8_t control = 0x0E;
const uint8_t status  = 0x0F;

}

namespace valueMask
{
  const uint8_t seconds = 0x0F;
  const uint8_t Dseconds = 0x70;
  const uint8_t minutes = 0x0F;
  const uint8_t Dminutes = 0x70;
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


const uint8_t stateOff    = 0x80;
const uint8_t stateOn     = 0x00;

class Ds3231
{
public:
	enum class convertMode {bcdToDec, decToBcd};
private:
  I2c * driver;
  Pin * sda, *scl;
  uint8_t bcdData [7];
  uint8_t decData [7];
  uint8_t selfAddress = 0xD0;
public:
  Ds3231(I2c &, Pin &sda_, Pin & scl_);
  void write (uint8_t reg, uint8_t val);
  uint8_t read (uint8_t reg);
  void readCalendar ();
  void writeCalendar ();
  void stop ();
  void start ();
  void setData ();
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
