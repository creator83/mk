#include "MKL26Z4.h"               // Device header
#include "delay.h"
#include "i2c.h"


#ifndef SHT20_H
#define SHT20_H

namespace sht20commands
{
  const uint8_t tMesurementHold  = 0xE3;
  const uint8_t rMesurementHold  = 0xE5;
  const uint8_t tMesurement      = 0xF3;
  const uint8_t rMesurement      = 0xF5;
  const uint8_t writeUserRegister= 0xE6;
  const uint8_t readUserRegister = 0xE7;
  const uint8_t softReset        = 0xFE;
}



const uint8_t selfAddress = 0x80;





class Sht20
{
public:
  enum resolution {bit14, bit12, bit13, bit11};

private:
  I2c * driver;
  uint8_t stopCommand;
  uint8_t startCommand;
  uint8_t res, uRegister, checksum;
  uint16_t temperature, hummdity;
  uint32_t temperatureCode, hummdityCode;
//variables
public:
  Sht20 (I2c *);
  void readTemperature ();
  void readHummidity  ();
  void setResolution (resolution);
  void readUserR ();
  void writeUserR  (uint8_t);
  uint8_t & getUserRegister ();
  uint16_t & getHummidity ();
  uint16_t & getTemperature ();
  
private:
  void convertHummdity ();
  void convertTemperature ();
};

#endif
