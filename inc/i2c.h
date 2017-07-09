#include "device.h"                 // Device header
#include "tact.h"

#ifndef I2C_H
#define I2C_H

const uint8_t readBit = 1;
const uint8_t writeBit = 0;

class I2c
{
//variables
public:

enum class nI2c {I2c0,I2c1};
enum class mode {receiver, transmitter};
enum class directionBit {write, read};


protected:


private:
	static I2C_Type * i2cAdr [2];
	uint8_t numberI2c;
//functions
public:
  I2c (nI2c);
  void start ();
  void restart ();
  void stop ();
  void putData (uint8_t);
  uint8_t getData ();
  void setAddress (uint8_t, directionBit);
  void write (uint8_t);
  uint8_t read (bool);
  void read (uint8_t *, uint8_t addr, uint8_t reg, uint16_t n);
  bool flagBusy ();
  bool flagTcf ();
  bool flagIicif ();
  bool flagRxak ();
  void clearIicif ();

  void wByte (uint8_t address, uint8_t reg, uint8_t data);
  uint8_t rByte (uint8_t address, uint8_t reg);

  void generateAck ();
  void generateNack ();
  bool waitAck ();
  void setMode (mode);

private:
};



#endif

