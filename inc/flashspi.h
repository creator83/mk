#include "device.h"
#include "pin.h"
#include "spi.h"
#include "delay.h"
#include "dma.h"
#include "pit.h"

#ifndef FLASHSPI_H
#define FLASHSPI_H


namespace flashCommands {
const uint8_t WriteEnable = 0x06;
const uint8_t WriteDisable = 0x04;
const uint8_t ReadStatusReg1 = 0x05;
const uint8_t ReadStatusReg2 = 0x35;
const uint8_t ReadStatusReg3 = 0x15;
const uint8_t WriteStatusReg = 0x01;
const uint8_t ReadData = 0x03;
const uint8_t FastReadData = 0x0B;
const uint8_t FastReadDual = 0x3B;
const uint8_t PageProgram = 0x02;
const uint8_t BlockErase = 0xD8;
const uint8_t SectorErase = 0x20;
const uint8_t ChipErase = 0xC7;
const uint8_t PowerDown = 0xB9;
const uint8_t ReleasePowerDown = 0xAB;
const uint8_t DeviceID = 0xAB;
const uint8_t ManufactDeviceID = 0x90;
const uint8_t JedecDeviceID = 0x9F;
}


class Flash
{
private:
  Spi * driver;
  Pin cs;
  uint16_t deviceId, dummy;
  uint8_t manufacturId;
  Dma * transmitter;
  Dma * txDummy;
  Pit * timer;

public:

  Flash (Spi &, Gpio::Port p, uint8_t pin);
  void writeByte (uint8_t);
  void writePage (const uint8_t * buffer, uint32_t addr, uint16_t n);
  void writePage16 (const uint16_t * buffer, uint32_t addr, uint16_t n);
  void write (uint8_t * buffer, uint32_t addr, uint16_t n);
  void readID ();
  uint16_t readStatus1 ();
  uint16_t readStatus2 ();
  uint16_t readStatus3 ();
  void writeStatus (uint8_t);
  void writeEnable ();
  void writeDisable ();
  void read (uint8_t * buffer, uint32_t addr, uint16_t n);
  void read16 (uint16_t * buffer, uint32_t addr, uint16_t n);
  void read16Dma (uint16_t * buffer, uint32_t addr, uint32_t n);
  void txToDma (uint32_t dest, uint32_t addr, uint32_t n);
  void dataDma (uint32_t dest, uint32_t n);
  void txDum (uint32_t n);
  void eraseSector (uint32_t addr);
  void eraseChip ();
  bool flagBusy ();
  uint8_t command (uint8_t);
  uint16_t command16 (uint8_t h, uint8_t l);
  void powerDown ();
  void powerUp ();
  void getCapacity ();
  void setDma (Dma &tx, Dma &dum);


};


#endif
