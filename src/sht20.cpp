#include "sht20.h"



Sht20::Sht20 (I2c *d)
{
  driver = d;
  delay_ms(16);
}

void Sht20::readTemperature ()
{
	driver->start ();
	while (!driver->flagBusy());
	//«аписываем в регистр данных адрес ведомого устройства
	driver->setAddress (selfAddress, I2c::directionBit::write);
	if (driver->waitAck() == 0 ) return;
	driver->putData (sht20commands::tMesurementHold);
	if (driver->waitAck() == 0 ) return;
	driver->restart();
	I2C0->C2 |= I2C_C2_SBRC_MASK;
	driver->setAddress (selfAddress, I2c::directionBit::read);
	if (driver->waitAck() == 0 ) return;
	driver->setMode (I2c::mode::receiver);
	driver->generateAck();
	I2C0->C2 &= ~I2C_C2_SBRC_MASK;
	uint8_t dummy = driver->getData();
	driver->waitAck();
	temperatureCode = driver->getData();
	driver->waitAck();
	temperatureCode <<= 8;
	driver->generateNack ();
	temperatureCode |= driver->getData();
	driver->waitAck();
	driver->stop ();
	checksum = driver->getData();
	while (driver->flagBusy());
	temperatureCode &= ~0x0003;
}

void Sht20::readHummidity  ()
{
	driver->start ();
	while (!driver->flagBusy());
	//«аписываем в регистр данных адрес ведомого устройства
	driver->setAddress (selfAddress, I2c::directionBit::write);
	if (driver->waitAck() == 0 ) return;
	driver->putData (sht20commands::rMesurementHold);
	if (driver->waitAck() == 0 ) return;
	driver->restart();
	I2C0->C2 |= I2C_C2_SBRC_MASK;
	driver->setAddress (selfAddress, I2c::directionBit::read);
	if (driver->waitAck() == 0 ) return;
	driver->setMode (I2c::mode::receiver);
	driver->generateAck();
	I2C0->C2 &= ~I2C_C2_SBRC_MASK;
	uint8_t dummy = driver->getData();
	driver->waitAck();
	hummdityCode = driver->getData();
	driver->waitAck();
	hummdityCode <<= 8;
	driver->generateNack ();
	hummdityCode |= driver->getData();
	driver->waitAck();
	driver->stop ();
	checksum = driver->getData();
	while (driver->flagBusy());
	hummdityCode &= ~0x0003;
}

void Sht20::setResolution (resolution r)
{
  res = r;
  
}

void Sht20::readUserR ()
{
	driver->start ();
	while (!driver->flagBusy());
	driver->setAddress (selfAddress, I2c::directionBit::write);
	driver->waitAck();
	driver->putData(sht20commands::readUserRegister);
	driver->waitAck();
	driver->restart();
	driver->setAddress (selfAddress, I2c::directionBit::read);
	driver->waitAck();
	driver->setMode (I2c::mode::receiver);
	driver->generateNack ();
	uRegister = driver->getData();
	driver->waitAck();
	driver->stop ();
	while (driver->flagBusy());
}

void Sht20::writeUserR  (uint8_t data)
{
  driver->start ();
  while (!driver->flagBusy());
  //«аписываем в регистр данных адрес ведомого устройства
  driver->setAddress (selfAddress, I2c::directionBit::write);
  driver->waitAck();
  driver->putData (sht20commands::writeUserRegister);
  driver->waitAck();
  //ќтправка данных
  driver->putData (data);
  driver->waitAck();
  driver->stop ();  
  while (driver->flagBusy());
}

uint8_t & Sht20::getUserRegister ()
{
  return uRegister;
}

uint16_t & Sht20::getHummidity ()
{
	convertHummdity ();
	return hummdity;
}

uint16_t & Sht20::getTemperature ()
{
	convertTemperature ();
	return temperature;
}
  
void Sht20::convertHummdity ()
{
	hummdityCode *=1250;
	hummdityCode>>=16;
	hummdity = hummdityCode - 60;
}

void Sht20::convertTemperature ()
{
	temperatureCode *= 17572;
	temperatureCode >>= 16;
	temperature = (temperatureCode - 4685)/10;
}

