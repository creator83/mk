#include "nrf24l01.h"

uint8_t Nrf24l01::selfAddress[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

uint8_t Nrf24l01::remoteAddress[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};

Nrf24l01::Nrf24l01 (Spi &d)
:cs (nrf24Def::csPort, nrf24Def::csPin), ce (nrf24Def::cePort, nrf24Def::cePin)
//,irq (nrf24Def::irqPort, nrf24Def::irqPin, Intrpt::mode::fallingEdge)
{
  delay_ms (15);
  //NVIC_EnableIRQ(PORTC_PORTD_IRQn);
  driver = &d;
  driver->setCpol(Spi::Cpol::neg);
  driver->setCpha(Spi::Cpha::first);
  driver->setDivision(Spi::Division::div64);
  driver->setFrameSize(Spi::Size::bit8);
  driver->setMode(Spi::Mode::software);
  Pin sck (Gpio::Port::E, 17, Gpio::mux::Alt2);
  Pin mosi (Gpio::Port::E, 18, Gpio::mux::Alt2);
  Pin miso (Gpio::Port::E, 19, Gpio::mux::Alt2);
  cs.set ();
  driver->start();
  delay_ms (15);


  writeRegister (CONFIG, (1 <<PWR_UP | 1 << EN_CRC));
  delay_ms (2);
  uint8_t status = readStatus();
  writeRegister (STATUS, status);
  comm (FLUSH_TX);
  writeRegister(RX_PW_P0, 1);
  chan = 3;
 /* //checking
  startup = init ();
  
  //settings register
  
  writeRegister(EN_AA, (1 << ENAA_P1)); // включение автоподтверждения только по каналу 1
  writeRegister(EN_RXADDR, (1 << ERX_P0) | (1 << ERX_P1)); // включение каналов 0 и 1
  writeRegister(SETUP_AW, SETUP_AW_5BYTES_ADDRESS); // выбор длины адреса 5 байт
  writeRegister(SETUP_RETR, SETUP_RETR_UP_TO_2_RETRANSMIT); 
  writeRegister(RF_CH, chan); // Выбор частотного канала
  writeRegister(RF_SETUP, RF_SETUP_0DBM); // выбор скорости 1 Мбит/с и мощности 0dBm
  
  writeRegister(RX_ADDR_P0, remoteAddress, 5); // Подтверждения приходят на канал 0 
  writeRegister(TX_ADDR, remoteAddress, 5);

  writeRegister(RX_ADDR_P1, selfAddress, 5);
  
  writeRegister(RX_PW_P0, 0);
  writeRegister(RX_PW_P1, 32); 
  writeRegister(DYNPD, (1 << DPL_P0) | (1 << DPL_P1)); // включение произвольной длины для каналов 0 и 1
  writeRegister(FEATURE, 0x04); // разрешение произвольной длины пакета данных
*/
  //===Standby-1 mode===//

  rxState ();
}

void Nrf24l01::rxState ()
{
  //переключение в RX Mode
  writeRegister (CONFIG, (1 <<PWR_UP | 1 << EN_CRC/*|1 << CRCO*/| 1 << PRIM_RX));
  //changeBit (CONFIG, PRIM_RX, 1);
  ce.set();
  delay_us(140);
}

void Nrf24l01::txState ()
{
  ce.clear ();
  writeRegister (CONFIG, (1 <<PWR_UP | 1 << EN_CRC));
  ce.set ();
  delay_us(15);
  ce.clear ();
  delay_us(140);
}

void Nrf24l01::command (uint8_t com)
{
  cs.clear ();
  while (!driver->flagSptef());
  driver->putDataDl(com);
}

void Nrf24l01::comm (uint8_t com)
{
  cs.clear ();
  while (!driver->flagSptef());
  driver->putDataDl(com);
  while (!driver->flagSprf());
  uint8_t status = driver->getDataDl();
  cs.set ();
}

uint8_t Nrf24l01::readRegister (uint8_t reg)
{
  command (R_REGISTER|reg);
  while (!driver->flagSprf());
  uint8_t status = driver->getDataDl();
  while (!driver->flagSptef());
  driver->putDataDl(NOP);
  while (!driver->flagSprf());
  uint8_t reg_val = driver->getDataDl();
  cs.set ();
  return reg_val;   
}

uint8_t Nrf24l01::readStatus ()
{
  command (NOP);
  while (!driver->flagSprf());
  uint8_t status = driver->getDataDl();
  cs.set ();
  return status;
}

void Nrf24l01::writeRegister (uint8_t reg , uint8_t val)
{
  command (W_REGISTER|reg);
  while (!driver->flagSprf());
  uint8_t status = driver->getDataDl();
  while (!driver->flagSptef());
  driver->putDataDl (val);
  while (!driver->flagSprf());
  uint8_t dummy = driver->getDataDl();
  cs.set ();
}

void Nrf24l01::writeRegister (uint8_t reg , uint8_t * val, uint8_t count)
{
  command (W_REGISTER|reg);
  while (!driver->flagSprf());
  uint8_t status = driver->getDataDl();
  while (count--)
  {
	while (!driver->flagSptef());
	driver->putDataDl (*val++);
	while (!driver->flagSprf());
	uint8_t dummy = driver->getDataDl();
  }
  cs.set ();
}

void Nrf24l01::changeBit (uint8_t reg, uint8_t bit, bool state)
{
  uint8_t val = readRegister (reg);
  
  val = val&(~(1 << bit));
  val = val|(state << bit);
  writeRegister (reg, val);
}

void Nrf24l01::sendByte (uint8_t val)
{
  command (W_TX_PAYLOAD);
  while (!driver->flagSprf());
  uint8_t status = driver->getDataDl();
  while (!driver->flagSptef());
  driver->putDataDl (val);
  while (!driver->flagSprf());
  uint8_t dummy = driver->getDataDl();
  cs.set ();
  txState ();
  rxState ();
}

bool Nrf24l01::init ()
{
   for(uint8_t i = 0;i<100;++i) 
   {
    writeRegister(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PRIM_RX)); // Выключение питания
    if (readRegister(CONFIG) == ((1 << EN_CRC) | (1 << CRCO) | (1 << PRIM_RX)))
                           
    {
      count = i;
      return true;
    }
      // Если после 100 попыток не удалось записать что нужно, то выходим с ошибкой
    delay_ms(1);
   }
  return false;
}

uint8_t Nrf24l01::receiveByte ()
{
  command (R_RX_PAYLOAD);
  while (!driver->flagSprf());
  uint8_t status = driver->getDataDl();
  while (!driver->flagSptef());
  driver->putDataDl (NOP);
  while (!driver->flagSprf());
  uint8_t value = driver->getDataDl();
  cs.set ();
  return value;
}

void Nrf24l01::clearFlag ()
{
	//irq.clearFlag();
}

void Nrf24l01::enableChannels (channel)
{

}


