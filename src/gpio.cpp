#include "gpio.h"

GPIO_Type * Gpio::GpioBase [5] = {PTA, PTB, PTC, PTD, PTE};
PORT_Type * Gpio::PortBase [5] = {PORTA, PORTB, PORTC, PORTD, PORTE};


Gpio::Gpio ()
{

}

Gpio::Gpio (Port p)
{
  prt = static_cast <uint8_t> (p);
  //takt port
  SIM->SCGC5 |= (0x200 << prt);
}


Gpio::Gpio (uint8_t p)
{
  prt = p;
  //takt port
  SIM->SCGC5 |= (0x200 << prt);
}



