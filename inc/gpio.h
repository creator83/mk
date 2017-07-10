#include "device.h"              // Device header


#ifndef GPIO_H
#define GPIO_H

class Gpio
{
//variables
public:
  enum class Port {A , B , C , D , E};
  enum class mux {Analog, GPIO , Alt2 , Alt3 , Alt4 , Alt5 , Alt6 , Alt7};
  enum class mode {Input, Output};
  enum class PP {PullDown, PullUp};
  enum class state {Off, On};

protected:
  static PORT_Type * PortBase [5];
  static GPIO_Type * GpioBase [5];
  uint8_t prt;
private:

//functions
public:
  Gpio ();
  Gpio(Port p );
  Gpio(uint8_t p );
};

#endif



