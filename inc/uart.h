#include "MKL17Z4.h"                 // Device header
#include "gpio.h"
#include "tact.h"


#ifndef UART_H
#define UART_H

typedef unsigned int* reg;


class uart
{
//variables
public:
  enum baud {baud9600 = 9600 , baud57600 = 57600 , baud115200 = 115200};
private:
  gpio pin;
  enum pin_def {TX = 16 , RX};
//functions
public:
  uart (baud b);
  void transmit (uint8_t data);
  void transmit (char * str);  
  uint8_t receive ();
};

#endif
