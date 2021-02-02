#include "device.h"                 // Device header
#include "pin.h"
#include "tact.h"

#ifndef UART_H
#define UART_H

class Uart
{
//variables
public:
    enum class baud {baud9600 = 9600 , baud57600 = 57600 , baud115200 = 115200};
private:
    Pin *rx, *tx;
    UART_Type * uartPtr;
    uint8_t number;
	uint32_t baudRate;
public:
	Uart (numberUart, Pin &rx_, Pin &tx, baud b);
	Uart (numberUart, Pin &tx, baud b);
	void transmit (uint8_t data);
	void transmit (char * str);  
	uint8_t receive ();
};

#endif
