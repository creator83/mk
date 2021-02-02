#include "device.h"                    // Device header
#include "uart.h"
#include "tact.h"



int main ()
{
	Tact frq;
	Pin uart0Tx (port::A, 14, mux::Alt3);
	Uart uart0 (numberUart::UART_0, uart0Tx, Uart::baud::baud9600);
	uart0.transmit ("Hello from Freescale!!!");
	while (1){		
		
	}
}
