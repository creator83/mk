#include "MKL17Z4.h"                    // Device header
#include "uart.h"
#include "tact.h"

tact frq;

int main ()
{
	uart uart2 (uart::baud9600);
	uart2.transmit ("Hello from Freescale!!!");
	int *ptr = nullptr;
	while (1)
	{
		
		
	}
}
