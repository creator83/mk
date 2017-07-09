#include "uart.h"


uart::uart (baud b)
:pin (gpio::E)
{
  SIM->SCGC4|= SIM_SCGC4_UART2_MASK;
  uint16_t sbr;
  
  //Settings TX
  pin.setOutPin (TX , gpio::Alt3);
  
  //Settings RX
  
  pin.setOutPin (RX , gpio::Alt3);
  
  
  //===Settings UART===//
	//Disable UART
	UART2->C2 &= ~(UART_C2_RE_MASK|UART_C2_TE_MASK);
	UART2->C1 = 0;
	
	//calculate baud
	sbr = (tact::get_frq_bus()*1000000)/(16*b);
	
	UART2->BDH = UART_BDH_SBR(sbr >> 8);

	UART2->BDL = UART_BDL_SBR(sbr);
	
	UART2->C2 |= (UART_C2_RE_MASK|UART_C2_TE_MASK);
}

void uart::transmit (uint8_t data)
{
	/* Wait until space is available in the FIFO */
  while(!(UART2->S1 & UART_S1_TC_MASK));
   
  /* Send the character */
  UART2->D = data;
}

void uart::transmit (char * str)
{
  while (*str)
  {
    while(!(UART2->S1 & UART_S1_TC_MASK));
    UART2->D = *str;
    str++;
  }
}
