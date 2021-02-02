#include "uart.h"


Uart::Uart (numberUart n, Pin &rx_, Pin &tx_, baud b)
:rx(&rx_), tx(&tx_), baudRate(static_cast <uint32_t>(b)){
	uint16_t sbr, brfa;
	uint8_t temp;
    number = static_cast<uint8_t>(n);
    uartPtr = ((UART_Type *)uartAddress[number]);
    //clock UART
    *((uint32_t*)uartClock[number][0]) |= uartClock[number][1];
    //Disable UART
	UART2->C2 &= ~(UART_C2_RE_MASK|UART_C2_TE_MASK);
	UART2->C1 = 0;
	//calculate baud
	sbr = (Tact::getFrqBus()*1000)/(16*baudRate);
	
	/* Save off the current value of the UARTx_BDH except for the SBR field */
    temp = uartPtr->BDH & ~( UART_BDH_SBR(0x1F));
    
    uartPtr->BDH = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
    uartPtr->BDL = (uint8_t)(sbr & UART_BDL_SBR_MASK);
    
    /* Determine if a fractional divider is needed to get closer to the baud rate */
    brfa = (((Tact::getFrqBus()*32000)/(baudRate * 16)) - (sbr * 32));
    
    /* Save off the current value of the UARTx_C4 register except for the BRFA field */
    temp = uartPtr->C4 & ~(UART_C4_BRFA(0x1F));
    
    uartPtr->C4 = temp |  UART_C4_BRFA(brfa);
	
	UART2->C2 |= (UART_C2_RE_MASK|UART_C2_TE_MASK);
}
Uart::Uart (numberUart n, Pin &tx_, baud b)
:tx(&tx_), baudRate(static_cast <uint32_t>(b)){
	uint16_t sbr, brfa;
	uint8_t temp;
    number = static_cast<uint8_t>(n);
    uartPtr = ((UART_Type *)uartAddress[number]);
    //clock UART
    *((uint32_t*)uartClock[number][0]) |= uartClock[number][1];
    //Disable UART
	UART2->C2 &= ~(UART_C2_RE_MASK|UART_C2_TE_MASK);
	UART2->C1 = 0;
    	//calculate baud
		//calculate baud
	sbr = (Tact::getFrqBus()*1000)/(16*baudRate);
	
	/* Save off the current value of the UARTx_BDH except for the SBR field */
    temp = uartPtr->BDH & ~( UART_BDH_SBR(0x1F));
    
    uartPtr->BDH = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
    uartPtr->BDL = (uint8_t)(sbr & UART_BDL_SBR_MASK);
    
    /* Determine if a fractional divider is needed to get closer to the baud rate */
    brfa = (((Tact::getFrqBus()*32000)/(baudRate * 16)) - (sbr * 32));
    
    /* Save off the current value of the UARTx_C4 register except for the BRFA field */
    temp = uartPtr->C4 & ~(UART_C4_BRFA(0x1F));
    
    uartPtr->C4 = temp |  UART_C4_BRFA(brfa);
	
	UART2->C2 |= UART_C2_TE_MASK;
}
void Uart::transmit (uint8_t data){
	/* Wait until space is available in the FIFO */
	while(!(UART2->S1 & UART_S1_TC_MASK));
   
  /* Send the character */
	UART2->D = data;
}

void Uart::transmit (char * str){
	while (*str){
    while(!(UART2->S1 & UART_S1_TC_MASK));
    UART2->D = *str;
    str++;
  }
}
