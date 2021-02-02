#include "MK02F12810.h"                 // Device header

uint16_t breakPacketDelay = 90;
uint16_t mabDelay = 10;
uint16_t breakFrameDelay = 0;
uint16_t startDelay = 10;
enum eState {INACTIVE, BREAK_PACKET, MAB, BREAK_FRAME, RECEIVE};
uint8_t state = INACTIVE;
uint8_t receiveBuffer[513];
uint8_t * dmxPtr = receiveBuffer;
uint16_t receiveCounter = 0;
uint16_t endBorder = 512;

void setPinsUart();
void initUart();
void setPinsLow();
void setPinsHigh();
void initFei ();
void initLed();
void initPit0();
void setPause(uint16_t value);
void startPit ();
void stopPit ();
void initFtm();
void initUart();
uint16_t ij = 0;
extern "C" {
	void PORTE_IRQHandler();
	void PIT0_IRQHandler();
	void UART0_RX_TX_IRQHandler();
}
void UART0_RX_TX_IRQHandler(){
	if (UART0->S1&UART_S1_RDRF_MASK){
		receiveBuffer[receiveCounter] = UART0->D;
		++receiveCounter;
	}
	if (receiveCounter>endBorder){
		receiveCounter = 0;
		state = BREAK_PACKET;
		setPinsLow();
	}
}
void PORTE_IRQHandler(){
	PORTE->ISFR |= 1 << 17;
	if (PTE->PDIR & (1 << 17)){//rising edge
		switch(state){
			case BREAK_PACKET:
				setPinsLow();
				stopPit();
			break;
			case MAB:
				setPause(mabDelay);
				setPinsLow();
				startPit();
			break;
		}
	}else{ //falling edge
		switch(state){
			case BREAK_PACKET:
				setPause(breakPacketDelay);
				setPinsHigh();
				startPit ();
			break;
			case MAB:
				state = BREAK_PACKET;
				stopPit();
			break;
		}
	}
}
void PIT0_IRQHandler(){
	PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
	switch(state){
		case BREAK_PACKET:
			state = MAB;
			setPinsHigh();
		break;
		case MAB:
			state = RECEIVE;
			setPinsUart();
			stopPit();
		break;
	}
}

int main(void) {
	//initFei ();
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	setPinsLow();
	state = BREAK_PACKET;
	initPit0();
	//NVIC_EnableIRQ (PORTE_IRQn);
	NVIC_EnableIRQ (PIT0_IRQn);
	NVIC_EnableIRQ (UART0_RX_TX_IRQn);
	setPause(breakPacketDelay);
	startPit ();
    while(1) {
    }
    return 0 ;
}
void initFtm(){
	SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
	FTM0->SC |= FTM_SC_TOIE_MASK |FTM_SC_TOF_MASK;
	FTM0->SC &= ~FTM_SC_PS_MASK ;
	FTM0->SC |= FTM_SC_CLKS(1) // busClock source
				|FTM_SC_PS(4) //div16
				;
}
void setPinsUart(){
	PORTE->PCR[17] = (3 << PORT_PCR_MUX_SHIFT);
}
void setPinsLow(){
	PORTE->PCR[17] = (1 << PORT_PCR_MUX_SHIFT)
	|PORT_PCR_PS_MASK|PORT_PCR_PE_MASK
	|PORT_PCR_IRQC(8)
	;
}
void setPinsHigh(){
	PORTE->PCR[17] = (1 << PORT_PCR_MUX_SHIFT)
	|PORT_PCR_PS_MASK|PORT_PCR_PE_MASK
	|PORT_PCR_IRQC(12)
	;
}


void initUart(){
	//UART2
	SIM->SCGC4 |=  SIM_SCGC4_UART0_MASK;

	uint16_t sbr;

  //===Settings UART===//
	//Disable UART
	UART0->C2 &= ~(UART_C2_RE_MASK|UART_C2_TE_MASK);
	UART0->C1 = 0;

	//calculate baud
	sbr = 20971520/(16*250000);

	UART0->BDH = UART_BDH_SBR(sbr >> 8);

	UART0->BDL = UART_BDL_SBR(sbr);

	UART0->C2 |= UART_C2_RE_MASK
			//|UART_C2_SBK_MASK
					;
}

void initFei (){
	SIM->CLKDIV1 =  SIM_CLKDIV1_OUTDIV1(0) //core clock division = 1
                    |SIM_CLKDIV1_OUTDIV4(1); // bus clock division = 2

	/* Switch to FEI Mode */
	MCG->C1 = MCG_C1_IREFS_MASK // The slow internal reference clock is selected
             |MCG_C1_CLKS(0) // Output of FLL or PLL is selected
             |MCG_C1_FRDIV(0) //
             |MCG_C1_IRCLKEN_MASK; // MCGIRCLK active

	/* MCG->C2: LOCRE0=0,RANGE0=0,HGO0=0,EREFS0=0,LP=0,IRCS=0 */
	//MCG->C2 = MCG_C2_RANGE0(0) // Low frequency range selected for the crystal oscillator
    //         |MCG_C2_HGO0(0) // Configure crystal oscillator for low-power operation.
     //        |MCG_C2_IRCS(1) // Fast internal reference clock selected.
    //         |MCG_C2_FCFTRIM_MASK
    ;
	/* MCG->C4: DMX32=0,DRST_DRS=1 */
	//MCG->C4 |= MCG_C4_DMX32(0) // range 31.25�39.0625 kHz
    //           |MCG_C4_DRST_DRS(1); // fll factor core 1280 * 32kHz = 40960kHz

	//=== Use FLL ===//
	MCG->C5 = 0;
	MCG->C6 = 0;

	while((MCG->S & MCG_S_IREFST_MASK) == 0x00U);

	while((MCG->S & MCG_S_CLKST_MASK) != 0) ;
}
void initPit0(){
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;
	PIT->MCR = 0;
}
void setPause(uint16_t value){
	PIT->CHANNEL[0].LDVAL = 20971520/1000000*value;
}
void startPit (){
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
}
void stopPit (){
	PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;
	PIT->CHANNEL[0].CVAL = 0;
}