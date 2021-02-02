#include "device.h"                   // Device header
#include "gpio.h"
#include "pin.h"

void initI2S(){
    //pin
	PORTA->PCR[5] = (6 << PORT_PCR_MUX_SHIFT);
	PORTA->PCR[12] = (6 << PORT_PCR_MUX_SHIFT);
	PORTA->PCR[13] = (6 << PORT_PCR_MUX_SHIFT);
    Pin tx_d0 (port::C, 1, mux::Alt6);
    Pin tx_fs (port::C, 2, mux::Alt6);
    Pin tx_bclk (port::C, 3, mux::Alt6);
	
    SIM->SCGC6 |= SIM_SCGC6_I2S_MASK;
	

	//tact
	I2S0->TCR2 = I2S_TCR2_SYNC(0)| I2S_TCR2_MSEL(1)|I2S_TCR2_BCP(1)|I2S_TCR2_BCD(1);
	I2S0->TCR3 = I2S_TCR3_TCE(1);
	I2S0->TCR4 = I2S_TCR4_FRSZ(1) //2 words
			|I2S_TCR4_SYWD(15) // 16 bit
			|I2S_TCR4_MF(1) // MSB
			|I2S_TCR4_FSE(1) // one bit early
			|I2S_TCR4_FSP(1) // frame active low
			|I2S_TCR4_FSD(1) // master mode
			;
	I2S0->TCR5 = I2S_TCR5_WNW(15)|I2S_TCR5_W0W(15);
	I2S0->TMR = 0;
	I2S0->TCSR = I2S_TCSR_TE_MASK // enable tx
			|I2S_TCSR_BCE_MASK // enable bitclock
			//|I2S_TCSR_FWDE_MASK // enable DMA
			;
	I2S0->MCR = I2S_MCR_MOE_MASK // MCLK = output
			|I2S_MCR_MICS(0);
	I2S0->TCR2 |= I2S_TCR2_DIV(3);
	I2S0->MDR = I2S_MDR_DIVIDE(63) | I2S_MDR_FRACT(124);
}

int main ()
{
    Pin led1 (port::B, 23);
    led1.set();
    led1.clear();


	while (1)
	{

	}
}
