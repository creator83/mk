#include "device.h"                   // Device header
#include "gpio.h"
#include "pin.h"

void initI2S(){
    //pin
    Pin tx_d0 (port::E, 10, mux::Alt4);
    Pin tx_fs (port::E, 11, mux::Alt4);
    Pin tx_bclk (port::E, 12, mux::Alt4);
	
    SIM->SCGC6 |= SIM_SCGC6_I2S_MASK;
	
	//tact
	I2S0->TCR2 = I2S_TCR2_SYNC(0)
                | I2S_TCR2_MSEL(0)
                |I2S_TCR2_BCP_MASK
                |I2S_TCR2_BCD_MASK;
	I2S0->TCR3 = I2S_TCR3_TCE(1);
	I2S0->TCR4 = I2S_TCR4_FRSZ(1) //2 words
			|I2S_TCR4_SYWD(15) // 16 bit
			|I2S_TCR4_MF_MASK // MSB
			|I2S_TCR4_FSE_MASK // one bit early
			|I2S_TCR4_FSP_MASK // frame active low
			|I2S_TCR4_FSD_MASK // master mode
			;
	I2S0->TCR5 = I2S_TCR5_WNW(15)|I2S_TCR5_W0W(15);
	I2S0->TMR = 0;
	I2S0->TCSR = I2S_TCSR_TE_MASK // enable tx
			|I2S_TCSR_BCE_MASK // enable bitclock
			//|I2S_TCSR_FWDE_MASK // enable DMA
			;
	I2S0->MCR = I2S_MCR_MICS(0)
                |I2S_MCR_MOE_MASK // MCLK = output
                ;
	I2S0->TCR2 |= I2S_TCR2_DIV(14);
	I2S0->MDR = I2S_MDR_DIVIDE(63) | I2S_MDR_FRACT(124);
}

uint16_t data [8] = {0x000f, 0x00f0, 0x0f00, 0xf000, 0x00ff, 0x0ff0, 0xff00, 0xf00f};

int main ()
{
    initI2S();
    uint8_t counter;
	while (1)
	{
        counter++;
        for(uint8_t i = 0;i<8;++i){
            I2S0->TDR[0] = (data[i]<<16)&0xFFFF0000;
        }
	}
}
