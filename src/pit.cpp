#include "pit.h"

ptr Pit::set_func [4] = {&Pit::khz_set, &Pit::hz_set, &Pit::ms_set, &Pit::us_set};



Pit::Pit (channel ch, uint16_t n, mode m){
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT->MCR = 0;
	n_ch = static_cast <uint8_t> (ch);
	(this->*(Pit::set_func[static_cast <uint8_t> (m)]))(n);
	tact = Tact::getInstance();


	//enable interrupt
	//NVIC_EnableIRQ (PIT_IRQn);

}

void Pit::khz_set (uint16_t val){
	PIT->CHANNEL[n_ch].LDVAL = tact->getFrqBus ()/val;
}
void Pit::hz_set (uint16_t val){
	PIT->CHANNEL[n_ch].LDVAL = tact->getFrqBus ()*1000/val;
}
void Pit::ms_set (uint16_t val){
	PIT->CHANNEL[n_ch].LDVAL = tact->getFrqBus ()*val;
}

void Pit::us_set (uint16_t val){
	PIT->CHANNEL[n_ch].LDVAL = tact->getFrqBus ()/1000*val;
}

void Pit::clear_flag (){
	PIT->CHANNEL[n_ch].TFLG |= PIT_TFLG_TIF_MASK;
}

void Pit::start (){
	PIT->CHANNEL[n_ch].TCTRL |= PIT_TCTRL_TEN_MASK;
}

void Pit::stop (){
	PIT->CHANNEL[n_ch].TCTRL &= ~PIT_TCTRL_TEN_MASK;
}

void Pit::interrupt_enable (){
	PIT->CHANNEL[n_ch].TCTRL |= PIT_TCTRL_TIE_MASK;
	NVIC->ISER[(((uint32_t)(int32_t)(irq_n+n_ch)) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)(irq_n+n_ch)) & 0x1FUL));
}

void Pit::interrupt_disable (){
	PIT->CHANNEL[n_ch].TCTRL &= ~PIT_TCTRL_TIE_MASK;
	NVIC->ICER[(((uint32_t)(int32_t)(irq_n+n_ch)) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)(irq_n+n_ch)) & 0x1FUL));
}

bool Pit::flag_TIF (){
	return (PIT->CHANNEL[n_ch].TFLG & PIT_TFLG_TIF_MASK);
}

