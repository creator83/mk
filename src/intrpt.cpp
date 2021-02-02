#include "intrpt.h"

Intrpt::Intrpt (port p, uint8_t pi_, mode m, Gpio::PP f)
:pin_(p, pi_, f){
    NVIC_EnableIRQ(pinInt[static_cast<uint8_t>(p)]);
}

void Intrpt::clearFlag (){
	pin_.getPort()->ISFR|= 1 << pin_.getPin();
}
