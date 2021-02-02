//#include "device.h"                    // Device header
#include "MK10D10.h"                    // Device header

//#include "gpio.h"
class Device {
	Device(){}
};
void BOARD_InitPins();
void BOARD_BootClockRUN();
int main(){
	uint32_t a;
    /* Initialize board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    while (1){
		
    }
}
void BOARD_InitPins(){
	
}
void BOARD_BootClockRUN(){
	
}
