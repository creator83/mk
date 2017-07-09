#include "device.h"                 // Device header
#include "pin.h"


#ifndef SPISOFT_H_
#define SPISOFT_H_

class Spis
{
public:


private:
	Pin * cs, *sck, *mosi, *miso;

public:
	Spis(Pin & cs_, Pin & sck_, Pin & mosi_, Pin & miso_);

	void transmite (uint8_t);
	uint8_t receive ();
	void chipEnable ();
	void chipDisable ();

};

#endif
