#include "spisoft.h"

Spis::Spis(Pin & cs_, Pin & sck_, Pin & mosi_, Pin & miso_)
:cs(&cs_), sck (&sck_), mosi(&mosi_), miso (&miso_)
{
	cs->set();
	sck->clear();
}

void Spis::transmite (uint8_t data)
{
	for (uint8_t i=0;i<8;++i)
	{

		if (data&(1<<(7-i)))
		{
			mosi->set();
		}
		else
		{
			mosi->clear();
		}
		sck->set();
		sck->clear();
	}
}

uint8_t Spis::receive ()
{
	uint8_t result=0;
	for (uint8_t i=0;i<8;++i)
	{
		sck->set();
		result |= miso->state() << (7-i);
		sck->clear();
	}
	return result;
}

void Spis::chipEnable ()
{
	cs->clear ();
}

void Spis::chipDisable ()
{
	cs->set();
}

