#include "device.h"                  // Device header

#ifndef DMA_H
#define DMA_H

class dma;


class Dma
{
	//variables
public:
	enum class size {bit32, bit8, bit16};
private:
	uint8_t ch;
	//functions
public:	
	Dma ();
	Dma (dmaChannel ch);
	void enableDmaMux (dmaMux);
	void setChannel (dmaChannel ch);
	void setSource (uint32_t  ptr);
	void setDestination (uint32_t ptr);
	void setMinorLoop (uint16_t);
    void setMajorLoop (uint16_t);
	void setSizes (size m, size p);
	void setSsize (size);
	void setDsize (size);
	void setOffsetDestination (uint32_t);
	void setOffsetSource (uint32_t);
	void setSLast (uint32_t);
	void setDLast (uint32_t);
	void enableInterrupt ();
	void disableInterrupt ();
	void enablePeriph ();
	void disablePeriph ();
	void start ();
	void clearFlags ();
	bool flagDone();
	uint8_t & getChannel ();
};

#endif
