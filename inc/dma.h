#include "device.h"                  // Device header

#ifndef DMA_H
#define DMA_H

class dma;


class Dma
{
	//variables
public:
	enum class size {bit8, bit16, bit32};
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
	void setCountMnrLoop (uint32_t);
    void setCountMjrLoop (uint16_t);
	void setSsize (size);
	void setDsize (size);
	void setOffsetDestination (uint32_t);
	void setOffsetSource (uint32_t);
	void setSLast (int32_t);
	void setDLast (int32_t);
    void setMjrLink(dmaChannel);
    void setMjrLink(Dma &);
    void triggerEnable();
    void triggerDisable();
	void enableInterruptMajor ();
	void disableInterruptMajor ();
    void startRequest ();
	void start ();
	void clearFlags ();
    void setDREQ();
	bool flagDone();
	uint8_t & getChannel ();
};

#endif
