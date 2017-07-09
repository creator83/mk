#include "device.h"


#ifndef LPTMR_H
#define LPTMR_H


class Lptmr
{
  //variables
public:
	enum class division {div2, div4, div8, div16, div32, div64, div128, div256, div512, div1024, div2048, div4096, div8192, div16384, div32768, div64536};
	enum class source {mcgirclk, lpo, erclk32k, oscerclk};
private:
	uint8_t s;
public:
	Lptmr(source s_= source::lpo);
	Lptmr(division d, source s_= source::lpo);
	void start ();
	void stop ();
	void setComp (uint16_t val);
	void setCnt (uint16_t val);
	void clearTCF ();
	void interruptEnable ();
	void interruptDisable ();
private:

};

#endif /* PIT_H */
