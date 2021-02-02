#include "device.h"    
#include "mcg.h"

#ifndef TACT_H
#define TACT_H

class Tact
{
  //variables
public:
	enum class pllFllSource {mcgFllClk, irc48=3};
	enum class clockMode {fei, fee, fbi, fbe, blpi, blpe};
	enum class lptmrSource {osc32, lpo=3};
	enum class dividers:uint8_t {div1, div2, div3, div4, div5, div6, div7, div8, div9, div10,
	                                div11, div12, div13, div14, div15, div16};
	enum class oscilator {mhz4, mhz8, mhz10, mhz16};
private:
	uint8_t nPllFllSource;
	uint8_t nLptmrSource;
	static uint32_t cpuClock, busClock, flexBusClock, flashClock, mcgIRClock, mcgFFClock, mcgFLLClock, altAdcClock;
	uint8_t busClockDivider, coreClockDivider, flashClockDivider, flexBusDivider, clockMode_;
	Mcg mcg;
	using ptrModeFunc = void (Tact::*)();
	static ptrModeFunc modeFunc [6];
public:
	Tact ();
	Tact (clockMode);
	uint32_t & getFrqBus ();
	uint32_t & getFrqCpu ();
	uint32_t & getFrqFlash ();
	uint32_t & getFrqFlexBus ();
protected:
	void initFei ();
	void initFee ();
	void initFbi ();
	void initFbe ();
	void initBlpi ();
	void initBlpe ();
	void setPllFllSource (pllFllSource);
	void setLptmrSource (lptmrSource);
private:
	void setCoreClockDvider (dividers);
	void setBusClockDvider (dividers);
	void setFlexBusClockDvider (dividers);
	void setFlashClockDvider (dividers);
};

 
#endif

