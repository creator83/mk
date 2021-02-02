#include "device.h"    
#include "mcg.h"

#ifndef STATE_H
#define STATE_H

class State
{
  //variables
public:
	enum class pllFllSource {mcgFllClk, irc48=3};
	enum class lptmrSource {osc32, lpo=3};
private:
	uint8_t nPllFllSource;
	uint8_t nLptmrSource;
    static State * instancePtr;
	static uint32_t cpuClock, busClock, mcgIRClock, mcgFFClock, mcgFLLClock;
	Mcg mcg;
public:
    
    static State & getInstance();
	uint32_t & getFrqBus ();
	uint32_t & getFrqCpu ();
	uint32_t & getFrqFlash ();
private:
    State ();
protected:
	void initFei ();
	void initFee ();
	void initFbi ();
	void initFbe ();
	void initPee ();
	void setPllFllSource (pllFllSource);
	void setLptmrSource (lptmrSource);
};

 
#endif

