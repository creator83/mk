#include "device.h"    


#ifndef TACT_H
#define TACT_H


class Tact
{
  //variables
public:
	

private:
	uint8_t nPllFllSource;
	uint8_t nLptmrSource;
	static uint32_t cpuClock, busClock;
	uint32_t mcgiIrClock;
	uint32_t mcgPllClock;
	uint32_t mcgFllClock;
	uint32_t mcgOutClock;
	uint32_t fllClock;
	uint32_t extClock;
	uint32_t ircSlowClock;
	uint32_t ircFastClock;
	uint32_t * mcgOutClockPtr[3];
	state currentState;
	uint8_t clockSource;
	uint8_t periphSourceValue;
	uint8_t extFllDividerValue;
	static Tact * _instance;
public:
  
	static Tact * getInstance();
	Tact ();
	static uint32_t & getFrqBus ();
	static uint32_t & getFrqCpu ();
	static uint32_t & getFrqFlex ();
private:
	
	void init ();
	void initFei ();
	void initFee ();
	void initFbi ();
	void initFbe ();
	void initPee ();
	void setPeriphSource (periphSource);
	uint8_t getPeriphSource ();
	void setLptmrSource (lptmrSource);
	void setClockSourceSelect (clockSourceSelect);
	uint8_t getClockSourceSelect ();
	void setExtFllDivider(fllDivider);
	void setClockDivider (dividers, clockDivider);
	void setReferenceClockFll (fllSource);
	uint8_t getReferenceClockFll ();
	void setPll (bool state);
	uint8_t getPllstate ();
};

 
#endif

