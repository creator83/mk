#include "device.h"    
#include "mcg.h"

#ifndef TACT_H
#define TACT_H


class Tact
{
  //variables
public:
	enum class pllFllSource {mcgFllClk, irc48=3};
	enum class lptmrSource {osc32, lpo=3};
private:
	uint8_t nPllFllSource;
	uint8_t nLptmrSource;
  /*static uint16_t cpuClock;
  static uint16_t busClock;
  static uint16_t mcgirClock;
  static uint16_t mcgpllClock;
  static uint16_t mcgfllClock;
*/
	uint16_t cpuClock, busClock, mcgirClock, mcgpllClock, mcgfllClock;
	Mcg mcg;
	static Tact * _instance;
public:
  
	static Tact * getInstance();
	uint16_t & getFrqBus ();
	uint16_t & getFrqCpu ();
	uint16_t & getFrqFlex ();
private:
	Tact ();
	void init ();
	void initFei ();
	void initFee ();
	void initFbi ();
	void initFbe ();
	void initPee ();
	void setPllFllSource (pllFllSource);
	void setLptmrSource (lptmrSource);
};

 
#endif

