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
  static uint16_t cpuClock;
  static uint16_t busClock;
  static uint16_t mcgirClock;
  static uint16_t mcgpllClock;
  static uint16_t mcgfllClock;
	Mcg mcg;
public:
  Tact ();
  
private:
	
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

