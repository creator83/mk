#include "device.h"             // Device header

#ifndef STATE_H
#define STATE_H

using stateVar = uint8_t;
class State
{
  //variables
public:

 private:
static uint16_t cpuClock;
static uint16_t busClock;
static uint16_t mcgirClock;
static uint16_t mcgpllClock;
static uint16_t mcgfllClock;
	

public:
  
  virtual void switchState (stateVar)=0;
	virtual ~State(){}

};

 
#endif

