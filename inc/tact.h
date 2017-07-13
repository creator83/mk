#include "device.h"   
#include "state.h"  
#include "fei.h"

#ifndef TACT_H
#define TACT_H


class Tact
{
  //variables
public:

private:
State *states[3];
struct
{
	stateVar feiTofee;
}*forFei;
Fei stateFei;

public:
  Tact ();
  

private:


};

 
#endif

