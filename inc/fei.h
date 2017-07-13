#include "device.h"  
#include "state.h" 

#ifndef FEI_H
#define FEI_H


class Fei : public State
{
  //variables
public:

private:
stateVar* st;

public:
  Fei ();
	void switchState (stateVar)override;

private:

  void init ();

};

 
#endif

