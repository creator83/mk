#include "tact.h"



Tact::Tact ()
{
	states[1] = &stateFei;
	forFei->feiTofee = 1;
	states[1]->switchState (forFei->feiTofee);
}
