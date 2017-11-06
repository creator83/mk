#include "device.h"    
#include "pin.h" 
#include "can_frame.h"

#ifndef CAN_H
#define CAN_H


class Can
{
public:
	enum class nCan {can0, can1};
	enum class filter {enable, disable};
private:
	using voidFunc = void (Can::*)();
	static CAN_Type * canBase[2];
	Pin *tx, *rx;
	uint8_t canNumber;
	static voidFunc tactPtr  [2];

	CanFrame * frame;
	
public:
  
  Can (nCan n, Pin *tx_, Pin *rx_);
	void receiveEnableMb (CanFrame *);
	bool transmite (CanFrame *, uint8_t *);
	bool receive (CanFrame *, uint8_t *); 
private:
	void tactCan0();
	void tactCan1();
	void init(filter f);
};

 
#endif

