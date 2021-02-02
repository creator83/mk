#include "device.h"    


#ifndef CAN_FRAME_H
#define CAN_FRAME_H


class CanFrame
{
public:
	enum class frameType {standart, extended};
	enum class remoteFrame {disable, enable};
private:
	
	uint32_t id;
	uint32_t mbIndex;
	frameType ide;
	remoteFrame rtr;
	uint8_t dlc;

		
public:
  
    CanFrame (uint32_t id, uint32_t mbIndex, frameType ide, remoteFrame rtr, uint8_t dlc);
	uint32_t & getId ();
	uint32_t & getMbIndex ();
	frameType & getIde ();
	remoteFrame & getRtr ();
	uint8_t & getDlc ();
	uint16_t getCode(uint32_t);
	uint8_t getLength(uint32_t);
	void setIde (frameType);
	void setId (uint32_t);
private:

};

 
#endif

