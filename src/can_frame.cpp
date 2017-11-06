#include "can_frame.h"

CanFrame::CanFrame (uint32_t id_, uint32_t mbIndex_, frameType ide_, remoteFrame rtr_, uint8_t dlc_){
	id = id_;
	mbIndex = mbIndex_;
	ide = ide_;
	rtr = rtr_;
	dlc = dlc_;
}

uint32_t & CanFrame::getId (){
	return id;
}
uint32_t & CanFrame::getMbIndex (){
	return mbIndex;
}
CanFrame::frameType & CanFrame::getIde (){
	return ide;
}
CanFrame::remoteFrame & CanFrame::getRtr (){
	return rtr;
}
uint8_t & CanFrame::getDlc (){
	return dlc;
}

uint16_t CanFrame::getCode(uint32_t cs){
	return (((cs) & CAN_CS_CODE_MASK)>>24);
}
uint8_t CanFrame::getLength(uint32_t cs){
	return (((cs) & CAN_CS_DLC_MASK)>>16);
}

void CanFrame::setIde (frameType type){
	ide = type;
}

void CanFrame::setId (uint32_t id_){
	id = id_;
}




