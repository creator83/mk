#include "can.h"


Can::voidFunc Can::tactPtr  [2] = {&Can::tactCan0, &Can::tactCan1};
//Can::voidFunc Can::canModes [7] = {&Can::setNormalMode, &Can::setListenMode, &Can::setTimesyncMode, &Can::setLoopbackMode, &Can::setBoffrecMode, &Can::setfreezeMode, &Can::setDisableMode};

Can::Can (numberCan n, Pin &tx_, Pin &rx_)
:tx(&tx_), rx(&rx_){
    canNumber = static_cast<uint8_t>(n);
    canPtr = (CAN_Type *)canBaseAddress[canNumber];
    (this->*(Can::tactPtr[canNumber]))();
    //mbPtr += static_cast<uint32_t*>(canPtr) + offsetMb;
    setDisableMode ();  
    bufferPtr[0] = (mailBuffer *)(canBaseAddress + FLEXCAN_MSG_BUFADDR_OFFSET);
    //bufferPtr[0]->buffer.
}
void Can::setDisableMode (){
    canPtr->MCR |= CAN_MCR_MDIS_MASK;
    while (!(canPtr->MCR & CAN_MCR_LPMACK_MASK));
	currentMode = mode::disable;
}
void Can::setNormalMode (){
    /* 
    ** Normal mode, check freeze ACK bit
    ** call start if bit is set 
    */
    if((canPtr->MCR & CAN_MCR_FRZACK_MASK) == CAN_MCR_FRZACK_MASK){
        if(start() == false );
    }
}

void Can::setListenMode (){
    canPtr->CTRL1 |= CAN_CTRL1_LOM_MASK;
}
void Can::setfreezeMode (){
    canPtr->MCR |= (CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK);
     /* check for freeze Ack */
     if( (canPtr->MCR & CAN_MCR_FRZACK_MASK) != CAN_MCR_FRZACK_MASK);
        //ret_code = FLEXCAN_FREEZE_FAILED;
}

void Can::tactCan0(){
	SIM->SCGC6 |= SIM_SCGC6_FLEXCAN0_MASK;
}

void Can::tactCan1(){
	SIM->SCGC3 |= SIM_SCGC3_FLEXCAN1_MASK;
}
/*
mailBuffer * Can::getMailBufferPtr (canMb n){
	return &bufferPtr[static_cast<uint8_t>(n)];
}*/
/*
void Can::MsgBuffer::setData(uint8_t data){
     mbPtr = mbPtrConst;
    *(mbPtr+2) = data;
}

void Can::MsgBuffer::setDLC (uint8_t n){
    mbPtr = mbPtrConst;
    *mbPtr &= ~CAN_CS_DLC_MASK ;
    *mbPtr |= CAN_CS_DLC(n);
}

void Can::MsgBuffer::setFrameType (frameType frame){
    mbPtr = mbPtrConst;
    *mbPtr &= ~CAN_CS_IDE_MASK;
    *mbPtr |= static_cast<uint8_t>(frame) << CAN_CS_IDE_SHIFT;
}
*/
//void Can::MsgBuffer::transmit(){
//    //1. Check whether the respective interrupt bit is set and clear it
//    if (canModule.getIflag(mailBoxNumber)){
//        canModule.clearIflag(mailBoxNumber);
//    }
//    /*If the MB is active (transmission pending), write the ABORT code (0b1001) to the
//    *CODE field of the Control and Status word to request an abortion of the
//    *transmission.Wait for the corresponding IFLAG to be asserted by polling the IFLAG
//    *register or by the interrupt request if enabled by the respective IMASK.Then read
//    *back the CODE field to check if the transmission was aborted or transmitted (see
//    *Transmission abort mechanism).
//    */
//    if (getMbCode()==static_cast<uint8_t>(MbTxCode::data)){
//        setCode(MbTxCode::abort);
//        while (getMbCode()!=static_cast<uint8_t>(MbTxCode::abort));
//        if (canModule.getIflag(mailBoxNumber)){
//        canModule.clearIflag(mailBoxNumber);
//    }
//    }
//    /*If backwards compatibility is desired (MCR[AEN]
//    *bit is negated), just write the INACTIVE code (0b1000) to the CODE field to
//    *inactivate the MB but then the pending frame may be transmitted without notification
//    *(see Mailbox inactivation)
//    */
//    setCode(MbTxCode::inactive);
//    setId (1500);
//    setData (0xFC);
//    setDLC (1);
//    setFrameType (Can::MsgBuffer::frameType::extended);
//    *mbPtr |= CAN_CS_SRR_MASK;
//    setCode (MbTxCode::data);
//}

//void Can::MsgBuffer::transmit(uint8_t *data, uint8_t n){
//    uint8_t *ptr = (uint8_t *)(mbPtr+2);
//    while(n--){
//        *ptr++=*data++;
//    }
//    CAN0->MB[0].CS &= ~CAN_CS_DLC_MASK;
//    CAN0->MB[0].CS |= n<< CAN_CS_DLC_SHIFT;
//    //setDLC(n);
//    setCode (MbTxCode::data);
//}
/*
void Can::MsgBuffer::initTxMailBox (){
    //1. Check whether the respective interrupt bit is set and clear it
    if (canModule.getIflag(mailBoxNumber)){
        canModule.clearIflag(mailBoxNumber);
    }
    /*If the MB is active (transmission pending), write the ABORT code (0b1001) to the
    *CODE field of the Control and Status word to request an abortion of the
    *transmission.Wait for the corresponding IFLAG to be asserted by polling the IFLAG
    *register or by the interrupt request if enabled by the respective IMASK.Then read
    *back the CODE field to check if the transmission was aborted or transmitted (see
    *Transmission abort mechanism).
    */
//    if (getMbCode()==static_cast<uint8_t>(MbTxCode::data)){
//        setCode(MbTxCode::abort);
//        while (getMbCode()!=static_cast<uint8_t>(MbTxCode::abort));
//        if (canModule.getIflag(mailBoxNumber)){
//        canModule.clearIflag(mailBoxNumber);
//    }
//    }
    /*If backwards compatibility is desired (MCR[AEN]
    *bit is negated), just write the INACTIVE code (0b1000) to the CODE field to
    *inactivate the MB but then the pending frame may be transmitted without notification
    *(see Mailbox inactivation)
    */
//    setCode(MbTxCode::inactive);
//    setFrameType (Can::MsgBuffer::frameType::extended);
//    setId (0xABC);
//}
/*
void Can::MsgBuffer::initRxMailBox (){
    mbPtr = mbPtrConst;
    uint16_t timeStamp = *mbPtr&CAN_CS_TIME_STAMP_MASK;
    
    // *mbPtr = CAN_CS_CODE(0);
    CAN1->MB[0].CS = CAN_CS_CODE(0);
    //setCode(MbRxCode::inactive);
    setFrameType (Can::MsgBuffer::frameType::extended);
    setId (0xABC);
    CAN1->MB[0].CS = CAN_CS_CODE(4);
    //setCode(MbRxCode::empty);
}
    
uint8_t Can::MsgBuffer::getMbCode (){
    uint32_t controlWord = *mbPtr;
    return (controlWord&CAN_CS_CODE_MASK >> CAN_CS_CODE_SHIFT);
}*/

bool Can::softReset (){
    // check for low power mode
    if(canPtr->MCR & CAN_MCR_LPMACK_MASK){
     // Enable clock
        canPtr->MCR &= ~CAN_MCR_MDIS_MASK;
     // wait until enabled
        while (canPtr->MCR & CAN_MCR_LPMACK_MASK);
    }
    // Reset the FLEXCAN
    canPtr->MCR = CAN_MCR_SOFTRST_MASK;

    // Wait for reset cycle to complete
    while (canPtr->MCR & CAN_MCR_SOFTRST_MASK);

    // Set Freeze, Halt
    canPtr->MCR |= CAN_MCR_HALT_MASK;
    
    // check for freeze Ack
    while(( (canPtr->MCR & CAN_MCR_FRZACK_MASK) != CAN_MCR_FRZACK_MASK ) ||
	 ( (canPtr->MCR & CAN_MCR_NOTRDY_MASK) != CAN_MCR_NOTRDY_MASK ));
    return true;
}

void Can::interruptEnable(canMb n){
    canPtr->IMASK1 |=CAN_IMASK1_BUFLM(n);
    NVIC_EnableIRQ (CAN0_Tx_Warning_IRQn);
    NVIC_EnableIRQ (CAN0_Error_IRQn);
}
/*
uint32_t * Can::getMbPtr (canMb number){
    uint32_t add = (uint32_t)canPtr;
    return (uint32_t *)(((uint32_t)canPtr) + FLEXCAN_MSG_BUFADDR_OFFSET + msgBuffSize*static_cast<uint8_t>(number));
}
*/

/*
void Can::init(filter f){
	canPtr->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;
	canPtr->MCR |= CAN_MCR_FRZ_MASK;
	canPtr->MCR |= CAN_MCR_MDIS_MASK;
	while((CAN_MCR_LPMACK_MASK & (canPtr->MCR)));
	canPtr->MCR |= CAN_MCR_SOFTRST_MASK;
	while(CAN_MCR_SOFTRST_MASK & (canPtr->MCR));
	while(!(CAN_MCR_FRZACK_MASK & (canPtr->MCR)));
	for(uint8_t i=0;i<16;i++)
	{
		canPtr->MB[i].CS = 0x00000000;
		canPtr->MB[i].ID = 0x00000000;
		canPtr->MB[i].WORD0 = 0x00000000;
        canPtr->MB[i].WORD1 = 0x00000000;
	}
	canPtr->CTRL2 = (0|CAN_CTRL2_TASD(22)); 
	canPtr->MCR |= CAN_MCR_IDAM(0); 
	
	if(f == filter::enable)
	{
		for(uint8_t i = 0; i < 16 ; i++)
		{
			canPtr->RXIMR[i] = 0x1FFFFFFF; //????16??????id??????J??? 
			canPtr->RXMGMASK = 0x1FFFFFFF;
		} 	
	}
	else
	{
		for(uint8_t i = 0; i < 16 ; i++)
		{
			canPtr->RXIMR[i] = 0; //????16??????id??????J??? 
			canPtr->RXMGMASK = 0;
		} 		
	}
	canPtr->MCR &= ~(CAN_MCR_HALT_MASK);

	while((CAN_MCR_FRZACK_MASK & (canPtr->MCR)));  

	while(((canPtr->MCR)&CAN_MCR_NOTRDY_MASK)); 
	
	
}*/
/*
void Can::receiveEnableMb (CanFrame *frame){
	canPtr->MB[frame->getMbIndex()].CS = CAN_CS_CODE(0);
	canPtr->MB[frame->getMbIndex()].ID = frame->getId();
	canPtr->MB[frame->getMbIndex()].CS = (0|CAN_CS_CODE(4)|CAN_CS_IDE_MASK);	 // ??????? MB ???????????
}

bool Can::transmite (CanFrame *frame, uint8_t * data){
	union {
		uint32_t word[2];
		uint8_t byte [8];
	}data_;
	for(uint8_t i=0, n = frame->getDlc();i<n;i++)
	{
		data_.byte[i] = *data++;
	}
	
	canPtr->MB[frame->getMbIndex()].CS = CAN_CS_CODE(8); // ?????????
	canPtr->MB[frame->getMbIndex()].ID = (1<<29)|frame->getId();  //extendet format  
	canPtr->MB[frame->getMbIndex()].WORD0 = data_.word[0];
	canPtr->MB[frame->getMbIndex()].WORD1 = data_.word[1];  
	for(uint8_t i = 0;i < 50;i++);
	canPtr->MB[frame->getMbIndex()].CS = CAN_CS_CODE(12)|CAN_CS_IDE_MASK|CAN_CS_DLC(frame->getDlc())|CAN_CS_SRR_MASK;
	
	canPtr->MB[frame->getMbIndex()].CS &= ~CAN_CS_RTR_MASK; //non remote frame
	uint32_t j=0; 
	while(!(canPtr->IFLAG1 & (1<<frame->getMbIndex())))
	{
		if((j++)>0x1000)
		return false;
	}
	canPtr->IFLAG1 = (1<<frame->getMbIndex());
	return true;
}


bool Can::receive (CanFrame *frame, uint8_t *data){
	uint16_t code;
	uint8_t length;
	union {
		uint32_t word[2];
		uint8_t byte [8];
	}data_;
	code = canPtr->TIMER; 
	
	if((canPtr->IFLAG1 & (1<<(frame->getMbIndex()))) == 0)
	{
		return false;
	}
	code = frame->getCode(canPtr->MB[frame->getMbIndex()].CS);
	if(code != 0x02)
	{
		//???????
		frame->setIde(CanFrame::frameType::standart);
		return false;
	}
	length = frame->getLength(canPtr->MB[frame->getMbIndex()].CS);
	if(length < 1)
	{
		frame->setIde(CanFrame::frameType::standart);
		return false;
	}
	frame->setIde(CanFrame::frameType::extended);
	code = canPtr->TIMER;    // ?????? MB ????
	canPtr->IFLAG1 = (1<<(frame->getMbIndex()));//???????
	data_.word[0] = canPtr->MB[frame->getMbIndex()].WORD0;   //????????????
	data_.word[1] = canPtr->MB[frame->getMbIndex()].WORD1;
	
	if(canPtr->MB[frame->getMbIndex()].CS & CAN_CS_IDE_MASK)
	{
		
		frame->setIde(CanFrame::frameType::extended);
		frame->setId (canPtr->MB[frame->getMbIndex()].ID);
	}
	else
	{
		frame->setIde(CanFrame::frameType::standart);
		frame->setId (canPtr->MB[frame->getMbIndex()].ID>>18);
	}
		
	for(uint8_t i=0;i<length;i++)
	{
		*data++ = data_.byte[i];
	}
	return true;
}*/
void Can::setClk(clockSource s){
    canPtr->CTRL1 &= ~ CAN_CTRL1_CLKSRC_MASK;
    canPtr->CTRL1 |= static_cast<uint8_t>(s) << CAN_CTRL1_CLKSRC_SHIFT;
}

void Can::setPresDiv (uint8_t val){
    canPtr->CTRL1 &= ~ CAN_CTRL1_PRESDIV_MASK;
    canPtr->CTRL1 |= CAN_CTRL1_PRESDIV(val-1);
}

void Can::setResyncJumpWidth (resyncJumpWidth val){
    canPtr->CTRL1 &= ~CAN_CTRL1_RJW_MASK;
    canPtr->CTRL1 |= CAN_CTRL1_RJW(static_cast<uint8_t>(val)-1);
}

void Can::setPhaseSegment1(phaseSegment pha){
    canPtr->CTRL1 &= ~CAN_CTRL1_PSEG1_MASK;
    canPtr->CTRL1 |= CAN_CTRL1_PSEG1(static_cast<uint8_t>(pha)-1);
}

void Can::setPhaseSegment2(phaseSegment pha){
    canPtr->CTRL1 &= ~CAN_CTRL1_PSEG2_MASK;
    canPtr->CTRL1 |= CAN_CTRL1_PSEG2(static_cast<uint8_t>(pha)-1);
}

void Can::setPropSeg(propagationSegment seg){
    canPtr->CTRL1 &= ~CAN_CTRL1_PROPSEG_MASK;
    canPtr->CTRL1 |= CAN_CTRL1_PROPSEG(static_cast<uint8_t>(seg)-1);
}

bool Can::start (){
    //Starting FLEXCAN in normal mode
    canPtr->MCR &= ~CAN_MCR_HALT_MASK;
   
    //wait for synchronization
   while(canPtr->MCR & CAN_MCR_FRZACK_MASK); 
   while(canPtr->MCR & CAN_MCR_NOTRDY_MASK);
   return true;
}

//void Can::setMode (mode m){
//    (this->*(Can::canModes[static_cast<uint8_t>(m)]))();
//}
/*
void Can::setTimesyncMode (){
    canPtr->CTRL1 |= CAN_CTRL1_TSYN_MASK;
}
void Can::setLoopbackMode (){
    canPtr->CTRL1 |= CAN_CTRL1_LPB_MASK;
}
void Can::setBoffrecMode (){
    canPtr->CTRL1 &= ~CAN_CTRL1_BOFFMSK_MASK;
}



bool Can::getIflag(canMb mb){
    return canPtr->IFLAG1&CAN_IFLAG1_BUF31TO8I(mb);
}
void Can::clearIflag(canMb mb){
    canPtr->IFLAG1|= CAN_IFLAG1_BUF31TO8I(mb);
}
*/
