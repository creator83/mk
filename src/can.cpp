#include "can.h"

CAN_Type * Can::canBase[2]={CAN0, CAN1};

Can::voidFunc Can::tactPtr  [2] = {&Can::tactCan0, &Can::tactCan1};

Can::Can (nCan n, Pin *tx_, Pin *rx_){
	canNumber = static_cast <uint8_t>(n);
	
	tx = tx_;
	rx = rx_;
	//tact can module
	(this->*(Can::tactPtr[canNumber]))();

	//canBase[canNumber]->MB
}

void Can::tactCan0(){
	SIM->SCGC6 |= SIM_SCGC6_FLEXCAN0_MASK;
}

void Can::tactCan1(){
	SIM->SCGC3 |= SIM_SCGC3_FLEXCAN1_MASK;
}

void Can::init(filter f){
	canBase[canNumber]->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;
	canBase[canNumber]->MCR |= CAN_MCR_FRZ_MASK;
	canBase[canNumber]->MCR |= CAN_MCR_MDIS_MASK;
	while((CAN_MCR_LPMACK_MASK & (canBase[canNumber]->MCR)));
	canBase[canNumber]->MCR |= CAN_MCR_SOFTRST_MASK;
	while(CAN_MCR_SOFTRST_MASK & (canBase[canNumber]->MCR));
	while(!(CAN_MCR_FRZACK_MASK & (canBase[canNumber]->MCR)));
	for(uint8_t i=0;i<16;i++)
	{
		canBase[canNumber]->MB[i].CS = 0x00000000;
		canBase[canNumber]->MB[i].ID = 0x00000000;
		canBase[canNumber]->MB[i].WORD0 = 0x00000000;
		canBase[canNumber]->MB[i].WORD1 = 0x00000000;
	}
	canBase[canNumber]->CTRL2 = (0|CAN_CTRL2_TASD(22)); 
	canBase[canNumber]->MCR |= CAN_MCR_IDAM(0); 
	
	if(f == filter::enable)
	{
		for(uint8_t i = 0; i < 16 ; i++)
		{
			canBase[canNumber]->RXIMR[i] = 0x1FFFFFFF; //????16??????id??????J??? 
			canBase[canNumber]->RXMGMASK = 0x1FFFFFFF;
		} 	
	}
	else
	{
		for(uint8_t i = 0; i < 16 ; i++)
		{
			canBase[canNumber]->RXIMR[i] = 0; //????16??????id??????J??? 
			canBase[canNumber]->RXMGMASK = 0;
		} 		
	}
	canBase[canNumber]->MCR &= ~(CAN_MCR_HALT_MASK);

	while((CAN_MCR_FRZACK_MASK & (canBase[canNumber]->MCR)));  

	while(((canBase[canNumber]->MCR)&CAN_MCR_NOTRDY_MASK));   
	
	
}

void Can::receiveEnableMb (CanFrame *frame){
	canBase[canNumber]->MB[frame->getMbIndex()].CS = CAN_CS_CODE(0);
	canBase[canNumber]->MB[frame->getMbIndex()].ID = frame->getId();
	canBase[canNumber]->MB[frame->getMbIndex()].CS = (0|CAN_CS_CODE(4)|CAN_CS_IDE_MASK);	 // ??????? MB ???????????
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
	
	canBase[canNumber]->MB[frame->getMbIndex()].CS = CAN_CS_CODE(8); // ?????????
	canBase[canNumber]->MB[frame->getMbIndex()].ID = (1<<29)|frame->getId();  //extendet format  
	canBase[canNumber]->MB[frame->getMbIndex()].WORD0 = data_.word[0];
	canBase[canNumber]->MB[frame->getMbIndex()].WORD1 = data_.word[1];  
	for(uint8_t i = 0;i < 50;i++);
	canBase[canNumber]->MB[frame->getMbIndex()].CS = CAN_CS_CODE(12)|CAN_CS_IDE_MASK|CAN_CS_DLC(frame->getDlc())|CAN_CS_SRR_MASK;
	
	canBase[canNumber]->MB[frame->getMbIndex()].CS &= ~CAN_CS_RTR_MASK; //non remote frame
	uint32_t j=0; 
	while(!(canBase[canNumber]->IFLAG1 & (1<<frame->getMbIndex())))
	{
		if((j++)>0x1000)
		return false;
	}
	canBase[canNumber]->IFLAG1 = (1<<frame->getMbIndex());
	return true;
}


bool Can::receive (CanFrame *frame, uint8_t *data){
	uint16_t code;
	uint8_t length;
	union {
		uint32_t word[2];
		uint8_t byte [8];
	}data_;
	code = canBase[canNumber]->TIMER; 
	
	if((canBase[canNumber]->IFLAG1 & (1<<(frame->getMbIndex()))) == 0)
	{
		return false;
	}
	code = frame->getCode(canBase[canNumber]->MB[frame->getMbIndex()].CS);
	if(code != 0x02)
	{
		//???????
		frame->setIde(CanFrame::frameType::normal);
		return false;
	}
	length = frame->getLength(canBase[canNumber]->MB[frame->getMbIndex()].CS);
	if(length < 1)
	{
		frame->setIde(CanFrame::frameType::standart);
		return false;
	}
	frame->setIde(CanFrame::frameType::extended);
	code = canBase[canNumber]->TIMER;    // ?????? MB ????
	canBase[canNumber]->IFLAG1 = (1<<(frame->getMbIndex()));//???????
	data_.word[0] = canBase[canNumber]->MB[frame->getMbIndex()].WORD0;   //????????????
	data_.word[1] = canBase[canNumber]->MB[frame->getMbIndex()].WORD1;
	
	if(canBase[canNumber]->MB[frame->getMbIndex()].CS & CAN_CS_IDE_MASK)
	{
		
		frame->setIde(CanFrame::frameType::extended);
		frame->setId (canBase[canNumber]->MB[frame->getMbIndex()].ID);
	}
	else
	{
		frame->setIde(CanFrame::frameType::standart);
		frame->setId (canBase[canNumber]->MB[frame->getMbIndex()].ID>>18);
	}
		
	for(uint8_t i=0;i<length;i++)
	{
		*data++ = data_.byte[i];
	}
	return true;
}

bool Can::start (nCan n){
	canNumber = static_cast <uint8_t>(n);
	//Starting FLEXCAN in normal mode
	canBase[canNumber]->MCR &=~ CAN_MCR_HALT_MASK;
	//wait for synchronization	
   while(canBase[canNumber]->MCR & CAN_MCR_FRZACK_MASK);
   while((canBase[canNumber]->MCR & CAN_MCR_NOTRDY_MASK));
   return true;
}

bool Can::softReset (nCan n){
	canNumber = static_cast <uint8_t>(n);
	// check for low power mode
  if(canBase[canNumber]->MCR & CAN_MCR_LPMACK_MASK )
  {
     // Enable clock
     canBase[canNumber]->MCR &= (~CAN_MCR_MDIS_MASK);
		 // wait until enabled
	 while (canBase[canNumber]->MCR & CAN_MCR_LPMACK_MASK);
  }

  // Reset the FLEXCAN
  canBase[canNumber]->MCR = CAN_MCR_SOFTRST_MASK;

  // Wait for reset cycle to complete
  while (canBase[canNumber]->MCR & CAN_MCR_SOFTRST_MASK);

  // Set Freeze, Halt
  canBase[canNumber]->MCR |= CAN_MCR_HALT_MASK;
	return true;	
}
