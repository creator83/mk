#include "device.h"    
#include "pin.h" 


#ifndef CAN_H
#define CAN_H



//Baudrate = Clock source / (PRESDIV * (PROPSEG + PSEG1 + PSEG2 +1))

class Can
{
public:
	enum class MbRxCode {inactive, empty=0x04, full=0x02, overrun=6, ranswer=0x0A};
	enum class MbTxCode {inactive=0x08, abort=0x09, data=0x0C, remote=0x0C, tanswer=0x0E};
	enum class frameType {standart, extended};
	enum class remoteFrame {disable, enable};
	enum class direction{receive, transmite};
    enum class mode {normal, listen, timesync, loopback, boffrec, freeze, disable, stop};
	union mailBuffer{
		uint32_t word[4];
		struct mb{
			uint16_t timeStamp;
			unsigned dlc: 4;
			unsigned rtr: 1;
			unsigned ide: 1;
			unsigned srr: 1;
			unsigned empty1: 1;
			unsigned code: 2;
			unsigned empty2: 4;
			struct idStruct{
				unsigned idExt: 29;
				struct {
					unsigned empty: 18;
					unsigned id: 11;
				}standart;
			}id;
			
			unsigned prio: 3;
			uint8_t data [8];
		}buffer;
	}*bufferPtr[16];
	
private:
    mode currentMode;            
	uint8_t getMbCode ();
public:
	enum class filter {enable, disable};
	
    enum class clockSource {osc, periph};
    enum class resyncJumpWidth {val1, val2, val3, val4};
    enum class phaseSegment {phase1, phase2, phase3, phase4, phase5, phase6, phase7, phase8};
    enum class propagationSegment {seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8};
    
private:
	using voidFunc = void (Can::*)();
	Pin *tx, *rx;
	uint8_t canNumber;
    static voidFunc tactPtr  [2];
//    static voidFunc canModes [7];
    
	CAN_Type * canPtr;
    using ucharPtr = uint8_t *;
    
    const uint8_t offsetMb = 0x80;
public:
  
    Can (numberCan, Pin &tx_, Pin &rx_);
	void setDisableMode ();
	void setNormalMode ();
	void setListenMode ();
	void setfreezeMode ();
	mailBuffer * getMailBufferPtr (canMb);
	/*void receiveEnableMb (CanFrame *);
	bool transmite (CanFrame *, uint8_t *);
	bool receive (CanFrame *, uint8_t *); */
    void setClk(clockSource s);
    void setPresDiv (uint8_t);
    void setResyncJumpWidth (resyncJumpWidth);
    void setPhaseSegment1(phaseSegment);
    void setPhaseSegment2(phaseSegment);
    void setPropSeg(propagationSegment);
    bool start ();
    void setMode (mode);
    bool softReset ();
    void interruptEnable(canMb);
    void setLoopbackMode ();

	//functions
	void setCode(MbRxCode);
	void setCode(MbTxCode);
	void setId(uint32_t );
	void setData(uint8_t);
	void setDLC (uint8_t);
	void setFrameType (frameType);
	void transmit();
	void transmit(uint8_t *data, uint8_t n);
	//void setData(uint8_t);
	void initTxMailBox ();
	void initRxMailBox ();
private:
    void tactCan0();
	void tactCan1();
 /*   
    uint32_t * getMbPtr (canMb number);
	
	void init(filter f);
	
    void initMb();
	
	void setTimesyncMode ();
	
	void setBoffrecMode ();
    bool getIflag(canMb);
    void clearIflag(canMb);*/
    
};

 
#endif

