#include "device.h"                 // Device header
#include "pin.h"


#ifndef ADC_H_
#define ADC_H_

class Adc
{
public:
	enum class channel {SE0, SE1, SE2, SE3, SE4a, SE5a, SE6a, SE7a, SE8, SE9, SE11=11, SE12, SE13, SE14, SE15, SE23=23};
	enum class resolution {bit8, bit12, bit10, bit16};
	enum class mode {softwareTrg, hardwareTrg};
	enum class samples {smpls4, smpls8, smpls16, smpls32};
	enum class hwTriger {external, cmp0=1, pit0= 4, pit1 = 5, tpm0=8, tpm1=9, tpm2=10, rtcAlarm=12, rtcSecond=13, lptmr0=14};

private:
	Pin * pinDriver;
	uint8_t n_channel;
	uint8_t res, nMode;
	bool intrpt;
	static uint32_t resReg [2];
	static uint32_t setReg [2];
public:
	Adc(channel ch_, resolution r_, Pin &);
	Adc(mode m,channel ch_, resolution r_, Pin &d);
	void interruptEnable ();
	void interruptDisable ();
	void setHwAVG (samples);
	void setHwTrg (hwTriger);
	void setADC ();
	uint16_t getResult ();
	bool calibrate ();
	uint16_t convert ();
};

#endif
