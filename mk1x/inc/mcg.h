#include "device.h"             // Device header

#ifndef MCG_H
#define MCG_H


class Mcg
{
  //variables
public:
	enum class mcgOutSource {fll, internal, external};
	enum class fllDivider {div1_32, div2_64, div4_128, div8_256, div16_512, 
	div32_1024, div64_1280, div128_1536};
    enum class mcgOutClkDivider:uint8_t {div1, div2, div3, div4, div5, div6, div7, div8, div9, div10,
                                div11, div12, div13, div14, div15, div16};
    enum class dcoRange:bool {notFine,fine};
    enum class multiplicationRangeFll:uint8_t {lowRange, midRange, midHighRange,highRange};
	enum class fllSource {external, internal};
	enum class freqRange {lowFreq, highFreq, vHighFreq};
    enum class gainOsc {lowPower, highGain};
	enum class extSource {external, oscillator};
	enum class intSource {slow, fast};
    enum class oscilator {extal, khz32, irc48};
 private:

	
    uint8_t nFllDivider;
	uint8_t mcgOutValue;
    uint8_t fllSourceValue;
	uint16_t fllInputClock;
    uint8_t coreDivider, busFlashDivider;
    uint32_t mcgOutClk;
    uint32_t mcgIRClock;
    uint32_t fllClock;
    uint32_t extClock;
    uint8_t fllDividerValue;
    static uint32_t khz32;
    
	uint8_t nFreqRange;
	uint8_t nExtSource;
	uint8_t nIntSource;
  //functions
public:
    Mcg ();
	void mcgIrclkEnable ();
	void mcgIrclkDisable ();
	void setMcgOutClk (mcgOutSource);
	void setFllDivider (fllDivider);
	void setFllSource (fllSource);
	void setFreqRange (freqRange);
	void setExtSource (extSource);
	void setIntSource (intSource);
    void setCoreDivider (mcgOutClkDivider);
    void setBusDivider (mcgOutClkDivider);
    void setFlashDivider (mcgOutClkDivider);
    void setFllMultiplication(dcoRange, multiplicationRangeFll);
    void setOscilator (oscilator);
    void setGain (gainOsc);
	uint8_t getFllSource ();
	uint8_t getMcgOutSource ();
	uint8_t getIntSource ();
    uint32_t & getMcgOutClk ();
    uint32_t & getInternalSlowClock ();
    uint32_t & getFllClk ();
    uint8_t & getCoreDivider();
    uint8_t & getBusDivider();
    uint8_t & getFlashDivider();
    uint8_t & getFllSourceValue();
private:


};

 
#endif

