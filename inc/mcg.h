#include "device.h"             // Device header

#ifndef MCG_H
#define MCG_H


class Mcg
{
  //variables
public:
	enum class mcgOutClk {fll, internal, external};
	enum class fllDivider {div1_32, div2_64, div4_128, div8_256, div16_512, 
	div32_1024, div64_1280, div128_1536};
	enum class fllSource {external, internal};
	enum class freqRange {lowFreq, highFreq, vHighFreq};
	enum class extSource {external, oscillator};
	enum class intSource {slow, fast};
 private:

	
  uint8_t nFllDivider;
	uint8_t nMcgOutClk;
	uint8_t nFllSource;
	uint8_t nFreqRange;
	uint8_t nExtSource;
	uint8_t nIntSource;
  //functions
public:
  Mcg ();
	void mcgIrclkEnable ();
	void mcgIrclkDisable ();
	void setMcgOutClk (mcgOutClk);
	void setFllDivider (fllDivider);
	void setFllSource (fllSource);
	void setFreqRange (freqRange);
	void setExtSource (extSource);
	void setIntSource (intSource);
	uint8_t getFllSource ();
	uint8_t getMcgOutClk ();
	uint8_t getIntSource ();


private:


};

 
#endif

