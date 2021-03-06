#include "device.h"    
#include "mcg.h"

#ifndef TACT_H
#define TACT_H

class Tact;
class TactDestroyer
{
  private:
    Tact* p_instance;
  public:    
    ~TactDestroyer();
    void initialize( Tact* p );
};
class Tact
{
  //variables
public:
	enum class pllFllSource {mcgFllClk, irc48=3};
	enum class lptmrSource {osc32, lpo=3};
private:
	uint8_t nPllFllSource;
	uint8_t nLptmrSource;

	static uint32_t cpuClock, busClock, mcgIRClock, mcgFFClock, mcgFLLClock;
	Mcg mcg;
	static Tact * p_instance;
    static TactDestroyer destroyer;
public:
  
	static Tact& getInstance();
	uint32_t & getFrqBus ();
	uint32_t & getFrqCpu ();
	uint32_t & getFrqFlash ();
protected:
	Tact ();
    Tact(Tact const&) = delete;
    Tact& operator=(Tact const&) = delete;
    friend class TactDestroyer;
	void initFei ();
	void initFee ();
	void initFbi ();
	void initFbe ();
	void initPee ();
	void setPllFllSource (pllFllSource);
	void setLptmrSource (lptmrSource);
};

 
#endif

