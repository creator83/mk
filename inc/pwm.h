#include "device.h"
#include "pin.h"
#include "tpm.h"

#ifndef PWM_H
#define PWM_H

class Pwm
{
  //variables
public:

	enum class mode {EdgePwm, CenterPwm, CombinePwm};
	enum class pulseMode {highPulse=2, lowPulse};


private:
	using PtrPwm = void(Pwm::*)(Pwm::pulseMode) ;
	static PtrPwm funcMode [3];
	Tpm * timer;
	uint8_t pwmChannel;
	uint8_t nCh;

public:
	Pwm (Tpm &, Pin &p, Tpm::channel ch, mode, pulseMode);
	void setValue (uint16_t);

private:
	void setMode ();
	void setEdgePwm (pulseMode m);
	void setCenterPwm ();
	void setCombinePwm ();


};



#endif
