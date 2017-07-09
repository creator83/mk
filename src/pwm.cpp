#include "pwm.h"

Pwm::PtrPwm Pwm::funcMode [3] = {&Pwm::setEdgePwm};


Pwm::Pwm (Tpm &t, Pin &p, Tpm::channel ch, mode mode_, pulseMode m_)
{
	pwmChannel = static_cast <uint8_t> (ch);
	timer = &t;
	nCh = static_cast<uint8_t>(ch);
	(this->*(Pwm::funcMode[static_cast<uint8_t>(mode_)]))(m_);
	timer->start();
}

void Pwm::setEdgePwm (pulseMode m)
{
	timer->getPtrTimer()->SC &= ~TPM_SC_CPWMS_MASK;
	timer->getPtrTimer()->CONTROLS[nCh].CnSC &= ~(TPM_CnSC_ELSA_MASK|TPM_CnSC_ELSB_MASK|TPM_CnSC_MSA_MASK|TPM_CnSC_MSB_MASK);
	timer->getPtrTimer()->CONTROLS[nCh].CnSC |= TPM_CnSC_MSB_MASK|(static_cast <uint8_t>(m) << TPM_CnSC_ELSA_SHIFT) ;
}

void Pwm::setCenterPwm ()
{

}

void Pwm::setCombinePwm ()
{

}

void Pwm::setValue (uint16_t val)
{
	timer->getPtrTimer()->CONTROLS[nCh].CnV = val;
}
