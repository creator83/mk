#include "tpm.h"

TPM_Type * Tpm::tpmPtr[2]={TPM0, TPM1};

Tpm::setF Tpm::fMode [6] = {};


Tpm::Tpm (nTpm n_, channel ch, division d)
{
	numTpm = static_cast <uint8_t>(n_);
	nCh = static_cast <uint8_t>(ch);
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(3);
	SIM->SCGC6 |= (SIM_SCGC6_TPM0_MASK << numTpm);
	tpmPtr[numTpm]->SC &= ~TPM_SC_PS_MASK;
	tpmPtr[numTpm]->SC |= TPM_SC_PS(d);
}

void Tpm::setModulo (uint16_t val)
{
	tpmPtr[numTpm]->MOD = val;
}

void Tpm::setCnt (uint16_t val)
{
	tpmPtr[numTpm]->CONTROLS[nCh].CnV = val;
}

TPM_Type * Tpm::getPtrTimer()
{
	return tpmPtr[numTpm];
}

void Tpm::start ()
{
	tpmPtr[numTpm]->SC |= TPM_SC_CMOD(1);
}

void Tpm::stop ()
{
	tpmPtr[numTpm]->SC &= ~TPM_SC_CMOD_MASK;
}

void Tpm::clearFlag ()
{
	tpmPtr[numTpm]->SC |= TPM_SC_TOF_MASK;
}

void Tpm::setMode (mode m, togPulseMode n)
{
	(this->*(Tpm::fMode[static_cast <uint8_t>(m)]))(n);
}

void Tpm::initOutputTogle(togPulseMode t_mode)
{
	tpmPtr[numTpm]->SC &= ~TPM_SC_CPWMS_MASK;
	tpmPtr[numTpm]->CONTROLS[nCh].CnSC &= ~(TPM_CnSC_ELSA_MASK|TPM_CnSC_ELSB_MASK|TPM_CnSC_MSA_MASK|TPM_CnSC_MSB_MASK);
	tpmPtr[numTpm]->CONTROLS[nCh].CnSC |= TPM_CnSC_MSA_MASK|(static_cast <uint8_t>(t_mode) << TPM_CnSC_ELSA_SHIFT);
}

void Tpm::initOutputPulse(togPulseMode p_mode)
{
	tpmPtr[numTpm]->SC &= ~TPM_SC_CPWMS_MASK;
	tpmPtr[numTpm]->CONTROLS[nCh].CnSC &= ~(TPM_CnSC_ELSA_MASK|TPM_CnSC_ELSB_MASK|TPM_CnSC_MSA_MASK|TPM_CnSC_MSB_MASK);
	tpmPtr[numTpm]->CONTROLS[nCh].CnSC |= TPM_CnSC_MSA_MASK|TPM_CnSC_MSB_MASK|(static_cast <uint8_t>(p_mode) << TPM_CnSC_ELSA_SHIFT );
}

void Tpm::initEdgePwm(togPulseMode e_mode)
{
	tpmPtr[numTpm]->SC &= ~TPM_SC_CPWMS_MASK;
	tpmPtr[numTpm]->CONTROLS[nCh].CnSC &= ~(TPM_CnSC_ELSA_MASK|TPM_CnSC_ELSB_MASK|TPM_CnSC_MSA_MASK|TPM_CnSC_MSB_MASK);
	tpmPtr[numTpm]->CONTROLS[nCh].CnSC |= TPM_CnSC_MSB_MASK|(static_cast <uint8_t>(e_mode) << TPM_CnSC_ELSA_SHIFT) ;

}

void Tpm::initCenterPwm(togPulseMode e_mode)
{
	tpmPtr[numTpm]->SC |= TPM_SC_CPWMS_MASK;
	tpmPtr[numTpm]->CONTROLS[nCh].CnSC &= ~(TPM_CnSC_ELSA_MASK|TPM_CnSC_ELSB_MASK|TPM_CnSC_MSA_MASK|TPM_CnSC_MSB_MASK);
	tpmPtr[numTpm]->CONTROLS[nCh].CnSC |= TPM_CnSC_MSB_MASK|(static_cast <uint8_t>(e_mode) << TPM_CnSC_ELSA_SHIFT) ;

}

void Tpm::setKhz (uint16_t val)
{

}

void Tpm::setHz (uint16_t val)
{

}

void Tpm::setMs (uint16_t val)
{

}

void Tpm::setUs (uint16_t val)
{

}
