#include "adc.h"


uint32_t Adc::resReg [2] = {ADC0->R[0], ADC0->R[1]};
uint32_t Adc::setReg [2] = {ADC0->SC1[0], ADC0->SC1[1]};

Adc::Adc(channel ch_, resolution r_, Pin &d)
:nMode(0)
{
	pinDriver = &d;
	n_channel = static_cast <uint8_t>(ch_);
	//tact ADC0
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

	//Set busclock
	ADC0->CFG1 &= ~ADC_CFG1_ADICLK_MASK;
	//Set divider - 8
	ADC0->CFG1 |= ADC_CFG1_ADIV(3);
	ADC0->SC3 |= ADC_SC3_AVGE_MASK|ADC_SC3_AVGS(3);
	calibrate ();

	ADC0->CFG1|= ADC_CFG1_ADLSMP_MASK|ADC_CFG1_MODE(r_);
}

Adc::Adc(mode m,channel ch_, resolution r_, Pin &d)
:nMode(1)
{
	pinDriver = &d;
	n_channel = static_cast <uint8_t>(ch_);
	//tact ADC0
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
	//Set busclock
	ADC0->CFG1 &= ~ADC_CFG1_ADICLK_MASK;
	//Set divider - 8
	ADC0->CFG1 |= ADC_CFG1_ADIV(3);
	calibrate ();

	ADC0->SC2 |= ADC_SC2_ADTRG_MASK;
}

void Adc::interruptEnable ()
{
	intrpt = true;
	NVIC_EnableIRQ(ADC0_IRQn);
}

void Adc::interruptDisable ()
{
	intrpt = false;
	NVIC_DisableIRQ(ADC0_IRQn);
}

void Adc::setHwAVG (samples s)
{
	ADC0->SC3 |= ADC_SC3_AVGE_MASK|ADC_SC3_AVGS(static_cast<uint8_t>(s));

}

void Adc::setHwTrg (hwTriger t)
{
	SIM->SOPT7 &= ~SIM_SOPT7_ADC0TRGSEL_MASK;
	SIM->SOPT7 |= SIM_SOPT7_ADC0TRGSEL (static_cast<uint8_t>(t));
}

void Adc::setADC ()
{
	setReg[nMode] = ADC_SC1_ADCH(n_channel)|intrpt << ADC_SC1_AIEN_SHIFT;

}

uint16_t Adc::getResult ()
{
	return resReg[nMode];
}

bool Adc::calibrate ()
{
	unsigned short cal_var;


    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;  // enable ADC0 clock

	ADC0->SC2 &=  ~ADC_SC2_ADTRG_MASK ; // Enable Software Conversion Trigger for Calibration Process
	ADC0->SC3 &= ( ~ADC_SC3_ADCO_MASK & ~ADC_SC3_AVGS_MASK ); // set single conversion, clear avgs bitfield for next writing

	//For best calibration results
	ADC0->SC3 |= ( ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3) );  // Turn averaging ON and set at max value ( 32 )

	ADC0->SC3 |= ADC_SC3_CAL_MASK ;      // Start CAL

	//ADC1_SC1A
	while ( (ADC0->SC1[0] & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK ); // Wait calibration end

	 //COCO=1 calibration complete
	if ((ADC0->SC3& ADC_SC3_CALF_MASK) == ADC_SC3_CALF_MASK )
	{
	   return true;    // Check for Calibration fail error and return
	}
	  // Calculate plus-side calibration
	  cal_var = 0x00;

	  cal_var =  ADC0->CLP0;
	  cal_var += ADC0->CLP1;
	  cal_var += ADC0->CLP2;
	  cal_var += ADC0->CLP3;
	  cal_var += ADC0->CLP4;
	  cal_var += ADC0->CLPS;

	  cal_var = cal_var/2;
	  cal_var |= 0x8000; // Set MSB


	  ADC0->PG = ADC_PG_PG(cal_var);

	  // Calculate minus-side calibration
    cal_var = 0x00;
    cal_var =  ADC0->CLM0;
	cal_var += ADC0->CLM1;
	cal_var += ADC0->CLM2;
	cal_var += ADC0->CLM3;
	cal_var += ADC0->CLM4;
	cal_var += ADC0->CLMS;

    cal_var = cal_var/2;

    cal_var |= 0x8000; // Set MSB

    ADC0->MG = ADC_MG_MG(cal_var);

	ADC0->SC3 &= ~ADC_SC3_CAL_MASK ; /* Clear CAL bit */

	  return false;
}

uint16_t Adc::convert ()
{
	//Select 4 channel and start conversation
	setReg[nMode] = ADC_SC1_ADCH(n_channel);
	while (!(setReg[nMode]&ADC_SC1_COCO_MASK));
	return resReg[nMode];
}




