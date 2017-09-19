#include "MKL17Z4.h"
#include "gpio.h"
#include "tact.h"
#include "delay.h"
#include "tpm.h"

tact frq;
const char pin_t = 20;





void tpm1_init ();

int main()
{

	//tpm1_init ();
	//frq.set_LIRC_div (tact::LIRC_8, tact::div_1,tact::div_1);
	//tact tpm1, set_ch=0, frq timer = 2Mhz
	tpm tpm1_pwm (tpm::TPM_1, tpm::ch0, tpm::div128);
	frq.set_LIRC_div (tact::LIRC_2, tact::div1, tact::div1);

	//Set pwm frq=20kHz
	tpm1_pwm.Set_MOD (62500);

	//Set duty
	tpm1_pwm.Set_CNV (62500);

	tpm1_pwm.init_output_togle (tpm::E, tpm::Alt3, pin_t, tpm::toggle);
	tpm1_pwm.start ();

	while (1)
  {
		/*
	  for (uint8_t i=0;i<=100;i++)
	  {
		  tpm1_pwm.Set_CNV (i);
		  delay_ms(50);
	  }
	  for (uint8_t i=100;i>0;i--)
	  {
		  tpm1_pwm.Set_CNV (i);
		  delay_ms(10);
	  }*/
  }
}



