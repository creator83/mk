#include "MKL26Z4.h"                    // Device header
#include "gpio.h"
#include "delay.h"
#include "pit.h"
#include "tact.h"
#include "segled.h"
#include "max6675.h"
#include "adc.h"

tact frq;
const char led = 0;
gpio E(gpio::E);
pit del(pit::ch0, 1, pit::ms);
segled indicator(segled::B);
const char adc_ch = 1;
uint16_t calibr;


bool init_adc ();
uint16_t conv_adc(uint8_t pin);

struct flags
{
	unsigned led_indicator_delay : 1;
}flag;


extern "C"
{
	void PIT_IRQHandler();
}

void PIT_IRQHandler()
{
	del.clear_flag();
	if (flag.led_indicator_delay)flag.led_indicator_delay = 0;
	else
	{
		indicator.digit();
		flag.led_indicator_delay = 1;
	}

}

uint16_t result [4];

int main()
{
	init_adc ();

	//adc t_couple (adc::SE0, adc::bit16);

	while (1)
	{
		//indicator.get_buffer (t_couple.convert());
		indicator.get_buffer (conv_adc(adc_ch));
		delay_ms(200);
	}
}

bool init_adc ()
{
	//tact ADC0
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
	//Set busclock
	ADC0->CFG1 &= ~ADC_CFG1_ADICLK_MASK;
	//Set divider - 8
	ADC0->CFG1 |= ADC_CFG1_ADIV(3);
	ADC0->SC3 |= ADC_SC3_AVGE_MASK|ADC_SC3_AVGS(3);
	ADC0->SC2 &= ~ ADC_SC2_ADTRG_MASK;
	ADC0->SC3 |= ADC_SC3_CAL_MASK ;
	while (!(ADC0->SC1[0]&ADC_SC1_COCO_MASK));
	if (ADC0->SC3&ADC_SC3_CALF_MASK) return false;
	calibr = ADC0->CLP0+ADC0->CLP1+ADC0->CLP2+ADC0->CLP3+ADC0->CLP4+ADC0->CLPS;
	calibr >>=1;
	ADC0->CFG1|= ADC_CFG1_ADLSMP_MASK|ADC_CFG1_MODE(2);
	return true;

}

uint16_t conv_adc(uint8_t pin)
{
	//Select 4 channal and start conversation
	ADC0->SC1[0] = ADC_SC1_ADCH(pin);
	while (!(ADC0->SC1[0]&ADC_SC1_COCO_MASK));
	return ADC0->R[0];
}
