#include "MKL17Z4.h"
#include "gpio.h"
#include "delay.h"
#include "tact.h"
#include "segled.h"
#include "max6675.h"
#include "pid.h"

tact frq;
segled indicator(segled::B);

/*! \brief P, I and D parameter values
 *
 * The K_P, K_I and K_D values (P, I and D gains)
 * need to be modified to adapt to the application at hand
 */
//! \xrefitem todo "Todo" "Todo list"
#define K_P     2.00
//! \xrefitem todo "Todo" "Todo list"
#define K_I     4.00
//! \xrefitem todo "Todo" "Todo list"
#define K_D     0.80

uint8_t stat;

/*! \brief Flags for status information
 */
struct GLOBAL_FLAGS {
  //! True when PID control loop should run one time
  uint8_t pidTimer:1;
  uint8_t dummy:7;
} gFlags = {0, 0};

//! Parameters for regulator
struct PID_DATA pidData;

struct task
{
	unsigned counter :12;
	unsigned flag :1;
}pid;

struct flags
{
	unsigned led_indicator_delay : 1;
}flag;



extern "C"
{
	void SysTick_Handler ();
}


void SysTick_Handler ()
{
		

	if (flag.led_indicator_delay)flag.led_indicator_delay = 0;
	else
	{
		indicator.digit();
		flag.led_indicator_delay = 1;
	}
		if (pid.counter>1000)
	{
		pid.flag = 1;
		pid.counter = 0;
	}
	else pid.counter++;
	
/*	
	if (flag.led_indicator_delay)flag.led_indicator_delay = 0;
	else
	{
		indicator.digit();
		flag.led_indicator_delay = 1;
	}*/

}

int main()
{
	int16_t referenceValue, measurementValue, inputValue;
	referenceValue = 60;
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);
	max6675 sensor;
	
  while (1)
  {
	/*indicator.get_buffer (sensor.readCelsius());
		delay_ms (500);*/
	  if (pid.flag)
	  {
		  measurementValue = sensor.readCelsius();
		  inputValue = pid_Controller(referenceValue, measurementValue, &pidData);
		  indicator.get_buffer (sensor.readCelsius());
		  //TIM3->CCR3 = inputValue << 10;
		  indicator.get_buffer (inputValue);
		  pid.flag = 0;
	  }

  }
}
