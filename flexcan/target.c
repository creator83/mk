#include "common.h"
#include "p2io_map.h"
#include "target.h"
#include "test_config.h"

#include <stdio.h>

void InitTarget(void)
{
    int i;
       
#if 0    
        printf("MCGC2 = %#08.8x\n",MCG_C2);
        printf("MCGC5 = %#08.8x\n",MCG_C5);
        printf("MCGC6 = %#08.8x\n",MCG_C6);

        printf("SIM_SCGC3 = %#08.8x\n",SIM_SCGC3);   
        printf("SIM_SCGC6 = %#08.8x\n",SIM_SCGC6);           
#endif
        SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;       // enable clock gating to LPTIMER
        SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK | SIM_SCGC6_PIT_MASK;  // enable FTM1 and PIT
        
        MCG_C5 |= MCGC5_PLLCLKEN_MASK;             // enable MCGPLLCLK clock and will enable external oscillator
#if defined(USE_EXTERNAL_CLOCK)
        // NOTE: 
        // ERCLKEN bit MUST be set in OSC_CR, otherwise strange behavior may occur:
        // CAN0 may not work or CAN1 may not work.
        OSC_CR |= OSC_CR_ERCLKEN_MASK | OSC_CR_EREFSTEN_MASK; 
#ifndef K60_CLK
        {
          MCG_C2 |= (1<<2);
          while(!(MCGS & (1<<1)));
        }
#endif   
#endif        
         printf("OSC_CR = %#08.8x\n",OSC_CR);
	//printf("MCGC4 = %#08.8x\n",MCGC4);
	//printf("CLKDIV1=%#8x\n",CLKDIV1);
	//printf("PMC_LVDSC1=%#8x,PMC_LVDSC2=%#8x,PMC_REGSC=%#8x\n",PMC_LVDSC1,PMC_LVDSC2,PMC_REGSC);

#if     (defined(TEST_CRC_FORM_STUF_ERROR) || defined(TEST_TKT029460))
	// Configure CAN_TX pin mulxed with 12 as FTM1_CH0 pins
	GPIOA_PCR(12) = (PIN_ALT(3)<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;
	GPIOA_PCR(13) = (PIN_ALT(3)<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;
	//GPIOA_PCR(13) = (PIN_ALT(2)<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;
      
        // PTB0
        PORTB_PCR0 = PORT_PCR_MUX(3) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
#else	

#ifdef  CAN0_USE_PTA12_13        
	// Configure CAN_RX/TX pins mulxed with PTA13/12 for FlexCAN0
	GPIOA_PCR(12) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;
	GPIOA_PCR(13) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;
	//GPIOA_PCR(12) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_ODE_MASK | !PIN_PULL_SELECT_MASK | !PIN_PULL_ENABLE_MASK; // open-drain, OK
	//GPIOA_PCR(13) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_ODE_MASK | !PIN_PULL_SELECT_MASK | !PIN_PULL_ENABLE_MASK;  
#endif
#ifdef  CAN0_USE_PTB18_19
	// Configure CAN_RX/TX pins mulxed with PTB18/19 for FlexCAN0
	GPIOB_PCR(18) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;
	GPIOB_PCR(19) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;
	//GPIOB_PCR(18) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_ODE_MASK | !PIN_PULL_SELECT_MASK | !PIN_PULL_ENABLE_MASK; // open-drain, OK
	//GPIOB_PCR(19) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_ODE_MASK | !PIN_PULL_SELECT_MASK | !PIN_PULL_ENABLE_MASK;  
#endif
        
#endif

#ifdef  MARCONI
	// Configure CAN_RX/TX pins mulxed with PTE25/24 for FlexCAN1
	GPIOE_PCR(25) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;
	GPIOE_PCR(24) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;
#endif
        
#ifdef  K53N        
        // Configure CAN_RX/TX pins mulxed with PTC16/17 for FlexCAN1
//       GPIOC_PCR(16) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK  ; // pullup disabled
//	GPIOC_PCR(17) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK  ;           

//	GPIOC_PCR(16) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_ODE_MASK | !PIN_PULL_SELECT_MASK | !PIN_PULL_ENABLE_MASK; // open-drain, OK
//	GPIOC_PCR(17) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_ODE_MASK | !PIN_PULL_SELECT_MASK | !PIN_PULL_ENABLE_MASK;  
        
       GPIOC_PCR(16) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK; // pullup enabled
       GPIOC_PCR(17) = (PIN_ALT2<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;           
#endif	
    /* Interrupt configuration in NVIC of M4 core */
#if TEST_FLEXCAN0
   // Enable clock gating to FlexCAN0
   SIM_SCGC6 |= SIM_SCGC6_FLEXCAN0_MASK;
   /*
    FLEXCAN0 interrupt vector number 45,46,...,52
   IRQ value: Vector#-16 = 29,30,...,36
   IRQ VEC reg: 29/32 = 0,30/32=0,31/32=0,32/32=1,..
   bit to set : 29%32 = 29,30,31, 0,1,2,3,4
   */
   for(i = 0; i < 6; i++)
   {
    enable_irq(29+i);       
   }     
#endif
#if TEST_FLEXCAN1
   // Enable clock gating to FlexCAN1
   SIM_SCGC3 |= SIM_SCGC3_FLEXCAN1_MASK;

   /*
    FLEXCAN1 interrupt vector number 53,54,55,56,57,58,59,60
   IRQ value: Vector#-16 = 37,38,39,40,41,42,43,44
   IRQ VEC reg: 37/32 = 1.xx
   bit to set : 37%32 = 5,6,7,8,9,10,11,12
   */
   for(i = 0; i < 6; i++)
   {
    enable_irq(37+i);       
   }     
   
 #endif	
}

