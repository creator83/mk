/**HEADER********************************************************************
* 
* Copyright (c) 2009 Freescale Semiconductor;
* All Rights Reserved
*
*************************************************************************** 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
*  FileName: fcandrv.c
*  Version : 1.0.0
*
* Comments:
*
*   Revision History:
*   Date             Version  Changes
*   ---------        -------  -------
*   12/8/2009        1.00     Initial version
*
*END************************************************************************/

#include <stdio.h>
#include "fcandrv.h"



 FLEXCAN_REG_STRUCT_PTR  _bsp_get_flexcan_base_address(uint_8 dev_num)
{
   FLEXCAN_REG_STRUCT_PTR fcan_base = (FLEXCAN_REG_STRUCT_PTR)NULL;
   switch (dev_num)
   {
   	case FLEXCAN0:   fcan_base = (FLEXCAN_REG_STRUCT_PTR)FLEXCAN0_BASE;
   		  break;
   	case FLEXCAN1:	  fcan_base = (FLEXCAN_REG_STRUCT_PTR)FLEXCAN1_BASE;
   		  break;
   	default:  break;
   }
   return fcan_base;
}


/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Start  
* Returned Value   : uint_32 
* Comments         :
*    This function starts the specified FlexCAN device
*
*END*********************************************************************/
uint_32 FLEXCAN_Start
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   /* Starting FLEXCAN in normal mode */
  can_reg_ptr->CANMCR &= ~(FLEXCAN_CANMCR_HALT);
 	
   
   /* wait for synchronization */
   while(can_reg_ptr->CANMCR & FLEXCAN_CANMCR_FRZACK){} 
   while((can_reg_ptr->CANMCR & FLEXCAN_CANMCR_NOTRDY)) {}
   
   return( FLEXCAN_OK );
} /* Endbody */	

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Softreset
* Returned Value   : uint_32 
* Comments         :
*    This function resets the specific CAN device
*
*END*********************************************************************/
uint_32 FLEXCAN_Softreset
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num
)
{ /* Body */

   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

  /* check for low power mode */
  if(can_reg_ptr->CANMCR & FLEXCAN_CANMCR_LPMACK)
  {
     /* Enable clock */
     can_reg_ptr->CANMCR &= (~FLEXCAN_CANMCR_MDIS);
	 /* wait until enabled */
	 while (can_reg_ptr->CANMCR & FLEXCAN_CANMCR_LPMACK){}
  }

  /* Reset the FLEXCAN */
  can_reg_ptr->CANMCR = FLEXCAN_CANMCR_SOFTRST;

  /* Wait for reset cycle to complete */
  while (can_reg_ptr->CANMCR & FLEXCAN_CANMCR_SOFTRST){}

  /* Set Freeze, Halt */
  can_reg_ptr->CANMCR |= (FLEXCAN_CANMCR_HALT);


#if 0
  /* check for freeze Ack */
  if( 
     ( (can_reg_ptr->CANMCR & FLEXCAN_CANMCR_FRZACK) != FLEXCAN_CANMCR_FRZACK ) ||
	 ( (can_reg_ptr->CANMCR & FLEXCAN_CANMCR_NOTRDY) != FLEXCAN_CANMCR_NOTRDY )
    )
  	return (FLEXCAN_SOFTRESET_FAILED);
#else
  /* check for freeze Ack */
  while( 
     ( (can_reg_ptr->CANMCR & FLEXCAN_CANMCR_FRZACK) != FLEXCAN_CANMCR_FRZACK ) ||
	 ( (can_reg_ptr->CANMCR & FLEXCAN_CANMCR_NOTRDY) != FLEXCAN_CANMCR_NOTRDY )
    )	{}
#endif   	
  return (FLEXCAN_OK);

} /* Endbody */

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Get_msg_object  
* Returned Value   : vpointer 
* Comments         :
*    This function returns a vpointer to the mailbox object specified
*    by the mailbox number.
*
*END*********************************************************************/
vpointer FLEXCAN_Get_msg_object
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number 
)
{ /* Body */

   volatile uchar_ptr temp_ptr;
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (NULL);
   }

   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB(NUMBER_OF_MB-1)) 
   {
      return (NULL);
   }

   temp_ptr = (uchar_ptr)can_reg_ptr;
   temp_ptr += FLEXCAN_MSG_BUFADDR_OFFSET;

 
   /*
   ** Calculate what message object to point to
   */
   temp_ptr += sizeof(FLEXCAN_MSG_OBJECT_STRUCT) * mailbox_number;

   return ((vpointer)temp_ptr);

} /* Endbody */

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Select_mode  
* Returned Value   : uint_32 
* Comments         :
*    This function sets FlexCAN module operation mode
*
*END*********************************************************************/
uint_32 FLEXCAN_Select_mode
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] operation Mode */
   uint_32 mode 
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR can_reg_ptr;
   volatile uint_32				   ret_code = FLEXCAN_OK;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   switch(mode)
   {
   case (FLEXCAN_NORMAL_MODE):
      /* 
      ** Normal mode, check freeze ACK bit
      ** call start if bit is set 
      */
	  if( (can_reg_ptr->CANMCR & FLEXCAN_CANMCR_FRZACK) == FLEXCAN_CANMCR_FRZACK)
	  {
	     if( FLEXCAN_Start(dev_num) )
	        ret_code = FLEXCAN_START_FAILED;
	  }
	  break;
   case (FLEXCAN_LISTEN_MODE):
      /* Set CANCTRL[LOM], listen-only mode */
      can_reg_ptr->CANCTRL |= FLEXCAN_CANCTRL_LOM;
	  break;
   case (FLEXCAN_TIMESYNC_MODE):
      /* Timer Synchronization mode (Global Network Time) */
      can_reg_ptr->CANCTRL |= FLEXCAN_CANCTRL_TSYNC;
	  break;
   case (FLEXCAN_LOOPBK_MODE):
      /* Self test mode, Loop Back */
      can_reg_ptr->CANCTRL |= FLEXCAN_CANCTRL_LPB;
	  break;
   case (FLEXCAN_BOFFREC_MODE):
      /* Bus Off Recovery mode (according to CAN 2.0b) */
      can_reg_ptr->CANCTRL &= ~(FLEXCAN_CANCTRL_BOFFREC);
	  break;
   case (FLEXCAN_FREEZE_MODE):
      /* Debug mode, Halt and Freeze */
      can_reg_ptr->CANMCR |= (FLEXCAN_CANMCR_FRZ | FLEXCAN_CANMCR_HALT);
     /* check for freeze Ack */
     if( (can_reg_ptr->CANMCR & FLEXCAN_CANMCR_FRZACK) != FLEXCAN_CANMCR_FRZACK)
        ret_code = FLEXCAN_FREEZE_FAILED;
	  break;
   case (FLEXCAN_DISABLE_MODE):
      /* Debug mode, Halt and Freeze */
      can_reg_ptr->CANMCR |= FLEXCAN_CANMCR_MDIS;
	  break;
   default:
      ret_code = FLEXCAN_INVALID_MODE;
   }
   
   return( ret_code );
} /* Endbody */	

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Select_CLK  
* Returned Value   : uint_32 
* Comments         :
*    This function sets FlexCAN module operation mode
*
*END*********************************************************************/
uint_32 FLEXCAN_Select_clk
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] FlexCAN clock source */
   uint_32 clk 
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   if(clk == FLEXCAN_IPBUS_CLK)
   /* Internal bus clock (fsys/2) */
	  can_reg_ptr->CANCTRL |= FLEXCAN_CANCTRL_CLK_SRC;
   
   else if (clk == FLEXCAN_OSC_CLK)
      /* External clock */
      can_reg_ptr->CANCTRL &= (~FLEXCAN_CANCTRL_CLK_SRC);
   
   else
      return (FLEXCAN_CLOCK_SOURCE_INVALID);
   
   
   return( FLEXCAN_OK );
} /* Endbody */	

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Initialize  
* Returned Value   : uint_32 
* Comments         :
*    This function initializes all mailbox-independant registers of the 
*    FLEXCAN chip.
*	 NOTE:
*         TimeSeg1 = PROP_SEG + PHASE_SEG1
*         TimeSeg2 = PHASE_SEG2
*		Ex. PHASE_SEG1 = 0xF and PROP_SEG = 0x7 => bit_timing0 = 0x000F0007
*
*END*********************************************************************/
uint_32 FLEXCAN_Initialize
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] MS 16 bits = TimeSeg1, LS 16 bits = TimeSeg2 */ 
   uint_32 bit_timing0,
   /* [IN] MS 16 bits = prescaller value, LS 16 bits = SJW */ 
   uint_32 bit_timing1,
   /* [IN] Bit rate in Kbps */ 
   uint_32 frequency,
   /* [IN] clock source */
   uint_32 clk,
   /* [IN] is loopback */
   uint_8  loopback
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR can_bufstruct_ptr;       
   volatile uchar_ptr                     temp_ptr;
   vuint_32								  PSEG1 =0;
   vuint_32								  PSEG2 = 0;
   vuint_32                               PROPSEG = 0;
   vuint_32								  RJW = 0;
   vuint_32								  PRESDIV = 0;
   uint_32                               i;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   temp_ptr = (uchar_ptr)can_reg_ptr;
   temp_ptr += FLEXCAN_MSG_BUFADDR_OFFSET;

   can_bufstruct_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)temp_ptr;
  
   /* 
   ** To access the memory mapped registers 
   ** Entre disable mode (hard reset).
   */
   if(!(can_reg_ptr->CANMCR & FLEXCAN_CANMCR_MDIS))
   {
      /* clock disable (module) */		
      can_reg_ptr->CANMCR = FLEXCAN_CANMCR_MDIS;

	  /* wait until disable mode acknowledged */
	  while (!can_reg_ptr->CANMCR & FLEXCAN_CANMCR_LPMACK) {}
   }   
   
   /* 
   ** Select the clock source before soft reset.
   ** Defualt: internal bus clock (fsys/2).
   */
   FLEXCAN_Select_clk (dev_num, clk);
   
   /* Reset FLEXCAN, Halt, freeze mode */
   if(FLEXCAN_Softreset(dev_num))
      return (FLEXCAN_INIT_FAILED);
  
   /* bit timing settings */
   if( (bit_timing0 == 0x00) && (bit_timing1 == 0x00) )
   {
      if (can_reg_ptr->CANCTRL & FLEXCAN_CANCTRL_CLK_SRC) 
      {
	      switch (BSP_SYSTEM_CLOCK)
	      {
	      	case (100000000UL):	// 100Mhz IPBus clock
		      switch (frequency)
		      {
		      case (33):	// 33.33K
			     /*
				 ** PROPSEG = 0x2, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0
				 ** RJW = 0x4, PSEG1 = 0x6, PSEG2 = 0x6, PRESDIV = 200
			     */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(1) | FLEXCAN_CANCTRL_RJW(3)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(5) | FLEXCAN_CANCTRL_PSEG2(5)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(199L));
			     break;
		      case (83):	// 83.33K
			     /*
				 ** PROPSEG = 0x1, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0
				 ** RJW = 0x4, PSEG1 = 0x5, PSEG2 = 0x5, PRESDIV = 100
			     */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(0) | FLEXCAN_CANCTRL_RJW(3)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(4) | FLEXCAN_CANCTRL_PSEG2(4)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(99));
			     break;
		      case (50):
		         /* 
		         ** PROPSEG = 0x1, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x4, PSEG1 = 0x4, PSEG2 = 0x4, PRESDIV = 200
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(0) | FLEXCAN_CANCTRL_RJW(3)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(4) | FLEXCAN_CANCTRL_PSEG2(4)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(199));
			     break;
		      case (100):
		         /* 
		         ** PROPSEG = 0x1, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x4, PSEG1 = 0x4, PSEG2 = 0x4, PRESDIV = 100
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(0) | FLEXCAN_CANCTRL_RJW(3)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(99));
			     break;
		      case (125):
		         /* 
		         ** PROPSEG = 0x1, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x3, PSEG1 = 0x3, PSEG2 = 0x3, PRESDIV = 100
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(0) | FLEXCAN_CANCTRL_RJW(2)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(2)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(99));
			     break;
		      case (250):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x4, PSEG1 = 0x8, PSEG2 = 0x8, PRESDIV = 16
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(3)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(15));
			     break;
		      case (500):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x4, PSEG1 = 0x8, PSEG2 = 0x8, PRESDIV = 8
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(3)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(7));
			     break;
		      case (1000):
			     /*
				 ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x1
				 ** RJW = 0x4, PSEG1 = 0x8, PSEG2 = 0x8, PRESDIV = 4
			     */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(3)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(3) | FLEXCAN_CANCTRL_SAMP);
			     break;
		      default: 
		            return (FLEXCAN_INVALID_FREQUENCY);
                            break;
			  }
			  break;
	      	case (48000000UL):	// 48Mhz IPBus clock on K53
		      switch (frequency)
		      {
		      case (33):	// 33.33K
		         /* 
                         ** 48M/120= 400k sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 120
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(2)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(119));
			     break;
		      case (83):	// 83.33K
		         /* 
                         ** 48M/48= 1M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 48
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(2)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(47));
			     break;
		      case (50):
		         /* 
                         ** 48M/80= 0.6M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 40
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(79));
			     break;
		      case (100):
		         /* 
                         ** 48M/40= 1.2M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 40
		         */
                        can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(2)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(39));
			     break;
		      case (125):
		         /* 
                         ** 48M/32= 1.5M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 32
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(2)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(31));
			     break;
		      case (250):
		         /* 
                         ** 48M/16= 3M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 2, PSEG1 = 4, PSEG2 = 4, PRESDIV = 16
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(15));
			     break;
		      case (500):
		         /* 
                         ** 48M/8=6M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 2, PSEG1 = 4, PSEG2 = 4, PRESDIV = 6
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(7));
			     break;
		      case (1000):  // PASS
			     /*  
                                 ** 48M/6=8M sclock
				 ** PROPSEG = 4, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 1, PSEG1 = 1, PSEG2 = 2, PRESCALER = 6
			     */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(3) | FLEXCAN_CANCTRL_RJW(0)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(0) | FLEXCAN_CANCTRL_PSEG2(1)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(5) | !FLEXCAN_CANCTRL_SAMP);
			     break;
		      default: 
		         return (FLEXCAN_INVALID_FREQUENCY);
			  }
			  break;                      
	      	default: return FLEXCAN_CLOCK_SOURCE_INVALID;
                          break;
	      }
      }
      else
      {	// external clock source
	      switch (BSP_SYSTEM_CLOCK)
	      {
	      	case (12000000UL):     // 12 MHz external crystal
		      switch (frequency)
		      {

		      case (33):	// 33.33K
		         /* 
                         ** 12M/20= 600k sclock, 18Tq
		         ** PROPSEG = 1, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 4, PSEG1 = 8, PSEG2 = 8,PRESDIV = 20
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(0) | FLEXCAN_CANCTRL_RJW(3)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(19));
			     break;
		      case (83):	// 83.33K = 1M/12
		         /* 
                         ** 12M/12= 1M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 12
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(2)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(11));
			     break;
		      case (50):
		         /* 
                         ** 12M/20= 0.6M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 20
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(2)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(19));
			     break;
		      case (100):
		         /* 
                         ** 12M/10= 1.2M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 10
		         */
                        can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(2)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(9));
			     break;
		      case (125):
		         /* 
                         ** 12M/8= 1.5M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 8
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(2)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(7));
			     break;
		      case (250):
		         /* 
                         ** 12M/4= 3M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 2, PSEG1 = 4, PSEG2 = 4, PRESDIV = 4
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(3));
			     break;
		      case (500):
		         /* 
                         ** 12M/2=6M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 2, PSEG1 = 4, PSEG2 = 4, PRESDIV = 2
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(1));
			     break;
		      case (1000):  // 
			     /*  
                                 ** 12M/1=12M sclock,12Tq
				 ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 4, PSEG1 = 4, PSEG2 = 4, PRESCALER = 1
			     */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(3)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(0) | !FLEXCAN_CANCTRL_SAMP);
			     break;
		      default: 
		         return (FLEXCAN_INVALID_FREQUENCY);
			  }
			  break;
	      	case (80000000UL):     // kirin3 - 48 MHz crystal
		      switch (frequency)
		      {
		      case (10):
			     /*
				 ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0
				 ** RJW = 0x2, PSEG1 = 0x7, PSEG2 = 0x8, PRESDIV = 200
			     */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(6) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(199L));
			     break;
		      case (20):
			     /*
				 ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0
				 ** RJW = 0x2, PSEG1 = 0x7, PSEG2 = 0x8, PRESDIV = 100
			     */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(6) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(99));
			     break;
		      case (50):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x2, PSEG1 = 0x7, PSEG2 = 0x8, PRESDIV = 40
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(6) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(39));
			     break;
		      case (100):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x2, PSEG1 = 0x7, PSEG2 = 0x8, PRESDIV = 20
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(6) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(39));
			     break;
		      case (125):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x2, PSEG1 = 0x7, PSEG2 = 0x8, PRESDIV = 16
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(6) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(15));
			     break;
		      case (250):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x2, PSEG1 = 0x7, PSEG2 = 0x8, PRESDIV = 8
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(6) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(7));
			     break;
		      case (500):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x2, PSEG1 = 0x7, PSEG2 = 0x8, PRESDIV = 4
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(6) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(3));
			     break;
		      case (800):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x2, PSEG1 = 0x4, PSEG2 = 0x8, PRESDIV = 3
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(2));
			     break;
		      case (1000):
			     /*
				 ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x1
				 ** RJW = 0x2, PSEG1 = 0x7, PSEG2 = 0x8, PRESDIV = 2
			     */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(6) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(1) | FLEXCAN_CANCTRL_SAMP);
			     break;
		      default: 
		         return (FLEXCAN_INVALID_FREQUENCY);
			  }
			  break;                      
	      	default:     // anything else: 25 Mhz crystal
		      switch (frequency)
		      {
		      case (10):
			     /*
				 ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0
				 ** RJW = 0x2, PSEG1 = 0x8, PSEG2 = 0x8, PRESDIV = 100
			     */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(99));
			     break;
		      case (20):
			     /*
				 ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0
				 ** RJW = 0x2, PSEG1 = 0x8, PSEG2 = 0x8, PRESDIV = 50
			     */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(49));
			     break;
		      case (50):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x2, PSEG1 = 0x8, PSEG2 = 0x8, PRESDIV = 20
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(19));
			     break;
		      case (100):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x2, PSEG1 = 0x8, PSEG2 = 0x8, PRESDIV = 10
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(9));
			     break;
		      case (125):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x2, PSEG1 = 0x8, PSEG2 = 0x8, PRESDIV = 8
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(7));
			     break;
		      case (250):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x2, PSEG1 = 0x8, PSEG2 = 0x8, PRESDIV = 4
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(3));
			     break;
		      case (500):
		         /* 
		         ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x0 
				 ** RJW = 0x2, PSEG1 = 0x8, PSEG2 = 0x8, PRESDIV = 2
		         */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(1));
			     break;
		      case (1000):
			     /*
				 ** PROPSEG = 0x8, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 0x1
				 ** RJW = 0x2, PSEG1 = 0x8, PSEG2 = 0x8, PRESDIV = 1
			     */
		         can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(7) | FLEXCAN_CANCTRL_RJW(1)
			 	    					    | FLEXCAN_CANCTRL_PSEG1(7) | FLEXCAN_CANCTRL_PSEG2(7)
			 	    					    | FLEXCAN_CANCTRL_PRESDIV(0) | FLEXCAN_CANCTRL_SAMP);
			     break;
		      default: 
		         return (FLEXCAN_INVALID_FREQUENCY);
			  }
			  break;
	      }
      }
   }
   else
   {
      PSEG1 = (bit_timing0 & 0x00070000) >> 16; 
      PROPSEG = bit_timing0 & 0x7;
	  PSEG2 = (bit_timing1 & 0x00070000) >> 16;
	  RJW = (bit_timing1 & 0x00000300) >> 8;
	  PRESDIV = bit_timing1 & 0x000000FF;

      can_reg_ptr->CANCTRL = (0 | FLEXCAN_CANCTRL_PROPSEG(PROPSEG) | FLEXCAN_CANCTRL_RJW(RJW)
	       					     | FLEXCAN_CANCTRL_PSEG1(PSEG1) | FLEXCAN_CANCTRL_PSEG2(PSEG2)
	 	    					 | FLEXCAN_CANCTRL_PRESDIV(PRESDIV));
   }
   
   if(loopback)
   {
   	 can_reg_ptr->CANCTRL |= FCAN_CANCTRL_LPB;
   }

   /* Set maximum number of message buffers */
   can_reg_ptr->CANMCR |= FLEXCAN_CANMCR_MAXMB(NUMBER_OF_MB-1);

   /* Rx global mask */
   can_reg_ptr->RXGMASK   = FLEXCAN_EXTENDED_ID_MASK(0xFFFFFFFF); 

   /* Rx reg 14 mask */
   can_reg_ptr->RX14MASK  = FLEXCAN_EXTENDED_ID_MASK(0xFFFFFFFF); 

   /* Rx reg 15 mask */
   can_reg_ptr->RX15MASK  = FLEXCAN_EXTENDED_ID_MASK(0xFFFFFFFF); 

   /* 
   * Initialize all message buffers as inactive 
   */
   for (i = 0; i <= FLEXCAN_CANMCR_MAXMB(NUMBER_OF_MB-1); i++)
   {
	   can_bufstruct_ptr[i].CONTROL  = 0x00000000;
   }
   
   return (FLEXCAN_OK);
} /* Endbody */


/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Initialize_mailbox 
* Returned Value   : uint_32 
* Comments         :
*    This function initializes the specified FlexCAN mailbox (Message Buffer)
*    with the corresponding activate code.
*
*END*********************************************************************/
uint_32 FLEXCAN_Initialize_mailbox
(
   FLEXCAN_MailBox_STRUCT_PTR pMailBox,
   uint_8  activate_it,
   uint_32 int_enable
)
{ /* Body */
   FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   FLEXCAN_MSG_OBJECT_STRUCT_PTR msg_obj_ptr;
   uint_32                                temp_msg_config;
   int16_t								  iDLC;
   uint_8	dev_num = pMailBox->dev_num;
   uint_16	mailbox_number = pMailBox->mailbox_number;
   uint_32	identifier = pMailBox->identifier;
   uchar_ptr pdata = (uchar_ptr)pMailBox->data;
   uint_8	data_len_code = pMailBox->data_len;
   uint_8	direction = pMailBox->direction;
   uint_8	format = pMailBox->format;
   uint_8	code = pMailBox->code;
   uint_8	remoteflag = pMailBox->remote_req_flag;
 

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if (  mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }
   
   if ( (1 > data_len_code) || (8 < data_len_code) )
   {
      return (FLEXCAN_DATA_SIZE_ERROR);
   }

   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(
                                                 dev_num, mailbox_number);   
   /*
   ** TX = 0x80: MB is not ready for transmit (code=0b1000)
   ** RX = 0x00: MB is not active (code=0b0000)
   ** Set code depending on direction and set the number of data bytes to be stored 
   */
   if (direction == FLEXCAN_TX) 
   {
      temp_msg_config =   FLEXCAN_TX_MSG_BUFFER_NOT_ACTIVE;	  
   } 
   else if (direction == FLEXCAN_RX) 
   {
      temp_msg_config =  FLEXCAN_RX_MSG_BUFFER_NOT_ACTIVE;
   } 
   else 
   {
      return (FLEXCAN_INVALID_DIRECTION);
   }/* Endif */
   
   /* Deactivate the MB */  
   msg_obj_ptr->CONTROL = temp_msg_config;
   
   if(activate_it)
   {
   		temp_msg_config =  FLEXCAN_set_msg_ctrlcode(code); 
   }
   /* Set remote bit */
   if(remoteflag)
   {
   		temp_msg_config |= FLEXCAN_MSGCTRL_RTR;
   }

   /* Set the ID according the format structure */
   if ( format == FLEXCAN_EXTENDED ) 
   {
      /* Set IDE in C/S */
	  temp_msg_config |= FLEXCAN_MSGCTRL_IDE;
   	  
  	  /* Set SRR bit for Tx buffers  in C/S*/
  	  if(direction == FLEXCAN_TX)
         temp_msg_config |= FLEXCAN_MSGCTRL_SRR;


   	  msg_obj_ptr->ID.ID = FLEXCAN_EXTENDED_ID_MASK(identifier);
   } 
   else if( format == FLEXCAN_STANDARD )
   {
	  /* make sure IDE and SRR are not set */
	  temp_msg_config &= ~(FLEXCAN_MSGCTRL_IDE | FLEXCAN_MSGCTRL_SRR);

	  /* ID[28-18] */
      msg_obj_ptr->ID.ID = FLEXCAN_STANDARD_ID_MASK(identifier);
   } 
   else
      return (FLEXCAN_MESSAGE_FORMAT_UNKNOWN);
   /* Endif */
   
   if(direction == FLEXCAN_TX)
   {
   		/* write data bytes */
   		for(iDLC = 0; iDLC < data_len_code; iDLC++)
   		{
   			msg_obj_ptr->DATA[iDLC] = pdata[iDLC];
   		}
   }

 
  /* Set transmit length to data length in bytes */ 
   temp_msg_config |= (data_len_code << 16);
   
   //printf("MB %d: c/s = %#08.8x, ID=%#08.8x\n",mailbox_number,temp_msg_config,msg_obj_ptr->ID.ID);
   
    /* Write actual code*/
   msg_obj_ptr->CONTROL = temp_msg_config;
  
   /* Enable CAN interrupt */
   if(int_enable)
   {
	  /* Enable interrupt line */
      if( FLEXCAN_Int_enable(dev_num, mailbox_number) )
   	     return (FLEXCAN_INT_ENABLE_FAILED);
   } 
   else
   {
	  /* Disable interrupt line */
	  if( FLEXCAN_Int_disable(dev_num, mailbox_number) )
	     return (FLEXCAN_INT_DISABLE_FAILED);
   } 

   return ( FLEXCAN_OK );
} /* Endbody */


/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Request_mailbox 
* Returned Value   : uint_32 
* Comments         :
*    This function sets the RTR bit (reomte frame) for the 
*    specified mailbox 
*
*END*********************************************************************/
uint_32 FLEXCAN_Request_mailbox
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */ 
   uint_32 mailbox_number,
   /* [IN] mailbox format (CAN_STANDARD OR CAN_EXTENDED) */ 
   uint_32 format
)
{ /* Body */
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR msg_obj_ptr;
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }

   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(
                                                 dev_num, mailbox_number);
   /* Set RTR */
   msg_obj_ptr->CONTROL |= FLEXCAN_MSGCTRL_RTR;
   
   return (FLEXCAN_OK);
} /* Endbody */

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Activate_mailbox  
* Returned Value   : uint_32 
* Comments         :
*    This function activates the specified mailbox with the activation code
*	 provided.
*
*END*********************************************************************/
uint_32 FLEXCAN_Activate_mailbox
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number,
   /* [IN] Code value for CONTROL/STATUS field */
   uint_32 code_val
)
{ /* Body */
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr; 
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }

   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, 
                                                          mailbox_number);
   /* Reset the code */
   msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCRTL_CODE);

   /* Activating mailbox */
   msg_obj_ptr->CONTROL |= code_val;
   
    
   return (FLEXCAN_OK);
   
} /* Endbody */	

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Abort_Tx_mailbox_block  
* Returned Value   : uint_32 
* 	 FLEXCAN_INVALID_ADDRESS	if CAN register vpointer is NULL
*	 FLEXCAN_INVALID_MAILBOX	if mailbox_number is out of boundary
*	 FLEXCAN_MB_ABORTED			if mailbox is aborted
*	 FLEXCAN_MB_TRANSMITTED		if mailbox is transmitted instead of being aborted
*	 
* Comments         :
*    This function aborts the specified mailbox 
*    until it is aborted. It is in block mode.
* 
*
*END*********************************************************************/
uint_32 FLEXCAN_Abort_Tx_mailbox_block
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number
)
{ /* Body */
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr; 
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   uint_32 code;
   uint_32 mask;
   uint_32 state = FLEXCAN_OK;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }

   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, 
                                                          mailbox_number);
   /* Write ABORT code in atomic code */
   msg_obj_ptr->CONTROL = (msg_obj_ptr->CONTROL & ~FLEXCAN_MSGCRTL_CODE) | FLEXCAN_TX_MB_ABORT;

   /* 
    * 1) CPU waits for the corresponding IFLAG indicating that the frame was either transmitted or
    * aborted.
    * 2) CPU reads the CODE field to check if the frame was either transmitted (CODE=0b1000) or
    * aborted (CODE=0b1001).
    * 3) It is necessary to clear the corresponding IFLAG in order to allow the MB to be reconfigured.
    */
   if(mailbox_number < 32)
   {
   	   // Disable MB interrupt
   	   mask =  FLEXCAN_EnterCritical(dev_num,mailbox_number);
	   while(!(can_reg_ptr->IFLAG1 & (1<<mailbox_number)) ) 
	   {	   	
	   }
   	   // restore MB interrupt
	   FLEXCAN_ExitCritical(dev_num,mailbox_number,mask);
   }
   else
   {
   	   // Disable MB interrupt
   	   mask =  FLEXCAN_EnterCritical(dev_num,mailbox_number);
	   while(!(can_reg_ptr->IFLAG2 & (1<<(mailbox_number-32))))
	   {	   	
	   }   	
   	   // restore MB interrupt
	   FLEXCAN_ExitCritical(dev_num,mailbox_number,mask);
   }
   code = msg_obj_ptr->CONTROL & FLEXCAN_MSGCRTL_CODE;
   if( code == FLEXCAN_TX_MB_ABORT)
   {
   		state = FLEXCAN_MB_ABORTED;
   }
   else if (code == FLEXCAN_TX_MSG_BUFFER_NOT_ACTIVE)
   {
   		state = FLEXCAN_MB_TRANSMITTED;
   }
	if(mailbox_number < 32)
	{
		can_reg_ptr->IFLAG1 = (1<<mailbox_number);
	}
	else
	{
		can_reg_ptr->IFLAG2 = (1<<(mailbox_number-32)); 			
	}    	   
   
   return (state);
   
} /* Endbody */	


/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Abort_Tx_mailbox_non_block 
* Returned Value   : 
*	 MB state
* 	 FLEXCAN_INVALID_ADDRESS	if CAN register vpointer is NULL
*	 FLEXCAN_INVALID_MAILBOX	if mailbox_number is out of boundary
*	 FLEXCAN_MB_ABORTED			if mailbox is aborted
*	 FLEXCAN_MB_TRANSMITTED		if mailbox is transmitted instead of being aborted
*	 FLEXCAN_MESSAGE_BUSY		if mailbox is still in transmitting
*	 
* Comments         :
*    This function aborts the specified mailbox but does not wait 
*    until it is aborted. It is in non block mode. 
*    If the returned MB state is FLEXCAN_MB_ABORTED or FLEXCAN_MB_TRANSMITTED, the corresponding
* 	 IFLAG bits are cleared for the given mailbox. Otherwise, the caller must 
*	 wait for the corresponding IFLAG bits to be set and then clear the corresponding IFLAG bits.
*END*********************************************************************/
uint_32 FLEXCAN_Abort_Tx_mailbox_non_block
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number
)
{ /* Body */
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr; 
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   uint_32 state = FLEXCAN_OK;
   uint_32 mask;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }

   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, 
                                                          mailbox_number);
   /* Write ABORT code in atomic code */
   msg_obj_ptr->CONTROL = (msg_obj_ptr->CONTROL & ~FLEXCAN_MSGCRTL_CODE) | FLEXCAN_TX_MB_ABORT;

   /* 
    * 1) CPU checks for the corresponding IFLAG indicating that the frame was either transmitted or
    * aborted.
    * 2) CPU reads the CODE field to check if the frame was either transmitted (CODE=0b1000) or
    * aborted (CODE=0b1001) if IFLAG bits are asserted
    * 3) It is necessary to clear the corresponding IFLAG in order to allow the MB to be reconfigured.
    */
   if(mailbox_number < 32)
   {
   	   // Disable MB interrupt
   	   mask =  FLEXCAN_EnterCritical(dev_num,mailbox_number);
   
	   if((can_reg_ptr->IFLAG1 & (1<<mailbox_number)) ) 
	   {	
	   		state = FLEXCAN_Check_TxMB_State(dev_num,mailbox_number);
	   		can_reg_ptr->IFLAG1 = (1<<mailbox_number);					// clear flag
	   }
	   else
	   {
	   		state = FLEXCAN_MESSAGE_BUSY;
	   }
   	   // restore MB interrupt
	   FLEXCAN_ExitCritical(dev_num,mailbox_number,mask);
	   
   }
   else
   {
   	   // Disable MB interrupt
   	   mask =  FLEXCAN_EnterCritical(dev_num,mailbox_number);
   
	   if((can_reg_ptr->IFLAG2 & (1<<(mailbox_number-32))))
	   {
	   		FLEXCAN_Check_TxMB_State(dev_num,mailbox_number);
	   		can_reg_ptr->IFLAG2 = 1<<(mailbox_number-32);	   			// clear flag
	   }
	   else
	   {
	   		state = FLEXCAN_MESSAGE_BUSY;	   		
	   }   	
   	   // restore MB interrupt
	   FLEXCAN_ExitCritical(dev_num,mailbox_number,mask);
   }
   
   return (state);
   
} /* Endbody */	



/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Reconfig_Rx_mailbox_block 
* Returned Value   : 
*	 MB state
* 	 FLEXCAN_INVALID_ADDRESS	if CAN register vpointer is NULL
*	 FLEXCAN_INVALID_MAILBOX	if mailbox_number is out of boundary
*	 FLEXCAN_MB_ABORTED			if mailbox is aborted
*	 FLEXCAN_MB_TRANSMITTED		if mailbox is transmitted instead of being aborted
*	 FLEXCAN_MESSAGE_BUSY		if mailbox is still in transmitting
*	 
* Comments         :
*    This function reconfigures the specified mailbox using safe way 
*    known as IMEU. It is in non block mode. It returns received message if
*	 IFLAG bit is set.
*    
*END*********************************************************************/
uint_32 FLEXCAN_Reconfig_Rx_mailbox_block
(
	FLEXCAN_MailBox_STRUCT_PTR pMailBox
)
{ /* Body */
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr; 
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   uint_32 mask;
   uint_8  dev_num = pMailBox->dev_num;
   uint_16 mailbox_number = pMailBox->mailbox_number;
   uint_32 id = pMailBox->identifier;
   uint_8  format = pMailBox->format;
   uint_32 state = FLEXCAN_OK;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }

   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, 
                                                          mailbox_number);
    
    /*
    1. check if the target element has its interrupt flag (IFLAG) is negated. If IFLAG is asserted, CPU
	must service this element and clear the corresponding IFLAG;
	2. wait for both IMEUREQ and IMEUACK bits to be negated;
	3. configure IMEUP bits with the respective element address vpointer;
	4. request IMEU by asserting IMEUREQ bit;
	5. check IMEUREQ and IMEUACK bits, if both are asserted
	a) update the Mailbox/RxFIFO matching elements;
	b) negate IMEUREQ bit;
	6. it is recommended to clear ESR2[IMEUF] flag to keep the coherence with the corresponding
	IMEUREQ and IMEUACK assertion;
	*/
	//printf("in Reconfig_Rx_mailbox_block to check and service a MB\n");
    // Service the target MB completed
    if(mailbox_number < 32)
	{
   	   // Disable MB interrupt
   	   mask =  FLEXCAN_EnterCritical(dev_num,mailbox_number);
	   if((can_reg_ptr->IFLAG1 & (1<<mailbox_number)) ) 
	   {	
	   		state = FLEXCAN_Rx_message(pMailBox,FALSE);
	   		// clear IFLAG bit
	   		can_reg_ptr->IFLAG1 = (1<<mailbox_number);
			//printf("Call FLEXCAN_Rx_message() to read a message\n");
	   }
	   // restore MB interrupt
   	   FLEXCAN_ExitCritical(dev_num,mailbox_number,mask);
	   }
   else
   {
   	   // Disable MB interrupt
   	   mask =  FLEXCAN_EnterCritical(dev_num,mailbox_number);
	   if((can_reg_ptr->IFLAG2 & (1<<(mailbox_number-32))))
	   {
	   		state = FLEXCAN_Rx_message(pMailBox,FALSE);
	   		// clear IFLAG bit
	   		can_reg_ptr->IFLAG2 = (1<<(mailbox_number-32));
			//printf("Call FLEXCAN_Rx_message() to read a message\n");
	   }   	
	   // restore MB interrupt
   	   FLEXCAN_ExitCritical(dev_num,mailbox_number,mask);
   }
   //printf("in Reconfig_Rx_mailbox_block to wait for both IMEUREQ and IMEUACK to be 0\n");
   // wait for both IMEUREQ and IMEUACK bits to be negated;
   while((can_reg_ptr->IMEUR & FLEXCAN_IMEUR_IMEUREQ_MASK) || 
   (can_reg_ptr->IMEUR & FLEXCAN_IMEUR_IMEUACK_MASK)
   ){}

   // Configure IMEUP bits with the respective element address vpointer
   FLEXCAN_Set_IMEUP(can_reg_ptr->IMEUR,mailbox_number);
   
   // request IMEU by asserting IMEUREQ bit;
   can_reg_ptr->IMEUR |= FLEXCAN_IMEUR_IMEUREQ_MASK;
   
   //printf("in Reconfig_Rx_mailbox_block to wait for both IMEUREQ and IMEUACK to be 1 \n");
	// check IMEUREQ and IMEUACK bits
	while( !(can_reg_ptr->IMEUR & FLEXCAN_IMEUR_IMEUREQ_MASK) || 
	!(can_reg_ptr->IMEUR & FLEXCAN_IMEUR_IMEUACK_MASK) )
	{
	}
	//printf("Start to update MB\n");
	//if both are asserted
	// a) update the Mailbox/RxFIFO matching elements;
  // Update the mailbox matching elements
   if ( format == FLEXCAN_EXTENDED ) 
   {
      /* Set IDE */
	  msg_obj_ptr->CONTROL |= FLEXCAN_MSGCTRL_IDE;

	  /* Set SRR bit */
	  msg_obj_ptr->CONTROL |= FLEXCAN_MSGCTRL_SRR;
   	  
   	  msg_obj_ptr->ID.ID = FLEXCAN_EXTENDED_ID_MASK(id);
   } 
   else if( format == FLEXCAN_STANDARD )
   {
	  /* make sure IDE and SRR are not set */
	  msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCTRL_IDE | FLEXCAN_MSGCTRL_SRR);

	  /* ID[28-18] */
      msg_obj_ptr->ID.ID = FLEXCAN_STANDARD_ID_MASK(id);
   } 
 	// b) negate IMEUREQ bit;	   
 	can_reg_ptr->IMEUR &= ~(FLEXCAN_IMEUR_IMEUREQ_MASK); 
 	    
	// it is recommended to clear ESR2[IMEUF] flag to keep the coherence with the corresponding
	// IMEUREQ and IMEUACK assertion;
    can_reg_ptr->ESR2 |= FLEXCAN_ESR2_IMEUF;
   //
    
   return (state);
   
} /* Endbody */	



/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Lock_mailbox  
* Returned Value   : uint_32 
* Comments         :
*    This function locks the specified mailbox 
*
*END*********************************************************************/
uint_32 FLEXCAN_Lock_mailbox
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number 
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr;
   vuint_32									temp; 
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return( FLEXCAN_INVALID_MAILBOX );
   } 

   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, mailbox_number);
   
   /* Lock the mailbox */
   temp = msg_obj_ptr->CONTROL;
   
   return( FLEXCAN_OK );
   
} /* Endbody */	

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_unlock_mailbox  
* Returned Value   : uint_32 
* Comments         :
*    This function unlocks flexcan mailbox 
*
*END*********************************************************************/
uint_32 FLEXCAN_Unlock_mailbox
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   vuint_32									temp;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   /* Unlock the mailbox */
   temp = can_reg_ptr->TIMER;

   return( FLEXCAN_OK );
   
} /* Endbody */	

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Set_global_extmask  
* Returned Value   : uint_32 
* Comments         :
*    This function sets the FLEXCAN chip global extended mask.
*
*END*********************************************************************/
uint_32 FLEXCAN_Set_global_extmask
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] Mask */ 
   uint_32 extmask
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR  can_reg_ptr;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   /* MID[28-0] */
   can_reg_ptr->RXGMASK = FLEXCAN_EXTENDED_ID_MASK(extmask);

   return (FLEXCAN_OK);

} /* Endbody */

 /*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Set_Buf14_extmask  
* Returned Value   : uint_32 
* Comments         :
*    This function sets the FlexCAN buf14 extended mask. 
*
*END*********************************************************************/
uint_32 FLEXCAN_Set_buf14_extmask
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] Mask */ 
   uint_32 extmask
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR  can_reg_ptr;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   /* MID[28-0] */
   can_reg_ptr->RX14MASK = FLEXCAN_EXTENDED_ID_MASK(extmask);
   
   return (FLEXCAN_OK);

} /* Endbody */

 /*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Set_Buf15_extmask  
* Returned Value   : uint_32 
* Comments         :
*    This function sets the FlexCAN buf15 extended mask. 
*
*END*********************************************************************/
uint_32 FLEXCAN_Set_buf15_extmask
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] Mask */ 
   uint_32 extmask
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR  can_reg_ptr;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   /* MID[28-0] */
   can_reg_ptr->RX15MASK = FLEXCAN_EXTENDED_ID_MASK(extmask);
   
   return (FLEXCAN_OK);

} /* Endbody */

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Set_global_stdmask  
* Returned Value   : uint_32 
* Comments         :
*    This function sets the CAN chip global standard mask.
*
*END*********************************************************************/
uint_32 FLEXCAN_Set_global_stdmask
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] Mask */ 
   uint_32 stdmask
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR  can_reg_ptr;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   /* 11 bit standard mask */
   can_reg_ptr->RXGMASK = FLEXCAN_STANDARD_ID_MASK(stdmask);

   return (FLEXCAN_OK);

} /* Endbody */
 /*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Set_Buf14_stdmask  
* Returned Value   : uint_32 
* Comments         :
*    This function sets the FlexCAN buf14 standard mask. 
*
*END*********************************************************************/
uint_32 FLEXCAN_Set_buf14_stdmask
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] Mask */ 
   uint_32 stdmask
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR  can_reg_ptr;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   can_reg_ptr->RX14MASK = FLEXCAN_STANDARD_ID_MASK(stdmask);

   return (FLEXCAN_OK);

} /* Endbody */

 /*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Set_Buf15_stdmask  
* Returned Value   : uint_32 
* Comments         :
*    This function sets the FlexCAN buf15 standard mask. 
*
*END*********************************************************************/
uint_32 FLEXCAN_Set_buf15_stdmask
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] Mask */ 
   uint_32 stdmask
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR  can_reg_ptr;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   can_reg_ptr->RX15MASK = FLEXCAN_STANDARD_ID_MASK(stdmask);

   return (FLEXCAN_OK);

} /* Endbody */

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Tx_successful  
* Returned Value   : boolean 
* Comments         :
*    This function returns TRUE or FALSE depending on whether or not 
*    the message has been placed in the mailbox. 
*
*END*********************************************************************/
boolean FLEXCAN_Tx_successful
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num
)
{  /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR  can_reg_ptr;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FALSE);
   }

   return ((can_reg_ptr->IFLAG1 & FLEXCAN_IMASK_VALUE) != 0);
     
} /* Endbody */   
 
/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Tx_mailbox  
* Returned Value   : uint_32 
* Comments         :
*    This function writes a message to the specified mailbox and sends once
*    ID is same as the specified by the given mailbox.
*
*END*********************************************************************/
uint_32 FLEXCAN_Tx_mailbox
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number, 
   /* [IN] Data to be written to the mailbox */
   vpointer data,
   /* [IN] data length in bytes */
   uint_8  dataSizeBytes
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr; 
   volatile uchar_ptr                       data_array;      
   uint_32                                  i;
   vuint_32									temp;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }
   
   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, mailbox_number);
   
   /* Deactivate the mailbox */
   msg_obj_ptr->CONTROL =  FLEXCAN_TX_MSG_BUFFER_NOT_ACTIVE;

   /* Copy user's buffer into mailbox data area */
   data_array = (uchar_ptr) data;
   for ( i = 0; i < dataSizeBytes ; i++ ) 
   {
      msg_obj_ptr->DATA[i] = data_array[i];
   } /* Endfor */
   
   /* transmit data in message buffer */
   msg_obj_ptr->CONTROL = (FLEXCAN_MESSAGE_TRANSMIT_ONCE | dataSizeBytes);
      
   /* Unlock message buffer */
  // temp = can_reg_ptr->TIMER;

   return (FLEXCAN_OK);
} /* Endbody */

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Rx_mailbox  
* Returned Value   : uint_32 
* Comments         :
*    This function reads a message from the specified mailbox 
*
*END*********************************************************************/
uint_32 FLEXCAN_Rx_mailbox
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32  mailbox_number, 
   /* [IN] Location to store the data from the mailbox */
   vpointer  data 
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr;
   uchar_ptr                        	    data_array;       
   uint_32                                  i;
   uint_32				    return_code = FLEXCAN_OK;
   vuint_32				    data_length;
   vuint_32									DLC;
   vuint_32									temp;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if (mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }
   
   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, mailbox_number);
   
   data_array = (uchar_ptr)data;

   /* Lock */
   temp = msg_obj_ptr->CONTROL; 

   /* Data length */
   data_length = ( temp & FLEXCAN_MSG_CTRL_DLEN );
   DLC = data_length >> 16;

   /* Check for mailbox activation code */
   if ( (temp & FLEXCAN_MSGCRTL_CODE) == FLEXCAN_RX_MSG_BUFFER_EMPTY) 
   {
      /* No data return */
	  return_code = FLEXCAN_NO_MESSAGE;
   } 

   if ( (temp & FLEXCAN_MSGCRTL_CODE) == FLEXCAN_RX_MSG_BUFFER_NOT_ACTIVE ) 
   {
      /* Reset the code */
      msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCRTL_CODE);

      /* mailbox is not active, return */
	  msg_obj_ptr->CONTROL |= FLEXCAN_RX_MSG_BUFFER_EMPTY | data_length;
	  
	  return_code = FLEXCAN_NO_MESSAGE;
   }
   /*
   ** Copy data to user buffer
   */
   if ( (temp & FLEXCAN_MSGCRTL_CODE) == FLEXCAN_RX_MSG_BUFFER_BUSY ) 
   {
      // Wait for copying data
      while(msg_obj_ptr->CONTROL & FLEXCAN_RX_MSG_BUFFER_BUSY) {}

	  // Copying data
      for ( i=0; i<DLC; i++ ) 
      {
         data_array[i] = msg_obj_ptr->DATA[i];
		 msg_obj_ptr->DATA[i] = 0;
      } /* Endfor */
      
      /* Reset the code */
      msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCRTL_CODE);
	  
	  /* Set the code */
	  msg_obj_ptr->CONTROL |= FLEXCAN_RX_MSG_BUFFER_EMPTY | data_length;
	  
	  return_code = FLEXCAN_MESSAGE_BUSY;
   } 
   else if ( (temp & FLEXCAN_MSGCRTL_CODE) == FLEXCAN_RX_MSG_BUFFER_FULL ) 
   {
      // Copying data
      for ( i=0; i<DLC; i++ ) 
      {
         data_array[i] = msg_obj_ptr->DATA[i];
		 msg_obj_ptr->DATA[i] = 0;
      } /* Endfor */
      
      /* Reset the code */
      msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCRTL_CODE);

	  /* Set the code */
	  msg_obj_ptr->CONTROL |= FLEXCAN_RX_MSG_BUFFER_EMPTY | data_length;

	  return_code = FLEXCAN_OK;

   } else if ( (temp & FLEXCAN_MSGCRTL_CODE) == FLEXCAN_RX_MSG_BUFFER_OVERRUN ) 
   {
      // Copying data
      for ( i=0; i<DLC; i++ ) 
      {
         data_array[i] = msg_obj_ptr->DATA[i];
		 msg_obj_ptr->DATA[i] = 0;
      } /* Endfor */
      
      /* Reset the code */
      msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCRTL_CODE);
	  
	  /* Set the code */
	  msg_obj_ptr->CONTROL |= FLEXCAN_RX_MSG_BUFFER_EMPTY | data_length;

	  return_code = FLEXCAN_MESSAGE_LOST;
   }

   /* Unlock message buffer */
   temp = can_reg_ptr->TIMER;

   return return_code;
      
} /* Endbody */

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Disable_mailbox  
* Returned Value   : uint_32 
* Comments         :
*    This function disables the specified mailbox 
*
*END*********************************************************************/
uint_32 FLEXCAN_Disable_mailbox
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number 
)
{ /* Body */

   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }
     
   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, mailbox_number);
   
   msg_obj_ptr->CONTROL &= FLEXCAN_MSG_DISABLE;

   return (FLEXCAN_OK);
   
} /* Endbody */

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Request_message  
* Returned Value   : uint_32 
* Comments         :
*    This function requests remote messages as specified by the
*    mailbox number and format parameters. Sets the message buffer's code 
*
*END*********************************************************************/
uint_32 FLEXCAN_Request_message
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */ 
   uint_32 mailbox_number,
   /* [IN] mailbox format (CAN_STANDARD OR CAN_EXTENDED) */ 
   uint_32 format
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr;
   uint_32                 		    error_code;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return( FLEXCAN_INVALID_MAILBOX );
   }

   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, mailbox_number);
      
   /* Set message format to remote */
   error_code = FLEXCAN_Request_mailbox( dev_num, mailbox_number, format );      
   if ( error_code != FLEXCAN_OK ) 
   {
      return( error_code );
   }
   
   /* 
   ** Activate mailbox: code 0b1100 
   ** Data field length and data in remote message is ignored.
   ** Mailbox becomes receive mailbox.
   */
   error_code = FLEXCAN_Activate_mailbox( dev_num, mailbox_number, FLEXCAN_MESSAGE_TRANSMIT_REMOTE );
   return( error_code );
} /* Endbody */   

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Rx_message  
* Returned Value   : uint_32 
* Comments         :
*    This function receives a message from the specified masked mailbox .
*	 It should be called when the corresponding IFLAG bit is set.
*
*END*********************************************************************/
uint_32 FLEXCAN_Rx_message
( 
   FLEXCAN_MailBox_STRUCT_PTR pMailBox,
   uint_32     int_enable
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr;

   vuint_32                code;			   
   uint_32                 i;
   uint_32				   return_code = FLEXCAN_OK;
   vuint_32				   temp;
   vuint_32				   DLC;
   vuint_32				   ID;
   vuint_32				   dlen;
   uint_8	   dev_num;
   uint_32     mailbox_number;
   uint_32     format;
    
   dev_num = pMailBox->dev_num;
   mailbox_number = pMailBox->mailbox_number;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return( FLEXCAN_INVALID_MAILBOX );
   }
   
   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, mailbox_number);
   
   /* Check Control/Status word first */
      
   /*
   ** if "code" value for specified Rx message buffer is full
   ** copy data to the buffer and set the code to empty and return. 
   ** if "code" value is set to not active, set the code to empty and
   ** return message buffer empty.
   ** if "code" value is set to overrun, copy data to the buffer and
   ** set the code to empty then return overrun.
   ** if "code" value is set to busy, copy data to the buffer and
   ** set the code to empty then return busy.
   */
   code = msg_obj_ptr->CONTROL & FLEXCAN_MSGCRTL_CODE;
   pMailBox->code = FLEXCAN_get_msg_ctrlcode(code);
   
   // Get ID type
   format = (msg_obj_ptr->CONTROL & FLEXCAN_MSGCTRL_IDE)? FLEXCAN_EXTENDED : FLEXCAN_STANDARD;
   pMailBox->format = format;
   
   // Get the message type
   pMailBox->remote_req_flag = (msg_obj_ptr->CONTROL & FLEXCAN_MSGCTRL_RTR)? TRUE : FALSE;
   
   if ( code == FLEXCAN_RX_MSG_BUFFER_EMPTY) 
   {      
      /* No data return */
      return_code = FLEXCAN_NO_MESSAGE;
   }
   else 
   if ( code == FLEXCAN_RX_MSG_BUFFER_NOT_ACTIVE ) 
   {
      /* Reset the code */
      msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCRTL_CODE);
      
      /* Activate this MB to receive message again */
      msg_obj_ptr->CONTROL |= (FLEXCAN_RX_MSG_BUFFER_EMPTY);
      
      /* No data return */
      return_code = FLEXCAN_NO_MESSAGE;
   }
   else
   {
	 /* store data len */
	   dlen = msg_obj_ptr->CONTROL & FLEXCAN_MSG_CTRL_DLEN;
	   DLC = dlen >> 16;
	   
	
	   /* Reassemble the message ID */		   
	   switch (format)
	   {
	   case (FLEXCAN_EXTENDED):
	      /* Start CR# 1730 */
	      ID = (msg_obj_ptr->ID.ID & 0x1FFFFFFF);
	      break;
	   case (FLEXCAN_STANDARD):
	      /* ID[28-18] */
		  ID = ((msg_obj_ptr->ID.ID & 0x1FFC0000) >> 18);
	      break;
	   default:
	      return (FLEXCAN_MESSAGE_FORMAT_UNKNOWN);
	   } /* Endswitch */
   
	   if ( code == FLEXCAN_RX_MSG_BUFFER_FULL ) 
	   {
	      for ( i=0 ; i<DLC ; i++ ) 
	      {
		 	 pMailBox->data[i] = msg_obj_ptr->DATA[i];
	         msg_obj_ptr->DATA[i] = 0;
	      } /* Endfor */
	      
	      /* Reset the code */
	      msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCRTL_CODE);
	      
	      /* Set the code */
	      msg_obj_ptr->CONTROL |= (FLEXCAN_RX_MSG_BUFFER_EMPTY);
	      return_code = FLEXCAN_OK;
	   } 
	   else if ( code == FLEXCAN_RX_MSG_BUFFER_OVERRUN ) 
	   {
	      for ( i=0 ;i<DLC ; i++ ) 
	      {
			 pMailBox->data[i] = msg_obj_ptr->DATA[i];
	         msg_obj_ptr->DATA[i] = 0;
	      } /* Endfor */
	
	      /* Reset the code */
	      msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCRTL_CODE);
	      
	      /* Set the code */
	      msg_obj_ptr->CONTROL |= (FLEXCAN_RX_MSG_BUFFER_EMPTY);
	      return_code = FLEXCAN_MESSAGE_OVERWRITTEN;
	   } 
	   else if ( code == FLEXCAN_RX_MSG_BUFFER_BUSY ) 
	   {
	      /* Shall wait till BUSY flag is cleared before read */
	       return_code = FLEXCAN_MB_BUSY_FAILED ;
	   } /* Endif */
	   
	   /* Store data length */
	   pMailBox->data_len = DLC;
	   
	   /* Store identifier */
	   pMailBox->identifier = ID;
	   
   }
       
   /* Unlock message buffer. only in polling mode */
   if(!int_enable)
   {
      temp = can_reg_ptr->TIMER;
   }
   
   return return_code;
} /* Endbody */	
   
/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Tx_message  
* Returned Value   : uint_32 
* Comments         :
*    This function sends a message from the specified mailbox 
*
*END*********************************************************************/
uint_32 FLEXCAN_Tx_message
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */ 
   uint_32 mailbox_number, 
   /* [IN] message ID */ 
   uint_32 identifier, 
   /* [IN] mailbox format (CAN_STANDARD OR CAN_EXTENDED) */ 
   uint_32 format,
   /* [IN] number of bytes to write to the mailbox (0 to 8) */  
   uint_32 data_len_code, 
   /* [IN] Location to store the data from the mailbox */
   vpointer data
)
{ /* Body */

   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr;
   uint_32                 i;
   vuint_32				   temp16;
   vuint_32                code;			   
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }
   
   if ( (1 > data_len_code) || (8 < data_len_code) )
   {
      return (FLEXCAN_DATA_SIZE_ERROR);
   }
   
   /*
   ** Check Control code
   */
   code = msg_obj_ptr->CONTROL & FLEXCAN_MSGCRTL_CODE;
   if( (code != FLEXCAN_TX_MSG_BUFFER_NOT_ACTIVE) &&
       (code != FLEXCAN_RX_MSG_BUFFER_NOT_ACTIVE)
    ) 
   {
      /* MB is active, should abort it somewhere after return.*/
      return (FLEXCAN_MB_NOT_EMPTY_FAILED);
   }
   /* Deactivate the code */
   msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCRTL_CODE);
 
   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, mailbox_number);   
     
   /* Set the ID according the format structure */
   if ( format == FLEXCAN_EXTENDED ) 
   {
      /* Set IDE */
	  msg_obj_ptr->CONTROL |= FLEXCAN_MSGCTRL_IDE;

	  /* Set SRR bit */
	  msg_obj_ptr->CONTROL |= FLEXCAN_MSGCTRL_SRR;
   	  
   	  /* Start CR# 1730 */
   	  msg_obj_ptr->ID.ID = FLEXCAN_EXTENDED_ID_MASK(identifier);
   } 
   else if( format == FLEXCAN_STANDARD )
   {
	  /* make sure IDE and SRR are not set */
	  msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCTRL_IDE | FLEXCAN_MSGCTRL_SRR);

	  /* ID[28-18] */
      msg_obj_ptr->ID.ID = FLEXCAN_STANDARD_ID_MASK(identifier);
   }
   else
   {
      return (FLEXCAN_MESSAGE_FORMAT_UNKNOWN);
   }/* Endif */
      
   /*
   ** Copy user's buffer into mailbox data area
   */
   for ( i=0 ; i < data_len_code ; i++ ) 
   {
	  msg_obj_ptr->DATA[i] = ((uchar_ptr)data)[i];
   } /* Endfor */
   
   data_len_code <<= 16;
   /*
   ** Set data up for send 
   ** send is automatic when remote message is received
   ** send when message buffer is not active
   */
   if( (code ) == FLEXCAN_TX_MSG_BUFFER_NOT_ACTIVE ) 
   {
      /* Activate code for transmit once */
      msg_obj_ptr->CONTROL |= (FLEXCAN_MESSAGE_TRANSMIT_ONCE | data_len_code);
   }

   /* Update data to respond to remote request frame */
   if( ((code ) == FLEXCAN_MESSAGE_TRANSMIT_RESPONSE_ONLY) ||
       ((code ) == FLEXCAN_MESSAGE_TRANSMIT_RESPONSE) )
   {
      /* Activate  code for respond to remote request */
      msg_obj_ptr->CONTROL |= (FLEXCAN_MESSAGE_TRANSMIT_RESPONSE | data_len_code);
   }      
   
   return (FLEXCAN_OK);
} /* Endbody */

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Read  
* Returned Value   : uint_32 
* Comments         :
*    This function reads the contents of the specified CAN memory.
*
*END*********************************************************************/
uint_32 FLEXCAN_Read
( 
   /* [IN] FlexCAN device number */
    uint_8 dev_num,
   /* [IN] offset from the base address of the device */
	uint_32         offset,
   /* [IN/OUT] vpointer to location of where the data is to be read to */
	uint_32_ptr     data_ptr
)
{ /* Body */

   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   uint_32                                  read_ptr;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   read_ptr = (vuint_32)can_reg_ptr + offset;
   *data_ptr = (uint_32) *(uchar_ptr)read_ptr;

   return (FLEXCAN_OK);

} /* Endbody */	

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Write  
* Returned Value   : uint_32 
* Comments         :
*    This function writes the specified value to the specified CAN 
*    memory.
*
*END*********************************************************************/
uint_32 FLEXCAN_Write
( 
   /* [IN] FlexCAN device number */
    uint_8 dev_num,
   /* [IN] offset from the base address of the device */ 
	uint_32 	offset, 
   /* Data to be written at the start of the offset */
	uint_32     value 
)
{ /* Body */

   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   vuint_32                                  write_ptr;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   write_ptr = (uint_32)can_reg_ptr + offset;
   *(uchar_ptr)write_ptr = (uchar)value;

   return (FLEXCAN_OK);

} /* Endbody */

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Get_status  
* Returned Value   : uint_32 
* Comments         :
*    This function gets the CAN chip in the specified state.
*
*END*********************************************************************/
uint_32 FLEXCAN_Get_status
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN/OUT] vpointer to Error/Status registry */
   uint_32_ptr can_status 
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   *can_status = can_reg_ptr->ESR1;

   return(FLEXCAN_OK);
} /* Endbody */	

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_update_message
* Returned Value   : uint_32 
* Comments         :
*    This function updates data as a response to a remote frame.
*
*END*********************************************************************/
uint_32 FLEXCAN_Update_message
( 
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] vpointer to data */
   vpointer data_ptr,
   /* [IN] number of bytes to write to the mailbox (0 to 8) */
   uint_32 data_len_code,
   /* [IN] mailbox format (CAN_STANDARD OR CAN_EXTENDED) */ 
   uint_32 format,
   /* [IN] mailbox number */
   uint_32 mailbox_number
)
{ /* Body */
   volatile FLEXCAN_REG_STRUCT_PTR          can_reg_ptr;
   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr;
   uint_32   i;
   uchar_ptr data_array;
   vuint_32  code;
   
   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }
   
   if ( (1 > data_len_code) || (8 < data_len_code) )
   {
      return (FLEXCAN_DATA_SIZE_ERROR);
   }
   
   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, mailbox_number);   
	
   /* check for RTR bit */
   code = msg_obj_ptr->CONTROL;
   if( (code & FLEXCAN_MSGCTRL_RTR) == FLEXCAN_MSGCTRL_RTR)
      return FLEXCAN_RTR_NOT_SET;

   /* check if mailbox is set for transmit */
   if( (code & FLEXCAN_MSGCRTL_CODE) == FLEXCAN_TX_MSG_BUFFER_NOT_ACTIVE)
   {
      /* Reset the code */
      msg_obj_ptr->CONTROL &= ~(FLEXCAN_MSGCRTL_CODE);

      /* store data */
      data_array = (uchar_ptr) data_ptr;
      for(i=0; i<data_len_code; i++)
      {
	     msg_obj_ptr->DATA[i] = data_array[i];
	  }
        
      /* 
      ** Data frame to be transmitted only as a response to 
      ** a remote frame. code 0b1010
      */
      msg_obj_ptr->CONTROL |= (FLEXCAN_MESSAGE_TRANSMIT_RESPONSE | (data_len_code<<16));
    }
    else
        return FLEXCAN_INVALID_MAILBOX;

   return FLEXCAN_OK;

}


/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_int_enable
* Returned Value   : uint_32 
* Comments         :
*    This function enables interrupt for requested mailbox 
*
*END*********************************************************************/
uint_32 FLEXCAN_Int_enable
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number
)
{
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   volatile PSP_INTERRUPT_TABLE_INDEX     index;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }
   
   if(mailbox_number >= 32)
   {
   		can_reg_ptr->IMASK2 |= (1 << (mailbox_number-32));
   }
   else
   {
	   /* IMASK, unmask the message buffer */
	   can_reg_ptr->IMASK1  |= (0x1 << mailbox_number);
   }

   return( FLEXCAN_OK );
}

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Error_int_enable
* Returned Value   : uint_32 
* Comments         :
*    This function unmasks (enables) error, wake up & Bus off interrupts.
*
*
*END*********************************************************************/
uint_32 FLEXCAN_Error_int_enable
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num
)
{
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   volatile PSP_INTERRUPT_TABLE_INDEX     index;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

				    
    /* BOFFMSK = 0x1, ERRMSK = 0x1 */
   can_reg_ptr->CANCTRL |= (FLEXCAN_CANCTRL_BOFFMSK | FLEXCAN_CANCTRL_ERRMSK);

   return ( FLEXCAN_OK );
}

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_int_disable
* Returned Value   : uint_32 
* Comments         :
*    This function masks (disables) interrupt for requested mailbox 
*
*END*********************************************************************/
uint_32 FLEXCAN_Int_disable
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number
)
{
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   volatile PSP_INTERRUPT_TABLE_INDEX     index;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

   if (  mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }

   if( mailbox_number >= 32)
   {
	   can_reg_ptr->IMASK2  &= ~(0x1 << (mailbox_number-32));   	
   }
   else
   {
	   can_reg_ptr->IMASK1  &= ~(0x1 << mailbox_number);
   }
      
   return (FLEXCAN_OK);
}

/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Error_int_disable
* Returned Value   : uint_32 
* Comments         :
*    This function masks (disables) error, wake up & Bus off interrupts 
*
*END*********************************************************************/
uint_32 FLEXCAN_Error_int_disable
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num
)
{
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   volatile PSP_INTERRUPT_TABLE_INDEX     index;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }

    /* BOFFMSK = 0x1, ERRMSK = 0x1 */
   can_reg_ptr->CANCTRL &= ~(FLEXCAN_CANCTRL_BOFFMSK | FLEXCAN_CANCTRL_ERRMSK);


   return ( FLEXCAN_OK );
}
 /* Endbody */


uint_32 FLEXCAN_EnterCritical(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number
   )
{
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   uint_32  mask;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }

   if(mailbox_number < 32)
   {
   	   // Disable MB interrupt
   	   mask =  can_reg_ptr->IMASK1;
	   can_reg_ptr->IMASK1  = 0;
   }
   else
   {
   	  mask = can_reg_ptr->IMASK2;
   	  can_reg_ptr->IMASK2 = 0;
   }
   return (mask);
}



void FLEXCAN_ExitCritical(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number,
   /* [IN] mask */
   uint_32 mask
   )
{
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return ;
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return;
   }

   if(mailbox_number < 32)
   {
   	   // Restore MB interrupt
   	  can_reg_ptr->IMASK1 = mask;
   }
   else
   {
   		// 
   		can_reg_ptr->IMASK2 = mask;
   }

}


uint_32	FLEXCAN_Check_TxMB_State(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number
   )
{
	uint_32	code;

   volatile FLEXCAN_MSG_OBJECT_STRUCT_PTR   msg_obj_ptr; 
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   uint_32 state = FLEXCAN_OK;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }

   msg_obj_ptr = (FLEXCAN_MSG_OBJECT_STRUCT_PTR)FLEXCAN_Get_msg_object(dev_num, 
                                                          mailbox_number);
   /* get control word */
   code = (msg_obj_ptr->CONTROL & FLEXCAN_MSGCRTL_CODE);
   
   /* check control word */
   if( code == FLEXCAN_TX_MB_ABORT)
   {
   		state = FLEXCAN_MB_ABORTED;
   }
   else if (code == FLEXCAN_TX_MSG_BUFFER_NOT_ACTIVE)
   {
   		state = FLEXCAN_MB_TRANSMITTED;
   }
   else 
   {
   		state = FLEXCAN_MESSAGE_BUSY;
   }
   return (state);
}


/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Is_MB_Done
* Returned Value   : uint_32 
* Comments         :
*    This function check mailbox status and return TRUE if the mailbox 
* IFLAG is set, FALSE otherwise.
*
*END*********************************************************************/
boolean FLEXCAN_Is_MB_Done
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_16 mailbox_number
)
{
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   volatile PSP_INTERRUPT_TABLE_INDEX     index;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FALSE);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FALSE);
   }
   
   if(mailbox_number >= 32)
   {
   		if(!(can_reg_ptr->IFLAG2 & (1 << (mailbox_number-32))))
   		{
   			return (FALSE);
   		}
   		
   }
   else
   {
	   if(!(can_reg_ptr->IFLAG1 & (0x1 << mailbox_number)))
	   {
	   		return (FALSE);
	   }
   }

   return( TRUE );
}


/*FUNCTION****************************************************************
* 
* Function Name    : FLEXCAN_Clear_MB_Flag
* Returned Value   : uint_32 
* Comments         :
*    This function clears the corresponding IFLAG for the mailbox
*END*********************************************************************/
uint_32 FLEXCAN_Clear_MB_Flag
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_16 mailbox_number
)
{
   volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
   volatile PSP_INTERRUPT_TABLE_INDEX     index;

   can_reg_ptr = _bsp_get_flexcan_base_address (dev_num);
   if (NULL == can_reg_ptr)  
   {
      return (FLEXCAN_INVALID_ADDRESS);
   }
   
   if ( mailbox_number > FLEXCAN_CANMCR_MAXMB (NUMBER_OF_MB-1) ) 
   {
      return (FLEXCAN_INVALID_MAILBOX);
   }
   
   if(mailbox_number >= 32)
   {
   		can_reg_ptr->IFLAG2 = (1 << (mailbox_number-32));  		
   }
   else
   {
	    can_reg_ptr->IFLAG1 = (0x1 << mailbox_number);

   }
   return( FLEXCAN_OK );
}


uint_32 _flexcan_int_init
   (
      // [IN} Interrupt number
      PSP_INTERRUPT_TABLE_INDEX irq,

      // [IN} Interrupt priority level
      _int_level                level,

      // [IN} Interrupt sub-priority level within priority
      _int_priority             sublevel,

      // [IN} Unmask the interrupt now?
      boolean                   unmask
   )
{ /* Body */


   return 0;

} /* Endbody */
