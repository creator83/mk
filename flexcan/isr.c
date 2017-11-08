/******************************************************************************
* File:    isr.c
*
* Purpose: Define custom interrupt service routines (ISRs).
******************************************************************************/
#include "vectors.h"
#include "common.h"
#include "isr.h"
#include "p2io_map.h"
#include "kinetis_flexcan.h"
#include <stdio.h>

/* Globals */
extern volatile uint32_t frames_intr_count,
				frames_done,
				fifo_warning,
				fifo_overflow,
				ref_frame_count,
				core_fault_occured,
				mb_intr,
				tx_warning_and_error_intr_occured ,
				bit1_error_occured,
				bit0_error_occured,
				cs[17],
				id[17],
				data0[17],
				data1[17];
extern volatile uint32_t timer[4];
extern volatile uint32_t uiMatchValue[4];	
extern volatile uint16_t guiErrCount;	
extern volatile uint32_t guiMB_ISR_Count;	
extern vuint16_t	iRxMBISRcount,giRxFIFOISRCount,giRxFIFOWarningCount,giRxFIFOOverflowCount;
extern vuint16_t	gNoMBsAvail;
extern vuint32_t	giTimeStamp[NUMBER_OF_MB];
extern vuint8_t    acces_mode;	
				

/* Prototypes */
void FlexCAN1_bit0_error_ISR (void);	
void FlexCAN1_bit1_error_ISR (void);
void FlexCAN1_fifo_warning_ISR (void);
void FlexCAN1_individual_frame_ISR (void);
void FlexCAN1_fifo_overflow_ISR (void);
void FlexCAN0_Selfloop2_MB_ISR(void);			
  

void FlexCAN1_AccessMode_RxFIFO_ISR(void)
{
	int16_t iMB;
	int16_t iMatchHit;

	// check RxFIFO interrupt flag
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)
	{
		 // read FIFO
		 cs[0] = FLEXCAN1_MBn_CS(0);
		 id[0]= FLEXCAN1_MBn_ID(0);
		 data0[0] = FLEXCAN1_MBn_WORD0(0);
		 data1[0] = FLEXCAN1_MBn_WORD1(0);	
		 
		 // read RXFIR
		 iMatchHit = FLEXCAN1_RXFIR & 0x1FF;
		 
		 // clear flag
		 FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;
		 guiMB_ISR_Count++;
		 printf("guiMB_ISR_Count =%d: RxFIFO: received a message ID=%#08.8x, match hit =%d\n",guiMB_ISR_Count,id[0],iMatchHit);
	}
	// Check MB interrupt flags per table Rx FIFO Filters
	for(iMB = 8; iMB < 15; iMB++)
	{
		if( FLEXCAN1_IFLAG1 & (1L<<iMB))
		{
			// read message buffer
			 cs[1] = FLEXCAN1_MBn_CS(iMB);
			 id[1]= FLEXCAN1_MBn_ID(iMB);
			 data0[1] = FLEXCAN1_MBn_WORD0(iMB);
			 data1[1] = FLEXCAN1_MBn_WORD1(iMB);
			 timer[1] = FLEXCAN1_TIMER;	// unlock it	
			 
			 FLEXCAN1_IFLAG1 = 1L<<iMB;
			 guiMB_ISR_Count++;					 
			 printf("guiMB_ISR_Count =%d: MB%d received a message ID = %#08.8x,CS=%#08.8x\n",guiMB_ISR_Count,iMB,id[1],cs[1]);
		}
	}
	// Send another frame
	if(FLEXCAN1_IFLAG1 & (1<<15))
	{
		// clear flag
		FLEXCAN1_IFLAG1 = (1<<15);
		
		if(guiMB_ISR_Count < 16)
		{
		  	FLEXCAN1_MBn_CS(15) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		  	FLEXCAN1_MBn_ID(15) = ((guiMB_ISR_Count+1)<<18);
		  	FLEXCAN1_MBn_WORD0(15) = 0x12345678+guiMB_ISR_Count;
		  	FLEXCAN1_MBn_WORD1(15) = 0x9ABCDEF0;
		  	FLEXCAN1_MBn_CS(15) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
		  						 | FLEXCAN_MB_CS_SRR
		 						 | FLEXCAN_MB_CS_LENGTH(8);	
		}	
	}	
}


void FlexCAN0_AccessMode_MB_ISR(void)
{
	extern uint32_t GetControlRegister(void);
	static uint8_t	uiMB0count = 0;
	
	printf("In FlexCAN0_SelfLooop01_MB_ISR,CONTROL = %#08.8x\n",GetControlRegister());
	// Check rx MBs first
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF1I)
	{
	#ifndef	TEST_MESSAGE_QUEUE		
		if((uiMB0count==1))
		{
		 /* Read received frame if queue not enabled and 2nd reception*/
		 cs[1] = FLEXCAN0_MBn_CS(1);
		 id[1]= FLEXCAN0_MBn_ID(1);
		 data0[1] = FLEXCAN0_MBn_WORD0(1);
		 data1[1] = FLEXCAN0_MBn_WORD1(1);
		 timer[1] = FLEXCAN0_TIMER;	// unlock it
		}
	#endif
		// For queue is enabled, delay read of receive frame so as to make MB1 as active but not free-to-receive
		// clear the flags
		FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF1I;
		guiMB_ISR_Count++;
		uiMB0count++;
		printf("MB1 received a message\n");
	}
		
	/* If message queue is enabled, wait for queue elements done */ 
#ifdef	TEST_MESSAGE_QUEUE	
	//if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ )
	{	
	 	if((FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF3I))
	 	{
			 cs[2] = FLEXCAN0_MBn_CS(3);
			 id[2]= FLEXCAN0_MBn_ID(3);
			 data0[2] = FLEXCAN0_MBn_WORD0(3);
			 data1[2] = FLEXCAN0_MBn_WORD1(3);
			 timer[2] = FLEXCAN0_TIMER;	 
			 
			// clear the flags
			FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF3I; 

			printf("MB3 received a message\n");		
				
			// Check received message
			 if(data0[2] != uiMatchValue[2])
			 {
			 	printf("Error: Data Mismatch in MB3 word0,received = %#08.8x,expected = %#08.8x!\n",data0[2],uiMatchValue[2]);
			 	guiErrCount++;
			 }
			 
			 if(data1[2] != uiMatchValue[3])
			 {
			 	printf("Error: Data Mismatch in MB3 word1received = %#08.8x,expected = %#08.8x!\n",data1[2],uiMatchValue[3]);
			 	guiErrCount++;
			 }
		 	 if(((id[2] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	printf("Error: MB3 got wrong ID = %#08.8x,expected = %#08.8x\n",id[2],FLEXCAN_MB_ID_IDSTD(0xE));
			 	guiErrCount++;
			 }	
			 guiMB_ISR_Count++;	 			 		
	 	}			
	}
#endif	
	
	// Check tx MBs
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF0I)
	{
		FLEXCAN0_IFLAG1 =  FLEXCAN_IFLAG1_BUF0I;		// clear tx MB0 flag
		guiMB_ISR_Count++;
	}
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF2I)
	{
		FLEXCAN0_IFLAG1 =  FLEXCAN_IFLAG1_BUF2I;		// clear tx MB2 flag
		guiMB_ISR_Count++;
	}
#ifndef	TEST_MESSAGE_QUEUE	
	//if(!(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ))
	{	
		// For queue not enabled,check the MB1 status for overrun
		if(uiMB0count == 2)
		{ 
			 if(((cs[1]>>24) & 0x0F) != FLEXCAN_MB_CODE_RX_OVERRUN)
			 {
				printf("Error: CS word = %#08.8x\n,expected overrun status!\n", cs[1]);
			 }
			 if(data0[1] != uiMatchValue[0])
			 {
			 	printf("Error: Data Mismatch in MB1 word0,received = %#08.8x,expected = %#08.8x!\n",data0[1],uiMatchValue[0]);
			 	guiErrCount++;
			 }
			 
			 if(data1[1] != uiMatchValue[1])
			 {
			 	printf("Error: Data Mismatch in MB1 word1,received = %#08.8x,expected = %#08.8x!\n",data1[1],uiMatchValue[1]);
			 	guiErrCount++;
			 }
			 
			 if(((id[1] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	printf("Error: MB1 got wrong ID = %#08.8x\n",id[1]);
			 	guiErrCount++;
			 }
			 
			 printf("MB1: timer  = %#08.8x\n\n", timer[1]);
		}	 
	}
#else
	{	// For queue enabled, check MB1 for FULL status
		if(uiMB0count==1)
		{
			 cs[1] = FLEXCAN0_MBn_CS(1);
			 id[1]= FLEXCAN0_MBn_ID(1);
			 data0[1] = FLEXCAN0_MBn_WORD0(1);
			 data1[1] = FLEXCAN0_MBn_WORD1(1);
			 timer[1] = FLEXCAN0_TIMER;	// unlock it
			
			 if(((cs[1]>>24) & 0x0F) != FLEXCAN_MB_CODE_RX_FULL)
			 {
				printf("Error: CS word = %#08.8x\n,expected FULL status!\n", cs[1]);
			 }
			 if(data0[1] != uiMatchValue[0])
			 {
			 	printf("Error: Data Mismatch in MB1 word0,received = %#08.8x,expected = %#08.8x!\n",data0[1],uiMatchValue[0]);
			 	guiErrCount++;
			 }
			 
			 if(data1[1] != uiMatchValue[1])
			 {
			 	printf("Error: Data Mismatch in MB1 word1,received = %#08.8x,expected = %#08.8x!\n",data1[1],uiMatchValue[1]);
			 	guiErrCount++;
			 }
			 
			 if(((id[1] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	printf("Error: MB1 got wrong ID = %#08.8x\n",id[1]);
			 	guiErrCount++;
			 }		
		}
	}
#endif
}

void HardFault_ISR(void)
{
	extern uint32_t GetControlRegister(void);
	
	if(!acces_mode)
	{
		guiErrCount++;
	}
	guiMB_ISR_Count++;
	printf("Enter HardFault ISR,CONTROL = %#08.8x\n",GetControlRegister());
}

void FlexCAN1_IMEU_ISR(void)
{
	printf("In FlexCAN1 IMEU ISR: FLEXCAN1_ESR2 = %#08.8x\n",FLEXCAN1_ESR2);
	FLEXCAN1_ESR2 = FLEXCAN_ESR2_IMEUF;
}

void FlexCAN0_IMEU_ISR(void)
{
	printf("In FlexCAN0 IMEU ISR: FLEXCAN0_ESR2 = %#08.8x\n",FLEXCAN0_ESR2);
	FLEXCAN0_ESR2 = FLEXCAN_ESR2_IMEUF;
}

void FlexCAN1_LostReceiveFrame_ISR(void)
{
	printf("In FlexCAN1 Lost Receive ISR: FLEXCAN1_ESR2 = %#08.8x\n",FLEXCAN1_ESR2);
	printf("Lost Rx Frames Register = %#08.8x\n",FLEXCAN1_LRFR);
	if(FLEXCAN1_ESR2 & FLEXCAN_ESR2_LOSTRLF)
	{
		FLEXCAN1_ESR2 = FLEXCAN_ESR2_LOSTRLF;
	}
	if(FLEXCAN1_ESR2 & FLEXCAN_ESR2_LOSTRMF)
	{
		FLEXCAN1_ESR2 = FLEXCAN_ESR2_LOSTRMF;
	}
}

void FlexCAN0_LostReceiveFrame_ISR(void)
{
	printf("In FlexCAN0 Lost Receive ISR: FLEXCAN0_ESR2 = %#08.8x\n",FLEXCAN0_ESR2);
	printf("Lost Rx Frames Register = %#08.8x\n",FLEXCAN0_LRFR);
	if(FLEXCAN0_ESR2 & FLEXCAN_ESR2_LOSTRLF)
	{
		FLEXCAN0_ESR2 = FLEXCAN_ESR2_LOSTRLF;
	}
	if(FLEXCAN0_ESR2 & FLEXCAN_ESR2_LOSTRMF)
	{
		FLEXCAN0_ESR2 = FLEXCAN_ESR2_LOSTRMF;
	}
}

void FlexCAN0_Reconfig_FIFO_ISR(void)
{
	int16_t iMB;
	// NOTE: only care bout rx MBs. For tx MBs, use polling mode
	for(iMB = gNoMBsAvail; iMB < (NUMBER_OF_MB-1); iMB++)
	{
		if( FLEXCAN0_IFLAG1 & (1L<<iMB))
		{
			//if(!(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ))
			{
				// read message buffer
				 cs[1] = FLEXCAN0_MBn_CS(iMB);
				 id[1]= FLEXCAN0_MBn_ID(iMB);
				 data0[1] = FLEXCAN0_MBn_WORD0(iMB);
				 data1[1] = FLEXCAN0_MBn_WORD1(iMB);
				 timer[1] = FLEXCAN0_TIMER;	// unlock it	
				 iRxMBISRcount++;	// increase Rx MB ISR count
				 
			     printf("MB %d received message id = %#08.8x,c/s = %#08.8x,WORD0 = %#08.8x\n",iMB,id[1],cs[1],data0[1]);
			}
			// clear buffer flag
			FLEXCAN0_IFLAG1 = (1L<<iMB);
			guiMB_ISR_Count++;		
		}
	}
}

void FlexCAN1_Reconfig_FIFO_ISR(void)
{
	FlexCAN1_Reconfig_MB_ISR();
}

void FlexCAN0_Reconfig_MB_ISR(void)
{
	int16_t iMB;
	uint32_t data;	
	
        printf("Enter FlexCAN0_Reconfig_MB_ISR,FLEXCAN0_IFLAG1=%#08.8x\n",FLEXCAN0_IFLAG1);
	for(iMB = 0; iMB < NUMBER_OF_MB; iMB++)
	{
		if( FLEXCAN0_IFLAG1 & (1L<<iMB))
		{
			// check if it is a read MB
			if(iMB <= FLEXCAN_RX_MB_END)
			{
				//if(!(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ))
				{
					// read message buffer
					 cs[1] = FLEXCAN0_MBn_CS(iMB);
					 id[1]= FLEXCAN0_MBn_ID(iMB);
					 data0[1] = FLEXCAN0_MBn_WORD0(iMB);
					 data1[1] = FLEXCAN0_MBn_WORD1(iMB);
					 timer[1] = FLEXCAN0_TIMER;	// unlock it	
					 iRxMBISRcount++;	// increase Rx MB ISR count
					 
				     printf("MB %d received message id = %#08.8x,c/s = %#08.8x,WORD0 = %#08.8x\n",iMB,id[1],cs[1],data0[1]);
				}
				// clear buffer flag
				FLEXCAN0_IFLAG1 = (1L<<iMB);
				guiMB_ISR_Count++;		
			}
                        else
                        {
                            FLEXCAN0_IFLAG1 = (1L<<iMB);
                            printf("MB%d transmitted a message\n",iMB);
                        }
		}                
	}
}


void FlexCAN1_Reconfig_MB_ISR(void)
{
	int16_t iMB;
	uint32_t data;	
	
	for(iMB = 0; iMB < NUMBER_OF_MB; iMB++)
	{
		if( FLEXCAN1_IFLAG1 & (1L<<iMB))
		{
			// check if it is a read MB
			if(iMB <= FLEXCAN_RX_MB_END)
			{
				//if(!(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ))
				{
					// read message buffer
					 cs[1] = FLEXCAN1_MBn_CS(iMB);
					 id[1]= FLEXCAN1_MBn_ID(iMB);
					 data0[1] = FLEXCAN1_MBn_WORD0(iMB);
					 data1[1] = FLEXCAN1_MBn_WORD1(iMB);
					 timer[1] = FLEXCAN1_TIMER;	// unlock it	
					 iRxMBISRcount++;	// increase Rx MB ISR count
					 
				     printf("MB %d received message id = %#08.8x,c/s = %#08.8x,WORD0 = %#08.8x\n",iMB,id[1],cs[1],data0[1]);
				}
				// clear buffer flag
				FLEXCAN1_IFLAG1 = (1L<<iMB);
				guiMB_ISR_Count++;		
			}
		}
	}
}

void PIT0_CANTx_ISR(void)
{
        PIT_CH0_TCTRL = 0x0; 
	PIT_CH0_TFLG  = PIT_TFLG_TIF;		
	printf("In PIT0_CANTx_ISR: Sending a frame via MB1...\n");
	// Send another frame via MB1
	FLEXCAN1_MBn_CS(1) = 0x00000000;
	FLEXCAN1_MBn_ID(1) = 0x01FFA5A5+1;
	FLEXCAN1_MBn_WORD0(1) = 0x12345678+1;
	FLEXCAN1_MBn_WORD1(1) = 0x9ABCDEF0+1;
	FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
							 | FLEXCAN_MB_CS_LENGTH(8)
							 | FLEXCAN_MB_CS_IDE
							 | FLEXCAN_MB_CS_SRR;	
  //     PIT_CH0_TCTRL = 0x0; 							 
}

void PIT0_ISR(void)
{
  // Disable Timer and clear  Flag
   PIT_CH0_TCTRL = 0x0; 
   PIT_CH0_TFLG  = PIT_TFLG_TIF;	
   printf("PIT0 isr occured!\n");
}


void FTM1_ISR(void)
{
	if(FTM1C0SC_CH0F)
	{
		//printf("Enter FTM1 ISR\n");
		TIMER_CH0_ISR();
	}
	else
	{
		printf("Error: other flextimer1 interrupt occurs than channel interrupt!\n");
	}
}


void FlexCAN0_Error_ISR(void)
{
	// check error flags
	uint32_t status;
	
	status = FLEXCAN0_ESR1;
	
	if( status  & FLEXCAN_ESR_ERR_INT)
	{
		guiMB_ISR_Count++;
		printf("FlexCAN0 detected errors!\n");
		
		if(status & FLEXCAN_ESR_BIT0_ERR)
		{
			printf("BIT0 error occured!\n");
		}	
		if(status & FLEXCAN_ESR_BIT1_ERR)
		{
			printf("BIT1 error occured!\n");
		}
		if(status & FLEXCAN_ESR_STF_ERR)
		{
			printf("Stuffing error occured!\n");
		}
		if(status & FLEXCAN_ESR_FRM_ERR)
		{
			printf("Form error occured!\n");
		}
		if(status & FLEXCAN_ESR_CRC_ERR)
		{
			printf("CRC error occured!\n");
		}
		if(status & FLEXCAN_ESR_ACK_ERR)
		{
			printf("Ack error occured!\n");
		}
		
		// clear the error flag
		FLEXCAN0_ESR1 = status;		
	}
	else
	{
		printf("Error: no error occured!\n");
		guiErrCount++;
	}
}

void FlexCAN1_MB_INT_ISR(void)
{
	int16_t iMB;
	// Check rx MB
	for(iMB = 0; iMB < NUMBER_OF_MB-1; iMB++)
	{
            if(FLEXCAN1_IMASK1 & (1<<iMB))
            {
		if(FLEXCAN1_IFLAG1 & (1<<iMB))
		{
                         // read message buffer
                         cs[1] = FLEXCAN1_MBn_CS(iMB);
                         id[1]= FLEXCAN1_MBn_ID(iMB);
                         data0[1] = FLEXCAN1_MBn_WORD0(iMB);
                         data1[1] = FLEXCAN1_MBn_WORD1(iMB);
                         timer[1] = FLEXCAN1_TIMER;	// unlock it	                             
			 // clear flag
			 FLEXCAN1_IFLAG1 = (1<<iMB);
			 guiMB_ISR_Count++;	
                         printf("MB %d received message id = %#08.8x,c/s = %#08.8x,WORD0 = %#08.8x,WORD1 = %#08.8x\n",
                                iMB,id[1],cs[1],data0[1],data1[1]);                 	
		}
            }
	}	
        // Check tx MB
        if((FLEXCAN1_IMASK1 & (1<<iMB)) && (FLEXCAN1_IFLAG1 & (1<<iMB)))
        {
                 // clear flag
                 FLEXCAN1_IFLAG1 = (1<<iMB);
                 guiMB_ISR_Count++;	
                 printf("MB%d is transmitted successfully\n",iMB);	 
        }       
}

void FlexCAN1_Error_ISR(void)
{
	// check error flags
	uint32_t status;
	
	status = FLEXCAN1_ESR1;
	
	if( status  & FLEXCAN_ESR_ERR_INT)
	{
		printf("FlexCAN1 detected errors!\n");
		if(status & FLEXCAN_ESR_BIT0_ERR)
		{
			printf("BIT0 error occured!\n");
		}	
		if(status & FLEXCAN_ESR_BIT1_ERR)
		{
			printf("BIT1 error occured!\n");
		}
		if(status & FLEXCAN_ESR_STF_ERR)
		{
			printf("Stuffing error occured!\n");
		}
		if(status & FLEXCAN_ESR_FRM_ERR)
		{
			printf("Form error occured!\n");
		}
		if(status & FLEXCAN_ESR_CRC_ERR)
		{
			printf("CRC error occured!\n");
		}
		if(status & FLEXCAN_ESR_ACK_ERR)
		{
			printf("Ack error occured!\n");
		}
		
		// clear the error flag
		FLEXCAN1_ESR1 = status;		
	}
	else
	{
		printf("Error: no error occured!\n");
		guiErrCount++;
	}
}


void FlexCAN0_BusOff_MB_ISR(void)
{
	int16_t iMB;
	// Check tx MB
	for(iMB = 0; iMB < NUMBER_OF_MB; iMB++)
	{
		if(FLEXCAN0_IFLAG1 & (1<<iMB))
		{
			 // clear flag
			 FLEXCAN0_IFLAG1 = (1<<iMB);
			 guiMB_ISR_Count++;	
			 printf("MB%d is completed successfully\n",iMB);	 
		}
	}		
}


void FlexCAN1_TxAbort_MB_ISR(void)
{
	int16_t iMB;
	// Check rx MB
	if(FLEXCAN1_IFLAG1 & (1<<0))
	{
		// read message buffer
		 cs[0] = FLEXCAN1_MBn_CS(0);
		 id[0]= FLEXCAN1_MBn_ID(0);
		 data0[0] = FLEXCAN1_MBn_WORD0(0);
		 data1[0] = FLEXCAN1_MBn_WORD1(0);
		 timer[0] = FLEXCAN1_TIMER;	// unlock it	
		 
		 // clear flag
		 FLEXCAN1_IFLAG1 = (1<<0);
		 guiMB_ISR_Count++;
		 printf("MB0 received a message ID=%#08.8x,C/S=%#08.8x\n",id[0],cs[0]);
	}
	
	// Check tx MB
	for(iMB = 1; iMB < NUMBER_OF_MB; iMB++)
	{
		if(FLEXCAN1_IFLAG1 & (1<<iMB))
		{
			// read message buffer
			 cs[1] = FLEXCAN1_MBn_CS(iMB);
			 // clear flag
			 FLEXCAN1_IFLAG1 = (1<<iMB);
			 guiMB_ISR_Count++;
			 printf("MB%d C/S=%#08.8x\n",iMB,cs[1]);
#if 0			 
			 if(FLEXCAN_get_code(cs[1]) == FLEXCAN_MB_CODE_TX_ABORT)
			 {
			 	printf("MB%d is aborted!\n",iMB);
			 }
			 else
			 {
			 	printf("MB%d is not NOT aborted!\n",iMB);
			 }
#endif			 
		}
	}	
}


void FlexCAN0_IndiMasking_MB_ISR(void)
{
	int16_t iMB;
	
	// Check rx MB
	for(iMB = 0; iMB < NUMBER_OF_MB/2; iMB++)
	{
		if(FLEXCAN0_IFLAG1 & (1<<iMB))
		{
			// read message buffer
			 cs[1] = FLEXCAN0_MBn_CS(iMB);
			 id[1]= FLEXCAN0_MBn_ID(iMB);
			 data0[1] = FLEXCAN0_MBn_WORD0(iMB);
			 data1[1] = FLEXCAN0_MBn_WORD1(iMB);
			 timer[1] = FLEXCAN0_TIMER;	// unlock it	
			 // clear flag
			 FLEXCAN0_IFLAG1 = (1<<iMB);
			 if( (id[1] != (0x1ABCDEF5+guiMB_ISR_Count)) ||
			 	(iMB != guiMB_ISR_Count) 
			 )
			 {
			 	 guiErrCount++;
				 printf("MB%d received unwanted message (ID=%#08.8x,C/S=%#08.8x)\n",iMB,id[1],cs[1]);			 	
			 }
			 guiMB_ISR_Count++;
			// printf("MB%d received a message: ID=%#08.8x,C/S=%#08.8x\n",iMB,id[1],cs[1]);
		}
	}		
}



void FlexCAN0_Priority_MB_ISR(void)
{
	int16_t iMB;
	
//	printf("FLEXCAN0_IFLAG1=%#08.8x\n");
	// Check tx MB
	for(iMB = 8; iMB < 16; iMB++)
	{
		if(FLEXCAN0_IFLAG1 & (1<<iMB))
		{
			 // Read C/S word to get timestamp
			 cs[iMB] = FLEXCAN0_MBn_CS(iMB);
			 giTimeStamp[iMB] = FLEXCAN_MB_CS_TIMESTAMP(cs[iMB]);
			 
			 // clear flag
			 FLEXCAN0_IFLAG1 = (1<<iMB);
			 guiMB_ISR_Count++;
		}
	}	
}

void FlexCAN1_Priority_MB_ISR(void)
{
	int16_t iMB;
	
//	printf("FLEXCAN1_IFLAG1=%#08.8x\n");
	// Check tx MB
	for(iMB = 8; iMB < 16; iMB++)
	{
		if(FLEXCAN1_IFLAG1 & (1<<iMB))
		{
			 // Read C/S word to get timestamp
			 cs[iMB] = FLEXCAN1_MBn_CS(iMB);
			 giTimeStamp[iMB] = FLEXCAN_MB_CS_TIMESTAMP(cs[iMB]);
			 
			 // clear flag
			 FLEXCAN1_IFLAG1 = (1<<iMB);
			 guiMB_ISR_Count++;
		}
	}	
}


void FlexCAN0_TSYNC_MB_ISR(void)
{
	int16_t iMB;
	int16_t iMatchHit;
	
	//printf("FLEXCAN0_IFLAG1=%#08.8x\n",FLEXCAN0_IFLAG1);
	// check RxFIFO interrupt flag
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)
	{
		 // read FIFO
		 cs[0] = FLEXCAN0_MBn_CS(0);
		 id[0]= FLEXCAN0_MBn_ID(0);
		 data0[0] = FLEXCAN0_MBn_WORD0(0);
		 data1[0] = FLEXCAN0_MBn_WORD1(0);	
		 
		 // read RXFIR
		 iMatchHit = FLEXCAN0_RXFIR & 0x1FF;
		 
		 // clear flag
		 FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;
		 guiMB_ISR_Count++;
		 if( (id[0] != ((guiMB_ISR_Count)<<18)) 	 
		 	|| (iMatchHit != (guiMB_ISR_Count-1))
		   )
		 {
		 	 guiErrCount++;
			 printf("guiMB_ISR_Count =%d: RxFIFO: received a message ID=%#08.8x, match hit =%d\n",guiMB_ISR_Count,id[0],iMatchHit);
		 }
		 giRxFIFOISRCount++;
	}
	// Check MB interrupt flags per table Rx FIFO Filters
	for(iMB = gNoMBsAvail; iMB < NUMBER_OF_MB-1; iMB++)
	{
		if( FLEXCAN0_IFLAG1 & (1L<<iMB))
		{
			
			// read message buffer
			 cs[1] = FLEXCAN0_MBn_CS(iMB);
			 id[1]= FLEXCAN0_MBn_ID(iMB);
			 data0[1] = FLEXCAN0_MBn_WORD0(iMB);
			 data1[1] = FLEXCAN0_MBn_WORD1(iMB);
			 timer[1] = FLEXCAN0_TIMER;	// unlock it	
			 
			if(iMB == gNoMBsAvail)
			{
				// check timer to see if it's reset
				timer[1] = FLEXCAN0_TIMER; 
				
				if(timer[1] > 5)
				{
					guiErrCount++;
					printf("Error: Timer (= %#08.8x) is not reset after receiving first MB\n",timer[1]); 
				}
			}
			 guiMB_ISR_Count++;		
			 	 			 	 
			 if(id[1] != ((guiMB_ISR_Count)<<18))
			 {
			 	guiErrCount++;
			 	printf("guiMB_ISR_Count =%d: MB%d received a message ID = %#08.8x,CS=%#08.8x\n",guiMB_ISR_Count,iMB,id[1],cs[1]);
			 }
			 FLEXCAN0_IFLAG1 = 1L<<iMB;

		}
	}	
}


void FlexCAN1_TSYNC_MB_ISR(void)
{
	int16_t iMB;
	int16_t iMatchHit;
	
	//printf("FLEXCAN1_IFLAG1=%#08.8x\n",FLEXCAN1_IFLAG1);
	// check RxFIFO interrupt flag
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)
	{
		 // read FIFO
		 cs[0] = FLEXCAN1_MBn_CS(0);
		 id[0]= FLEXCAN1_MBn_ID(0);
		 data0[0] = FLEXCAN1_MBn_WORD0(0);
		 data1[0] = FLEXCAN1_MBn_WORD1(0);	
		 
		 // read RXFIR
		 iMatchHit = FLEXCAN1_RXFIR & 0x1FF;
		 
		 // clear flag
		 FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;
		 guiMB_ISR_Count++;
		 if( (id[0] != ((guiMB_ISR_Count)<<18)) 	 
		 	|| (iMatchHit != (guiMB_ISR_Count-1))
		   )
		 {
		 	 guiErrCount++;
			 printf("guiMB_ISR_Count =%d: RxFIFO: received a message ID=%#08.8x, match hit =%d\n",guiMB_ISR_Count,id[0],iMatchHit);
		 }
		 giRxFIFOISRCount++;
	}
	// Check MB interrupt flags per table Rx FIFO Filters
	for(iMB = gNoMBsAvail; iMB < NUMBER_OF_MB-1; iMB++)
	{
		if( FLEXCAN1_IFLAG1 & (1L<<iMB))
		{
			
			// read message buffer
			 cs[1] = FLEXCAN1_MBn_CS(iMB);
			 id[1]= FLEXCAN1_MBn_ID(iMB);
			 data0[1] = FLEXCAN1_MBn_WORD0(iMB);
			 data1[1] = FLEXCAN1_MBn_WORD1(iMB);
			 timer[1] = FLEXCAN1_TIMER;	// unlock it	
			 
			if(iMB == gNoMBsAvail)
			{
				// check timer to see if it's reset
				timer[1] = FLEXCAN1_TIMER; 
				
				if(timer[1] > 5)
				{
					guiErrCount++;
					printf("Error: Timer (= %#08.8x) is not reset after receiving first MB\n",timer[1]); 
				}
			}
			 guiMB_ISR_Count++;		
			 	 			 	 
			 if(id[1] != ((guiMB_ISR_Count)<<18))
			 {
			 	guiErrCount++;
			 	printf("guiMB_ISR_Count =%d: MB%d received a message ID = %#08.8x,CS=%#08.8x\n",guiMB_ISR_Count,iMB,id[1],cs[1]);
			 }
			 FLEXCAN1_IFLAG1 = 1L<<iMB;

		}
	}	
}

void FlexCAN0_RxFIFO_INT_ISR(void)
{
	int16_t iMB;
	int16_t iMatchHit;
	
	//printf("FLEXCAN0_IFLAG1 = %#08.8x in FlexCAN0_RxFIFO_INT_ISR\n",FLEXCAN0_IFLAG1);
	#if 0
	// check RxFIFO interrupt flag
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)
	{
		#if 0
		 if(giRxFIFOISRCount<1)
		 {
			 // read FIFO
			 cs[0] = FLEXCAN0_MBn_CS(0);
			 id[0]= FLEXCAN0_MBn_ID(0);
			 data0[0] = FLEXCAN0_MBn_WORD0(0);
			 data1[0] = FLEXCAN0_MBn_WORD1(0);	
			 
			 printf("RxFIFO message received,ID = %#08.8x,data0 =%#08.8x,data1=%#08.8x!\n",id[0],data0[0],data1[0]);			 
		 }
		 else
		 {
		 	FLEXCAN0_IMASK1 &= ~FLEXCAN_IMASK1_BUF5M;	// disable fifo int 
		 }
		 // read RXFIR
		 iMatchHit = FLEXCAN0_RXFIR & 0x1FF;
		 
		 // clear flag to allow update of FIFO
		 FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;
		#endif
		 giRxFIFOISRCount++;
		 guiMB_ISR_Count++;
	}
	#endif
	// check warning flag
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF6I)
	{
		FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF6I;
		giRxFIFOWarningCount++;
		guiMB_ISR_Count++;
		printf("RxFIFO Warning Interrupt occurs!\n");
	}
	// check overflow flag
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF7I)
	{
		FLEXCAN0_IFLAG1 =  FLEXCAN_IFLAG1_BUF7I;
		giRxFIFOOverflowCount++;
		guiMB_ISR_Count++;
		printf("RxFIFO Overflow Interrupt occurs!\n");
	}	
}

void FlexCAN1_RxFIFO_INT_ISR(void)
{
	int16_t iMB;
	int16_t iMatchHit;
	
	// check warning flag
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF6I)
	{
		FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF6I;
		giRxFIFOWarningCount++;
		guiMB_ISR_Count++;
		printf("RxFIFO Warning Interrupt occurs!\n");		
	}
	// check overflow flag
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF7I)
	{
		FLEXCAN1_IFLAG1 =  FLEXCAN_IFLAG1_BUF7I;
		giRxFIFOOverflowCount++;
		guiMB_ISR_Count++;
		printf("RxFIFO Overflow Interrupt occurs!\n");
	}	
}

void FlexCAN0_RxFIFO_Filter_MB_ISR(void)
{
	int16_t iMB;
	int16_t iMatchHit;
//	printf("FLEXCAN0_IFLAG1 =%#08.8x\n",FLEXCAN0_IFLAG1);
	// check RxFIFO interrupt flag
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)
	{
		 // read FIFO
		 cs[0] = FLEXCAN0_MBn_CS(0);
		 id[0]= FLEXCAN0_MBn_ID(0);
		 data0[0] = FLEXCAN0_MBn_WORD0(0);
		 data1[0] = FLEXCAN0_MBn_WORD1(0);	
		 
		 // read RXFIR
		 iMatchHit = FLEXCAN0_RXFIR & 0x1FF;
                 printf("iMatchHit = %d\n",iMatchHit);
		 
		 // clear flag
		 FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;
		 guiMB_ISR_Count++;
#ifndef	TEST_RXFIFO_FILTER_FORMAT_C	 
		 if( (id[0] != ((guiMB_ISR_Count)<<18)) 
#else
		 if( (id[0] != ((guiMB_ISR_Count<<21))) 
#endif		 
		 	|| (iMatchHit != (guiMB_ISR_Count-1))
		   )
		 {
		 	 guiErrCount++;
			 printf("ERROR: guiMB_ISR_Count =%d: RxFIFO: received a message ID=%#08.8x, match hit =%d\n",guiMB_ISR_Count,id[0],iMatchHit);
		 }
		 giRxFIFOISRCount++;
	}
	// Check MB interrupt flags per table Rx FIFO Filters
	for(iMB = gNoMBsAvail; iMB < NUMBER_OF_MB-1; iMB++)
	{
		if( FLEXCAN0_IFLAG1 & (1L<<iMB))
		{
			// read message buffer
			 cs[1] = FLEXCAN0_MBn_CS(iMB);
			 id[1]= FLEXCAN0_MBn_ID(iMB);
			 data0[1] = FLEXCAN0_MBn_WORD0(iMB);
			 data1[1] = FLEXCAN0_MBn_WORD1(iMB);
			 timer[1] = FLEXCAN0_TIMER;	// unlock it	
			 
			 FLEXCAN0_IFLAG1 = 1L<<iMB;
			 guiMB_ISR_Count++;		
			 
#ifndef	TEST_RXFIFO_FILTER_FORMAT_C		 			 	 
			 if(id[1] != ((guiMB_ISR_Count)<<18))
#else
			 if(id[1] != ((guiMB_ISR_Count<<21)))
#endif			 
			 {
			 	guiErrCount++;
			 	printf("guiMB_ISR_Count =%d: MB%d received a message ID = %#08.8x,CS=%#08.8x\n",guiMB_ISR_Count,iMB,id[1],cs[1]);
			 }
		}
	}
// check warning flag
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF6I)
	{
		FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF6I;
		giRxFIFOWarningCount++;
		guiMB_ISR_Count++;
		printf("RxFIFO Warning Interrupt occurs!\n");
	}
	// check overflow flag
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF7I)
	{
		FLEXCAN0_IFLAG1 =  FLEXCAN_IFLAG1_BUF7I;
		giRxFIFOOverflowCount++;
		guiMB_ISR_Count++;
		printf("RxFIFO Overflow Interrupt occurs!\n");
	}	                      
}



void FlexCAN0_Selfloop2_MB_ISR(void)
{
	int16_t iMB;
	uint32_t data;	
	
	for(iMB = 0; iMB < NUMBER_OF_MB; iMB++)
	{
		if( FLEXCAN0_IFLAG1 & (1L<<iMB))
		{
			// check if it is a read MB
			if(iMB <= FLEXCAN_RX_MB_END)
			{
				if(!(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ))
				{
					// read message buffer
					 cs[1] = FLEXCAN0_MBn_CS(iMB);
					 id[1]= FLEXCAN0_MBn_ID(iMB);
					 data0[1] = FLEXCAN0_MBn_WORD0(iMB);
					 data1[1] = FLEXCAN0_MBn_WORD1(iMB);
					 timer[1] = FLEXCAN0_TIMER;	// unlock it	
					 iRxMBISRcount++;	// increase Rx MB ISR count
					 
					 // check data
					 data = (((0x00F9uL+3)<<24L) | ((0x00F9uL+2)<<16L) | ((0x00F9uL+1)<<8L) | (0x00F9uL-iRxMBISRcount) );		
					
					 if(data0[1] != data)
					 {
					 	printf("Error: MB %d received incorrect data0 = %#08.8x,expected = %#08.8x\n",iMB,data0[1],data);	
					 	guiErrCount++;		 	
					 }
					 else if(data1[1] != (((0xF9ul+7)<<24) | ((0xF9ul+6)<<16) | ((0xF9ul+5)<<8)	| (0xF9ul+4)))
					 {
					 	printf("Error: MB %d received incorrect data1 = %#08.8x\n",iMB,data1[1]);	
					 	guiErrCount++;			 	
					 }
					// printf("MB %d received message id = %#08.8x,c/s = %#08.8x\n",iMB,id[1],cs[1]);
				}
			}
			//if(iMB >10)
			//printf("MB#%d isr occurs,IFLAG1=%#08.8x\n",iMB,FLEXCAN0_IFLAG1);
			// clear buffer flag
			FLEXCAN0_IFLAG1 = (1L<<iMB);
			guiMB_ISR_Count++;		
		}
	}
}

void FlexCAN0_SelfLooop01_MB_ISR(void)
//__irq void FlexCAN0_SelfLooop01_MB_ISR(void)
{
	extern uint32_t GetControlRegister(void);
	static uint8_t	uiMB0count = 0;
	
	printf("In FlexCAN0_SelfLooop01_MB_ISR,CONTROL = %#08.8x\n",GetControlRegister());
	// Check rx MBs first
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF1I)
	{
		if(!(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ) && (uiMB0count==1))
		{
		 /* Read received frame if queue not enabled and 2nd reception*/
		 cs[1] = FLEXCAN0_MBn_CS(1);
		 id[1]= FLEXCAN0_MBn_ID(1);
		 data0[1] = FLEXCAN0_MBn_WORD0(1);
		 data1[1] = FLEXCAN0_MBn_WORD1(1);
		 timer[1] = FLEXCAN0_TIMER;	// unlock it
		}
		// For queue is enabled, delay read of receive frame so as to make MB1 as active but not free-to-receive
		// clear the flags
		FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF1I;
		guiMB_ISR_Count++;
		uiMB0count++;
		printf("MB1 received a message\n");
	}
		
	/* If message queue is enabled, wait for queue elements done */ 
	if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ )
	{	
	 	if((FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF3I))
	 	{
			 cs[2] = FLEXCAN0_MBn_CS(3);
			 id[2]= FLEXCAN0_MBn_ID(3);
			 data0[2] = FLEXCAN0_MBn_WORD0(3);
			 data1[2] = FLEXCAN0_MBn_WORD1(3);
			 timer[2] = FLEXCAN0_TIMER;	 
			 
			// clear the flags
			FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF3I; 

			printf("MB3 received a message\n");		
				
			// Check received message
			 if(data0[2] != uiMatchValue[2])
			 {
			 	printf("Error: Data Mismatch in MB3 word0,received = %#08.8x,expected = %#08.8x!\n",data0[2],uiMatchValue[2]);
			 	guiErrCount++;
			 }
			 
			 if(data1[2] != uiMatchValue[3])
			 {
			 	printf("Error: Data Mismatch in MB3 word1received = %#08.8x,expected = %#08.8x!\n",data1[2],uiMatchValue[3]);
			 	guiErrCount++;
			 }
		 	 if(((id[2] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	printf("Error: MB3 got wrong ID = %#08.8x,expected = %#08.8x\n",id[2],FLEXCAN_MB_ID_IDSTD(0xE));
			 	guiErrCount++;
			 }	
			 guiMB_ISR_Count++;	 			 		
	 	}			
	}
	
	// Check tx MBs
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF0I)
	{
		FLEXCAN0_IFLAG1 =  FLEXCAN_IFLAG1_BUF0I;		// clear tx MB0 flag
		guiMB_ISR_Count++;
	}
	if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF2I)
	{
		FLEXCAN0_IFLAG1 =  FLEXCAN_IFLAG1_BUF2I;		// clear tx MB2 flag
		guiMB_ISR_Count++;
	}
	if(!(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ))
	{	
		// For queue not enabled,check the MB1 status for overrun
		if(uiMB0count == 2)
		{ 
			 if(((cs[1]>>24) & 0x0F) != FLEXCAN_MB_CODE_RX_OVERRUN)
			 {
				printf("Error: CS word = %#08.8x\n,expected overrun status!\n", cs[1]);
			 }
			 if(data0[1] != uiMatchValue[0])
			 {
			 	printf("Error: Data Mismatch in MB1 word0,received = %#08.8x,expected = %#08.8x!\n",data0[1],uiMatchValue[0]);
			 	guiErrCount++;
			 }
			 
			 if(data1[1] != uiMatchValue[1])
			 {
			 	printf("Error: Data Mismatch in MB1 word1,received = %#08.8x,expected = %#08.8x!\n",data1[1],uiMatchValue[1]);
			 	guiErrCount++;
			 }
			 
			 if(((id[1] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	printf("Error: MB1 got wrong ID = %#08.8x\n",id[1]);
			 	guiErrCount++;
			 }
			 
			 printf("MB1: timer  = %#08.8x\n\n", timer[1]);
		}	 
	}
	else
	{	// For queue enabled, check MB1 for FULL status
		if(uiMB0count==1)
		{
			 cs[1] = FLEXCAN0_MBn_CS(1);
			 id[1]= FLEXCAN0_MBn_ID(1);
			 data0[1] = FLEXCAN0_MBn_WORD0(1);
			 data1[1] = FLEXCAN0_MBn_WORD1(1);
			 timer[1] = FLEXCAN0_TIMER;	// unlock it
			
			 if(((cs[1]>>24) & 0x0F) != FLEXCAN_MB_CODE_RX_FULL)
			 {
				printf("Error: CS word = %#08.8x\n,expected FULL status!\n", cs[1]);
			 }
			 if(data0[1] != uiMatchValue[0])
			 {
			 	printf("Error: Data Mismatch in MB1 word0,received = %#08.8x,expected = %#08.8x!\n",data0[1],uiMatchValue[0]);
			 	guiErrCount++;
			 }
			 
			 if(data1[1] != uiMatchValue[1])
			 {
			 	printf("Error: Data Mismatch in MB1 word1,received = %#08.8x,expected = %#08.8x!\n",data1[1],uiMatchValue[1]);
			 	guiErrCount++;
			 }
			 
			 if(((id[1] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	printf("Error: MB1 got wrong ID = %#08.8x\n",id[1]);
			 	guiErrCount++;
			 }		
		}
	}		
}


                      
void FlexCAN1_SelfLooop01_MB_ISR(void)
{
	extern uint32_t GetControlRegister(void);
	static uint8_t	uiMB0count = 0;
	
	//printf("In FlexCAN1_SelfLooop01_MB_ISR,CONTROL = %#08.8x\n",GetControlRegister());
	// Check rx MBs first
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF1I)
	{
		if(!(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ) && (uiMB0count==1))
		{
		 /* Read received frame if queue not enabled and 2nd reception*/
		 cs[1] = FLEXCAN1_MBn_CS(1);
		 id[1]= FLEXCAN1_MBn_ID(1);
		 data0[1] = FLEXCAN1_MBn_WORD0(1);
		 data1[1] = FLEXCAN1_MBn_WORD1(1);
		 timer[1] = FLEXCAN1_TIMER;	// unlock it
		}
		// For queue is enabled, delay read of receive frame so as to make MB1 as active but not free-to-receive
		// clear the flags
		FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF1I;
		guiMB_ISR_Count++;
		uiMB0count++;
		printf("MB1 received a message\n");
	}
		
	/* If message queue is enabled, wait for queue elements done */ 
	if(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ )
	{	
	 	if((FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF3I))
	 	{
			 cs[2] = FLEXCAN1_MBn_CS(3);
			 id[2]= FLEXCAN1_MBn_ID(3);
			 data0[2] = FLEXCAN1_MBn_WORD0(3);
			 data1[2] = FLEXCAN1_MBn_WORD1(3);
			 timer[2] = FLEXCAN1_TIMER;	 
			 
			// clear the flags
			FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF3I; 

			printf("MB3 received a message\n");		
				
			// Check received message
			 if(data0[2] != uiMatchValue[2])
			 {
			 	printf("Error: Data Mismatch in MB3 word0,received = %#08.8x,expected = %#08.8x!\n",data0[2],uiMatchValue[2]);
			 	guiErrCount++;
			 }
			 
			 if(data1[2] != uiMatchValue[3])
			 {
			 	printf("Error: Data Mismatch in MB3 word1received = %#08.8x,expected = %#08.8x!\n",data1[2],uiMatchValue[3]);
			 	guiErrCount++;
			 }
		 	 if(((id[2] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	printf("Error: MB3 got wrong ID = %#08.8x,expected = %#08.8x\n",id[2],FLEXCAN_MB_ID_IDSTD(0xE));
			 	guiErrCount++;
			 }	
			 guiMB_ISR_Count++;	 			 		
	 	}			
	}
	
	// Check tx MBs
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF0I)
	{
		FLEXCAN1_IFLAG1 =  FLEXCAN_IFLAG1_BUF0I;		// clear tx MB0 flag
		guiMB_ISR_Count++;
	}
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF2I)
	{
		FLEXCAN1_IFLAG1 =  FLEXCAN_IFLAG1_BUF2I;		// clear tx MB2 flag
		guiMB_ISR_Count++;
	}
	if(!(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ))
	{	
		// For queue not enabled,check the MB1 status for overrun
		if(uiMB0count == 2)
		{ 
			 if(((cs[1]>>24) & 0x0F) != FLEXCAN_MB_CODE_RX_OVERRUN)
			 {
				printf("Error: CS word = %#08.8x\n,expected overrun status!\n", cs[1]);
			 }
			 if(data0[1] != uiMatchValue[0])
			 {
			 	printf("Error: Data Mismatch in MB1 word0,received = %#08.8x,expected = %#08.8x!\n",data0[1],uiMatchValue[0]);
			 	guiErrCount++;
			 }
			 
			 if(data1[1] != uiMatchValue[1])
			 {
			 	printf("Error: Data Mismatch in MB1 word1,received = %#08.8x,expected = %#08.8x!\n",data1[1],uiMatchValue[1]);
			 	guiErrCount++;
			 }
			 
			 if(((id[1] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	printf("Error: MB1 got wrong ID = %#08.8x\n",id[1]);
			 	guiErrCount++;
			 }
			 
			 printf("MB1: timer  = %#08.8x\n\n", timer[1]);
		}	 
	}
	else
	{	// For queue enabled, check MB1 for FULL status
		if(uiMB0count==1)
		{
			 cs[1] = FLEXCAN1_MBn_CS(1);
			 id[1]= FLEXCAN1_MBn_ID(1);
			 data0[1] = FLEXCAN1_MBn_WORD0(1);
			 data1[1] = FLEXCAN1_MBn_WORD1(1);
			 timer[1] = FLEXCAN1_TIMER;	// unlock it
			
			 if(((cs[1]>>24) & 0x0F) != FLEXCAN_MB_CODE_RX_FULL)
			 {
				printf("Error: CS word = %#08.8x\n,expected FULL status!\n", cs[1]);
			 }
			 if(data0[1] != uiMatchValue[0])
			 {
			 	printf("Error: Data Mismatch in MB1 word0,received = %#08.8x,expected = %#08.8x!\n",data0[1],uiMatchValue[0]);
			 	guiErrCount++;
			 }
			 
			 if(data1[1] != uiMatchValue[1])
			 {
			 	printf("Error: Data Mismatch in MB1 word1,received = %#08.8x,expected = %#08.8x!\n",data1[1],uiMatchValue[1]);
			 	guiErrCount++;
			 }
			 
			 if(((id[1] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	printf("Error: MB1 got wrong ID = %#08.8x\n",id[1]);
			 	guiErrCount++;
			 }		
		}
	}		
}                      

void FlexCAN1_frames_available_ISR(void)
{
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)
	{
		cs[frames_intr_count] = FLEXCAN1_MB0_CS;
		id[frames_intr_count] = FLEXCAN1_MB0_ID;
		data0[frames_intr_count] = FLEXCAN1_MB0_WORD0;
		data1[frames_intr_count] = FLEXCAN1_MB0_WORD1;
		
		FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;

		if(frames_intr_count == ref_frame_count)  //For Format A --> 7
			frames_done =1;			  //	Format B --> 15
	
		frames_intr_count++;		
		//printf("FlexCAN1 rx FIFO received a frame\n");
	}
        // check warning flag
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF6I)
	{
		FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF6I;
	
		printf("RxFIFO Warning Interrupt occurs!\n");
	}
	// check overflow flag
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF7I)
	{
		FLEXCAN1_IFLAG1 =  FLEXCAN_IFLAG1_BUF7I;
		printf("RxFIFO Overflow Interrupt occurs!\n");
	}	        
}

void FlexCAN1_fifo_warning_ISR (void)
{
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF6I)
	{
		fifo_warning = 1;
		FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF6I;
		
		printf("FIFO Warning interrupt serviced\n");		
	}	
}

void FlexCAN1_fifo_overflow_ISR (void)
{
	if(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF7I)
	{
		fifo_overflow = 1;
		FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF7I;
		
		printf("FIFO Overflow interrupt serviced\n");		
	}	
}

void FlexCAN1_individual_frame_ISR (void)
{
	mb_intr = FLEXCAN1_IFLAG1;	
	
	FLEXCAN1_IFLAG1 = mb_intr;	
}
	
	
void FlexAN0_tx_warning_ISR (void)
	{
		if((FLEXCAN0_ESR1 & FLEXCAN_ESR_TX_WRN) && 
			(FLEXCAN0_ESR1 & FLEXCAN_ESR_ACK_ERR)) 
		{
			FLEXCAN0_CTRL1 &= ~(FLEXCAN_CTRL_TWRN_MSK 
			| FLEXCAN_CTRL_ERR_MSK); //Disable Tx Warning and Error Interrupt 
			tx_warning_and_error_intr_occured = 1;
	        	printf("Tx Warning Interrupt and Ack. Error Interrupt Occured\n");
		}
	}
	
void FlexCAN1_bit1_error_ISR (void)
	{
		if(FLEXCAN1_ESR1 & FLEXCAN_ESR_BIT1_ERR) 
		{
			FLEXCAN1_CTRL1 &= ~FLEXCAN_CTRL_ERR_MSK; //Disable Error Interrupt 
			bit1_error_occured = 1;
	        	printf("Bit 1 Error Interrupt Occured\n");
		}
	}

void FlexCAN1_bit0_error_ISR (void)
	{
		if(FLEXCAN1_ESR1 & FLEXCAN_ESR_BIT0_ERR) 
		{
			FLEXCAN1_CTRL1 &= ~FLEXCAN_CTRL_ERR_MSK; //Disable Error Interrupt 
			bit0_error_occured = 1;
	        printf("Bit 0 Error Interrupt Occured\n");
		}
	}
	
void FlexCAN0_Wakeup_ISR(void)
{
	if(FLEXCAN0_ESR1 & FLEXCAN_ESR_WAK_INT)
	{
		FLEXCAN0_ESR1 = FLEXCAN_ESR_WAK_INT;	// clear wakeup interrupt
		printf("FlexCAN0 wakeup interrupt occurs!\n");
	}
	else
	{
		printf("FlexCAN0_Wakeup_ISR: WAK_INT bit in ESR1 is not set when entering FlexCAN0_Wakeup_ISR\n");
	}
}

void FlexCAN1_Err_ISR(void)
{
	if(FLEXCAN1_ESR1 & FLEXCAN_ESR_ERR_INT)
	{
		FlexCAN1_bit0_error_ISR();
		FlexCAN1_bit1_error_ISR();
		FLEXCAN1_ESR1 = FLEXCAN_ESR_ERR_INT;	// clear  flag
	}
	else
	{
		printf("FlexCAN1_Err_ISR: ERR_INT bit in ESR1 is not set when entering FlexCAN1_Err_ISR\n");
	}
}

void FlexCAN1_BOFF_ISR(void)
{
        printf("Enter FlexCAN1 busoff ISR!\n");
	if(FLEXCAN1_ESR1 & FLEXCAN_ESR_BOFF_INT)
	{
		FLEXCAN1_ESR1 = FLEXCAN_ESR_BOFF_INT;	// clear  flag
	}
	else
	{
		printf("FlexCAN1_BOFF_ISR: BOFF_INT bit in ESR1 is not set when entering FlexCAN1_BOFF_ISR\n");
	}
}

void FlexCAN0_TxWarningISR(void)
{
    FLEXCAN0_ESR1 |= FLEXCAN_ESR_TX_WRN;
}
                      
void FlexCAN0_RxWarningISR(void)
{
  FLEXCAN0_ESR1 |= FLEXCAN_ESR_RX_WRN;
}

void FlexCAN1_TxWarningISR(void)
{
    FLEXCAN1_ESR1 |= FLEXCAN_ESR_TX_WRN;
}
                      
void FlexCAN1_RxWarningISR(void)
{
  FLEXCAN1_ESR1 |= FLEXCAN_ESR_RX_WRN;
}
                      
void LPTmr_ISR(void)
{
  LPTMR0_CSR = 0x80;  // clear LPTMR interrupt flag and disable it  
}
                      
void FTM1_TKT029460_ISR(void)
{
    uint32  iBitCounts;
 
     // Assert a dominant on EOF 7th bit
    // Change this pin to output mode
    //GPIOA_POER |= (1<<4);
    //GPIOA_PDOR &= ~(1<<4);
    GPIOA_PDOR &= ~(1<<12);
    //iBitCounts =   FLEXCAN1_TIMER;
    FLEXCAN1_TIMER=0;   
    while(FLEXCAN1_TIMER < 1){}
 //   while(FLEXCAN1_TIMER < (iBitCounts+1)){}
    //GPIOA_PDOR = (1<<4);
    
    // Change this pin to output mode
    //GPIOA_POER &= ~(1<<4);
    GPIOA_PDOR |= (1<<12);
          
    // Clear FTM1 flag and stop the timer
    FTM1C0SC_CH0F = 0;    
    FTM1SC_CLKSx= 0;      /* disable timer */
    FTM1C0SC_CH0IE = 0;    /* disable channel interrupt */    
}
    
void PortA_ISR(void)
{
  //if((PORTA_PCR4 & PORT_PCR_ISF_MASK) && (PORTA_ISFR & (1<<4)))
  {     
      // Start up the timer
      FTM1SC_CLKSx= 1;        /* select bus clock as source clock */  
//      FLEXCAN1_TIMER = 0;    
      
      // Clear the flag
      PORTA_ISFR = (1<<4);
      
      // Change this pin to output mode
      //GPIOA_POER |= (1<<4);
       
      // Disable PortA interrupt
      PORTA_PCR4 &= ~(PORT_PCR_IRQC_MASK);
  }
 // else
  {
 //   printf("PortA_ISR: unknow interrupt on PortA happened!\r\n");
  }
}
                      
                      