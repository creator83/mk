/*
 * File:    intc_reset_check.c
 * Purpose:   Main process
 *
 */
//#define	TEST_SELF_RECEPTION_DISABLE
//#define	TEST_USE_POLL
#define	TEST_EXTENDED_ID
#define	TEST_SCAN_PRIORITY_MB_FIRST
//#define	TEST_REMOTE_REQUEST_FEATURE
//#define TEST_ACCESS_MODE_FLEXCAN0_MB


// macro switches for low power mode test
#define	USE_PIT0_WAKEUP
#define	USE_WFI



// others
#define FIFO_OVERFLOW_TEST  0
#define SAMPLING_TEST		1
//#define TIMER_SYNC			1


#ifdef  FTM_USE_EXT_CLOCK
/* CAN bit time count for TPM using external clock
 * when PS=2 (prescaler =4)
 */
#define CAN_BIT_TIME    24     // 83.33kbps for external clock of 8MHz

#else
/* CAN bit time count for TPM using IPbus clock
 * when PS=2 (prescaler =4)
 */
#define CAN_BIT_TIME    144     // 83.33kbps for IPBus clock of 48MHz
//#define	CAN_BIT_TIME	150	// for IPBus = 48Mhz, get 80Kbps
                                // for IPBus = 50Mhz, get 83.33Kbps
//#define	CAN_BIT_TIME	12	// for IPBus = 48Mhz, get 1Mbps
                                // for IPBus = 50Mhz, get 1.042Mbps
//#define	CAN_BIT_TIME	25	// for IPBus = 100Mhz
#endif

#include "common.h"
#include "p2io_map.h"
#include "target.h"
#include <stdio.h>
#include "kinetis_flexcan.h"
#include "fcandrv.h"
#include "test_config.h"



#define 	STOP_MODE	0 //1 //0: for wait mode, 1: for stop mode
//#define 	STOP_MODE	1 //0: for wait mode, 1: for stop mode
#if (STOP_MODE==1)
  #if !defined(USE_EXTERNAL_CLOCK)
  #error  "To use STOP_MODE =1, must define USE_EXTERNAL_CLOCK\n"
  #endif
#endif

#define error	printf

//#define printf  

#undef TESTCASE_FAIL
#define TESTCASE_FAIL printf("ERROR: Test case FAILed!\n")

#undef TESTCASE_PASS
#define TESTCASE_PASS printf("SUCCESS: Test case PASSed!\n")


#if 0
#define NVIC_EnableInterruptBit(start_bit,end_bit,bit_no)   \
  { \    
NVIC_IRQ_##start_bit##_##end_bit##_CLR_PENDING_REG     |=  (1<<bit_no);        //Enable interrupts        \     
   NVIC_IRQ_##start_bit##_##end_bit##_SET_ENABLE_REG      |=  (1<<bit_no);        //Clear any pending interrupts  \     
  }
#endif

/* Default value after reset for registers */
#define FLEXCAN_MCR_INIT 		0xD890000FL
#define FLEXCAN_CTRL1_INIT		0x00000000L
#define FLEXCAN_TIMER_INIT		0x00000000L
//#define FLEXCAN_TCR_INIT		0x00000000L	// not accessible when not in test mode
#define FLEXCAN_RXGMASK_INIT	0xFFFFFFFFL
#define FLEXCAN_RX14MASK_INIT  0xFFFFFFFFL
#define FLEXCAN_RX15MASK_INIT	0xFFFFFFFFL
#define FLEXCAN_ECR_INIT		0x00000000L
#define FLEXCAN_ESR1_INIT		0x00000000L
#define FLEXCAN_IMASK1_INIT	0x00000000L
#define FLEXCAN_IFLAG1_INIT	0x00000000L
#define FLEXCAN_CTRL2_INIT		0x00C00000 // confirmed with IP owner
#define FLEXCAN_ESR2_INIT		0x00000000L
#define FLEXCAN_FUREQ_INIT		0x00000000L
#define FLEXCAN_FUACK_INIT		0x00000000L
#define FLEXCAN_CRCR_INIT		0x00000000L
#define FLEXCAN_RXFGMASK_INIT	0xFFFFFFFFL
//#define FLEXCAN_RXFIR_INIT	0x00000000L	// not valid if IFLAG[BUF5I] is not asserted
#define FLEXCAN_DBG1_INIT		0x01000000L
#define FLEXCAN_DBG2_INIT		0x00000000L


/* For bit test */
#define MSB 0x80
#define MSB_16bit 0x8000
#define CORRECT_FRAME   0x00
#define FAULT_FRAME	 0x01


//Rx FIFO macros
#define	MAX_RX_FIFO_FILTER	40
#define	RX_FIFO_DEEP		6		// in MBs

// Rx FIFO Filter format
#define FILTER_FORMAT_A		0
#define FILTER_FORMAT_B		1
#define FILTER_FORMAT_C		2
#define FILTER_FORMAT_D		3

// utility macros
#define swap_4bytes(ptr)	{ uint8_t	byte;	\
								byte = (ptr)[0]; (ptr)[0] = (ptr)[3]; (ptr)[3]=byte;	\
								byte = (ptr)[1]; (ptr)[1] = (ptr)[2]; (ptr)[2]=byte;	\
							}
#define min(a,b)			((a)>(b)?(b):(a))


// Low power mode macros
#ifdef	USE_WFI
	#define	ENTER_LP_MODE_INSTRUCTION		WFI		// wait for interrupt
#else
	#define	ENTER_LP_MODE_INSTRUCTION		WFE		// wait for event to resume
#endif

/* Type definitions */
#ifdef	USE_BIG_ENDIAN
	typedef union {
	  unsigned long EXTENDED;
	  unsigned int STANDARD;
	  struct {
	    unsigned char B3;                                       
	    unsigned char B2;                    
	    unsigned char B1;                    
	    unsigned char B0;                    
	  } BYTE;
	} type_ID;
#else
	typedef union {
	  unsigned long EXTENDED;
	  unsigned int STANDARD;
	  struct {
	    unsigned char B0;                                       
	    unsigned char B1;                    
	    unsigned char B2;                    
	    unsigned char B3;                    
	  } BYTE;
	} type_ID;
#endif

typedef struct MsgData
{
 unsigned char LENGTH; 
 unsigned char BYTE0; 
 unsigned char BYTE1; 
 unsigned char BYTE2; 
 unsigned char BYTE3; 
 unsigned char BYTE4;
 unsigned char BYTE5;
 unsigned char BYTE6;
 unsigned char BYTE7;
} TMSG_Data, *PTMSG_Data;

enum 
{	CAN_FRAME_TYPE_CORRECT = 0,
	CAN_FRAME_TYPE_BIT0_ERROR,
	CAN_FRAME_TYPE_BIT1_ERROR,
	CAN_FRAME_TYPE_STUF_ERROR,
	CAN_FRAME_TYPE_FORM_ERROR,
	CAN_FRAME_TYPE_CRC_ERROR,
	CAN_FRAME_TYPE_ACK_ERROR,
        CAN_FRAME_TYPE_EOF_7th_BIT_ERROR,
        CAN_FRAME_TYPE_EOF_6th_BIT_ERROR,
        CAN_FRAME_TYPE_EOF_5th_BIT_ERROR,
        CAN_FRAME_TYPE_EOF_4th_BIT_ERROR,
        CAN_FRAME_TYPE_EOF_3th_BIT_ERROR,
        CAN_FRAME_TYPE_EOF_2th_BIT_ERROR,
        CAN_FRAME_TYPE_EOF_1th_BIT_ERROR      
}TCANFrameType;

// Rx Fifo filter
typedef	struct TRxFIFOFilterParams 
{
	uint8_t	rxFIFOFilterNo;	// Number of Rx FIFO filters
	uint8_t topRxFIFOMBNo;	// Message Buffers occupied by Rx FIFO and ID Filter Table
	uint8_t topAvailMBNo;		// Remaining Available Mailboxes
	uint8_t topRxFIFOTabElementNoByIMask;// Rx FIFO ID Filter Table Elements Affected by Rx Individual Masks
	uint8_t	topRxFIFOTabElementNoByFGMask;// Rx FIFO ID Filter Table Elements Affected by Rx FIFO Global Mask
										  // 0 means none 
} TRxFIFOFilterParams;



/* Prototypes */
void FlexCAN_Reset_Test(void);
void FlexCAN_Registers_Test(void);
void FlexCAN_SelfLoop0(void);
void FlexCAN_SelfLoop01(void);
void FlexCAN_SelfLoop2(void);
void FlexCAN1_SelfLoop0(void);
void FlexCAN1_SelfLoop01(void);
void FlexCAN_RxFifoFilter_Test(void);
void FlexCAN_RxFifoInt_Test(void);
void FlexCAN0_RxFifoInt_Test(void);
void FlexCAN1_RxFifoInt_Test(void);
void FlexCAN1_RxFifoInt_Test2(void);
void FlexCAN_Priority_Test(void);
void FlexCAN0_Priority_Test(void);
void FlexCAN1_Priority_Test(void);
void FlexCAN_RemoteFrame_Test(void);
void FlexCAN0_RemoteFrame_Test(void);
void FlexCAN_LowPowerMode_Test(void);
void FlexCAN0_DozeStopMode_test(void);
void FlexCAN_Disable_Test(void);
void FlexCAN_Tsync_Test(void);
void FlexCAN0_Tsync_Test(void);
void FlexCAN1_Tsync_Test(void);
void FlexCAN_CrcStatus_Test(void);
void FlexCAN1_CrcStatus_Test(void);
void FlexCAN_IndividualMasking_Test(void);
void FlexCAN0_IndividualMasking_Test(void);
void FlexCAN_StdExt_IDs_Test(void);
void FlexCAN_BusOffMode_Test(void);
void FlexCAN_Error_Int_Test(void);
void FlexCAN_CrcFormStuf_Err_Test(void);
void FlexCAN_FreezeMode_Test(void);
void FlexCAN1_FreezeMode_Test(void);
void FlexCAN_TxRx_Test(void);
void FlexCAN1_TxRx_Test(void);
void FlexCAN1_TxRx_Test2(void);
void FlexCAN1_TxRx_Test3_External(void);
void FlexCAN_AccessMode_Test(void);
void FlexCAN_ClkSrc_Test(void);
void FlexCAN_Baud_Test(void);
void FlexCAN1_Baud_Test(void);
void FlexCAN_TxAbort_Test(void);
void FlexCAN1_TxAbort_Test(void);
void FlexCAN1_TxAbort_Test2(void);
void FlexCAN0_RxFifoFilter_Test(void);
void FlexCAN0_RxFifoFilter_Test_FormatA(void);
void FlexCAN0_RxFifoFilter_Test_FormatB(void);
void FlexCAN0_RxFifoFilter_Test_FormatC(void);
void FlexCAN_ListenOnlyMode_Test(void);
void FlexCAN_SafeReConfigurationMBs_Test(void);
void FlexCAN0_SafeReConfigurationFIFO_Test_FormatA(void);
void FlexCAN_IPgear35536_Test(void);
void FlexCAN_IPgear35532_Test(void);
void FlexCAN1_ERR002623_Test(void);
void FlexCAN_TKT029460_Test(void);

////////////////////////////////////////////////////////////////////////////////
void flexcan0_tx_can1_rx_fifo_formatA_bcc_test(void);
void flexcan0_tx_can1_rx_fifo_formatB_bcc_test(void);
void flexcan0_tx_can1_rx_fifo_formatC_bcc_test(void);
void flexcan0_tx_can1_rx_fifo_warning_overflow_bcc_test(void);
void flexcan0_tx_can1_rx_priority_bcc_test(void);
void flexcan0_tx_can1_rx_remote_frame_bcc_test(void);
void flexcan0_tx_can1_rx_extended_frame_test(void);
void flexcan0_tx_can1_rx_extended_frame_receive_queue_test(void);
void flexcan0_tx_can1_rx_fifo_extended_frame_test(void);
void flexcan0_rx_can1_tx_extended_frame_test(void);
void flexcan0_tx_can1_rx_extended_frame_MB_interrupt_test(void);
void flexcan0_tx_can1_rx_extended_frame_baud_rate_test(void);
void flexcan0_module_disable_test(void);
void flexcan0_supervisor_access_only_test(void);
void flexcan0_tx_can1_rx_extended_frame_oscillator_clk_src_test(void);
void flexcan0_tx_warning_ack_error_intr_test(void);
void flexcan0_tx_can1_tx_bit1_error_intr_test(void);
void flexcan0_tx_can1_lom_bit0_error_intr_test(void);
///////////////////////////////////////////////////////////////////////////////////
void ConfigureStopMode(void);
void ConfigureDozeMode(void);
void EnterDozeMode(void);
void EnterStopMode(void);
void ConfigurePIT(void);
void ConfigureLPT(void);
void StartPIT(void);
void StartLPT(void);
void StartLPWakeupSource(void);
void ConfigureLPWakeupSource(void);
void PrintPassFailMessage(uint16_t uiErrCount);

uint16_t FlexCAN_PrepareFrameBitsWithCRC(uint32_t MsgID, uint8_t IDType,uint8_t RTR,
   PTMSG_Data Data,
   uint8_t MsgStatus );  
uint16_t BuildCANFrame(uint32_t MsgID, uint8_t IDType,uint8_t RTR,PTMSG_Data Data,
 uint8_t iFrameType );    
void CalcCheckSum(unsigned char NXTBIT);
void CheckBitStuffing(unsigned char BIT);
void DelayBits(uint32_t bits);
void DelayBits1(uint32_t bits);
void TPM_SendBitStream(void);
void TIMER_CH0_ISR(void);
void SelectionSort(uint32_t a[], int array_size);

// system control utilities
uint32_t EnterPrivilegeMode(void);
uint32_t EnterUserMode(void);  
uint32_t GetControlRegister(void);
void ConfigureAIPS_Lite(void);
void NVIC_enable_PIT_interrupts(void);

// Globals
volatile uint32_t frames_intr_count = 0,
				frames_done = 0,
				fifo_warning = 0,
				fifo_overflow = 0,
				ref_frame_count,
				core_fault_occured = 0,
				mb_intr = 0,
				tx_warning_and_error_intr_occured = 0,
				bit1_error_occured = 0,
				bit0_error_occured = 0,
				cs[17],
				id[17],
				data0[17],
				data1[17];
volatile uint32_t timer[4];
volatile uint32_t uiMatchValue[4];	
volatile uint16_t guiErrCount;		
volatile uint32_t guiMB_ISR_Count,giRxFIFOISRCount,giRxFIFOWarningCount,giRxFIFOOverflowCount;	
vuint16_t	iRxMBISRcount;
vuint16_t	gNoMBsAvail;	
vuint32_t	giTimeStamp[NUMBER_OF_MB];	
vuint8_t    acces_mode;

unsigned char TransmitDataFrame[16*2];
unsigned char TxBitCount,TxSentBit,BitMask;
unsigned char CAN_Msg_Sending;
uint16_t  CeckSum;
static unsigned char dominant= 0,recessive = 0;
uint16    nPortAISRCount;


// rx FIFO
TRxFIFOFilterParams	rxFIFOFilterParams[MAX_RX_FIFO_FILTER/8];


/********************************************************************/

void  main (void)
{
	
        InitTarget();	
 
        // Initialize globals
        dominant= 0;
        recessive = 0;   
        
 	#ifdef	TEST_RESET_TEST
 		 FlexCAN_Reset_Test();
 	#endif
 	#ifdef	TEST_REGISTERS_TEST
 		 FlexCAN_Registers_Test();
  	#endif
	#ifdef	TEST_SELF_LOOP0
#if TEST_FLEXCAN0
		FlexCAN_SelfLoop0();
#endif
#if TEST_FLEXCAN1
	FlexCAN1_SelfLoop0();                
#endif                
	#endif
	#ifdef	TEST_SELF_LOOP01
#if TEST_FLEXCAN0        
		FlexCAN_SelfLoop01();  
#endif
#if TEST_FLEXCAN1
                FlexCAN1_SelfLoop01();
#endif                
	#endif
	#ifdef	TEST_SELF_LOOP2
 		FlexCAN_SelfLoop2(); 
 	#endif
 	#ifdef	TEST_CRC_STATUS
#if TEST_FLEXCAN0                
 		FlexCAN_CrcStatus_Test(); 
#endif
#if TEST_FLEXCAN1
                FlexCAN1_CrcStatus_Test();                
#endif                
 	#endif
 	#ifdef	TEST_TXRX_Test
 		FlexCAN_TxRx_Test();
 	#endif
 	#ifdef	TEST_TXRX_Test2
 		FlexCAN_TxRx_Test();
        #endif
 	
#ifdef  TEST_TXRX_Test3
                FlexCAN1_TxRx_Test3_External();
#endif                
 	#ifdef	TEST_RXFIFO_FILTER
 		FlexCAN_RxFifoFilter_Test();
 	#endif
 	#ifdef	TEST_RXFIFO_INT
 		FlexCAN_RxFifoInt_Test();
 	#endif
 	#ifdef TEST_TSYNC
 		FlexCAN_Tsync_Test();
 	#endif
 	#ifdef	TEST_REMOTE_FRAME
 		FlexCAN_RemoteFrame_Test();
 	#endif
 	#ifdef	TEST_PRIORITY
 		FlexCAN_Priority_Test();
 	#endif
 	#ifdef	TEST_INDIVIDUAL_MASKING
 		FlexCAN_IndividualMasking_Test();
 	#endif
 	#ifdef	TEST_TX_ABORT
 		FlexCAN_TxAbort_Test();
 	#endif
 	#ifdef	TEST_BAUD
#if TEST_FLEXCAN0                
 		FlexCAN_Baud_Test();
#endif
#if TEST_FLEXCAN1
                FlexCAN1_Baud_Test();
#endif                
 	#endif
 	#ifdef	TEST_FREEZE_MODE
#if TEST_FLEXCAN0                
 		FlexCAN_FreezeMode_Test();
#endif
#if TEST_FLEXCAN1
 		FlexCAN1_FreezeMode_Test();
#endif                
 	#endif
 	#ifdef	TEST_DISABLE_MODE	
 		FlexCAN_Disable_Test();
 	#endif
 	#ifdef	TEST_BUS_OFF
 		FlexCAN_BusOffMode_Test();
 	#endif
 	#ifdef	TEST_MB_ERROR_INT
 		FlexCAN_Error_Int_Test();
 	#endif
 	#ifdef	TEST_CRC_FORM_STUF_ERROR
 		FlexCAN_CrcFormStuf_Err_Test();
 	#endif
 	#ifdef	TEST_LOW_POWER_MODE
 		FlexCAN_LowPowerMode_Test();
 	#endif
 	#ifdef	TEST_LISTEN_ONLY_MODE
 		FlexCAN_ListenOnlyMode_Test();
 	#endif
 	#ifdef TEST_SAFE_RECONFIG_MB
 		FlexCAN_SafeReConfigurationMBs_Test();
 	#endif
 	#ifdef TEST_SAFE_RECONFIG_FIFO_FORMAT_A
 		FlexCAN0_SafeReConfigurationFIFO_Test_FormatA();
 	#endif
 	#ifdef	TEST_ACCESS_MODE
 		FlexCAN_AccessMode_Test();
 	#endif
 	#ifdef	TEST_IPgear_35536
 		FlexCAN_IPgear35536_Test();
 	#endif
	#ifdef	TEST_IPgear_35532
		FlexCAN_IPgear35532_Test();
	#endif
        #ifdef  TEST_ERR002623
                FlexCAN1_ERR002623_Test();
        #endif     
        #ifdef  TEST_TKT029460
                FlexCAN_TKT029460_Test();
        #endif                
  printf("\nTestcase Completed!\n");  
}


void FlexCAN_IPgear35532_Test(void)
{
/*
* 'Title' ==> Bug: MB inactivation causes reception failure. 
'Description' ==>  Reception fails in the following scenario:

 - Only one Mailbox is configured as Rx: RXMB.(at least MB0)
 - MB0 is inactivated at the same clock Rx Match FSM compares RXMB ID field with ID received on CAN bus (Rx SMB). 

 - The ID received should match with RX MB ID, but FlexCAN lost a Frame that should be received.

OBS: This problem can occur in any MB comparison from MB1 to last MB if MB0 is inactivated during Rx Match Comparison State.  
*/

	uint_32 state; 
	uint_32 id;
	int16_t iRxMB;
	int16_t iTxMB;
	int16_t i;
	int8_t	bIntrp;
	uint8_t bActivate_mb;
	uint16_t baudrate;
	volatile FLEXCAN_MailBox_STRUCT mb_config;
	
	guiMB_ISR_Count  = 0;
    guiErrCount = 0;
    iRxMBISRcount = 0;
    
	bIntrp = FALSE;

   
	id = 0x1ABCDEF9;
	baudrate = 83;//100;//1000;	// 1000Kbps
	for(i = 0; i< 8; i++)
	{
		mb_config.data[i] = i+1; //0xF9+i;
	}
	// Initialize bit timing
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
#ifndef	TEST_ON_EVM	
	TRUE	// for loopback
#else
	FALSE
#endif	
	);
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}

	#ifdef	TEST_MESSAGE_QUEUE
		FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;
		printf("Message Queue is enabled!\n");
		// set individual masking
	    for( iRxMB = FLEXCAN_RX_MB_START; iRxMB <= FLEXCAN_RX_MB_END; iRxMB++)
		{
			FLEXCAN0_RXIMRn(iRxMB) = 0; // do not care
		}
	#endif

    if(!(FLEXCAN0_CTRL1 & FLEXCAN_CTRL_LPB))
    {
      // Disable self-reception
       FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;
    }
    // Start CAN communication
    //printf("start CAN communication, MCR = %#08.8x\n",FLEXCAN0_MCR);
    state = FLEXCAN_Start(FLEXCAN0);
  	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Start() returned state = %#08x\n",state);
	}  
    printf("FLEXCAN0_CTRL1 =%#08.8x,FLEXCAN0_MCR = %#08.8x\n",FLEXCAN0_CTRL1,FLEXCAN0_MCR);
       
    // Initialize the mailbox structure for Rx 
    mb_config.dev_num = FLEXCAN0;
    mb_config.data_len = 8;
    mb_config.identifier = id;
    mb_config.format = FLEXCAN_EXTENDED;
    mb_config.direction = FLEXCAN_RX;
    mb_config.remote_req_flag = 0;
    mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
    bActivate_mb = TRUE;
    
    for (iRxMB = FLEXCAN_RX_MB_START; iRxMB <= FLEXCAN_RX_MB_END; iRxMB++)
    {
    	/* Initialize the only Rx MB */
    	mb_config.mailbox_number = iRxMB;
    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,bIntrp);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
		}  	
    }	
    mb_config.direction = FLEXCAN_TX;
    mb_config.remote_req_flag = 0;
    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
  
    GPIOA_PCR(4) = (PIN_ALT1<<PIN_MUX_BIT_NO); // configure PTA4 as GPIO
    GPIOA_PDOR = (1<<4);
    GPIOA_POER |= (1<<4);               
    
    for( iTxMB = FLEXCAN_TX_MB_START; iTxMB <= FLEXCAN_TX_MB_END; iTxMB++)
    {
    	/* Initialize each Tx MB */
    	mb_config.mailbox_number = iTxMB;
    	mb_config.data[0]--;
        bActivate_mb = FALSE; // do not activate it
    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,bIntrp);
    	if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
		}

                // Activate this tx MB
		FLEXCAN0_MBn_CS(iTxMB) |= FLEXCAN_set_msg_ctrlcode(FLEXCAN_MB_CODE_TX_ONCE); 
           
		// Toggle PTA4
                GPIOA_PDOR = 0;

		DelayBits(100); // 
                //DelayBits(145); // issue
		GPIOA_PDOR |= (1<<4);

#ifdef  SEE_QUEUE_BUG             
		// Inactivate the rx MB0
                iRxMB = FLEXCAN_RX_MB_START;
		FLEXCAN0_MBn_CS(iRxMB++) &= ~(FLEXCAN_MB_CS_CODE_MASK);
#endif
		
		// wait till IFLAG1 set                
		while(!(FLEXCAN0_IFLAG1 & ((1<<(FLEXCAN_RX_MB_END-FLEXCAN_RX_MB_START+1))-1)))
                {
                  printf("FLEXCAN0_IFLAG1 = %#08.8x\n",FLEXCAN0_IFLAG1);
                }
 	 
 	 	// check ID and message bytes to see if it is correct.
                for (iRxMB = FLEXCAN_RX_MB_START; iRxMB <= FLEXCAN_RX_MB_END; iRxMB++)
                {
                  if(!(FLEXCAN0_IFLAG1 & (1<<iRxMB)))
                  {
                    continue;
                  }
                   id= FLEXCAN0_MBn_ID(iRxMB);
                   
                   data0[1] = FLEXCAN0_MBn_WORD0(iRxMB);
                   data1[1] = FLEXCAN0_MBn_WORD1(iRxMB);
                   timer[1] = FLEXCAN0_TIMER;	// unlock it	
                   printf("RxMB[%d] received a message: id =%#08.8x,word0=%#08.8x, word1=%#08.8x\n",iRxMB,id,data0[1],data1[1]);
                   state = FLEXCAN_Clear_MB_Flag(FLEXCAN0,iRxMB); 
                  if(state != FLEXCAN_OK)
                  {
                          printf("FLEXCAN_Clear_MB_Flag: returned state = %#08.8x for mailbox %d\n",state,iRxMB);
                  }	
                }
                
                // wait for TxMB to be done
                while(!(FLEXCAN0_IFLAG1 & (1<<iTxMB)));
                
                // clear flag for tx MB
                FLEXCAN0_IFLAG1 = (1<<iTxMB);
    }
    
}




void FlexCAN_IPgear35536_Test(void)
{
/*	
- FlexCAN receives a frame which matches with RXMB(i).
- CPU locks RXMB(i) by reading CS from current RXMB
- FLexCAN receives a new frame which matches with the same RXMB(i).
- FLexCAN receives frame which matches with RXMB(i) again.
- CPU request Freeze or Low Power.
- Freeze Acknowledge (MCR[FRZ_ACK]) or Low Power Acknowledge (MCR[LPM_ACK])is not asserted.
*/

	uint16_t baudrate;
	uint32_t state;
	int32_t	 i;
	
	printf("Start to test FlexCAN behavior in Freeze mode when locking a RxMB on FlexCAN0 \n");
 
 	// Initialize error counter
	guiErrCount = 0;

  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		TRUE	// for loopback
		);	

	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN0_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;

#if (!STOP_MODE)
	 	FLEXCAN0_MCR |=   // FLEXCAN_MCR_SRX_DIS 	|
	 

	 					FLEXCAN_MCR_DOZE		 //Enable Doze Mode
#endif	 					  
	 					;

	 // Enable wake up and wakeup interrupt
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SLF_WAK | FLEXCAN_MCR_WAK_MSK
				  |  FLEXCAN_MCR_WAK_SRC	 		// enable wakeup filter
	 			 ;
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_RXIMRn(i) = 0x00000000;
	}
			 
	//	individual Rx masking and queue enable
	FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;

	// clear CTRL2 but TASD
	FLEXCAN0_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB0 to MB7 as a receive MB with ID don't care
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
		FLEXCAN0_MBn_CS(i) = 0x00000000;
		FLEXCAN0_MBn_ID(i) = 0x00000000;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}	
	
	// Write timer to some value
	FLEXCAN0_TIMER = 0xCCCC;
	
	// Start CAN communication
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
	
	while(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	
	// Initialize MB8 to 15 as transmit MBs
	for(i = NUMBER_OF_MB/2;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN0_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN0_MBn_WORD0(i) = 0x5555;
		FLEXCAN0_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}	

	// wait till MB0 receives a message
	while(!(FLEXCAN0_IFLAG1 & (1<<0)));
 
 	// Lock the MB0
 	cs[0] = FLEXCAN0_MBn_CS(0);
 	
    // clear the flags
	FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF0I;	
	
	// Send another message to MB0
	FLEXCAN0_MBn_CS(i-1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	FLEXCAN0_MBn_ID(i-1) = 0x18588885L+i;
	FLEXCAN0_MBn_WORD0(i-1) = 0x5555;
	FLEXCAN0_MBn_WORD1(i-1) = 0xAAAA;
	FLEXCAN0_MBn_CS(i-1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
							 | FLEXCAN_MB_CS_LENGTH(8)
							 | FLEXCAN_MB_CS_IDE
							 | FLEXCAN_MB_CS_SRR;
								 
	// wait till MBi completes transmission of a message
	while(!(FLEXCAN0_IFLAG1 & (1<<(i-1))));
	
	// wait till MB0 receives a message
	while(!(FLEXCAN0_IFLAG1 & (1<<0)));

    // clear the flags
	FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF0I;	
		
	// Send 3rd message to MB0
	FLEXCAN0_MBn_CS(i-1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	FLEXCAN0_MBn_ID(i-1) = 0x18588885L+i;
	FLEXCAN0_MBn_WORD0(i-1) = 0x5555;
	FLEXCAN0_MBn_WORD1(i-1) = 0xAAAA;
	FLEXCAN0_MBn_CS(i-1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
							 | FLEXCAN_MB_CS_LENGTH(8)
							 | FLEXCAN_MB_CS_IDE
							 | FLEXCAN_MB_CS_SRR;
								 
	// wait till MBi completes transmission of a message
	while(!(FLEXCAN0_IFLAG1 & (1<<(i-1))));
	
	// wait till MB0 receives a message
	while(!(FLEXCAN0_IFLAG1 & (1<<0)));

    // clear the flags
	FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF0I;	
			
	// Put flexcan in freeze mode
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
	
	printf("Request to enter the freeze mode now\n");
	while( !(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK)) ||
	
	        !(FLEXCAN0_MCR & FLEXCAN_MCR_NOT_RDY)) {}
	
	printf("Freeze mode is entered\n");
	
	// Now exit freeze mode
	FLEXCAN0_MCR &= ~(FLEXCAN_MCR_FRZ);
	while(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK));
	
	//
#if STOP_MODE
	ConfigureStopMode();
#else
	ConfigureDozeMode();
#endif	

	ConfigureLPWakeupSource();
	printf("Begin to enter doze/stop mode...\n");
	
	//
	StartLPWakeupSource();
	
 	//Enter Doze/Stop mode
	EnterStopMode();
	
	printf("Exit from doze/stop mode, FLEXCAN0_MCR = %#08.8x\n",FLEXCAN0_MCR);	
        if(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK))
        {
          printf("Error: FRZ_ACK is not cleared!\r\n");
          guiErrCount++;
        }
        if(FLEXCAN0_MCR & (FLEXCAN_MCR_LPM_ACK))
        {
          printf("Error: LPM_ACK is not cleared!\r\n");
          guiErrCount++;
        }
	PrintPassFailMessage(guiErrCount);		
}

/********************************************************************/
// For checking reset value of all FlexCAN registers
void FlexCAN_Reset_Test(void)
{
  uint16_t uiErrCount = 0;
   
  printf("Starting Reset values check...\n\n");
  
 /* For FlexCAN0*/
  
if(FLEXCAN0_MCR != FLEXCAN_MCR_INIT)
{
	error("FlexCAN0 register reset value FLEXCAN0_MCR is not correct %08X \n",FLEXCAN0_MCR);
	uiErrCount++;
}
  
if(FLEXCAN0_CTRL1 != FLEXCAN_CTRL1_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_CTRL1 is not correct %08X \n",FLEXCAN0_CTRL1);
  uiErrCount++;
}
if(FLEXCAN0_TIMER != FLEXCAN_TIMER_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_TIMER is not correct %08X \n",FLEXCAN0_TIMER);
  uiErrCount++;
}
#if 0
printf("Checking FLEXCAN0_TCR...\n");
if(FLEXCAN0_TCR != FLEXCAN_TCR_INIT) //can be only accessed in TEST mode.reset value:0x00000000
{
  error("FlexCAN0 register reset value FLEXCAN0_TCR is not correct %08X \n",FLEXCAN0_TCR);
  uiErrCount++;
 }
printf("Checking FLEXCAN0_TCR DONE\n"); 
#endif
if(FLEXCAN0_RXMGMASK != FLEXCAN_RXGMASK_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_RXGMASK is not correct %08X \n",FLEXCAN0_RXMGMASK);
  uiErrCount++;
 }
if(FLEXCAN0_RX14MASK != FLEXCAN_RX14MASK_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_RX14MASK is not correct %08X \n",FLEXCAN0_RX14MASK);
  uiErrCount++;
}
if(FLEXCAN0_RX15MASK != FLEXCAN_RX15MASK_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_RX15MASK is not correct %08X \n",FLEXCAN0_RX15MASK);
}
if(FLEXCAN0_ECR != FLEXCAN_ECR_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_ECR is not correct %08X \n",FLEXCAN0_ECR);
  uiErrCount++;
}
if(FLEXCAN0_ESR1 != FLEXCAN_ESR1_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_ESR1 is not correct %08X \n",FLEXCAN0_ESR1);
  uiErrCount++;
}
if(FLEXCAN0_IMASK1 != FLEXCAN_IMASK1_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_IMASK1 is not correct %08X \n",FLEXCAN0_IMASK1);
  uiErrCount++;
}
/* No IMASK2 and IFLAG2 due to only 16 MBs for P2 */
#if 0 
if(FLEXCAN0_IMASK2 != FLEXCAN_IMASK2_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_IMASK2 is not correct %08X \n",FLEXCAN0_IMASK2);
  uiErrCount++;
}

if(FLEXCAN0_IFLAG2 != FLEXCAN_IFLAG2_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_IFLAG2 is not correct %08X \n",FLEXCAN0_IFLAG2);
  uiErrCount++;
}
#endif

if(FLEXCAN0_IFLAG1 != FLEXCAN_IFLAG1_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_IFLAG1 is not correct %08X \n",FLEXCAN0_IFLAG1);
  uiErrCount++;
}

if(FLEXCAN0_CTRL2 != FLEXCAN_CTRL2_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_CTRL2 is not correct %08X \n",FLEXCAN0_CTRL2);
  uiErrCount++;
}

if(FLEXCAN0_ESR2 != FLEXCAN_ESR2_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_ESR2 is not correct %08X \n",FLEXCAN0_ESR2);
  uiErrCount++;
}

if(FLEXCAN0_FUREQ != FLEXCAN_FUREQ_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_FUREQ is not correct %08X \n",FLEXCAN0_FUREQ);
  uiErrCount++;
}

if(FLEXCAN0_FUACK != FLEXCAN_FUACK_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_FUACK is not correct %08X \n",FLEXCAN0_FUACK);
  uiErrCount++;
}

if(FLEXCAN0_CRCR != FLEXCAN_CRCR_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_CRCR is not correct %08X \n",FLEXCAN0_CRCR);
  uiErrCount++;
}

if(FLEXCAN0_RXFGMASK != FLEXCAN_RXFGMASK_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_CRCR is not correct %08X \n",FLEXCAN0_RXFGMASK);
  uiErrCount++;
}

#if 0
/* This register is only valid whilst the IFLAG[BUF5I] is asserted */
if(FLEXCAN0_RXFIR != FLEXCAN_RXFIR_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_RXFIR is not correct %08X \n",FLEXCAN0_RXFIR);
  printf("FLEXCAN0_RXFIR = %04X\n", *((uint8_t*)&FLEXCAN0_RXFIR));
  uiErrCount++;
}
#endif

if(FLEXCAN0_DBG1 != FLEXCAN_DBG1_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_DBG1 is not correct %08X \n",FLEXCAN0_DBG1);
  uiErrCount++;
}

if(FLEXCAN0_DBG2 != FLEXCAN_DBG2_INIT)
{
  error("FlexCAN0 register reset value FLEXCAN0_DBG2 is not correct %08X \n",FLEXCAN0_DBG2);
  uiErrCount++;
}

 /* For FlexCAN1*/
printf("Checking FLEXCAN1 registers...\n");
if(FLEXCAN1_MCR != FLEXCAN_MCR_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_MCR is not correct %08X \n",FLEXCAN1_MCR);
  uiErrCount++;
}

if(FLEXCAN1_CTRL1 != FLEXCAN_CTRL1_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_CTRL1 is not correct %08X \n",FLEXCAN1_CTRL1);
  uiErrCount++;
}
if(FLEXCAN1_TIMER != FLEXCAN_TIMER_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_TIMER is not correct %08X \n",FLEXCAN1_TIMER);
  uiErrCount++;
}
#if 0
printf("Checking FLEXCAN1_TCR...\n");
if(FLEXCAN1_TCR != FLEXCAN_TCR_INIT) //can be only accessed in TEST mode.reset value:0x00000000
{
  error("FlexCAN1 register reset value FLEXCAN1_TCR is not correct %08X \n",FLEXCAN1_TCR);
  uiErrCount++;
 }
printf("FLEXCAN1_TCR checking DONE!\n");
#endif
if(FLEXCAN1_RXMGMASK != FLEXCAN_RXGMASK_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_RXGMASK is not correct %08X \n",FLEXCAN1_RXMGMASK);
  uiErrCount++;
 }
if(FLEXCAN1_RX14MASK != FLEXCAN_RX14MASK_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_RX14MASK is not correct %08X \n",FLEXCAN1_RX14MASK);
  uiErrCount++;
}
if(FLEXCAN1_RX15MASK != FLEXCAN_RX15MASK_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_RX15MASK is not correct %08X \n",FLEXCAN1_RX15MASK);
}
if(FLEXCAN1_ECR != FLEXCAN_ECR_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_ECR is not correct %08X \n",FLEXCAN1_ECR);
  uiErrCount++;
}
if(FLEXCAN1_ESR1 != FLEXCAN_ESR1_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_ESR1 is not correct %08X \n",FLEXCAN1_ESR1);
  uiErrCount++;
}
if(FLEXCAN1_IMASK1 != FLEXCAN_IMASK1_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_IMASK1 is not correct %08X \n",FLEXCAN1_IMASK1);
  uiErrCount++;
}
/* No IMASK2 and IFLAG2 due to only 16 MBs for P2 */
#if 0 
if(FLEXCAN1_IMASK2 != FLEXCAN_IMASK2_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_IMASK2 is not correct %08X \n",FLEXCAN1_IMASK2);
  uiErrCount++;
}

if(FLEXCAN1_IFLAG2 != FLEXCAN_IFLAG2_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_IFLAG2 is not correct %08X \n",FLEXCAN1_IFLAG2);
  uiErrCount++;
}
#endif

if(FLEXCAN1_IFLAG1 != FLEXCAN_IFLAG1_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_IFLAG1 is not correct %08X \n",FLEXCAN1_IFLAG1);
  uiErrCount++;
}

if(FLEXCAN1_CTRL2 != FLEXCAN_CTRL2_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_CTRL2 is not correct %08X \n",FLEXCAN1_CTRL2);
  uiErrCount++;
}

if(FLEXCAN1_ESR2 != FLEXCAN_ESR2_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_ESR2 is not correct %08X \n",FLEXCAN1_ESR2);
  uiErrCount++;
}

if(FLEXCAN1_FUREQ != FLEXCAN_FUREQ_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_FUREQ is not correct %08X \n",FLEXCAN1_FUREQ);
  uiErrCount++;
}

if(FLEXCAN1_FUACK != FLEXCAN_FUACK_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_FUACK is not correct %08X \n",FLEXCAN1_FUACK);
  uiErrCount++;
}

if(FLEXCAN1_CRCR != FLEXCAN_CRCR_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_CRCR is not correct %08X \n",FLEXCAN1_CRCR);
  uiErrCount++;
}

if(FLEXCAN1_RXFGMASK != FLEXCAN_RXFGMASK_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_CRCR is not correct %08X \n",FLEXCAN1_RXFGMASK);
  uiErrCount++;
}
#if 0
/* This register is only valid whilst the IFLAG[BUF5I] is asserted */
printf("FLEXCAN1_RXFGMASK checking PASS!\n");
if(FLEXCAN1_RXFIR != FLEXCAN_RXFIR_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_RXFIR is not correct %08X \n",FLEXCAN1_RXFIR);

  uiErrCount++;
}
#endif

if(FLEXCAN1_DBG1 != FLEXCAN_DBG1_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_DBG1 is not correct %08X \n",FLEXCAN1_DBG1);
  uiErrCount++;
}

if(FLEXCAN1_DBG2 != FLEXCAN_DBG2_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_DBG2 is not correct %08X \n",FLEXCAN1_DBG2);
  uiErrCount++;
}

 /* For FlexCAN1*/
if(FLEXCAN1_MCR != FLEXCAN_MCR_INIT)
{
  error("FlexCAN1 register reset value FLEXCAN1_MCR is not correct %08X \n",FLEXCAN1_MCR);
  uiErrCount++;
}
	PrintPassFailMessage(uiErrCount);
}

void FlexCAN_Registers_Test(void)
{
  uint32_t i,j;
  uint16_t uiErrCount = 0;
  uint32_t uiMatchValue;
  uint8_t  iFilterTableElementsNoList[] = {8,16,24,32,40};
  
  printf("Start testing register access...\n");	
  acces_mode = 0;
  
#if TEST_FLEXCAN0
  
  /* For FlexCAN0 */
  
#ifndef	USE_EXTERNAL_CLOCK  
  FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC;	// bus clock
#endif   
  
  FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS;

  while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR));
  
  FLEXCAN0_MCR |= FLEXCAN_MCR_WRN_EN;
  
#ifndef	USE_EXTERNAL_CLOCK 
  FLEXCAN0_CTRL1 ^= FLEXCAN_CTRL_CLK_SRC;
  
  if(!(FLEXCAN_CTRL_CLK_SRC & FLEXCAN0_CTRL1))
  {
  	uiErrCount++;
  	printf("Error: FLEXCAN0_CTRL1[CLK_SRC] bit can be written in Freeze mode\n ");  	
  }
#endif
   
  FLEXCAN0_CTRL1 = ~(0x00000000);
  
  if(FLEXCAN0_CTRL1 != 0xFFFFFCFF) 
  {
  	uiErrCount++;
  	printf("FLEXCAN0_CTRL1 = %#08.8x\n ",FLEXCAN0_CTRL1);
  }
  
#ifndef	USE_EXTERNAL_CLOCK   
  FLEXCAN0_CTRL1 = 0x0;  // for build 0.9, it is ok
  if(FLEXCAN0_CTRL1 !=  0x00002000L)
  {
  	printf("Error: FLEXCAN0_CTRL1 = %#08.8x,expect =  0x00002000\n",FLEXCAN0_CTRL1);
  	uiErrCount++;
  } 
#endif  
  // for build 1.0
  FLEXCAN0_CTRL1 |=     
 					   FLEXCAN_CTRL_PRESDIV(50L)  |
 					   FLEXCAN_CTRL_RJW(0x1)	  |
 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
 					  // FLEXCAN_CTRL_BOFF_REC	  |
 					  // FLEXCAN_CTRL_LBUF		  |	// with LBUF: D_IP_FlexCAN3_SYN - d_ip_flexcan3_syn.01.00.06.00 has bug
 					   FLEXCAN_CTRL_LPB			  | /* Enable Internal loopback */
 					   FLEXCAN_CTRL_PROPSEG(0x2) ;

  printf("FLEXCAN0_MCR = %#08.8x,FLEXCAN0_CTRL1=%#08.8x\n",FLEXCAN0_MCR,FLEXCAN0_CTRL1);
   
  printf("FLEXCAN0_CTRL1=%#08.8x\r\n",FLEXCAN0_CTRL1);

  uiMatchValue = FLEXCAN0_MCR;
  
  FLEXCAN0_TIMER = ~(0x00000000);
  while((FLEXCAN0_TIMER & 0xFFFF) != 0xFFFF);
  
  
  if(FLEXCAN0_TIMER != 0xFFFF)
  {
  	uiErrCount++;
  	printf("FLEXCAN0_TIMER = %#08.8x\n ",FLEXCAN0_TIMER);
  }
  
#if 0
  FLEXCAN0_RXMGMASK = ~(0xFFFFFFFF);
 
  if(FLEXCAN0_RXMGMASK != 0x00000000)
  {
  	uiErrCount++;
  	 printf("FLEXCAN0_RXMGMASK = %#08.8x\n ",FLEXCAN0_RXMGMASK);
  }  
#endif
  FLEXCAN0_RX14MASK = ~(0xFFFFFFFF);
  
  if(FLEXCAN0_RX14MASK != 0x00000000) 
  {
  	uiErrCount++;
  	printf("FLEXCAN0_RX14MASK = %#08.8x\n ",FLEXCAN0_RX14MASK);
  }    

  FLEXCAN0_RX15MASK = ~(0xFFFFFFFF);
  
  if(FLEXCAN0_RX15MASK != 0x00000000)
  {
  	uiErrCount++;
  	printf("FLEXCAN0_RX15MASK = %#08.8x\n ",FLEXCAN0_RX15MASK);
  }  
  printf("Before writing ~(0x00000000) to ECR,FLEXCAN0_ESR1 = %#08.8x\n",FLEXCAN0_ESR1);
  
  
  FLEXCAN0_ECR = ~(0x00000000);
  while((FLEXCAN0_ECR & 0xFFFF) != 0xFFFF);

  if(FLEXCAN0_ECR != 0xFFFF)
  {
  	uiErrCount++;
  	printf("FLEXCAN0_ECR = %#08.8x\n ",FLEXCAN0_ECR);
  }    
  printf("After writing %#08.8x to ECR,FLEXCAN0_ESR1 = %#08.8x\n",FLEXCAN0_ECR,FLEXCAN0_ESR1);
  
  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	}
	// Leave freeze mode
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
	while((FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR)){}	
	while((FLEXCAN_MCR_NOT_RDY & FLEXCAN0_MCR)){}	
	
	// check ESR1 again
  if(FLEXCAN0_ESR1 != 0x00030310)
  {
  	uiErrCount++;
    printf("Error: FLEXCAN0_ESR1 = %#08.8x,expected = 0x00030310 after leaving freeze mode \n ",FLEXCAN0_ESR1); 	
  }
  // Now put into freeze mode again
  FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR)){}	
  
  
  FLEXCAN0_ESR1 = ~(0x00000000);
 // delay  
  for(i=0;i<1000;i++);
  if(FLEXCAN0_ESR1 != 0x00000310)  
  {
  	uiErrCount++;
    printf("Error: FLEXCAN0_ESR1 = %#08.8x,expected = 0x00000310\n ",FLEXCAN0_ESR1);
  }  
  
  // Change both counters to below 127.
  FLEXCAN0_ECR = (126<<8) | 126;
  
  if(FLEXCAN0_ECR != ((126L<<8)|126L))
  {
  	uiErrCount++;
    printf("FLEXCAN0_ECR = %#08.8x,not (126<<8)| 126 \n ",FLEXCAN0_ECR); 	
  }
  // delay  
  for(i=0;i<1000;i++);
  
  // Check ESR1 to see if it is error active
  if(FLEXCAN0_ESR1 != 0x00000300L)
  {
  	uiErrCount++;
    printf("FLEXCAN0_ESR1 = %#08.8x, not 0x300 \n",FLEXCAN0_ESR1); 	
  }
 #if 0
  // Increase TX_ERR_CNT  to be greater than 255
  FLEXCAN0_ECR = 0x7EFF;
  *((uint8_t*)&FLEXCAN0_ECR) += 1;
  
  if(FLEXCAN0_ECR != ((126L<<8)))
  {
  	uiErrCount++;
    printf("FLEXCAN0_ECR = %#08.8x,not (126<<8) \n ",FLEXCAN0_ECR); 	
  }  
 // delay  
  for(i=0;i<1000;i++);

  // Check ESR1 to see if it is bus off active
  if(FLEXCAN0_ESR1 != 0x00000324)
  {
  	uiErrCount++;
    printf("FLEXCAN0_ESR1 = %#08.8x, not 0x324 \n ",FLEXCAN0_ESR1); 	
  } 
#endif  
  
  FLEXCAN0_IMASK1 = ~(0x00000000);
 
  if(FLEXCAN0_IMASK1 != 0xFFFF) 
  {
  	uiErrCount++;
    printf("FLEXCAN0_IMASK1 = %#08.8x\n",FLEXCAN0_IMASK1);
   }  
  FLEXCAN0_IFLAG1 = ~(0x00000000);
  if(FLEXCAN0_IFLAG1 != 0x0)
  {
  	uiErrCount++;
    printf("FLEXCAN0_IFLAG1 = %#08.8x\n",FLEXCAN0_IFLAG1);
  }  
  FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;
 // delay  
  for(i=0;i<1000;i++);
 #if 1
  for(i=0;i<16;i++) // only 16 RXIMRs are supported in P2
 #else 
  for(i=0;i<17;i++)		
 #endif  
  {
	  FLEXCAN0_RXIMRn(i) = 0xFF345678 + i;
	  if( FLEXCAN0_RXIMRn(i) != (0xFF345678 + i))
	  {
	    printf("Error: FLEXCAN0_RXIMR%d = %#08.8x,expect = %#08.8x\n",i,FLEXCAN0_RXIMRn(i),
	    		0xFF345678 + i);
	  	uiErrCount++;
	  }  	  
  }
  // Enable Rx FIFO
  FLEXCAN0_MCR |= FLEXCAN_MCR_FEN;
  printf("Rx FIFO is enabled\n");
  
  for(j = 0; j <= sizeof(iFilterTableElementsNoList)/sizeof(uint8_t); j++)
  {
	  FLEXCAN0_CTRL2 = (FLEXCAN0_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | (j<<FLEXCAN_CTRL2_RFFN_BIT_NO);
	  
	  // Filter table elements 
	  for(i=0;i<iFilterTableElementsNoList[j];i++)		// 
	  {
	  		FLEXCAN0_IDFLT_TAB(i) = 0xA5A5B8B8L;
	  		if( FLEXCAN0_IDFLT_TAB(i) != 0xA5A5B8B8L)
	  		{
		 	    printf("Error: FLEXCAN0_IDFLT_TAB(%d) = %#08.8x,expect = 0xA5A5B8B8L\n",i,FLEXCAN0_IDFLT_TAB(i));
			  	uiErrCount++; 			
	  		}
	  }  
  }
  FLEXCAN0_CTRL2 = (FLEXCAN0_CTRL2 & ~(FLEXCAN_CTRL2_RFFN));
  
  // disable Rx FIFO
  FLEXCAN0_MCR ^= FLEXCAN_MCR_FEN;
  
  printf("Rx FIFO is disabled\n");
  // delay  
  for(i=0;i<1000;i++);
  
  //
  //printf("FLEXCAN_CTRL2 = %#08.8x\n",FLEXCAN0_CTRL2);
  printf("start to test MBn read/write...\n");
  //NOTE: for Tx MBs (see Table 5), the BUSY bit should be ignored upon read, 
  // except when AEN bit is set in the MCR register.
  // MUST enable AEN to allow writing 1 to bit 24 or bit 24 will be cleared  
  FLEXCAN0_MCR |= FLEXCAN_MCR_AEN;	// enable abort
	
// build 1.0 changed the behavior of CS bit 24. only when AEN = 1, this bit can be written
 
  for(i=0;i<16;i++)
  {
 
  	FLEXCAN0_MBn_CS(i) = 0xFFAA5500+i;

	if(FLEXCAN0_MCR & FLEXCAN_MCR_AEN)
	{
		uiMatchValue = 0xFFAA5500 + i;
	}
	else
	{
		uiMatchValue = 0xFEAA5500 + i;	// bit 24 is cleared	
	}
	
    if( FLEXCAN0_MBn_CS(i) != uiMatchValue)
    {
		printf("Error: FLEXCAN0_MB%d_CS = %#08.8x,expected = %#08.8x\n",i, FLEXCAN0_MBn_CS(i),uiMatchValue);
		uiErrCount++;
	}

  	FLEXCAN0_MBn_ID(i) = uiMatchValue = 0xFFAA5500+i;
    if( FLEXCAN0_MBn_ID(i) != uiMatchValue) 
    {
		printf("Error: FLEXCAN0_MB%d_ID = %#08.8x,expect = %#08.8x\n",i,FLEXCAN0_MBn_ID(i),uiMatchValue); 
		uiErrCount++;   	
    }

  	FLEXCAN0_MBn_WORD0(i) = uiMatchValue = 0xFF345678 + i;
	

    if( FLEXCAN0_MBn_WORD0(i) != uiMatchValue)
    {
 		printf("Error: FLEXCAN0_MB%d_WORD0 = %#08.8x,expect = %#08.8x\n",i,FLEXCAN0_MBn_WORD0(i),uiMatchValue);
 		uiErrCount++;
    }

  	FLEXCAN0_MBn_WORD1(i) = uiMatchValue;
    if( FLEXCAN0_MBn_WORD1(i) != uiMatchValue)
    {
 		printf("Error: FLEXCAN0_MB%d_WORD1 = %#08.8x,expect = %#08.8x\n",i,FLEXCAN0_MBn_WORD1(i), uiMatchValue);
 		uiErrCount++;
    }
  }
 
 
  printf("MBn read/write OK\n");
  
  FLEXCAN0_CTRL2 = 0x84870003L;
  if(FLEXCAN0_CTRL2 != 0x84870003L)
  {
	  printf("Error: FLEXCAN0_CTRL2 is not correct %#08.8x,expect = 0x84870003L \n",FLEXCAN0_CTRL2);
	  uiErrCount++;
  }
  
 
  FLEXCAN0_MCR = FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT | FLEXCAN_MCR_DOZE 	|
  					   FLEXCAN_MCR_FEN 		|
  					   FLEXCAN_MCR_SRX_DIS  | 
  					   FLEXCAN_MCR_IRMQ 		| 
  					   FLEXCAN_MCR_LPRIO_EN	|
  					   FLEXCAN_MCR_AEN		|
  					   FLEXCAN_MCR_IDAM(0x3) |
  					   FLEXCAN_MCR_MAXMB(8);
  
  if(FLEXCAN0_MCR != 0x79073308L)
  {
 		printf("Error: FLEXCAN0_MCR = %#08.8x\n",FLEXCAN0_MCR);  
 		uiErrCount++; 	  	
  }  					   
  					   
  FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT | FLEXCAN_MCR_SUPV |
                   FLEXCAN_MCR_FEN |  FLEXCAN_MCR_SRX_DIS  | 
				   FLEXCAN_MCR_IRMQ | FLEXCAN_MCR_LPRIO_EN	| FLEXCAN_MCR_AEN | 
                   FLEXCAN_MCR_IDAM(0x3) | 
                   FLEXCAN_MCR_MAXMB(0x7F));
  
  while(FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR);                
                   
  FLEXCAN0_MCR |= (FLEXCAN_MCR_DOZE 	|
  					   FLEXCAN_MCR_FEN 		|
  					   FLEXCAN_MCR_SRX_DIS  | 
  					   FLEXCAN_MCR_IRMQ 		| 
  					   FLEXCAN_MCR_LPRIO_EN	|
  					   FLEXCAN_MCR_AEN		|
  					   FLEXCAN_MCR_IDAM(0x3) |
  					   FLEXCAN_MCR_MAXMB(8)
  					   );
  // delay  
  //for(i=0;i<1000;i++);

   if(FLEXCAN0_MCR != 0x00840077L)
   {
 		printf("Error: FLEXCAN0_MCR = %#08.8x\n",FLEXCAN0_MCR);  
 		uiErrCount++; 	
   }
#endif
#if TEST_FLEXCAN1   
   printf("start to test FlexCAN1 register read/write\n");
  
  /* For FlexCAN1 */
#ifndef	USE_EXTERNAL_CLOCK  
  FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC;	// bus clock
#endif   
  
  FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS;
  while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR));
  printf("FLEXCAN1_MCR = %#08.8x\n",FLEXCAN1_MCR);
  
  FLEXCAN1_MCR |= FLEXCAN_MCR_WRN_EN;

#if GENERATE_HARD_FAULT
  FLEXCAN1_CTRL1 ^= FLEXCAN_CTRL_CLK_SRC;
  if(!(FLEXCAN_CTRL_CLK_SRC & FLEXCAN1_CTRL1))
  {
  	uiErrCount++;
  	printf("Error: FLEXCAN1_CTRL1[CLK_SRC] bit can be written in Freeze mode\n ");  	
  } 
 
  FLEXCAN1_CTRL1 = ~(0x00000000);
 
  if(FLEXCAN1_CTRL1 != 0xFFFFFCFF)
  {
     printf("Error: FLEXCAN1_CTRL = %#08.8x\n",FLEXCAN1_CTRL1); 
     uiErrCount++; 		
  }
  
  FLEXCAN1_CTRL1 = 0x0;
  if(FLEXCAN1_CTRL1 != 0x00002000L)
  {
  	printf("Error: FLEXCAN1_CTRL1 = %#08.8x,expect = 0x00002000\n",FLEXCAN1_CTRL1);
  	uiErrCount++;
  }
#endif  
  FLEXCAN1_CTRL1 |=     
 					   FLEXCAN_CTRL_PRESDIV(50L)  |
 					   FLEXCAN_CTRL_RJW(0x1)	  |
 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
 					   FLEXCAN_CTRL_BOFF_REC	  |
 					  // FLEXCAN_CTRL_LBUF		  |	// with LBUF: D_IP_FlexCAN3_SYN - d_ip_flexcan3_syn.01.00.06.00 has bug
 					   FLEXCAN_CTRL_LPB			  | /* Enable Internal loopback */
 					   FLEXCAN_CTRL_PROPSEG(0x2) ; 					   
  
  FLEXCAN1_TIMER = ~(0x00000000);
  while((FLEXCAN1_TIMER & 0xFFFF) != 0xFFFF);
  if(FLEXCAN1_TIMER != 0xFFFF)
  {
      printf("Error: FLEXCAN1_TIMER = %#08.8x\n",FLEXCAN1_TIMER);
      uiErrCount++;
  }

  FLEXCAN1_RXMGMASK = ~(0xFFFFFFFF);
  if(FLEXCAN1_RXMGMASK != 0x00000000)
  {
	  printf("Error: FLEXCAN1_RXMGMASK = %#08.8x\n",FLEXCAN1_RXMGMASK);  	
     uiErrCount++; 		
  }

  FLEXCAN1_RX14MASK = ~(0xFFFFFFFF);
   if(FLEXCAN1_RX14MASK != 0x00000000)
   {
   	 printf("Error: FLEXCAN1_RX14MASK = %#08.8x\n",FLEXCAN1_RX14MASK);  
   	 uiErrCount++; 		 	
   }

  FLEXCAN1_RX15MASK = ~(0xFFFFFFFF);
  if(FLEXCAN1_RX15MASK != 0x00000000)
  {
  	  printf("Error: FLEXCAN1_RX15MASK = %#08.8x\n",FLEXCAN1_RX15MASK);  	
     uiErrCount++; 		
  }
  
  FLEXCAN1_ECR = ~(0x00000000);
  while((FLEXCAN1_ECR & 0xFFFF) != 0xFFFF);
  if(FLEXCAN1_ECR != 0xFFFF)
  {
     printf("Error: FLEXCAN1_ECR = %#08.8x\n",FLEXCAN1_ECR); 	
      uiErrCount++; 		
  }
  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	// MB0 is active
	FLEXCAN1_MBn_CS(0) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
	
	// Leave freeze mode
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
	while((FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR)){}	
	while((FLEXCAN_MCR_NOT_RDY & FLEXCAN1_MCR)){}	
	
	// now enter freeze mode
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
	while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR)){}		
	  
  FLEXCAN1_ESR1 = ~(0x00000000);
  if(FLEXCAN1_ESR1 != 0x00000310)
  {
     printf("Error: FLEXCAN1_ESR1 = %#08.8x\n",FLEXCAN1_ESR1);
      uiErrCount++; 		
  }
  
  FLEXCAN1_ESR2 = ~(0x00000000);
  if(FLEXCAN1_ESR2 != 0x00016000)
  {
     printf("Error: FLEXCAN1_ESR2 = %#08.8x\n",FLEXCAN1_ESR2);
     uiErrCount++; 		
  }  
  FLEXCAN1_IMASK1 = ~(0x00000000);
  if(FLEXCAN1_IMASK1 != 0xFFFF)
  {
  	 printf("Error: FLEXCAN1_IMASK1 = %#08.8x\n",FLEXCAN1_IMASK1);  	
     uiErrCount++; 		
  }
  
  FLEXCAN1_IFLAG1 = ~(0x00000000);
  if(FLEXCAN1_IFLAG1 != 0x0)
  {
     printf("Error: FLEXCAN1_IFLAG1 = %#08.8x\n",FLEXCAN1_IFLAG1);  	
      uiErrCount++; 		
  }
  
  FLEXCAN1_MCR |= FLEXCAN_MCR_IRMQ;
  
  for(i=0;i<16;i++)
  {
	  FLEXCAN1_RXIMRn(i) = 0x12345678 + i;
	  if(FLEXCAN1_RXIMRn(i) != 0x12345678 + i)
	  {
	  	  printf("Error: FLEXCAN1_RXIMR%d = %#08.8x\n",i,FLEXCAN1_RXIMRn(i));
		  uiErrCount++; 		
	  }
  }
  // Enable Rx FIFO
  FLEXCAN1_MCR |= FLEXCAN_MCR_FEN;
  printf("Rx FIFO is enabled\n");
  
  for(j = 0; j <= sizeof(iFilterTableElementsNoList)/sizeof(uint8_t); j++)
  {
	  FLEXCAN1_CTRL2 = (FLEXCAN1_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | (j<<FLEXCAN_CTRL2_RFFN_BIT_NO);
	  
	  // Filter table elements 
	  for(i=0;i<iFilterTableElementsNoList[j];i++)		// 
	  {
	  		FLEXCAN1_IDFLT_TAB(i) = 0xA5A5B8B8L;
	  		if( FLEXCAN1_IDFLT_TAB(i) != 0xA5A5B8B8L)
	  		{
		 	    printf("Error: FLEXCAN1_IDFLT_TAB(%d) = %#08.8x,expect = 0xA5A5B8B8L\n",i,FLEXCAN1_IDFLT_TAB(i));
			  	uiErrCount++; 			
	  		}
	  }  
  }
  FLEXCAN1_CTRL2 = (FLEXCAN1_CTRL2 & ~(FLEXCAN_CTRL2_RFFN));
  
  // disable Rx FIFO
  FLEXCAN1_MCR ^= FLEXCAN_MCR_FEN;
  
  printf("Rx FIFO is disabled\n");
  // delay  
  for(i=0;i<1000;i++);
  
  //NOTE: for Tx MBs (see Table 5), the BUSY bit should be ignored upon read, 
  // except when AEN bit is set in the MCR register.
  // MUST enable AEN to allow writing 1 to bit 24 or bit 24 will be cleared
  FLEXCAN1_MCR |= FLEXCAN_MCR_AEN;	// enable abort
  
  for(i=0;i<16;i++)
  {
  	FLEXCAN1_MBn_CS(i) = 0xFFAA5500;
    if( FLEXCAN1_MBn_CS(i) != 0xFFAA5500)
    {
	     printf("Error: FLEXCAN1_MB%d_CS = %#08.8x\n",i,FLEXCAN1_MBn_CS(i)); 
	     uiErrCount++; 		   	
    }

  	FLEXCAN1_MBn_ID(i) = 0xFFAA5500;
    if( FLEXCAN1_MBn_ID(i) != 0xFFAA5500)
    {
		printf("Error: FLEXCAN1_MB%d_ID = %#08.8x\n",i,FLEXCAN1_MBn_ID(i));  
		
		uiErrCount++; 		  	
    }

  	FLEXCAN1_MBn_WORD0(i) = 0x12345678 + i;
    if(FLEXCAN1_MBn_WORD0(i) != 0x12345678 + i)
    {
		printf("Error: FLEXCAN1_MB%d_WORD0 = %#08.8x\n",i,FLEXCAN1_MBn_WORD0(i));
		uiErrCount++; 		
    }

  	FLEXCAN1_MBn_WORD1(i) = 0x12345678 + i;
    if(FLEXCAN1_MBn_WORD1(i) != 0x12345678 + i)
    {
		printf("Error: FLEXCAN1_MB%d_WORD1 = %#08.8x\n",i,FLEXCAN1_MBn_WORD1(i));
	    uiErrCount++; 		
    }
  }
  
  // clear AEN bit
  FLEXCAN1_MCR ^= FLEXCAN_MCR_AEN;
  
  // leave freeze mode
  FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT | FLEXCAN_MCR_SUPV | FLEXCAN_MCR_MAXMB(0x3F));
	while((FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR)){}	
	while((FLEXCAN_MCR_NOT_RDY & FLEXCAN1_MCR)){}	
  
  FLEXCAN1_MCR |= (FLEXCAN_MCR_DOZE 	|
  					   FLEXCAN_MCR_FEN 		|
  					   FLEXCAN_MCR_SRX_DIS  | 
  					   FLEXCAN_MCR_IRMQ 		| 
  					   FLEXCAN_MCR_LPRIO_EN	|
  					   FLEXCAN_MCR_AEN		|
  					   FLEXCAN_MCR_IDAM(0x3)
  					   );
  for(i=0;i<1000;i++);	// delay for some time
  if(FLEXCAN1_MCR != 0x00250030L)
  {
  	  printf("Error: FLEXCAN1_MCR = %#08.8x\n",FLEXCAN1_MCR);  
  	  uiErrCount++; 		
  }
  
  // Put FlexCAN to freeze mode
  FLEXCAN1_MCR |= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
  while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR)){}
 
  // Configure AIPS_lite to allow user mode access
  ConfigureAIPS_Lite(); 
  		
  // change FlexCAN to user mode
  FLEXCAN1_MCR &= ~(FLEXCAN_MCR_SUPV);
  printf("FLEXCAN1_MCR = %#08.8x\n",FLEXCAN1_MCR);
  printf("Now change access mode...\n");
  // Now enter user mode
  i = EnterUserMode();
// i =  EnterPrivilegeMode();
 
  
  printf("Access mode is changed, previous CONTROL register = %#08.8x\n",i);
    
  i = GetControlRegister();
  printf("Current Control register = %#08.8x\n",i);
  acces_mode = 1;
  guiMB_ISR_Count = 0;
  guiErrCount = 0;

  printf("Access to user mode registers...\n");
 // Access to user mode registers
  i = FLEXCAN1_CTRL1;
  uiMatchValue = FLEXCAN1_ECR;
  uiMatchValue = FLEXCAN1_ESR1; 
  uiMatchValue = FLEXCAN1_IFLAG1;
  
  FLEXCAN1_MBn_CS(0) = 0;
  FLEXCAN1_MBn_ID(0) = 0;
  FLEXCAN1_MBn_WORD0(0) = 0;

   uiMatchValue = FLEXCAN1_CTRL2;
   uiMatchValue = FLEXCAN1_ESR2;
  
    printf("Done to access to user mode registers\n");
    printf("Access to supervisor mode registers: MCR ...\n");
 
  // access of MCR will incur hard fault
  uiMatchValue = FLEXCAN1_MCR;  
 // FLEXCAN1_MCR = 0;
  
  if(guiMB_ISR_Count<2)
  {
  	uiErrCount++;
  	printf("Error: Hard fault ISR not happened when accessing MCR in user mode!\n");
  }
  uiErrCount +=  guiErrCount;
#endif  
  PrintPassFailMessage(uiErrCount);
}



void FlexCAN1_SelfLoop0(void)
{
	 uint32_t i, cs[4],id[4],data0[4],data1[4], timer[4];
	 uint16_t uiErrCount = 0;
	 uint32_t uiMatchValue[4];
	 	 
	 printf("Start to test loopback mode for FlexCAN1 with LBUF = 1!\n");
	 /* FlexCAN1 Settings */

	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; // enable module
		 

	 while((FLEXCAN_MCR_LPM_ACK & FLEXCAN1_MCR));	

	// Now can apply Soft Reset
	FLEXCAN1_MCR ^= FLEXCAN_MCR_SOFT_RST;
	while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 	
         printf("After soft reset, FLEXCAN1_MCR =%#08.8x \n",FLEXCAN1_MCR);
	
	 // Now it should be in Freeze mode  
	 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR));

	#ifdef	TEST_MESSAGE_QUEUE
		FLEXCAN1_MCR |= FLEXCAN_MCR_IRMQ;
	#endif
	// printf("FLEXCAN1_MCR =%#8x \n",FLEXCAN0_MCR);
	 	 
	 FLEXCAN1_CTRL1 |=     //(FLEXCAN_CTRL_PRESDIV(1L)  |	// 24M IP clock
	 					   (FLEXCAN_CTRL_PRESDIV(50L)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |	// with LBUF: D_IP_FlexCAN3_SYN - d_ip_flexcan3_syn.01.00.06.00 has bug
	 					   FLEXCAN_CTRL_LPB			  | /* Enable Internal loopback */
	 					   FLEXCAN_CTRL_PROPSEG(0x2) 
					   );
    printf("FLEXCAN1_CTRL1 =%#08x\n",FLEXCAN1_CTRL1);					   
  /* Initialize all 16 MBs */		  
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	 
	 FLEXCAN1_RXMGMASK = 0x1FFBFFFF;
	 FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	 if(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ)
	 {
	 	FLEXCAN1_RXIMRn(1) = 0x1FFBFFFFL;
	 	FLEXCAN1_RXIMRn(3) = 0x1FFBFFFFL;	 	 
	 }	
	 	 
		  
    /* De-assert Freeze Mode */
     FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
     
     /* wait till exit of freeze mode */
     while( FLEXCAN1_MCR & FLEXCAN_MCR_FRZ_ACK);

 	 // Configure MB1 and MB3 as rx MBs
 	 // MB0 and MB2 as tx MBs
   	 FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x0);
   	 FLEXCAN1_MBn_ID(1) = FLEXCAN_MB_ID_IDSTD(0xF);
 //	 FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x4);

   	 FLEXCAN1_MBn_CS(3) = FLEXCAN_MB_CS_CODE(0x0);
   	 FLEXCAN1_MBn_ID(3) = FLEXCAN_MB_ID_IDSTD(0xF);
   	 
  	 FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x4);
   	 FLEXCAN1_MBn_CS(3) = FLEXCAN_MB_CS_CODE(0x4);
 

 	 FLEXCAN1_MBn_CS(0) = (FLEXCAN_MB_CS_CODE(0x8) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN1_MBn_ID(0) = FLEXCAN_MB_ID_IDSTD(0xE);
	 FLEXCAN1_MBn_WORD0(0) = 0x12345678;
	 FLEXCAN1_MBn_WORD1(0) = 0x11223344;
	 

 	 FLEXCAN1_MBn_CS(2) = (FLEXCAN_MB_CS_CODE(0x8) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN1_MBn_ID(2) = FLEXCAN_MB_ID_IDSTD(0xE);
	 FLEXCAN1_MBn_WORD0(2) = 0x5A5A5A5A;
	 FLEXCAN1_MBn_WORD1(2) = 0x5555AAAA;

	 /* Pass condition:  */
	if(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ)
	{
		uiMatchValue[0] = FLEXCAN1_MBn_WORD0(0);
		uiMatchValue[1] = FLEXCAN1_MBn_WORD1(0);
		uiMatchValue[2] = FLEXCAN1_MBn_WORD0(2);
		uiMatchValue[3] = FLEXCAN1_MBn_WORD1(2);
	}
	else
	{
		uiMatchValue[0] = FLEXCAN1_MBn_WORD0(2);
		uiMatchValue[1] = FLEXCAN1_MBn_WORD1(2);
		uiMatchValue[2] = FLEXCAN1_MBn_WORD0(2);
		uiMatchValue[3] = FLEXCAN1_MBn_WORD1(2);		
	}

	 FLEXCAN1_MBn_CS(0) = (FLEXCAN_MB_CS_CODE(0xC) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN1_MBn_CS(2) |= FLEXCAN_MB_CS_CODE(0xC); 

	// for(i=0;i<500;i++);	// delay for a while
	 
	 /* Poll for MB0  Completion */
	 while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF0I));
 	 /* Poll for MB2  Completion */
	 while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF2I));
 
	 /* Poll for MB1 and MB3 Completion */
	 while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF1I));

	 /* Read received frame */
	 cs[1] = FLEXCAN1_MBn_CS(1);
	 id[1]= FLEXCAN1_MBn_ID(1);
	 data0[1] = FLEXCAN1_MBn_WORD0(1);
	 data1[1] = FLEXCAN1_MBn_WORD1(1);
	 
	 timer[1] = FLEXCAN1_TIMER;
	// clear the flags
	FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF1I;
	
	
	//printf("MB1 received a message!\n");
	/* If message queue is enabled, wait for queue elements done */ 
	if(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ )
	{	
	 	while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF3I));
		// clear the flags
		FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF3I; 	
		printf("MB3 received a message!\n");
	}
	else
	{
		 /* Poll for MB1 for 2nd transfer Completion */
		 while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF1I));
	
		 /* Read received frame */
		 cs[1] = FLEXCAN1_MBn_CS(1);
		 id[1]= FLEXCAN1_MBn_ID(1);
		 data0[1] = FLEXCAN1_MBn_WORD0(1);
		 data1[1] = FLEXCAN1_MBn_WORD1(1);
		 
		 timer[1] = FLEXCAN1_TIMER;
		// clear the flags
		FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF1I;		
		printf("MB1 received a message\n");
	}	 	 

	 printf("CS word = %#08.8x\n", cs[1]);
	 if(data0[1] != uiMatchValue[0])
	 {
	 	error("Error: Data Mismatch in MB1 word0,received = %#08.8x,expected = %#08.8x!\n",data0[1],uiMatchValue[0]);
	 	uiErrCount++;
	 }
	 
	 //printf("data received = %#08.8x\n", data1[1]);
	 if(data1[1] != uiMatchValue[1])
	 {
	 	error("Error: Data Mismatch in MB1 word1,received = %#08.8x,expected = %#08.8x!\n",data1[1],uiMatchValue[1]);
	 	uiErrCount++;
	 }
	 
	 if(((id[1] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
	 {
	 	error("Error: MB1 got wrong ID = %#08.8x\n",id[1]);
	 	uiErrCount++;
	 }
	 
	 printf("MB1: timer  = %#08.8x\n\n", timer[1]);

	if(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ )
	{	
		 cs[2] = FLEXCAN1_MBn_CS(3);
		 id[2]= FLEXCAN1_MBn_ID(3);
		 data0[2] = FLEXCAN1_MBn_WORD0(3);
		 data1[2] = FLEXCAN1_MBn_WORD1(3);
		 timer[2] = FLEXCAN1_TIMER;

		 if(data0[2] != uiMatchValue[2])
		 {
		 	error("Error: Data Mismatch in MB3 word0!\n");
		 	uiErrCount++;
		 }
		 
		 if(data1[2] != uiMatchValue[3])
		 {
		 	error("Error: Data Mismatch in MB3 word1!\n");
		 	uiErrCount++;
		 }
	 	 if(((id[2] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
		 {
		 	error("Error: MB3 got wrong ID = %#08.8x\n",id[2]);
		 	uiErrCount++;
		 }		 
		printf("MB3: timer  = %#08.8x\n", timer[2]);
	}
	
	PrintPassFailMessage(uiErrCount);	 
}

/* without LBUF set */
void FlexCAN1_SelfLoop01(void)
{
	 uint32_t i;
	 
	 guiErrCount = 0;
	 guiMB_ISR_Count = 0;
	 	 
	 printf("Start to test loopback mode for FlexCAN1 without LBUF set\n");
	 /* FlexCAN1 Settings */
#ifndef  USE_EXTERNAL_CLOCK
         
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
#endif	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; // enable module
		 

	 while((FLEXCAN_MCR_LPM_ACK & FLEXCAN1_MCR));	

	// Now can apply Soft Reset
	FLEXCAN1_MCR ^= FLEXCAN_MCR_SOFT_RST;
	while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 	
    //printf("After soft reset, FLEXCAN1_MCR =%#08.8x \n",FLEXCAN0_MCR);
	
	 // Now it should be in Freeze mode  
	 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR));

	#ifdef	TEST_MESSAGE_QUEUE
		FLEXCAN1_MCR |= FLEXCAN_MCR_IRMQ;
		if(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ)
		printf("Message queue & individual masking feature is enabled\n");
	#endif
	
	#ifdef	TEST_SELF_RECEPTION_DISABLE
		FLEXCAN1_MCR |= FLEXCAN_MCR_SRX_DIS;
		printf("NOTE: Self reception is disabled!\n");
	#endif
	// printf("FLEXCAN1_MCR =%#8x \n",FLEXCAN1_MCR);
#if 0	 
	 // Assume ip bus = 100M, baudrate = 222kbps	 
	 FLEXCAN1_CTRL1 |=     //(FLEXCAN_CTRL_PRESDIV(1L)  |	// 24M IP clock
	 					   (FLEXCAN_CTRL_PRESDIV(50L)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   //FLEXCAN_CTRL_LBUF		  |	// with LBUF: D_IP_FlexCAN3_SYN - d_ip_flexcan3_syn.01.00.06.00 has bug
#ifndef	TEST_ON_EVM
	 					   FLEXCAN_CTRL_LPB			  | /* Enable Internal loopback */
#endif
	 					   FLEXCAN_CTRL_PROPSEG(0x2) 
					   );
#else
                 if(FLEXCAN1_CTRL1 & FLEXCAN_CTRL_CLK_SRC)
                 {

                         /* 
                         ** 96M/96= 1M sclock, 12Tq
                         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
                                 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 48
                         */
                         FLEXCAN1_CTRL1 = (0 | FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(2)
                                                                            | FLEXCAN_CTRL_PSEG1(3) | FLEXCAN_CTRL_PSEG2(3)
                                                                            | FLEXCAN_CTRL_PRESDIV(95));
                 }
                 else
                 {
		         /* 
                         ** 12M/12= 1M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 12
		         */
		         FLEXCAN1_CTRL1 = (0 | FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(2)
			 	    					    | FLEXCAN_CTRL_PSEG1(3) | FLEXCAN_CTRL_PSEG2(3)
			 	    					    | FLEXCAN_CTRL_PRESDIV(11));
                   
                 }   
#ifndef	TEST_ON_EVM                 
    FLEXCAN1_CTRL1 |=   FLEXCAN_CTRL_LPB;              
#endif    
#endif         
    printf("FLEXCAN1_CTRL1 =%#08x\n",FLEXCAN1_CTRL1);					   
  /* Initialize all 16 MBs */		  
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	 
	 FLEXCAN1_RXMGMASK = 0x1FFBFFFF;
	 FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	 
	 if(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ)
	 {
	 	FLEXCAN1_RXIMRn(1) = 0x1FFBFFFFL;
	 	FLEXCAN1_RXIMRn(3) = 0x1FFBFFFFL;	 	 
	 }	
	 	 
	  
    /* De-assert Freeze Mode */
     FLEXCAN1_MCR ^= (FLEXCAN_MCR_HALT);
     
     /* wait till exit of freeze mode */
     while( FLEXCAN1_MCR & FLEXCAN_MCR_FRZ_ACK);

#ifndef	TEST_USE_POLL
	FLEXCAN1_IMASK1 = 0xF;	// enable interrupts for MB0,1,2,3
#endif

 	 // Configure MB1 and MB3 as rx MBs
 	 // MB0 and MB2 as tx MBs
   	 FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x0);
   	 FLEXCAN1_MBn_ID(1) = FLEXCAN_MB_ID_IDSTD(0xF);
 
   	 FLEXCAN1_MBn_CS(3) = FLEXCAN_MB_CS_CODE(0x0);
   	 FLEXCAN1_MBn_ID(3) = FLEXCAN_MB_ID_IDSTD(0xF);
   	 
  	 FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x4);
   	 FLEXCAN1_MBn_CS(3) = FLEXCAN_MB_CS_CODE(0x4);
 

 	 FLEXCAN1_MBn_CS(0) = (FLEXCAN_MB_CS_CODE(0x8) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN1_MBn_ID(0) = FLEXCAN_MB_ID_IDSTD(0xE);
	 FLEXCAN1_MBn_WORD0(0) = 0x12345678;
	 FLEXCAN1_MBn_WORD1(0) = 0x11223344;
	 


 	 FLEXCAN1_MBn_CS(2) = (FLEXCAN_MB_CS_CODE(0x8) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN1_MBn_ID(2) = FLEXCAN_MB_ID_IDSTD(0xE);
	 FLEXCAN1_MBn_WORD0(2) = 0x5A5A5A5A;
	 FLEXCAN1_MBn_WORD1(2) = 0x5555AAAA;

	 /* Pass condition:  */
	if(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ)
	{
		uiMatchValue[0] = FLEXCAN1_MBn_WORD0(0);
		uiMatchValue[1] = FLEXCAN1_MBn_WORD1(0);
		uiMatchValue[2] = FLEXCAN1_MBn_WORD0(2);
		uiMatchValue[3] = FLEXCAN1_MBn_WORD1(2);
	}
	else
	{
		uiMatchValue[0] = FLEXCAN1_MBn_WORD0(2);
		uiMatchValue[1] = FLEXCAN1_MBn_WORD1(2);
		uiMatchValue[2] = FLEXCAN1_MBn_WORD0(2);
		uiMatchValue[3] = FLEXCAN1_MBn_WORD1(2);		
	}

	 FLEXCAN1_MBn_CS(0) = (FLEXCAN_MB_CS_CODE(0xC) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN1_MBn_CS(2) |= FLEXCAN_MB_CS_CODE(0xC); 


#ifdef	TEST_USE_POLL
	 /* Poll for MB2  Completion */
	 while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF2I));
	
	 printf("delay for some time before reading MBs\n");
 
 	 /* Poll for MB1 and MB3 Completion */
	 while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF1I));
	
		 /* Read received frame */
		 cs[1] = FLEXCAN1_MBn_CS(1);
		 id[1]= FLEXCAN1_MBn_ID(1);
		 data0[1] = FLEXCAN1_MBn_WORD0(1);
		 data1[1] = FLEXCAN1_MBn_WORD1(1);
		 timer[1] = FLEXCAN1_TIMER;
		
		// clear the flags
		FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF1I;
	
		
		/* If message queue is enabled, wait for queue elements done */ 
		if(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ )
		{	
		 	while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF3I));
		 	
		 	// read MB3
		 	 cs[2] = FLEXCAN1_MBn_CS(3);
			 id[2]= FLEXCAN1_MBn_ID(3);
			 data0[2] = FLEXCAN1_MBn_WORD0(3);
			 data1[2] = FLEXCAN1_MBn_WORD1(3);
			 timer[2] = FLEXCAN1_TIMER;
		 	
			// clear the flags
			FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF3I; 	
			printf("MB3 received a message!\n");
		}
		 
		 	 
		 printf("CS word = %#08.8x\n", cs[1]);
		 if(data0[1] != uiMatchValue[0])
		 {
		 	error("Error: Data Mismatch in MB1 word0,received = %#08.8x,expected = %#08.8x!\n",data0[1],uiMatchValue[0]);
		 	guiErrCount++;
		 }
		 
		 //printf("data received = %#08.8x\n", data1[1]);
		 if(data1[1] != uiMatchValue[1])
		 {
		 	error("Error: Data Mismatch in MB1 word1,received = %#08.8x,expected = %#08.8x!\n",data1[1],uiMatchValue[1]);
		 	guiErrCount++;
		 }
		 
		 if(((id[1] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
		 {
		 	error("Error: MB1 got wrong ID = %#08.8x\n",id[1]);
		 	guiErrCount++;
		 }
		 
		 printf("MB1: timer  = %#08.8x\n\n", timer[1]);
	
		if(FLEXCAN1_MCR & FLEXCAN_MCR_IRMQ )
		{		
			 //printf("check MB3 received message...\n");
			 if(data0[2] != uiMatchValue[2])
			 {
			 	error("Error: Data Mismatch in MB3 word0!\n");
			 	guiErrCount++;
			 }
			 
			 if(data1[2] != uiMatchValue[3])
			 {
			 	error("Error: Data Mismatch in MB3 word1!\n");
			 	guiErrCount++;
			 }
		 	 if(((id[2] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	error("Error: MB3 got wrong ID = %#08.8x\n",id[2]);
			 	guiErrCount++;
			 }		 
			printf("MB3: timer  = %#08.8x\n", timer[2]);
		}	
	PrintPassFailMessage(guiErrCount);
#else
	while( guiMB_ISR_Count < 4);

	PrintPassFailMessage(guiErrCount);
	guiErrCount = 0;
#endif		 
}

void FlexCAN_SelfLoop0(void)
{
	 uint32_t i, cs[4],id[4],data0[4],data1[4], timer[4];
	 uint16_t uiErrCount = 0;
	 uint32_t uiMatchValue[4];
	 	 
	 printf("Start to test loopback mode for FlexCAN0 with LBUF = 1!\n");
	 /* FlexCAN0 Settings */

	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; // enable module
		 

	 while((FLEXCAN_MCR_LPM_ACK & FLEXCAN0_MCR));	

	// Now can apply Soft Reset
	FLEXCAN0_MCR ^= FLEXCAN_MCR_SOFT_RST;
	while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 	
    printf("After soft reset, FLEXCAN0_MCR =%#08.8x \n",FLEXCAN0_MCR);
	
	 // Now it should be in Freeze mode  
	 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR));

	#ifdef	TEST_MESSAGE_QUEUE
		FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;
	#endif
	// printf("FLEXCAN0_MCR =%#8x \n",FLEXCAN0_MCR);
	 	 
	 FLEXCAN0_CTRL1 |=     //(FLEXCAN_CTRL_PRESDIV(1L)  |	// 24M IP clock
	 					   (FLEXCAN_CTRL_PRESDIV(50L)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |	// with LBUF: D_IP_FlexCAN3_SYN - d_ip_flexcan3_syn.01.00.06.00 has bug
	 					   FLEXCAN_CTRL_LPB			  | /* Enable Internal loopback */
	 					   FLEXCAN_CTRL_PROPSEG(0x2) 
					   );
    printf("FLEXCAN0_CTRL1 =%#08x\n",FLEXCAN0_CTRL1);					   
  /* Initialize all 16 MBs */		  
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }
	 
	 FLEXCAN0_RXMGMASK = 0x1FFBFFFF;
	 FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN0_RX15MASK = 0x1FFFFFFF;
	 if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ)
	 {
	 	FLEXCAN0_RXIMRn(1) = 0x1FFBFFFFL;
	 	FLEXCAN0_RXIMRn(3) = 0x1FFBFFFFL;	 	 
	 }	
	 	 
		  
    /* De-assert Freeze Mode */
     FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
     
     /* wait till exit of freeze mode */
     while( FLEXCAN0_MCR & FLEXCAN_MCR_FRZ_ACK);
#ifdef  CHECK_RXIMRn_VALUES
    FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
     
     /* wait till exit of freeze mode */
     while(!(FLEXCAN0_MCR & FLEXCAN_MCR_FRZ_ACK));
     
     // check RXIMR
     uiMatchValue[0] = FLEXCAN0_RXIMRn(1);
     uiMatchValue[1] = FLEXCAN0_RXIMRn(3);
     
   /* De-assert Freeze Mode */
     FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
     
     /* wait till exit of freeze mode */
     while( FLEXCAN0_MCR & FLEXCAN_MCR_FRZ_ACK);
#endif     
 	 // Configure MB1 and MB3 as rx MBs
 	 // MB0 and MB2 as tx MBs
   	 FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x0);
   	 FLEXCAN0_MBn_ID(1) = FLEXCAN_MB_ID_IDSTD(0xF);
 //	 FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x4);

   	 FLEXCAN0_MBn_CS(3) = FLEXCAN_MB_CS_CODE(0x0);
   	 FLEXCAN0_MBn_ID(3) = FLEXCAN_MB_ID_IDSTD(0xF);
   	 
  	 FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x4);
   	 FLEXCAN0_MBn_CS(3) = FLEXCAN_MB_CS_CODE(0x4);
 

 	 FLEXCAN0_MBn_CS(0) = (FLEXCAN_MB_CS_CODE(0x8) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN0_MBn_ID(0) = FLEXCAN_MB_ID_IDSTD(0xE);
	 FLEXCAN0_MBn_WORD0(0) = 0x12345678;
	 FLEXCAN0_MBn_WORD1(0) = 0x11223344;
	 

 	 FLEXCAN0_MBn_CS(2) = (FLEXCAN_MB_CS_CODE(0x8) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN0_MBn_ID(2) = FLEXCAN_MB_ID_IDSTD(0xE);
	 FLEXCAN0_MBn_WORD0(2) = 0x5A5A5A5A;
	 FLEXCAN0_MBn_WORD1(2) = 0x5555AAAA;

	 /* Pass condition:  */
	if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ)
	{
		uiMatchValue[0] = FLEXCAN0_MBn_WORD0(0);
		uiMatchValue[1] = FLEXCAN0_MBn_WORD1(0);
		uiMatchValue[2] = FLEXCAN0_MBn_WORD0(2);
		uiMatchValue[3] = FLEXCAN0_MBn_WORD1(2);
	}
	else
	{
		uiMatchValue[0] = FLEXCAN0_MBn_WORD0(2);
		uiMatchValue[1] = FLEXCAN0_MBn_WORD1(2);
		uiMatchValue[2] = FLEXCAN0_MBn_WORD0(2);
		uiMatchValue[3] = FLEXCAN0_MBn_WORD1(2);		
	}

	 FLEXCAN0_MBn_CS(0) = (FLEXCAN_MB_CS_CODE(0xC) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN0_MBn_CS(2) |= FLEXCAN_MB_CS_CODE(0xC); 

	// for(i=0;i<500;i++);	// delay for a while
	 
	 /* Poll for MB0  Completion */
	 while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF0I));
 	 /* Poll for MB2  Completion */
	 while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF2I));
 
	 /* Poll for MB1 and MB3 Completion */
	 while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF1I));

	 /* Read received frame */
	 cs[1] = FLEXCAN0_MBn_CS(1);
	 id[1]= FLEXCAN0_MBn_ID(1);
	 data0[1] = FLEXCAN0_MBn_WORD0(1);
	 data1[1] = FLEXCAN0_MBn_WORD1(1);
	 
	 timer[1] = FLEXCAN0_TIMER;
	// clear the flags
	FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF1I;
	
	
	//printf("MB1 received a message!\n");
	/* If message queue is enabled, wait for queue elements done */ 
	if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ )
	{	
	 	while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF3I));
		// clear the flags
		FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF3I; 	
		printf("MB3 received a message!\n");
	}
	else
	{
		 /* Poll for MB1 for 2nd transfer Completion */
		 while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF1I));
	
		 /* Read received frame */
		 cs[1] = FLEXCAN0_MBn_CS(1);
		 id[1]= FLEXCAN0_MBn_ID(1);
		 data0[1] = FLEXCAN0_MBn_WORD0(1);
		 data1[1] = FLEXCAN0_MBn_WORD1(1);
		 
		 timer[1] = FLEXCAN0_TIMER;
		// clear the flags
		FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF1I;		
		printf("MB1 received a message\n");
	}	 	 

	 printf("CS word = %#08.8x\n", cs[1]);
	 if(data0[1] != uiMatchValue[0])
	 {
	 	error("Error: Data Mismatch in MB1 word0,received = %#08.8x,expected = %#08.8x!\n",data0[1],uiMatchValue[0]);
	 	uiErrCount++;
	 }
	 
	 //printf("data received = %#08.8x\n", data1[1]);
	 if(data1[1] != uiMatchValue[1])
	 {
	 	error("Error: Data Mismatch in MB1 word1,received = %#08.8x,expected = %#08.8x!\n",data1[1],uiMatchValue[1]);
	 	uiErrCount++;
	 }
	 
	 if(((id[1] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
	 {
	 	error("Error: MB1 got wrong ID = %#08.8x\n",id[1]);
	 	uiErrCount++;
	 }
	 
	 printf("MB1: timer  = %#08.8x\n\n", timer[1]);

	if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ )
	{	
		 cs[2] = FLEXCAN0_MBn_CS(3);
		 id[2]= FLEXCAN0_MBn_ID(3);
		 data0[2] = FLEXCAN0_MBn_WORD0(3);
		 data1[2] = FLEXCAN0_MBn_WORD1(3);
		 timer[2] = FLEXCAN0_TIMER;

		 if(data0[2] != uiMatchValue[2])
		 {
		 	error("Error: Data Mismatch in MB3 word0!\n");
		 	uiErrCount++;
		 }
		 
		 if(data1[2] != uiMatchValue[3])
		 {
		 	error("Error: Data Mismatch in MB3 word1!\n");
		 	uiErrCount++;
		 }
	 	 if(((id[2] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
		 {
		 	error("Error: MB3 got wrong ID = %#08.8x\n",id[2]);
		 	uiErrCount++;
		 }		 
		printf("MB3: timer  = %#08.8x\n", timer[2]);
	}
	
	PrintPassFailMessage(uiErrCount);	 
}

/* without LBUF set */
void FlexCAN_SelfLoop01(void)
{
	 uint32_t i;
	 
	 guiErrCount = 0;
	 guiMB_ISR_Count = 0;
	 	 
	 printf("Start to test loopback mode for FlexCAN1 without LBUF set\n");
	 /* FlexCAN1 Settings */

	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; // enable module
		 

	 while((FLEXCAN_MCR_LPM_ACK & FLEXCAN0_MCR));	

	// Now can apply Soft Reset
	FLEXCAN0_MCR ^= FLEXCAN_MCR_SOFT_RST;
	while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 	
    //printf("After soft reset, FLEXCAN0_MCR =%#08.8x \n",FLEXCAN0_MCR);
	
	 // Now it should be in Freeze mode  
	 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR));

	#ifdef	TEST_MESSAGE_QUEUE
		FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;
		if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ)
		printf("Message queue & individual masking feature is enabled\n");
	#endif
	
	#ifdef	TEST_SELF_RECEPTION_DISABLE
		FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;
		printf("NOTE: Self reception is disabled!\n");
	#endif
	// printf("FLEXCAN0_MCR =%#8x \n",FLEXCAN0_MCR);
	 
	 // Assume ip bus = 100M, baudrate = 222kbps	 
	 FLEXCAN0_CTRL1 |=     //(FLEXCAN_CTRL_PRESDIV(1L)  |	// 24M IP clock
	 					   (FLEXCAN_CTRL_PRESDIV(50L)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   //FLEXCAN_CTRL_LBUF		  |	// with LBUF: D_IP_FlexCAN3_SYN - d_ip_flexcan3_syn.01.00.06.00 has bug
#ifndef	TEST_ON_EVM
	 					   FLEXCAN_CTRL_LPB			  | /* Enable Internal loopback */
#endif
	 					   FLEXCAN_CTRL_PROPSEG(0x2) 
					   );
    printf("FLEXCAN0_CTRL1 =%#08x\n",FLEXCAN0_CTRL1);					   
  /* Initialize all 16 MBs */		  
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }
	 
	 FLEXCAN0_RXMGMASK = 0x1FFBFFFF;
	 FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN0_RX15MASK = 0x1FFFFFFF;
	 
	 if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ)
	 {
	 	FLEXCAN0_RXIMRn(1) = 0x1FFBFFFFL;
	 	FLEXCAN0_RXIMRn(3) = 0x1FFBFFFFL;	 	 
	 }	
	 	 
	  
    /* De-assert Freeze Mode */
     FLEXCAN0_MCR ^= (FLEXCAN_MCR_HALT);
     
     /* wait till exit of freeze mode */
     while( FLEXCAN0_MCR & FLEXCAN_MCR_FRZ_ACK);

#ifndef	TEST_USE_POLL
	FLEXCAN0_IMASK1 = 0xF;	// enable interrupts for MB0,1,2,3
#endif

 	 // Configure MB1 and MB3 as rx MBs
 	 // MB0 and MB2 as tx MBs
   	 FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x0);
   	 FLEXCAN0_MBn_ID(1) = FLEXCAN_MB_ID_IDSTD(0xF);
 
   	 FLEXCAN0_MBn_CS(3) = FLEXCAN_MB_CS_CODE(0x0);
   	 FLEXCAN0_MBn_ID(3) = FLEXCAN_MB_ID_IDSTD(0xF);
   	 
  	 FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x4);
   	 FLEXCAN0_MBn_CS(3) = FLEXCAN_MB_CS_CODE(0x4);
 

 	 FLEXCAN0_MBn_CS(0) = (FLEXCAN_MB_CS_CODE(0x8) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN0_MBn_ID(0) = FLEXCAN_MB_ID_IDSTD(0xE);
	 FLEXCAN0_MBn_WORD0(0) = 0x12345678;
	 FLEXCAN0_MBn_WORD1(0) = 0x11223344;
	 


 	 FLEXCAN0_MBn_CS(2) = (FLEXCAN_MB_CS_CODE(0x8) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN0_MBn_ID(2) = FLEXCAN_MB_ID_IDSTD(0xE);
	 FLEXCAN0_MBn_WORD0(2) = 0x5A5A5A5A;
	 FLEXCAN0_MBn_WORD1(2) = 0x5555AAAA;

	 /* Pass condition:  */
	if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ)
	{
		uiMatchValue[0] = FLEXCAN0_MBn_WORD0(0);
		uiMatchValue[1] = FLEXCAN0_MBn_WORD1(0);
		uiMatchValue[2] = FLEXCAN0_MBn_WORD0(2);
		uiMatchValue[3] = FLEXCAN0_MBn_WORD1(2);
	}
	else
	{
		uiMatchValue[0] = FLEXCAN0_MBn_WORD0(2);
		uiMatchValue[1] = FLEXCAN0_MBn_WORD1(2);
		uiMatchValue[2] = FLEXCAN0_MBn_WORD0(2);
		uiMatchValue[3] = FLEXCAN0_MBn_WORD1(2);		
	}

	 FLEXCAN0_MBn_CS(0) = (FLEXCAN_MB_CS_CODE(0xC) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN0_MBn_CS(2) |= FLEXCAN_MB_CS_CODE(0xC); 


#ifdef	TEST_USE_POLL
	 /* Poll for MB2  Completion */
	 while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF2I));
	
	 printf("delay for some time before reading MBs\n");
 
 	 /* Poll for MB1 and MB3 Completion */
	 while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF1I));
	
		 /* Read received frame */
		 cs[1] = FLEXCAN0_MBn_CS(1);
		 id[1]= FLEXCAN0_MBn_ID(1);
		 data0[1] = FLEXCAN0_MBn_WORD0(1);
		 data1[1] = FLEXCAN0_MBn_WORD1(1);
		 timer[1] = FLEXCAN0_TIMER;
		
		// clear the flags
		FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF1I;
	
		
		/* If message queue is enabled, wait for queue elements done */ 
		if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ )
		{	
		 	while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF3I));
		 	
		 	// read MB3
		 	 cs[2] = FLEXCAN0_MBn_CS(3);
			 id[2]= FLEXCAN0_MBn_ID(3);
			 data0[2] = FLEXCAN0_MBn_WORD0(3);
			 data1[2] = FLEXCAN0_MBn_WORD1(3);
			 timer[2] = FLEXCAN0_TIMER;
		 	
			// clear the flags
			FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF3I; 	
			printf("MB3 received a message!\n");
		}
		 
		 	 
		 printf("CS word = %#08.8x\n", cs[1]);
		 if(data0[1] != uiMatchValue[0])
		 {
		 	error("Error: Data Mismatch in MB1 word0,received = %#08.8x,expected = %#08.8x!\n",data0[1],uiMatchValue[0]);
		 	guiErrCount++;
		 }
		 
		 //printf("data received = %#08.8x\n", data1[1]);
		 if(data1[1] != uiMatchValue[1])
		 {
		 	error("Error: Data Mismatch in MB1 word1,received = %#08.8x,expected = %#08.8x!\n",data1[1],uiMatchValue[1]);
		 	guiErrCount++;
		 }
		 
		 if(((id[1] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
		 {
		 	error("Error: MB1 got wrong ID = %#08.8x\n",id[1]);
		 	guiErrCount++;
		 }
		 
		 printf("MB1: timer  = %#08.8x\n\n", timer[1]);
	
		if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ )
		{		
			 //printf("check MB3 received message...\n");
			 if(data0[2] != uiMatchValue[2])
			 {
			 	error("Error: Data Mismatch in MB3 word0!\n");
			 	guiErrCount++;
			 }
			 
			 if(data1[2] != uiMatchValue[3])
			 {
			 	error("Error: Data Mismatch in MB3 word1!\n");
			 	guiErrCount++;
			 }
		 	 if(((id[2] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	error("Error: MB3 got wrong ID = %#08.8x\n",id[2]);
			 	guiErrCount++;
			 }		 
			printf("MB3: timer  = %#08.8x\n", timer[2]);
		}	
	PrintPassFailMessage(guiErrCount);
#else
	while( guiMB_ISR_Count < 4);

	PrintPassFailMessage(guiErrCount);
	guiErrCount = 0;
#endif		 
}

void FlexCAN_SelfLoop2(void)
{
	uint_32 state; 
	uint_32 id;
	int16_t iRxMB;
	int16_t iTxMB;
	int16_t i;
	int8_t	bIntrp;
	uint8_t bActivate_mb;
	uint16_t baudrate;
	volatile FLEXCAN_MailBox_STRUCT mb_config;
	
	guiMB_ISR_Count  = 0;
    guiErrCount = 0;
    iRxMBISRcount = 0;
    
#ifdef	TEST_USE_POLL
	bIntrp = FALSE;
#else
	bIntrp = TRUE;	 
#endif 
   
	id = 0x1ABCDEF9;
	baudrate = 83;//100;//1000;	// 1000Kbps
	for(i = 0; i< 8; i++)
	{
		mb_config.data[i] = 0xF9+i;
	}
	// Initialize bit timing
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
  #ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,
#ifndef	TEST_ON_EVM	
	TRUE	// for loopback
#else
	FALSE
#endif	
	);
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}
	#ifdef	TEST_SELF_RECEPTION_DISABLE
		FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;
		printf("NOTE: Self reception is disabled!\n");
	#endif
                
	#ifdef	TEST_MESSAGE_QUEUE
		FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;
		printf("Message Queue is enabled!\n");
		// set individual masking
	    for( iRxMB = FLEXCAN_RX_MB_START; iRxMB <= FLEXCAN_RX_MB_END; iRxMB++)
		{
			FLEXCAN0_RXIMRn(iRxMB) = 0; // do not care
		}
	#endif
	
    
    // Start CAN communication
    //printf("start CAN communication, MCR = %#08.8x\n",FLEXCAN0_MCR);
    state = FLEXCAN_Start(FLEXCAN0);
  	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Start() returned state = %#08x\n",state);
	}  
    //printf("FLEXCAN0_CTRL1 =%#08.8x,FLEXCAN0_MCR = %#08.8x\n",FLEXCAN0_CTRL1,FLEXCAN0_MCR);
       
    // Initialize the mailbox structure for Rx 
    mb_config.dev_num = FLEXCAN0;
    mb_config.data_len = 8;
    mb_config.identifier = id;
    mb_config.format = FLEXCAN_EXTENDED;
    mb_config.direction = FLEXCAN_RX;
    mb_config.remote_req_flag = 0;
    mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
    bActivate_mb = TRUE;
    
    for( iRxMB = FLEXCAN_RX_MB_START; iRxMB <= FLEXCAN_RX_MB_END; iRxMB++)
    {
    	/* Initialize each Rx MB */
    	mb_config.mailbox_number = iRxMB;
    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,bIntrp);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
		}  	
    }	
    mb_config.direction = FLEXCAN_TX;
    mb_config.remote_req_flag = 0;
    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
    
    for( iTxMB = FLEXCAN_TX_MB_START; iTxMB <= FLEXCAN_TX_MB_END; iTxMB++)
    {
    	/* Initialize each Tx MB */
    	mb_config.mailbox_number = iTxMB;
    	mb_config.data[0]--;
    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,bIntrp);
    	if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
		}  	
    }
#ifdef	TEST_USE_POLL    
   do{
   	   // check if there is any error
   	   state = FLEXCAN0_ESR1;
   	   if((state & FLEXCAN_ESR_ERR_INT) || (state & FLEXCAN_ESR_BOFF_INT))
   	   {
   	   		guiErrCount++;
   	   		printf("CAN Error occurs, FLEXCAN_ESR1 = %#08.8x\n",state);
   	   		break;
   	   }
	   // check received message
	   for( iRxMB = FLEXCAN_RX_MB_START; iRxMB <= FLEXCAN_RX_MB_END; iRxMB++)
	    {
	    	if(FLEXCAN_Is_MB_Done(FLEXCAN0,iRxMB))
	    	{
	    		// Read message from this mailbox
	    		mb_config.mailbox_number = iRxMB;
	    		
			 	for(i = 0; i< 8; i++)
				{
					mb_config.data[i] = 0;	// reset data
				}
	 		
				state = FLEXCAN_Rx_message(&mb_config,0);
		   		if(state != FLEXCAN_OK)
				{
					printf("FLEXCAN_Rx_message: returned state = %#08.8x for mailbox %d\n",state,iRxMB);
				}
				else
				{
					// check received message
					printf("MB %d received a message:",iRxMB);
					for(i = 0; i < mb_config.data_len; i++)
					{
						printf("%#02.2x",mb_config.data[i]);
						if(i < mb_config.data_len-1)
						{
							printf(",");
						}
					}	
					printf("\n");			
				} 
				// clear flag
				state = FLEXCAN_Clear_MB_Flag(FLEXCAN0,iRxMB); 
		   		if(state != FLEXCAN_OK)
				{
					printf("FLEXCAN_Clear_MB_Flag: returned state = %#08.8x for mailbox %d\n",state,iRxMB);
				}								  		
	    	}
	    }	    
   }while (1);
#else

  	while(guiMB_ISR_Count < NUMBER_OF_MB);
	if((FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ))
	{
	  	for(iRxMB = FLEXCAN_RX_MB_START; iRxMB <= FLEXCAN_RX_MB_END; iRxMB++)
	  	{
	  		uint32_t data;
	  		
			// read message buffer
			 cs[1] = FLEXCAN0_MBn_CS(iRxMB);
			 
			 id= FLEXCAN0_MBn_ID(iRxMB);
			 
			 data0[1] = FLEXCAN0_MBn_WORD0(iRxMB);
			 data1[1] = FLEXCAN0_MBn_WORD1(iRxMB);
			 timer[1] = FLEXCAN0_TIMER;	// unlock it	
			 
			 // check data
			 data = (((0x00F9uL+3)<<24L) | ((0x00F9uL+2)<<16L) | ((0x00F9uL+1)<<8L) | (0x00F9uL-iRxMB-1) );		
			
			 if(data0[1] != data)
			 {
			 	printf("Error: MB %d received incorrect data0 = %#08.8x,expected = %#08.8x\n",iRxMB,data0[1],data);	
			 	guiErrCount++;		 	
			 }
			 else if(data1[1] != (((0xF9ul+7)<<24) | ((0xF9ul+6)<<16) | ((0xF9ul+5)<<8)	| (0xF9ul+4)))
			 {
			 	printf("Error: MB %d received incorrect data1 = %#08.8x\n",iRxMB,data1[1]);	
			 	guiErrCount++;			 	
			 }
			 printf("MB %d received message id = %#08.8x,c/s = %#08.8x\n",iRxMB,id,cs[1]);
	  	}
	}
#endif 
	  PrintPassFailMessage(guiErrCount);
	  guiErrCount = 0; 
 
  
}

void FlexCAN_RxFifoFilter_Test(void)
{
#ifndef	TEST_ON_EVM
	FlexCAN0_RxFifoFilter_Test();
#else	
  #ifdef  TEST_RXFIFO_FILTER_FORMAT_A
	flexcan0_tx_can1_rx_fifo_formatA_bcc_test();
  #endif        
  #ifdef  TEST_RXFIFO_FILTER_FORMAT_B        
	flexcan0_tx_can1_rx_fifo_formatB_bcc_test();
  #endif        
  #ifdef  TEST_RXFIFO_FILTER_FORMAT_C
	flexcan0_tx_can1_rx_fifo_formatC_bcc_test();	
  #endif        
#endif			
}

void FlexCAN_RxFifoInt_Test(void)
{
#ifndef	TEST_ON_EVM
//	FlexCAN0_RxFifoInt_Test();	
//	FlexCAN1_RxFifoInt_Test();	
      FlexCAN1_RxFifoInt_Test2();  
#else	
	//flexcan0_tx_can1_rx_fifo_warning_overflow_bcc_test();
#if  TEST_FLEXCAN0
	FlexCAN0_RxFifoInt_Test();	
#endif
#if  TEST_FLEXCAN1
	FlexCAN1_RxFifoInt_Test();	
#endif
        
#endif	
}


void FlexCAN0_RxFifoInt_Test(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;

	printf("Start to test Rx FIFO interrupt on FlexCAN0 \n");
 
 	/* Initialize all 16 MBs */		  
	  for(i=0;i<NUMBER_OF_MB;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }
	 	 

	 // Initialize globals
	 

	// Initialize the RxFIFO filter parameters
	for(i = 0; i < MAX_RX_FIFO_FILTER/8; i++)
	{
		rxFIFOFilterParams[i].rxFIFOFilterNo = 8*(i+1);
		rxFIFOFilterParams[i].topRxFIFOMBNo = (rxFIFOFilterParams[i].rxFIFOFilterNo>>2)+RX_FIFO_DEEP-1;	// Message Buffers occupied by Rx FIFO and ID Filter Table
		rxFIFOFilterParams[i].topAvailMBNo = (FLEXCAN0_MCR & 0x7F);		// Remaining Available Mailboxes
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByIMask = min(31,rxFIFOFilterParams[i].topRxFIFOMBNo);// Rx FIFO ID Filter Table Elements Affected by Rx Individual Masks
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].rxFIFOFilterNo-1;// Rx FIFO ID Filter Table Elements Affected by Rx FIFO Global Mask
		
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask >7?rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask:0;
		
	}	
	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
	
	// Initialize error counter
	guiErrCount = 0;
	
	rxFilterNo = 0;
	{
		// Initialize ISR counters
		giRxFIFOISRCount = 0;		
	 	guiMB_ISR_Count = 0;
	 	giRxFIFOWarningCount = 0;
	 	giRxFIFOOverflowCount = 0;
	 	
		state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                           
			TRUE	// for loopback
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
	
		FLEXCAN0_RXMGMASK = 0x1FFBFFFF;	// can only be written in Freeze mode
	 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
	
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		FLEXCAN0_MCR |= FLEXCAN_MCR_FEN 	// enable Rx FIFO
#ifdef	TEST_MESSAGE_QUEUE		
					| FLEXCAN_MCR_IRMQ
#endif					
					;	
		//FLEXCAN0_MCR &= ~FLEXCAN_MCR_IDAM_MASK;					
		//printf("MCR = %#08.8x\n",FLEXCAN0_MCR);
	#ifdef	TEST_IMEU_FEATURE	
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_IMEUEN;	// enable individual matching elements update
	#endif	
	#ifdef	TEST_SCAN_PRIORITY_MB_FIRST
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_MRP;
	#endif
	#ifdef	TEST_REMOTE_REQUEST_FEATURE
		//RRS ！ Remote Request Storing
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_RRS;
	#endif	
		FLEXCAN0_CTRL2 = (FLEXCAN0_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | rxFilterNo;
		
		FLEXCAN0_IMASK1 &= ~FLEXCAN_IMASK1_BUF5M;	// disable rxFIFO interrupt 
		FLEXCAN0_IMASK1 |= FLEXCAN_IMASK1_BUF6M;	// enable rxFIFO warning interrupt 
		FLEXCAN0_IMASK1 |= FLEXCAN_IMASK1_BUF7M;	// enable rxFIFO overflow interrupt 
		
		//printf("CTRL2 = %#08.8x\n",FLEXCAN0_CTRL2);
		printf("FLEXCAN0_IMASK1=%#08.8x\n",FLEXCAN0_IMASK1);
		
		// Configure filter table elements
		iNoOfFilterTableElements = rxFIFOFilterParams[rxFilterNo].rxFIFOFilterNo;
		
		printf("# of Filter Table Elements = %d\n",iNoOfFilterTableElements);
		
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			
			FLEXCAN0_IDFLT_TAB(i) = (i+1L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;
					
			//printf("Filter table element %d = %#08.8x\n",i,FLEXCAN0_IDFLT_TAB(i));
		}	
		
		// Configure inidividual mask and rx FIFO global mask	
		iNoOfFilterTableElements = min(16,iNoOfFilterTableElements);
			
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
#if 1
			FLEXCAN0_RXIMRn(i) = 0;
#else			
			FLEXCAN0_RXIMRn(i) = 0xFFF80000L		// matching bit 30 to bit 19
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif	
#endif				
							;						
		}
		
		// Initialize the rest of individual mask registers
		for(;i<NUMBER_OF_MB;i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFFFFFFFL;	// compare all bits
		}
		
		// Set global mask for rx FIFO	
		
		FLEXCAN0_RXFGMASK =  0xFFF80000L		// matching bit 30 to bit 19
		
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif		
		;					
		
		// Start FlexCAN0
		state = FLEXCAN_Start(FLEXCAN0);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Start() returned state = %#08x\n",state);
		}	
		if(!(FLEXCAN0_IMASK1 & FLEXCAN_IMASK1_BUF5M))
		{
			// 
			printf("Error: FLEXCAN_IMASK1_BUF5M is not set!\n");
		}
		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
		printf("MB%d to MB15 is available for MBs not for FIFO\n",NoMBsAvail); 
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
	    	    	    
		// Initialize last MB as  tx MB
		iTxMB = NUMBER_OF_MB-1;
	    mb_config.dev_num = FLEXCAN0;
	    mb_config.data_len = 8;
	    bActivate_mb = TRUE;			
		mb_config.mailbox_number = iTxMB;
	    mb_config.direction = FLEXCAN_TX;
	    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
	    mb_config.remote_req_flag = TRUE;
#else
	    mb_config.remote_req_flag = 0;		    
#endif
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
		    
		for(i = 0; i< RX_FIFO_DEEP;i++)	// send  RX_FIFO_DEEP messages
		{	
#ifdef	TEST_EXTENDED_ID					    
				mb_config.identifier = (i+1)<<18;			
#else
				mb_config.identifier = (i+1);
#endif		
			mb_config.data[0]--;		
		    //printf("Transmit message ID = %#08.8x (in filter table element format,shift left 1 bit)\n",mb_config.identifier<<1);
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
			}  	
			// wait for the tx done
			while(!FLEXCAN_Is_MB_Done(FLEXCAN0,iTxMB)){}	
			
			// clear the flag
			FLEXCAN0_IFLAG1 = (1<<iTxMB);								
		}
		// Wait warning flag set
		while(giRxFIFOWarningCount!=1);
		
		// Now send another frame
		state = FLEXCAN_Tx_mailbox(FLEXCAN0,iTxMB,(vpointer)mb_config.data,8);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
		}  	
		// wait for the tx done
		while(!FLEXCAN_Is_MB_Done(FLEXCAN0,iTxMB)){}	
		
		// clear the flag
		FLEXCAN0_IFLAG1 = (1<<iTxMB);		
		
		// Wait overflow flag set
		while(giRxFIFOOverflowCount!=1);
		
		// Now read a message from FIFO
		while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)){}

		 cs[0] = FLEXCAN0_MBn_CS(0);
		 id= FLEXCAN0_MBn_ID(0);
		 data0[0] = FLEXCAN0_MBn_WORD0(0);
		 data1[0] = FLEXCAN0_MBn_WORD1(0);	
		 
		 printf("RxFIFO message received,ID = %#08.8x,data0 =%#08.8x,data1=%#08.8x!\n",id,data0[0],data1[0]);			 

		 // read RXFIR
		 state = FLEXCAN0_RXFIR & 0x1FF;
		 
		 // clear flag to allow update of FIFO
		 FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;

		// Now read another message from FIFO
		while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)){}

		 cs[0] = FLEXCAN0_MBn_CS(0);
		 id= FLEXCAN0_MBn_ID(0);
		 data0[0] = FLEXCAN0_MBn_WORD0(0);
		 data1[0] = FLEXCAN0_MBn_WORD1(0);	
		 
		 printf("RxFIFO message received,ID = %#08.8x,data0 =%#08.8x,data1=%#08.8x!\n",id,data0[0],data1[0]);			 

		 // read RXFIR
		 state = FLEXCAN0_RXFIR & 0x1FF;
		 
		 // clear flag to allow update of FIFO
		 FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;	

		// Now send another 4 frames
		for(i=0;i<3;i++)
		{
			state = FLEXCAN_Tx_mailbox(FLEXCAN0,iTxMB,(vpointer)mb_config.data,8);
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
			}  	
			// wait for the tx done
			while(!FLEXCAN_Is_MB_Done(FLEXCAN0,iTxMB)){}	
			// clear the flag
			FLEXCAN0_IFLAG1 = (1<<iTxMB);								

		}
						 						
		// Warning flag and overflow flag count should be 2
		
		// wait till rx FIFO interrupts are completed 
		while(guiMB_ISR_Count< 4) 
		{}	
			
		if( (giRxFIFOWarningCount != 2) || (giRxFIFOOverflowCount != 2))
		{
			guiErrCount++;
		}
		PrintPassFailMessage(guiErrCount);				    		
	}	
}

void FlexCAN1_RxFifoInt_Test(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;

	printf("Start to test Rx FIFO interrupt on FlexCAN1 \n");
 
 	/* Initialize all 16 MBs */		  
	  for(i=0;i<NUMBER_OF_MB;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	 	 

	 // Initialize globals
	 

	// Initialize the RxFIFO filter parameters
	for(i = 0; i < MAX_RX_FIFO_FILTER/8; i++)
	{
		rxFIFOFilterParams[i].rxFIFOFilterNo = 8*(i+1);
		rxFIFOFilterParams[i].topRxFIFOMBNo = (rxFIFOFilterParams[i].rxFIFOFilterNo>>2)+RX_FIFO_DEEP-1;	// Message Buffers occupied by Rx FIFO and ID Filter Table
		rxFIFOFilterParams[i].topAvailMBNo = (FLEXCAN0_MCR & 0x7F);		// Remaining Available Mailboxes
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByIMask = min(31,rxFIFOFilterParams[i].topRxFIFOMBNo);// Rx FIFO ID Filter Table Elements Affected by Rx Individual Masks
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].rxFIFOFilterNo-1;// Rx FIFO ID Filter Table Elements Affected by Rx FIFO Global Mask
		
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask >7?rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask:0;
		
	}	
	// Initialize the FlexCAN0
	baudrate = 83;//1000;	// 1Mbps
	
	// Initialize error counter
	guiErrCount = 0;
	
	rxFilterNo = 0;
	{
		// Initialize ISR counters
		giRxFIFOISRCount = 0;		
	 	guiMB_ISR_Count = 0;
	 	giRxFIFOWarningCount = 0;
	 	giRxFIFOOverflowCount = 0;
	 	
		state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          , 
#ifdef  TEST_ON_EVM
        FALSE
#else
			TRUE	// for loopback
#endif                          
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
	
		FLEXCAN1_RXMGMASK = 0x1FFBFFFF;	// can only be written in Freeze mode
	 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		FLEXCAN1_MCR |= FLEXCAN_MCR_FEN 	// enable Rx FIFO
#ifdef	TEST_MESSAGE_QUEUE		
					| FLEXCAN_MCR_IRMQ
#endif					
					;	
	#ifdef	TEST_IMEU_FEATURE	
		FLEXCAN1_CTRL2 |= FLEXCAN_CTRL2_IMEUEN;	// enable individual matching elements update
	#endif	
	#ifdef	TEST_SCAN_PRIORITY_MB_FIRST
		FLEXCAN1_CTRL2 |= FLEXCAN_CTRL2_MRP;
	#endif
	#ifdef	TEST_REMOTE_REQUEST_FEATURE
		//RRS ！ Remote Request Storing
		FLEXCAN1_CTRL2 |= FLEXCAN_CTRL2_RRS;
	#endif	
		FLEXCAN1_CTRL2 = (FLEXCAN1_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | rxFilterNo;
		
		FLEXCAN1_IMASK1 &= ~FLEXCAN_IMASK1_BUF5M;	// disable rxFIFO interrupt 
		FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF6M;	// enable rxFIFO warning interrupt 
		FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF7M;	// enable rxFIFO overflow interrupt 
		
			
		// Configure filter table elements
		iNoOfFilterTableElements = rxFIFOFilterParams[rxFilterNo].rxFIFOFilterNo;
		
		printf("# of Filter Table Elements = %d\n",iNoOfFilterTableElements);
		
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			
			FLEXCAN1_IDFLT_TAB(i) = (i+1L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;
					
			}	
		
		// Configure inidividual mask and rx FIFO global mask	
		iNoOfFilterTableElements = min(16,iNoOfFilterTableElements);
			
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			FLEXCAN1_RXIMRn(i) = 0;		
							;						
		}
		
		// Initialize the rest of individual mask registers
		for(;i<NUMBER_OF_MB;i++)
		{
			FLEXCAN1_RXIMRn(i) = 0xFFFFFFFFL;	// compare all bits
		}
		
		// Set global mask for rx FIFO	
		
		FLEXCAN1_RXFGMASK =  0xFFF80000L		// matching bit 30 to bit 19
		
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif		
		;					
		
		// Start FlexCAN0
		state = FLEXCAN_Start(FLEXCAN1);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Start() returned state = %#08x\n",state);
		}	
		if(!(FLEXCAN1_IMASK1 & FLEXCAN_IMASK1_BUF5M))
		{
			// 
			printf("Error: FLEXCAN_IMASK1_BUF5M is not set!\n");
		}
		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
		printf("MB%d to MB15 is available for MBs not for FIFO\n",NoMBsAvail); 
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
	    	    	    
		// Initialize last MB as  tx MB
		iTxMB = NUMBER_OF_MB-1;
	    mb_config.dev_num = FLEXCAN1;
	    mb_config.data_len = 8;
	    bActivate_mb = TRUE;			
		mb_config.mailbox_number = iTxMB;
	    mb_config.direction = FLEXCAN_TX;
	    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
	    mb_config.remote_req_flag = TRUE;
#else
	    mb_config.remote_req_flag = 0;		    
#endif
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
		    
		for(i = 0; i< RX_FIFO_DEEP;i++)	// send  RX_FIFO_DEEP messages
		{	
#ifdef	TEST_EXTENDED_ID					    
				mb_config.identifier = (i+1)<<18;			
#else
				mb_config.identifier = (i+1);
#endif		
			mb_config.data[0]--;		

	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
			}  	
			// wait for the tx done
			while(!FLEXCAN_Is_MB_Done(FLEXCAN1,iTxMB)){}	
			
			// clear the flag
			FLEXCAN1_IFLAG1 = (1<<iTxMB);								
		}
		// Wait warning flag set
		while(giRxFIFOWarningCount!=1);
		
		// Now send another frame
		state = FLEXCAN_Tx_mailbox(FLEXCAN1,iTxMB,(vpointer)mb_config.data,8);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
		}  	
		// wait for the tx done
		while(!FLEXCAN_Is_MB_Done(FLEXCAN1,iTxMB)){}	
		// clear the flag
		FLEXCAN1_IFLAG1 = (1<<iTxMB);								
		
		
		// Wait overflow flag set
		while(giRxFIFOOverflowCount!=1);
		
		// Now read a message from FIFO
		while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)){}

		 cs[0] = FLEXCAN1_MBn_CS(0);
		 id= FLEXCAN1_MBn_ID(0);
		 data0[0] = FLEXCAN1_MBn_WORD0(0);
		 data1[0] = FLEXCAN1_MBn_WORD1(0);	
		 
		 printf("RxFIFO message received,ID = %#08.8x,data0 =%#08.8x,data1=%#08.8x!\n",id,data0[0],data1[0]);			 

		 // read RXFIR
		 state = FLEXCAN1_RXFIR & 0x1FF;
		 
		 // clear flag to allow update of FIFO
		 FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;

		// Now read another message from FIFO
		while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)){}

		 cs[0] = FLEXCAN1_MBn_CS(0);
		 id= FLEXCAN1_MBn_ID(0);
		 data0[0] = FLEXCAN1_MBn_WORD0(0);
		 data1[0] = FLEXCAN1_MBn_WORD1(0);	
		 
		 printf("RxFIFO message received,ID = %#08.8x,data0 =%#08.8x,data1=%#08.8x!\n",id,data0[0],data1[0]);			 

		 // read RXFIR
		 state = FLEXCAN1_RXFIR & 0x1FF;
		 
		 // clear flag to allow update of FIFO
		 FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;	
		

		// Now send another 4 frames so that there are 4 messages (warn flag condition) and 
		// then 4 more because there are two SMBRXs 
		for(i=0;i<3;i++)
		{
			state = FLEXCAN_Tx_mailbox(FLEXCAN1,iTxMB,(vpointer)mb_config.data,8);
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
			}  	
			// wait for the tx done
			while(!FLEXCAN_Is_MB_Done(FLEXCAN1,iTxMB)){}	
			
			// clear the flag
			FLEXCAN1_IFLAG1 = (1<<iTxMB);								
			//
		}
						 						
		// Warning flag and overflow flag count should be 2
		
		// wait till rx FIFO interrupts are completed 
		while(guiMB_ISR_Count< 4) 
		{}	
			
		if( (giRxFIFOWarningCount != 2) || (giRxFIFOOverflowCount != 2))
		{
			guiErrCount++;
		}
		PrintPassFailMessage(guiErrCount);				    		
	}		
}

void FlexCAN1_RxFifoInt_Test2(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;

	printf("Start to test Rx FIFO interrupt on FlexCAN1 for ERROO2522 \n");
 

 	/* Initialize all 16 MBs */		  
	  for(i=0;i<NUMBER_OF_MB;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	 

	 // Initialize globals
	 

	// Initialize the RxFIFO filter parameters
	for(i = 0; i < MAX_RX_FIFO_FILTER/8; i++)
	{
		rxFIFOFilterParams[i].rxFIFOFilterNo = 8*(i+1);
		rxFIFOFilterParams[i].topRxFIFOMBNo = (rxFIFOFilterParams[i].rxFIFOFilterNo>>2)+RX_FIFO_DEEP-1;	// Message Buffers occupied by Rx FIFO and ID Filter Table
		rxFIFOFilterParams[i].topAvailMBNo = (FLEXCAN1_MCR & 0x7F);		// Remaining Available Mailboxes
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByIMask = min(31,rxFIFOFilterParams[i].topRxFIFOMBNo);// Rx FIFO ID Filter Table Elements Affected by Rx Individual Masks
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].rxFIFOFilterNo-1;// Rx FIFO ID Filter Table Elements Affected by Rx FIFO Global Mask
		
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask >7?rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask:0;
		
	}	
	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
	
	// Initialize error counter
	guiErrCount = 0;
	
	rxFilterNo = 0;
	{
		// Initialize ISR counters
		giRxFIFOISRCount = 0;		
	 	guiMB_ISR_Count = 0;
	 	giRxFIFOWarningCount = 0;
	 	giRxFIFOOverflowCount = 0;
	 	
		state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                           
			TRUE	// for loopback
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
	
		FLEXCAN1_RXMGMASK = 0x1FFBFFFF;	// can only be written in Freeze mode
	 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		FLEXCAN1_MCR |= FLEXCAN_MCR_FEN 	// enable Rx FIFO
#ifdef	TEST_MESSAGE_QUEUE		
					| FLEXCAN_MCR_IRMQ
#endif					
					;	
	#ifdef	TEST_IMEU_FEATURE	
		FLEXCAN1_CTRL2 |= FLEXCAN_CTRL2_IMEUEN;	// enable individual matching elements update
	#endif	
	#ifdef	TEST_SCAN_PRIORITY_MB_FIRST
		FLEXCAN1_CTRL2 |= FLEXCAN_CTRL2_MRP;
	#endif
	#ifdef	TEST_REMOTE_REQUEST_FEATURE
		//RRS ！ Remote Request Storing
		FLEXCAN1_CTRL2 |= FLEXCAN_CTRL2_RRS;
	#endif	
		FLEXCAN1_CTRL2 = (FLEXCAN1_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | rxFilterNo;
		
		FLEXCAN1_IMASK1 &= ~FLEXCAN_IMASK1_BUF5M;	// disable rxFIFO interrupt 
		FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF6M;	// enable rxFIFO warning interrupt 
		FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF7M;	// enable rxFIFO overflow interrupt 
		
			
		// Configure filter table elements
		iNoOfFilterTableElements = rxFIFOFilterParams[rxFilterNo].rxFIFOFilterNo;
		
		printf("# of Filter Table Elements = %d\n",iNoOfFilterTableElements);
		
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			
			FLEXCAN1_IDFLT_TAB(i) = (i+1L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;
					
			}	
		
		// Configure inidividual mask and rx FIFO global mask	
		iNoOfFilterTableElements = min(16,iNoOfFilterTableElements);
			
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			FLEXCAN1_RXIMRn(i) = 0;		
							;						
		}
		
		// Initialize the rest of individual mask registers
		for(;i<NUMBER_OF_MB;i++)
		{
			FLEXCAN1_RXIMRn(i) = 0xFFFFFFFFL;	// compare all bits
		}
		
		// Set global mask for rx FIFO	
		
		FLEXCAN1_RXFGMASK =  0xFFF80000L		// matching bit 30 to bit 19
		
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif		
		;					
		
		// Start FlexCAN1
		state = FLEXCAN_Start(FLEXCAN1);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Start() returned state = %#08x\n",state);
		}	
		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
		printf("MB%d to MB15 is available for MBs not for FIFO\n",NoMBsAvail); 
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
	    	    	    
		// Initialize last MB as  tx MB
		iTxMB = NUMBER_OF_MB-1;
	    mb_config.dev_num = FLEXCAN1;
	    mb_config.data_len = 8;
	    bActivate_mb = TRUE;			
		mb_config.mailbox_number = iTxMB;
	    mb_config.direction = FLEXCAN_TX;
	    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
	    mb_config.remote_req_flag = TRUE;
#else
	    mb_config.remote_req_flag = 0;		    
#endif
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
		    
		for(i = 0; i< RX_FIFO_DEEP;i++)	// send  RX_FIFO_DEEP messages
		{	
#ifdef	TEST_EXTENDED_ID					    
				mb_config.identifier = (i+1)<<18;			
#else
				mb_config.identifier = (i+1);
#endif		
			mb_config.data[0]--;		

	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
			}  	
			// wait for the tx done
			while(!FLEXCAN_Is_MB_Done(FLEXCAN1,iTxMB)){}	
			
			// clear the flag
			FLEXCAN1_IFLAG1 = (1<<iTxMB);								
		}
		// Wait warning flag set
		while(giRxFIFOWarningCount!=1);


#ifdef	TEST_EXTENDED_ID					    
		mb_config.identifier = (0+1)<<18;			
#else
		mb_config.identifier = (0+1);
#endif	                
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
			}  	
			// wait for the tx done
			while(!FLEXCAN_Is_MB_Done(FLEXCAN1,iTxMB)){}	
			
			// clear the flag
			FLEXCAN1_IFLAG1 = (1<<iTxMB);				                
	
		// Wait overflow flag set
		while(giRxFIFOOverflowCount!=1);


#ifdef	TEST_EXTENDED_ID					    
				mb_config.identifier = (0+1)<<18;			
#else
				mb_config.identifier = (0+1);
#endif	                
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
			}  	
			// wait for the tx done
			while(!FLEXCAN_Is_MB_Done(FLEXCAN1,iTxMB)){}	
			
			// clear the flag
			FLEXCAN1_IFLAG1 = (1<<iTxMB);				                
	                
		// Now read a message from FIFO
		while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)){}

		 cs[0] = FLEXCAN1_MBn_CS(0);
		 id= FLEXCAN1_MBn_ID(0);
		 data0[0] = FLEXCAN1_MBn_WORD0(0);
		 data1[0] = FLEXCAN1_MBn_WORD1(0);	
		 
		 printf("RxFIFO message received,ID = %#08.8x,data0 =%#08.8x,data1=%#08.8x!\n",id,data0[0],data1[0]);			 

		 // read RXFIR
		 state = FLEXCAN1_RXFIR & 0x1FF;
		 
		 // clear flag to allow update of FIFO
		 FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;
                  
		// Now read another message from FIFO
		while(!(FLEXCAN1_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)){}                                

		 cs[0] = FLEXCAN1_MBn_CS(0);
		 id= FLEXCAN1_MBn_ID(0);
		 data0[0] = FLEXCAN1_MBn_WORD0(0);
		 data1[0] = FLEXCAN1_MBn_WORD1(0);	
		 
		 printf("RxFIFO message received,ID = %#08.8x,data0 =%#08.8x,data1=%#08.8x!\n",id,data0[0],data1[0]);			 

		 // read RXFIR
		 state = FLEXCAN1_RXFIR & 0x1FF;
		 
		 // clear flag to allow update of FIFO
		 FLEXCAN1_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;	
		

		// Now send another 4 frames so that there are 4 messages (warn flag condition) and 
		// then 4 more because there are two SMBRXs 
		for(i=0;i<3;i++)
		{
			state = FLEXCAN_Tx_mailbox(FLEXCAN1,iTxMB,(vpointer)mb_config.data,8);
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
			}  	
			// wait for the tx done
			while(!FLEXCAN_Is_MB_Done(FLEXCAN1,iTxMB)){}	
			
			// clear the flag
			FLEXCAN1_IFLAG1 = (1<<iTxMB);								
			//
		}
						 						
		// Warning flag and overflow flag count should be 2
		
		// wait till rx FIFO interrupts are completed 
		while(guiMB_ISR_Count< 5) 
		{}	
			
		if( (giRxFIFOWarningCount != 2) || (giRxFIFOOverflowCount != 3))
		{
			guiErrCount++;
		}
		PrintPassFailMessage(guiErrCount);				    		
	}		
}



void FlexCAN_Priority_Test(void)
{
#ifdef	TEST_ON_EVM	
	//flexcan0_tx_can1_rx_priority_bcc_test();
  #if TEST_FLEXCAN0
        FlexCAN0_Priority_Test();
  #endif
  #if TEST_FLEXCAN1
        FlexCAN1_Priority_Test();
  #endif  
#else
	FlexCAN0_Priority_Test();
#endif	
}

void FlexCAN0_Priority_Test(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,k,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;
	int16_t	iTxMB_no_sorted[NUMBER_OF_MB];
	int16_t	iTxMB_order_expected[NUMBER_OF_MB] = {15,14,13,12,11,10,9,8
	};

	printf("Start to test priority feature: Highest priority Mailbox first and Lowest number Mailbox first\n");	
	// Initialize ISR counters
	giRxFIFOISRCount = 0;		
 	guiMB_ISR_Count = 0;
  	
	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
	
	// Initialize error counter
	guiErrCount = 0;
	
	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	}	 	
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
			TRUE	// for loopback
			);	
	if(state != FLEXCAN_OK)
	{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// negate RRS		
	FLEXCAN0_CTRL2 &= ~FLEXCAN_CTRL2_RRS;
	
	// enable tx MB8 to 15 interrupts
	FLEXCAN0_IMASK1 = 0x0000FF00L;
	
	//Highest priority Mailbox first
	// case: CTRL1[LBUF] bit = 0 and MCR[LPRIO_EN] =1
	// Mailbox Arbitration Value is 35 bits with PRIO as msb
	FLEXCAN0_CTRL1 &= ~(FLEXCAN_CTRL_LBUF);
	FLEXCAN0_MCR	|= (FLEXCAN_MCR_LPRIO_EN);
	
	//
		
	// Configure MB0 to MB7 as rx 
	for(i = 0; i< 8; i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x1ABCDEF0+(i);
	 	FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY)
	 						 | FLEXCAN_MB_CS_IDE
	 						 ;
	}	
	// Configure MB8 to MB15 as tx with local priority field set (PRIO) in reverse order
	for(i = 8; i< 16; i++)
	{
	  	FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	  	FLEXCAN0_MBn_ID(i) = (0x1ABCDEF0+(i)) | (FLEXCAN_MB_ID_PRIO(16-i-1));	// 
	  	FLEXCAN0_MBn_WORD0(i) = 0x55AA+(i);
	  	FLEXCAN0_MBn_WORD1(i) = 0xAA88+(i);
	}
	// Activate each tx MB
	for(i = 8; i<16; i++)
	{	
	 	FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
	 						 | FLEXCAN_MB_CS_IDE
	 						 | FLEXCAN_MB_CS_SRR
	 						 ;
	}
	// Start FlexCAN0
	state = FLEXCAN_Start(FLEXCAN0);
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Start() returned state = %#08x\n",state);
	}	
	printf("check ISR count...\n");
	
	// Check tranmission order
	while(guiMB_ISR_Count<8){}
	
	// Sort the tx MB per timestamp
	SelectionSort((uint32_t*)&giTimeStamp[8],8);
	 
	for(i=8;i<16;i++)
	{
	 	for(k=8;k<16;k++)
	 	{
	 		if(giTimeStamp[i] == (cs[k] & 0xFFFF))
	 		{
	 			iTxMB_no_sorted[i] = k;	// find the corresponding MB in timestamp order
	 			break; 
	 		}
	 	}
	}
	for(i=8;i<16;i++)
	{
		if(iTxMB_no_sorted[i] != iTxMB_order_expected[i-8])
		{
			printf("Error: tx MB order issue, expected MB is %d, but result is %d",
				iTxMB_order_expected[i-8],iTxMB_no_sorted[i]);
			guiErrCount++;
		}
	}	
	 
	printf("Start to test tx with lowest number mailbox first.\n");
	// Now test Lowest number Mailbox first
	guiMB_ISR_Count = 0;
	
	// Put FlexCAN0 into freeze mode 
	FLEXCAN0_MCR |= FLEXCAN_MCR_HALT;
	while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR)){}
	
	// Clear timer
	FLEXCAN0_TIMER = 0;
	
	//Lowest number Mailbox first
	// case: CTRL1[LBUF] bit = 1 and MCR[LPRIO_EN] =1
	// Mailbox Arbitration Value is 35 bits with PRIO as msb
	FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_LBUF);
	FLEXCAN0_MCR	|= (FLEXCAN_MCR_LPRIO_EN);

	// Activate each tx MB
	for(i = 8; i<16; i++)
	{	
	 	FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
	 						 | FLEXCAN_MB_CS_IDE
	 						 | FLEXCAN_MB_CS_SRR
	 						 ;
	}	
	
	// Leave freeze mode
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
	while((FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR)){}	
	while((FLEXCAN_MCR_NOT_RDY & FLEXCAN0_MCR)){}	
	
	//
	// Check tranmission order
	while(guiMB_ISR_Count<8){}

	// Sort the tx MB per timestamp
	SelectionSort((uint32_t*)&giTimeStamp[8],8);
	 
	for(i=8;i<16;i++)
	{
	 	for(k=8;k<16;k++)
	 	{
	 		if(giTimeStamp[i] == (cs[k] & 0xFFFF))
	 		{
	 			iTxMB_no_sorted[i] = k;	// find the corresponding MB in timestamp order
	 			break; 
	 		}
	 	}
	}
	// Set expected order of tx MB
	for(i=0;i<8;i++)
	{
		iTxMB_order_expected[i] = i+8;
	}
	
	for(i=8;i<16;i++)
	{
		if(iTxMB_no_sorted[i] != iTxMB_order_expected[i-8])
		{
			printf("Error: tx MB order issue, expected MB is %d, but result is %d",
				iTxMB_order_expected[i-8],iTxMB_no_sorted[i]);
			guiErrCount++;
		}
	}	
	PrintPassFailMessage(guiErrCount);				    		
			
}

void FlexCAN1_Priority_Test(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,k,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;
	int16_t	iTxMB_no_sorted[NUMBER_OF_MB];
	int16_t	iTxMB_order_expected[NUMBER_OF_MB] = {15,14,13,12,11,10,9,8
	};

	printf("Start to test priority feature: Highest priority Mailbox first and Lowest number Mailbox first\n");	
	// Initialize ISR counters
	giRxFIFOISRCount = 0;		
 	guiMB_ISR_Count = 0;
  	
	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
	
	// Initialize error counter
	guiErrCount = 0;
	
	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}	 	
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
			TRUE	// for loopback
			);	
	if(state != FLEXCAN_OK)
	{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// negate RRS		
	FLEXCAN1_CTRL2 &= ~FLEXCAN_CTRL2_RRS;
	
	// enable tx MB8 to 15 interrupts
	FLEXCAN1_IMASK1 = 0x0000FF00L;
	
	//Highest priority Mailbox first
	// case: CTRL1[LBUF] bit = 0 and MCR[LPRIO_EN] =1
	// Mailbox Arbitration Value is 35 bits with PRIO as msb
	FLEXCAN1_CTRL1 &= ~(FLEXCAN_CTRL_LBUF);
	FLEXCAN1_MCR	|= (FLEXCAN_MCR_LPRIO_EN);
	
	//
		
	// Configure MB0 to MB7 as rx 
	for(i = 0; i< 8; i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x1ABCDEF0+(i);
	 	FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY)
	 						 | FLEXCAN_MB_CS_IDE
	 						 ;
	}	
	// Configure MB8 to MB15 as tx with local priority field set (PRIO) in reverse order
	for(i = 8; i< 16; i++)
	{
	  	FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	  	FLEXCAN1_MBn_ID(i) = (0x1ABCDEF0+(i)) | (FLEXCAN_MB_ID_PRIO(16-i-1));	// 
	  	FLEXCAN1_MBn_WORD0(i) = 0x55AA+(i);
	  	FLEXCAN1_MBn_WORD1(i) = 0xAA88+(i);
	}
	// Activate each tx MB
	for(i = 8; i<16; i++)
	{	
	 	FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
	 						 | FLEXCAN_MB_CS_IDE
	 						 | FLEXCAN_MB_CS_SRR
	 						 ;
	}
	// Start FlexCAN0
	state = FLEXCAN_Start(FLEXCAN1);
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Start() returned state = %#08x\n",state);
	}	
	printf("check ISR count...\n");
	
	// Check tranmission order
	while(guiMB_ISR_Count<8){}
	
	// Sort the tx MB per timestamp
	SelectionSort((uint32_t*)&giTimeStamp[8],8);
	 
	for(i=8;i<16;i++)
	{
	 	for(k=8;k<16;k++)
	 	{
	 		if(giTimeStamp[i] == (cs[k] & 0xFFFF))
	 		{
	 			iTxMB_no_sorted[i] = k;	// find the corresponding MB in timestamp order
	 			break; 
	 		}
	 	}
	}
	for(i=8;i<16;i++)
	{
		if(iTxMB_no_sorted[i] != iTxMB_order_expected[i-8])
		{
			printf("Error: tx MB order issue, expected MB is %d, but result is %d",
				iTxMB_order_expected[i-8],iTxMB_no_sorted[i]);
			guiErrCount++;
		}
	}	
	 
	printf("Start to test tx with lowest number mailbox first.\n");
	// Now test Lowest number Mailbox first
	guiMB_ISR_Count = 0;
	
	// Put FlexCAN0 into freeze mode 
	FLEXCAN1_MCR |= FLEXCAN_MCR_HALT;
	while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR)){}
	
	// Clear timer
	FLEXCAN1_TIMER = 0;
	
	//Lowest number Mailbox first
	// case: CTRL1[LBUF] bit = 1 and MCR[LPRIO_EN] =1
	// Mailbox Arbitration Value is 35 bits with PRIO as msb
	FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_LBUF);
	FLEXCAN1_MCR	|= (FLEXCAN_MCR_LPRIO_EN);

	// Activate each tx MB
	for(i = 8; i<16; i++)
	{	
	 	FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
	 						 | FLEXCAN_MB_CS_IDE
	 						 | FLEXCAN_MB_CS_SRR
	 						 ;
	}	
	
	// Leave freeze mode
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
	while((FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR)){}	
	while((FLEXCAN_MCR_NOT_RDY & FLEXCAN1_MCR)){}	
	
	//
	// Check tranmission order
	while(guiMB_ISR_Count<8){}

	// Sort the tx MB per timestamp
	SelectionSort((uint32_t*)&giTimeStamp[8],8);
	 
	for(i=8;i<16;i++)
	{
	 	for(k=8;k<16;k++)
	 	{
	 		if(giTimeStamp[i] == (cs[k] & 0xFFFF))
	 		{
	 			iTxMB_no_sorted[i] = k;	// find the corresponding MB in timestamp order
	 			break; 
	 		}
	 	}
	}
	// Set expected order of tx MB
	for(i=0;i<8;i++)
	{
		iTxMB_order_expected[i] = i+8;
	}
	
	for(i=8;i<16;i++)
	{
		if(iTxMB_no_sorted[i] != iTxMB_order_expected[i-8])
		{
			printf("Error: tx MB order issue, expected MB is %d, but result is %d",
				iTxMB_order_expected[i-8],iTxMB_no_sorted[i]);
			guiErrCount++;
		}
	}	
	PrintPassFailMessage(guiErrCount);				    		
			
}



void FlexCAN_RemoteFrame_Test(void)
{
#ifndef	TEST_ON_EVM
	FlexCAN0_RemoteFrame_Test();
#else 	
	//flexcan0_tx_can1_rx_remote_frame_bcc_test();
        FlexCAN0_RemoteFrame_Test();
#endif	
}

void FlexCAN0_RemoteFrame_Test(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;

	printf("Start to test remote frame\n");
	printf("case1: automatic response with a data frame...\n");
	// case 1: automatic response with a data frame
	// Initialize ISR counters
		giRxFIFOISRCount = 0;		
	 	guiMB_ISR_Count = 0;
	 	
		// Initialize the FlexCAN0
		baudrate = 1000;	// 1Mbps
		
		// Initialize error counter
		guiErrCount = 0;
	
	/* Initialize all 16 MBs */		  
	  for(i=0;i<NUMBER_OF_MB;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }	 	
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,
                        
			TRUE	// for loopback
			);	
	if(state != FLEXCAN_OK)
	{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// negate RRS		
	FLEXCAN0_CTRL2 &= ~FLEXCAN_CTRL2_RRS;
	
	// Disable MB0 to 7 interrupts
	FLEXCAN0_IMASK1 = 0;
		
	// Start FlexCAN0
	state = FLEXCAN_Start(FLEXCAN0);
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Start() returned state = %#08x\n",state);
	}	
		
	// Configure MB0 to MB7 as automatic response to a remote request frame
	for(i = 0; i< 8; i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x1ABCDEF0+(i);
	  	FLEXCAN0_MBn_WORD0(i) = 0x55AA+(i);
	  	FLEXCAN0_MBn_WORD1(i) = 0xAA88+(i);
	 	FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_RESPONSE)
	 						 | FLEXCAN_MB_CS_IDE
	 						 | FLEXCAN_MB_CS_SRR
	 						 ;
	}

	
 	// Configure MB8 to 15 as tx MB to transmit remote requests
 	for(i = 8; i< 16; i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x1ABCDEF0+(i-8);
	 	FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
	 						 | FLEXCAN_MB_CS_RTR
	 						 | FLEXCAN_MB_CS_SRR
	 						 ;
	}
 		
	// Loop to check if MB8 to MB15
	for(i=8;i<16;i++)
	{
		if(FLEXCAN0_IFLAG1 & (1<<i))
		{
			//
			cs[0] = FLEXCAN0_MBn_CS(i);	// lock the MB
			id = FLEXCAN0_MBn_ID(i);
			data0[0] = FLEXCAN0_MBn_WORD0(i);
			data1[0] = FLEXCAN0_MBn_WORD1(i);
			if(id != 0x1ABCDEF0+(i-8))
			{
				printf("Error: MB%d received a message ID = %#08.8x,data0=%#08.8x,data1=%#08.8x\n",
						i,id,data0[0],data1[0]);
				guiErrCount++;
			}
			// unlock the MB
			state = FLEXCAN0_TIMER;
			// clear the flag
			FLEXCAN0_IFLAG1 = (1<<i);			
		}
	}
	// Check MB0 to MB7 to see if code is returned to 0x0A
	for(i=0;i<8;i++)
	{
		if(FLEXCAN0_IFLAG1 & (1<<i))
		{
			//
			cs[0] = FLEXCAN0_MBn_CS(i);	// lock the MB
			if(FLEXCAN_get_code(cs[0]) != FLEXCAN_MB_CODE_TX_RESPONSE)
			{
				printf("Error: MB%d's code is not 0x0A, cs word = %#08.8x\n",i,cs[0]);
			} 
			// clear the flag
			FLEXCAN0_IFLAG1 = (1<<i);			
		}
		
	}
	// 
	printf("Change remote response MBs' code to intermediate code 0x0E\n");  
	// Configure MB0 to MB7 as rx to receive remote request response
	for(i = 0; i< 8; i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x1ABCDEF0+(i);
	 	FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY)
	 						 | FLEXCAN_MB_CS_IDE
	 						 ;
	}	
	// Configure MB8 to 15 as tx MB with intermediate code 0x0E to transmit remote request response
 	for(i = 8; i< 16; i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x1ABCDEF0+(i-8);
	  	FLEXCAN0_MBn_WORD0(i) = 0x112266BB+(i);
	  	FLEXCAN0_MBn_WORD1(i) = 0x00FFCC88+(i);
	 	FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_RESPONSE_TEMPO)
	 						 | FLEXCAN_MB_CS_RTR
	 						 | FLEXCAN_MB_CS_SRR
	 						 ;
	}	
	// Loop to check if MB0 to MB7 have received remote request response frames
	for(i=0;i<8;i++)
	{
		if(FLEXCAN0_IFLAG1 & (1<<i))
		{
			//
			cs[0] = FLEXCAN0_MBn_CS(i);	// lock the MB
			id = FLEXCAN0_MBn_ID(i);
			data0[0] = FLEXCAN0_MBn_WORD0(i);
			data1[0] = FLEXCAN0_MBn_WORD1(i);
			if(id != 0x1ABCDEF0+(i))
			{
				printf("Error: MB%d received a message ID = %#08.8x,data0=%#08.8x,data1=%#08.8x\n",
						i,id,data0[0],data1[0]);
				guiErrCount++;
			}
			// unlock the MB
			state = FLEXCAN0_TIMER;
			// clear the flag
			FLEXCAN0_IFLAG1 = (1<<i);			
		}
	}
	
	// case 2: store remote frame as a data frame
 	printf("Test case 2 to store remote frame as a data frame, please define TEST_REMOTE_REQUEST_FEATURE macro!!! \n");
  	// disable FlexCAN
  	FLEXCAN0_MCR	|= FLEXCAN_MCR_MDIS;
  	while(!(FLEXCAN_MCR_LPM_ACK & FLEXCAN0_MCR));
  	
 	/* Initialize all 16 MBs */		  
	  for(i=0;i<NUMBER_OF_MB;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }	 	 
	//printf("end of MBs init\n");
	// Initialize globals 

	// Initialize the RxFIFO filter parameters
	for(i = 0; i < MAX_RX_FIFO_FILTER/8; i++)
	{
		rxFIFOFilterParams[i].rxFIFOFilterNo = 8*(i+1);
		rxFIFOFilterParams[i].topRxFIFOMBNo = (rxFIFOFilterParams[i].rxFIFOFilterNo>>2)+RX_FIFO_DEEP-1;	// Message Buffers occupied by Rx FIFO and ID Filter Table
		rxFIFOFilterParams[i].topAvailMBNo = (FLEXCAN0_MCR & 0x7F);		// Remaining Available Mailboxes
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByIMask = min(31,rxFIFOFilterParams[i].topRxFIFOMBNo);// Rx FIFO ID Filter Table Elements Affected by Rx Individual Masks
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].rxFIFOFilterNo-1;// Rx FIFO ID Filter Table Elements Affected by Rx FIFO Global Mask
		
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask >7?rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask:0;
		
	}	
	
	rxFilterNo = 0;
	{	 	
	 	
		state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                           
			TRUE	// for loopback
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
		FLEXCAN0_RXMGMASK = 0x1FFFFFFF;	// can only be written in Freeze mode
	 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		FLEXCAN0_MCR |= FLEXCAN_MCR_FEN 	// enable Rx FIFO
					| FLEXCAN_MCR_IRMQ
					;	
		//printf("MCR = %#08.8x\n",FLEXCAN0_MCR);
	#ifdef	TEST_IMEU_FEATURE	
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_IMEUEN;	// enable individual matching elements update
	#endif	
	#ifdef	TEST_SCAN_PRIORITY_MB_FIRST
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_MRP;
	#endif
	#ifdef	TEST_REMOTE_REQUEST_FEATURE
		//RRS ！ Remote Request Storing
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_RRS;
	#endif	
		FLEXCAN0_CTRL2 = (FLEXCAN0_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | rxFilterNo;
		
		FLEXCAN0_IMASK1 |= FLEXCAN_IMASK1_BUF5M;	// enable rxFIFO interrupt 
		
		//printf("CTRL2 = %#08.8x\n",FLEXCAN0_CTRL2);
		
		// Configure filter table elements
		iNoOfFilterTableElements = rxFIFOFilterParams[rxFilterNo].rxFIFOFilterNo;
		
		printf("# of Filter Table Elements = %d\n",iNoOfFilterTableElements);
		
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			
			FLEXCAN0_IDFLT_TAB(i) = (i+1L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;
					
			//printf("Filter table element %d = %#08.8x\n",i,FLEXCAN0_IDFLT_TAB(i));
		}	
		
		// Configure inidividual mask and rx FIFO global mask	
		iNoOfFilterTableElements = min(16,iNoOfFilterTableElements);
			
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFF80000L		// matching bit 30 to bit 19
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif	
				
							;						
		}
		id = i;		
		
		// Initialize the rest of individual mask registers
		for(;i<NUMBER_OF_MB;i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFFFFFFFL;	// compare all bits
		}
		
		// Set global mask for rx FIFO	
		FLEXCAN0_RXFGMASK =  0xFFF80000L		// matching bit 30 to bit 19
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif		
		;					
		
		// Start FlexCAN0
		state = FLEXCAN_Start(FLEXCAN0);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Start() returned state = %#08x\n",state);
		}	
		if(!(FLEXCAN0_IMASK1 & FLEXCAN_IMASK1_BUF5M))
		{
			// 
			printf("Error: FLEXCAN_IMASK1_BUF5M is not set!\n");
		}
		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
	    mb_config.dev_num = FLEXCAN0;
	    mb_config.data_len = 8;
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
	    mb_config.direction = FLEXCAN_RX;
	    
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
		    mb_config.remote_req_flag = TRUE;
#else
		    mb_config.remote_req_flag = 0;		    
#endif
	    mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
	    bActivate_mb = TRUE;
	    
	    // Store rx MB to the global to be accessed in isr
	    gNoMBsAvail = NoMBsAvail;
	    
	    printf("MB %d to MB15 are available for MBs not used for rx FIFO\n",NoMBsAvail);
	    
		for(iRxMB=NoMBsAvail;iRxMB < (NUMBER_OF_MB-1);iRxMB++)	// last MB for tx
		{			
			//
#ifdef	TEST_EXTENDED_ID			 
 	    	mb_config.identifier = id++<<18;
#else
	    	mb_config.identifier = id++;
#endif 	    		
  	    	//printf("rx MB%d ID = %#08.8x\n",iRxMB,mb_config.identifier);
 	    	mb_config.mailbox_number = iRxMB;
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,TRUE);
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
			}  				
			
		}
		if(iRxMB <= NUMBER_OF_MB-1)
		{
			// Initialize last MB as  tx MB
			iTxMB = iRxMB;
			mb_config.mailbox_number = iTxMB;
		    mb_config.direction = FLEXCAN_TX;
		    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
		    mb_config.remote_req_flag = TRUE;
#else
		    mb_config.remote_req_flag = 0;		    
#endif
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
		    
		    for(i = 0; i< iNoOfFilterTableElements+(NUMBER_OF_MB-NoMBsAvail-1)+2;i++)	// send additional 2 messages
			{	
#ifdef	TEST_EXTENDED_ID					    
				mb_config.identifier = (i+1)<<18;			
#else
				mb_config.identifier = (i+1);
#endif				
		    	//printf("Transmit message ID = %#08.8x (in filter table element format,shift left 1 bit)\n",mb_config.identifier<<1);
		    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
				if(state != FLEXCAN_OK)
				{
					printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
				}  	
				// wait for the tx done
				while(!FLEXCAN_Is_MB_Done(FLEXCAN0,iTxMB)){}	
				
				// clear the flag
				FLEXCAN0_IFLAG1 = (1<<iTxMB);								
			}
		}
		// wait till rx FIFO interrupt and MB interrupts are completed.
		while(guiMB_ISR_Count<(iNoOfFilterTableElements+(NUMBER_OF_MB-NoMBsAvail-2))){}		
		PrintPassFailMessage(guiErrCount);				    		
	}	
}



void FlexCAN_LowPowerMode_Test(void)
{
	FlexCAN0_DozeStopMode_test();
}

void FlexCAN_Disable_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i;

	
	printf("Start to test FlexCAN behavior in Disable mode on FlexCAN0 \n");
 
 	// Initialize error counter
	guiErrCount = 0;

  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		TRUE	// for loopback
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN0_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_RXIMRn(i) = 0x00000000;
	}
			 
	//	individual Rx masking and queue enable
	FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;

	// clear CTRL2 but TASD
	FLEXCAN0_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB0 to MB7 as a receive MB with ID don't care
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
		FLEXCAN0_MBn_CS(i) = 0x00000000;
		FLEXCAN0_MBn_ID(i) = 0x00000000;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}	
	
	// Write timer to some value
	FLEXCAN0_TIMER = 0xCCCC;
	
	// Start CAN communication
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
	
	while(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	
	// Initialize MB8 to 15 as transmit MBs
	for(i = NUMBER_OF_MB/2;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN0_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN0_MBn_WORD0(i) = 0x5555;
		FLEXCAN0_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}	

	DelayBits(147);	// wait for 147 bits
	
	while(!(FLEXCAN0_ESR2 & FLEXCAN_ESR2_VPS )){}
	while((FLEXCAN0_ESR2 & FLEXCAN_ESR2_IMB)){}

	// Put flexcan in freeze mode
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
	while( !(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK)) ||
	
	        !(FLEXCAN0_MCR & FLEXCAN_MCR_NOT_RDY)) {}
		
	// Put flexcan in disable mode
	FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS;
	
	while( !(FLEXCAN0_MCR & (FLEXCAN_MCR_LPM_ACK)) ||
	
	        !(FLEXCAN0_MCR & FLEXCAN_MCR_NOT_RDY)) {}
	        
	// Check FRZ_ACK flag
	if(	FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK))
	{
		guiErrCount++;
	}
	
	// Check all flags to see if they are set
	for(i = 0;i<NUMBER_OF_MB;i++)
	{
		if(!(FLEXCAN0_IFLAG1 & (1<<i)))
		{
			cs[0] = FLEXCAN0_MBn_CS(i);
			printf("MB%d is not completed,C/S=%#08.8x!\n",i,cs[0]);
		}
	}
	PrintPassFailMessage(guiErrCount);		
}

void FlexCAN_TxAbort_Test(void)
{
	//FlexCAN1_TxAbort_Test();
        FlexCAN1_TxAbort_Test2();
}


void FlexCAN1_TxAbort_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i,iTxMB;

	printf("Start to test tx abort feature for FlexCAN1 \n");
 
 	// Initialize error counter
	guiErrCount = 0;
	guiMB_ISR_Count = 0;

  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
 	  	
	// case 1: Rx FIFO is disabled
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		TRUE	// for loopback
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	// enable ABORT feature
	FLEXCAN1_MCR |= FLEXCAN_MCR_AEN;
	
	FLEXCAN1_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
 	FLEXCAN1_IMASK1 = 0xFFFF;	// enable interrupts for MB0 to MB15
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_RXIMRn(i) = 0x00000000;
	}
			 
	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	printf("FLEXCAN1_CTRL1=%#08.8x\n",FLEXCAN1_CTRL1);
	
	// Initialize MB0 as a receive MB with ID don't care
	FLEXCAN1_MBn_CS(0) = 0x00000000;
	FLEXCAN1_MBn_ID(0) = 0x00000000;
	FLEXCAN1_MBn_CS(0) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
						 | FLEXCAN_MB_CS_IDE
	;
		
	for(i=1;i<NUMBER_OF_MB;i++)
	{	
		// Initialize MB1 to MB15 as transmit MBs
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN1_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN1_MBn_WORD0(i) = 0x5555;
		FLEXCAN1_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}
		
	// Write timer to some value
	FLEXCAN1_TIMER = 0xCCCC;
	
	// Start CAN communication
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
	
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	// 
	// Wait for some times
	//DelayBits(200);
			
	// Now abort all tx MBs
	for(i=1;i<NUMBER_OF_MB;i++)
	{	
		// Initialize MB1 to MB15 as transmit MBs
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ABORT);
	}


	while (	guiMB_ISR_Count < 15);
	
	guiMB_ISR_Count = 0;
	// Retransmit MB1 to MB15
	for(i=1;i<NUMBER_OF_MB;i++)
	{	
		// Initialize MB1 to MB15 as transmit MBs
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN1_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN1_MBn_WORD0(i) = 0x5555;
		FLEXCAN1_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}
	//DelayBits(100);
	//printf("Find the inactive lowest number of tx MB\n");
	// Find the inactive lowest number tx MB 
	state = FLEXCAN1_ESR2;
	while(!((state) & FLEXCAN_ESR2_VPS))
	{
		state = FLEXCAN1_ESR2;
	}
	//printf("ESR2=%#08.8x\n",state);
	iTxMB = FLEXCAN_get_LTM(state);	
	
	printf("Lowest # of inactive tx MB is %d to be aborted...\n",iTxMB);
	printf("MB%d C/S = %#08.8x before abort\n",iTxMB,FLEXCAN1_MBn_CS(iTxMB));
	
	//FLEXCAN1_MBn_CS(iTxMB) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ABORT);	

	// Now abort all tx MBs
	for(i=1;i<NUMBER_OF_MB;i++)
	{	
		// Initialize MB1 to MB15 as transmit MBs
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ABORT);
	}
	printf("Now put flexcan in freeze mode...\n");

	// put flexcan into freeze mode
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
	
	while(!(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK))) {}

	printf("Now leave freeze mode\n");
	// leave freeze mode
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
	
	while((FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK))) {}				

	
	while (	guiMB_ISR_Count < 15);
	
}


void FlexCAN1_TxAbort_Test2(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i,iTxMB;

	printf("Start to test tx abort feature for FlexCAN1: only one tx MB and multiple rx MBs \n");
 
 	// Initialize error counter
	guiErrCount = 0;
	guiMB_ISR_Count = 0;

  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
 	  	
	// case 1: Rx FIFO is disabled
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		TRUE	// for loopback
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	// enable ABORT feature
	FLEXCAN1_MCR |= FLEXCAN_MCR_AEN;
	
	FLEXCAN1_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
 	FLEXCAN1_IMASK1 = 0xFFFF;	// enable interrupts for MB0 to MB15
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_RXIMRn(i) = 0x00000000;
	}
			 
	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	printf("FLEXCAN1_CTRL1=%#08.8x\n",FLEXCAN1_CTRL1);
	
	// Initialize MB0 as a receive MB with ID don't care
	FLEXCAN1_MBn_CS(0) = 0x00000000;
	FLEXCAN1_MBn_ID(0) = 0x00000000;
	FLEXCAN1_MBn_CS(0) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
						 | FLEXCAN_MB_CS_IDE
	;
		
	for(i=1;i<2;i++)
	{	
		// Initialize MB1  as transmit MBs
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN1_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN1_MBn_WORD0(i) = 0x5555;
		FLEXCAN1_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}
		
	// Write timer to some value
	FLEXCAN1_TIMER = 0xCCCC;
	
	// Start CAN communication
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
	
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	// 
	// Wait for some times
	DelayBits1(10);
			
	// Now abort all tx MBs
	for(i=1;i<2;i++)
	{	
		
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ABORT);
	}
	while (	guiMB_ISR_Count < 2);
	
}

void FlexCAN_Tsync_Test(void)
{
#if TEST_FLEXCAN0  
	FlexCAN0_Tsync_Test();
#endif
#if TEST_FLEXCAN1
	FlexCAN1_Tsync_Test();
#endif        
}

void FlexCAN0_Tsync_Test(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;

	printf("Start to test Tsync feature on FlexCAN0 \n");
 
 	// Initialize error counter
	guiErrCount = 0;

  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
 	  	
	// case 1: Rx FIFO is disabled
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		TRUE	// for loopback
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN0_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_RXIMRn(i) = 0x00000000;
	}
			 
	//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
	FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;

	// clear CTRL2 but TASD
	FLEXCAN0_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	// enable TSYNC mode
	FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_TSYNC;	
	printf("FLEXCAN0_CTRL1=%#08.8x\n",FLEXCAN0_CTRL1);
	
	// Initialize MB0 as a receive MB with ID don't care
	FLEXCAN0_MBn_CS(0) = 0x00000000;
	FLEXCAN0_MBn_ID(0) = 0x00000000;
	FLEXCAN0_MBn_CS(0) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
						 | FLEXCAN_MB_CS_IDE
	;
		
#if 1	
	// Initialize MB1 as transmit MB
	FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	FLEXCAN0_MBn_ID(1) = 0x18588885L;
	FLEXCAN0_MBn_WORD0(1) = 0x5555;
	FLEXCAN0_MBn_WORD1(1) = 0xAAAA;
	FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
							 | FLEXCAN_MB_CS_LENGTH(8)
							 | FLEXCAN_MB_CS_IDE
							 | FLEXCAN_MB_CS_SRR;
	printf("FLEXCAN0_MB1_CS = %#08.8x\n",FLEXCAN0_MBn_CS(1));
#endif		
	// Write timer to some value
	FLEXCAN0_TIMER = 0xCCCC;
	
	// Start CAN communication
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
	
	
	while(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}

#if 0	
	// Initialize MB1 as transmit MB
	FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	FLEXCAN0_MBn_ID(1) = 0x18588885L;
	FLEXCAN0_MBn_WORD0(1) = 0x5555;
	FLEXCAN0_MBn_WORD1(1) = 0xAAAA;
	FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
							 | FLEXCAN_MB_CS_LENGTH(8)
							 | FLEXCAN_MB_CS_IDE
							 | FLEXCAN_MB_CS_SRR;
	printf("FLEXCAN0_MB1_CS = %#08.8x\n",FLEXCAN0_MBn_CS(1));
#endif		
	
	// Wait till MB0 interrupt flag set 
	while(!(FLEXCAN0_IFLAG1 & 1)){}
	
	// Now check timer
	timer[0] = FLEXCAN0_TIMER;
	if(timer[0] > 20)
	{
		guiErrCount++;
		printf("Error: Timer (= %#08.8x) is not reset after MB0 receiving a message\n",timer[0]); 		
	}
	printf("FLEXCAN0_MCR = %#08.8x\n",FLEXCAN0_MCR);
	
	// clear the MB0 flag
	// clear the MB1 flag
	
	// case 2: Rx FIFO is enabled	
	/*
	 * If the RFEN bit in MCR is set (Rx FIFO enabled),
	 * the first available Mailbox, according to CTRL2[RFFN] setting, 
	 * is used for timer synchronization instead of MB0
	 */

 	/* Initialize all 16 MBs */		  
	  for(i=0;i<NUMBER_OF_MB;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }
	 	 

	 // Initialize globals
	 

	// Initialize the RxFIFO filter parameters
	for(i = 0; i < MAX_RX_FIFO_FILTER/8; i++)
	{
		rxFIFOFilterParams[i].rxFIFOFilterNo = 8*(i+1);
		rxFIFOFilterParams[i].topRxFIFOMBNo = (rxFIFOFilterParams[i].rxFIFOFilterNo>>2)+RX_FIFO_DEEP-1;	// Message Buffers occupied by Rx FIFO and ID Filter Table
		rxFIFOFilterParams[i].topAvailMBNo = (FLEXCAN0_MCR & 0x7F);		// Remaining Available Mailboxes
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByIMask = min(31,rxFIFOFilterParams[i].topRxFIFOMBNo);// Rx FIFO ID Filter Table Elements Affected by Rx Individual Masks
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].rxFIFOFilterNo-1;// Rx FIFO ID Filter Table Elements Affected by Rx FIFO Global Mask
		
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask >7?rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask:0;
		
	}	
	// Init	RFFN		
	rxFilterNo = 0;
	{
		// Initialize ISR counters
		giRxFIFOISRCount = 0;		
	 	guiMB_ISR_Count = 0;
	 	giRxFIFOWarningCount = 0;
	 	giRxFIFOOverflowCount = 0;
	 	
		state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                           
			TRUE	// for loopback
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
		FLEXCAN0_RXMGMASK = 0x1FFBFFFF;	// can only be written in Freeze mode
	 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;

		FLEXCAN0_IMASK1 |= FLEXCAN_IMASK1_BUF5M;	// enable rxFIFO interrupt 
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		FLEXCAN0_MCR |= FLEXCAN_MCR_FEN 	// enable Rx FIFO
					| FLEXCAN_MCR_IRMQ
					;	
		//FLEXCAN0_MCR &= ~FLEXCAN_MCR_IDAM_MASK;					
		//printf("MCR = %#08.8x\n",FLEXCAN0_MCR);
	#ifdef	TEST_IMEU_FEATURE	
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_IMEUEN;	// enable individual matching elements update
	#endif	
	#ifdef	TEST_SCAN_PRIORITY_MB_FIRST
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_MRP;
	#endif
	#ifdef	TEST_REMOTE_REQUEST_FEATURE
		//RRS ！ Remote Request Storing
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_RRS;
	#endif	
		FLEXCAN0_CTRL2 = (FLEXCAN0_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | rxFilterNo;
		
		// enable TSYNC mode
		FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_TSYNC;
				
		//printf("CTRL2 = %#08.8x\n",FLEXCAN0_CTRL2);
		
		// Configure filter table elements
		iNoOfFilterTableElements = rxFIFOFilterParams[rxFilterNo].rxFIFOFilterNo;
		
		printf("# of Filter Table Elements = %d\n",iNoOfFilterTableElements);
		
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			
			FLEXCAN0_IDFLT_TAB(i) = (i+1L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;
					
			//printf("Filter table element %d = %#08.8x\n",i,FLEXCAN0_IDFLT_TAB(i));
		}	
		id = i;	// save last ID for filter table
			
		// Configure inidividual mask and rx FIFO global mask	
		iNoOfFilterTableElements = min(16,iNoOfFilterTableElements);
			
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFF80000L		// matching bit 30 to bit 19
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif				
							;						
		}
		
		// Initialize the rest of individual mask registers
		for(;i<NUMBER_OF_MB;i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFFFFFFFL;	// compare all bits
		}
		// Set global mask for rx FIFO	
		FLEXCAN0_RXFGMASK =  0xFFF80000L		// matching bit 30 to bit 19
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif		
		;					
		
		// Start FlexCAN0
		state = FLEXCAN_Start(FLEXCAN0);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Start() returned state = %#08x\n",state);
		}	
		if(!(FLEXCAN0_IMASK1 & FLEXCAN_IMASK1_BUF5M))
		{
			// 
			printf("Error: FLEXCAN_IMASK1_BUF5M is not set!\n");
		}
		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
	    mb_config.dev_num = FLEXCAN0;
	    mb_config.data_len = 8;
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
	    mb_config.direction = FLEXCAN_RX;
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
		    mb_config.remote_req_flag = TRUE;
#else
		    mb_config.remote_req_flag = 0;		    
#endif
	    mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
	    bActivate_mb = TRUE;
	    
	    // Store rx MB to the global to be accessed in isr
	    gNoMBsAvail = NoMBsAvail;
	    
	    //printf("MB %d to MB15 are available for MBs not used for rx FIFO\n",NoMBsAvail);
	    
		for(iRxMB=NoMBsAvail;iRxMB < (NUMBER_OF_MB-1);iRxMB++)	// last MB for tx
		{			
			//
#ifdef	TEST_EXTENDED_ID	
			mb_config.identifier = id++	<<18;
#else		
 	    	mb_config.identifier = id++;
#endif 	    	

  	    	//printf("rx MB%d ID = %#08.8x\n",iRxMB,mb_config.identifier);
 	    	mb_config.mailbox_number = iRxMB;
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,TRUE);
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
			}  				
			
		}			    	    	    
		// Initialize last MB as  tx MB
		iTxMB = NUMBER_OF_MB-1;
	    mb_config.dev_num = FLEXCAN0;
	    mb_config.data_len = 8;
	    bActivate_mb = FALSE;			// deactivate it after initialization of tx MB
		mb_config.mailbox_number = iTxMB;
	    mb_config.direction = FLEXCAN_TX;
	    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
	    mb_config.remote_req_flag = TRUE;
#else
	    mb_config.remote_req_flag = 0;		    
#endif
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
		    
		    for(i = 0; i< iNoOfFilterTableElements+(NUMBER_OF_MB-NoMBsAvail-1);i++)	// send additional RX_FIFO_DEEP messages
			{	
#ifdef	TEST_EXTENDED_ID					    
				mb_config.identifier = (i+1)<<18;			
#else
				mb_config.identifier = (i+1);
#endif				
		    //printf("Transmit message ID = %#08.8x (in filter table element format,shift left 1 bit)\n",mb_config.identifier<<1);
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
			}  
				
			// Write timer to some value
			FLEXCAN0_TIMER = 0xBBBB;
			
			// Activate the tx MB
			FLEXCAN0_MBn_CS(iTxMB) |=  FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE);
						
			// wait for the tx done
			while(!(FLEXCAN0_IFLAG1 & (1<<iTxMB))){}	
			
			// clear the flag
			FLEXCAN0_IFLAG1 = (1<<iTxMB);								
		}
		// wait till rx FIFO interrupts are completed 
		while(guiMB_ISR_Count<(iNoOfFilterTableElements+(NUMBER_OF_MB-NoMBsAvail-2))){}	
			
		PrintPassFailMessage(guiErrCount);				    		
	}		
}


void FlexCAN1_Tsync_Test(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;

	printf("Start to test Tsync feature on FlexCAN1 \n");
 
 	// Initialize error counter
	guiErrCount = 0;

  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
 	  	
	// case 1: Rx FIFO is disabled
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		TRUE	// for loopback
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN1_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_RXIMRn(i) = 0x00000000;
	}
			 
	//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
	FLEXCAN1_MCR |= FLEXCAN_MCR_IRMQ;

	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	// enable TSYNC mode
	FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_TSYNC;	
	printf("FLEXCAN1_CTRL1=%#08.8x\n",FLEXCAN1_CTRL1);
	
	// Initialize MB0 as a receive MB with ID don't care
	FLEXCAN1_MBn_CS(0) = 0x00000000;
	FLEXCAN1_MBn_ID(0) = 0x00000000;
	FLEXCAN1_MBn_CS(0) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
						 | FLEXCAN_MB_CS_IDE
	;
		
#if 1	
	// Initialize MB1 as transmit MB
	FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	FLEXCAN1_MBn_ID(1) = 0x18588885L;
	FLEXCAN1_MBn_WORD0(1) = 0x5555;
	FLEXCAN1_MBn_WORD1(1) = 0xAAAA;
	FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
							 | FLEXCAN_MB_CS_LENGTH(8)
							 | FLEXCAN_MB_CS_IDE
							 | FLEXCAN_MB_CS_SRR;
	printf("FLEXCAN1_MB1_CS = %#08.8x\n",FLEXCAN1_MBn_CS(1));
#endif		
	// Write timer to some value
	FLEXCAN1_TIMER = 0xCCCC;
	
	// Start CAN communication
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
	
	
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}

#if 0	
	// Initialize MB1 as transmit MB
	FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	FLEXCAN1_MBn_ID(1) = 0x18588885L;
	FLEXCAN1_MBn_WORD0(1) = 0x5555;
	FLEXCAN1_MBn_WORD1(1) = 0xAAAA;
	FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
							 | FLEXCAN_MB_CS_LENGTH(8)
							 | FLEXCAN_MB_CS_IDE
							 | FLEXCAN_MB_CS_SRR;
	printf("FLEXCAN1_MB1_CS = %#08.8x\n",FLEXCAN1_MBn_CS(1));
#endif		
	
	// Wait till MB0 interrupt flag set 
	while(!(FLEXCAN1_IFLAG1 & 1)){}
	
	// Now check timer
	timer[0] = FLEXCAN1_TIMER;
	if(timer[0] > 20)
	{
		guiErrCount++;
		printf("Error: Timer (= %#08.8x) is not reset after MB0 receiving a message\n",timer[0]); 		
	}
	printf("FLEXCAN1_MCR = %#08.8x\n",FLEXCAN1_MCR);
	
	// clear the MB0 flag
	// clear the MB1 flag
	
	// case 2: Rx FIFO is enabled	
	/*
	 * If the RFEN bit in MCR is set (Rx FIFO enabled),
	 * the first available Mailbox, according to CTRL2[RFFN] setting, 
	 * is used for timer synchronization instead of MB0
	 */

 	/* Initialize all 16 MBs */		  
	  for(i=0;i<NUMBER_OF_MB;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	 	 

	 // Initialize globals
	 

	// Initialize the RxFIFO filter parameters
	for(i = 0; i < MAX_RX_FIFO_FILTER/8; i++)
	{
		rxFIFOFilterParams[i].rxFIFOFilterNo = 8*(i+1);
		rxFIFOFilterParams[i].topRxFIFOMBNo = (rxFIFOFilterParams[i].rxFIFOFilterNo>>2)+RX_FIFO_DEEP-1;	// Message Buffers occupied by Rx FIFO and ID Filter Table
		rxFIFOFilterParams[i].topAvailMBNo = (FLEXCAN0_MCR & 0x7F);		// Remaining Available Mailboxes
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByIMask = min(31,rxFIFOFilterParams[i].topRxFIFOMBNo);// Rx FIFO ID Filter Table Elements Affected by Rx Individual Masks
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].rxFIFOFilterNo-1;// Rx FIFO ID Filter Table Elements Affected by Rx FIFO Global Mask
		
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask >7?rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask:0;
		
	}	
	// Init	RFFN		
	rxFilterNo = 0;
	{
		// Initialize ISR counters
		giRxFIFOISRCount = 0;		
	 	guiMB_ISR_Count = 0;
	 	giRxFIFOWarningCount = 0;
	 	giRxFIFOOverflowCount = 0;
	 	
		state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                           
			TRUE	// for loopback
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
		FLEXCAN1_RXMGMASK = 0x1FFBFFFF;	// can only be written in Freeze mode
	 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;

		FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF5M;	// enable rxFIFO interrupt 
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		FLEXCAN1_MCR |= FLEXCAN_MCR_FEN 	// enable Rx FIFO
					| FLEXCAN_MCR_IRMQ
					;	
		//FLEXCAN1_MCR &= ~FLEXCAN_MCR_IDAM_MASK;					
		//printf("MCR = %#08.8x\n",FLEXCAN1_MCR);
	#ifdef	TEST_IMEU_FEATURE	
		FLEXCAN1_CTRL2 |= FLEXCAN_CTRL2_IMEUEN;	// enable individual matching elements update
	#endif	
	#ifdef	TEST_SCAN_PRIORITY_MB_FIRST
		FLEXCAN1_CTRL2 |= FLEXCAN_CTRL2_MRP;
	#endif
	#ifdef	TEST_REMOTE_REQUEST_FEATURE
		//RRS ！ Remote Request Storing
		FLEXCAN1_CTRL2 |= FLEXCAN_CTRL2_RRS;
	#endif	
		FLEXCAN1_CTRL2 = (FLEXCAN1_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | rxFilterNo;
		
		// enable TSYNC mode
		FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_TSYNC;
				
		//printf("CTRL2 = %#08.8x\n",FLEXCAN1_CTRL2);
		
		// Configure filter table elements
		iNoOfFilterTableElements = rxFIFOFilterParams[rxFilterNo].rxFIFOFilterNo;
		
		printf("# of Filter Table Elements = %d\n",iNoOfFilterTableElements);
		
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			
			FLEXCAN1_IDFLT_TAB(i) = (i+1L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;
					
			//printf("Filter table element %d = %#08.8x\n",i,FLEXCAN0_IDFLT_TAB(i));
		}	
		id = i;	// save last ID for filter table
			
		// Configure inidividual mask and rx FIFO global mask	
		iNoOfFilterTableElements = min(16,iNoOfFilterTableElements);
			
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			FLEXCAN1_RXIMRn(i) = 0xFFF80000L		// matching bit 30 to bit 19
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif				
							;						
		}
		
		// Initialize the rest of individual mask registers
		for(;i<NUMBER_OF_MB;i++)
		{
			FLEXCAN1_RXIMRn(i) = 0xFFFFFFFFL;	// compare all bits
		}
		// Set global mask for rx FIFO	
		FLEXCAN1_RXFGMASK =  0xFFF80000L		// matching bit 30 to bit 19
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif		
		;					
		
		// Start FlexCAN0
		state = FLEXCAN_Start(FLEXCAN1);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Start() returned state = %#08x\n",state);
		}	
		if(!(FLEXCAN1_IMASK1 & FLEXCAN_IMASK1_BUF5M))
		{
			// 
			printf("Error: FLEXCAN_IMASK1_BUF5M is not set!\n");
		}
		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
	    mb_config.dev_num = FLEXCAN1;
	    mb_config.data_len = 8;
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
	    mb_config.direction = FLEXCAN_RX;
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
		    mb_config.remote_req_flag = TRUE;
#else
		    mb_config.remote_req_flag = 0;		    
#endif
	    mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
	    bActivate_mb = TRUE;
	    
	    // Store rx MB to the global to be accessed in isr
	    gNoMBsAvail = NoMBsAvail;
	    
	    //printf("MB %d to MB15 are available for MBs not used for rx FIFO\n",NoMBsAvail);
	    
		for(iRxMB=NoMBsAvail;iRxMB < (NUMBER_OF_MB-1);iRxMB++)	// last MB for tx
		{			
			//
#ifdef	TEST_EXTENDED_ID	
			mb_config.identifier = id++	<<18;
#else		
 	    	mb_config.identifier = id++;
#endif 	    	

  	    	//printf("rx MB%d ID = %#08.8x\n",iRxMB,mb_config.identifier);
 	    	mb_config.mailbox_number = iRxMB;
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,TRUE);
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
			}  				
			
		}			    	    	    
		// Initialize last MB as  tx MB
		iTxMB = NUMBER_OF_MB-1;
	    mb_config.dev_num = FLEXCAN1;
	    mb_config.data_len = 8;
	    bActivate_mb = FALSE;			// deactivate it after initialization of tx MB
		mb_config.mailbox_number = iTxMB;
	    mb_config.direction = FLEXCAN_TX;
	    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
	    mb_config.remote_req_flag = TRUE;
#else
	    mb_config.remote_req_flag = 0;		    
#endif
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
		    
		    for(i = 0; i< iNoOfFilterTableElements+(NUMBER_OF_MB-NoMBsAvail-1);i++)	// send additional RX_FIFO_DEEP messages
			{	
#ifdef	TEST_EXTENDED_ID					    
				mb_config.identifier = (i+1)<<18;			
#else
				mb_config.identifier = (i+1);
#endif				
		    //printf("Transmit message ID = %#08.8x (in filter table element format,shift left 1 bit)\n",mb_config.identifier<<1);
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
			}  
				
			// Write timer to some value
			FLEXCAN1_TIMER = 0xBBBB;
			
			// Activate the tx MB
			FLEXCAN1_MBn_CS(iTxMB) |=  FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE);
						
			// wait for the tx done
			while(!(FLEXCAN1_IFLAG1 & (1<<iTxMB))){}	
			
			// clear the flag
			FLEXCAN1_IFLAG1 = (1<<iTxMB);								
		}
		// wait till rx FIFO interrupts are completed 
		while(guiMB_ISR_Count<(iNoOfFilterTableElements+(NUMBER_OF_MB-NoMBsAvail-2))){}	
			
		PrintPassFailMessage(guiErrCount);				    		
	}		
}



void FlexCAN_CrcStatus_Test(void)
{
	uint_32 state; 
	uint_32 id;
	int16_t iRxMB;
	int16_t iTxMB;
	int16_t i;
	uint16_t crc;
	uint8_t bActivate_mb;	
	volatile FLEXCAN_MailBox_STRUCT rx_mb_config;
	volatile FLEXCAN_MailBox_STRUCT tx_mb_config[FLEXCAN_TX_MB_END-FLEXCAN_TX_MB_START+1];

	guiErrCount = 0;
#define TX_POLL_CRC
			
	id = 0x1ABCDEF9;
	// Initialize bit timing
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,1000,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
                                   TRUE);
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}

	#ifdef	TEST_MESSAGE_QUEUE
		FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;
		printf("Message queue is enabled!\n");
	#endif
	
	// Initialize all MBs to inactive state
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }	
     
    // Start CAN communication
    state = FLEXCAN_Start(FLEXCAN0);
  	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Start() returned state = %#08x\n",state);
	}  
    printf("FLEXCAN0_CTRL1 =%#08.8x,FLEXCAN0_MCR = %#08.8x\n",FLEXCAN0_CTRL1,FLEXCAN0_MCR);
       
    // Initialize the mailbox structure for Rx 
    rx_mb_config.dev_num = FLEXCAN0;
    rx_mb_config.data_len = 8;
    rx_mb_config.identifier = id;
    rx_mb_config.format = FLEXCAN_EXTENDED;
    rx_mb_config.direction = FLEXCAN_RX;
    rx_mb_config.remote_req_flag = 0;
    rx_mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
    bActivate_mb = TRUE;
    
    for( iRxMB = FLEXCAN_RX_MB_START; iRxMB <= FLEXCAN_RX_MB_END; iRxMB++)
    {
    	/* Initialize each Rx MB */
    	rx_mb_config.mailbox_number = iRxMB;
    	state = FLEXCAN_Initialize_mailbox(&rx_mb_config,bActivate_mb,0);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
		}  	
    }	
    
    for( iTxMB = FLEXCAN_TX_MB_START; iTxMB <= FLEXCAN_TX_MB_END; iTxMB++)
    {
    	/* Initialize each Tx MB */
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].dev_num = FLEXCAN0;
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].data_len = 8;
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].identifier = id;
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].format = FLEXCAN_EXTENDED;        	
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].direction = FLEXCAN_TX;
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].remote_req_flag = 0;
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].code = FLEXCAN_MB_CODE_TX_ONCE; 
    	tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].mailbox_number = iTxMB;
		for(i = 0; i< 8; i++)
		{
			tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].data[i] = 0xF9+i;
		}
    	
    	tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].data[0] -= iTxMB;
    	tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].crc = FlexCAN_PrepareFrameBitsWithCRC(id, FLEXCAN_EXTENDED, FALSE,
   			 (PTMSG_Data)&tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].data_len, CORRECT_FRAME );
   
    	state = FLEXCAN_Initialize_mailbox(&tx_mb_config[iTxMB-FLEXCAN_TX_MB_START],bActivate_mb,0);
    	if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
		}
#ifdef	TX_POLL_CRC		  	
		// Wait till Tx done
		while(!FLEXCAN_Is_MB_Done(FLEXCAN0,iTxMB)){};
	    {
	    	// check the CRC register to see if it is correct
	    	iRxMB=FLEXCAN_get_crc_mb(FLEXCAN0_CRCR);
	    	crc=FLEXCAN_get_crc(FLEXCAN0_CRCR);
	    	//printf("MB%d crc = %#08.8x\n",iTxMB,crc);
	    	if( (iRxMB) != iTxMB)
	    	{
	    		printf("CRCR error: MB no =%d not as expected (=%d)\n",iRxMB,iTxMB);
	    		guiErrCount++;
	    	} 
	    	if( (crc) != (tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].crc & 0x7FFF))
	    	{
	    		printf("CRCR error: crc =%#04.4x not as expected (=%#04.4x)\n",crc,tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].crc);
	    		guiErrCount++;
	    	}
	    }
#endif	    
    }
    
   do{
 	   // check received message in reverse order to support message queue
	   for( iRxMB = FLEXCAN_RX_MB_END;iRxMB >=FLEXCAN_RX_MB_START; iRxMB--)
	    {
	    	if(FLEXCAN_Is_MB_Done(FLEXCAN0,iRxMB))
	    	{
	    		// Read message from this mailbox
	    		rx_mb_config.mailbox_number = iRxMB;
	    		
			 	for(i = 0; i< 8; i++)
				{
					rx_mb_config.data[i] = 0;	// reset data
				}
	 		
				state = FLEXCAN_Rx_message(&rx_mb_config,0);
		   		if( (state != FLEXCAN_OK) & (state != FLEXCAN_MESSAGE_OVERWRITTEN))
				{
					printf("FLEXCAN_Rx_message: returned state = %#08.8x for mailbox %d\n",state,iRxMB);
				}
				else
				{
					// check received message
					printf("MB %d received a message:",iRxMB);
					for(i = 0; i < rx_mb_config.data_len; i++)
					{
						printf("%#02.2x",rx_mb_config.data[i]);
						if(i < rx_mb_config.data_len-1)
						{
							printf(",");
						}
					}	
					printf("\n");			
				} 
				// clear flag
				state = FLEXCAN_Clear_MB_Flag(FLEXCAN0,iRxMB); 
		   		if(state != FLEXCAN_OK)
				{
					printf("FLEXCAN_Clear_MB_Flag: returned state = %#08.8x for mailbox %d\n",state,iRxMB);
				}								  		
	    	}
	    }	    
   }while (1);	
	PrintPassFailMessage(guiErrCount);   
}

void FlexCAN1_CrcStatus_Test(void)
{
	uint_32 state; 
	uint_32 id;
	int16_t iRxMB;
	int16_t iTxMB;
	int16_t i;
	uint16_t crc;
	uint8_t bActivate_mb;	
	volatile FLEXCAN_MailBox_STRUCT rx_mb_config;
	volatile FLEXCAN_MailBox_STRUCT tx_mb_config[FLEXCAN_TX_MB_END-FLEXCAN_TX_MB_START+1];

	guiErrCount = 0;
#define TX_POLL_CRC
			
	id = 0x1ABCDEF9;
	// Initialize bit timing
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,1000,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
                                   TRUE
                                     );
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}

	#ifdef	TEST_MESSAGE_QUEUE
		FLEXCAN1_MCR |= FLEXCAN_MCR_IRMQ;
		printf("Message queue is enabled!\n");
	#endif
	
	// Initialize all MBs to inactive state
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }	
     
    // Start CAN communication
    state = FLEXCAN_Start(FLEXCAN1);
  	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Start() returned state = %#08x\n",state);
	}  
    printf("FLEXCAN1_CTRL1 =%#08.8x,FLEXCAN1_MCR = %#08.8x\n",FLEXCAN1_CTRL1,FLEXCAN1_MCR);
       
    // Initialize the mailbox structure for Rx 
    rx_mb_config.dev_num = FLEXCAN1;
    rx_mb_config.data_len = 8;
    rx_mb_config.identifier = id;
    rx_mb_config.format = FLEXCAN_EXTENDED;
    rx_mb_config.direction = FLEXCAN_RX;
    rx_mb_config.remote_req_flag = 0;
    rx_mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
    bActivate_mb = TRUE;
    
    for( iRxMB = FLEXCAN_RX_MB_START; iRxMB <= FLEXCAN_RX_MB_END; iRxMB++)
    {
    	/* Initialize each Rx MB */
    	rx_mb_config.mailbox_number = iRxMB;
    	state = FLEXCAN_Initialize_mailbox(&rx_mb_config,bActivate_mb,0);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
		}  	
    }	
    
    for( iTxMB = FLEXCAN_TX_MB_START; iTxMB <= FLEXCAN_TX_MB_END; iTxMB++)
    {
    	/* Initialize each Tx MB */
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].dev_num = FLEXCAN1;
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].data_len = 8;
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].identifier = id;
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].format = FLEXCAN_EXTENDED;        	
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].direction = FLEXCAN_TX;
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].remote_req_flag = 0;
	    tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].code = FLEXCAN_MB_CODE_TX_ONCE; 
    	tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].mailbox_number = iTxMB;
		for(i = 0; i< 8; i++)
		{
			tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].data[i] = 0xF9+i;
		}
    	
    	tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].data[0] -= iTxMB;
    	tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].crc = FlexCAN_PrepareFrameBitsWithCRC(id, FLEXCAN_EXTENDED, FALSE,
   			 (PTMSG_Data)&tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].data_len, CORRECT_FRAME );
   
    	state = FLEXCAN_Initialize_mailbox(&tx_mb_config[iTxMB-FLEXCAN_TX_MB_START],bActivate_mb,0);
    	if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
		}
#ifdef	TX_POLL_CRC		  	
		// Wait till Tx done
		while(!FLEXCAN_Is_MB_Done(FLEXCAN1,iTxMB)){};
	    {
	    	// check the CRC register to see if it is correct
	    	iRxMB=FLEXCAN_get_crc_mb(FLEXCAN1_CRCR);
	    	crc=FLEXCAN_get_crc(FLEXCAN1_CRCR);
	    	//printf("MB%d crc = %#08.8x\n",iTxMB,crc);
	    	if( (iRxMB) != iTxMB)
	    	{
	    		printf("CRCR error: MB no =%d not as expected (=%d)\n",iRxMB,iTxMB);
	    		guiErrCount++;
	    	} 
	    	if( (crc) != (tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].crc & 0x7FFF))
	    	{
	    		printf("CRCR error: crc =%#04.4x not as expected (=%#04.4x)\n",crc,tx_mb_config[iTxMB-FLEXCAN_TX_MB_START].crc);
	    		guiErrCount++;
	    	}
	    }
#endif	    
    }
    
   do{
 	   // check received message in reverse order to support message queue
	   for( iRxMB = FLEXCAN_RX_MB_END;iRxMB >=FLEXCAN_RX_MB_START; iRxMB--)
	    {
	    	if(FLEXCAN_Is_MB_Done(FLEXCAN1,iRxMB))
	    	{
	    		// Read message from this mailbox
	    		rx_mb_config.mailbox_number = iRxMB;
	    		
			 	for(i = 0; i< 8; i++)
				{
					rx_mb_config.data[i] = 0;	// reset data
				}
	 		
				state = FLEXCAN_Rx_message(&rx_mb_config,0);
		   		if( (state != FLEXCAN_OK) & (state != FLEXCAN_MESSAGE_OVERWRITTEN))
				{
					printf("FLEXCAN_Rx_message: returned state = %#08.8x for mailbox %d\n",state,iRxMB);
				}
				else
				{
					// check received message
					printf("MB %d received a message:",iRxMB);
					for(i = 0; i < rx_mb_config.data_len; i++)
					{
						printf("%#02.2x",rx_mb_config.data[i]);
						if(i < rx_mb_config.data_len-1)
						{
							printf(",");
						}
					}	
					printf("\n");			
				} 
				// clear flag
				state = FLEXCAN_Clear_MB_Flag(FLEXCAN1,iRxMB); 
		   		if(state != FLEXCAN_OK)
				{
					printf("FLEXCAN_Clear_MB_Flag: returned state = %#08.8x for mailbox %d\n",state,iRxMB);
				}								  		
	    	}
	    }
           if(iRxMB<0)
           {
             break;
           }
   }while (1);	
	PrintPassFailMessage(guiErrCount);   
}

void FlexCAN_IndividualMasking_Test(void)
{
	FlexCAN0_IndividualMasking_Test();
}


void FlexCAN0_IndividualMasking_Test(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;

	printf("Start to test Individual Masking for rx MBs\n");
	printf("TEST_MESSAGE_QUEUE must be defined before doing this test!\n");
 
 	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	}	 	 

	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
	
	// Initialize error counter
	guiErrCount = 0;
	
	// Initialize ISR counters
	giRxFIFOISRCount = 0;		
 	guiMB_ISR_Count = 0;
	 	
	 	
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifndef	USE_EXTERNAL_CLOCK                                    
                        FLEXCAN_IPBUS_CLK
#else
                        FLEXCAN_OSC_CLK
#endif                          
                          ,
			TRUE	// for loopback
			);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN0_RXMGMASK = 0xFFFFFFF;//0;	// receive any message without mask,can only be written in Freeze mode
 	FLEXCAN0_RX14MASK = 0;
 	FLEXCAN0_RX15MASK = 0;

	printf("case: CTRL2[EACEN]=1 -> MG31/30 to mask RTR/IDE bits\n");
	
	// Set CTRL2[EACEN] = 1 to enable MG31/30 to mask RTR and IDE
	FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_EACEN;
	
#ifdef	TEST_REMOTE_REQUEST_FEATURE
	FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_RRS;	// enable reception of remote frames
#endif	
	
	printf("FLEXCAN0_CTRL2 = %#08.8x\n",FLEXCAN0_CTRL2);	 
	// Enable individual Rx individual masking and queue enable
	// and 	enable interrupts for rx MBs
#ifdef	TEST_MESSAGE_QUEUE	
	FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;
#endif	
	
	// Initialize all RXIMRs
	for(i = 0; i<NUMBER_OF_MB;i++)
	{
		FLEXCAN0_RXIMRn(i) = 0;
	}
	
	for(i = 0; i<NUMBER_OF_MB/2;i++)
	{
		FLEXCAN0_RXIMRn(i) = 0xFFFFFFFF; //(0x1ABCDEF5+i) | BIT31 | BIT30;
		FLEXCAN0_IMASK1 |= (1<<i);
	}

	FLEXCAN0_RXIMRn(14) = 0x1ABCDEF5+14;
	FLEXCAN0_RXIMRn(15) = 0x1ABCDEF5+15;
	FLEXCAN0_IMASK1	|= 0xC000;
	
	
	// Configure rx MB
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x1ABCDEF5+i;
	  	FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY)
	 						 | FLEXCAN_MB_CS_IDE
#ifdef	TEST_REMOTE_REQUEST_FEATURE	 						 
	 						 | FLEXCAN_MB_CS_RTR
#endif	 	 						 
	 						 ;
	}	
	
	printf("start flexcan...\n");
	// Start FlexCAN0
	state = FLEXCAN_Start(FLEXCAN0);
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Start() returned state = %#08x\n",state);
	}	
	
	// Prepare transmit wanted message
	iTxMB = NUMBER_OF_MB/2;
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
	  	FLEXCAN0_MBn_CS(iTxMB) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	  	FLEXCAN0_MBn_ID(iTxMB) = 0x1ABCDEF5+i;
	  	FLEXCAN0_MBn_WORD0(iTxMB) = 0x12345678;
	  	FLEXCAN0_MBn_WORD1(iTxMB) = 0x9ABCDEF0;	  	
	  	FLEXCAN0_MBn_CS(iTxMB) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
	 						 | FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_SRR
	 						 | FLEXCAN_MB_CS_LENGTH(8)
	 						 ;	
	 						 
		// wait till tx done
		while(!(FLEXCAN0_IFLAG1 & (1<<iTxMB))){}	
		
		// clear flag
		FLEXCAN0_IFLAG1 = (1<<iTxMB);		 						  						 
	}
	
	// Check received messages
	while(guiMB_ISR_Count < NUMBER_OF_MB/2);	
				 
	guiMB_ISR_Count = 0;
	
	// Prepare to transmit unwanted message
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
	  	FLEXCAN0_MBn_CS(iTxMB) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	  	FLEXCAN0_MBn_ID(iTxMB) = i;
	  	FLEXCAN0_MBn_CS(iTxMB) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
	 						 | FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_SRR
#ifdef	TEST_REMOTE_REQUEST_FEATURE	 						 
	 						 | FLEXCAN_MB_CS_RTR
#endif	 						 
	 						 ;	
	 						 
		// wait till tx done
		while(!(FLEXCAN0_IFLAG1 & (1<<iTxMB))){}	
				 						  						 
		// clear flag
		FLEXCAN0_IFLAG1 = (1<<iTxMB);		 						  						 
	}
	// delay until timeout expires
	j = 0;
	while((guiMB_ISR_Count < NUMBER_OF_MB/2) && (j<2000))
	{
		j++;
	}	
	if(guiMB_ISR_Count)
	{
		// error: should be no ISR occurs
		guiErrCount++;
		printf("Error: rx MB received unwanted messages!\n");
	}
	guiMB_ISR_Count = 0;
	// put FlexCAN in freeze mode
	FLEXCAN0_MCR	|= FLEXCAN_MCR_HALT;
	while(!(FLEXCAN0_MCR &(FLEXCAN_MCR_FRZ_ACK))){}
	
	printf("case: CTRL2[EACEN]=0 -> MG31/30 not used to mask RTR/IDE bits\n");
	
	// Set CTRL2[EACEN] = 1 to enable MG31/30 to mask RTR and IDE
	FLEXCAN0_CTRL2 ^= FLEXCAN_CTRL2_EACEN;	
	
	// enable flexcan
	FLEXCAN0_MCR	^= FLEXCAN_MCR_HALT;
	while((FLEXCAN0_MCR &(FLEXCAN_MCR_FRZ_ACK | FLEXCAN_MCR_NOT_RDY))){}
	
	//
	// Prepare transmit wanted message
	iTxMB = NUMBER_OF_MB/2;
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
	  	FLEXCAN0_MBn_CS(iTxMB) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	  	FLEXCAN0_MBn_ID(iTxMB) = 0x1ABCDEF5+i;
	  	FLEXCAN0_MBn_CS(iTxMB) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
	 						 | FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_SRR
	 						 ;	
	 						 
		// wait till tx done
		while(!(FLEXCAN0_IFLAG1 & (1<<iTxMB))){}			 						  						 
		// clear flag
		FLEXCAN0_IFLAG1 = (1<<iTxMB);		 						  						 

	}
	// Check received messages
	while(guiMB_ISR_Count < NUMBER_OF_MB/2);	
				 
	guiMB_ISR_Count = 0;
	
	// Prepare to transmit unwanted message
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
	  	FLEXCAN0_MBn_CS(iTxMB) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	  	FLEXCAN0_MBn_ID(iTxMB) = 0x1ABCDEF5+i;
	  	FLEXCAN0_MBn_CS(iTxMB) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
	 						  | FLEXCAN_MB_CS_SRR
#ifdef	TEST_REMOTE_REQUEST_FEATURE	 						 
	 						 | FLEXCAN_MB_CS_RTR
#endif	 						 
	 						 ;	
	 						 
		// wait till tx done
		while(!(FLEXCAN0_IFLAG1 & (1<<iTxMB))){}	
				 						  						 
		// clear flag
		FLEXCAN0_IFLAG1 = (1<<iTxMB);		 						  						 
	}
	// delay until timeout expires
	j = 0;
	while((guiMB_ISR_Count < NUMBER_OF_MB/2) && (j<2000))
	{
		j++;
	}	
	if(guiMB_ISR_Count)
	{
		// error: should be no ISR occurs
		guiErrCount++;
		printf("Error: rx MB received unwanted messages!\n");
	}
	PrintPassFailMessage(guiErrCount);					
}



void FlexCAN_StdExt_IDs_Test(void)
{
}


void FlexCAN_ListenOnlyMode_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i,id;
	uint16_t err_counter;
	/*
	The module enters this mode when the LOM bit in the Control Register is asserted. 
	In this mode,transmission is disabled, all error counters are frozen and
	the module operates in a CAN Error Passive mode [Ref. 1]. 
	Only messages acknowledged by another CAN station will be received. If
	FlexCAN detects a message that has not been acknowledged, it will flag a BIT0 error 
	(without changing the REC), as if it was trying to acknowledge the message
	*/	
	
	printf("Start to test CAN behavior in Listen Only mode.\n");
	printf("FlexCAN0 and FlexCAN1 must be interconnected.\n");
 
 	// Initialize error counter
	guiErrCount = 0;
	guiMB_ISR_Count = 0;


	/* Initialize all 16 MBs of FlexCAN1 */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN1 for listen only mode
	baudrate = 83;//1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		FALSE
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	err_counter = FLEXCAN1_ECR;	// save error counter
	
	// Enable listen only mode for FlexCAN1
	FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_LOM;
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN1_RXMGMASK = 0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0;
 	FLEXCAN1_RX15MASK = 0;
 	
 	// disable error interrupts
 	FLEXCAN1_CTRL1 &= ~FLEXCAN_CTRL_ERR_MSK;
 	
			 
	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB8 to MB15 as a receive MB with ID don't care
	for(i=8;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN1_MBn_CS(i) = 0x00000000;
		FLEXCAN1_MBn_ID(i) = 0x00000000;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}
	// Initialize MB0 to MB7 as transmit MBs
	for(i=0;i<8;i++)
	{
		FLEXCAN1_MBn_CS(i) = 0x00000000;
		FLEXCAN1_MBn_ID(i) = 0x01FFA5A5+i;
		FLEXCAN1_MBn_WORD0(i) = 0x5555;
		FLEXCAN1_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(3)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}		
	FLEXCAN1_IMASK1 = 0;	// disable interrupts
	
	// Disable bus-off recovery mode
	// disable error interrupt
	//FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_BOFF_REC 
	; 	
	
	// Start CAN communication of FlexCAN1
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
		
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}	
	
	//////////////////////////////////////////////////////////////////////////////////// 
	// Now initialize FlexCAN0 to transmit some messages
	///////////////////////////////////////////////////////////////////////////////////
	/* Initialize all 16 MBs of FlexCAN0 */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	}
	
        baudrate = 125;	//        
	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		FALSE
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
		
	// Now FlexCAN is in Freeze mode.
	FLEXCAN0_RXMGMASK = 0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN0_RX14MASK = 0;
 	FLEXCAN0_RX15MASK = 0;
 	
 	// disable error interrupts
 	FLEXCAN0_CTRL1 &= ~FLEXCAN_CTRL_ERR_MSK;
 	
			 
	// clear CTRL2 but TASD
	FLEXCAN0_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB8 to MB15 as a receive MB with ID don't care
	for(i=8;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN0_MBn_CS(i) = 0x00000000;
		FLEXCAN0_MBn_ID(i) = 0x00000000;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}
	// Initialize MB0 to MB7 as transmit MBs
	for(i=0;i<8;i++)
	{
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN0_MBn_ID(i) = 0x01FFA5A5+i;
		FLEXCAN0_MBn_WORD0(i) = 0x8989;
		FLEXCAN0_MBn_WORD1(i) = 0xEBEB;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}		
	FLEXCAN0_IMASK1 = 0;	// disable interrupts
	
	// Disable bus-off recovery mode
	// disable error interrupt
	FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_BOFF_REC 
	; 	
#ifdef	TEST_SELF_RECEPTION_DISABLE
	FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;
	printf("NOTE: Self reception is disabled for FLEXCAN0!\n");
#endif	
                
	// Start CAN communication of FlexCAN0
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
		
	while(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}		
	
	// wait for some times
	DelayBits(800);//0x7000);
	
	// Check FlexCAN1 behavior in listen only mode
	// error counter not frozen?
	if(FLEXCAN1_ECR != err_counter)	
	{
		guiErrCount++;
		printf("ERROR: FLEXCAN1_ECR = %#08.8x\n",FLEXCAN1_ECR);		 
	}
	// should be in error passive mode
	state = FLEXCAN1_ESR1;	// save error information
	if(FLEXCAN_ESR_get_fault_code(state) != CAN_ERROR_PASSIVE)
	{
		guiErrCount++;	
		printf("ERROR: FLEXCAN1 not in Error Passive mode\n");	
	} 
#ifndef	TEST_ON_EVM
	// BIT0 error will be detected because no message is acknowledged for only two nodes including the listenonly node
	if(!(state & FLEXCAN_ESR_BIT0_ERR))
	{
		if(!FLEXCAN0_IFLAG1)
		{
			printf("FLEXCAN1_ESR1 = %#08.8x,FLEXCAN0_ESR1 = %#08.8x\n",state,FLEXCAN0_ESR1);
			
			// If not ACKed by FlexCAN0 (received), then it has issue with LOM mode for FlexCAN1 
			guiErrCount++;
			printf("ERROR: FLEXCAN1 BIT0 error not detected\n");
		}
	}
	// In two nodes (including listenonly node), no message will be received
	if(FLEXCAN1_IFLAG1)
	{
		guiErrCount++;
		printf("ERROR: FLEXCAN1 received messages unexpectedly!\n");
	} 	
#endif
	PrintPassFailMessage(guiErrCount);			
}


void FlexCAN_CrcFormStuf_Err_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i,id;
	TMSG_Data	msg_data;
	unsigned char	*pBytes;
	
	
	printf("Start to test CAN protocol errors on FlexCAN1 \n");
	printf("FTM1_CH0 and FlexCAN1 are interconnected.\n");
 
 	// Initialize error counter
	guiErrCount = 0;
	guiMB_ISR_Count = 0;


	/* Initialize all 16 MBs of FlexCAN1 */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 83;//250; //1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		FALSE
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN1_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
 		
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_RXIMRn(i) = 0x00000000;
	}
			 
	//	individual Rx masking and queue enable
	FLEXCAN1_MCR |= FLEXCAN_MCR_IRMQ;

	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB0 to MB14 as a receive MB with ID don't care
	for(i=0;i<NUMBER_OF_MB-1;i++)
	{
		FLEXCAN1_MBn_CS(i) = 0x00000000;
		FLEXCAN1_MBn_ID(i) = 0x00000000;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}	
	FLEXCAN1_IMASK1 = 0xFFFF;	// enable interrupts
	
	
	// Start CAN communication of FlexCAN0 and FlexCAN1
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
		
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	
	// Now generate CAN frames with errors
	id = 0x1F678957;
	msg_data.LENGTH = 8;
	pBytes = &(msg_data.BYTE0);
       // printf("Bytes:\r\n");
	for(i=0;i<msg_data.LENGTH;i++)
	{
		pBytes[i] = 0xf0+i;
              //  printf("%#02.2x ",pBytes[i]);
	}
#if 0       
        // Send this frame out via MB15
        i = 15;
	FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	FLEXCAN1_MBn_ID(i) = 0x1F678957;
	FLEXCAN1_MBn_WORD0(i) = (pBytes[0] <<24) | (pBytes[1] << 16) | (pBytes[2] << 8) | (pBytes[3]);
	FLEXCAN1_MBn_WORD1(i) = (pBytes[4] <<24) | (pBytes[5] << 16) | (pBytes[6] << 8) | (pBytes[7]);
	FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
       								 | FLEXCAN_MB_CS_SRR;
       // printf("\r\nword0 = %#08.8x,word1=%#08.8x\r\n",FLEXCAN1_MBn_WORD0(i),FLEXCAN1_MBn_WORD1(i));       
        while(1);
#endif        
	// Disable bus-off recovery mode
	// enable error interrupt
	FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_BOFF_REC 
					| FLEXCAN_CTRL_ERR_MSK
	; 	
        
        // Convert little endian to big endian bytes order
        swap_4bytes(pBytes);
        swap_4bytes(pBytes+4);
        
        printf("a correct CAN frame is generated and sent via TPM\n");        
	BuildCANFrame(id, FLEXCAN_EXTENDED, FALSE,&msg_data,CAN_FRAME_TYPE_CORRECT);
 	TPM_SendBitStream();
	
	while(CAN_Msg_Sending){}
        
	printf("a CRC error frame is sent by TPM\n");
	BuildCANFrame(id, FLEXCAN_EXTENDED, FALSE,&msg_data,CAN_FRAME_TYPE_CRC_ERROR);
 	TPM_SendBitStream();

	while(CAN_Msg_Sending){} 	
	
	printf("a FORM error frame is sent by TPM\n");
        BuildCANFrame(id, FLEXCAN_EXTENDED, FALSE,&msg_data,CAN_FRAME_TYPE_FORM_ERROR);
 	TPM_SendBitStream();
	while(CAN_Msg_Sending){}
 	
	printf("a STUF error frame is sent by TPM\n");
	BuildCANFrame(id, FLEXCAN_EXTENDED, FALSE,&msg_data,CAN_FRAME_TYPE_STUF_ERROR);
 	TPM_SendBitStream(); 	
 	while(CAN_Msg_Sending){}
 	
 	while(guiMB_ISR_Count<4);
 	
}



void FlexCAN_Error_Int_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i;
	
	
	printf("Start to test FlexCAN bit0 error handling on FlexCAN0 \n");
	printf("FlexCAN0 and FlexCAN1 are interconnected.\n");
 
 	// Initialize error counter
	guiErrCount = 0;
	guiMB_ISR_Count = 0;

  	/* Initialize all 16 MBs of FlexCAN0 */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	}
	/* Initialize all 16 MBs of FlexCAN1 */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		FALSE	// not loopback
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN0_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
 	
 	// Enable error interrupts
 	FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_ERR_MSK;
 	 
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_RXIMRn(i) = 0x00000000;
	}
			 
	//	individual Rx masking and queue enable
	FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;

	// clear CTRL2 but TASD
	FLEXCAN0_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB0 to MB7 as a receive MB with ID don't care
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
		FLEXCAN0_MBn_CS(i) = 0x00000000;
		FLEXCAN0_MBn_ID(i) = 0x00000000;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}	
	
	// Disable bus-off recovery mode
	FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_BOFF_REC; 

	// Initialize FlexCAN1
 	baudrate = 500;	// use different bit rate for FlexCAN1
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		FALSE
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN1_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
 	
 	// Enable error interrupts
 	FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_ERR_MSK;
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_RXIMRn(i) = 0x00000000;
	}
			 
	//	individual Rx masking and queue enable
	FLEXCAN1_MCR |= FLEXCAN_MCR_IRMQ;

	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB0 to MB7 as a receive MB with ID don't care
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
		FLEXCAN1_MBn_CS(i) = 0x00000000;
		FLEXCAN1_MBn_ID(i) = 0x00000000;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}	
	
	// Disable bus-off recovery mode
	FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_BOFF_REC; 	
	
	// Start CAN communication of FlexCAN0 and FlexCAN1
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
		
	while(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}

#if 1
        // Initialize MB8 to 15 as transmit MBs for FlexCANn
	for(i = NUMBER_OF_MB/2;i<NUMBER_OF_MB;i++)
	{
                FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN1_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN1_MBn_WORD0(i) = 0x5555;
		FLEXCAN1_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;	
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN0_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN0_MBn_WORD0(i) = 0x5555;
		FLEXCAN0_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	
        }	
	
#else	
	// Initialize MB8 to 15 as transmit MBs for FlexCAN0
	for(i = NUMBER_OF_MB/2;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN0_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN0_MBn_WORD0(i) = 0x5555;
		FLEXCAN0_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}	
	// Initialize MB8 to 15 as transmit MBs for FlexCAN1
	for(i = NUMBER_OF_MB/2;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN1_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN1_MBn_WORD0(i) = 0x5555;
		FLEXCAN1_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}
#endif        
	DelayBits(0x8000);	// wait for some bits time	
	
	// check error interrupts
	while(guiMB_ISR_Count<5){}	
	
	PrintPassFailMessage(guiErrCount);					
}



void FlexCAN_BusOffMode_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i;
	
	
	printf("Start to test FlexCAN Bus off mode on FlexCAN0 \n");
 
 	// Initialize error counter
	guiErrCount = 0;
	guiMB_ISR_Count = 0;

  	/* Initialize all 16 MBs of FlexCAN0 */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	}
	/* Initialize all 16 MBs of FlexCAN1 */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,
		FALSE	// not loopback
		);	
	if(state != FLEXCAN_OK)
	{
          printf("ERROR: FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN0_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_RXIMRn(i) = 0x00000000;
	}
			 
	//	individual Rx masking and queue enable
	FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;

	// clear CTRL2 but TASD
	FLEXCAN0_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB0 to MB7 as a receive MB with ID don't care
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
		FLEXCAN0_MBn_CS(i) = 0x00000000;
		FLEXCAN0_MBn_ID(i) = 0x00000000;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}	
	
	// Disable bus-off recovery mode
	FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_BOFF_REC; 

#ifdef	TEST_SELF_RECEPTION_DISABLE
	FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;
	printf("NOTE: Self reception is disabled for FLEXCAN0!\n");
#endif	        
	// Initialize FlexCAN1
 	baudrate = 500;	// use different bit rate for FlexCAN1
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,
		FALSE
		);	
	if(state != FLEXCAN_OK)
	{
          printf("ERROR: FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
         printf("After 1st time init,FLEXCAN1_CTRL1=%#08.8x\n",FLEXCAN1_CTRL1);
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN1_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_RXIMRn(i) = 0x00000000;
	}
			 
	//	individual Rx masking and queue enable
	FLEXCAN1_MCR |= FLEXCAN_MCR_IRMQ;

	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);

#ifdef	TEST_SELF_RECEPTION_DISABLE
	FLEXCAN1_MCR |= FLEXCAN_MCR_SRX_DIS;
	printf("NOTE: Self reception is disabled for FLEXCAN1!\n");
#endif		
	
	// Initialize MB0 to MB7 as a receive MB with ID don't care
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
		FLEXCAN1_MBn_CS(i) = 0x00000000;
		FLEXCAN1_MBn_ID(i) = 0x00000000;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}	
	
	// Disable bus-off recovery mode
	FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_BOFF_REC; 	
	
	// Start CAN communication of FlexCAN0 and FlexCAN1
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
		
	while(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	
	// Initialize MB8 to 15 as transmit MBs for FlexCAN0
	for(i = NUMBER_OF_MB/2;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN0_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN0_MBn_WORD0(i) = 0x5555;
		FLEXCAN0_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}	
	// Initialize MB8 to 15 as transmit MBs for FlexCAN1
	for(i = NUMBER_OF_MB/2;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN1_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN1_MBn_WORD0(i) = 0x5555;
		FLEXCAN1_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}
	//DelayBits(147);	// wait for 147 bits	

	// Put into bus-off state externally
	
	// check if bus-off flag is set
	while(!(FLEXCAN0_ESR1 & (FLEXCAN_ESR_BOFF_INT)) &&
              !(FLEXCAN1_ESR1 & (FLEXCAN_ESR_BOFF_INT)))
        {
//          printf("FLEXCAN0_ESR1=%#08.8x,FLEXCAN1_ESR1=%#08.8x\n",FLEXCAN0_ESR1,FLEXCAN1_ESR1);
            // halt for some times
        }	
        if(FLEXCAN0_ESR1 & FLEXCAN_ESR_BOFF_INT)
        {
            if((FLEXCAN_ESR_get_fault_code(FLEXCAN0_ESR1) & 0x2) != CAN_ERROR_BUS_OFF)
            {
                    // not in bus-off state
                    guiErrCount++;
                    printf("Error: FlexCAN is not in bus-off state\n");
            }	
            // check rx MBs to see if transmission is completed
            for(i=0; i < NUMBER_OF_MB/2;i++)
            {
                    if(FLEXCAN0_IFLAG1 & (1<<i))
                    {
                            guiErrCount++;
                            printf("Error: MB%d is received a message in bus-off state\n",i);
                    }
            }
            // check tx MBs
            for(i= NUMBER_OF_MB/2;i<NUMBER_OF_MB;i++)
            {
                    if(FLEXCAN0_IFLAG1 & (1<<i))
                    {
                            guiErrCount++;
                            printf("Error: MB%d sucessfully transmitted a message in bus-off state\n",i);
                    }
            }
        }
        else
        {
            if((FLEXCAN_ESR_get_fault_code(FLEXCAN1_ESR1) & 0x2) != CAN_ERROR_BUS_OFF)
            {
                    // not in bus-off state
                    guiErrCount++;
                    printf("Error: FlexCAN is not in bus-off state\n");
            }	
            // check rx MBs to see if transmission is completed
            for(i=0; i < NUMBER_OF_MB/2;i++)
            {
                    if(FLEXCAN1_IFLAG1 & (1<<i))
                    {
                            guiErrCount++;
                            printf("Error: MB%d is received a message in bus-off state\n",i);
                    }
            }
            // check tx MBs
            for(i= NUMBER_OF_MB/2;i<NUMBER_OF_MB;i++)
            {
                    if(FLEXCAN1_IFLAG1 & (1<<i))
                    {
                            guiErrCount++;
                            printf("Error: MB%d sucessfully transmitted a message in bus-off state\n",i);
                    }
            }          
        }
	
	// Enable MB interrupts for FlexCAN0
	FLEXCAN0_IMASK1 = 0xFFFF;
			
	// Enable bus-off recovery mode
	FLEXCAN0_CTRL1 ^= FLEXCAN_CTRL_BOFF_REC; 
	FLEXCAN1_CTRL1 ^= FLEXCAN_CTRL_BOFF_REC;
        
        printf("Before reinit,FLEXCAN1_CTRL1=%#08.8x\n",FLEXCAN1_CTRL1);

	// remove bus-off condition externally
	// re-initialize FlexCAN1 bit rate while in freeze mode
	baudrate = 1000;	// use same bit rate for FlexCAN1
 	  	
	// NOTE: CTRL1/2 are not affected by soft reset
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		FALSE
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	printf("After reinit,FLEXCAN1_CTRL1=%#08.8x\n",FLEXCAN1_CTRL1);
            
        // NOW clear busoff flag for FLEXCAN0
        FLEXCAN0_ESR1 = FLEXCAN_ESR_BOFF_INT;
          
	// Start CAN communication of FlexCAN0 and FlexCAN1
	state = FLEXCAN_Start(FLEXCAN1);
  	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Start(FLEXCAN1) returned state = %#08x\n",state);
	}  

	// Wait for a while
	DelayBits(0xc000);	
	
	// check if bus-off flag is set
	while((FLEXCAN0_ESR1 & (FLEXCAN_ESR_BOFF_INT))) {
         // printf("FLEXCAN0_ESR1=%#08.8x,FLEXCAN1_ESR1=%#08.8x\n",FLEXCAN0_ESR1,FLEXCAN1_ESR1);          
        }	
	if((FLEXCAN_ESR_get_fault_code(FLEXCAN0_ESR1) & 0x2) == CAN_ERROR_BUS_OFF)
	{
		// still in bus-off state
		guiErrCount++;
		printf("Error: FlexCAN0 is still in bus-off state,FLEXCAN0_ESR1=%#08.8x\n",FLEXCAN0_ESR1);
	}
	// wait for a  while
	DelayBits(0xc000);	
	
	// check all MBs
	while(guiMB_ISR_Count<8){}	
	
	PrintPassFailMessage(guiErrCount);		
}


void FlexCAN_FreezeMode_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i;
		/*
	 * When Freeze Mode is requested during transmission or reception, FlexCAN does the following:
	 * Waits to be in either Intermission, Passive Error, Bus Off or Idle state
	 * Waits for all internal activities like arbitration, matching, move-in and move-out to finish
	 * Ignores the Rx input pin and drives the Tx pin as recessive
	 * Stops the prescaler, thus halting all CAN protocol activities
	 * Grants write access to the Error Counters Register, which is read-only in other modes
	 * Sets the NOT_RDY and FRZ_ACK bits in MCR
	 */
	
	printf("Start to test FlexCAN behavior in Freeze mode on FlexCAN0 \n");
 
 	// Initialize error counter
	guiErrCount = 0;

  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,
                      TRUE	// for loopback
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN0_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_RXIMRn(i) = 0x00000000;
	}
			 
	//	individual Rx masking and queue enable
	FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;

	// clear CTRL2 but TASD
	FLEXCAN0_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB0 to MB7 as a receive MB with ID don't care
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
		FLEXCAN0_MBn_CS(i) = 0x00000000;
		FLEXCAN0_MBn_ID(i) = 0x00000000;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}	
	
	// Write timer to some value
	FLEXCAN0_TIMER = 0xCCCC;
	
	// Start CAN communication
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
	
	while(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	
	// Initialize MB8 to 15 as transmit MBs
	for(i = NUMBER_OF_MB/2;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN0_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN0_MBn_WORD0(i) = 0x5555;
		FLEXCAN0_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}	

	DelayBits(147);	// wait for 147 bits
	
	while(!(FLEXCAN0_ESR2 & FLEXCAN_ESR2_VPS )){}
	while((FLEXCAN0_ESR2 & FLEXCAN_ESR2_IMB)){}
	
	// Put flexcan in freeze mode
	FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
	
	while( !(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK)) ||
	
	        !(FLEXCAN0_MCR & FLEXCAN_MCR_NOT_RDY)) {}
	
	// Check Error counters to see if can write
	FLEXCAN0_ECR = 0x55AA;
	if(0x55AA != FLEXCAN0_ECR)
	{
		guiErrCount++;
	}
	// Check all flags to see if they are set
	for(i = 0;i<NUMBER_OF_MB;i++)
	{
		if(!(FLEXCAN0_IFLAG1 & (1<<i)))
		{
			cs[0] = FLEXCAN0_MBn_CS(i);
			printf("MB%d is not completed,C/S=%#08.8x!\n",i,cs[0]);
		}
	}
	PrintPassFailMessage(guiErrCount);	
}


void FlexCAN1_FreezeMode_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i;
		/*
	 * When Freeze Mode is requested during transmission or reception, FlexCAN does the following:
	 * Waits to be in either Intermission, Passive Error, Bus Off or Idle state
	 * Waits for all internal activities like arbitration, matching, move-in and move-out to finish
	 * Ignores the Rx input pin and drives the Tx pin as recessive
	 * Stops the prescaler, thus halting all CAN protocol activities
	 * Grants write access to the Error Counters Register, which is read-only in other modes
	 * Sets the NOT_RDY and FRZ_ACK bits in MCR
	 */
	
	printf("Start to test FlexCAN behavior in Freeze mode on FlexCAN1 \n");
 
 	// Initialize error counter
	guiErrCount = 0;

  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
 #ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,
 //TRUE	// for loopback
                FALSE
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN1_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_RXIMRn(i) = 0x00000000;
	}
			 
	//	individual Rx masking and queue enable
	FLEXCAN1_MCR |= FLEXCAN_MCR_IRMQ;

	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB0 to MB7 as a receive MB with ID don't care
	for(i=0;i<NUMBER_OF_MB/2;i++)
	{
		FLEXCAN1_MBn_CS(i) = 0x00000000;
		FLEXCAN1_MBn_ID(i) = 0x00000000;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}	
	
	// Write timer to some value
	FLEXCAN1_TIMER = 0xCCCC;
	
	// Start CAN communication
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
	
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	
	// Initialize MB8 to 15 as transmit MBs
	for(i = NUMBER_OF_MB/2;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN1_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN1_MBn_WORD0(i) = 0x5555;
		FLEXCAN1_MBn_WORD1(i) = 0xAAAA;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}	

	DelayBits1(147);	// wait for 147 bits
	
	while(!(FLEXCAN1_ESR2 & FLEXCAN_ESR2_VPS )){}
	while((FLEXCAN1_ESR2 & FLEXCAN_ESR2_IMB)){}
	
	// Put flexcan in freeze mode
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
	
	while( !(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK)) ||
	
	        !(FLEXCAN1_MCR & FLEXCAN_MCR_NOT_RDY)) {}
	
	// Check Error counters to see if can write
	FLEXCAN1_ECR = 0x55AA;
	if(0x55AA != FLEXCAN1_ECR)
	{
		guiErrCount++;
	}
	// Check all flags to see if they are set
	for(i = 0;i<NUMBER_OF_MB;i++)
	{
		if(!(FLEXCAN1_IFLAG1 & (1<<i)))
		{
			cs[0] = FLEXCAN1_MBn_CS(i);
			printf("MB%d is not completed,C/S=%#08.8x!\n",i,cs[0]);
		}
	}
	PrintPassFailMessage(guiErrCount);	
}


void FlexCAN_TxRx_Test(void)
{
#ifdef	TEST_TXRX_Test	
	//FlexCAN_SelfLoop2();
	FlexCAN1_TxRx_Test();
#endif
#ifdef	TEST_TXRX_Test2
	FlexCAN1_TxRx_Test2();
#endif			
}

	
void FlexCAN1_TxRx_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i,id,length;
	uint16_t err_counter;
	int32_t mask;
	

	
	printf("Start to test CAN transmission with variable size of data\n");

 
 	// Initialize error counter
	guiErrCount = 0;
	guiMB_ISR_Count = 0;


	/* Initialize all 16 MBs of FlexCAN1 */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN1 for listen only mode
	baudrate = 1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		TRUE	// loopback
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	err_counter = FLEXCAN1_ECR;	// save error counter
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN1_RXMGMASK = 0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0;
 	FLEXCAN1_RX15MASK = 0;
 	
 	// disable error interrupts
 	FLEXCAN1_CTRL1 &= ~FLEXCAN_CTRL_ERR_MSK;
 	
			 
	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB8 to MB15 as a receive MB with ID don't care
	for(i=8;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN1_MBn_CS(i) = 0x00000000;
		FLEXCAN1_MBn_ID(i) = 0x00000000;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}
	// Initialize MB0 to MB7 as transmit MBs
	for(i=0;i<8;i++)
	{
		FLEXCAN1_MBn_CS(i) = 0x00000000;
		FLEXCAN1_MBn_ID(i) = 0x01FFA5A5+i;
		FLEXCAN1_MBn_WORD0(i) = 0x12345678+i;
		FLEXCAN1_MBn_WORD1(i) = 0x9ABCDEF0+i;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(i)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}		
	FLEXCAN1_IMASK1 = 0;	// disable interrupts
		
	// Start CAN communication of FlexCAN1
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
		
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}	
	
	for(length = 0; length < 8; length++)
	{		
		// 
		while(!(FLEXCAN1_IFLAG1 & (1<<8))){}
		
		// lock it first
		cs[8] = FLEXCAN1_MBn_CS(8);
		id = FLEXCAN1_MBn_ID(8);
		data0[8] = FLEXCAN1_MBn_WORD0(8);
		data1[8] = FLEXCAN1_MBn_WORD1(8);
		
		// unlock it by read MB9
		state = FLEXCAN1_MBn_CS(9);
		//timer[1] = FLEXCAN1_TIMER;
		
		FLEXCAN1_IFLAG1 |= (1<<8);			// clear the flag
		
		// check ID and data, length
		if( id != (0x01FFA5A5+length))
		{
			guiErrCount++;
			printf("Error: ID receive is incorrect (%#08.8x),expect %#08.8x\n",id,0x01FFA5A5+length);
		}
		if(FLEXCAN_get_length(cs[8]) != length)
		{
			guiErrCount++;
			printf("Error: data size received is incorrect (%d), expect %d\n",FLEXCAN_get_length(cs[8]),length);
		}
		if(length > 4)
		{
			mask = (0x100000000L-(1<<((8-length)<<3)));
			
			if((data1[8] & mask) != ((0x9ABCDEF0+length) & mask))			
			{
				guiErrCount++;
				printf("Error: WORD1 received (%#08.8x) is not as expected\n",data1[8]);
			}  
			if(data0[8] != (0x12345678+length))
			{
				guiErrCount++;
				printf("Error: WORD0 received (%#08.8x) is not as expected\n",data0[8]);				
			}
		}
		else if(length >0)
		{
			mask = (0x100000000L-(1<<((4-length)<<3)));
			if( (data0[8] & mask) != ((0x12345678+length) & mask))
			{
				guiErrCount++;
				printf("Error: WORD0 received (%#08.8x) is not as expected\n",data0[8]);				
			}			
		}
	}				
	
	PrintPassFailMessage(guiErrCount);			
}


void FlexCAN1_TxRx_Test2(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i,id,length;
	uint16_t err_counter;
	int32_t mask;
	

	// Now test accessing tx MB while receiving a frame
	printf("Test accessing tx MB while receiving a frame...\n");
	
	// Initialize error counter
	guiErrCount = 0;
	guiMB_ISR_Count = 0;


	/* Initialize all 16 MBs of FlexCAN1 */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN1 for listen only mode
	baudrate = 1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		TRUE	// loopback
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	err_counter = FLEXCAN1_ECR;	// save error counter
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN1_RXMGMASK = 0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0;
 	FLEXCAN1_RX15MASK = 0;
 	
 	// disable error interrupts
 	FLEXCAN1_CTRL1 &= ~FLEXCAN_CTRL_ERR_MSK;
 	
			 
	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB8 to MB15 as a receive MB with ID don't care
	for(i=8;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN1_MBn_CS(i) = 0x00000000;
		FLEXCAN1_MBn_ID(i) = 0x00000000;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}		
	FLEXCAN1_IMASK1 = 0;	// disable interrupts
		
	// Start CAN communication of FlexCAN1
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
		
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}	
	
        NVIC_enable_PIT_interrupts();
        
	// Configure PIT clock input 
        PIT_PITMCR= 0x00;
	PIT_CH0_LDVAL = 0x2B00;	// 100Mhz system clock
	PIT_CH0_TCTRL =  PIT_TCTRL_TIE;
	
	// Send a frame via MB0	
	FLEXCAN1_MBn_CS(0) = 0x00000000;
	FLEXCAN1_MBn_ID(0) = 0x01FFA5A5+0;
	FLEXCAN1_MBn_WORD0(0) = 0x12345678+0;
	FLEXCAN1_MBn_WORD1(0) = 0x9ABCDEF0+0;
	FLEXCAN1_MBn_CS(0) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
							 | FLEXCAN_MB_CS_LENGTH(8)
							 | FLEXCAN_MB_CS_IDE
							 | FLEXCAN_MB_CS_SRR;
	
	while(!(FLEXCAN1_ESR1 & FLEXCAN_ESR_TX)) {}
	 
	// Delay to arbitration window	and in this window send frames					 
	//DelayBits1(110);
	NVIC_enable_PIT_interrupts();
	PIT_CH0_TCTRL |= PIT_TCTRL_TEN;		//  enable interrupt;
		
	i = 0;
	do
	{
		// Loop until the MB8 receives a frame
		while(!(FLEXCAN1_IFLAG1 & (1<<8))) {}
		
		
		// lock it first
		cs[8] = FLEXCAN1_MBn_CS(8);
		id = FLEXCAN1_MBn_ID(8);
		data0[8] = FLEXCAN1_MBn_WORD0(8);
		data1[8] = FLEXCAN1_MBn_WORD1(8);
		
		// unlock it by read MB9
		state = FLEXCAN1_MBn_CS(9);
		//timer[1] = FLEXCAN1_TIMER;
		
		FLEXCAN1_IFLAG1 |= (1<<8);			// clear the flag
		
		printf("Received a message: CS=%#08.8x, ID=%#08.8x,WORD0 = %#08.8x,WORD1=%#08.8x\n",cs[8],id,data0[8],data1[8]);
		
		// check ID and data, length
		if( id != (0x01FFA5A5+i))
		{
			guiErrCount++;
			printf("Error: ID receive is incorrect (%#08.8x),expect %#08.8x\n",id,0x01FFA5A5+i);
		}
		if((data1[8]) != (0x9ABCDEF0+i))	
		{
			guiErrCount++;
			printf("Error: WORD1 received (%#08.8x) is not as expected\n",data1[8]);
		}  
		if(data0[8] != (0x12345678+i))
		{
			guiErrCount++;
			printf("Error: WORD0 received (%#08.8x) is not as expected\n",data0[8]);				
		}		
		i++;
	}while (i<2);		
	
	PrintPassFailMessage(guiErrCount);		
}

void FlexCAN1_TxRx_Test3_External(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i,id,length;
	uint16_t err_counter;
	int32_t mask;
	

	// Now test accessing tx MB while receiving a frame
	printf("Test accessing tx MB while receiving a frame from external node...\n");
	
	// Initialize error counter
	guiErrCount = 0;
	guiMB_ISR_Count = 0;


	/* Initialize all 16 MBs of FlexCAN1 */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN1 for listen only mode
	baudrate = 83;//1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		FALSE	// external
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	err_counter = FLEXCAN1_ECR;	// save error counter
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN1_RXMGMASK = 0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0;
 	FLEXCAN1_RX15MASK = 0;
 	
 	// disable error interrupts
 	FLEXCAN1_CTRL1 &= ~FLEXCAN_CTRL_ERR_MSK;
 	
			 
	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);

	#ifdef	TEST_SELF_RECEPTION_DISABLE
		FLEXCAN1_MCR |= FLEXCAN_MCR_SRX_DIS;
		printf("NOTE: Self reception is disabled!\n");
	#endif	
	
	// Initialize MB8 to MB15 as a receive MB with ID don't care
	for(i=8;i<NUMBER_OF_MB;i++)
	{
		FLEXCAN1_MBn_CS(i) = 0x00000000;
		FLEXCAN1_MBn_ID(i) = 0x00000000;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}		
	FLEXCAN1_IMASK1 = 0;	// disable interrupts
		
	// Start CAN communication of FlexCAN1
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
		
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}	

      /*
        PIT interrupt vector number 84,85,86,87
       IRQ value: 84-16 = 68,69,70,71
       IRQ VEC reg: 68/32 = 2.xx
       bit to set : 68%32 = 4,5,6,7
       */
        NVIC_enable_PIT_interrupts();
                                  
	// Configure PIT clock input 
        PIT_PITMCR= 0x00;
	PIT_CH0_LDVAL = 0x2B00;	// 100Mhz system clock
        PIT_CH0_TFLG  = PIT_TFLG_TIF;
	PIT_CH0_TCTRL =  PIT_TCTRL_TIE;
	
	// Send a frame via MB0	
	FLEXCAN1_MBn_CS(0) = 0x00000000;
	FLEXCAN1_MBn_ID(0) = 0x01FFA5A5+0;
	FLEXCAN1_MBn_WORD0(0) = 0x12345678+0;
	FLEXCAN1_MBn_WORD1(0) = 0x9ABCDEF0+0;
	FLEXCAN1_MBn_CS(0) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
							 | FLEXCAN_MB_CS_LENGTH(8)
							 | FLEXCAN_MB_CS_IDE
							 | FLEXCAN_MB_CS_SRR;
	
	while(!(FLEXCAN1_ESR1 & FLEXCAN_ESR_TX)) {}
	 
	// Delay to arbitration window	and in this window send frames					 
	//DelayBits1(110);
	PIT_CH0_TCTRL |= PIT_TCTRL_TEN;		//  enable interrupt;
		
	i = 0;
	do
	{
		// Loop until the MB8 receives a frame
		while(!(FLEXCAN1_IFLAG1 & (1<<8))) {}
		
		
		// lock it first
		cs[8] = FLEXCAN1_MBn_CS(8);
		id = FLEXCAN1_MBn_ID(8);
		data0[8] = FLEXCAN1_MBn_WORD0(8);
		data1[8] = FLEXCAN1_MBn_WORD1(8);
		
		// unlock it by read MB9
		state = FLEXCAN1_MBn_CS(9);
		//timer[1] = FLEXCAN1_TIMER;
		
		FLEXCAN1_IFLAG1 |= (1<<8);			// clear the flag
		
		printf("Received a message: CS=%#08.8x, ID=%#08.8x,WORD0 = %#08.8x,WORD1=%#08.8x\n",cs[8],id,data0[8],data1[8]);
		
		// check ID and data, length
		if( id != (0x01FFA5A5+i))
		{
			guiErrCount++;
			printf("Error: ID receive is incorrect (%#08.8x),expect %#08.8x\n",id,0x01FFA5A5+i);
		}
		if((data1[8]) != (0x9ABCDEF0+i))	
		{
			guiErrCount++;
			printf("Error: WORD1 received (%#08.8x) is not as expected\n",data1[8]);
		}  
		if(data0[8] != (0x12345678+i))
		{
			guiErrCount++;
			printf("Error: WORD0 received (%#08.8x) is not as expected\n",data0[8]);				
		}		
		i++;
	}while (i<2);		
	
	PrintPassFailMessage(guiErrCount);		
}


void FlexCAN_AccessMode_Test(void)
{
	 uint32_t i;
	 
	 guiErrCount = 0;
	 guiMB_ISR_Count = 0;

#ifdef	TEST_ACCESS_MODE_FLEXCAN0_MB	 	 
	 printf("Start to test loopback mode for FlexCAN0 in user mode without LBUF set\n");
	 /* FlexCAN0 Settings */

#ifndef  USE_EXTERNAL_CLOCK
	FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
#else        
	FLEXCAN0_CTRL1 &= ~FLEXCAN_CTRL_CLK_SRC; //Source --> external crystal clock
#endif	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; // enable module
		 

	 while((FLEXCAN_MCR_LPM_ACK & FLEXCAN0_MCR));	

	// Now can apply Soft Reset
	FLEXCAN0_MCR ^= FLEXCAN_MCR_SOFT_RST;
	while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 	
    //printf("After soft reset, FLEXCAN0_MCR =%#08.8x \n",FLEXCAN0_MCR);
	
	 // Now it should be in Freeze mode  
	 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR));

	#ifdef	TEST_MESSAGE_QUEUE
		FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;
		if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ)
		printf("Message queue & individual masking feature is enabled\n");
	#endif
	
	#ifdef	TEST_SELF_RECEPTION_DISABLE
		FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;
		printf("NOTE: Self reception is disabled!\n");
	#endif
 
#if 1            
	 // Assume ip bus = 100M, baudrate = 222kbps	 
	 FLEXCAN0_CTRL1 |=     (FLEXCAN_CTRL_PRESDIV(50L)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   //FLEXCAN_CTRL_LBUF		  |	// with LBUF: D_IP_FlexCAN3_SYN - d_ip_flexcan3_syn.01.00.06.00 has bug
#ifndef	TEST_ON_EVM
	 					   FLEXCAN_CTRL_LPB			  | /* Enable Internal loopback */
#endif
	 					   FLEXCAN_CTRL_PROPSEG(0x2) 
					   );
#else
         // BAUD = 33.33K 
         FLEXCAN0_CTRL1 |=    (FLEXCAN_CTRL_PRESDIV(119)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
#ifndef	TEST_ON_EVM
	 					   FLEXCAN_CTRL_LPB			  | /* Enable Internal loopback */
#endif
	 					   
	 					   #if SAMPLING_TEST
	 					   FLEXCAN_CTRL_SMP	          |
	 					   #endif
	 					   
	 					  // FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x3)
	 					   );            
#endif         
    printf("FLEXCAN0_CTRL1 =%#08x\n",FLEXCAN0_CTRL1);					   
  /* Initialize all 16 MBs */		  
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }
	 
	 FLEXCAN0_RXMGMASK = 0x1FFBFFFF;
	 FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN0_RX15MASK = 0x1FFFFFFF;
	 
	 if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ)
	 {
	 	FLEXCAN0_RXIMRn(1) = 0x1FFBFFFFL;
	 	FLEXCAN0_RXIMRn(3) = 0x1FFBFFFFL;	 	 
	 }	
	 	 
	 // change FlexCAN to user mode
	  FLEXCAN0_MCR &= ~(FLEXCAN_MCR_SUPV);
	  
    /* De-assert Freeze Mode */
     FLEXCAN0_MCR ^= (FLEXCAN_MCR_HALT);
     
     /* wait till exit of freeze mode */
     while( FLEXCAN0_MCR & FLEXCAN_MCR_FRZ_ACK);

	 // Put flexcan to user mode
	  ConfigureAIPS_Lite();
	  EnterUserMode();
	  
	  printf("Now FlexCAN is in user mode...\n");

#ifndef	TEST_USE_POLL
	FLEXCAN0_IMASK1 = 0xF;	// enable interrupts for MB0,1,2,3
#endif

 	 // Configure MB1 and MB3 as rx MBs
 	 // MB0 and MB2 as tx MBs
   	 FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x0);
   	 FLEXCAN0_MBn_ID(1) = FLEXCAN_MB_ID_IDSTD(0xF);
 
   	 FLEXCAN0_MBn_CS(3) = FLEXCAN_MB_CS_CODE(0x0);
   	 FLEXCAN0_MBn_ID(3) = FLEXCAN_MB_ID_IDSTD(0xF);
   	 
  	 FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(0x4);
   	 FLEXCAN0_MBn_CS(3) = FLEXCAN_MB_CS_CODE(0x4);
 

 	 FLEXCAN0_MBn_CS(0) = (FLEXCAN_MB_CS_CODE(0x8) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN0_MBn_ID(0) = FLEXCAN_MB_ID_IDSTD(0xE);
	 FLEXCAN0_MBn_WORD0(0) = 0x12345678;
	 FLEXCAN0_MBn_WORD1(0) = 0x11223344;
	 


 	 FLEXCAN0_MBn_CS(2) = (FLEXCAN_MB_CS_CODE(0x8) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN0_MBn_ID(2) = FLEXCAN_MB_ID_IDSTD(0xE);
	 FLEXCAN0_MBn_WORD0(2) = 0x5A5A5A5A;
	 FLEXCAN0_MBn_WORD1(2) = 0x5555AAAA;

	 /* Pass condition:  */
#ifdef	TEST_MESSAGE_QUEUE
	{
		uiMatchValue[0] = FLEXCAN0_MBn_WORD0(0);
		uiMatchValue[1] = FLEXCAN0_MBn_WORD1(0);
		uiMatchValue[2] = FLEXCAN0_MBn_WORD0(2);
		uiMatchValue[3] = FLEXCAN0_MBn_WORD1(2);
	}
#else
	{
		uiMatchValue[0] = FLEXCAN0_MBn_WORD0(2);
		uiMatchValue[1] = FLEXCAN0_MBn_WORD1(2);
		uiMatchValue[2] = FLEXCAN0_MBn_WORD0(2);
		uiMatchValue[3] = FLEXCAN0_MBn_WORD1(2);		
	}
#endif

	 FLEXCAN0_MBn_CS(0) = (FLEXCAN_MB_CS_CODE(0xC) | FLEXCAN_MB_CS_LENGTH(0x8));
	 FLEXCAN0_MBn_CS(2) |= FLEXCAN_MB_CS_CODE(0xC); 


#ifdef	TEST_USE_POLL
	 /* Poll for MB2  Completion */
	 while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF2I));
	
	 printf("delay for some time before reading MBs\n");
 
 	 /* Poll for MB1 and MB3 Completion */
	 while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF1I));
	
		 /* Read received frame */
		 cs[1] = FLEXCAN0_MBn_CS(1);
		 id[1]= FLEXCAN0_MBn_ID(1);
		 data0[1] = FLEXCAN0_MBn_WORD0(1);
		 data1[1] = FLEXCAN0_MBn_WORD1(1);
		 timer[1] = FLEXCAN0_TIMER;
		
		// clear the flags
		FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF1I;
	
#ifdef	TEST_MESSAGE_QUEUE		
		/* If message queue is enabled, wait for queue elements done */ 
		{	
		 	while(!(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF3I));
		 	
		 	// read MB3
		 	 cs[2] = FLEXCAN0_MBn_CS(3);
			 id[2]= FLEXCAN0_MBn_ID(3);
			 data0[2] = FLEXCAN0_MBn_WORD0(3);
			 data1[2] = FLEXCAN0_MBn_WORD1(3);
			 timer[2] = FLEXCAN0_TIMER;
		 	
			// clear the flags
			FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF3I; 	
			printf("MB3 received a message!\n");
		}
#endif		 
		 	 
		 printf("CS word = %#08.8x\n", cs[1]);
		 if(data0[1] != uiMatchValue[0])
		 {
		 	error("Error: Data Mismatch in MB1 word0,received = %#08.8x,expected = %#08.8x!\n",data0[1],uiMatchValue[0]);
		 	guiErrCount++;
		 }
		 
		 //printf("data received = %#08.8x\n", data1[1]);
		 if(data1[1] != uiMatchValue[1])
		 {
		 	error("Error: Data Mismatch in MB1 word1,received = %#08.8x,expected = %#08.8x!\n",data1[1],uiMatchValue[1]);
		 	guiErrCount++;
		 }
		 
		 if(((id[1] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
		 {
		 	error("Error: MB1 got wrong ID = %#08.8x\n",id[1]);
		 	guiErrCount++;
		 }
		 
		 printf("MB1: timer  = %#08.8x\n\n", timer[1]);

#ifdef	TEST_MESSAGE_QUEUE	
		//if(FLEXCAN0_MCR & FLEXCAN_MCR_IRMQ )
		{		
			 //printf("check MB3 received message...\n");
			 if(data0[2] != uiMatchValue[2])
			 {
			 	error("Error: Data Mismatch in MB3 word0!\n");
			 	guiErrCount++;
			 }
			 
			 if(data1[2] != uiMatchValue[3])
			 {
			 	error("Error: Data Mismatch in MB3 word1!\n");
			 	guiErrCount++;
			 }
		 	 if(((id[2] & FLEXCAN_MB_ID_STD_MASK) != FLEXCAN_MB_ID_IDSTD(0xE)))
			 {
			 	error("Error: MB3 got wrong ID = %#08.8x\n",id[2]);
			 	guiErrCount++;
			 }		 
			printf("MB3: timer  = %#08.8x\n", timer[2]);
		}	
#endif		
	
#else
	while( guiMB_ISR_Count < 4);
#endif	

#else
	guiMB_ISR_Count = 0;
	
	 // Put flexcan1 to Supervise mode
	//ConfigureAIPS_Lite();
	//EnterPrivilegeMode();
	
	printf("Now FlexCAN1 is in supervise mode and test Rx FIFO filter...\n");

	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable	 

	 while((FLEXCAN_MCR_LPM_ACK & FLEXCAN1_MCR));	

	// Now can apply Soft Reset
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 		
	 // Now it should be in Freeze mode  
	 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR));

	 
	 FLEXCAN1_MCR |= (	  FLEXCAN_MCR_FEN	  |
	 					  FLEXCAN_MCR_IDAM(0x0) //ID Table Format A
						  );

#if 0              
	 // Assume ip bus = 100M, baudrate = 222kbps	 
	 FLEXCAN1_CTRL1 |=     (FLEXCAN_CTRL_PRESDIV(50L)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   //FLEXCAN_CTRL_LBUF		  |	// with LBUF: D_IP_FlexCAN3_SYN - d_ip_flexcan3_syn.01.00.06.00 has bug
#ifndef	TEST_ON_EVM
	 					   FLEXCAN_CTRL_LPB			  | /* Enable Internal loopback */
#endif
	 					   FLEXCAN_CTRL_PROPSEG(0x2) 
					   );
#else
         // BAUD = 33.33K 
         FLEXCAN1_CTRL1 |=    (FLEXCAN_CTRL_PRESDIV(119)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   
	 					   #if SAMPLING_TEST
	 					   FLEXCAN_CTRL_SMP	          |
	 					   #endif
#ifndef	TEST_ON_EVM
	 					   FLEXCAN_CTRL_LPB			  | /* Enable Internal loopback */
#endif
	 					   
	 					  // FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x3)
	 					   );            
#endif       
	 printf("FLEXCAN1_CTRL1 = %#08.8x,FLEXCAN1_MCR=%#08.8x\n",FLEXCAN1_CTRL1,FLEXCAN1_MCR);

	 FLEXCAN1_CTRL2 &= ~FLEXCAN_CTRL2_RFFN;	// 8 filters
	 
	 printf("FLEXCAN1_CTRL2 = %#08.8x\n",FLEXCAN1_CTRL2);
	 			   
	  /* Initialize ID table */
	  for(i=0;i<8;i++)
	  {
	  	FLEXCAN1_IDFLT_TAB(i) = ((i+1)<<19);
	  }
	  	  
	  for(i=8;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = ((i+1)<<18);
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
                
                FLEXCAN1_IMASK1 |= (1<<i); 
	  }
	 
	 printf("end of ID filter table init and MB init\n");
	  
	  /* Enable Frames available interrupt */
	 FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF5M ; 
	  
	 FLEXCAN1_RXFGMASK = 0x3FFFFFFF;	// receive all message in rx FIFO
	 FLEXCAN1_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	 
	 
	 printf("change flexcan1 running mode...\n");
	 	 
	  /* Enable FlexCAN1 and put it in user mode */
	 FLEXCAN1_MCR &= ~FLEXCAN_MCR_SUPV;
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_HALT);
 
 	 // leave freeze mode
 	 while(FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR){}
 
 	 // Now enter user mode
	 ConfigureAIPS_Lite();
 	 EnterUserMode();
 	 printf("FLEXCAN1 is in user mode.\n");
 		 
  	  /* Trigger Individual Rx MBs of FlexCAN1 */
  	  for(i=8;i<15;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(0x4);
	  }	
	  
	  // Send frames
	  i = 0;
	  	FLEXCAN1_MBn_CS(15) = FLEXCAN_MB_CS_CODE(8);
	  	FLEXCAN1_MBn_ID(15) = ((i+1)<<18);
	  	FLEXCAN1_MBn_WORD0(15) = 0x12345678+i;
	  	FLEXCAN1_MBn_WORD1(15) = 0x9ABCDEF0;
	  	FLEXCAN1_MBn_CS(15) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
	  						 | FLEXCAN_MB_CS_SRR
	 						 | FLEXCAN_MB_CS_LENGTH(8);
 	
 	while(	guiMB_ISR_Count < 16 ){}
#endif
 	
 	PrintPassFailMessage(guiErrCount);
	  	
}

void FlexCAN_ClkSrc_Test(void)
{
	FlexCAN_Baud_Test();
}

void FlexCAN_Baud_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;
	uint16_t baudrate_list[] = {
  //        33,
  //        83,
  //        50,
  //        100,
          125,
  //        250,
  //        500,
  //        1000
        };

	printf("Start to test baudrates on FlexCAN0 \n");
 
 	// Initialize error counter
	guiErrCount = 0;

  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	}
	// loop to scan the baudrate list
  	for(j=0; j < sizeof(baudrate_list)/sizeof(uint16_t); j++)
 	{
		baudrate = baudrate_list[j];	// Kbps
	 	printf("Loop %d,baud rate = %d\n",j,baudrate);
	 	  	
		// Initialize the FlexCAN0
		state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef	USE_EXTERNAL_CLOCK
			FLEXCAN_OSC_CLK,
#else		
			FLEXCAN_IPBUS_CLK,
#endif		
#ifdef	TEST_ON_EVM
			FALSE
#else	
			TRUE	// for loopback
#endif			
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
		FLEXCAN0_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
	 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
	 	
	 	// receive any messages
		for(i=0;i<NUMBER_OF_MB;i++)
		{
		  	FLEXCAN0_RXIMRn(i) = 0x00000000;
		}
	#ifdef	TEST_SELF_RECEPTION_DISABLE
		FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;
		printf("NOTE: Self reception is disabled!\n");
	#endif	                
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		//FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;
	
		// clear CTRL2 but TASD
		FLEXCAN0_CTRL2 &= (FLEXCAN_CTRL2_TASD);
		
		// enable TSYNC mode
		//FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_TSYNC;	
		//printf("FLEXCAN0_CTRL1=%#08.8x\n",FLEXCAN0_CTRL1);
		
		// Initialize MB0 as a receive MB with ID don't care
		FLEXCAN0_MBn_CS(0) = 0x00000000;
		FLEXCAN0_MBn_ID(0) = 0x00000000;
		FLEXCAN0_MBn_CS(0) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
				
		// Initialize MB1 as transmit MB
		FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN0_MBn_ID(1) = 0x18588885L+j;
		FLEXCAN0_MBn_WORD0(1) = 0x5555;
		FLEXCAN0_MBn_WORD1(1) = 0xAAAA;
		FLEXCAN0_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
		//printf("FLEXCAN0_MB1_CS = %#08.8x\n",FLEXCAN0_MBn_CS(1));
		
		// Write timer to some value
		FLEXCAN0_TIMER = 0x8888;
		
		// Start CAN communication
		FLEXCAN0_MCR ^= FLEXCAN_MCR_HALT;
		
		while(FLEXCAN0_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}	
		
		// Check CAN_TX pin with scope!
		
		// Wait till MB0 received a message
		while(!(FLEXCAN0_IFLAG1 &1)){}
		// read message 
		 cs[0] = FLEXCAN0_MBn_CS(0);
		 id[0]= FLEXCAN0_MBn_ID(0);
		 data0[0] = FLEXCAN0_MBn_WORD0(0);
		 data1[0] = FLEXCAN0_MBn_WORD1(0);
		 timer[0] = FLEXCAN0_TIMER;	// unlock it	
		 
		 // clear flag
		 FLEXCAN0_IFLAG1 = (1<<0);
		 if(id[0] != (0x18588885L+j))
		 {
		 	guiErrCount++;
		 	printf("MB0 received unwanted message ID=%#08.8x,C/S=%#08.8x\n",id[0],cs[0]);		 	
		 }		
 	}	
	PrintPassFailMessage(guiErrCount);	
}


void FlexCAN1_Baud_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;
	uint16_t baudrate_list[] = {
//          33,
//          83,
//          50,
//          100,
//          125,
//          250,
//          500,
          1000
        };

	printf("Start to test baudrates on FlexCAN1 \n");
 
 	// Initialize error counter
	guiErrCount = 0;

  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	// loop to scan the baudrate list
  	for(j=0; j < sizeof(baudrate_list)/sizeof(uint16_t); j++)
 	{
		baudrate = baudrate_list[j];	// Kbps
	 	printf("Loop %d,baud rate = %d\n",j,baudrate);
	 	  	
		// Initialize the FlexCAN0
		state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef	USE_EXTERNAL_CLOCK
			FLEXCAN_OSC_CLK,
#else		
			FLEXCAN_IPBUS_CLK,
#endif		
#ifdef	TEST_ON_EVM
			FALSE
#else	
			TRUE	// for loopback
#endif			
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
		FLEXCAN1_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
	 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	 	
	 	// receive any messages
		for(i=0;i<NUMBER_OF_MB;i++)
		{
		  	FLEXCAN1_RXIMRn(i) = 0x00000000;
		}
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		//FLEXCAN1_MCR |= FLEXCAN_MCR_IRMQ;
	
		// clear CTRL2 but TASD
		FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
		
		// enable TSYNC mode
		//FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_TSYNC;	
		//printf("FLEXCAN1_CTRL1=%#08.8x\n",FLEXCAN1_CTRL1);
		
		// Initialize MB0 as a receive MB with ID don't care
		FLEXCAN1_MBn_CS(0) = 0x00000000;
		FLEXCAN1_MBn_ID(0) = 0x00000000;
		FLEXCAN1_MBn_CS(0) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
				
		// Initialize MB1 as transmit MB
		FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN1_MBn_ID(1) = 0x18588885L+j;
		FLEXCAN1_MBn_WORD0(1) = 0x5555;
		FLEXCAN1_MBn_WORD1(1) = 0xAAAA;
		FLEXCAN1_MBn_CS(1) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
		//printf("FLEXCAN1_MB1_CS = %#08.8x\n",FLEXCAN0_MBn_CS(1));
		
		// Write timer to some value
		FLEXCAN1_TIMER = 0x8888;
	
	#ifdef	TEST_SELF_RECEPTION_DISABLE
		FLEXCAN1_MCR |= FLEXCAN_MCR_SRX_DIS;
		printf("NOTE: Self reception is disabled!\n");
	#endif	                
		// Start CAN communication
		FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
		
		while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}	
		
		// Check CAN_TX pin with scope!
		
		// Wait till MB0 received a message
		while(!(FLEXCAN1_IFLAG1 &1)){}
		// read message 
		 cs[0] = FLEXCAN1_MBn_CS(0);
		 id[0]= FLEXCAN1_MBn_ID(0);
		 data0[0] = FLEXCAN1_MBn_WORD0(0);
		 data1[0] = FLEXCAN1_MBn_WORD1(0);
		 timer[0] = FLEXCAN1_TIMER;	// unlock it	
		 
		 // clear flag
		 FLEXCAN1_IFLAG1 = (1<<0);
		 if(id[0] != (0x18588885L+j))
		 {
		 	guiErrCount++;
		 	printf("MB0 received unwanted message ID=%#08.8x,C/S=%#08.8x\n",id[0],cs[0]);		 	
		 }		
 	}	
	PrintPassFailMessage(guiErrCount);	
}                                  
                                  
void FlexCAN0_RxFifoFilter_Test(void)
{
#ifdef	TEST_RXFIFO_FILTER_FORMAT_A
	FlexCAN0_RxFifoFilter_Test_FormatA();
#endif
#ifdef	TEST_RXFIFO_FILTER_FORMAT_B	
	// Test Filter B
	FlexCAN0_RxFifoFilter_Test_FormatB();	
#endif	
#ifdef	TEST_RXFIFO_FILTER_FORMAT_C	
	// Test Filter C
	FlexCAN0_RxFifoFilter_Test_FormatC();	
#endif		
#ifdef	TEST_RXFIFO_FILTER_FORMAT_D
	// Test Filter D
	FlexCAN0_RxFifoFilter_Test_FormatC();	
#endif
}

void FlexCAN0_RxFifoFilter_Test_FormatA(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;

	printf("Start to test Rx FIFO filter format A\n");
 
 	/* Initialize all 16 MBs */		  
	  for(i=0;i<NUMBER_OF_MB;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }
	 	 

	 // Initialize globals
	 

	// Initialize the RxFIFO filter parameters
	for(i = 0; i < MAX_RX_FIFO_FILTER/8; i++)
	{
		rxFIFOFilterParams[i].rxFIFOFilterNo = 8*(i+1);
		rxFIFOFilterParams[i].topRxFIFOMBNo = (rxFIFOFilterParams[i].rxFIFOFilterNo>>2)+RX_FIFO_DEEP-1;	// Message Buffers occupied by Rx FIFO and ID Filter Table
		rxFIFOFilterParams[i].topAvailMBNo = (FLEXCAN0_MCR & 0x7F);		// Remaining Available Mailboxes
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByIMask = min(31,rxFIFOFilterParams[i].topRxFIFOMBNo);// Rx FIFO ID Filter Table Elements Affected by Rx Individual Masks
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].rxFIFOFilterNo-1;// Rx FIFO ID Filter Table Elements Affected by Rx FIFO Global Mask
		
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask >7?rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask:0;
		
		/*
		printf("# of rxFIFOFilter =%d, top rxFIFO MB No =%d, top AvailMBNo =%d,"
		      "topRxFIFOTabElementNoByIMask = %d,topRxFIFOTabElementNoByFGMask = %d\n",
		      rxFIFOFilterParams[i].rxFIFOFilterNo,
		      rxFIFOFilterParams[i].topRxFIFOMBNo,
		      rxFIFOFilterParams[i].topAvailMBNo,
		      rxFIFOFilterParams[i].topRxFIFOTabElementNoByIMask,
		      rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask);
	   */
	}	
	// Initialize the FlexCAN0
	baudrate = 125;	// 1Mbps
     //   baudrate = 1000;	// 1Mbps
	
	// Initialize error counter
	guiErrCount = 0;
	
	for(rxFilterNo = 0;rxFilterNo < 4;rxFilterNo++)
	//for(rxFilterNo = 0;rxFilterNo <= 4;rxFilterNo++)          
        //rxFilterNo = 0;
        //rxFilterNo = 1;
        //rxFilterNo = 2;
        //rxFilterNo = 3;
	{
		// Initialize ISR counters
		giRxFIFOISRCount = 0;		
	 	guiMB_ISR_Count = 0;
	 	
	 	
		state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifndef USE_EXTERNAL_CLOCK                                           
                        FLEXCAN_IPBUS_CLK
#else
                        FLEXCAN_OSC_CLK
#endif                                                
			,
                        TRUE	// for loopback
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
		FLEXCAN0_RXMGMASK = 0x1FFFFFFF;	// can only be written in Freeze mode
	 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		FLEXCAN0_MCR |= FLEXCAN_MCR_FEN 	// enable Rx FIFO
					| FLEXCAN_MCR_IRMQ
					;	
		//printf("MCR = %#08.8x\n",FLEXCAN0_MCR);
	#ifdef	TEST_IMEU_FEATURE	
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_IMEUEN;	// enable individual matching elements update
	#endif	
	#ifdef	TEST_SCAN_PRIORITY_MB_FIRST
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_MRP;
	#endif
	#ifdef	TEST_REMOTE_REQUEST_FEATURE
		//RRS ！ Remote Request Storing
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_RRS;
	#endif	
		//FLEXCAN0_CTRL2 = (FLEXCAN0_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | (rxFilterNo<<FLEXCAN_CTRL2_RFFN_BIT_NO);
		FLEXCAN_set_rffn(FLEXCAN0_CTRL2,rxFilterNo);
		
		FLEXCAN0_IMASK1 |= FLEXCAN_IMASK1_BUF5M;	// enable rxFIFO interrupt 
		
		printf("CTRL2 = %#08.8x\n",FLEXCAN0_CTRL2);
		
		// Configure filter table elements
		iNoOfFilterTableElements = rxFIFOFilterParams[rxFilterNo].rxFIFOFilterNo;
		
		printf("# of Filter Table Elements = %d\n",iNoOfFilterTableElements);
		
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			
			FLEXCAN0_IDFLT_TAB(i) = (i+1L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;
					
		}	
		
		// Configure inidividual mask and rx FIFO global mask	
			
		for(i = 0; i< min(16,iNoOfFilterTableElements);i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFFF0000L		// matching bit 30 to bit 19
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif	
				
							;	
			//printf("FLEXCAN0_RXIMR(%d)=%#08.8x\n",i,FLEXCAN0_RXIMRn(i));								
		}
		id = i;		
		
		// Initialize the rest of individual mask registers
		for(;i<NUMBER_OF_MB;i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFFFFFFFL;	// compare all bits
		}
		// Set global mask for rx FIFO	
		FLEXCAN0_RXFGMASK =  0xFFFF0000L		// matching bit 30 to bit 19

#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif		
		;					

		printf("FLEXCAN0_RXFGMASK = %#08.8x\n",FLEXCAN0_RXFGMASK);
		
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{				
			printf("Filter table element %d = %#08.8x\n",i,FLEXCAN0_IDFLT_TAB(i));
		}
				
			
		// Start FlexCAN0
		state = FLEXCAN_Start(FLEXCAN0);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Start() returned state = %#08x\n",state);
		}	
		if(!(FLEXCAN0_IMASK1 & FLEXCAN_IMASK1_BUF5M))
		{
			// 
			printf("Error: FLEXCAN_IMASK1_BUF5M is not set!\n");
		}
		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
	    mb_config.dev_num = FLEXCAN0;
	    mb_config.data_len = 8;
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
	    mb_config.direction = FLEXCAN_RX;
	    
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
		    mb_config.remote_req_flag = TRUE;
#else
		    mb_config.remote_req_flag = 0;		    
#endif
	    mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
	    bActivate_mb = TRUE;
	    
	    // Store rx MB to the global to be accessed in isr
	    gNoMBsAvail = NoMBsAvail;
	    
	    printf("MB %d to MB15 are available for MBs not used for rx FIFO\n",NoMBsAvail);
	    
		for(iRxMB=NoMBsAvail;iRxMB < (NUMBER_OF_MB-1);iRxMB++)	// last MB for tx
		{			
			//
#ifdef	TEST_EXTENDED_ID			 
 	    	mb_config.identifier = ++id<<18;
#else
	    	mb_config.identifier = ++id;
#endif 	    		
  	    	printf("rx MB%d ID = %#08.8x\n",iRxMB,id); //mb_config.identifier<<18);
 	    	mb_config.mailbox_number = iRxMB;
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,TRUE);
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
			}  				
			
		}
		if(iRxMB <= NUMBER_OF_MB-1)
		{
			// Initialize last MB as  tx MB
			iTxMB = iRxMB;
			mb_config.mailbox_number = iTxMB;
		    mb_config.direction = FLEXCAN_TX;
		    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
		    mb_config.remote_req_flag = TRUE;
#else
		    mb_config.remote_req_flag = 0;		    
#endif
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
		    
		    for(i = 0; i< iNoOfFilterTableElements+(NUMBER_OF_MB-NoMBsAvail-1)+2;i++)	// send additional 2 messages
			{	
#ifdef	TEST_EXTENDED_ID					    
				mb_config.identifier = (i+1)<<18;			
#else
				mb_config.identifier = (i+1);
#endif				
		    	//printf("Transmit message ID = %#08.8x (in filter table element format,shift left 1 bit)\n",mb_config.identifier<<1);
		    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
				if(state != FLEXCAN_OK)
				{
					printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
				}  	
				// wait for the tx done
				while(!FLEXCAN_Is_MB_Done(FLEXCAN0,iTxMB)){}	
				
				// clear the flag
				FLEXCAN0_IFLAG1 = (1<<iTxMB);								
			}
		}
		// wait till rx FIFO interrupt and MB interrupts are completed.
		while(guiMB_ISR_Count<(iNoOfFilterTableElements+(NUMBER_OF_MB-NoMBsAvail-1))){}		
	}
	PrintPassFailMessage(guiErrCount);				    			
}



void FlexCAN0_RxFifoFilter_Test_FormatB(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;

	printf("Start to test Rx FIFO filter format B\n");
 
 	/* Initialize all 16 MBs */		  
	  for(i=0;i<NUMBER_OF_MB;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }
	 	 

	 // Initialize globals
	 

	// Initialize the RxFIFO filter parameters
	for(i = 0; i < MAX_RX_FIFO_FILTER/8; i++)
	{
		rxFIFOFilterParams[i].rxFIFOFilterNo = 8*(i+1);
		rxFIFOFilterParams[i].topRxFIFOMBNo = (rxFIFOFilterParams[i].rxFIFOFilterNo>>2)+RX_FIFO_DEEP-1;	// Message Buffers occupied by Rx FIFO and ID Filter Table
		rxFIFOFilterParams[i].topAvailMBNo = (FLEXCAN0_MCR & 0x7F);		// Remaining Available Mailboxes
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByIMask = min(31,rxFIFOFilterParams[i].topRxFIFOMBNo);// Rx FIFO ID Filter Table Elements Affected by Rx Individual Masks
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].rxFIFOFilterNo-1;// Rx FIFO ID Filter Table Elements Affected by Rx FIFO Global Mask
		
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask >7?rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask:0;		
	}	
	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
	
	// Initialize error counter
	guiErrCount = 0;
	
	for(rxFilterNo = 0;rxFilterNo <= 3;rxFilterNo++)
	//rxFilterNo = 4;	// no MB available
	{
		// Initialize ISR counters
		giRxFIFOISRCount = 0;		
	 	guiMB_ISR_Count = 0;
	 	
	 	
		state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifndef USE_EXTERNAL_CLOCK                                           
                        FLEXCAN_IPBUS_CLK
#else
                        FLEXCAN_OSC_CLK
#endif                          
                          ,
			TRUE	// for loopback
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
		FLEXCAN0_RXMGMASK = 0x1FFBFFFF;	// can only be written in Freeze mode
	 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		FLEXCAN0_MCR |= FLEXCAN_MCR_FEN 	// enable Rx FIFO
					| FLEXCAN_MCR_IRMQ
					| (FLEXCAN_MCR_IDAM(FILTER_FORMAT_B))
					;	
		//printf("MCR = %#08.8x\n",FLEXCAN0_MCR);
	#ifdef	TEST_IMEU_FEATURE	
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_IMEUEN;	// enable individual matching elements update
	#endif	
	#ifdef	TEST_SCAN_PRIORITY_MB_FIRST
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_MRP;
	#endif
	#ifdef	TEST_REMOTE_REQUEST_FEATURE
		//RRS ！ Remote Request Storing
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_RRS;
	#endif	
		//FLEXCAN0_CTRL2 = (FLEXCAN0_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | (rxFilterNo<<FLEXCAN_CTRL2_RFFN_BIT_NO);
		FLEXCAN_set_rffn(FLEXCAN0_CTRL2,rxFilterNo);
		
		FLEXCAN0_IMASK1 |= FLEXCAN_IMASK1_BUF5M;	// enable rxFIFO interrupt 
		
		printf("CTRL2 = %#08.8x\n",FLEXCAN0_CTRL2);
		
		// Configure filter table elements
		iNoOfFilterTableElements = rxFIFOFilterParams[rxFilterNo].rxFIFOFilterNo;
		
		printf("# of Filter Table Elements = %d\n",iNoOfFilterTableElements);
		
		for(i = 0; i< iNoOfFilterTableElements*2;i += 2)
		{
			// uppder half-word filter 
			FLEXCAN0_IDFLT_TAB(i/2) = (((i+1L)<< FILTER0_SHIFT_BITS_UPPER) & 0x1FFFFFFL)	// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
									;
			// low half-word filter
			FLEXCAN0_IDFLT_TAB(i/2) |= (((i+2L)<< FILTER0_SHIFT_BITS_LOWER) & 0x3FFF)	// 11msb of ID: 13-3
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT15					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT14 
#endif																						
						;						
					
			//printf("Filter table element %d = %#08.8x\n",i/2,FLEXCAN0_IDFLT_TAB(i/2));
		}	
		id = i;	// i is ok // save last ID for filter table
		
		// Configure inidividual mask and rx FIFO global mask			
		for(i = 0; i< min(16,iNoOfFilterTableElements);i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFFFFFFFL; 		// matching whole bits								
		}
	
		
		// Initialize the rest of individual mask registers
		for(;i<NUMBER_OF_MB;i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFFFFFFFL;	// compare all bits
		}
		// Set global mask for rx FIFO	
		FLEXCAN0_RXFGMASK =  0xFFFFFFFFL 		// matching bit 30 to bit 19
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif		
		;					
		
		// Start FlexCAN0
		state = FLEXCAN_Start(FLEXCAN0);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Start() returned state = %#08x\n",state);
		}	
		if(!(FLEXCAN0_IMASK1 & FLEXCAN_IMASK1_BUF5M))
		{
			// 
			printf("Error: FLEXCAN_IMASK1_BUF5M is not set!\n");
		}
		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
	    mb_config.dev_num = FLEXCAN0;
	    mb_config.data_len = 8;
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
	    mb_config.direction = FLEXCAN_RX;
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
		    mb_config.remote_req_flag = TRUE;
#else
		    mb_config.remote_req_flag = 0;		    
#endif
	    mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
	    bActivate_mb = TRUE;
	    
	    // Store rx MB to the global to be accessed in isr
	    gNoMBsAvail = NoMBsAvail;
	    
	    //printf("MB %d to MB15 are available for MBs not used for rx FIFO\n",NoMBsAvail);
	    
		for(iRxMB=NoMBsAvail;iRxMB < (NUMBER_OF_MB-1);iRxMB++)	// last MB for tx
		{			
			//
#ifdef	TEST_EXTENDED_ID	
			mb_config.identifier = ++id	<<18;;//id++	<<18;
#else		
 	    	mb_config.identifier = ++id;//id++;
#endif 	    	

  	    	//printf("rx MB%d ID = %#08.8x\n",iRxMB,mb_config.identifier);
 	    	mb_config.mailbox_number = iRxMB;
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,TRUE);
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
			}  				
			
		}
		if(iRxMB <= NUMBER_OF_MB-1)
		{
			// Initialize last MB as  tx MB
			iTxMB = iRxMB;
			mb_config.mailbox_number = iTxMB;
		    mb_config.direction = FLEXCAN_TX;
		    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
		    mb_config.remote_req_flag = TRUE;
#else
		    mb_config.remote_req_flag = 0;		    
#endif
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
		    
		    for(i = 0; i< iNoOfFilterTableElements*2+(NUMBER_OF_MB-NoMBsAvail-1)+2;i++)	// send additional 2 messages
			{	
#ifdef	TEST_EXTENDED_ID
				mb_config.identifier = (i+1)<<18;	
#else
				mb_config.identifier = (i+1);
#endif					
			
		    	//printf("Transmit message ID = %#08.8x (in filter table element format,shift left 1 bit)\n",mb_config.identifier<<1);
		    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
				if(state != FLEXCAN_OK)
				{
					printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
				}  	
				// wait for the tx done
				while(!FLEXCAN_Is_MB_Done(FLEXCAN0,iTxMB)){}	
				
				// clear the flag
				FLEXCAN0_IFLAG1 = (1<<iTxMB);								
			}
		}
		// wait till rx FIFO interrupt and MB interrupts are completed.
		while(guiMB_ISR_Count<(iNoOfFilterTableElements*2+(NUMBER_OF_MB-NoMBsAvail-1))){}					    		
	}
	PrintPassFailMessage(guiErrCount);		
}


void FlexCAN0_RxFifoFilter_Test_FormatC(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;

#ifdef	TEST_RXFIFO_FILTER_FORMAT_D
	printf("Start to test Rx FIFO filter format D\n");
#else
	printf("Start to test Rx FIFO filter format C\n");
#endif	
 
 	/* Initialize all 16 MBs */		  
	  for(i=0;i<NUMBER_OF_MB;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }
	 	 

	 // Initialize globals
	 

	// Initialize the RxFIFO filter parameters
	for(i = 0; i < MAX_RX_FIFO_FILTER/8; i++)
	{
		rxFIFOFilterParams[i].rxFIFOFilterNo = 8*(i+1);
		rxFIFOFilterParams[i].topRxFIFOMBNo = (rxFIFOFilterParams[i].rxFIFOFilterNo>>2)+RX_FIFO_DEEP-1;	// Message Buffers occupied by Rx FIFO and ID Filter Table
		rxFIFOFilterParams[i].topAvailMBNo = (FLEXCAN0_MCR & 0x7F);		// Remaining Available Mailboxes
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByIMask = min(31,rxFIFOFilterParams[i].topRxFIFOMBNo);// Rx FIFO ID Filter Table Elements Affected by Rx Individual Masks
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].rxFIFOFilterNo-1;// Rx FIFO ID Filter Table Elements Affected by Rx FIFO Global Mask
		
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask >7?rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask:0;	
	}	
	// Initialize the FlexCAN0
	baudrate = 125;//1000;	// 1Mbps
	
	// Initialize error counter
	guiErrCount = 0;
	
//	for(rxFilterNo = 0;rxFilterNo <= 4;rxFilterNo++)
	rxFilterNo = 3;
	{
		// Initialize ISR counters
		giRxFIFOISRCount = 0;		
	 	guiMB_ISR_Count = 0;
	 	
	 	
		state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifndef USE_EXTERNAL_CLOCK                                           
                        FLEXCAN_IPBUS_CLK
#else
                        FLEXCAN_OSC_CLK
#endif     
                         ,
			TRUE	// for loopback
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
		FLEXCAN0_RXMGMASK = 0x1FFBFFFF;	// can only be written in Freeze mode
	 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		FLEXCAN0_MCR |= FLEXCAN_MCR_FEN 	// enable Rx FIFO
					| FLEXCAN_MCR_IRMQ
#ifdef	TEST_RXFIFO_FILTER_FORMAT_D
					| (FLEXCAN_MCR_IDAM(FILTER_FORMAT_D))
#else					
					| (FLEXCAN_MCR_IDAM(FILTER_FORMAT_C))
#endif					
					;	
		//printf("MCR = %#08.8x\n",FLEXCAN0_MCR);
	#ifdef	TEST_IMEU_FEATURE	
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_IMEUEN;	// enable individual matching elements update
	#endif	
	#ifdef	TEST_SCAN_PRIORITY_MB_FIRST
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_MRP;
	#endif
	#ifdef	TEST_REMOTE_REQUEST_FEATURE
		//RRS ！ Remote Request Storing
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_RRS;
	#endif
		//FLEXCAN0_CTRL2 = (FLEXCAN0_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | (rxFilterNo<<FLEXCAN_CTRL2_RFFN_BIT_NO);
		FLEXCAN_set_rffn(FLEXCAN0_CTRL2,rxFilterNo);
		
		FLEXCAN0_IMASK1 |= FLEXCAN_IMASK1_BUF5M;	// enable rxFIFO interrupt 
		
		printf("CTRL2 = %#08.8x\n",FLEXCAN0_CTRL2);
		
		// Configure filter table elements
		iNoOfFilterTableElements = rxFIFOFilterParams[rxFilterNo].rxFIFOFilterNo;
		
		printf("# of Filter Table Elements = %d\n",iNoOfFilterTableElements);
		
		for(i = 0; i< iNoOfFilterTableElements*4;i += 4)
		{
			// uppder 8-bit filters: filter 0,1
			FLEXCAN0_IDFLT_TAB(i/4) = (((i+1L)<< 24) & 0xFF000000L) | ((i+2)<<16) & 0x00FF0000L;	// 8msb of ID: 29 - 22, etc																			
			// low 8-bit filters: filter 2,3
			FLEXCAN0_IDFLT_TAB(i/4) |= (((i+3L)<< 8) & 0x0000FF00L) | (i+4) & 0x000000FFL;										
			//printf("Filter table element %d = %#08.8x\n",i/4,FLEXCAN0_IDFLT_TAB(i/4));
		}	
		id = i;	// save last ID of filter table
		
		// Configure inidividual mask and rx FIFO global mask				
		for(i = 0; i< min(16,iNoOfFilterTableElements);i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFFFFFFFL; //0xFFF8FFF8;//		// matching whole bits								
		}
	
		
		// Initialize the rest of individual mask registers
		for(;i<NUMBER_OF_MB;i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFFFFFFFL;	// compare all bits
		}
		// Set global mask for rx FIFO	
		FLEXCAN0_RXFGMASK =  0xFFFFFFFF		// matching bit 30 to bit 19
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif		
		;					
		
		// Start FlexCAN0
		state = FLEXCAN_Start(FLEXCAN0);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Start() returned state = %#08x\n",state);
		}	
		if(!(FLEXCAN0_IMASK1 & FLEXCAN_IMASK1_BUF5M))
		{
			// 
			printf("Error: FLEXCAN_IMASK1_BUF5M is not set!\n");
		}
		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
	    mb_config.dev_num = FLEXCAN0;
	    mb_config.data_len = 8;
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
	    mb_config.direction = FLEXCAN_RX;
	    mb_config.remote_req_flag = 0;
	    mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
	    bActivate_mb = TRUE;
	    
	    // Store rx MB to the global to be accessed in isr
	    gNoMBsAvail = NoMBsAvail;
	    
	    //printf("MB %d to MB15 are available for MBs not used for rx FIFO\n",NoMBsAvail);
	    
		for(iRxMB=NoMBsAvail;iRxMB < (NUMBER_OF_MB-1);iRxMB++)	// last MB for tx
		{			
			//
#ifdef	TEST_EXTENDED_ID	
			mb_config.identifier = ++id<<21;//id++	<<21;
#else		
 	    	mb_config.identifier = ++id<<3;//id++ <<3;
#endif 	    	

  	    	//printf("rx MB%d ID = %#08.8x\n",iRxMB,mb_config.identifier);
 	    	mb_config.mailbox_number = iRxMB;
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,TRUE);
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
			}  				
			
		}
		if(iRxMB <= NUMBER_OF_MB-1)
		{
			// Initialize last MB as  tx MB
			iTxMB = iRxMB;
			mb_config.mailbox_number = iTxMB;
		    mb_config.direction = FLEXCAN_TX;
		    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
		    mb_config.remote_req_flag = TRUE;
#else
		    mb_config.remote_req_flag = 0;		    
#endif
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
		    
		    for(i = 0; i< iNoOfFilterTableElements*4+(NUMBER_OF_MB-NoMBsAvail-1)+2;i++)	// send additional 2 messages
			{	
#ifdef	TEST_EXTENDED_ID
				mb_config.identifier = (i+1)<<21;	
#else
				mb_config.identifier = (i+1)<<3;
#endif					
			
		    	//printf("Transmit message ID = %#08.8x (in filter table element format,shift left 1 bit)\n",mb_config.identifier<<1);
		    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
				if(state != FLEXCAN_OK)
				{
					printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
				}  	
				// wait for the tx done
				while(!FLEXCAN_Is_MB_Done(FLEXCAN0,iTxMB)){}	
				
				// clear the flag
				FLEXCAN0_IFLAG1 = (1<<iTxMB);								
			}
		}
		// wait till rx FIFO interrupt and MB interrupts are completed.
		while(guiMB_ISR_Count<(iNoOfFilterTableElements*4+(NUMBER_OF_MB-NoMBsAvail-1))){}					    		
	}
	PrintPassFailMessage(guiErrCount);			
}


void FlexCAN_SafeReConfigurationMBs_Test(void)
{
	// this is to test IMEU feature
	uint_32 state; 
	uint_32 id;
	int16_t iRxMB;
	int16_t iTxMB;
	int16_t i;
	int8_t	bIntrp;
	uint8_t bActivate_mb;
	uint16_t baudrate;
	volatile FLEXCAN_MailBox_STRUCT mb_config;
	
	printf("Start to test IMEU feature\n");
	
	guiMB_ISR_Count  = 0;
    guiErrCount = 0;
    iRxMBISRcount = 0;
    

	bIntrp = TRUE;	 
  	id = 0x1ABCDEF9;
	baudrate = 1000;//100;//1000;	// 1000Kbps
	for(i = 0; i< 8; i++)
	{
		mb_config.data[i] = 0xF9+i;
	}
	// Initialize bit timing
	state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,
#ifndef	TEST_ON_EVM	
	TRUE	// for loopback
#else
	FALSE
#endif	
	);
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}

	#ifdef	TEST_MESSAGE_QUEUE
		FLEXCAN0_MCR |= FLEXCAN_MCR_IRMQ;
		printf("Message Queue is enabled!\n");
		// set individual masking
	    for( iRxMB = FLEXCAN_RX_MB_START; iRxMB <= FLEXCAN_RX_MB_END; iRxMB++)
		{
			FLEXCAN0_RXIMRn(iRxMB) = 0; // do not care
		}
	#endif

	// Enable IMEU feature
	FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_IMEUEN;

#if 0   
    FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_LOSTRLMSK | FLEXCAN_CTRL2_LOSTRMMSK 
#if 1 
    	| FLEXCAN_CTRL2_IMEUMASK
#endif 	
	;
#endif    
    // Start CAN communication
     state = FLEXCAN_Start(FLEXCAN0);
  	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Start() returned state = %#08x\n",state);
	}  
       
    // Initialize the mailbox structure for Rx 
    mb_config.dev_num = FLEXCAN0;
    mb_config.data_len = 8;
    mb_config.identifier = id;
    mb_config.format = FLEXCAN_EXTENDED;
    mb_config.direction = FLEXCAN_RX;
    mb_config.remote_req_flag = 0;
    mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
    bActivate_mb = TRUE;
    
    for( iRxMB = FLEXCAN_RX_MB_START; iRxMB <= FLEXCAN_RX_MB_END; iRxMB++)
    {
    	/* Initialize each Rx MB */
    	mb_config.mailbox_number = iRxMB;
    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,bIntrp);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
		}  	
    }	
    mb_config.direction = FLEXCAN_TX;
    mb_config.remote_req_flag = 0;
    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
  	#if 1 
	    FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_LOSTRLMSK | FLEXCAN_CTRL2_LOSTRMMSK 
	    	| FLEXCAN_CTRL2_IMEUMASK
	    	; 
	#endif    
    
    for( iTxMB = FLEXCAN_TX_MB_START; iTxMB <= FLEXCAN_TX_MB_END; iTxMB++)
    {
    	/* Initialize each Tx MB */
    	mb_config.mailbox_number = iTxMB;
    	mb_config.data[0]--;
    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,bIntrp);
    	if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
		}  
	    mb_config.dev_num = FLEXCAN0;
	    mb_config.identifier = id+2-5;			// change ID of the Rx MB
	    mb_config.format = FLEXCAN_EXTENDED;
	    mb_config.direction = FLEXCAN_RX;
	    mb_config.remote_req_flag = 0;
	    mb_config.mailbox_number = FLEXCAN_RX_MB_START;
	    
	    printf("Now to safe reconfig the MB%d\n",mb_config.mailbox_number);
	    state = FLEXCAN_Reconfig_Rx_mailbox_block(&mb_config);
	    if(state != FLEXCAN_OK)
	    {
	    	printf("state = %#08.8x returned by FLEXCAN_Reconfig_Rx_mailbox_block for MB%d\n",state,mb_config.mailbox_number);
	    }			
    }
     
    // Now safe reconfiguration of a Rx MB
    for(i = 0; i < 3; i++)
    {
	    mb_config.dev_num = FLEXCAN0;
	    mb_config.identifier = id+2-i;//id+8;			// change ID of the Rx MB
	    mb_config.format = FLEXCAN_EXTENDED;
	    mb_config.direction = FLEXCAN_RX;
	    mb_config.remote_req_flag = 0;
	    mb_config.mailbox_number = FLEXCAN_RX_MB_START+i;
	    
	    printf("Now to safe reconfig the MB%d\n",mb_config.mailbox_number);
	    state = FLEXCAN_Reconfig_Rx_mailbox_block(&mb_config);
	    if(state != FLEXCAN_OK)
	    {
	    	printf("state = %#08.8x returned by FLEXCAN_Reconfig_Rx_mailbox_block for MB%d\n",state,mb_config.mailbox_number);
	    }
    }
    while(iRxMBISRcount	 < (FLEXCAN_TX_MB_END-FLEXCAN_TX_MB_START+1)){}
}


void FlexCAN0_SafeReConfigurationFIFO_Test_FormatA(void)
{
	uint16_t baudrate;
	uint32_t state,id;
	uint8_t  rxFilterNo,bActivate_mb;
	int32_t	 i,j,NoMBsAvail,iRxMB,iTxMB,iNoOfFilterTableElements;
	volatile FLEXCAN_MailBox_STRUCT mb_config;
	uint8_t  iFIFOFilterTabElement2Update;
	int16_t iMatchHit;

	printf("Start to test Rx FIFO filter format A with safe reconfiguration\n");
 
 	/* Initialize all 16 MBs */		  
	  for(i=0;i<NUMBER_OF_MB;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }
	 	 

	 // Initialize globals
	 

	// Initialize the RxFIFO filter parameters
	for(i = 0; i < MAX_RX_FIFO_FILTER/8; i++)
	{
		rxFIFOFilterParams[i].rxFIFOFilterNo = 8*(i+1);
		rxFIFOFilterParams[i].topRxFIFOMBNo = (rxFIFOFilterParams[i].rxFIFOFilterNo>>2)+RX_FIFO_DEEP-1;	// Message Buffers occupied by Rx FIFO and ID Filter Table
		rxFIFOFilterParams[i].topAvailMBNo = (FLEXCAN0_MCR & 0x7F);		// Remaining Available Mailboxes
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByIMask = min(31,rxFIFOFilterParams[i].topRxFIFOMBNo);// Rx FIFO ID Filter Table Elements Affected by Rx Individual Masks
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].rxFIFOFilterNo-1;// Rx FIFO ID Filter Table Elements Affected by Rx FIFO Global Mask
		
		rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask = rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask >7?rxFIFOFilterParams[i].topRxFIFOTabElementNoByFGMask:0;		
	}	
	// Initialize the FlexCAN0
	baudrate = 1000;	// 1Mbps
	
	// Initialize error counter
	guiErrCount = 0;
	
	//for(rxFilterNo = 0;rxFilterNo <= 4;rxFilterNo++)
	rxFilterNo = 3;
	{
		// Initialize ISR counters
		giRxFIFOISRCount = 0;		
	 	guiMB_ISR_Count = 0;
	 	
	 	
		state = FLEXCAN_Initialize(FLEXCAN0, 0, 0,baudrate,
 #ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,
                          TRUE	// for loopback
			);	
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
		}	
		
		// Now FlexCAN is in Freeze mode.
		FLEXCAN0_RXMGMASK = 0x1FFFFFFF;	// can only be written in Freeze mode
	 	FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 	FLEXCAN0_RX15MASK = 0x1FFFFFFF;
				 
		//	individual Rx masking and queue enable and MRP ！ Mailboxes Reception Priority = 0
		FLEXCAN0_MCR |= FLEXCAN_MCR_FEN 	// enable Rx FIFO
					| FLEXCAN_MCR_IRMQ
					;	
		//printf("MCR = %#08.8x\n",FLEXCAN0_MCR);
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_IMEUEN;	// enable individual matching elements update
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_LOSTRLMSK;	// enable lost RX Locked interrupt
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_LOSTRMMSK;	// enable lost RX Matched interrupt
	#ifdef	TEST_SCAN_PRIORITY_MB_FIRST
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_MRP;
	#endif
	#ifdef	TEST_REMOTE_REQUEST_FEATURE
		//RRS ！ Remote Request Storing
		FLEXCAN0_CTRL2 |= FLEXCAN_CTRL2_RRS;
	#endif	
		//FLEXCAN0_CTRL2 = (FLEXCAN0_CTRL2 & ~(FLEXCAN_CTRL2_RFFN)) | (rxFilterNo<<FLEXCAN_CTRL2_RFFN_BIT_NO);
		FLEXCAN_set_rffn(FLEXCAN0_CTRL2,rxFilterNo);
		
		FLEXCAN0_IMASK1 &= ~FLEXCAN_IMASK1_BUF5M;	// disable rxFIFO interrupt 
		
		printf("CTRL2 = %#08.8x\n",FLEXCAN0_CTRL2);
		
		// Configure filter table elements
		iNoOfFilterTableElements = rxFIFOFilterParams[rxFilterNo].rxFIFOFilterNo;
		
		printf("# of Filter Table Elements = %d\n",iNoOfFilterTableElements);
		
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{
			
			FLEXCAN0_IDFLT_TAB(i) = (i+1L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;
					
		}	
		
		// Configure inidividual mask and rx FIFO global mask	
			
		for(i = 0; i< min(16,iNoOfFilterTableElements);i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFFF0000L		// matching bit 30 to bit 19
#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif	
				
							;	
			//printf("FLEXCAN0_RXIMR(%d)=%#08.8x\n",i,FLEXCAN0_RXIMRn(i));								
		}
		id = i;		
		
		// Initialize the rest of individual mask registers
		for(;i<NUMBER_OF_MB;i++)
		{
			FLEXCAN0_RXIMRn(i) = 0xFFFFFFFFL;	// compare all bits
		}
		// Set global mask for rx FIFO	
		FLEXCAN0_RXFGMASK =  0xFFFF0000L		// matching bit 30 to bit 19

#ifdef	TEST_EXTENDED_ID			 
							 | BIT1
#endif		
		;					

		printf("FLEXCAN0_RXFGMASK = %#08.8x\n",FLEXCAN0_RXFGMASK);
		
		for(i = 0; i< iNoOfFilterTableElements;i++)
		{				
			//printf("Filter table element %d = %#08.8x\n",i,FLEXCAN0_IDFLT_TAB(i));
		}
				
			
		// Start FlexCAN0
		state = FLEXCAN_Start(FLEXCAN0);
		if(state != FLEXCAN_OK)
		{
			printf("FLEXCAN_Start() returned state = %#08x\n",state);
		}	

		// Initialize rx MBs and tx MBs
		NoMBsAvail = rxFIFOFilterParams[rxFilterNo].topRxFIFOMBNo+1;
		
	   // Initialize the mailbox structure for Rx 
		for(i = 0; i< 8; i++)
		{
			mb_config.data[i] = 0xF9+i;
		}
	    mb_config.dev_num = FLEXCAN0;
	    mb_config.data_len = 8;
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
	    mb_config.direction = FLEXCAN_RX;
	    
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
		    mb_config.remote_req_flag = TRUE;
#else
		    mb_config.remote_req_flag = 0;		    
#endif
	    mb_config.code = FLEXCAN_MB_CODE_RX_EMPTY; 
	    bActivate_mb = TRUE;
	    
	    // Store rx MB to the global to be accessed in isr
	    gNoMBsAvail = NoMBsAvail;
	    
	    printf("MB %d to MB15 are available for MBs not used for rx FIFO\n",NoMBsAvail);
	    
		for(iRxMB=NoMBsAvail;iRxMB < (NUMBER_OF_MB-1);iRxMB++)	// last MB for tx
		{			
			//
#ifdef	TEST_EXTENDED_ID			 
 	    	mb_config.identifier = id++<<18;
#else
	    	mb_config.identifier = id++;
#endif 	    		
  	    	//printf("rx MB%d ID = %#08.8x\n",iRxMB,mb_config.identifier<<18);
 	    	mb_config.mailbox_number = iRxMB;
	    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,TRUE);
			if(state != FLEXCAN_OK)
			{
				printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iRxMB);
			}  				
			
		}
		if(iRxMB <= NUMBER_OF_MB-1)
		{
			// Initialize last MB as  tx MB
			iTxMB = iRxMB;
			mb_config.mailbox_number = iTxMB;
		    mb_config.direction = FLEXCAN_TX;
		    mb_config.code = FLEXCAN_MB_CODE_TX_ONCE; 
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
		    mb_config.remote_req_flag = TRUE;
#else
		    mb_config.remote_req_flag = 0;		    
#endif
#ifdef	TEST_EXTENDED_ID
	    mb_config.format = FLEXCAN_EXTENDED;
#else
	    mb_config.format = FLEXCAN_STANDARD;
#endif
		    
		    for(i = 0; i< iNoOfFilterTableElements+(NUMBER_OF_MB-NoMBsAvail-1)+2;i++)	// send additional 2 messages
			{	
#ifdef	TEST_EXTENDED_ID					    
				mb_config.identifier = (i+1)<<18;			
#else
				mb_config.identifier = (i+1);
#endif				
		    	//printf("Transmit message ID = %#08.8x (in filter table element format,shift left 1 bit)\n",mb_config.identifier<<1);
		    	state = FLEXCAN_Initialize_mailbox(&mb_config,bActivate_mb,FALSE);	// use poll method for tx MB
				if(state != FLEXCAN_OK)
				{
					printf("FLEXCAN_Initialize_mailbox: returned state = %#08x for mailbox %d\n",state,iTxMB);
				}  	
				// wait for the tx done
				while(!FLEXCAN_Is_MB_Done(FLEXCAN0,iTxMB)){}	
				
				// clear the flag
				FLEXCAN0_IFLAG1 = (1<<iTxMB);	
				
				// Check the rx FIFO flag
				if(FLEXCAN0_IFLAG1 & FLEXCAN_IFLAG1_BUF5I)
				{
					// Service the rx FIFO
					 // read FIFO
					 cs[0] = FLEXCAN0_MBn_CS(0);
					 id= FLEXCAN0_MBn_ID(0);
					 data0[0] = FLEXCAN0_MBn_WORD0(0);
					 data1[0] = FLEXCAN0_MBn_WORD1(0);	
					 
					 // read RXFIR
					 iMatchHit = FLEXCAN0_RXFIR & 0x1FF;
					 
					 // clear flag
					 FLEXCAN0_IFLAG1 = FLEXCAN_IFLAG1_BUF5I;
					 printf("RxFIFO: received a message ID=%#08.8x, match hit =%d\n",id,iMatchHit);			
				}
				// wait for both IMEUREQ and IMEUACK bits to be negated;
				while( (FLEXCAN0_IMEUR & FLEXCAN_IMEUR_IMEUREQ_MASK) || 
				(FLEXCAN0_IMEUR & FLEXCAN_IMEUR_IMEUACK_MASK) )
				{
				}				
				// configure IMEUP bits with the respective element address vpointer;
				iFIFOFilterTabElement2Update = 0;	// update FIFO filter table elements n+0 to n+3 
				FLEXCAN_Set_IMEUP(FLEXCAN0_IMEUR,iFIFOFilterTabElement2Update);	
					
				// request IMEU by asserting IMEUREQ bit;
				FLEXCAN0_IMEUR |= FLEXCAN_IMEUR_IMEUREQ_MASK;
				// check IMEUREQ and IMEUACK bits
				while( !(FLEXCAN0_IMEUR & FLEXCAN_IMEUR_IMEUREQ_MASK) || 
					!(FLEXCAN0_IMEUR & FLEXCAN_IMEUR_IMEUACK_MASK) )
				{
				}
				// wait for either IMEU Interrupt or IMEUF bit assertion. IMEUF is asserted when IMEUACK is
				// asserted;
				// a) update the Mailbox/RxFIFO matching elements;
				// b) negate IMEUREQ bit;
				// clear IMEUF flag	
				
			FLEXCAN0_IDFLT_TAB(iFIFOFilterTabElement2Update) = (iFIFOFilterTabElement2Update+4L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;
			FLEXCAN0_IDFLT_TAB(iFIFOFilterTabElement2Update+1) = (iFIFOFilterTabElement2Update+1+4L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;
			FLEXCAN0_IDFLT_TAB(iFIFOFilterTabElement2Update+2) = (iFIFOFilterTabElement2Update+2+4L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;						

			FLEXCAN0_IDFLT_TAB(iFIFOFilterTabElement2Update+3) = (iFIFOFilterTabElement2Update+3+4L)<<19				// 11msb of ID: 29 - 19
#ifdef TEST_REMOTE_REQUEST_FEATURE			 
									| BIT31					// RTR bit
#endif
#ifdef	TEST_EXTENDED_ID
									| BIT30 
#endif																						
						;						

					// b) negate IMEUREQ bit;	   
				 	FLEXCAN0_IMEUR &= ~(FLEXCAN_IMEUR_IMEUREQ_MASK); 
				 	    
					// it is recommended to clear ESR2[IMEUF] flag to keep the coherence with the corresponding
					// IMEUREQ and IMEUACK assertion;
				    FLEXCAN0_ESR2 |= FLEXCAN_ESR2_IMEUF;   												
			}
		}
		// wait till rx FIFO interrupt and MB interrupts are completed.
		while(guiMB_ISR_Count<(iNoOfFilterTableElements+(NUMBER_OF_MB-NoMBsAvail-2))){}		
	}
	PrintPassFailMessage(guiErrCount);				    			
}

void FlexCAN0_ECC_Test(void)
{
	
}


void flexcan0_tx_can1_rx_fifo_formatA_bcc_test(void)
{

	 /* INFO: Timer Sync test is applicable
	 		  for Rx.If enabled,the 10th Rx MB,i.e,
	 		  MB[9] will have a much reduced time stamp
	 		  compared to MB[8].This will prove that
	 		  free running timer has been reset.
	*/	 		  
	 
	 volatile uint32_t i;
	 
	 printf("FIFO Format A Test\n\n");
	 
	 #if SAMPLING_TEST
	 printf("Sampling Test Enabled \n");
	 #endif
	 
	 #if TIMER_SYNC
 	 printf("Timer Sync Test with FIFO Enabled\n");
 	 #endif
 
	 ref_frame_count = 7;
	 frames_intr_count = 0;
	 frames_done = 0;
 	
	 /* FlexCAN0 Settings */	 
	FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
        while((FLEXCAN_MCR_LPM_ACK & FLEXCAN0_MCR));	

	// Now can apply Soft Reset
	FLEXCAN0_MCR ^= FLEXCAN_MCR_SOFT_RST;
	while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 	
	 // Now it should be in Freeze mode  
	 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR));	 
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;	// disable receiving self-transmitted message

         /* 
         ** baud = 125kHz, 48M/32= 1.5M sclock, 12Tq
         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
                 ** RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 32
         */         
	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(31)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   
	 					   #if SAMPLING_TEST
	 					   FLEXCAN_CTRL_SMP	          |
	 					   #endif
	 					   
	 					  // FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );

	   #if SAMPLING_TEST
		 printf("FLEXCAN0_CTRL1 = %#08.8x\n", FLEXCAN0_CTRL1);
	   #endif
		 					   
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x08080000;
	  	FLEXCAN0_MBn_ID(i) = ((i+1)<<18);
	  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
	  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	  }

	 FLEXCAN0_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN0_RX15MASK = 0x1FFFFFFF;
	 
	 /* Enable FlexCAN 0*/
        FLEXCAN0_MCR ^= (FLEXCAN_MCR_HALT);
        // Now it should be not in Freeze mode  
	 while((FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR));	 
	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
         while((FLEXCAN_MCR_LPM_ACK & FLEXCAN1_MCR));	

	// Now can apply Soft Reset
	FLEXCAN1_MCR ^= FLEXCAN_MCR_SOFT_RST;
	while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);

         // Now it should be in Freeze mode  
	 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR));	 
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_FEN	  |
	 					  FLEXCAN_MCR_IDAM(0x0) //ID Table Format A
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(31)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   
	 					   #if SAMPLING_TEST
	 					   FLEXCAN_CTRL_SMP	          |
	 					   #endif
	 					   
	 					  // FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	 					   
	   #if SAMPLING_TEST
		 printf("FLEXCAN1_CTRL1 = %#08.8x\n", FLEXCAN1_CTRL1);
	   #endif
	 					   
	  /* Initialize ID table */
	  for(i=0;i<8;i++)
	  {
	  	FLEXCAN1_IDFLT_TAB(i) = ((i+1)<<19);
	  }
	  
	  for(i=8;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = ((i+1)<<18);
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	  
	  /* Enable Frames available interrupt */
	 FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF5M; 
	  
	 //FLEXCAN1_RXFGMASK = 0;	// receive all message in rx FIFO
         
         FLEXCAN1_RXFGMASK = 0x1FFFFFFE;	// match all tab elements in rx FIFO
	 FLEXCAN1_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
         while((FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR));	
  
  	  /* Trigger Individual Rx MBs of FlexCAN1 */
  	  for(i=8;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(0x4);
	  }
	  
	  /* Trigger Tx MBs of FlexCAN0 */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) |= FLEXCAN_MB_CS_CODE(0xC);
	  }

	 printf("wait for frames to be transferred...\n\r\n");
          
	 /* Poll for Completion */
	 while(!(frames_done)) 
	 {
	 	printf("FLEXCAN1_IFLAG1 = %#08.8X\n",FLEXCAN1_IFLAG1);	 	
	 }
	 
	 /* Disable Interrupt */
	 FLEXCAN1_IMASK1 = 0;
	 
	 /* Wait for other MBs to Complete */
	 while((FLEXCAN1_IFLAG1 & 0xFF00) != 0xFF00)
	 {
	 	printf("FLEXCAN1_IFLAG1 = %#08.8X\n",FLEXCAN1_IFLAG1);
	 }
	 
	 /* Read other Rx MBs of FlexCAN1 */
	 for(i=8;i<16;i++)
	 {
		cs[i] = FLEXCAN1_MBn_CS(i);
		id[i] = FLEXCAN1_MBn_ID(i);
		data0[i] = FLEXCAN1_MBn_WORD0(i);
		data1[i] = FLEXCAN1_MBn_WORD1(i);
	 }
	 
	 /* Print received frame */
	 for(i=0;i<16;i++)
	 {
		 if(data0[i] != (0x12345678+i))
		 {
			 printf("data received = %#08.8x\n", data0[i]);
			 printf("Data Expected = %#08.8x\n", (0x12345678+i));
			 error("Data Mismatch\n");
		 }
		 
		 if(data1[i] != (0x11112222+i))
		 {
			 printf("data received = %#08.8x\n", data1[i]);
			 printf("Data Expected = %#08.8x\n", (0x11112222+i));
			 error("Data Mismatch\n");
		 }
		 
		 printf("cs received = %#08.8x\n", cs[i]);
		 printf("id received = %#08.8x\n\n", id[i]);
	 }	 
}



void flexcan0_tx_can1_rx_fifo_formatB_bcc_test(void)
{

	 volatile uint32_t i,formB_id =1;
	 
 	 printf("FIFO Format B Test\n");
 	 
 	 ref_frame_count = 15;
	 frames_intr_count = 0;
	 frames_done = 0;

	 /* FlexCAN0 Settings */
#ifndef USE_EXTERNAL_CLOCK	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
#endif	 
        FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
        while((FLEXCAN_MCR_LPM_ACK & FLEXCAN0_MCR));	

	// Now can apply Soft Reset
	FLEXCAN0_MCR ^= FLEXCAN_MCR_SOFT_RST;
	while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 	
	 // Now it should be in Freeze mode  
	 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR));	 
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;	// disable receiving self-transmitted message

#ifndef USE_EXTERNAL_CLOCK     
#if 1
          /* 
         BAUD = 83.33K
         ** 48M/48= 1M sclock, 12Tq
         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
                 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 48
         */
	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(47)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   
	 					   #if SAMPLING_TEST
	 					   FLEXCAN_CTRL_SMP	          |
	 					   #endif
	 					   
	 					  // FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );   
#else
         /* 
         BAUD = 33.33K
         ** 48M/120= 400k sclock, 12Tq
         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
                 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 120
         */
	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(119)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   
	 					   #if SAMPLING_TEST
	 					   FLEXCAN_CTRL_SMP	          |
	 					   #endif
	 					   
	 					  // FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x3)
	 					   );            
#endif         
#else         
         /*  
         Baud = 125k:
                         ** 12M/8= 1.5M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 8
	*/         
	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(7)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(2)
	 					   );
#endif         
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x08080000;
	  	FLEXCAN0_MBn_ID(i) = ((i+1)<<18);
	  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
	  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	  }

	 FLEXCAN0_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN0_RX15MASK = 0x1FFFFFFF;
	 
	 /* Enable FlexCAN 0*/
         FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	
        // Now it should be not in Freeze mode  
	 while((FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR));	
         
	 /* FlexCAN1 Settings */
#ifndef USE_EXTERNAL_CLOCK 	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
#endif         

	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
         while((FLEXCAN_MCR_LPM_ACK & FLEXCAN1_MCR));	

	// Now can apply Soft Reset
	FLEXCAN1_MCR ^= FLEXCAN_MCR_SOFT_RST;
	while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);

         // Now it should be in Freeze mode  
	 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR));	 
	 
          // Initialize MB RAMs
          for(i=0;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
  
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_FEN	  |
	 					  FLEXCAN_MCR_IDAM(0x1) //ID Table Format B
						  );
						  
#ifndef USE_EXTERNAL_CLOCK      
#if 1
          /* 
         BAUD = 83.33K
         ** 48M/48= 1M sclock, 12Tq
         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
                 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 48
         */
	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(47)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   
	 					   #if SAMPLING_TEST
	 					   FLEXCAN_CTRL_SMP	          |
	 					   #endif
	 					   
	 					  // FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );   
#else
         /* 
         BAUD = 33.33K
         ** 48M/120= 400k sclock, 12Tq
         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
                 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 120
         */
	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(119)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   
	 					   #if SAMPLING_TEST
	 					   FLEXCAN_CTRL_SMP	          |
	 					   #endif
	 					   
	 					  // FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x3)
	 					   );            
#endif         
#else         
         /*  
         Baud = 125k:
                         ** 12M/8= 1.5M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 8
	*/         
	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(7)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(2)
	 					   );
#endif         
	 
	  /* Initialize ID and ID filter table for FlexCAN1 */
	  for(i=0;i<8;i++)
	  {
//	  	FLEXCAN1_IDFLT_TAB(i) = ((i+1)<<19) | ((i+2)<<3);
	  	FLEXCAN1_IDFLT_TAB(i) = ((formB_id)<<19) | ((formB_id+1)<<3);
	  	formB_id = formB_id + 2;
	  }
	  
	  
	  /* Enable Frames available interrupt */
	 FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF5M; 
	  
//         FLEXCAN1_RXFGMASK = 0xFFFFFFFF;	// match all tab elements in rx FIFO
          
	 FLEXCAN1_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
         while((FLEXCAN_MCR_FRZ_ACK & FLEXCAN1_MCR));	

	  for(i=8;i<16;i++)
	  {	
	  	FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(0x4);
	  }         
	  /* Trigger Tx MBs of FlexCAN0 */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) |= FLEXCAN_MB_CS_CODE(0xC);
	  }

	 /* Poll for Completion */
          while(!(frames_done)){
	 	printf("FLEXCAN1_IFLAG1 = %#08.8X,FLEXCAN0_ESR1 = %#08.8X\n",FLEXCAN1_IFLAG1,FLEXCAN0_ESR1);	 	
	 	printf("FLEXCAN1_ESR1 = %#08.8X,frames_intr_count=%#08.8x\n",FLEXCAN1_ESR1,frames_intr_count);	 	
          }            
	 
	 /* Disable Interrupt */
	 FLEXCAN1_IMASK1 = 0;
	 
	 /* Print received frame */
	 for(i=0;i<16;i++)
	 {
		 if(data0[i] != (0x12345678+i))
		 {
			 printf("data received = %#08.8x\n", data0[i]);
			 printf("Data Expected = %#08.8x\n", (0x12345678+i));
			 error("Data Mismatch\n");
		 }
		 
		 if(data1[i] != (0x11112222+i))
		 {
			 printf("data received = %#08.8x\n", data1[i]);
			 printf("Data Expected = %#08.8x\n", (0x11112222+i));
			 error("Data Mismatch\n");
		 }
		 printf("cs received = %#08.8x\n", cs[i]);
		 printf("id received = %#08.8x\n\n", id[i]);
	 }
	 
}

void flexcan0_tx_can1_rx_fifo_formatC_bcc_test(void)
{

	 volatile uint32_t i,formC_id =1;
	 
 	 printf("FIFO Format C Test\n");
 	 
 	 ref_frame_count = 15;
        // ref_frame_count = 5;
	 frames_intr_count = 0;
	 frames_done = 0;

	 /* FlexCAN0 Settings */
#ifndef USE_EXTERNAL_CLOCK   	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
#endif	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;

#ifndef USE_EXTERNAL_CLOCK      
         /* 
         BAUD = 83.33K
         ** 48M/48= 1M sclock, 12Tq
         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
                 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 48
         */
	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(47)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   
	 					   #if SAMPLING_TEST
	 					   FLEXCAN_CTRL_SMP	          |
	 					   #endif
	 					   
	 					  // FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );     
#else         
         /*  
         Baud = 125k:
                         ** 12M/8= 1.5M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 8
	*/         
	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(7)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(2)
	 					   );
#endif         
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x08080000;
	  	FLEXCAN0_MBn_ID(i) = 0 | ((i+1)<<21);
	  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
	  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	  }

	 FLEXCAN0_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN0_RX15MASK = 0x1FFFFFFF;
	 
	 /* Enable FlexCAN 0*/
     FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* FlexCAN1 Settings */
#ifndef USE_EXTERNAL_CLOCK   	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
#endif         
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_FEN	  |
	 					  FLEXCAN_MCR_IDAM(0x2) //ID Table Format C
						  );
						  
#ifndef USE_EXTERNAL_CLOCK      
         /* 
         BAUD = 83.33K
         ** 48M/48= 1M sclock, 12Tq
         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
                 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 48
         */
	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(47)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   
	 					   #if SAMPLING_TEST
	 					   FLEXCAN_CTRL_SMP	          |
	 					   #endif
	 					   
	 					  // FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );         
#else         
         /*  
         Baud = 125k:
                         ** 12M/8= 1.5M sclock, 12Tq
		         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
				 ** RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 8
	*/         
	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(7)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(2)
	 					   );
#endif         
	 
	  /* Initialize ID table for FlexCAN1*/
	  for(i=0;i<8;i++)
	  {
	  	FLEXCAN1_IDFLT_TAB(i) = (formC_id<<24) | ((formC_id+1)<<16) | ((formC_id+2)<<8) | (formC_id+3);
	  	formC_id = formC_id + 4;
	  }
	  
	  for(i=8;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	  
	  /* Enable Frames available interrupt */
	 FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF5M; 
	  
	 FLEXCAN1_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
 
	  /* Trigger Tx MBs of FlexCAN0 */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) |= FLEXCAN_MB_CS_CODE(0xC);
	  }

	 /* Poll for Completion */
          while(!(frames_done)){
	 	printf("FLEXCAN1_IFLAG1 = %#08.8X\n",FLEXCAN1_IFLAG1);	 	
          }
	 
	 /* Disable Interrupt */
	 FLEXCAN1_IMASK1 = 0;
	 
	 /* Print received frame */
	 for(i=0;i<16;i++)
	 {
		 if(data0[i] != (0x12345678+i))
		 {
			 printf("data received = %#08.8x\n", data0[i]);
			 printf("Data Expected = %#08.8x\n", (0x12345678+i));
			 error("Data Mismatch\n");
		 }
		 
		 if(data1[i] != (0x11112222+i))
		 {
			 printf("data received = %#08.8x\n", data1[i]);
			 printf("Data Expected = %#08.8x\n", (0x11112222+i));
			 error("Data Mismatch\n");
		 }
		 printf("cs received = %#08.8x\n", cs[i]);
		 printf("id received = %#08.8x\n\n", id[i]);
	 }
	 
}



void flexcan0_tx_can1_rx_fifo_warning_overflow_bcc_test(void)
{

	 volatile uint32_t i,no_tx_mb = 5;
	 
	 printf("FIFO Warning/Overflow Test\n");
	
	
	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x08080000;
	  	FLEXCAN0_MBn_ID(i) = 0 | ((i+1)<<18);
	  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
	  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	  }

	 FLEXCAN0_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN0_RX15MASK = 0x1FFFFFFF;
	 
	 /* Enable FlexCAN 0*/
     FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_FEN	  |
	 					  FLEXCAN_MCR_IDAM(0x0) //ID Table Format A
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	 
	  /* Initialize ID table for FlexCAN1 */
	  for(i=0;i<8;i++)
	  {
	  	FLEXCAN1_IDFLT_TAB(i) = ((i+1)<<19);
	  }
	  
	  for(i=8;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	  
	  /* Enable Frames available interrupt */
	 FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF6M;
	 
	 #if FIFO_OVERFLOW_TEST
		 FLEXCAN1_IMASK1 = 0;
		 FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF7M;
	 #endif
	 
	 FLEXCAN1_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
 
	  /* Trigger Tx MBs of FlexCAN0 */
	  for(i=0;i<no_tx_mb;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) |= FLEXCAN_MB_CS_CODE(0xC);
	  }

	 /* Poll for Completion */
	 #if FIFO_OVERFLOW_TEST
		 while(!(fifo_overflow));
	 #else
		 while(!(fifo_warning));
	 #endif
	 
	 /* Disable Interrupt */
	 FLEXCAN1_IMASK1 = 0;	 
}


void flexcan0_tx_can1_rx_priority_bcc_test(void)
{

	 volatile int32_t i, local_prio = 7;
	 uint32_t time_stamp[17],received_mb_no_sorted[17],k,
	 expected_mb_no_sorted[] = {7,15,6,14,5,13,4,12,3,11,2,10,1,9,0,8};
	 
	 printf("Priority Test\n\n");
	 	 
	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS	|
	 					  FLEXCAN_MCR_LPRIO_EN	  //Local priority enabled	
	 					  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	  /* Initialize Tx MBs */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x08080000;
	  	FLEXCAN0_MBn_ID(i) = (local_prio<<29) | ((i+1)<<18);
	  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
	  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	  	FLEXCAN0_MBn_CS(i) = 0x0C080000;
	  
	  	local_prio = local_prio -1;
	  	if(local_prio<0)
	  		local_prio = 7;
	  }
	  
	 FLEXCAN0_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN0_RX15MASK = 0x1FFFFFFF;
	 
	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_IDAM(0x0) //ID Table Format A
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(0x2)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x5)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x2)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_PROPSEG(0x1)
	 					   );
	 
	  /* Initialize Rx MBs */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0 | ((i+1)<<18);
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  	FLEXCAN1_MBn_CS(i) = 0x04000000;
	  }
	  
	 FLEXCAN1_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	 /* Poll for Completion */
	 while((FLEXCAN1_IFLAG1 & 0xFFFF) != 0xFFFF);
	 
	 /* Read Rx MBs of FlexCAN1 */
	 for(i=0;i<16;i++)
	 {
		cs[i] = FLEXCAN1_MBn_CS(i);
		time_stamp[i] = cs[i] & 0xFFFF;
		id[i] = FLEXCAN1_MBn_ID(i);
		data0[i] = FLEXCAN1_MBn_WORD0(i);
		data1[i] = FLEXCAN1_MBn_WORD1(i);
	 }
	 
	 SelectionSort(&time_stamp[0],16);
	 
	 for(i=0;i<16;i++)
	 {
	 	for(k=0;k<16;k++)
	 	{
	 		if(time_stamp[i] == (cs[k] & 0xFFFF))
	 		{
	 			received_mb_no_sorted[i] = k;	// find the corresponding MB in timestamp order
	 			break; 
	 		}
	 	}
	 }
	 
 	 for(i=0;i<16;i++)
 	 {
 	 	if(received_mb_no_sorted[i] != expected_mb_no_sorted[i])
 	 	{
		 	printf("Received MB No. =%#08.8x\n",received_mb_no_sorted[i]);
		 	printf("Expected MB No. =%#08.8x\n",expected_mb_no_sorted[i]);
		 	error("Priority Test Failed\n");
		 	
 	 	}
 	 }
	 
	 /* Expected reception of MB
	 
	    MB No    ID         Ordering according
	    					to Priority(To be
	    					matched with reception
	    					timestamp)
		--------------------------------------
		--------------------------------------	    					
	 	id[1] = E0040000 --->15
		id[2] = C0080000 --->13
		id[3] = A00C0000 --->11
		id[4] = 80100000 --->9
		id[5] = 60140000 --->7
		id[6] = 40180000 --->5
		id[7] = 201C0000 --->3
		id[8] =   200000 --->1
		id[9] = E0240000 --->16
		id[10] = C0280000 --->14
		id[11] = A02C0000 --->12
		id[12] = 80300000 --->10
		id[13] = 60340000 --->8
		id[14] = 40380000 --->6
		id[15] = 203C0000 --->4
		id[16] =   400000 --->2*/
		
	 /* Print received frame */
	 for(i=0;i<16;i++)
	 {
		 if(data0[i] != (0x12345678+i))
		 {
			 printf("data received = %#08.8x\n", data0[i]);
			 printf("Data Expected = %#08.8x\n", (0x12345678+i));
			 error("Data Mismatch\n");
		 }
		 
		 if(data1[i] != (0x11112222+i))
		 {
			 printf("data received = %#08.8x\n", data1[i]);
			 printf("Data Expected = %#08.8x\n", (0x11112222+i));
			 error("Data Mismatch\n");
		 }
		 printf("cs received = %#08.8x\n", cs[i]);
		 printf("id received = %#08.8x\n\n", id[i]);
	 }
	 
}

void flexcan0_tx_can1_rx_remote_frame_bcc_test(void)
{

	 volatile uint32_t i;
	 
	 printf("Remote Frame Test\n\n");
	 
	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	  /* Initialize MBs which will respond to Remote Frames */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	  	FLEXCAN0_MBn_ID(i) = 0 | ((i+1)<<18);
	  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
	  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	  	FLEXCAN0_MBn_CS(i) = 0x0A080000;
	  }
	  
	 FLEXCAN0_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN0_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN0_RX15MASK = 0x1FFFFFFF;
	 
	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_IDAM(0x0) //ID Table Format A
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	 
	  /* Initialize MBs which will transmit Remote Frames */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x0C100000;
	  	FLEXCAN1_MBn_ID(i) = ((i+1)<<18);
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	  
	 FLEXCAN1_RXMGMASK = 0x1FFFFFFF;
	 FLEXCAN1_RX14MASK = 0x1FFFFFFF;
	 FLEXCAN1_RX15MASK = 0x1FFFFFFF;
	 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	 /* Poll for Completion */
	 while((FLEXCAN1_IFLAG1 & 0xFFFF) != 0xFFFF);
	 
	 /* Read Rx MBs of FlexCAN1 */
	 for(i=0;i<16;i++)
	 {
		cs[i] = FLEXCAN1_MBn_CS(i);
		id[i] = FLEXCAN1_MBn_ID(i);
		data0[i] = FLEXCAN1_MBn_WORD0(i);
		data1[i] = FLEXCAN1_MBn_WORD1(i);
	 }
	 
	 /* Print received frame */
	 for(i=0;i<16;i++)
	 {
		 if(data0[i] != (0x12345678+i))
		 {
			 printf("data received = %8x\n", data0[i]);
			 printf("Data Expected = %8x\n", (0x12345678+i));
			 error("Data Mismatch\n");
		 }
		 
		 if(data1[i] != (0x11112222+i))
		 {
			 printf("data received = %8x\n", data1[i]);
			 printf("Data Expected = %8x\n", (0x11112222+i));
			 error("Data Mismatch\n");
		 }
		 printf("cs received = %8x\n", cs[i]);
		 printf("id received = %8x\n\n", id[i]);
	 }
	 
}

void flexcan0_tx_can1_rx_extended_frame_test(void)
{

	 volatile uint32_t i;
	 
	 printf("Extended Frame With Individual Rx masking Test\n\n");
	 
	 /* Enable CAN0 and CAN1 pins */
	// GPIO_PAR_CANI2C =0x5F;
	 
	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	  /* Initialize Tx MBs */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x08680000; //Have to set SRR and IDE bits
	    FLEXCAN0_MBn_ID(i) = 0 | ((i+1)<<18) | ((i+1)<<2);
	  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
	  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	  	FLEXCAN0_MBn_CS(i) = 0x0C680000; //Have to set SRR and IDE bits
	  	
	  	if(i==15)
	  	{
		  	FLEXCAN0_MBn_CS(i) = 0x08680000; //Have to set SRR and IDE bits
		  	FLEXCAN0_MBn_ID(i) = 0 | ((i+1)<<18) | ((i+1)<<4);
		  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
		  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	  	    FLEXCAN0_MBn_CS(i) = 0x0C680000; //Have to set SRR and IDE bits
	  	}
	  }
	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN0_RXIMRn(i) = 0x1FFFFFFF;
	 }

	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	 
	  /* Initialize Rx MBs */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00200000; //Have to set IDE bit
	  	FLEXCAN1_MBn_ID(i) = 0x3 | ((i+1)<<18) | ((i+1)<<2);
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  	FLEXCAN1_MBn_CS(i) = 0x04200000; //Have to set IDE bit
	  	
	  	if(i==15)
	  	{
		  	FLEXCAN1_MBn_CS(i) = 0x00200000; //Have to set IDE bit
		  	FLEXCAN1_MBn_ID(i) = 0xF | ((i+1)<<18) | ((i+1)<<4);
		  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
		  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  	    FLEXCAN1_MBn_CS(i) = 0x04200000; //Have to set IDE bit
	  	}
	  }
	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN1_RXIMRn(i) = 0x1FFFFFFC;
	 	
	 	if(i==15)
		 	FLEXCAN1_RXIMRn(i) = 0x1FFFFFF0;
	 }
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	 /* Poll for Completion */
	 while((FLEXCAN1_IFLAG1 & 0xFFFF) != 0xFFFF);
	 
	 /* Read Rx MBs of FlexCAN1 */
	 for(i=0;i<16;i++)
	 {
		cs[i] = FLEXCAN1_MBn_CS(i);
		id[i] = FLEXCAN1_MBn_ID(i);
		data0[i] = FLEXCAN1_MBn_WORD0(i);
		data1[i] = FLEXCAN1_MBn_WORD1(i);
	 }
	 
	 /* Print received frame */
	 for(i=0;i<16;i++)
	 {
		 if(data0[i] != (0x12345678+i))
		 {
			 printf("data received = %8x\n", data0[i]);
			 printf("Data Expected = %8x\n", (0x12345678+i));
			 error("Data Mismatch\n");
		 }
		 
		 if(data1[i] != (0x11112222+i))
		 {
			 printf("data received = %8x\n", data1[i]);
			 printf("Data Expected = %8x\n", (0x11112222+i));
			 error("Data Mismatch\n");
		 }
		 printf("cs received = %8x\n", cs[i]);
		 printf("id received = %8x\n\n", id[i]);
	 }
	 
}

void flexcan0_tx_can1_rx_extended_frame_receive_queue_test(void)
{

	 volatile uint32_t i;
	 
	 printf("Extended Frame with Receive Queue Test\n\n");
	 	 
	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	  /* Initialize Tx MBs */
	  for(i=0;i<16;i++)
	  {
	    FLEXCAN0_MBn_CS(i) = 0x08680000; //Have to set SRR and IDE bits
	  	FLEXCAN0_MBn_ID(i) = 0 | 0x15A5A5A5;
	  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
	  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	    FLEXCAN0_MBn_CS(i) = 0x0C680000; //Have to set SRR and IDE bits
	  }
	  
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN0_RXIMRn(i) = 0x1FFFFFFF;
	 }

	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(0x2)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x5)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x2)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x1)
	 					   );
	 
	  /* Initialize Rx MBs */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00200000; //Have to set IDE bit
	  	FLEXCAN1_MBn_ID(i) = 0 | 0x15A5A5A5;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  	FLEXCAN1_MBn_CS(i) = 0x04200000; //Have to set IDE bit
	  }
	  
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN1_RXIMRn(i) = 0x1FFFFFFF;
	 }
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	 /* Poll for Completion */
	 while((FLEXCAN1_IFLAG1 & 0xFFFF) != 0xFFFF);
	 
	 /* Read Rx MBs of FlexCAN1 */
	 for(i=0;i<16;i++) 
	 {
		cs[i] = FLEXCAN1_MBn_CS(i);
		id[i] = FLEXCAN1_MBn_ID(i);
		data0[i] = FLEXCAN1_MBn_WORD1(i);
		data1[i] = FLEXCAN1_MBn_WORD1(i);
	 }
	 
	 /* Print received frame */
	 for(i=0;i<16;i++)
	 {
		 if(data0[i] != (0x12345678+i))
		 {
			 printf("data received = %8x\n", data0[i]);
			 printf("Data Expected = %8x\n", (0x12345678+i));
			 error("Data Mismatch\n");
		 }
		 
		 if(data1[i] != (0x11112222+i))
		 {
			 printf("data received = %8x\n", data1[i]);
			 printf("Data Expected = %8x\n", (0x11112222+i));
			 error("Data Mismatch\n");
		 }
		 printf("cs received = %8x\n", cs[i]);
		 printf("id received = %8x\n\n", id[i]);
	 }
	 
}

void flexcan0_tx_can1_rx_fifo_extended_frame_test(void)
{

	 volatile uint32_t i;
	 
	 printf("Extended Frame With FIFO Test\n\n");
	 
	 ref_frame_count = 7;

	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x3)
	 					   );
	  /* Initialize Tx MBs */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0; //Have to set SRR and IDE bits
	  	FLEXCAN0_MBn_ID(i) = 0 | ((i+1)<<18) | ((i+1)<<2);
	  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
	  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	  	FLEXCAN0_MBn_CS(i) = 0x0C680000; //Have to set SRR and IDE bits
	  }
	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN0_RXIMRn(i) = 0x1FFFFFFF;
	 }
	 
	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS  |
	 					  FLEXCAN_MCR_IDAM(0x0)|   //Format A
	 					  FLEXCAN_MCR_FEN	   |   //FIFO Enable
	 					  FLEXCAN_MCR_IRMQ	       //Enable individual Rx masking and queue feature
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x3)
	 					   );
	 
	  /* Initialize ID table */
	  for(i=0;i<8;i++)
	  {
	  	FLEXCAN1_IDFLT_TAB(i) = (1<<30) | ((i+1)<<19) | ((i+1)<<3); //Setting EXT bit
	  }
	  
	  for(i=8;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0;
	  	FLEXCAN1_MBn_ID(i) = 0 | ((i+1)<<18) | ((i+1)<<2);
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  	FLEXCAN1_MBn_CS(i) = 0x04200000;
	  }
	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	if(i<=7)
	 		FLEXCAN1_RXIMRn(i) = 0x3FFFFFF8;
	 	else
	 		FLEXCAN1_RXIMRn(i) = 0x1FFFFFFC;
	 }
	 
	  /* Enable Frames available interrupt */
	 FLEXCAN1_IMASK1 |= FLEXCAN_IMASK1_BUF5M; 

	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
 	 
 	 /* Poll for Completion */
	 while(!(frames_done));
	 
	 /* Disable Interrupt */
	 FLEXCAN1_IMASK1 ^= FLEXCAN_IMASK1_BUF5M;

	 /* Poll for Completion */
	 while((FLEXCAN1_IFLAG1 & 0xFF00) != 0xFF00);
	 
	 /* Read Rx MBs of FlexCAN1 */
	 for(i=8;i<16;i++)
	 {
		cs[i] = FLEXCAN1_MBn_CS(i);
		id[i] = FLEXCAN1_MBn_ID(i);
		data0[i] = FLEXCAN1_MBn_WORD0(i);
		data1[i] = FLEXCAN1_MBn_WORD1(i);
	 }
	 
	 /* Print received frame */
	 for(i=0;i<16;i++)
	 {
		 if(data0[i] != (0x12345678+i))
		 {
			 printf("data received = %8x\n", data0[i]);
			 printf("Data Expected = %8x\n", (0x12345678+i));
			 error("Data Mismatch\n");
		 }
		 
		 if(data1[i] != (0x11112222+i))
		 {
			 printf("data received = %8x\n", data1[i]);
			 printf("Data Expected = %8x\n", (0x11112222+i));
			 error("Data Mismatch\n");
		 }
		 printf("cs received = %8x\n", cs[i]);
		 printf("id received = %8x\n\n", id[i]);
	 }
	 
}

void flexcan0_rx_can1_tx_extended_frame_test(void)
{

	 volatile uint32_t i;
	 
	 printf("Extended Frame With Individual Rx masking Test\n\n");
	 printf("FlexCAN0 --> RX, FlexCAN1 --> TX \n\n");
	 	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	  /* Initialize Tx MBs */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0; //Have to set SRR and IDE bits
	  	FLEXCAN1_MBn_ID(i) = 0 | ((i+1)<<18) | ((i+1)<<2);
	  	FLEXCAN1_MBn_WORD0(i) = 0x12345678+i;
	  	FLEXCAN1_MBn_WORD1(i) = 0x11112222+i;
	  	FLEXCAN1_MBn_CS(i) = 0x0C680000; //Have to set SRR and IDE bits
	  	
	  	if(i==15)
	  	{
		  	FLEXCAN1_MBn_CS(i) = 0x0C680000; //Have to set SRR and IDE bits
		  	FLEXCAN1_MBn_ID(i) = 0 | ((i+1)<<18) | ((i+1)<<4);
		  	FLEXCAN1_MBn_WORD0(i) = 0x12345678+i;
		  	FLEXCAN1_MBn_WORD1(i) = 0x11112222+i;
	  	}
	  }
	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN0_RXIMRn(i) = 0x1FFFFFFF;
	 }

	 
	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
						  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	 
	  /* Initialize Rx MBs */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0; //Have to set IDE bit
	  	FLEXCAN0_MBn_ID(i) = 0x3 | ((i+1)<<18) | ((i+1)<<2);
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  	FLEXCAN0_MBn_CS(i) = 0x04200000; //Have to set IDE bit
	  	
	  	if(i==15)
	  	{
		  	FLEXCAN0_MBn_CS(i) = 0; //Have to set IDE bit
		  	FLEXCAN0_MBn_ID(i) = 0xF | ((i+1)<<18) | ((i+1)<<4);
		  	FLEXCAN0_MBn_WORD0(i)  = 0x00000000;
		  	FLEXCAN0_MBn_WORD1(i)  = 0x00000000;
		  	FLEXCAN0_MBn_CS(i) = 0x04200000; //Have to set IDE bit
	  	}
	  }
	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN0_RXIMRn(i) = 0x1FFFFFFC;
	 	
	 	if(i==15)
		 	FLEXCAN0_RXIMRn(i) = 0x1FFFFFF0;
	 }
	 
	  /* Enable FlexCAN0 */
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* Enable FlexCAN 1*/
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	 /* Poll for Completion */
	 while((FLEXCAN0_IFLAG1 & 0xFFFF) != 0xFFFF);
	 
	 /* Read Rx MBs of FlexCAN0 */
	 for(i=0;i<16;i++)
	 {
		cs[i] = FLEXCAN0_MBn_CS(i);
		id[i] = FLEXCAN0_MBn_ID(i);
		data0[i] = FLEXCAN0_MBn_WORD0(i);
		data1[i] = FLEXCAN0_MBn_WORD1(i);
	 }
	 
	 /* Print received frame */
	 for(i=0;i<16;i++)
	 {
		 if(data0[i] != (0x12345678+i))
		 {
			 printf("data received = %8x\n", data0[i]);
			 printf("Data Expected = %8x\n", (0x12345678+i));
			 error("Data Mismatch\n");
		 }
		 
		 if(data1[i] != (0x11112222+i))
		 {
			 printf("data received = %8x\n", data1[i]);
			 printf("Data Expected = %8x\n", (0x11112222+i));
			 error("Data Mismatch\n");
		 }
		 printf("cs received = %8x\n", cs[i]);
		 printf("id received = %8x\n\n", id[i]);
	 }
	 
}

void flexcan0_tx_can1_rx_extended_frame_MB_interrupt_test(void)
{

	 volatile uint32_t i;
	 
	 printf("Extended Frame With Individual MB Interrupt Test\n\n");

	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x3)
	 					   );
	  /* Initialize Tx MBs */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x08680000; //Have to set SRR and IDE bits
	  	FLEXCAN0_MBn_ID(i) = 0 | ((i+1)<<18) | ((i+1)<<2);
	  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
	  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	  	
	  	if(i==15)
	  	{
		  	FLEXCAN0_MBn_CS(i) = 0x08680000; //Have to set SRR and IDE bits
		  	FLEXCAN0_MBn_ID(i) = 0 | ((i+1)<<18) | ((i+1)<<4);
		  	FLEXCAN0_MBn_WORD0(i) = 0x12345678+i;
		  	FLEXCAN0_MBn_WORD1(i) = 0x11112222+i;
	  	}
	  }
	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN0_RXIMRn(i) = 0x1FFFFFFF;
	 }

	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	 
	  /* Initialize Rx MBs */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00200000; //Have to set IDE bit
	  	FLEXCAN0_MBn_ID(i) = 0x3 | ((i+1)<<18) | ((i+1)<<2);
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  	
	  	if(i==15)
	  	{
		  	FLEXCAN0_MBn_CS(i) = 0x00200000; //Have to set IDE bit
		  	FLEXCAN0_MBn_ID(i) = 0xF | ((i+1)<<18) | ((i+1)<<4);
		  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
		  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  	}
	  }
	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN1_RXIMRn(i) = 0x1FFFFFFC;
	 	
	 	if(i==15)
		 	FLEXCAN1_RXIMRn(i) = 0x1FFFFFF0;
	 }
	 
	 /* Enable MB Interrupts */
	 FLEXCAN1_IMASK1 = 0xFFFF; 
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* Trigger Rx MBs of FlexCAN 1 */
	  for(i=0;i<16;i++)
	  {
	  	 FLEXCAN1_MBn_CS(i) |= FLEXCAN_MB_CS_CODE(0x4);
	  }
	  
	  /* Trigger Tx MBs of FlexCAN0 */
	  for(i=0;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) |= FLEXCAN_MB_CS_CODE(0xC);
	  	while(mb_intr != (1<<i));
	  	printf("Checked Interrupt for MB[%d] : %8x\n",i,mb_intr);
	  }

	 /* Read Rx MBs of FlexCAN1 */
	 for(i=0;i<16;i++)
	 {
		cs[i] = FLEXCAN1_MBn_CS(i);
		id[i] = FLEXCAN1_MBn_ID(i);
		data0[i] = FLEXCAN1_MBn_WORD0(i);
		data1[i] = FLEXCAN1_MBn_WORD1(i);
	 }
	 
	 /* Print received frame */
	 for(i=0;i<16;i++)
	 {
		 if(data0[i] != (0x12345678+i))
		 {
			 printf("data received = %8x\n", data0[i]);
			 printf("Data Expected = %8x\n", (0x12345678+i));
			 error("Data Mismatch\n");
		 }
		 
		 if(data1[i] != (0x11112222+i))
		 {
			 printf("data received = %8x\n", data1[i]);
			 printf("Data Expected = %8x\n", (0x11112222+i));
			 error("Data Mismatch\n");
		 }
		 printf("cs received = %8x\n", cs[i]);
		 printf("id received = %8x\n\n", id[i]);
	 }
	 
}

void flexcan0_tx_can1_rx_extended_frame_baud_rate_test(void)
{

	 volatile uint32_t i,
	 				 baud = 1,
	 				 loop_count,
	 				 data_tx = 0x11223344,
	 				 data_rx,
	 				 cs_rx,
	 				 id_rx;
	 
	 printf("Baud Rate Test\n\n");
	 
	 /* Clock Source Selection */
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
 	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
 	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable

	 for(loop_count=0;loop_count<8;loop_count++)
	 {
		 /* FlexCAN0 Settings */
		 
		 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
		 
		 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
		 
		 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
		 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
		 					  );

		 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(baud<<loop_count)   |
		 					   FLEXCAN_CTRL_RJW(0x1)	  				|
		 					   FLEXCAN_CTRL_PSEG1(0x5)	  				|
		 					   FLEXCAN_CTRL_PSEG2(0x2)	  				|
		 					   FLEXCAN_CTRL_BOFF_REC	  				|
		 					   FLEXCAN_CTRL_LBUF		  				|
		 					   FLEXCAN_CTRL_PROPSEG(0x1)
		 					   );
		  /* Initialize Tx MBs */

  		  	FLEXCAN0_MBn_CS(0) = 0; //Have to set SRR and IDE bits
		  	FLEXCAN0_MBn_ID(0) = 0 | ((loop_count+1)<<18) | ((loop_count+1)<<2);
		  	FLEXCAN0_MBn_WORD0(0) = data_tx;
		  	FLEXCAN0_MBn_WORD1(0) = 0x00000000;
  		  	FLEXCAN0_MBn_CS(0) = 0x0C640000; //Have to set SRR and IDE bits

		  for(i=1;i<16;i++)
		  {
		  	FLEXCAN0_MBn_CS(i) = 0x00000000;
		  	FLEXCAN0_MBn_ID(i) = 0x00000000;
		  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
		  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
		  	
		  }

		 /* Initialize RXIMR */ 
		 FLEXCAN0_RXIMRn(0) = 0x1FFFFFFF;
		 
		 /* FlexCAN1 Settings */
		 
		 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
		 
		 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
		 
		 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
		 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
							  );

		 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(baud<<loop_count)   |
		 					   FLEXCAN_CTRL_RJW(0x1)	  				|
		 					   FLEXCAN_CTRL_PSEG1(0x5)	  				|
		 					   FLEXCAN_CTRL_PSEG2(0x2)	  				|
		 					   FLEXCAN_CTRL_BOFF_REC	  				|
		 					   FLEXCAN_CTRL_LBUF		  				|
		 					   FLEXCAN_CTRL_PROPSEG(0x1)
		 					   );
		 
		  /* Initialize Rx MBs */
		  i = 0;
 		  	FLEXCAN1_MBn_CS(i) = 0x00000000; 
 		  	FLEXCAN1_MBn_ID(i) = 0x3 | ((loop_count+1)<<18) | ((loop_count+1)<<2);
		  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
		  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
 		  	FLEXCAN1_MBn_CS(i) = 0x04200000; //Have to set IDE bit

		  for(i=1;i<16;i++)
		  {
		  	FLEXCAN1_MBn_CS(i) = 0x00000000; 
		  	FLEXCAN1_MBn_ID(i) = 0x00000000;
		  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
		  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
		  }
		 
		 /* Initialize RXIMR */ 
		 FLEXCAN1_RXIMRn(i) = 0x1FFFFFFC;
		 
		  /* Enable FlexCAN1 */
		 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
		 
		 /* Enable FlexCAN 0*/
		 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

		 /* Poll for Completion */
		 while((FLEXCAN1_IFLAG1 & 0x1) != 0x1);
		 
		 cs_rx = FLEXCAN1_MBn_CS(0);
		 id_rx = FLEXCAN1_MBn_ID(0);
		 data_rx = FLEXCAN1_MBn_WORD0(0);
		 
		 if(data_rx == data_tx)
		 {
		 	printf("ID Received = %8x\n",id_rx);
		 	printf("Data Received = %8x\n",data_rx);
		 	printf("Passed for baud rate %d\n\n",baud<<loop_count);
		 }
		 else
		 {
		 	printf("ID Received = %8x\n",id_rx);
		 	printf("Data Received = %8x\n",data_rx);
		 	error("Failed for baud rate %d\n\n",baud<<loop_count);
		 }
		 
		 data_tx = data_tx+1;
		 data_rx = 0;
	 }


	 /* Max Baud Rate Test */

	 /* FlexCAN0 Settings */

	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(0x0)   |
	 					   FLEXCAN_CTRL_RJW(0x1)	   |
	 					   FLEXCAN_CTRL_PSEG1(0x5)	   |
	 					   FLEXCAN_CTRL_PSEG2(0x2)	   |
	 					   FLEXCAN_CTRL_BOFF_REC	   |
	 					   FLEXCAN_CTRL_LBUF		   |
	 					   FLEXCAN_CTRL_PROPSEG(0x1)
	 					   );
	  /* Initialize Tx MBs */
	 i = 0;
	 FLEXCAN0_MBn_CS(i) = 0; //Have to set SRR and IDE bits
	 FLEXCAN0_MBn_ID(i) = 0 | (1<<18) | (1<<2);
	 FLEXCAN0_MBn_WORD0(i) = data_tx;
	 FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	 FLEXCAN0_MBn_CS(i) = 0x0C640000; //Have to set SRR and IDE bits

	  for(i=1;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;	  	
	  }

	 /* Initialize RXIMR */ 
	 FLEXCAN0_RXIMRn(0) = 0x1FFFFFFF;
	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(0x0)   |
	 					   FLEXCAN_CTRL_RJW(0x1)	   |
	 					   FLEXCAN_CTRL_PSEG1(0x5)	   |
	 					   FLEXCAN_CTRL_PSEG2(0x2)	   |
	 					   FLEXCAN_CTRL_BOFF_REC	   |
	 					   FLEXCAN_CTRL_LBUF		   |
	 					   FLEXCAN_CTRL_PROPSEG(0x1)
	 					   );
	 
	  /* Initialize Rx MBs */
	  i = 0;
	  FLEXCAN1_MBn_CS(i) = 0; //Have to set IDE bit
	  FLEXCAN1_MBn_ID(i) = 0x3 | (1<<18) | (1<<2);
	  FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  FLEXCAN1_MBn_CS(i) = 0x04200000; //Have to set IDE bit

	  for(i=1;i<16;i++)
	  {
		  FLEXCAN1_MBn_CS(i) = 0; //Have to set IDE bit
		  FLEXCAN1_MBn_ID(i) = 0;
		  FLEXCAN1_MBn_WORD0(i) = 0x00000000;
		  FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	 
	 /* Initialize RXIMR */ 
	 FLEXCAN1_RXIMRn(0) = 0x1FFFFFFC;
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	 /* Poll for Completion */
	 while((FLEXCAN1_IFLAG1 & 0x1) != 0x1);
	 
	 cs_rx = FLEXCAN1_MBn_CS(0);
	 id_rx = FLEXCAN1_MBn_ID(0);
	 data_rx = FLEXCAN1_MBn_WORD0(0);
	 
	 if(data_rx == data_tx)
	 {
	 	printf("ID Received = %8x\n",id_rx);
	 	printf("Data Received = %8x\n",data_rx);
 		printf("Passed for Max baud rate\n\n");
	 }
	 else
	 {
	 	printf("ID Received = %8x\n",id_rx);
	 	printf("Data Received = %8x\n",data_rx);
	 	error("Failed for Max baud rate\n\n");
	 }
	 
	 data_tx = data_tx+1;


	 /* Min Baud rate test */
	 
	 /* FlexCAN0 Settings */

	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(0xFFL)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	   |
	 					   FLEXCAN_CTRL_PSEG1(0x5)	   |
	 					   FLEXCAN_CTRL_PSEG2(0x2)	   |
	 					   FLEXCAN_CTRL_BOFF_REC	   |
	 					   FLEXCAN_CTRL_LBUF		   |
	 					   FLEXCAN_CTRL_PROPSEG(0x1)
	 					   );
	  /* Initialize Tx MBs */
	 i = 0;	
	  	FLEXCAN0_MBn_CS(i) = 0; //Have to set SRR and IDE bits
	  	FLEXCAN0_MBn_ID(i) = 0 | (1<<18) | (1<<2);
	  	FLEXCAN0_MBn_WORD0(i) = data_tx;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  	FLEXCAN0_MBn_CS(i) = 0x0C640000; //Have to set SRR and IDE bits

	  for(i=1;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0; //Have to set SRR and IDE bits
	  	FLEXCAN0_MBn_ID(i) = 0 ;
	  	FLEXCAN0_MBn_WORD0(i) = 0;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;	  	
	  }

	 /* Initialize RXIMR */ 
	 FLEXCAN0_RXIMRn(0) = 0x1FFFFFFF;
	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(0xFFL)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	   |
	 					   FLEXCAN_CTRL_PSEG1(0x5)	   |
	 					   FLEXCAN_CTRL_PSEG2(0x2)	   |
	 					   FLEXCAN_CTRL_BOFF_REC	   |
	 					   FLEXCAN_CTRL_LBUF		   |
	 					   FLEXCAN_CTRL_PROPSEG(0x1)
	 					   );
	 
	  /* Initialize Rx MBs */
	  i = 0;
	  	FLEXCAN1_MBn_CS(i) = 0; //Have to set SRR and IDE bits
	  	FLEXCAN1_MBn_ID(i) = 0x3 | (1<<18) | (1<<2) ;
	  	FLEXCAN1_MBn_WORD0(i) = 0;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  	FLEXCAN1_MBn_CS(i) = 0x04200000; //Have to set SRR and IDE bits

	  for(i=1;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0; //Have to set SRR and IDE bits
	  	FLEXCAN1_MBn_ID(i) = 0 ;
	  	FLEXCAN1_MBn_WORD0(i) = 0;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	 
	 /* Initialize RXIMR */ 
	 FLEXCAN1_RXIMRn(0) = 0x1FFFFFFC;
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	 /* Poll for Completion */
	 while((FLEXCAN1_IFLAG1 & 0x1) != 0x1);
	 
	 cs_rx = FLEXCAN1_MBn_CS(0);
	 id_rx = FLEXCAN1_MBn_ID(0);
	 data_rx = FLEXCAN1_MBn_WORD0(0);
	 
	 if(data_rx == data_tx)
	 {
	 	printf("ID Received = %8x\n",id_rx);
	 	printf("Data Received = %8x\n",data_rx);
	 	printf("Passed for Min baud rate\n\n");
	 }
	 else
	 {
	 	printf("ID Received = %8x\n",id_rx);
	 	printf("Data Received = %8x\n",data_rx);
	 	error("Failed for Min baud rate\n\n");
	 }
	 
}

void flexcan0_module_disable_test(void)
{

 	 	uint32_t i,baud = 50;

		 guiErrCount = 0;
		 printf("Module Disable Test\n\n");
		 
		 /* FlexCAN0 Settings */
		 
	  	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock

		 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
		 while((FLEXCAN_MCR_LPM_ACK & FLEXCAN0_MCR));
		 
		 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
		 
		 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
		 
		 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
		 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
		 					  );
        
         printf("FLEXCAN0_MCR = %8x\n",FLEXCAN0_MCR);
		 					  
		 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(baud)   			|
		 					   FLEXCAN_CTRL_RJW(0x1)	  				|
		 					   FLEXCAN_CTRL_PSEG1(0x3)	  				|
		 					   FLEXCAN_CTRL_PSEG2(0x3)	  				|
		 					   FLEXCAN_CTRL_BOFF_REC	  				|
		 					   FLEXCAN_CTRL_LBUF		  				|
		 					   FLEXCAN_CTRL_PROPSEG(0x2)
		 					   );
		  /* Initialize Tx MBs */
  		  FLEXCAN0_MBn_CS(0) = 0; //Have to set SRR and IDE bits
		  FLEXCAN0_MBn_ID(0) = 0 | (1<<18) | (1<<2);
		  FLEXCAN0_MBn_WORD0(0) = 0x11223344;
		  FLEXCAN0_MBn_WORD1(0) = 0x00000000;
 		  FLEXCAN0_MBn_CS(0) = 0x0C640000; //Have to set SRR and IDE bits

		  /* Initialize Rx MBs */
		  for(i=1;i<16;i++)
		  {
		  	FLEXCAN0_MBn_ID(i) = 0x00000000;
		  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
		  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;		  	
			
			/* Initialize RXIMR */ 
		    FLEXCAN0_RXIMRn(i) = 0x1FFFFFFF;
		  	FLEXCAN0_MBn_CS(i) = 0x04000000;	// empty to receive
		  }
		 
		 /* Enable FlexCAN 0*/
		 FLEXCAN0_MCR ^= (FLEXCAN_MCR_HALT);
		 while((FLEXCAN_MCR_FRZ_ACK | FLEXCAN_MCR_NOT_RDY) & FLEXCAN0_MCR);

		 /* Disable FlexCAN0*/
		 FLEXCAN0_MCR |= FLEXCAN_MCR_MDIS;
		 
		 /* Wait for LPM_ACK bit to be set */
		 while(!(FLEXCAN_MCR_LPM_ACK & FLEXCAN0_MCR));
		 
         printf("FLEXCAN0_MCR = %8x\n\n",FLEXCAN0_MCR);		 
		 printf("FLEXCAN0_IFLAG1 =%8x\n",FLEXCAN0_IFLAG1);	
		 DelayBits(128);
		 if(FLEXCAN0_IFLAG1)
		 {
		 	guiErrCount++;
		 }	
		 PrintPassFailMessage(guiErrCount); 	 
}
	 

void flexcan0_tx_can1_rx_extended_frame_oscillator_clk_src_test(void)
{

	 volatile uint32_t i,
	 				 baud = 50,
	 				 data_tx = 0x11223344,
	 				 data_rx,
	 				 cs_rx,
	 				 id_rx;
	 
	 printf("Oscillator as Clock Source Test\n\n");
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable

	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(baud)   			|
	 					   FLEXCAN_CTRL_RJW(0x1)	  				|
	 					   FLEXCAN_CTRL_PSEG1(0x5)	  				|
	 					   FLEXCAN_CTRL_PSEG2(0x2)	  				|
	 					   FLEXCAN_CTRL_BOFF_REC	  				|
	 					   FLEXCAN_CTRL_LBUF		  				|
	 					   FLEXCAN_CTRL_PROPSEG(0x1)
	 					   );
	 					   
	 printf("FLEXCAN0_CTRL1 = %8x\n\n",FLEXCAN0_CTRL1);
	 	 					   
	  /* Initialize Tx MBs */

	  /* Initialize Tx MBs */
	  FLEXCAN0_MBn_CS(0) = 0; //Have to set SRR and IDE bits
	  FLEXCAN0_MBn_ID(0) = 0 | (1<<18) | (1<<2);
	  FLEXCAN0_MBn_WORD0(0) = 0x11223344;
	  FLEXCAN0_MBn_WORD1(0) = 0x00000000;
	  FLEXCAN0_MBn_CS(0) = 0x0C640000; //Have to set SRR and IDE bits

	  /* Initialize Rx MBs */
	  for(i=1;i<16;i++)
	  {
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;		  	
		
		/* Initialize RXIMR */ 
	    FLEXCAN0_RXIMRn(i) = 0x1FFFFFFF;
	  	FLEXCAN0_MBn_CS(i) = 0x04000000;	// empty to receive
	  }

	 /* Initialize RXIMR */ 
	 FLEXCAN0_RXIMRn(0) = 0x1FFFFFFF;
	 
	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(baud)   			|
	 					   FLEXCAN_CTRL_RJW(0x1)	  				|
	 					   FLEXCAN_CTRL_PSEG1(0x5)	  				|
	 					   FLEXCAN_CTRL_PSEG2(0x2)	  				|
	 					   FLEXCAN_CTRL_BOFF_REC	  				|
	 					   FLEXCAN_CTRL_LBUF		  				|
	 					   FLEXCAN_CTRL_PROPSEG(0x1)
	 					   );
	 					   
	 printf("FLEXCAN1_CTRL1 = %8x\n\n",FLEXCAN1_CTRL1);
	 
	  /* Initialize Rx MBs */
	  i = 0;
	  	FLEXCAN1_MBn_CS(i) = 0; //Have to set SRR and IDE bits
	  	FLEXCAN1_MBn_ID(i) = 0x3 | (1<<18) | (1<<2) ;
	  	FLEXCAN1_MBn_WORD0(i) = 0;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  	FLEXCAN1_MBn_CS(i) = 0x04200000; //Have to set SRR and IDE bits

	  for(i=1;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0; //Have to set SRR and IDE bits
	  	FLEXCAN1_MBn_ID(i) = 0 ;
	  	FLEXCAN1_MBn_WORD0(i) = 0;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }
	 
	 /* Initialize RXIMR */ 
	 FLEXCAN1_RXIMRn(0) = 0x1FFFFFFC;
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	 /* Poll for Completion */
	 while((FLEXCAN1_IFLAG1 & 0x1) != 0x1);
	 
	 cs_rx = FLEXCAN1_MBn_CS(0);
	 id_rx = FLEXCAN1_MBn_ID(0);
	 data_rx = FLEXCAN1_MBn_WORD0(0);
	 
	 if(data_rx == data_tx)
	 {
	 	printf("ID Received = %8x\n",id_rx);
	 	printf("Data Received = %8x\n",data_rx);
	 	printf("Passed with oscillator as clock source\n\n");
	 }
	 else
	 {
	 	printf("ID Received = %8x\n",id_rx);
	 	printf("Data Received = %8x\n",data_rx);
	 	error("Failed with oscillator as clock source\n\n");
	 }
		 
	 
}

void FlexCAN0_DozeStopMode_test(void)
{

	 volatile uint32_t i,
	 				 baud = 83,
	 				 data_tx = 0x11223344,
	 				 data_rx,
	 				 cs_rx,
	 				 id_rx;
	 
	 #if STOP_MODE
		 printf("Stop Mode Test\n\n");
	 #else
		 printf("Doze Mode Test\n\n");
	 #endif
#ifndef  USE_EXTERNAL_CLOCK	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
#endif         
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable

	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 while(!(FLEXCAN_MCR_FRZ_ACK & FLEXCAN0_MCR));
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 
	 					  #if (!STOP_MODE)
	 					  FLEXCAN_MCR_DOZE		| //Enable Doze Mode
	 					  #endif
	 					  
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );
	 
	 // Enable wake up and wakeup interrupt
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SLF_WAK | FLEXCAN_MCR_WAK_MSK
				  |  FLEXCAN_MCR_WAK_SRC	 		// enable wakeup filter
	 			 ;

#ifndef  USE_EXTERNAL_CLOCK	
          /* 
         BAUD = 83.33K
         ** 48M/48= 1M sclock, 12Tq
         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
                 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 48
         */
	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(47)  |
	 					   FLEXCAN_CTRL_RJW(0x2)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   
	 					   #if SAMPLING_TEST
	 					   FLEXCAN_CTRL_SMP	          |
	 					   #endif
	 					   
	 					  // FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );  
#else   
#if 0
         /*
         BAUD = 83.33K
         ** 12M/12= 1M sclock, 12Tq
         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
                 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 12
         */
         FLEXCAN0_CTRL1 = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(2)
                                                            | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
                                                            | FLEXCAN_CANCTRL_PRESDIV(11));
#else
         /* Use 4M external crystal
         BAUD = 83.33K
         ** 4M/4= 1M sclock, 12Tq
         ** PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
                 ** RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 12
         */
         FLEXCAN0_CTRL1 = (0 | FLEXCAN_CANCTRL_PROPSEG(2) | FLEXCAN_CANCTRL_RJW(2)
                                                            | FLEXCAN_CANCTRL_PSEG1(3) | FLEXCAN_CANCTRL_PSEG2(3)
                                                            | FLEXCAN_CANCTRL_PRESDIV(3));
#endif         
#endif                         
        
         // Disable all MB interrupts
         FLEXCAN0_IMASK1 = 0;
         
         // Do not mask any messsage 
         FLEXCAN0_RX14MASK = 0;
         FLEXCAN0_RX15MASK = 0;
         FLEXCAN0_RXMGMASK = 0;
         // Initialize individual mask register to receive any message
         if(FLEXCAN_MCR_IRMQ & FLEXCAN0_MCR)
         {
            for(i=0;i<NUMBER_OF_MB;i++)
            {
                   /* Initialize RXIMR */ 
                   FLEXCAN0_RXIMRn(i) = 0;  // don't care
            }  
         }
         
	  /* Initialize Tx MBs */
	  FLEXCAN0_MBn_CS(0) = 0x08000000; //Inactivate
	  FLEXCAN0_MBn_ID(0) = 0 | (1<<18) | (1<<2);
	  FLEXCAN0_MBn_WORD0(0) = data_tx;
	  FLEXCAN0_MBn_WORD1(1) = 0x00000000;
	  FLEXCAN0_MBn_CS(0) = 0x0C640000; //Have to set SRR and IDE bits

	  for(i=1;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
        	FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);	  	
	  }

	 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_HALT);
#if STOP_MODE
	ConfigureStopMode();
#else
	ConfigureDozeMode();
#endif	

	//ConfigureLPWakeupSource();
	printf("Begin to enter doze/stop mode...\n");
	
	//
	//StartLPWakeupSource();
	
 	//Enter Doze/Stop mode
	EnterStopMode();
	printf("Exit from doze/stop mode, FLEXCAN0_MCR = %#08.8x\n",FLEXCAN0_MCR);	
}





void flexcan0_tx_warning_ack_error_intr_test(void)
{

	 volatile uint32_t i,
	 				 baud = 50,
	 				 data_tx = 0x11223344,
	 				 data_rx,
	 				 cs_rx,
	 				 id_rx;
	 
	 printf("TX Warning and Ack. Error Interrupt Test\n\n");
	
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Bus Clock as Source
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable

	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 					  FLEXCAN_MCR_WRN_EN	| //Enable Warning Interrupt Flag Generation 
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(baud)   |
	 					   FLEXCAN_CTRL_RJW(0x1)	  	|
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  	|
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  	|
	 					   FLEXCAN_CTRL_BOFF_REC	  	|
	 					   FLEXCAN_CTRL_LBUF		  	|
	 					   FLEXCAN_CTRL_TWRN_MSK		| //Enable Tx Warning Interrupt
	 					   FLEXCAN_CTRL_ERR_MSK			| //Enable Error Interrupt
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	 					   
	 /* Preset the Tx Error Count to 95 */
	 FLEXCAN0_ECR = FLEXCAN_ECR_TX_ERR_COUNTER(95);
	 
	 while((FLEXCAN0_ECR & 0xFF) != 95);

  	/* Initialize Tx MBs */	 
	  FLEXCAN0_MBn_CS(0) = 0x08000000; //Inactivate
	  FLEXCAN0_MBn_ID(0) = 0 | (1<<18) | (1<<2);
	  FLEXCAN0_MBn_WORD0(0) = data_tx;
	  FLEXCAN0_MBn_WORD1(1) = 0x00000000;
	  FLEXCAN0_MBn_CS(0) = 0x0C640000; //Have to set SRR and IDE bits

	  for(i=1;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  	
	  }	 					   
	

	 /* Initialize RXIMR */ 
	 FLEXCAN0_RXIMRn(0) = 0x1FFFFFFF;
	 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	 while(!tx_warning_and_error_intr_occured);
	 
	 printf("FLEXCAN0_ESR1 = %8x\n",FLEXCAN0_ESR1);
}



void flexcan0_tx_can1_tx_bit1_error_intr_test(void)
{

	 volatile uint32_t i;
	 
	 printf("Bit 1 Error Interrupt Test\n\n");

	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(0x2)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x5)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x2)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x1)
	 					   );
	  /* Initialize Tx MBs */
 	  FLEXCAN0_MBn_CS(0) = 0x08000000; //Inactivate
	  FLEXCAN0_MBn_ID(0) = 0 | (1<<18) | (1<<2);
	  FLEXCAN0_MBn_WORD0(0) = 0x12345678;
	  FLEXCAN0_MBn_WORD1(1) = 0x11112222;
	  FLEXCAN0_MBn_CS(0) = 0x0C680000; //Have to set SRR and IDE bits

	  for(i=1;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }
	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN0_RXIMRn(i) = 0x1FFFFFFF;
	 }


	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(0x2)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x5)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x2)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
   	 					   FLEXCAN_CTRL_ERR_MSK		  | //Enable Error Interrupt
	 					   FLEXCAN_CTRL_PROPSEG(0x1)
	 					   );
	 
	  /* Initialize Rx MBs */
 	  FLEXCAN1_MBn_CS(0) = 0x08000000; //Inactivate
	  FLEXCAN1_MBn_ID(0) = 0 | (1<<18) | (1<<2);
	  FLEXCAN1_MBn_WORD0(0) = 0x5555AAAA;
	  FLEXCAN1_MBn_WORD1(1) = 0;
	  FLEXCAN1_MBn_CS(0) = 0x0C640000; //Have to set SRR and IDE bits

	  for(i=1;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }	  
 
	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN1_RXIMRn(i) = 0x1FFFFFFC;
	 }
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	 while(!bit1_error_occured);
	 
	 printf("FLEXCAN0_ESR1 = %8x\n",FLEXCAN0_ESR1);
	 printf("FLEXCAN1_ESR1 = %8x\n",FLEXCAN1_ESR1);
	 
}

void flexcan0_tx_can1_lom_bit0_error_intr_test(void)
{
	 volatile uint32_t i;
	 
	 printf("Bit 0 Error Interrupt Test\n\n");

	 
	 /* FlexCAN0 Settings */
	 
	 FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN0_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN0_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN0_MCR);
	 
	 FLEXCAN0_MCR |= (FLEXCAN_MCR_SRX_DIS 	|
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
	 					  );

	 FLEXCAN0_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(50)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x3)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x3)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_PROPSEG(0x2)
	 					   );
	  /* Initialize Tx MBs */
	  FLEXCAN0_MBn_CS(0) = 0x08000000; //Inactivate
	  FLEXCAN0_MBn_ID(0) = 0 | (1<<18) | (1<<2);
	  FLEXCAN0_MBn_WORD0(0) = 0x12345678;
	  FLEXCAN0_MBn_WORD1(1) = 0x11112222;
	  FLEXCAN0_MBn_CS(0) = 0x0C680000; //Have to set SRR and IDE bits

	  for(i=1;i<16;i++)
	  {
	  	FLEXCAN0_MBn_CS(i) = 0x00000000;
	  	FLEXCAN0_MBn_ID(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN0_MBn_WORD1(i) = 0x00000000;
	  }  
 
	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN0_RXIMRn(i) = 0x1FFFFFFF;
	 }


	 /* FlexCAN1 Settings */
	 
	 FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_CLK_SRC; //Source --> bus clock
	 
	 FLEXCAN1_MCR ^= FLEXCAN_MCR_MDIS; //Module Enable
	 
	 FLEXCAN1_MCR |= FLEXCAN_MCR_SOFT_RST; //Apply Soft Reset
	 
	 while(FLEXCAN_MCR_SOFT_RST & FLEXCAN1_MCR);
	 
	 FLEXCAN1_MCR |= (FLEXCAN_MCR_SRX_DIS |
	 					  FLEXCAN_MCR_IRMQ		  //Enable individual Rx masking and queue feature
						  );

	 FLEXCAN1_CTRL1 |= (FLEXCAN_CTRL_PRESDIV(0x2)  |
	 					   FLEXCAN_CTRL_RJW(0x1)	  |
	 					   FLEXCAN_CTRL_PSEG1(0x5)	  |
	 					   FLEXCAN_CTRL_PSEG2(0x2)	  |
	 					   FLEXCAN_CTRL_BOFF_REC	  |
	 					   FLEXCAN_CTRL_LBUF		  |
	 					   FLEXCAN_CTRL_LOM			  | //Listen only mode Activated
   	 					   FLEXCAN_CTRL_ERR_MSK		  | //Enable Error Interrupt
	 					   FLEXCAN_CTRL_PROPSEG(0x1)
	 					   );
	 
	  /* Initialize Rx MBs */
	  FLEXCAN1_MBn_CS(0) = 0; //Inactivate
	  FLEXCAN1_MBn_ID(0) = 0 | (1<<18) | (1<<2);
	  FLEXCAN1_MBn_WORD0(0) = 0x5555AAAA;
	  FLEXCAN1_MBn_WORD1(1) = 0;
	  FLEXCAN1_MBn_CS(0) = 0x04200000; //Have to set SRR and IDE bits

	  for(i=1;i<16;i++)
	  {
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	  }	  

	 
	 /* Initialize RXIMR */ 
	 for(i=0;i<16;i++)
	 {
	 	FLEXCAN1_RXIMRn(i) = 0x1FFFFFFC;
	 }
	 
	  /* Enable FlexCAN1 */
	 FLEXCAN1_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);
	 
	 /* Enable FlexCAN 0*/
	 FLEXCAN0_MCR ^= (FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT);

	 while(!bit0_error_occured);
	 
	 printf("FLEXCAN0_ESR = %8x\n",FLEXCAN0_ESR1);
	 printf("FLEXCAN1_ESR1 = %8x\n",FLEXCAN1_ESR1);
	 
}


void ConfigureLPWakeupSource(void)
{
#ifdef  STOP_MODE
  ConfigureLPT();
#else 
  #ifdef	USE_PIT0_WAKEUP
    ConfigurePIT();
  #endif  
#endif  
}

void ConfigureLPT(void)
{
// Enable LPT interrupt via NVIC  
  enable_irq(85);
  
  // Configure MCG&OSC to FBE mode (Need updated)
  // Set ERCLKEN=1, EREFSTEN=1 
  OSC_CR |=  0xA0;

  // select Sleep deep mode
  SCB_SCR= 0x4;

  // LPTMR configuration 
  LPTMR0_PSR = 0x0001;  // Confige PCS = 0x01 (1KHZ LPO)
  LPTMR0_CMR = 1000; //0xFFFF;  // Confige bigger value in post-Si validation for STOP Mode check
  LPTMR0_CSR = 0x40;  	//  enable interrupt  
}

void StartLPWakeupSource(void)
{
#ifdef  STOP_MODE
        StartLPT();
#else  
#ifdef	USE_PIT0_WAKEUP
	StartPIT();
#endif	
#endif        
}

void ConfigureStopMode(void)
{
  SCB_SCR |= BIT2;	//set SLEEPDEEP bit
  
}

void ConfigureDozeMode(void)
{
  SCB_SCR &= ~BIT2;	// clear SLEEPDEEP bit  
}

#if (PALLADIUM==1)
__asm void EnterDozeMode(void)
{
  		ENTER_LP_MODE_INSTRUCTION	// enter low power mode instruction
}
__asm void EnterStopMode(void)
{
  		ENTER_LP_MODE_INSTRUCTION	// enter low power mode instruction  	
}
#else
void EnterDozeMode(void)
{
  asm("wfi \n");
}
void EnterStopMode(void)
{
    asm("wfi \n");
}
#endif




void ConfigurePIT(void)
{
        // Enable PIT interrupts in NVIC
        /*
         PIT interrupt vector number 45,46,...,52
         IRQ value: Vector#-16 = 29,30,...,36
         IRQ VEC reg: 29/32 = 0,30/32=0,31/32=0,32/32=1,..
         bit to set : 29%32 = 29,30,31, 0,1,2,3,4
         */
        NVIC_enable_PIT_interrupts();
        
	// Enable PIT clock input 
        PIT_PITMCR= 0x00;
	PIT_CH0_LDVAL = 0x4000;
	PIT_CH0_TCTRL =  0
#ifdef	USE_WFI	
			| PIT_TCTRL_TIE			// if using WFI instruction, then enable interrupt;
									// if using WFE instruction, interrupt flag will resume CPU
#endif			
			; 
	printf("PIT_CH0_TCTRL = %#08.8x\n",	PIT_CH0_TCTRL);		
}

void StartPIT(void)
{
	PIT_CH0_TCTRL |= PIT_TCTRL_TEN;
	//printf("PIT_CH0_TCTRL = %#08.8x\n",	PIT_CH0_TCTRL);			
}

void StartLPT(void)
{
	LPTMR0_CSR |= 0x1;	
}

void PrintPassFailMessage(uint16_t uiErrCount)
{
  	if(uiErrCount)
	{
		TESTCASE_FAIL;
	}
	else
	{
		TESTCASE_PASS;
	}
}



uint16_t FlexCAN_PrepareFrameBitsWithCRC(uint32_t MsgID, uint8_t IDType,uint8_t RTR,
   PTMSG_Data Data,
   uint8_t MsgStatus )
{
	type_ID IDENT;
    unsigned char i,ByteAccess,DataLength,DataByte,BYTE;
    unsigned char *DataPointer;
    uint16_t crc;
    uint8_t data_field[8];

	
	 /*prepare message*/
	 TxBitCount = 0;
	 CeckSum = 0;
	 
	 /* Initialize the data frame buffer */
	 for(i = 0; i < sizeof(TransmitDataFrame); i++)
	 {
	 	TransmitDataFrame[i]=0;
	 }
	 
	 /* Start of Frame */
	 TxBitCount++;														
	 ByteAccess = (TxBitCount-1)/8;
	 TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 -> dominant SOF*/
	 CalcCheckSum(TransmitDataFrame[ByteAccess]);
	 CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	
	 /*Arbitration Field*/
	 if(IDType == FLEXCAN_EXTENDED)
	 {
	 	 IDENT.EXTENDED = (MsgID << 3);		// 29 bits ID
		 	 	 
	 }
	 else
	 {
	     IDENT.EXTENDED = (MsgID << 21); 	// 11 bits ID
	 }
	 for(i=0; i < 11; i++)										 /* ID */
	 {
		  TxBitCount++;
		  ByteAccess = (TxBitCount-1)/8;
		  TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
		   
		  if(IDENT.BYTE.B3 & MSB)		/*if MSB is '1' */
		  {
		    TransmitDataFrame[ByteAccess] |= 0x01;
		  }
		  CalcCheckSum(TransmitDataFrame[ByteAccess]);
		  CheckBitStuffing(TransmitDataFrame[ByteAccess]);
		  IDENT.EXTENDED <<= 1;										 /* Left shift by 1 for checking the next bit */
	 }
	 if(IDType == FLEXCAN_EXTENDED)
	 {
	 	// Generate SRR and IDE bits
	 	TxBitCount++;
	    ByteAccess = (TxBitCount-1)/8;
		TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
        TransmitDataFrame[ByteAccess] |= 0x01;	 /* SRR bit */
 		CalcCheckSum(TransmitDataFrame[ByteAccess]);
		CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	 	TxBitCount++;
	    ByteAccess = (TxBitCount-1)/8;
		TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
        TransmitDataFrame[ByteAccess] |= 0x01;	 /* IDE bit */
 		CalcCheckSum(TransmitDataFrame[ByteAccess]);
		CheckBitStuffing(TransmitDataFrame[ByteAccess]);  
		
		// Generate low 18-bit ID    
		 for(i=0; i < 18; i++)										 /* ID */
		 {
			  TxBitCount++;
			  ByteAccess = (TxBitCount-1)/8;
			  TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
			   
			  if(IDENT.BYTE.B3 & MSB)		/*if MSB is '1' */
			  {
			    TransmitDataFrame[ByteAccess] |= 0x01;
			  }
			  CalcCheckSum(TransmitDataFrame[ByteAccess]);
			  CheckBitStuffing(TransmitDataFrame[ByteAccess]);
			  IDENT.EXTENDED <<= 1;										 /* Left shift by 1 for checking the next bit */
		 }	
	 }
	 
	// Generate RTR bit
 	TxBitCount++;
    ByteAccess = (TxBitCount-1)/8;
	TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
    TransmitDataFrame[ByteAccess] |= (RTR? 0x01: 0);	 /* RTR bit */
	CalcCheckSum(TransmitDataFrame[ByteAccess]);
	CheckBitStuffing(TransmitDataFrame[ByteAccess]);  			  		 	
	
	
	 /*Control Field*/
	 /* r1/IDE & r0 both a dominant for standard ID.
	  *  
	  */
	 if(MsgStatus == CORRECT_FRAME)
	 {
	    for(i=0; i < 2; i++)										
	    {
	       TxBitCount++;														
	       ByteAccess = (TxBitCount-1)/8;
	       TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 -> dominant r1/r0*/
	       CalcCheckSum(TransmitDataFrame[ByteAccess]);
	       CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	    }
	  }
	  else
	  {
	       /* r1 correct */
	       TxBitCount++;														
	       ByteAccess = (TxBitCount-1)/8;
	       TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 -> dominant r1*/
	       CalcCheckSum(TransmitDataFrame[ByteAccess]);
	       CheckBitStuffing(TransmitDataFrame[ByteAccess]);
		   /* r0 fault */
	       TxBitCount++;														
	       ByteAccess = (TxBitCount-1)/8;
	       TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 -> dominant r0*/
	       CalcCheckSum(TransmitDataFrame[ByteAccess]);
	       CheckBitStuffing(TransmitDataFrame[ByteAccess]);
		   TransmitDataFrame[ByteAccess] |= 0x01;   /* Fault state of r1 */
	  }
	
	
	  /*DLC*/
	  DataLength = Data->LENGTH << 4;            /* Left shift by 4 for access to the MSB*/
	  for(i=0; i < 4; i++)										
	    {
	     TxBitCount++;														
	     ByteAccess = (TxBitCount-1)/8;
	     TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
	
	     if(DataLength & MSB)		/*if MSB is '1' */
	       {
	       TransmitDataFrame[ByteAccess] |= 0x01;
	       }
	     CalcCheckSum(TransmitDataFrame[ByteAccess]);
	     CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	     DataLength <<= 1;												 /* Left shift by 1 for checking the next bit */
	    }
	
	 /*Data Field*/
	  DataPointer = &Data->BYTE0;									 /* point to the first data byte */
	  /* Save to local data field buffer. */
	  for(DataByte =0; DataByte < Data->LENGTH; DataByte++)
	  {
	  		data_field[DataByte]= DataPointer[DataByte];
	  }  
	  swap_4bytes(data_field);	// swap bytes to big endian mode
	  swap_4bytes(data_field+4);// swap bytes to big endian mode
	  
	  for(DataByte=0; DataByte < Data->LENGTH; DataByte++)										
	  {
	    BYTE = data_field[DataByte];
	   // printf("data =%#02.2x\n",BYTE);
	    for(i=0; i < 8; i++)										
	    {
	       TxBitCount++;														
	       ByteAccess = (TxBitCount-1)/8;
	       TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
	
	       if(BYTE & MSB)		/*if MSB is '1' */
	       {
	         TransmitDataFrame[ByteAccess] |= 0x01;
	       }
	       CalcCheckSum(TransmitDataFrame[ByteAccess]);
	       CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	       BYTE <<= 1;														 /* Left shift by 1 for checking the next bit */
		}
		DataPointer++; /* point to the next byte */
	 }
	 /* Save CRC */
	 crc = CeckSum;
	 
	 /*CRC Field */
	 /* CRC sequence */
	  CeckSum <<= 1;            /* Left shift by 1 for access to the MSB*/
	  for(i=0; i < 15; i++)										
	  {
	     TxBitCount++;														
	     ByteAccess = (TxBitCount-1)/8;
	     TransmitDataFrame[ByteAccess] <<= 1;    
	
	     if(CeckSum & MSB_16bit)		/*if MSB is '1' */
	       {
	       TransmitDataFrame[ByteAccess] |= 0x01;
	       }
	     CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	     CeckSum <<= 1;												 /* Left shift by 1 for checking the next bit */
	  }
	  /* CRC delemiter*/
	  TxBitCount++;														
	  ByteAccess = (TxBitCount-1)/8;
	  TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
	  TransmitDataFrame[ByteAccess] |= 0x01;	 /* CRC delemiter is always recessive*/
          //NOTE: The delimeters, ACK field and EOFs are not stuffed.
	
	/*Acknowledge Field */
	  for(i=0; i < 2; i++)										
	  {
	     TxBitCount++;														
	     ByteAccess = (TxBitCount-1)/8;
	     TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 -> recessive Ack Slot and Del*/
	     TransmitDataFrame[ByteAccess] |= 0x01;	  /* Ack field is always Tx as recessive by the transmitter */
	   }
	 i= TxBitCount;
	 while((i%8) !=0)
	 {
	  TransmitDataFrame[ByteAccess] <<= 1;
	  i++;
	 }
	return (crc);
}


uint16_t BuildCANFrame(uint32_t MsgID, uint8_t IDType,uint8_t RTR,PTMSG_Data Data,
 uint8_t iFrameType )
{
    type_ID IDENT;
    unsigned char i,ByteAccess,DataLength,DataByte,BYTE;
    unsigned char *DataPointer;
    uint16_t crc;
    uint8_t data_field[8];
    uint8    bIsErrFrame;

	
	 /*prepare message*/
	 TxBitCount = 0;
	 CeckSum = 0;
	 
	 /* Initialize the data frame buffer */
	 for(i = 0; i < sizeof(TransmitDataFrame); i++)
	 {
	 	TransmitDataFrame[i]=0;
	 }
	 
	 /* Start of Frame */
	 TxBitCount++;														
	 ByteAccess = (TxBitCount-1)/8;
	 TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 -> dominant SOF*/
	 CalcCheckSum(TransmitDataFrame[ByteAccess]);
	 CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	
	 /*Arbitration Field*/
	 if(IDType == FLEXCAN_EXTENDED)
	 {
	 	 IDENT.EXTENDED = (MsgID << 3);		// 29 bits ID
		 	 	 
	 }
	 else
	 {
	     IDENT.EXTENDED = (MsgID << 21); 	// 11 bits ID
	 }
	 for(i=0; i < 11; i++)										 /* ID */
	 {
		  TxBitCount++;
		  ByteAccess = (TxBitCount-1)/8;
		  TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
		   
		  if(IDENT.BYTE.B3 & MSB)		/*if MSB is '1' */
		  {
		    TransmitDataFrame[ByteAccess] |= 0x01;
		  }
		  CalcCheckSum(TransmitDataFrame[ByteAccess]);
		  CheckBitStuffing(TransmitDataFrame[ByteAccess]);
		  IDENT.EXTENDED <<= 1;										 /* Left shift by 1 for checking the next bit */
	 }
	 if(IDType == FLEXCAN_EXTENDED)
	 {
	 	// Generate SRR and IDE bits
	 	TxBitCount++;
	        ByteAccess = (TxBitCount-1)/8;
		TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
                TransmitDataFrame[ByteAccess] |= 0x01;	 /* SRR bit */
 		CalcCheckSum(TransmitDataFrame[ByteAccess]);
		CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	 	TxBitCount++;
	        ByteAccess = (TxBitCount-1)/8;
		TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
                TransmitDataFrame[ByteAccess] |= 0x01;	 /* IDE bit */
 		CalcCheckSum(TransmitDataFrame[ByteAccess]);
		CheckBitStuffing(TransmitDataFrame[ByteAccess]);  
		
		// Generate low 18-bit ID    
		 for(i=0; i < 18; i++)										 /* ID */
		 {
			  TxBitCount++;
			  ByteAccess = (TxBitCount-1)/8;
			  TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
			   
			  if(IDENT.BYTE.B3 & MSB)		/*if MSB is '1' */
			  {
			    TransmitDataFrame[ByteAccess] |= 0x01;
			  }
			  CalcCheckSum(TransmitDataFrame[ByteAccess]);
			  CheckBitStuffing(TransmitDataFrame[ByteAccess]);
			  IDENT.EXTENDED <<= 1;										 /* Left shift by 1 for checking the next bit */
		 }	
	 }
	 
	// Generate RTR bit
 	TxBitCount++;
    ByteAccess = (TxBitCount-1)/8;
	TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
    TransmitDataFrame[ByteAccess] |= (RTR? 0x01: 0);	 /* RTR bit */
	CalcCheckSum(TransmitDataFrame[ByteAccess]);
	CheckBitStuffing(TransmitDataFrame[ByteAccess]);  			  		 	
	
	
	 /*Control Field*/
	 /* r1/IDE & r0 both a dominant for standard ID.
	  *  
	  */
    for(i=0; i < 2; i++)										
    {
       TxBitCount++;														
       ByteAccess = (TxBitCount-1)/8;
       TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 -> dominant r1/r0*/
       CalcCheckSum(TransmitDataFrame[ByteAccess]);
       CheckBitStuffing(TransmitDataFrame[ByteAccess]);
    }

	  /*DLC*/
	  DataLength = Data->LENGTH << 4;            /* Left shift by 4 for access to the MSB*/
	  for(i=0; i < 4; i++)										
	    {
	     TxBitCount++;														
	     ByteAccess = (TxBitCount-1)/8;
	     TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
	
	     if(DataLength & MSB)		/*if MSB is '1' */
	     {
	       TransmitDataFrame[ByteAccess] |= 0x01;
	     }
	     CalcCheckSum(TransmitDataFrame[ByteAccess]);
	     CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	     DataLength <<= 1;												 /* Left shift by 1 for checking the next bit */
	    }
	
	 /*Data Field*/
	  DataPointer = &Data->BYTE0;									 /* point to the first data byte */
	  /* Save to local data field buffer. */
	  for(DataByte =0; DataByte < Data->LENGTH; DataByte++)
	  {
	  		data_field[DataByte]= DataPointer[DataByte];
	  }  
	  swap_4bytes(data_field);	// swap bytes to big endian mode
	  swap_4bytes(data_field+4);// swap bytes to big endian mode
	  
	  for(DataByte=0; DataByte < Data->LENGTH; DataByte++)										
	  {
	    BYTE = data_field[DataByte];
	   // printf("data =%#02.2x\n",BYTE);
	    for(i=0; i < 8; i++)										
	    {
	       TxBitCount++;														
	       ByteAccess = (TxBitCount-1)/8;
	       TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
	
	       if(BYTE & MSB)		/*if MSB is '1' */
	       {
	         TransmitDataFrame[ByteAccess] |= 0x01;
	       }
	       CalcCheckSum(TransmitDataFrame[ByteAccess]);
	       CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	       BYTE <<= 1;														 /* Left shift by 1 for checking the next bit */
		}
		DataPointer++; /* point to the next byte */
	 }
	 /* Save CRC */
	 crc = CeckSum;
	 if(iFrameType == CAN_FRAME_TYPE_CRC_ERROR)
	 {
	 	CeckSum++;	// generate wrong CRC to be transmitted
	 }
	 /*CRC Field */
	 /* CRC sequence */
	  CeckSum <<= 1;            /* Left shift by 1 for access to the MSB*/
	  for(i=0; i < 15; i++)										
	  {
	     TxBitCount++;														
	     ByteAccess = (TxBitCount-1)/8;
	     TransmitDataFrame[ByteAccess] <<= 1;    
	
	     if(CeckSum & MSB_16bit)		/*if MSB is '1' */
	     {
	       TransmitDataFrame[ByteAccess] |= 0x01;
	     }
	     CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	     CeckSum <<= 1;												 /* Left shift by 1 for checking the next bit */
	  }
	  /* CRC delemiter*/
	  TxBitCount++;														
	  ByteAccess = (TxBitCount-1)/8;
	  TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 */
	  if(iFrameType != CAN_FRAME_TYPE_FORM_ERROR)
	  {	  
	  	TransmitDataFrame[ByteAccess] |= 0x01;	 /* CRC delemiter is always recessive*/
	  }
	  //CheckBitStuffing(TransmitDataFrame[ByteAccess]);
	  if(iFrameType == CAN_FRAME_TYPE_STUF_ERROR)
	  {
	  	 TransmitDataFrame[ByteAccess] ^= 1;	// generate wrong stuf bit
	  }
	
	/*Acknowledge Field */
	  for(i=0; i < 2; i++)										
	  {
	     TxBitCount++;														
	     ByteAccess = (TxBitCount-1)/8;
	     TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 -> recessive Ack Slot and Del*/
	     TransmitDataFrame[ByteAccess] |= 0x01;	  /* Ack field is always Tx as recessive by the transmitter */
	   }
          
        /* EOF field */
         bIsErrFrame = 0;
         for(i=0; i <7;i++)
         {
            TxBitCount++;
            ByteAccess = (TxBitCount-1)/8;                  
	    TransmitDataFrame[ByteAccess] <<= 1;   
            TransmitDataFrame[ByteAccess] |= 1;      // filled with Receceive bits      
            bIsErrFrame = ((iFrameType == CAN_FRAME_TYPE_EOF_7th_BIT_ERROR) && (i==6)) 
                        || ((iFrameType == CAN_FRAME_TYPE_EOF_6th_BIT_ERROR) && (i==5))
                        || ((iFrameType == CAN_FRAME_TYPE_EOF_5th_BIT_ERROR) && (i==4))
                        || ((iFrameType == CAN_FRAME_TYPE_EOF_4th_BIT_ERROR) && (i==3))
                        || ((iFrameType == CAN_FRAME_TYPE_EOF_3th_BIT_ERROR) && (i==2))
                        || ((iFrameType == CAN_FRAME_TYPE_EOF_2th_BIT_ERROR) && (i==1))
                        || ((iFrameType == CAN_FRAME_TYPE_EOF_1th_BIT_ERROR) && (i==0))
                        ;
                        
            if(bIsErrFrame)
            {
              TransmitDataFrame[ByteAccess] ^= 1;   // generate dominant bit at EOF
            }
         }
         //
	 i= TxBitCount;
	 while((i%8) !=0)
	 {
            TxBitCount++;
            //ByteAccess = (TxBitCount-1)/8;
	    TransmitDataFrame[ByteAccess] <<= 1;
            TransmitDataFrame[ByteAccess] |= 1;      // filled with Receceive bits      
	    i++;
	 }
	return (crc);
}

/******************************************************************************
Function Name	:	CheckBitStuffing
Engineer      :	
Date          :	

Parameters    :	none
Returns       :	none
Notes         :	check whether bit stuffing is required
******************************************************************************/

void CheckBitStuffing(unsigned char BIT)
{
unsigned char ByteAccess;
//static unsigned char dominant= 0,recessive = 0;

  if(BIT & 0x01)
    {
		/* recessive bit */
    recessive++;
    dominant= 0;

    if(recessive == 5)
      {
       /* bit stuffing of a dominant bit */
       recessive = 0;
       TxBitCount++;														
       ByteAccess = (TxBitCount-1)/8;
       TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 -> dominant data frame */
      }
    }
  else
    {
		/* dominant bit */
    dominant++;
    recessive= 0;

    if(dominant == 5)
      {
       /* bit stuffing of a recessive bit */
       dominant = 0;
       TxBitCount++;														
       ByteAccess = (TxBitCount-1)/8;
       TransmitDataFrame[ByteAccess] <<= 1;     /* Left shift by 1 -> dominant data frame */
       TransmitDataFrame[ByteAccess] |= 0x01;   /* recessive bit */
      }
  
    }  
}


void TPM_SendBitStream(void)
{
 word ByteAccess;
 
 /* Enable FTM1 interrupt in NVIC */
  enable_irq(63);
 
 /* append 1 bit at the end of TransmitDataFrame */
 TransmitDataFrame[TxBitCount/8] |= 1<<(7-TxBitCount%8);
   
 /* Start sending message */ 
 CAN_Msg_Sending = TRUE;


 /* NOTE: Write mechanism */
 FTM1_SC = 0;
 FTM1_CNT = 0;
// FTM1_OUTINIT = 0xFFFF; /* set initial output to HIGH/recessive */
// FTM1_MODE = 3;
 
//TxSentBit = 0;	 /* flag SOF bit sent at the start of TPM */
 //BitMask = 0x80; /* point to 2nd bit which shall send */

  TxSentBit = 1;	 /* flag SOF bit sent at the start of TPM */
  BitMask = 0x40; /* point to 2nd bit which shall send */ 
#if 0
 /* Write TPMxSC first to reset latch mechanism of MOD/CnV registers */
 FTM1SC_CPWMS = 0;
 FTM1SC_PS =   2;        /* prescalor= /4 */
 
 FTM1C0SC_MS0x = 1;     /* output compare mode */


 ByteAccess = TxSentBit/8;
 

 if (TransmitDataFrame[ByteAccess] & BitMask)
 {
      FTM1C0SC_ELS0x = 3;    /* set output when output compare*/ 
 }
 else
 {
      FTM1C0SC_ELS0x = 2;    /* clear output when output compare*/ 
 }
 
 FTM1_MOD = 0xFFFF;
 
 FTM1C0VL = CAN_BIT_TIME;	    /* set bit time  */
 FTM1C0VH = 0;
 
 TxSentBit++;	/* move to next bit */
 BitMask >>= 1; 

 FTM1C0SC_CH0IE = 1;    /* enable channel interrupt */
 
#else

/* Write TPMxSC first to reset latch mechanism of MOD/CnV registers */
 FTM1_SC &= ~(FTM_SC_CPWMS_MASK);
 FTM1_SC = (FTM1_SC & ~(FTM_SC_PS_MASK)) | FTM_SC_PS(2);  /* prescalor= /4 */
 
 FTM1C0SC_MS0x = 1;     /* output compare mode */


 ByteAccess = TxSentBit/8;
 

 if (TransmitDataFrame[ByteAccess] & BitMask)
 {
      FTM1C0SC_ELS0x = 3;    /* set output when output compare*/ 
 }
 else
 {
      FTM1C0SC_ELS0x = 2;    /* clear output when output compare*/ 
 }
 

 FTM1_MOD = 0xFFFF;
 
 FTM1C0V = CAN_BIT_TIME;	    /* set bit time  */
 
 TxSentBit++;	/* move to next bit */
 BitMask >>= 1; 

 FTM1C0SC_CH0IE = 1;    /* enable channel interrupt */
 

#endif
 
#ifdef  FTM_USE_EXT_CLOCK
  /* NOTE: as soon as TPM is enabled, it outputs 0:
  * SOF bit is the first bit.
  */
 FTM1SC_CLKSx= 3;        /* select external clock as source clock */ 
#else
 /* NOTE: as soon as TPM is enabled, it outputs 0:
  * SOF bit is the first bit.
  */
 FTM1SC_CLKSx= 1;        /* select bus clock as source clock */  
#endif
 
}

/******************************************************************************
Function Name	:	TIMER_CH0_ISR
Engineer      :	
Date          :	

Parameters    :	none
Returns       :	none
Notes         :	Timer ch 0 interrupt
******************************************************************************/
void TIMER_CH0_ISR(void)
{
unsigned char ByteAccess;  
 
#if 0
   FTM1CNTH = 0;

   /* Clear interrupt flag */
   FTM1C0SC;  // read register
   FTM1C0SC_CH0F = 0;

 
   if(TxSentBit > TxBitCount)
    {
      CAN_Msg_Sending = FALSE;
      
      /* all bits are sent */
       FTM1SC_CLKSx= 0;      /* disable timer */
       FTM1C0SC_CH0IE = 0;    /* disable channel interrupt */
      /* NOTE: make sure this pin is HIGH */
      
       return;
    }
   ByteAccess = TxSentBit/8;
    
   if (TransmitDataFrame[ByteAccess] & BitMask)
      FTM1C0SC_ELS0x = 3;    /* set output when output compare*/ 
   else
      FTM1C0SC_ELS0x = 2;    /* clear output when output compare*/ 
 
  // FTM1CNTH = 0;
#else
   FTM1_C0V += CAN_BIT_TIME;

   /* Clear interrupt flag */
   FTM1C0SC;  // read register
   FTM1C0SC_CH0F = 0;
 
   if(TxSentBit >= TxBitCount)
    {
      CAN_Msg_Sending = FALSE;
      
      /* all bits are sent */
       FTM1SC_CLKSx= 0;      /* disable timer */
       FTM1C0SC_CH0IE = 0;    /* disable channel interrupt */
      /* NOTE: make sure this pin is HIGH */
      
       return;
    }
   ByteAccess = TxSentBit/8;
    
   if (TransmitDataFrame[ByteAccess] & BitMask)
      FTM1C0SC_ELS0x = 3;    /* set output when output compare*/ 
   else
      FTM1C0SC_ELS0x = 2;    /* clear output when output compare*/    
#endif
   BitMask >>= 1;
   
   if (BitMask == 0)
    BitMask = 0x80;
   
   TxSentBit++; 
}


/******************************************************************************
Function Name	:	CalcCheckSum
Engineer      :	
Date          :	

Parameters    :	Boolin
Returns       :	none
Notes         :	caculate check sum - > how to do check CAN specification
******************************************************************************/
void CalcCheckSum(unsigned char NXTBIT)
{
unsigned char CRCNXT;

		NXTBIT &= 0x01; /* just bit 0 will be used */

    if(CeckSum & 0x4000)  /*check the MSB of the check sum*/
      CRCNXT = NXTBIT ^ 1;
    else
      CRCNXT = NXTBIT ^ 0; 

		CeckSum <<=1;					/* Left shift by 1 */

		if (CRCNXT == 1)
		  {
		  CeckSum ^= 0x4599;
		  }

}


void DelayBits(uint32_t bits)
{
	FLEXCAN0_TIMER = 0;
	while(FLEXCAN0_TIMER < bits);
}

void DelayBits1(uint32_t bits)
{
	FLEXCAN1_TIMER = 0;
	while(FLEXCAN1_TIMER < bits);
}

void SelectionSort(uint32_t a[], int array_size)
{
     int i;
     for (i = 0; i < array_size - 1; ++i)
     {
          int j, min, temp;
          min = i;
          for (j = i+1; j < array_size; ++j)
          {
               if (a[j] < a[min])
                    min = j;
          }

          temp = a[i];
          a[i] = a[min];
          a[min] = temp;
     }
}	


#if (0)
__asm uint32_t EnterPrivilegeMode(void)
{
  	mrs r0,control	
  	mov	r1,#0
  	msr	control,r1	// ENTER privilege mode
  	bx  lr
}


__asm uint32_t EnterUserMode(void)
{
  	mrs r0,control
  	mov	r1,#1
  	msr	control,r1	// ENTER user mode
  	bx  lr
}


__asm uint32_t GetControlRegister(void)
{
	mrs r0,control
	bx  lr
}
#else
uint32_t EnterPrivilegeMode(void)
{
  
 asm(
 	"mrs r0,control	\n"
  	"mov	r1,#0 \n"
  	"msr	control,r1 \n"  // ENTER privilege mode
  	"bx  lr  \n"        
   );

  return 1;
}


uint32_t EnterUserMode(void)
{

 asm(
  	"mrs r0,control \n"
  	"mov	r1,#1 \n"
  	"msr	control,r1 \n"	// ENTER user mode
  	"bx  lr \n"
);

  	return 1;
}


uint32_t GetControlRegister(void)
{

    asm(
      "mrs r0,control \n"
      "bx  lr \n"
       );
  
	return 1;
}
#endif


void ConfigureAIPS_Lite(void)
{
	int32_t i;
	/*
	 * Clearing the PACR (Peripheral Access Control Register) and 
	 * OPACR (Off-Platform Peripheral Access Control Register) 
	 * contents will let the AIPS_LITE allow non-supervisor-privledged bus masters 
	 * to access the on and off platform IPS peripherals.  This includes the CPU 
	 * when the CPU is put into user mode.
	 */
#if 1       
   for(i=0;i<4;i++) /* PACR0-31 (16-bit each) */ 
    { 
		AIPS0_PACR(i) = 0x00000000; 
		AIPS1_PACR(i) = 0x00000000; 
    } 
    for(i=0;i<12;i++) /* OPACR0-95 */ 
    { 
		AIPS0_OPACR(i) = 0x00000000; 
		AIPS1_OPACR(i) = 0x00000000; 
    } 	
#else
   for(i=0;i<16;i++) /* PACR0-31 (16-bit each) */ 
    { 
		AIPS0_PACR(i) = 0x00000000; 
		AIPS1_PACR(i) = 0x00000000; 
    }    
#endif   
}


void NVIC_enable_PIT_interrupts(void)
{
      /*
        PIT interrupt vector number 84,85,86,87
       IRQ value: 84-16 = 68,69,70,71
       IRQ VEC reg: 68/32 = 2.xx
       bit to set : 68%32 = 4,5,6,7
       */
       enable_irq(68);       
       enable_irq(69);       
       enable_irq(70);       
       enable_irq(71);       
}
    
                                  
void FlexCAN1_ERR002623_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i,iTxMB;
        uint32_t  cs;

	printf("Start to test ERR002623... \n");
 
 	// Initialize error counter
	guiErrCount = 0;
	guiMB_ISR_Count = 0;

  	/* Initialize all 16 MBs */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 83;//1000;	// 1Mbps
 	  	
	// case 1: Rx FIFO is disabled
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,  
#ifdef  TEST_ON_EVM  
                FALSE
#else
		TRUE	// for loopback
#endif                  
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	// enable ABORT feature
	FLEXCAN1_MCR |= FLEXCAN_MCR_AEN;
	
	FLEXCAN1_RXMGMASK = 0x1FFFFFFF;	// 
 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
 	FLEXCAN1_IMASK1 = 0;	// disable interrupts
 	
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_RXIMRn(i) = 0x00000000;
	}
			 
	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	printf("FLEXCAN1_CTRL1=%#08.8x\n",FLEXCAN1_CTRL1);
	
	// Initialize MB0 as a receive MB with ID don't care
	FLEXCAN1_MBn_CS(0) = 0x00000000;
	FLEXCAN1_MBn_ID(0) = 0x18588885L;
	FLEXCAN1_MBn_CS(0) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
						 | FLEXCAN_MB_CS_IDE
	;
		
        // last but one MB is configured as Tx MB
	i = NUMBER_OF_MB-2;
	{	
		// Initialize MB1  as transmit MBs
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
		FLEXCAN1_MBn_ID(i) = 0x18588885L+i;
		FLEXCAN1_MBn_WORD0(i) = 0x11223344;
		FLEXCAN1_MBn_WORD1(i) = 0x55667788;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
								 | FLEXCAN_MB_CS_SRR;
	}
		
	// Write timer to some value
	FLEXCAN1_TIMER = 0;
	
	// Start CAN communication
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
	
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	// 
	// Wait for some times
	DelayBits1(128);
        
        
        // Send another frame that should be received on MB12
        i = 12;
        FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
        FLEXCAN1_MBn_ID(i) = 0x18588885L;
        FLEXCAN1_MBn_WORD0(i) = 0x5555;
        FLEXCAN1_MBn_WORD1(i) = 0xAAAA;
        FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                                                         | FLEXCAN_MB_CS_LENGTH(8)
                                                         | FLEXCAN_MB_CS_IDE
                                                         | FLEXCAN_MB_CS_SRR;
        
        // Delay for some time to matching window
        DelayBits1(128);
        
        // now invalidate the last MB
        i = 15;
        FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
        
        // check MB0
        while(!(FLEXCAN1_IFLAG1 & 1));	
        
        cs = FLEXCAN1_MBn_CS(0);  // lock MB0
        if(FLEXCAN1_MBn_ID(0) != 0x18588885L)
        {
          printf("Error: ID received is wrong!\r\n");
        }
        if(FLEXCAN1_MBn_WORD0(0) != 0x5555)
        {
          printf("Error: word0 received is wrong!\r\n");
        }
        if(FLEXCAN1_MBn_WORD1(0) != 0xAAAA)
        {
          printf("Error: word1 received is wrong!\r\n");
        }        
        
        printf("Test PASSed!\r\n");
} 
  


void FlexCAN_TKT029460_Test(void)
{
	uint16_t baudrate;
	uint32_t state;
	int32_t	 i,id;
	TMSG_Data	msg_data;
	unsigned char	*pBytes;
	/*
        Bug found in the following scenario:
        - MBi is configured as Rx Mailbox
        - Self Reception is enabled (MCR[SRX_DIS] = 0)
        - FlexCAN starts to transmit a frame whose ID matched with MBi ID.
        - FlexCAN wins bus arbitration.
        - A dominant bit occurs  during 7th bit of EOF
        - Frame reception fails (Iflag is not asserted and Rx Move is not performed)
        
        Obs: This behavior is only observed during self reception.
        */
	
	printf("Start to test CAN ticket tkt029460 on FlexCAN1 \n");
 
 	// Initialize error counter
	guiErrCount = 0;
	guiMB_ISR_Count = 0;


	/* Initialize all 16 MBs of FlexCAN1 */		  
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_MBn_CS(i) = 0x00000000;
	  	FLEXCAN1_MBn_ID(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD0(i) = 0x00000000;
	  	FLEXCAN1_MBn_WORD1(i) = 0x00000000;
	}
	
 	// Initialize the FlexCAN0
	baudrate = 83;//250; //1000;	// 1Mbps
 	  	
	// 
	state = FLEXCAN_Initialize(FLEXCAN1, 0, 0,baudrate,
#ifdef  USE_EXTERNAL_CLOCK    
                        FLEXCAN_OSC_CLK
#else
                        FLEXCAN_IPBUS_CLK
#endif                          
                          ,                                   
		FALSE
		);	
	if(state != FLEXCAN_OK)
	{
		printf("FLEXCAN_Initialize() returned state = %#08x\n",state);
	}	
	
	// Now FlexCAN is in Freeze mode.
	FLEXCAN1_RXMGMASK = 0x0;	// receive any message,can only be written in Freeze mode
 	FLEXCAN1_RX14MASK = 0x1FFFFFFF;
 	FLEXCAN1_RX15MASK = 0x1FFFFFFF;
 		
 	// receive any messages
	for(i=0;i<NUMBER_OF_MB;i++)
	{
	  	FLEXCAN1_RXIMRn(i) = 0x00000000;
	}
			 
	// Enable individual Rx masking and queue enable
	FLEXCAN1_MCR |= FLEXCAN_MCR_IRMQ;

        // Enable self reception
        FLEXCAN1_MCR &= ~CAN_MCR_SRXDIS_MASK;
        
	// clear CTRL2 but TASD
	FLEXCAN1_CTRL2 &= (FLEXCAN_CTRL2_TASD);
	
	
	// Initialize MB0 to MB14 as a receive MB with ID don't care
	for(i=0;i<NUMBER_OF_MB-1;i++)
	{
		FLEXCAN1_MBn_CS(i) = 0x00000000;
		FLEXCAN1_MBn_ID(i) = 0x00000000;
		FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) 
							 | FLEXCAN_MB_CS_IDE
		;
	}	
	FLEXCAN1_IMASK1 = 0xFFFF;	// enable interrupts
	
	
	// Start CAN communication of FlexCAN0 and FlexCAN1
	FLEXCAN1_MCR ^= FLEXCAN_MCR_HALT;
		
	while(FLEXCAN1_MCR & (FLEXCAN_MCR_FRZ_ACK|FLEXCAN_MCR_NOT_RDY)) {}
	
	// Now generate CAN frames with errors
	id = 0x1F678957;
	msg_data.LENGTH = 8;
	pBytes = &(msg_data.BYTE0);
       // printf("Bytes:\r\n");
	for(i=0;i<msg_data.LENGTH;i++)
	{
		pBytes[i] = 0xf0+i;
              //  printf("%#02.2x ",pBytes[i]);
	}
	// Disable bus-off recovery mode
	// enable error interrupt
	FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_BOFF_REC 
					| FLEXCAN_CTRL_ERR_MSK
	; 	        
#if 1   
        // Disable MB15 interrupt
        FLEXCAN1_IMASK1 &= ~(1<<15);
        GPIOA_PCR(4) = (PIN_ALT1<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;; // configure PTA4 as GPIO
        GPIOA_PDOR |= (1<<4);
        GPIOA_POER &= ~(1<<4);
        PORTA_PCR4 |= PORT_PCR_IRQC(10);  // interrupt on falling edge
        enable_irq(87);

        GPIOA_PDOR |= (1<<12);
        GPIOA_POER |= (1<<12);
        PORTA_PCR12 = (PIN_ALT1<<PIN_MUX_BIT_NO) | PIN_PULL_SELECT_MASK | PIN_PULL_ENABLE_MASK;; // configure PTA12 as GPIO
        
        // Set up timer
        FTM1SC_CPWMS = 0;
        FTM1SC_PS =   2;        /* prescalor= /4 */  
        FTM1C0SC_MS0x = 1;     /* output compare mode */
        FTM1_MOD = 0xFFFF;    
        FTM1C0V = CAN_BIT_TIME * 131;	    /* set bit time  */  
        FTM1C0SC_CH0IE = 1;    /* enable channel interrupt */
        enable_irq(63);        /* enable NVIC FTM1 interrupt */
        
        // Send this frame out via MB15
        i = 15;
	FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
	FLEXCAN1_MBn_ID(i) = 0x1F678957;
	FLEXCAN1_MBn_WORD0(i) = (pBytes[0] <<24) | (pBytes[1] << 16) | (pBytes[2] << 8) | (pBytes[3]);
	FLEXCAN1_MBn_WORD1(i) = (pBytes[4] <<24) | (pBytes[5] << 16) | (pBytes[6] << 8) | (pBytes[7]);
	FLEXCAN1_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
								 | FLEXCAN_MB_CS_LENGTH(8)
								 | FLEXCAN_MB_CS_IDE
       								 | FLEXCAN_MB_CS_SRR;
       // printf("\r\nword0 = %#08.8x,word1=%#08.8x\r\n",FLEXCAN1_MBn_WORD0(i),FLEXCAN1_MBn_WORD1(i)); 
        FLEXCAN1_TIMER = 0;
        while(!(FLEXCAN1_IFLAG1 & (1<<i)))
        {

        }       
        // clear MB flags
        FLEXCAN1_IFLAG1 = (1<<i);
#else     
        
        // Convert little endian to big endian bytes order
        swap_4bytes(pBytes);
        swap_4bytes(pBytes+4);
#if 0
        printf("a correct CAN frame is generated and sent via TPM\n");        
	BuildCANFrame(id, FLEXCAN_EXTENDED, FALSE,&msg_data,CAN_FRAME_TYPE_CORRECT);
 	TPM_SendBitStream();
	
	while(CAN_Msg_Sending){}
#endif  
#if 0        
        printf("a CAN frame with the EOF 6th bit error is generated and sent via TPM\n");        
	BuildCANFrame(id, FLEXCAN_EXTENDED, FALSE,&msg_data,CAN_FRAME_TYPE_EOF_6th_BIT_ERROR);
 	TPM_SendBitStream();
	
	while(CAN_Msg_Sending){}
#endif
#if 1       
        printf("a CAN frame with the EOF 7th bit error is generated and sent via TPM\n");        
	BuildCANFrame(id, FLEXCAN_EXTENDED, FALSE,&msg_data,CAN_FRAME_TYPE_EOF_7th_BIT_ERROR);
 	TPM_SendBitStream();
	
	while(CAN_Msg_Sending){}
#endif
#endif        
 	while(guiMB_ISR_Count<1); 	
        printf("Test completed!\r\n");
        while(1);
}



                                  
                                  