/******************************************************************************
* File:    isr.h
* Purpose: Define interrupt service routines referenced by the vector table.
* Note: Only "vectors.c" should include this header file.
******************************************************************************/

#ifndef __ISR_H
#define __ISR_H 1

#include "test_config.h"

// This is an example of how to define custom ISRs in the vector table.

#ifdef	TEST_REGISTERS_TEST
	#undef		VECTOR_003
	#define	VECTOR_003	HardFault_ISR
#if TEST_FLEXCAN0
        #undef  VECTOR_048
        #undef  VECTOR_049
        #define VECTOR_048      FlexCAN0_TxWarningISR
        #define VECTOR_049      FlexCAN0_RxWarningISR
#endif
#if TEST_FLEXCAN1
        #undef  VECTOR_056
        #undef  VECTOR_057    
        #define VECTOR_056      FlexCAN1_TxWarningISR
        #define VECTOR_057      FlexCAN1_RxWarningISR
#endif
#endif

#ifdef	TEST_SELF_LOOP01
	#undef	 VECTOR_045	
	#define	VECTOR_045	FlexCAN0_SelfLooop01_MB_ISR
#if TEST_FLEXCAN1
	#undef		VECTOR_053	
	#define	VECTOR_053	FlexCAN1_SelfLooop01_MB_ISR
#endif

#elif	defined(TEST_SELF_LOOP2)
	#undef	 VECTOR_045	
	#define	VECTOR_045	FlexCAN0_Selfloop2_MB_ISR
#endif

#ifdef	TEST_ACCESS_MODE
	#undef	 VECTOR_045	
	#define	VECTOR_045	FlexCAN0_AccessMode_MB_ISR
	#undef		VECTOR_053	
	#define	VECTOR_053	FlexCAN1_AccessMode_RxFIFO_ISR	
#endif

#ifdef	TEST_RXFIFO_FILTER
	#undef		VECTOR_045	
	#undef		VECTOR_053
	#define	VECTOR_045	FlexCAN0_RxFIFO_Filter_MB_ISR	
	#define	VECTOR_053  FlexCAN1_frames_available_ISR	
#endif

#ifdef	TEST_RXFIFO_INT
	#undef		VECTOR_045
	#undef		VECTOR_053
	#define	VECTOR_045	FlexCAN0_RxFIFO_INT_ISR
	#define	VECTOR_053	FlexCAN1_RxFIFO_INT_ISR
#endif

#ifdef	TEST_TSYNC
	#undef		VECTOR_045	
	#define	VECTOR_045	FlexCAN0_TSYNC_MB_ISR	
        #undef          VECTOR_053
	#define	VECTOR_053	FlexCAN1_TSYNC_MB_ISR      
#endif

#ifdef	TEST_REMOTE_FRAME
	#undef		VECTOR_045	
	#define	VECTOR_045	FlexCAN0_RxFIFO_Filter_MB_ISR	
#endif

#ifdef	TEST_PRIORITY
	#undef		VECTOR_045	
	#define	VECTOR_045	FlexCAN0_Priority_MB_ISR	
        #undef          VECTOR_053
	#define	VECTOR_053	FlexCAN1_Priority_MB_ISR      
#endif

#ifdef	TEST_INDIVIDUAL_MASKING
	#undef		VECTOR_045	
	#define	VECTOR_045	FlexCAN0_IndiMasking_MB_ISR	
#endif

#ifdef	TEST_TX_ABORT
	#undef		VECTOR_053	
	#define	VECTOR_053	FlexCAN1_TxAbort_MB_ISR	
#endif
#ifdef	TEST_BUS_OFF
	#undef		VECTOR_045	
	#define	VECTOR_045	FlexCAN0_BusOff_MB_ISR		
#endif
#ifdef	TEST_MB_ERROR_INT
	#undef		VECTOR_047
	#undef		VECTOR_055
	#define	VECTOR_047	FlexCAN0_Error_ISR
	#define	VECTOR_055	FlexCAN1_Error_ISR
#endif

#ifdef	TEST_CRC_FORM_STUF_ERROR
	#undef  	VECTOR_079
	#undef	 	VECTOR_055
	#undef		VECTOR_053
	
	#define	VECTOR_053	FlexCAN1_MB_INT_ISR
	#define	VECTOR_055	FlexCAN1_Error_ISR	
	#define 	VECTOR_079  FTM1_ISR	
#endif

#ifdef	TEST_TKT029460
	#undef  	VECTOR_079
	#undef	 	VECTOR_055
	#undef		VECTOR_053
        #undef          VECTOR_103
	
	#define	VECTOR_053	FlexCAN1_MB_INT_ISR
	#define	VECTOR_055	FlexCAN1_Error_ISR	
	#define VECTOR_079      FTM1_TKT029460_ISR	
        #define VECTOR_103      PortA_ISR
#endif

#ifdef	TEST_LOW_POWER_MODE
	#undef		VECTOR_050	// WAKEUP
	#define	VECTOR_050	FlexCAN0_Wakeup_ISR
	#undef		VECTOR_084
	#define	VECTOR_084	PIT0_ISR
#endif

#if	defined(TEST_TXRX_Test2) || defined(TEST_TXRX_Test3)
	#undef		VECTOR_084
	#define	VECTOR_084	PIT0_CANTx_ISR
#endif

#ifdef	TEST_SAFE_RECONFIG_MB
	#undef	 VECTOR_045	
	#define	VECTOR_045	FlexCAN0_Reconfig_MB_ISR
	#undef	 VECTOR_052
	#define VECTOR_052	FlexCAN0_LostReceiveFrame_ISR
	#undef	 VECTOR_051
	#define VECTOR_051	FlexCAN0_IMEU_ISR

	#undef	 VECTOR_053	
	#define	VECTOR_053	FlexCAN1_Reconfig_MB_ISR
	#undef	 VECTOR_060
	#define VECTOR_060	FlexCAN1_LostReceiveFrame_ISR
	#undef	 VECTOR_059
	#define VECTOR_059	FlexCAN1_IMEU_ISR
#endif

#if	defined(TEST_SAFE_RECONFIG_FIFO_FORMAT_A)
	#undef	 VECTOR_045	
	#define	VECTOR_045	FlexCAN0_Reconfig_FIFO_ISR
	#undef	 VECTOR_052
	#define VECTOR_052	FlexCAN0_LostReceiveFrame_ISR
	#undef	 VECTOR_051
	#define VECTOR_051	FlexCAN0_IMEU_ISR

	#undef	 VECTOR_053	
	#define	VECTOR_053	FlexCAN1_Reconfig_FIFO_ISR
	#undef	 VECTOR_060
	#define VECTOR_060	FlexCAN1_LostReceiveFrame_ISR
	#undef	 VECTOR_059
	#define VECTOR_059	FlexCAN1_IMEU_ISR

#endif


//#define VECTOR_050  FlexCAN0_Wakeup_ISR
#undef  VECTOR_054
#define VECTOR_054  FlexCAN1_BOFF_ISR
//#define VECTOR_055  FlexCAN1_Err_ISR

#undef  VECTOR_101
#define VECTOR_101   LPTmr_ISR

// ISR(s) are defined in "isr.c".
extern void  default_isr(void);
extern void HardFault_ISR(void);
extern void FlexCAN1_frames_available_ISR(void);
extern void FlexCAN1_BOFF_ISR(void);
extern void FlexCAN1_Err_ISR(void);
extern void FlexAN0_tx_warning_ISR (void);
extern void FlexCAN0_SelfLooop01_MB_ISR(void);
extern void FlexCAN1_SelfLooop01_MB_ISR(void);
extern void FlexCAN0_Selfloop2_MB_ISR(void);
extern void FlexCAN0_RxFIFO_Filter_MB_ISR(void);
extern void FlexCAN0_RxFIFO_INT_ISR(void);
extern void FlexCAN1_RxFIFO_INT_ISR(void);
extern void FlexCAN0_TSYNC_MB_ISR(void);
extern void FlexCAN1_TSYNC_MB_ISR(void);
extern void FlexCAN0_Priority_MB_ISR(void);
extern void FlexCAN1_Priority_MB_ISR(void);
extern void FlexCAN0_IndiMasking_MB_ISR(void);
extern void FlexCAN1_TxAbort_MB_ISR(void);
extern void FlexCAN0_BusOff_MB_ISR(void);
extern void FlexCAN0_Error_ISR(void);
extern void FlexCAN1_Error_ISR(void);
extern void FTM1_ISR(void);
extern void TIMER_CH0_ISR(void);
extern void FlexCAN1_MB_INT_ISR(void);
extern void FlexCAN0_Wakeup_ISR(void);
extern void PIT0_ISR(void);
extern void PIT0_CANTx_ISR(void);
extern void FlexCAN0_Reconfig_MB_ISR(void);
extern void FlexCAN1_Reconfig_MB_ISR(void);
extern void FlexCAN0_Reconfig_FIFO_ISR(void);
extern void FlexCAN1_Reconfig_FIFO_ISR(void);
extern void FlexCAN0_AccessMode_MB_ISR(void);
extern void FlexCAN1_AccessMode_RxFIFO_ISR(void);
extern void FlexCAN0_LostReceiveFrame_ISR(void);
extern void FlexCAN1_LostReceiveFrame_ISR(void);
extern void FlexCAN0_IMEU_ISR(void);
extern void FlexCAN1_IMEU_ISR(void);
extern void FlexCAN0_TxWarningISR(void);
extern void FlexCAN0_RxWarningISR(void);
extern void FlexCAN1_TxWarningISR(void);
extern void FlexCAN1_RxWarningISR(void);
extern void LPTmr_ISR(void);

extern void PortA_ISR(void);
extern void FTM1_TKT029460_ISR(void);

#endif  //__ISR_H

/* End of "isr.h" */
