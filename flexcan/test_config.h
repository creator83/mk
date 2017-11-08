#ifndef __TEST_CONFIG_H_
#define __TEST_CONFIG_H_

// Target to be selected
#define K53N
//#define MARCONI
// define a macro to activate which test case to be tested
#define   TEST_FLEXCAN0   1
#define   TEST_FLEXCAN1   1

//#define CAN0_USE_PTB18_19   // works for CAN0
#define CAN0_USE_PTA12_13 // not work on K53 BGA, but work on K60 LQFP

//#define USE_EXTERNAL_CLOCK
//#define	TEST_ON_EVM
#define	TEST_MESSAGE_QUEUE

//#define	TEST_RESET_TEST
//#define	TEST_REGISTERS_TEST
//#define	TEST_SELF_LOOP0
//#define	TEST_SELF_LOOP01
//#define	TEST_SELF_LOOP2 // can work in external loop mode
//#define	TEST_TXRX_Test
//#define	TEST_TXRX_Test2
//#define	TEST_TXRX_Test3
//#define	TEST_CRC_STATUS
//#define	TEST_RXFIFO_INT
//#define	TEST_REMOTE_FRAME
//#define	TEST_TSYNC
//#define	TEST_PRIORITY
//#define	TEST_INDIVIDUAL_MASKING
//#define	TEST_TX_ABORT
//#define	TEST_BAUD
//#define	TEST_FREEZE_MODE
//#define	TEST_DISABLE_MODE
//#define	TEST_BUS_OFF
//#define 	TEST_MB_ERROR_INT
//#define	TEST_CRC_FORM_STUF_ERROR
//#define	TEST_LOW_POWER_MODE
//#define	TEST_LISTEN_ONLY_MODE
//#define	TEST_SAFE_RECONFIG_MB
//#define	TEST_SAFE_RECONFIG_FIFO_FORMAT_A
#define	TEST_ACCESS_MODE
//#define	TEST_IPgear_35536
//#define 	TEST_IPgear_35532
//#define	TEST_ECC
//#define TEST_ERR002623
//#define	TEST_RXFIFO_FILTER
//#define TEST_TKT029460

#ifdef		TEST_RXFIFO_FILTER
//#define	TEST_RXFIFO_FILTER_FORMAT_A
//#define	TEST_RXFIFO_FILTER_FORMAT_B
//#define	TEST_RXFIFO_FILTER_FORMAT_C
//#define	TEST_RXFIFO_FILTER_FORMAT_D
#endif



/* MB # used for Rx MB and Tx MBs */
#define	NUMBER_OF_MB	16
#define	FLEXCAN_RX_MB_START	0
#define	FLEXCAN_RX_MB_END	((NUMBER_OF_MB >>1)-1)
#define FLEXCAN_TX_MB_START	(FLEXCAN_RX_MB_END+1)
#define	FLEXCAN_TX_MB_END	(NUMBER_OF_MB-1)

/* Rx FIFO filter 0 shift bits */
#define FILTER0_SHIFT_BITS_UPPER	19	// for filter format B
#define FILTER0_SHIFT_BITS_LOWER	3

#endif /*__TEST_CONFIG_H_*/
