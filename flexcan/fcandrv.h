#ifndef __FlexCAN_h_
#define __FlexCAN_h_

#include "types.h"
#include "common.h"
#include "kinetis_flexcan.h"
#include "test_config.h"

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
* FileName: fcandrv.h
* Version : 1.0.0
*
* Comments:
*
*   This include file is used to provide constant and structure definitions
*   specific to the FlexCAN Serial Communications Controller
*   Revision History:
*   Dec 9, 2009   2.50          Initial version
*
*END************************************************************************/

#define MAX_MB_SUPPORTED		64		// max # of MBs supported by this FlexCAN module
#define NUMBER_OF_MB			16		// acutal # of MBs implemented in this chip

#define SYSTEM_CLOCK			(96000000L)  //(100000000L) // refer to CORE_CLK_MHZ

#ifdef  USE_EXTERNAL_CLOCK
  #define FLEXCAN_OSC_CLK			(1) /* must be changed TO 1 for external clock */ 
#else
  #define FLEXCAN_OSC_CLK			(0) /* must be changed TO 1 for external clock */ 
#endif

#if (FLEXCAN_OSC_CLK)
  #define BSP_SYSTEM_CLOCK		 (12000000L)  // 12MHz external crystal
#else
  #define BSP_SYSTEM_CLOCK		 (SYSTEM_CLOCK/2)
#endif

#define FLEXCAN_IPBUS_CLK		  (SYSTEM_CLOCK/2) /* This is a the actul frequecy feeds CAN clock module*/

#define FLEXCAN_MSG_BUFADDR_OFFSET   		(0x80)
#define FLEXCAN_MSG_BUFF_MAXDATA_SIZE		(8)


#define FLEXCAN0		0
#define FLEXCAN1		1


// concrete HW settings:
    
#define FLEXCAN_MESSBUF_INT_LEVEL		(3)
#define FLEXCAN_MESSBUF_INT_SUBLEVEL		(4)

#define FLEXCAN_ERROR_INT_LEVEL			(3)
#define FLEXCAN_ERROR_INT_SUBLEVEL		(2)

#define FLEXCAN_BUSOFF_INT_LEVEL		(3)
#define FLEXCAN_BUSOFF_INT_SUBLEVEL		(3)

#define FLEXCAN_WAKEUP_INT_LEVEL		(3)
#define FLEXCAN_WAKEUP_INT_SUBLEVEL		(1)


// iterface provided:

/*
** CAN interrupt types enum
*/
#define FLEXCAN_INT_BUF					(0)
#define FLEXCAN_INT_ERR					(1)
#define FLEXCAN_INT_BOFF				(2)
#define FLEXCAN_INT_WAKEUP				(3)

/*
** CAN commands
*/
#define FLEXCAN_TX                      (0xFFu)
#define FLEXCAN_RX                      (0x00u)
#define FLEXCAN_EXTENDED                (0xFFu)
#define FLEXCAN_STANDARD                (0x00u)

/*
** Module mode
*/
#define FLEXCAN_NORMAL_MODE				(0)
#define FLEXCAN_LISTEN_MODE				(1)
#define FLEXCAN_TIMESYNC_MODE			(2)
#define FLEXCAN_LOOPBK_MODE				(3)
#define FLEXCAN_BOFFREC_MODE			(4)
#define FLEXCAN_FREEZE_MODE				(5)
#define FLEXCAN_DISABLE_MODE			(6)

/*
** FLEXCAN error codes
*/
#define FLEXCAN_OK                       (0x00)
#define FLEXCAN_ERROR_BASE				 (0x100)
#define FLEXCAN_UNDEF_ERROR              (FLEXCAN_ERROR_BASE | 0x01)
#define FLEXCAN_MESSAGE14_TX             (FLEXCAN_ERROR_BASE | 0x02)
#define FLEXCAN_MESSAGE15_TX             (FLEXCAN_ERROR_BASE | 0x03)
#define FLEXCAN_MESSAGE_OVERWRITTEN      (FLEXCAN_ERROR_BASE | 0x04)
#define FLEXCAN_NO_MESSAGE               (FLEXCAN_ERROR_BASE | 0x05)
#define FLEXCAN_MESSAGE_LOST             (FLEXCAN_ERROR_BASE | 0x06)
#define FLEXCAN_MESSAGE_BUSY		     (FLEXCAN_ERROR_BASE | 0x07)
#define FLEXCAN_MESSAGE_ID_MISSMATCH     (FLEXCAN_ERROR_BASE | 0x08)
#define FLEXCAN_MESSAGE14_START	         (FLEXCAN_ERROR_BASE | 0x09)
#define FLEXCAN_MESSAGE15_START		     (FLEXCAN_ERROR_BASE | 0x0A)
#define FLEXCAN_INVALID_ADDRESS          (FLEXCAN_ERROR_BASE | 0x0B)
#define FLEXCAN_INVALID_MAILBOX          (FLEXCAN_ERROR_BASE | 0x0C)
#define FLEXCAN_TIMEOUT                  (FLEXCAN_ERROR_BASE | 0x0D)
#define FLEXCAN_INVALID_FREQUENCY        (FLEXCAN_ERROR_BASE | 0x0E)
#define FLEXCAN_INT_ENABLE_FAILED		 (FLEXCAN_ERROR_BASE | 0x0F)
#define FLEXCAN_INT_DISABLE_FAILED		 (FLEXCAN_ERROR_BASE | 0x10)
#define FLEXCAN_INT_INSTALL_FAILED		 (FLEXCAN_ERROR_BASE | 0x11)
#define FLEXCAN_REQ_MAILBOX_FAILED		 (FLEXCAN_ERROR_BASE | 0x12)
#define FLEXCAN_DATA_SIZE_ERROR			 (FLEXCAN_ERROR_BASE | 0x13)
#define FLEXCAN_MESSAGE_FORMAT_UNKNOWN	 (FLEXCAN_ERROR_BASE | 0x14)
#define FLEXCAN_INVALID_DIRECTION		 (FLEXCAN_ERROR_BASE | 0x15)
#define FLEXCAN_RTR_NOT_SET				 (FLEXCAN_ERROR_BASE | 0x16)
#define FLEXCAN_SOFTRESET_FAILED		 (FLEXCAN_ERROR_BASE | 0x17)
#define FLEXCAN_INVALID_MODE			 (FLEXCAN_ERROR_BASE | 0x18)
#define FLEXCAN_START_FAILED			 (FLEXCAN_ERROR_BASE | 0x19)
#define FLEXCAN_CLOCK_SOURCE_INVALID	 (FLEXCAN_ERROR_BASE | 0x1A)
#define FLEXCAN_INIT_FAILED				 (FLEXCAN_ERROR_BASE | 0x1B)
#define FLEXCAN_ERROR_INT_ENABLE_FAILED  (FLEXCAN_ERROR_BASE | 0x1C)
#define FLEXCAN_ERROR_INT_DISABLE_FAILED (FLEXCAN_ERROR_BASE | 0x1D)
#define FLEXCAN_FREEZE_FAILED			 (FLEXCAN_ERROR_BASE | 0x1E)
#define FLEXCAN_MB_NOT_EMPTY_FAILED		 (FLEXCAN_ERROR_BASE | 0x1F)
#define FLEXCAN_MB_BUSY_FAILED			 (FLEXCAN_ERROR_BASE | 0x20)
#define FLEXCAN_MB_ABORTED				 (FLEXCAN_ERROR_BASE | 0x21)
#define FLEXCAN_MB_TRANSMITTED			 (FLEXCAN_ERROR_BASE | 0x21)
#define FLEXCAN_MB_IMEU_FLAG_ERROR		 (FLEXCAN_ERROR_BASE | 0x22)	

/* Message buffer memory map */
#define FLEXCAN_MSGCTRL_RTR					(0x00100000)
#define FLEXCAN_MSGCTRL_IDE					(0x00200000)
#define FLEXCAN_MSGCTRL_SRR					(0x00400000)
#define FLEXCAN_MSGCRTL_CODE					(0x0F000000)
#define FLEXCAN_MSG_CTRL_DLEN					(0x000F0000)
#define FLEXCAN_MSG_DISABLE          			(~FLEXCAN_MSGCRTL_CODE)
#define FLEXCAN_MSGCTRL_CODE_BIT_NO					(24)
#define FLEXCAN_set_msg_ctrlcode(code)					((code & 0x0F)<<(FLEXCAN_MSGCTRL_CODE_BIT_NO))
#define FLEXCAN_get_msg_ctrlcode(cs)					((cs>>(FLEXCAN_MSGCTRL_CODE_BIT_NO)) & 0x0F)
/*
** Message Buffer Codes for Rx Buffers
*/
#define FLEXCAN_RX_MSG_BUFFER_NOT_ACTIVE     (0x00000000)
#define FLEXCAN_RX_MSG_BUFFER_EMPTY          (0x04000000)
#define FLEXCAN_RX_MSG_BUFFER_FULL           (0x02000000)
#define FLEXCAN_RX_MSG_BUFFER_OVERRUN        (0x06000000)
#define FLEXCAN_RX_MSG_BUFFER_BUSY           (0x01000000)

/*
 * IMEU bit masks for Rx Message Buffer 
 */
#define FLEXCAN_RX_MB_IMEU_REQUEST_MASK		(0x80000000) 
#define FLEXCAN_RX_MB_IMEU_ACK_MASK			(0x40000000)

/*
 * IMEU bit masks for Rx FIFO (refer to FUREQ/FUACK registers)
 */
#define FLEXCAN_RX_FIFO_IMEU_REQUEST_MASK(n)	(BIT##n)
#define FLEXCAN_RX_FIFO_IMEU_ACK_MASK(n)		(BIT##n)


/* Message Buffer Codes for Tx Buffers */
#define FLEXCAN_TX_MSG_BUFFER_NOT_ACTIVE     	(0x08000000)
#define FLEXCAN_MESSAGE_TRANSMIT_ONCE 			(0x0C000000)
#define FLEXCAN_MESSAGE_TRANSMIT_REMOTE		(0x0C000000)
#define FLEXCAN_MESSAGE_TRANSMIT_RESPONSE		(0x0A000000)
#define FLEXCAN_MESSAGE_TRANSMIT_RESPONSE_ONLY	(0x0E000000)
#define FLEXCAN_TX_MB_ABORT					(0x09000000)

/* Interrupt masks */
#define FLEXCAN_WAKEUP_INT			 	(0x0001)
#define FLEXCAN_ERROR_INT			 	(0x0002)
#define FLEXCAN_BUSOFF_INT			 	(0x0004)
#define FLEXCAN_ALL_INT              	(0x0007)
#define FLEXCAN_TX_RX_INT				(~ FLEXCAN_ALL_INT)
#define FLEXCAN_IMASK_VALUE				(0xFFFFFFFFL)


/*------------------------------------------------------------------------*/
/*
** FCAN  registers bit set.
*/

/* Bit definitions and macros for FCAN_CANMCR */
#define FCAN_CANMCR_MAXMB(x)          (((x)&0x7F))
#define FCAN_CANMCR_LPMACK            (0x100000)
#define FCAN_CANMCR_SUPV              (0x800000)
#define FCAN_CANMCR_FRZACK            (0x1000000)
#define FCAN_CANMCR_SOFTRST           (0x2000000)
#define FCAN_CANMCR_NOTRDY            (0x8000000)
#define FCAN_CANMCR_HALT              (0x10000000)
#define FCAN_CANMCR_FRZ               (0x40000000)
#define FCAN_CANMCR_MDIS              (0x80000000)

/* Bit definitions and macros for FCAN_CANCTRL */
#define FCAN_CANCTRL_PROPSEG(x)       (((x)&0x7L))
#define FCAN_CANCTRL_LOM              (0x8)
#define FCAN_CANCTRL_LBUF             (0x10)
#define FCAN_CANCTRL_TSYNC            (0x20)
#define FCAN_CANCTRL_BOFFREC          (0x40)
#define FCAN_CANCTRL_SAMP             (0x80)
#define FCAN_CANCTRL_LPB              (0x1000L)
#define FCAN_CANCTRL_CLK_SRC          (0x2000L)
#define FCAN_CANCTRL_ERRMSK           (0x4000L)
#define FCAN_CANCTRL_BOFFMSK          (0x8000L)
#define FCAN_CANCTRL_PSEG2(x)         (((x)&0x7L)<<16)
#define FCAN_CANCTRL_PSEG1(x)         (((x)&0x7L)<<19)
#define FCAN_CANCTRL_RJW(x)           (((x)&0x3L)<<22)
#define FCAN_CANCTRL_PRESDIV(x)       (((x)&0xFFL)<<24)

/* Bit definitions and macros for FCAN_TIMER */
#define FCAN_TIMER_TIMER(x)           (((x)&0xFFFF)<<0)

/* Bit definitions and macros for FCAN_RXGMASK */
#define FCAN_RXGMASK_MI(x)            (((x)&0x1FFFFFFF)<<0)

/* Bit definitions and macros for FCAN_RX14MASK */
#define FCAN_RX14MASK_MI(x)           (((x)&0x1FFFFFFF)<<0)

/* Bit definitions and macros for FCAN_RX15MASK */
#define FCAN_RX15MASK_MI(x)           (((x)&0x1FFFFFFF)<<0)

/* Bit definitions and macros for FCAN_RX15MASK */
#define FCAN_RX15MASK_MI(x)           (((x)&0x1FFFFFFF)<<0)

/* Bit definitions and macros for ID masks */
#define FCAN_STANDARD_ID_MASK(x)      (((x)&0x7FF)<<18)
#define FCAN_EXTENDED_ID_MASK(x)      (((x)&0x1FFFFFFF)<<0)

/* Bit definitions and macros for FCAN_ERRCNT */
#define FCAN_ERRCNT_TXECTR(x)         (((x)&0xFF)<<0)
#define FCAN_ERRCNT_RXECTR(x)         (((x)&0xFF)<<0x8)

/* Bit definitions and macros for FCAN_ERRSTAT */
#define FCAN_ERRSTAT_ERRINT           (0x2)
#define FCAN_ERRSTAT_BOFFINT          (0x4)
#define FCAN_ERRSTAT_FLTCONF(x)       (((x)&0x3)<<0x4)
#define FCAN_ERRSTAT_FLTCONF_ACTIVE   (0)
#define FCAN_ERRSTAT_FLTCONF_PASSIVE  (0x10)
#define FCAN_ERRSTAT_FLTCONF_BUSOFF   (0x20)
#define FCAN_ERRSTAT_TXRX             (0x40)
#define FCAN_ERRSTAT_IDLE             (0x80)
#define FCAN_ERRSTAT_RXWRN            (0x100)
#define FCAN_ERRSTAT_TXWRN            (0x200)
#define FCAN_ERRSTAT_STFERR           (0x400)
#define FCAN_ERRSTAT_FRMERR           (0x800)
#define FCAN_ERRSTAT_CRCERR           (0x1000)
#define FCAN_ERRSTAT_ACKERR           (0x2000)
#define FCAN_ERRSTAT_BIT0ERR          (0x4000)
#define FCAN_ERRSTAT_BIT1ERR          (0x8000)

/* Bit definitions and macros for FCAN_IMASK */
#define FCAN_IMASK_BUF0M              (0x1)
#define FCAN_IMASK_BUF1M              (0x2)
#define FCAN_IMASK_BUF2M              (0x4)
#define FCAN_IMASK_BUF3M              (0x8)
#define FCAN_IMASK_BUF4M              (0x10)
#define FCAN_IMASK_BUF5M              (0x20)
#define FCAN_IMASK_BUF6M              (0x40)
#define FCAN_IMASK_BUF7M              (0x80)
#define FCAN_IMASK_BUF8M              (0x100)
#define FCAN_IMASK_BUF9M              (0x200)
#define FCAN_IMASK_BUF10M             (0x400)
#define FCAN_IMASK_BUF11M             (0x800)
#define FCAN_IMASK_BUF12M             (0x1000)
#define FCAN_IMASK_BUF13M             (0x2000)
#define FCAN_IMASK_BUF14M             (0x4000)
#define FCAN_IMASK_BUF15M             (0x8000)
#define FCAN_IMASK_BUF(x)             (0x1<<(x))

/* Bit definitions and macros for FCAN_IFLAG */
#define FCAN_IFLAG_BUF0I              (0x1)
#define FCAN_IFLAG_BUF1I              (0x2)
#define FCAN_IFLAG_BUF2I              (0x4)
#define FCAN_IFLAG_BUF3I              (0x8)
#define FCAN_IFLAG_BUF4I              (0x10)
#define FCAN_IFLAG_BUF5I              (0x20)
#define FCAN_IFLAG_BUF6I              (0x40)
#define FCAN_IFLAG_BUF7I              (0x80)
#define FCAN_IFLAG_BUF8I              (0x100)
#define FCAN_IFLAG_BUF9I              (0x200)
#define FCAN_IFLAG_BUF10I             (0x400)
#define FCAN_IFLAG_BUF11I             (0x800)
#define FCAN_IFLAG_BUF12I             (0x1000)
#define FCAN_IFLAG_BUF13I             (0x2000)
#define FCAN_IFLAG_BUF14I             (0x4000)
#define FCAN_IFLAG_BUF15I             (0x8000)
#define FCAN_IFLAG_BUF(x)             (0x1<<(x))

/*-------------------------------------------------------------------------
 * FCAN ESR2 register bit definitions
\*------------------------------------------------------------------------*/
#define FCAN_ESR2_LPTM_MASK			(0x007F0000)				
#define FCAN_ESR2_LPTM_MASK			(0x007F0000)				
#define FCAN_ESR2_VPS_MASK			(0x00004000)
#define FCAN_ESR2_IMB_MASK			(0x00002000)
#define FCAN_ESR2_MUF_MASK			(0x00000800)
#define FCAN_ESR2_FUF_MASK			(0x00000400)

#define FCAN_ESR2_LPTM_BIT_NO		(16)

/*------------------------------------------------------------------------*/
/*
** FCAN registers bit set GENERALIZED
*/

#define FLEXCAN_CANMCR_MAXMB                  FCAN_CANMCR_MAXMB
#define FLEXCAN_CANMCR_FRZACK                 FCAN_CANMCR_FRZACK
#define FLEXCAN_CANMCR_SOFTRST                FCAN_CANMCR_SOFTRST
#define FLEXCAN_CANMCR_HALT                   FCAN_CANMCR_HALT
#define FLEXCAN_CANMCR_FRZ                    FCAN_CANMCR_FRZ
#define FLEXCAN_CANMCR_MDIS                   FCAN_CANMCR_MDIS
#define FLEXCAN_CANMCR_LPMACK                 FCAN_CANMCR_LPMACK
#define FLEXCAN_CANMCR_NOTRDY                 FCAN_CANMCR_NOTRDY

#define FLEXCAN_CANCTRL_LOM                   FCAN_CANCTRL_LOM
#define FLEXCAN_CANCTRL_TSYNC                 FCAN_CANCTRL_TSYNC
#define FLEXCAN_CANCTRL_LPB                   FCAN_CANCTRL_LPB
#define FLEXCAN_CANCTRL_BOFFREC               FCAN_CANCTRL_BOFFREC
#define FLEXCAN_CANCTRL_SAMP                  FCAN_CANCTRL_SAMP
#define FLEXCAN_CANCTRL_CLK_SRC               FCAN_CANCTRL_CLK_SRC
#define FLEXCAN_CANCTRL_ERRMSK                FCAN_CANCTRL_ERRMSK
#define FLEXCAN_CANCTRL_BOFFMSK               FCAN_CANCTRL_BOFFMSK

#define FLEXCAN_CANCTRL_PROPSEG               FCAN_CANCTRL_PROPSEG
#define FLEXCAN_CANCTRL_PSEG2                 FCAN_CANCTRL_PSEG2
#define FLEXCAN_CANCTRL_PSEG1                 FCAN_CANCTRL_PSEG1
#define FLEXCAN_CANCTRL_RJW                   FCAN_CANCTRL_RJW
#define FLEXCAN_CANCTRL_PRESDIV              FCAN_CANCTRL_PRESDIV

#define FLEXCAN_STANDARD_ID_MASK              FCAN_STANDARD_ID_MASK
#define FLEXCAN_EXTENDED_ID_MASK              FCAN_EXTENDED_ID_MASK

/* Useful utilities */
#define FLEXCAN_get_crc_mb(crcr)				((crcr & FLEXCAN_CRCR_MBCRC_MASK)>>FLEXCAN_CRCR_MBCRC_BIT_NO) 
#define FLEXCAN_get_crc(crcr)					((crcr & FLEXCAN_CRCR_CRC_MASK)>>FLEXCAN_CRCR_CRC_BIT_NO) 
/*
**                                                                    
** FlexCAN Registers
*/
/* FlexCAN MSG Object structure */
typedef	union TID 
{
   uint_32 ID;
   struct 
   {
   	uint_32	ID_LOW : 18;
   	uint_32 ID_HIGH : 11;
   	uint_32 PRIO: 3;
   } IDBits;
} TID;

typedef struct TFCAN_MsgBuf
{
   uint_32 CONTROL;        /* CTRL/STATUS word */
   TID     ID;             /* ID word */
   uint_8  DATA[8];	       /* bytes Data Field */

} TFCAN_MSG_STRUCT, * FCAN_MSG_STRUCT_PTR;

typedef struct TFCAN_Regs
{   
   uint_32  CANMCR;      // Module Configuration Register
   uint_32  CANCTRL;     // FlexCAN Control Register
   uint_32  TIMER;       // Free Running TImer
   uint_32  TCR;	     // Test configuration register
   uint_32  RXGMASK;     // Rx Global Mask
   uint_32  RX14MASK;    // Rx Buffer 14 Mask
   uint_32  RX15MASK;    // Rx Buffer 15 Mask
   uint_32  ECR; 	     // Error Counter Register
   uint_32  ESR1;            // Error and Status1
   uint_32  IMASK2;	     // Interrupt Mask2
   uint_32  IMASK1;	     // Interrupt Mask1
   uint_32  IFLAG2;	     // Interrupt flag 2
   uint_32  IFLAG1;	     // Interrupt flag 2   
   uint_32  CTRL2;	     // Control 2 register
   uint_32  ESR2;	     // Error and Status 2 Register
   uint_32  IMEUR;		// Individual Matching Elements Update Register
   uint_32  LRFR;		// Lost Rx Frames Register
   uint_32  CRCR;		// CRC Register
   uint_32  RXFGMASK;	// Rx FIFO Global Mask Register
   uint_32  RXFIR;		// Rx FIFO Information Register
   uint_32  rsv1[2];
   uint_32  DBG1;		// Debug 1 Register
   uint_32  DBG2;		// Debug 2 Register
   uint_32  rsv2[8];
   TFCAN_MSG_STRUCT MB[MAX_MB_SUPPORTED];	// 64 MB for FlexCAN3
   uint_32  rsv3[256];
   uint_32  RXIMR[MAX_MB_SUPPORTED];		// Rx Individual Mask Registers
} TFCAN_STRUCT, *TFCAN_STRUCT_PTR;

typedef volatile TFCAN_STRUCT_PTR VFCAN_STRUCT_PTR;

typedef volatile TFCAN_MSG_STRUCT VFCAN_MSG_STRUCT;
typedef volatile FCAN_MSG_STRUCT_PTR  VFCAN_MSG_STRUCT_PTR;

typedef VFCAN_MSG_STRUCT      FLEXCAN_MSG_OBJECT_STRUCT;
typedef VFCAN_MSG_STRUCT_PTR  FLEXCAN_MSG_OBJECT_STRUCT_PTR;
typedef VFCAN_STRUCT_PTR      FLEXCAN_REG_STRUCT_PTR;


/* Definition of mailbox */
//typedef  struct {
typedef volatile struct {	
   uint_8	dev_num;				/*  FlexCAN device number */
   uint_16	mailbox_number; 		/*  mailbox number */
   uint_32	identifier;   			/*  message ID */
   uint_8	format;					/*  mailbox format (FLEXCAN_STANDARD OR FLEXCAN_EXTENDED) */ 
   uint_8	direction; 				/*  transmission or reception direction (FLEXCAN_TX or FLEXCAN_RX) */
   uint_8	remote_req_flag;		/*  is remote request?   */  
   uint_8	data_len;			/*  number of bytes to write to or read from the mailbox (0 to 8) */ 
   uint_8	data[8]; 				/*  data bytes */
   uint_8	code;				/* control code */ 
   uint_8	priority;			/*  priority of the message in the mailbox */
   uint_32	timestamp;			/*  timestamp */
   uint_16	crc;				/*  crc of the message frame */
   uint_32	imask;				/* individual mask */
 } TFCAN_MailBox, *PTFCAN_MailBox;
 typedef	TFCAN_MailBox		VTFCAN_MailBox;
 typedef	PTFCAN_MailBox	VPTFCAN_MailBox;
 //typedef	volatile TFCAN_MailBox		VTFCAN_MailBox;
 //typedef	volatile PTFCAN_MailBox	VPTFCAN_MailBox;
 typedef	VTFCAN_MailBox		FLEXCAN_MailBox_STRUCT;
 typedef	VPTFCAN_MailBox		FLEXCAN_MailBox_STRUCT_PTR;
 
#ifdef __cplusplus
extern "C" {
#endif
extern uint_32 _flexcan_int_init
   (
      // [IN} Interrupt number
      PSP_INTERRUPT_TABLE_INDEX irq,

      // [IN} Interrupt priority level
      _int_level                level,

      // [IN} Interrupt sub-priority level within priority
      _int_priority             sublevel,

      // [IN} Unmask the interrupt now?
      boolean                   unmask
   );
extern FLEXCAN_REG_STRUCT_PTR _bsp_get_flexcan_base_address(uint_8 dev_num);
extern uint_32 FLEXCAN_Softreset(uint_8);
extern uint_32 FLEXCAN_Start(uint_8);
extern vpointer FLEXCAN_Get_msg_object(uint_8,uint_32);
extern uint_32 FLEXCAN_Select_mode(uint_8,uint_32);
extern uint_32 FLEXCAN_Select_clk(uint_8,uint_32);
extern uint_32 FLEXCAN_Initialize
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
);
extern uint_32 FLEXCAN_Initialize_mailbox
(
   FLEXCAN_MailBox_STRUCT_PTR pMailBox,
   uint_8  activate_it,
   uint_32 int_enable
);
extern uint_32 FLEXCAN_Request_mailbox(uint_8,uint_32,uint_32);
extern uint_32 FLEXCAN_Activate_mailbox(uint_8,uint_32,uint_32);
extern uint_32 FLEXCAN_Lock_mailbox(uint_8,uint_32);
extern uint_32 FLEXCAN_Unlock_mailbox(uint_8);
extern uint_32 FLEXCAN_Set_global_extmask(uint_8,uint_32);
extern uint_32 FLEXCAN_Set_buf14_extmask(uint_8,uint_32);
extern uint_32 FLEXCAN_Set_buf15_extmask(uint_8,uint_32);
extern uint_32 FLEXCAN_Set_global_stdmask(uint_8,uint_32);
extern uint_32 FLEXCAN_Set_buf14_stdmask(uint_8,uint_32);
extern uint_32 FLEXCAN_Set_buf15_stdmask(uint_8,uint_32);
extern boolean FLEXCAN_Tx_successful(uint_8);
extern uint_32 FLEXCAN_Tx_mailbox(uint_8,uint_32,vpointer,uint_8);
extern uint_32 FLEXCAN_Rx_mailbox(uint_8,uint_32,vpointer);
extern uint_32 FLEXCAN_Disable_mailbox(uint_8,uint_32);
extern uint_32 FLEXCAN_Request_message(uint_8,uint_32,uint_32);
extern uint_32 FLEXCAN_Rx_message
( 
   FLEXCAN_MailBox_STRUCT_PTR pMailBox,
   uint_32     int_enable
);
extern uint_32 FLEXCAN_Tx_message(uint_8,uint_32,uint_32,uint_32,uint_32,vpointer);
extern uint_32 FLEXCAN_Read(uint_8,uint_32,uint_32_ptr);
extern uint_32 FLEXCAN_Write(uint_8,uint_32,uint_32);
extern uint_32 FLEXCAN_Get_status(uint_8,uint_32_ptr);
extern uint_32 FLEXCAN_Update_message(uint_8,vpointer,uint_32,uint_32,uint_32);
extern uint_32 FLEXCAN_Int_enable(uint_8,uint_32);
extern uint_32 FLEXCAN_Error_int_enable(uint_8);
extern uint_32 FLEXCAN_Int_disable(uint_8,uint_32);
extern uint_32 FLEXCAN_Error_int_disable(uint_8);
extern uint_32 FLEXCAN_Int_status(uint_8);
extern uint_32	FLEXCAN_Check_TxMB_State( /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number
   );
extern uint_32 FLEXCAN_EnterCritical(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number
   );   
extern void FLEXCAN_ExitCritical(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_32 mailbox_number,
   /* [IN] mask */
   uint_32 mask
   );
uint_32 FLEXCAN_Reconfig_Rx_mailbox_block
(
 FLEXCAN_MailBox_STRUCT_PTR pMailBox
);
  
boolean FLEXCAN_Is_MB_Done
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_16 mailbox_number
);
uint_32 FLEXCAN_Clear_MB_Flag
(
   /* [IN] FlexCAN device number */
   uint_8 dev_num,
   /* [IN] mailbox number */
   uint_16 mailbox_number
);
#ifdef __cplusplus
}
#endif

#endif
/* EOF */
