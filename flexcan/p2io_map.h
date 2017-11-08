#ifndef P2IO_MAP_H_
#define P2IO_MAP_H_

#include "types.h"

// AIPS_Lite registers
#define	AIPS0_RBASE		(0x40000000L)
#define	AIPS1_RBASE		(0x40080000L)

// PACR (Peripheral Access Control Register) registers
#define	AIPS0_PACR(i)			(*(vuint32_t*)(AIPS0_RBASE+0x20+(i<<2)))
#define	AIPS1_PACR(i)			(*(vuint32_t*)(AIPS1_RBASE+0x20+(i<<2)))

// OPACR (Off-Platform Peripheral Access Control Register) 
#define	AIPS0_OPACR(i)			(*(vuint32_t*)(AIPS0_RBASE+0x40+(i<<2)))
#define	AIPS1_OPACR(i)			(*(vuint32_t*)(AIPS1_RBASE+0x40+(i<<2)))

// SIM registers
#define 	SIM_BASE		(0x40048000L)
#define	CLKDIV1			(*(vuint32_t*)(SIM_BASE+0x2C))
#define	CLKDIV2			(*(vuint32_t*)(SIM_BASE+0x30))
#define SIM_SCGC3               (*(vuint32_t*)(SIM_BASE+0x30))
#define SIM_SCGC6               (*(vuint32_t*)(SIM_BASE+0x3C))

// MCG register
#define	MCG_BASE		(0x40064000L)
#define	MCGC1			(*(vuint8_t*)(MCG_BASE))
#define	MCGC2			(*(vuint8_t*)(MCG_BASE+1))
#define	MCGC3			(*(vuint8_t*)(MCG_BASE+2))
#define	MCGC4			(*(vuint8_t*)(MCG_BASE+3))
#define	MCGC5			(*(vuint8_t*)(MCG_BASE+4))
#define	MCGC6			(*(vuint8_t*)(MCG_BASE+5))
#define	MCGS			(*(vuint8_t*)(MCG_BASE+6))
#define	MCGT1			(*(vuint8_t*)(MCG_BASE+7))
#define	MCGT2			(*(vuint8_t*)(MCG_BASE+8))
#define	MCGT3			(*(vuint8_t*)(MCG_BASE+9))
#define	MCGATCVH		(*(vuint8_t*)(MCG_BASE+10))
#define	MCGATCVL		(*(vuint8_t*)(MCG_BASE+11))

#define MCGC5_PLLCLKEN_MASK   (0x40)

// OSC registers & bits
#define OSC_CR_ERCLKEN_MASK    (0x80)
#define OSC_CR_EREFSTEN_MASK   (0x20) 


// Pin control and interrupt (PCTL_IRQ/GPIOC)
#define	GPIOA_PCTL_IRQ_BASE	(0x40049000L)
#define	GPIOA_PCR0			(*(vuint32_t*)(GPIOA_PCTL_IRQ_BASE))
#define	GPIOA_PCR(n)		        (*(vuint32_t*)(GPIOA_PCTL_IRQ_BASE+(n<<2)))
#define	GPIOA_GPCLR			(*(vuint32_t*)(GPIOA_PCTL_IRQ_BASE+0x80))
#define	GPIOA_GPCHR			(*(vuint32_t*)(GPIOA_PCTL_IRQ_BASE+0x84))
#define	GPIOA_ISFR			(*(vuint32_t*)(GPIOA_PCTL_IRQ_BASE+0xA0))
#define	GPIOA_IOFR			(*(vuint32_t*)(GPIOA_PCTL_IRQ_BASE+0xA4))
#define	GPIOA_DFER			(*(vuint32_t*)(GPIOA_PCTL_IRQ_BASE+0xC0))
#define	GPIOA_DFCR			(*(vuint32_t*)(GPIOA_PCTL_IRQ_BASE+0xC4))
#define	GPIOA_DFWR			(*(vuint32_t*)(GPIOA_PCTL_IRQ_BASE+0xC8))

#define	GPIOB_PCTL_IRQ_BASE	(0x4004A000L)
#define	GPIOB_PCR0			(*(vuint32_t*)(GPIOB_PCTL_IRQ_BASE))
#define	GPIOB_PCR(n)		        (*(vuint32_t*)(GPIOB_PCTL_IRQ_BASE+(n<<2)))
#define	GPIOB_GPCLR			(*(vuint32_t*)(GPIOB_PCTL_IRQ_BASE+0x80))
#define	GPIOB_GPCHR			(*(vuint32_t*)(GPIOB_PCTL_IRQ_BASE+0x84))
#define	GPIOB_ISFR			(*(vuint32_t*)(GPIOB_PCTL_IRQ_BASE+0xA0))
#define	GPIOB_IOFR			(*(vuint32_t*)(GPIOB_PCTL_IRQ_BASE+0xA4))
#define	GPIOB_DFER			(*(vuint32_t*)(GPIOB_PCTL_IRQ_BASE+0xC0))
#define	GPIOB_DFCR			(*(vuint32_t*)(GPIOB_PCTL_IRQ_BASE+0xC4))
#define	GPIOB_DFWR			(*(vuint32_t*)(GPIOB_PCTL_IRQ_BASE+0xC8))

#define	GPIOE_PCTL_IRQ_BASE	(0x4004D000L)
#define	GPIOE_PCR0			(*(vuint32_t*)(GPIOE_PCTL_IRQ_BASE))
#define	GPIOE_PCR(n)		(*(vuint32_t*)(GPIOE_PCTL_IRQ_BASE+(n<<2)))
#define	GPIOE_GPCLR			(*(vuint32_t*)(GPIOE_PCTL_IRQ_BASE+0x80))
#define	GPIOE_GPCHR			(*(vuint32_t*)(GPIOE_PCTL_IRQ_BASE+0x84))
#define	GPIOE_ISFR			(*(vuint32_t*)(GPIOE_PCTL_IRQ_BASE+0xA0))
#define	GPIOE_IOFR			(*(vuint32_t*)(GPIOE_PCTL_IRQ_BASE+0xA4))
#define	GPIOE_DFER			(*(vuint32_t*)(GPIOE_PCTL_IRQ_BASE+0xC0))
#define	GPIOE_DFCR			(*(vuint32_t*)(GPIOE_PCTL_IRQ_BASE+0xC4))
#define	GPIOE_DFWR			(*(vuint32_t*)(GPIOE_PCTL_IRQ_BASE+0xC8))


#define	GPIOC_PCTL_IRQ_BASE	(0x4004B000L)
#define	GPIOC_PCR0			(*(vuint32_t*)(GPIOC_PCTL_IRQ_BASE))
#define	GPIOC_PCR(n)		        (*(vuint32_t*)(GPIOC_PCTL_IRQ_BASE+(n<<2)))
#define	GPIOC_GPCLR			(*(vuint32_t*)(GPIOC_PCTL_IRQ_BASE+0x80))
#define	GPIOC_GPCHR			(*(vuint32_t*)(GPIOC_PCTL_IRQ_BASE+0x84))
#define	GPIOC_ISFR			(*(vuint32_t*)(GPIOC_PCTL_IRQ_BASE+0xA0))
#define	GPIOC_IOFR			(*(vuint32_t*)(GPIOC_PCTL_IRQ_BASE+0xA4))
#define	GPIOC_DFER			(*(vuint32_t*)(GPIOC_PCTL_IRQ_BASE+0xC0))
#define	GPIOC_DFCR			(*(vuint32_t*)(GPIOC_PCTL_IRQ_BASE+0xC4))
#define	GPIOC_DFWR			(*(vuint32_t*)(GPIOC_PCTL_IRQ_BASE+0xC8))


// Pin mux control 
#define	PIN_DISABLED		(0)
#define	PIN_ALT1			(1)
#define	PIN_ALT2			(2)
#define	PIN_ALT(n)			(n)
#define	PIN_MUX_BIT_NO		(8)

// Pull Select mask
#define	PIN_PULL_SELECT_MASK	(1)
#define PIN_PULL_ENABLE_MASK    (2)

// Open-drain mask
#define PIN_ODE_MASK            (1<<5)


#define	GPIO_BASE			(0x400FF000L)
#define	GPIOA_PDOR			(*(vuint32_t*)(GPIO_BASE))
#define	GPIOA_PSOR			(*(vuint32_t*)(GPIO_BASE+4))
#define	GPIOA_PCOR			(*(vuint32_t*)(GPIO_BASE+8))
#define	GPIOA_PTOR			(*(vuint32_t*)(GPIO_BASE+0x0C))
#define	GPIOA_PDIR			(*(vuint32_t*)(GPIO_BASE+0x10))
#define	GPIOA_POER			(*(vuint32_t*)(GPIO_BASE+0x14))
#define	GPIOA_PIER			(*(vuint32_t*)(GPIO_BASE+0x18))
#define	GPIOA_P0DIR			(*(vuint32_t*)(GPIO_BASE+0x20))
#define	GPIOA_PnDIR(n)		        (*(vuint32_t*)(GPIO_BASE+0x20+(n<<2)))


// PMC
#define	PMC_BASE			(0x4007D000L)
#define	PMC_PMCTRL			(*(vuint8_t*)(PMC_BASE+3))
#define	PMC_LVDSC1			(*(vuint8_t*)(PMC_BASE+5))
#define	PMC_LVDSC2			(*(vuint8_t*)(PMC_BASE+6))
#define	PMC_REGSC			(*(vuint8_t*)(PMC_BASE+7))


// RAM
#define	TCMU_RAM_BASE		(0x20000000L)
#define	TCMU_RAM0			(*(vuint32_t*)(TCMU_RAM_BASE))


/*** FTM1SC - FTM1 Status and Control Register; 0x40039000 ***/
typedef union {
  byte Byte;
  struct {
    byte PS0         :1;                                       /* Prescale Divisor Select Bit 0 */
    byte PS1         :1;                                       /* Prescale Divisor Select Bit 1 */
    byte PS2         :1;                                       /* Prescale Divisor Select Bit 2 */
    byte CLKSA       :1;                                       /* Clock Source Select A */
    byte CLKSB       :1;                                       /* Clock Source Select B */
    byte CPWMS       :1;                                       /* Center-Aligned PWM Select */
    byte TOIE        :1;                                       /* Timer Overflow Interrupt Enable */
    byte TOF         :1;                                       /* Timer Overflow Flag */
  } Bits;
  struct {
    byte grpPS   :3;
    byte grpCLKSx :2;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} FTM1SCSTR;
//extern volatile FTM1SCSTR _FTM1SC @0x40039000;
#define P_FTM1SC	((volatile FTM1SCSTR*)0x40039000)
#define _FTM1SC	(*P_FTM1SC)

#define FTM1SC                          _FTM1SC.Byte
#define FTM1SC_PS0                      _FTM1SC.Bits.PS0
#define FTM1SC_PS1                      _FTM1SC.Bits.PS1
#define FTM1SC_PS2                      _FTM1SC.Bits.PS2
#define FTM1SC_CLKSA                    _FTM1SC.Bits.CLKSA
#define FTM1SC_CLKSB                    _FTM1SC.Bits.CLKSB
#define FTM1SC_CPWMS                    _FTM1SC.Bits.CPWMS
#define FTM1SC_TOIE                     _FTM1SC.Bits.TOIE
#define FTM1SC_TOF                      _FTM1SC.Bits.TOF
#define FTM1SC_PS                       _FTM1SC.MergedBits.grpPS
#define FTM1SC_CLKSx                    _FTM1SC.MergedBits.grpCLKSx

#define FTM1SC_PS0_MASK                 0x01
#define FTM1SC_PS1_MASK                 0x02
#define FTM1SC_PS2_MASK                 0x04
#define FTM1SC_CLKSA_MASK               0x08
#define FTM1SC_CLKSB_MASK               0x10
#define FTM1SC_CPWMS_MASK               0x20
#define FTM1SC_TOIE_MASK                0x40
#define FTM1SC_TOF_MASK                 0x80
#define FTM1SC_PS_MASK                  0x07
#define FTM1SC_PS_BITNUM                0x00
#define FTM1SC_CLKSx_MASK               0x18
#define FTM1SC_CLKSx_BITNUM             0x03


/*** FTM1CNT - FTM1 Timer Counter Register; 0x40039004 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** FTM1CNTH - FTM1 Timer Counter Register High; 0x40039004 ***/
    union {
      byte Byte;
    } FTM1CNTLSTR;
    #define FTM1CNTH                    _FTM1CNT.Overlap_STR.FTM1CNTHSTR.Byte
    

    /*** FTM1CNTL - FTM1 Timer Counter Register Low; 0x40039005 ***/
    union {
      byte Byte;
    } FTM1CNTHSTR;
    #define FTM1CNTL                    _FTM1CNT.Overlap_STR.FTM1CNTLSTR.Byte
    
    byte  Rsvd1;
    byte  Rsvd2;
  } Overlap_STR;

} FTM1CNTSTR;
//extern volatile FTM1CNTSTR _FTM1CNT @0x40039004;
#define P_FTM1CNT 	((volatile FTM1CNTSTR*)0x40039004)
#define _FTM1CNT	(*P_FTM1CNT)

#define FTM1CNT                         _FTM1CNT.Word


/*** FTM1MOD - FTM1 Timer Counter Modulo Register; 0x40039008 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** FTM1MODH - FTM1 Timer Counter Modulo Register High; 0x40039008 ***/
    union {
      byte Byte;
    } FTM1MODLSTR;
    #define FTM1MODH                    _FTM1MOD.Overlap_STR.FTM1MODHSTR.Byte
    

    /*** FTM1MODL - FTM1 Timer Counter Modulo Register Low; 0x40039009 ***/
    union {
      byte Byte;
    } FTM1MODHSTR;
    #define FTM1MODL                    _FTM1MOD.Overlap_STR.FTM1MODLSTR.Byte
    byte Rsvd1;
    byte Rsvd2;
  } Overlap_STR;

} FTM1MODSTR;
//extern volatile FTM1MODSTR _FTM1MOD @0x40039008;
#define  P_FTM1MOD	((volatile FTM1MODSTR*)0x40039008)
#define  _FTM1MOD	(*P_FTM1MOD)

#define FTM1MOD                         _FTM1MOD.Word


/*** FTM1C0SC - FTM1 Timer Channel 0 Status and Control Register; 0x4003900C ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte ELS0A       :1;                                       /* Edge/Level Select Bit A */
    byte ELS0B       :1;                                       /* Edge/Level Select Bit B */
    byte MS0A        :1;                                       /* Mode Select A for FTM1 Channel 0 */
    byte MS0B        :1;                                       /* Mode Select B for FTM1 Channel 0 */
    byte CH0IE       :1;                                       /* Channel 0 Interrupt Enable */
    byte CH0F        :1;                                       /* Channel 0 Flag */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte grpELS0x :2;
    byte grpMS0x :2;
    byte         :1;
    byte         :1;
  } MergedBits;
} FTM1C0SCSTR;
//extern volatile FTM1C0SCSTR _FTM1C0SC @0x4003900C;
#define _FTM1C0SC	(*(volatile FTM1C0SCSTR*)0x4003900C)

#define FTM1C0SC                        _FTM1C0SC.Byte
#define FTM1C0SC_ELS0A                  _FTM1C0SC.Bits.ELS0A
#define FTM1C0SC_ELS0B                  _FTM1C0SC.Bits.ELS0B
#define FTM1C0SC_MS0A                   _FTM1C0SC.Bits.MS0A
#define FTM1C0SC_MS0B                   _FTM1C0SC.Bits.MS0B
#define FTM1C0SC_CH0IE                  _FTM1C0SC.Bits.CH0IE
#define FTM1C0SC_CH0F                   _FTM1C0SC.Bits.CH0F
#define FTM1C0SC_ELS0x                  _FTM1C0SC.MergedBits.grpELS0x
#define FTM1C0SC_MS0x                   _FTM1C0SC.MergedBits.grpMS0x

#define FTM1C0SC_ELS0A_MASK             0x04
#define FTM1C0SC_ELS0B_MASK             0x08
#define FTM1C0SC_MS0A_MASK              0x10
#define FTM1C0SC_MS0B_MASK              0x20
#define FTM1C0SC_CH0IE_MASK             0x40
#define FTM1C0SC_CH0F_MASK              0x80
#define FTM1C0SC_ELS0x_MASK             0x0C
#define FTM1C0SC_ELS0x_BITNUM           0x02
#define FTM1C0SC_MS0x_MASK              0x30
#define FTM1C0SC_MS0x_BITNUM            0x04


/*** FTM1C0V - FTM1 Timer Channel 0 Value Register; 0x40039010 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** FTM1C0VH - FTM1 Timer Channel 0 Value Register High; 0x40039010 ***/
    union {
      byte Byte;
    } FTM1C0VLSTR;
    #define FTM1C0VH                    _FTM1C0V.Overlap_STR.FTM1C0VHSTR.Byte
    

    /*** FTM1C0VL - FTM1 Timer Channel 0 Value Register Low; 0x40039011 ***/
    union {
      byte Byte;
    } FTM1C0VHSTR;
    #define FTM1C0VL                    _FTM1C0V.Overlap_STR.FTM1C0VLSTR.Byte
    byte Rsvd1;
    byte Rsvd2;    
  } Overlap_STR;

} FTM1C0VSTR;
//extern volatile FTM1C0VSTR _FTM1C0V @0x40039010;
#define _FTM1C0V 	(*(volatile FTM1C0VSTR*)0x40039010)

#define FTM1C0V                         _FTM1C0V.Word


/*********************************************************************
*
* Periodic Interrupt Timer (PIT)
*
*********************************************************************/

/* peripheral base addresses */

#define PIT_REGS_BASE  0x40037000  // PIT Starting Address



/***************** Module-Relative Register Offsets *************************/
// Defines the register map
#define  PIT_PITMCR	(*(vuint32_t*)(PIT_REGS_BASE + 0x0000))


#define  PIT_CH0_LDVAL	(*(vuint32_t*)(PIT_REGS_BASE + 0x0100))
#define  PIT_CH0_CVAL	(*(vuint32_t*)(PIT_REGS_BASE + 0x0104))
#define  PIT_CH0_TCTRL	(*(vuint32_t*)(PIT_REGS_BASE + 0x0108))
#define  PIT_CH0_TFLG	(*(vuint32_t*)(PIT_REGS_BASE + 0x010C))

#define  PIT_PITMCR_FRZ	(BIT0)	//(0x80000000)
#define  PIT_PITMCR_MDIS	(BIT1)	//(0x40000000)

// bits for TCTRL register
#define  PIT_TCTRL_TEN		(BIT0)	//(0x80000000)
#define  PIT_TCTRL_TIE		(BIT1)	//(0x40000000)

// bits for TFLG register
#define  PIT_TFLG_TIF		(BIT0)	//(0x80000000)

#endif /*P2IO_MAP_H_*/
