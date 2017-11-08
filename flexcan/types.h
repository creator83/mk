
#ifndef __TYPES_H
#define __TYPES_H

#ifndef TRUE
  #define TRUE  1
#endif

#ifndef FALSE
  #define FALSE  0
#endif

#ifndef NULL
  #define NULL  0
#endif

typedef unsigned long	uint_32;
typedef unsigned short  uint_16;
typedef unsigned char   uint_8;
typedef unsigned char   uchar;

typedef void *		vpointer;
typedef uint_16		PSP_INTERRUPT_TABLE_INDEX;
typedef uint_16		_int_level;
typedef uint_16		_int_priority;
typedef uint_8		boolean;

typedef uint_32	* uint_32_ptr;
typedef uint_8	    * uchar_ptr;
typedef volatile  uint_16	vuint_16;
typedef volatile  uint_32	vuint_32;

typedef volatile  uint_8	vuint8_t;
typedef volatile  uint_16	vuint16_t;
typedef volatile  uint_32	vuint32_t;

typedef unsigned char byte;
typedef unsigned long word;

typedef unsigned long uint32;
typedef unsigned short uint16;
typedef unsigned char uint8;


#endif /* __TYPES_H */
