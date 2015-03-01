/*******************************************************************************
              Copyright 2011 SDyP SA, All Rights Reserved
 *******************************************************************************
 ***************************************************************************//**

  @file     common.h

  @brief    General and common definitions, available for all modules

  @author   Nicolas Magliola

  @date     29/Sep/2011

  @version  1.0

 ******************************************************************************/

#ifndef _COMMON_H_
#define _COMMON_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

//#include "system_config.h"

#ifdef  __IAR_SYSTEMS_ICC__

#include <inavr.h>

#else

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <inttypes.h>

#endif


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define MILLISECONDS_PER_SECOND        1000L
#define MICROSECONDS_PER_SECOND        1000000L
#define SECONDS_PER_MINUTE             60L

#ifndef POSITIVE
enum {POSITIVE, NEGATIVE};
#endif // POSITIVE

#ifndef FALSE
enum {FALSE, TRUE};
#endif // FALSE


#define error_exception(error_id)

#define boolean(a)                  ( (a)? 1 : 0 )
#define BITMSK(nbit)                (1<<(nbit))
#define BITSET(var, nbit)           ((var) |= 1<<(nbit))
#define BITCLR(var, nbit)           ((var) &= ~(1<<(nbit)))
#define BITCHK(var, nbit)           boolean((var) & (1<<(nbit)))
#define SET_BIT(port,nbit,value) 	((value)?(port=port|(1<<(nbit))):(port=port & ~(1<<(nbit))))
#define TOGGLE_BIT(port,nbit)       ((port)^=(1<<nbit))

#define num_elementos(array)        (sizeof(array)/sizeof((array)[0]))

#define INC_MOD(v, m)               if (++(v) >= (m)) (v)=0

#ifdef  __IAR_SYSTEMS_ICC__

#define _enable_interrupts()     __enable_interrupt()
#define _disable_interrupts()    __disable_interrupt()

#else

#define _enable_interrupts()     sei()
#define _disable_interrupts()    cli()

#endif


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Variable types definition
typedef union {
   int word;
   struct {
      char low;
      char high;
   }bytes;
} doublebyte;

typedef unsigned char   uint8;
typedef signed char     int8;
typedef unsigned int    uint16;
typedef signed int      int16;
typedef unsigned long   uint32;
typedef signed long     int32;

// void-void function pointer definition
typedef void (*void_fptr_void) (void);


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/



/*******************************************************************************
 ******************************************************************************/
#endif // _COMMON_H_

/*******************************************************************************
 * REVISION RECORDS
 *******************************************************************************
 *
 * 1.0  Initial Release
 *
 ******************************************************************************/
