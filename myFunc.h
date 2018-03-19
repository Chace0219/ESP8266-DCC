

/**********************************************************************

Comm.h
COPYRIGHT (c) 2013-2016 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/
#ifndef myFunc_h
#define myFunc_h
#include "stdint.h"
#include "stdarg.h"

typedef unsigned int size_t;
typedef int ptrdiff_t;

#define UINT_MAX 4294967295U
/* Using an integer's bits as flags for both the conversion flags and length
modifiers.
*/
#define E_suppressed 1<<0
#define E_char       1<<6
#define E_short      1<<7
#define E_long       1<<8
#define E_llong      1<<9
#define E_intmax     1<<10
#define E_size       1<<11
#define E_ptrdiff    1<<12
#define E_intptr     1<<13
#define E_ldouble    1<<14
#define E_unsigned   1<<16
#define E_float      1<<17
#define E_double     1<<18

#define SIZE_MAX 500

#define EOF -1
#define NULL 0
#define bool int
#define true 1
#define false 0

/* Status structure required by _PDCLIB_print(). */
struct _PDCLIB_status_t
{
	/* XXX This structure is horrible now. scanf needs its own */

	int              base;   /* base to which the value shall be converted   */
	unsigned int flags; /* flags and length modifiers                */
	unsigned         n;      /* print: maximum characters to be written (snprintf) */
							 /* scan:  number matched conversion specifiers  */
	unsigned         i;      /* number of characters read/written            */
	unsigned         current;/* chars read/written in the CURRENT conversion */
	unsigned         width;  /* specified field width                        */
	int              prec;   /* specified field precision                    */

	const char *     s;      /* input string for scanf */

	va_list  arg;    /* argument stack                               */
};


void * nmemchr(const void * s, int c, size_t n);


/* Helper function to get a character from the string or stream, whatever is
used for input. When reading from a string, returns EOF on end-of-string
so that handling of the return value can be uniform for both streams and
strings.
*/
static int GET(struct _PDCLIB_status_t * status);

/* Helper function to put a read character back into the string or stream,
whatever is used for input.
*/
static void UNGET(int c, struct _PDCLIB_status_t * status);

/* Helper function to check if a character is part of a given scanset */
static bool IN_SCANSET(const char * scanlist, const char * end_scanlist, int rc);

const char * _PDCLIB_scan(const char * spec, struct _PDCLIB_status_t * status);

int nsscanf(const char *s, const char *format, ...);

struct timerflashing
{
    unsigned IN: 1; // IN option
    unsigned Q: 1; // Output
    unsigned long PT1; // Set Timeout
    unsigned long PT2; // Set Timeout
    unsigned long ET; // Elapsed time
};
typedef struct timerflashing TFLASH;

struct tonblock
{
    unsigned IN: 1; // IN option
    unsigned Q: 1; // Output
    unsigned long PT; // Set Timeout
    unsigned long ET; // Elapsed time
};
typedef struct tonblock TON;

struct RisingTrg
{
    unsigned IN : 1;
    unsigned PRE : 1;
    unsigned Q : 1;
    unsigned : 5;
};
typedef struct RisingTrg Rtrg;

// When any condition is true, it returns true
void TFLASHFunc(TFLASH *pTP);

// When any condition is true, it returns true
void TONFunc(TON *pTP);

// It should be used with TONFunc together
void RTrgFunc(Rtrg *pTrg);

#endif
