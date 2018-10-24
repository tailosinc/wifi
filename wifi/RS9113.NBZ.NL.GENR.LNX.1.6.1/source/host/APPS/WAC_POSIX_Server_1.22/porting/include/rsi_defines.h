#ifndef _RSIDEFINES_H_
#define _RSIDEFINES_H_

/*
 * @ Type Definitions
 */
#if 0
typedef unsigned char     uint8;
typedef unsigned short    uint16;
typedef unsigned int      uint32;
typedef char              int8;
typedef short             int16;
typedef int               int32;
#endif

#define rsi_malloc(ptr)  malloc(ptr)
#define rsi_free(ptr)    free(ptr)

#endif //_RSIDEFINES_H_
