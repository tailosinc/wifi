/**
 * @file     rsi_lib_util.h
 * @version  2.7
 * @date     2012-Sep-26
 *
 * Copyright(C) Redpine Signals 2012
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief HEADER UTIL: Util Header file, the things that are used in the library 
 *
 * @section Description
 * This is the util.h file for the utility functions used by library.
 * Contains prototypes of utils used in rsi_lib_util.c
 *
 */


#ifndef _RSILIBUTIL_H_
#define _RSILIBUTIL_H_

#include "rsi_global.h"

void rsi_uint32_to_4bytes(uint8 *dBuf, uint32 val);
void rsi_uint16_to_2bytes(uint8 *dBuf, uint16 val);
uint16 rsi_bytes2R_to_uint16(uint8 *dBuf);
#endif
