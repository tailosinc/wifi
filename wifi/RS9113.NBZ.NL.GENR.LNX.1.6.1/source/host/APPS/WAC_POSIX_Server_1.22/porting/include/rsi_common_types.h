/**
 * @file         rsi_common_types.h
 * @version      1.0
 * @date         2015-Feb-17
 *
 * Copyright(C) Redpine Signals 2014
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief HEADER, APP, APPLICATION Header file which contains application specific structures 
 *
 * @section Description
 * This file contains the Application related information.
 *
 *
 */

/**
 * Includes
 * */

#ifndef __RSI_COMMON_TYPES_H
#define __RSI_COMMON_TYPES_H
typedef unsigned char	          UINT08;
typedef signed char             INT08;

#ifndef UINT8
typedef unsigned char           UINT8;
#endif
#ifndef INT8
typedef signed char             INT8;
#endif
#ifndef UINT16
typedef unsigned short int      UINT16;
#endif
#ifndef INT16
typedef short                   INT16;
#endif
#ifndef UINT32
typedef unsigned int       UINT32;
#endif

#ifndef INT32
typedef int       			INT32;
#endif

typedef long                    SINT32;
typedef long                    SINT_32;
typedef unsigned long long int  UINT64;
typedef long long  int          INT64;

#ifndef WINDOWS
typedef unsigned char           BOOL;
#endif

typedef unsigned char           uint8;
typedef unsigned short          uint16;
typedef unsigned int            uint32;
typedef signed char             int8;
typedef short                   int16;
typedef long                    int32;

//ZIgb Datatypes
typedef int sint32, sint_32;
typedef unsigned long long int uint64;
typedef long long  int int64;
typedef unsigned char RSI_ZB_STATUS;
#if (defined WINDOWS || defined LINUX_PLATFORM)
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef short int16_t;
typedef unsigned int uint32_t;
typedef int int32_t;
#endif
#ifdef LINUX_PLATFORM
typedef uint16_t profile_id_t;
typedef uint16_t cluster_id_t;
typedef uint16_t ProfileID;
typedef uint16_t ClusterID;
typedef uint16_t GroupID;
#endif

#endif
