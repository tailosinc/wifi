/**
 * @file      rsi_wireless_copy.h
 * @version   1.2
 * @date      2013-Feb-12
 *
 * Copyright(C) Redpine Signals 2013
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief HEADER, WIRELESS COPY
 *
 * @section Description
 * This is the header file for things that are related to WIRELESS extensions.
 *
 * @Improvements
 *
 */

#ifndef _RSI_WIRELESS_COPY_H
#define _RSI_WIRELESS_COPY_H

/***************************** INCLUDES *****************************/
#if 1
/* To minimise problems in user space, I might remove those headers */
#include <linux/types.h>                /* for "caddr_t" et al          */
#include <linux/socket.h>               /* for "struct sockaddr" et al  */
#include <linux/if.h>                   /* for IFNAMSIZ and co... */
#else
#include <sys/types.h>
#include <net/if.h>
typedef __uint32_t __u32;
typedef __int32_t __s32;
typedef __uint16_t __u16;
typedef __int16_t __s16;
typedef __uint8_t __u8;
#endif

#define RSI_WSC_PRIV_IOCTLS     SIOCDEVPRIVATE
#define OID_WSC_GET_STATUS      RSI_WSC_PRIV_IOCTLS + 0x1
#define OID_WSC_BOOT_READ        RSI_WSC_PRIV_IOCTLS + 0x2
#define OID_WSC_BOOT_WRITE       RSI_WSC_PRIV_IOCTLS + 0x3
#define OID_WSC_BOOT_PING_WRITE  RSI_WSC_PRIV_IOCTLS + 0x4
#define OID_WSC_BOOT_PONG_WRITE  RSI_WSC_PRIV_IOCTLS + 0x5
#define OID_WSC_POWER_SAVE_ENABLE  RSI_WSC_PRIV_IOCTLS + 0x6
#define OID_WSC_WAKEUP  RSI_WSC_PRIV_IOCTLS + 0x7


/****************************** TYPES ******************************/

/*
 *      For all data larger than 16 octets, we need to use a
 *      pointer to memory allocated in user space. Here,
 *      for all the same is used.
 */
struct  iw_point
{
  void    *pointer;     //@ Pointer to the data  (in user space) 
  __u16    length;      //@ number of fields or size in bytes 
  __u16     flags;      //@ Optional params 
};


/* ------------------------ IOCTL REQUEST ------------------------ */
/*
 * This structure defines the payload of an ioctl, and is used
 * below.
 *
 * Note that this structure should fit on the memory footprint
 * of iwreq (which is the same as ifreq), which mean a max size of
 * 16 octets = 128 bits. Warning, pointers might be 64 bits wide...
 * You should check this when increasing the structures defined
 * above in this file...
 */
union   iwreq_data
{
  //! Config - generic 
  char            name[IFNAMSIZ];
  //! Name of the protocol/provider...
  struct iw_point data;           //! Other large parameters 
};

/*
 * The structure to exchange data for ioctl.
 * This structure is the same as 'struct ifreq', but (re)defined for
 * convenience...
 * Do I need to remind you about structure size (32 octets) ?
 */
struct  iwreq
{
  union
  {
    char    ifrn_name[IFNAMSIZ];    //! if name, e.g. "wlan0" 
  } ifr_ifrn;

  //! Data part (defined just above) 
  union   iwreq_data      u;
};

#endif  /* _RSI_WIRELESS_COPY_H */
