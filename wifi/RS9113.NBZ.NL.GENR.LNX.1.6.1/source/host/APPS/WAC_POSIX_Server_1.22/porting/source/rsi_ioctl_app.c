
/**
 * @file      rsi_ioctl_app.c
 * @version   1.3
 * @date      2013-Feb-28
 *
 * Copyright(C) Redpine Signals 2013
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief SPI, UPDATE INFO, sends any info needed for kernel as a netlink message
 *
 * @section Description
 * This file contains the functions to send any info needed for kernel as a
 * netlink message and the functions to get the info needed from kernel through
 * an ioctl request.
 *
 */

/**
 * Includes
 */
#include "rsi_global.h"
#include "rsi_app.h"
#include "rsi_nl_app.h"
#include "rsi_wireless_copy.h"

#include <sys/ioctl.h>
#include "fcntl.h"

/**
 * Global Variables
 */


/*==============================================*/
/**
 * @fn             int32 rsi_ioctl_send_req(int32 sockfd,
 *                                          uint8 *buf,
 *                                          int32 buf_len,
 *                                          int32 req_type)
 * @brief          update the info to kernel
 * @param[in]      int32 sockfd, socket descriptor
 * @param[in]      uint8 *buf, pointer to the buffer
 * @param[in]      int32 buf_len, length of the buffer
 * @param[in]      int32 req_type, request type
 * @param[out]     none
 * @return         errCode
 *                 0  = SUCCESS
 *                 else Failure
 * @section description
 * This function is used to send some info which is required
 * by the kernel netdevice driver.
 */

int32 rsi_ioctl_send_req(int32 sockfd, uint8 *buf, int32 buf_len, int32 req_type)
{
  struct iwreq iwr;
  int32 val;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_ioctl_send_req:\n");
#endif
  memset(&iwr, 0, sizeof(iwr));
  strncpy(iwr.ifr_name, "rsi_wlan0", IFNAMSIZ);
  iwr.u.data.pointer= buf;
  iwr.u.data.length= buf_len;
  while((val=ioctl(sockfd, req_type, &iwr))<0)
  {
#ifdef RSI_DEBUG_PRINT
    RSI_DPRINT(RSI_PL0,"unable to issue ioctl for %lx request %ld reason %d\n",req_type,val,(errno));
#endif
    return val;
  }
  return 0;
}
