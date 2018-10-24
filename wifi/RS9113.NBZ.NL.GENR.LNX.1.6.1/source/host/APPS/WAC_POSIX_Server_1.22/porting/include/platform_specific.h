/**
 * @file     platform_specific.h
 * @version  3.6
 * @date     2013-May-16
 *
 * Copyright(C) Redpine Signals 2013
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief HEADER ,PLATFORM SPECIFIC
 *
 * @section Description
 * This file contains the API's prototypes and defines used on linux platform
 *
 *
 */

#ifndef __PLATFORM_SPECIFIC_H_
#define __PLATFORM_SPECIFIC_H_
#ifdef WLAN_ENABLE
#include "rsi_global.h"
#endif
#include "rsi_defines.h"
#include "rsi_config.h"
#ifdef SPI_INTERFACE
#include "rsi_spi_cmd.h"
#endif
#include <stdio.h>
#include <string.h>
#include <pthread.h>

//! Some things are boolean values
#define RSI_TRUE        1
#define RSI_FALSE       0
#define RSI_RXPKT_HEAD_ROOM     16
#define RSI_DPRINT(lvl, fmt, args...)              if (lvl & RSI_DEBUG_LVL) printf(fmt, ##args)

typedef struct pkt_struct_s
{
  struct pkt_struct_s *next;
  uint8  *data;
}pkt_struct_t;

//! Receive queue 
typedef struct
{
  pkt_struct_t *head;                  //! queue head 
  pkt_struct_t *tail;                  //! queue tail
  uint16 pending_pkt_count;            //! pending packets in the queue
}pkt_queue_t;

//! Application control block 
typedef struct {
  int32                nl_sd;          //! netlink socket descriptor 
  int32                ioctl_sd;       //! socket descriptor of ioctl 
  int32                family_id;      //! family id 
  uint8                rsi_glbl_genl_nl_hdr[20];
  uint8                mac_addr[6];
  uint32               num_rcvd_packets;
  pthread_mutex_t      mutex1;
  
  uint8                pending_cmd_type;

  volatile int16       rsi_status;
  uint32               interrupt_rcvd;
  pkt_queue_t          rcv_queue;
  //! socket desc for serial device
  int                  ttyfd;        
#ifdef WLAN_ENABLE
  volatile uint8       wfd_dev_found;
  rsi_wfdDevResponse   wfdDevData;
  uint8                udp_sock_flag;
  //! for set certificate
  uint8                cert_morechunks;
  struct SET_CHUNK_S   set_chunks;
  //! for webpage write 
  uint8                webpage_morechunks;
  //! for power save
  rsi_powerstate       rsi_pwstate;
#endif  
}rsi_linux_app_cb_t;

extern rsi_linux_app_cb_t rsi_linux_app_cb;
int16 rsi_register_interrupt_irq(void);
#ifdef WLAN_ENABLE
#ifdef LINUX_PLATFORM
void measure_throughput(uint32  pkt_length, uint32 tx_rx);
int32 rsi_ioctl_send_req(int32 sockfd, uint8 *buf, int32 buf_len, int32 req_type);
#endif
#endif
#endif
