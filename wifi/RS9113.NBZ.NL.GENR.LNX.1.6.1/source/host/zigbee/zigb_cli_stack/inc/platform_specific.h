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
#include "rsi_common_types.h"
//#include "rsi_defines.h"
//#include "rsi_config.h"
//#include "rsi_spi_cmd.h"
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <malloc.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h> 
#include <termios.h>
#include <stdbool.h>
#include "pthread.h"

#define BAUDRATE B115200
#define RSI_PRE_DESC_LEN 4
//! Skip card ready if in UART mode
#define RSI_SKIP_CARD_READY      0

//! Some things are boolean values
#define RSI_TRUE        1
#define RSI_FALSE       0
//! Serial Types
#define UART         0
#define USB_CDC      1

#define RSI_RXPKT_HEAD_ROOM     16
#define rsi_malloc(a)           malloc(a)
#define rsi_free(a)             free(a)
#define rsi_memcpy(a, b, c)     memcpy(a, b, c)
#define rsi_memset(a, b, c)     memset(a, b, c)
//SIDDIQ
/*typedef struct rsi_rx_pkt_s
{
  //! next packet pointer 
  struct rsi_rx_pkt_s *next;

typedef struct pkt_struct_s
{
  struct pkt_struct_s *next;
  uint8_t  *data;
}pkt_struct_t;*/
/****************************************************/
typedef struct rsi_pkt_s
{
  //! next packet pointer 
  struct rsi_pkt_s *next;

  //! host descriptor 
  uint8_t desc[16];

  //! payload 
  uint8_t data[1];
}rsi_pkt_t;
/****************************************************/
//! Receive queue 
typedef struct
{
  rsi_pkt_t *head;                  //! queue head 
  rsi_pkt_t *tail;                  //! queue tail
  uint16_t pending_pkt_count;            //! pending packets in the queue
}pkt_queue_t;

//! RX Progrss States
typedef enum rsi_uart_rx_state_e{
  //! 0 - IDLE  state
  RSI_UART_IDLE_STATE = 0,

  //! Pre descriptor receive state
  RSI_UART_LEN_DESC_RECV_STATE,

  //! Host descriptor receive state
  RSI_UART_WAIT_FOR_HOST_DESC_STATE,

  //! Payload receive state
  RSI_UART_PAYLOAD_RECV_STATE
} rsi_uart_rx_state_t;
/****************************************************/

//! Application control block 
typedef struct {
  int32_t                nl_sd;          //! netlink socket descriptor 
  int32_t                ioctl_sd;       //! socket descriptor of ioctl 
  int32_t                family_id;      //! family id 
  uint8_t                rsi_glbl_genl_nl_hdr[20];
  uint8_t                mac_addr[6];
  uint32_t               num_rcvd_packets;
  pthread_mutex_t      mutex1;
  
  uint8_t                pending_cmd_type;

  volatile int16_t       rsi_status;
  uint32_t               interrupt_rcvd;
  pkt_queue_t          rcv_queue;
  //! socket desc for serial device
  int                  ttyfd;        
#ifdef WLAN_ENABLE
  volatile uint8_t       wfd_dev_found;
  rsi_wfdDevResponse   wfdDevData;
  uint8_t                udp_sock_flag;
  //! for set certificate
  uint8_t                cert_morechunks;
  struct SET_CHUNK_S   set_chunks;
  //! for webpage write 
  uint8_t                webpage_morechunks;
  //! for power save
  rsi_powerstate       rsi_pwstate;
#endif
  //! Pre descriptor read 
  //uint8_t               pre_desc_buf[RSI_PRE_DESC_LEN];//reyaz
  //! RX packet allocated
  rsi_pkt_t             *rcvPktPtr;
  //! RX receive state 
  rsi_uart_rx_state_t   rx_is_in_progress;
  
  //! payload size of the packet
  uint16_t              payload_size;
  //! byte count of received bytes
  uint16_t              byte_count; 
  UINT8 	       serial_type; 
}rsi_linux_app_cb_t;

extern rsi_linux_app_cb_t rsi_linux_app_cb;
int16_t rsi_register_interrupt_irq(void);
#ifdef WLAN_ENABLE
#ifdef LINUX_PLATFORM
void measure_throughput(uint32_t  pkt_length, uint32_t tx_rx);
int32_t rsi_ioctl_send_req(int32_t sockfd, uint8_t *buf, int32_t buf_len, int32_t req_type);
#endif
#endif
#endif
