/**
 * @file     rsi_nl_app.h
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
 * @brief HEADER  
 *
 * @section Description
 * 
 *
 */


#ifndef __RSI_GNUSER_H_
#define __RSI_GNUSER_H_

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <signal.h>
#include <linux/genetlink.h>

#include "rsi_defines.h"

/*
 * Generic macros for dealing with netlink sockets. Might be duplicated
 * elsewhere. It is recommended that commercial grade applications use
 * libnl or libnetlink and use the interfaces provided by the library
 */
#define GENLMSG_DATA(glh) ((void *)(NLMSG_DATA(glh) + GENL_HDRLEN))
#define GENLMSG_PAYLOAD(glh) (NLMSG_PAYLOAD(glh, 0) - GENL_HDRLEN)
#define NLA_DATA(na) ((void *)((char*)(na) + NLA_HDRLEN))
#define MAX_RCV_SIZE  1600
#define RSI_NL_HEAD_SIZE         (sizeof(struct nlmsghdr) + sizeof(struct genlmsghdr) + sizeof(struct nlattr))
#define RSI_STATUS_OFFSET         12
#define RSI_TWOBYTE_STATUS_OFFSET 12
#define RSI_RSP_TYPE_OFFSET       2
#define GET_SEND_LENGTH(a) ((uint16 )(*(uint32 *)(a)))

/* User to Kernel Update Types */
enum {
  MODULE_POWER_CYCLE               = 0x01,
  UPDATE_JOIN_DONE                 = 0x02,
  PS_CONTINUE                      = 0x03,
  WKP_FROM_HOST                    = 0x04,
  UPDATE_CONCURRENT_AP_JOIN_DONE   = 0x05,
};


/* type defines */
typedef struct {
  struct nlmsghdr n;
  struct genlmsghdr g;
} rsi_nlPkt_t;

/* Function prototypes */
int32 rsi_nl_socket_init(void);
void rsi_fill_genl_nl_hdrs_for_cmd(void);
uint8 *rsi_alloc_and_init_cmdbuff(uint8 *Desc, uint8 *payload, uint16 payload_size);
int16 rsi_send_usr_cmd(uint8 *buff, uint16 bufLen);

int32 rsi_sendto_fd(int32 s, const uint8 *buf, int32 bufLen);
int32 rsi_get_family_id(int32 sd);
void rsi_enqueue_to_rcv_q(pkt_struct_t *Pkt);
pkt_struct_t *rsi_dequeue_from_rcv_q(void);
void * RecvThreadBody(void * arg );
uint8 *rsi_wrapper_to_rsp(uint8 *rsp, uint8 rsp_type);
int16 rsi_update_info(uint8 type);

#endif //__RSI_GNUSER_H_
