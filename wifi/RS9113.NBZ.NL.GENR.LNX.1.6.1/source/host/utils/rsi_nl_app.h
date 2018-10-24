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

typedef unsigned char   uint_8, uint8;
typedef char            int_8, int8;
typedef unsigned short  uint_16, uint16;
typedef short           int_16, int16;
typedef unsigned int    uint_32, uint32;
typedef int             int_32, int32;

#define rsi_free    free
#define rsi_malloc  malloc
#define rsi_memcpy  memcpy

#define RSI_FAMILYID "CTRL_PKT_TXRX"


/*
 * Generic macros for dealing with netlink sockets. Might be duplicated
 * elsewhere. It is recommended that commercial grade applications use
 * libnl or libnetlink and use the interfaces provided by the library
 */
#define GENLMSG_DATA(glh) ((void *)(NLMSG_DATA(glh) + GENL_HDRLEN))
#define GENLMSG_PAYLOAD(glh) (NLMSG_PAYLOAD(glh, 0) - GENL_HDRLEN)
#define NLA_DATA(na) ((void *)((char*)(na) + NLA_HDRLEN))
#define MAX_RCV_SIZE  1600
#define MAX_PACKET_LEN MAX_RCV_SIZE
//#define NLA_PAYLOAD(len) (len - NLA_HDRLEN)
#define RSI_NL_HEAD_SIZE         (sizeof(struct nlmsghdr) + sizeof(struct genlmsghdr) + sizeof(struct nlattr))
//#define RSI_STATUS_OFFSET         2
#define RSI_STATUS_OFFSET         12
#define RSI_TWOBYTE_STATUS_OFFSET 12
//#define RSI_RSP_TYPE_OFFSET       0
#define RSI_RSP_TYPE_OFFSET       2
#define GET_SEND_LENGTH(a) ((uint16 )(*(uint32 *)(a)))

/* User to Kernel Update Types */
enum {
  MODULE_POWER_CYCLE = 0x01,
  UPDATE_JOIN_DONE   = 0x02,
  PS_CONTINUE        = 0x03,
  WKP_FROM_HOST      = 0x04,
};


/* type defines */
typedef struct {
	struct nlmsghdr n;
	struct genlmsghdr g;
	char buf[2000];
} rsi_nlPkt_t;

struct rsi_nlpriv {
	int fd;
	int family_id;	
};

/* Function prototypes */
int rsi_netlink_init(void);
int rsi_netlink_deinit(void);
int rsi_send_to_drv(uint_8 *buf, uint_32 type, uint_32 len);
int rsi_recv_from_drv(uint_8 *buf, uint_32 *type, uint_32 *len);
int32 rsi_sendto_fd(int32 s, const uint_8 *buf, int32 bufLen);
int32 rsi_get_family_id(int32 sd);
int recv_data(uint_8 *);
#endif
