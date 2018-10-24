/*
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	1. Redistributions of source code must retain the above copyright
 * 	   notice, this list of conditions and the following disclaimer.
 *
 * 	2. Redistributions in binary form must reproduce the above copyright
 * 	   notice, this list of conditions and the following disclaimer in the
 * 	   documentation and/or other materials provided with the distribution.
 *
 * 	3. Neither the name of the copyright holder nor the names of its
 * 	   contributors may be used to endorse or promote products derived from
 * 	   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _NET_IF_NDIFACE_H_
#define _NET_IF_NDIFACE_H_

#include <linux/netdevice.h>
#include <linux/version.h>
#include "common.h"
#include "mbuf.h"

#define ifnet net_device

#define if_xname name
#define if_dname name
#define if_type  type
#define if_addrlen addr_len
#define if_hdrlen hard_header_len
#define if_drv_flags  priv_flags
#define if_broadcastaddr broadcast
#define if_flags flags

#if KERNEL_VERSION_BTWN_2_6_(18,30)
#define if_ioctl do_ioctl
#endif

#define if_watchdog  tx_timeout
#if KERNEL_VERSION_BTWN_2_6_(18,30)
#define if_init   open 
#endif
#define if_mtu   mtu
#define if_resolvemulti   set_multicast_list 
#define ifmultihead     dev_mc_list
#define if_multiaddrs   mc_list /* multicast mac addresses */
#define if_addr  dev_addr 
#define if_start ieee80211_start
#define if_stop  stop

#define IFF_SIMPLEX   0x800      /* (i) can't hear own transmissions */
#ifndef IFF_MULTICAST
#define IFF_MULTICAST 0x1000 /* (i) supports multicast */
#endif
#define ETH_PROTOCOL_OFFSET             12
#define ETH_HDR_OFFSET                  0
#define ETH_P_80211_RAW  0x0019

#ifndef ETHERTYPE_IP
#define ETHERTYPE_IP   0x0800
#endif

#ifndef ETHERTYPE_IPV6
#define	ETHERTYPE_IPV6		0x86dd		/* IP protocol version 6 */
#endif

#define BPF_MTAP(arg1,arg2)                 bpf_mtap(ifp,m)
#define  IFQ_SET_MAXLEN(ifq,len)      ((ifq)->ifq_maxlen = (len))
#define  IFQ_SET_READY(arg1)            null_function()
#define  ether_ifdetach(arg1)           null_function()
#define  if_clone_attach(arg1)          null_function()
#define  if_clone_detach(arg1)          null_function()
#define  if_clone_destroyif(arg1,arg2)  null_function()
#define if_register_com_alloc(arg1,arg2,arg3)  null_function()
#define if_deregister_com_alloc(arg1)  null_function()
#define IFC_SIMPLE_DECLARE(arg1,arg2)   null_function()

#define  ifaddr_byindex(arg1)                null_function()
#define  ifa_free(arg1)                      null_function()
#define if_link_state_change(arg1,arg2)      null_function()
#define if_setlladdr(arg1,arg2)              null_function()
#define IF_LLADDR(arg1)                      null_function()

#define IFQ_DEQUEUE     IF_DEQUEUE
#define IFNET_IS_UP_RUNNING(_ifp) \
        (((_ifp)->if_flags & IFF_UP) && \
         ((_ifp)->if_drv_flags & IFF_DRV_RUNNING))


#define rt_newmaddrmsg(arg1,arg2)        null_function()

#ifndef IF_PREPEND_LIST
#define _IF_PREPEND_LIST(ifq, mhead, mtail, mcount) do {\
                         (mtail)->m_nextpkt = (ifq)->ifq_head;\
                         if ((ifq)->ifq_tail == NULL)\
                             (ifq)->ifq_tail = (mtail);\
                         (ifq)->ifq_head = (mhead);\
                         (ifq)->ifq_len += (mcount);\
} while (0)
#define IF_PREPEND_LIST(ifq, mhead, mtail, mcount) do {\
	IF_LOCK(ifq);\
	_IF_PREPEND_LIST(ifq, mhead, mtail, mcount);\
	IF_UNLOCK(ifq);\
} while (0)
#endif /* IF_PREPEND_LIST */


/*
 * Output queues (ifp->if_snd) and slow device input queues (*ifp->if_slowq)
 * are queues of messages stored on ifqueue structures
 * (defined above).  Entries are added to and deleted from these structures
 * by these macros, which should be called with ipl raised to splimp().
 */
#define _IF_QFULL(ifq)  ((ifq)->ifq_len >= (ifq)->ifq_maxlen)
#define _IF_DROP(ifq)   ((ifq)->ifq_drops++)
#define _IF_QLEN(ifq)   ((ifq)->ifq_len)

#define IF_ENQUEUE(ifq, m) do {					\
	IF_LOCK(ifq); 						\
	_IF_ENQUEUE(ifq, m); 					\
	IF_UNLOCK(ifq); 					\
} while (0)

#define	_IF_PREPEND(ifq, m) do {				\
	(m)->m_nextpkt = (ifq)->ifq_head; 			\
	if ((ifq)->ifq_tail == NULL) 				\
		(ifq)->ifq_tail = (m); 				\
	(ifq)->ifq_head = (m); 					\
	(ifq)->ifq_len++; 					\
} while (0)

#define IF_PREPEND(ifq, m) do {		 			\
	IF_LOCK(ifq); 						\
	_IF_PREPEND(ifq, m); 					\
	IF_UNLOCK(ifq); 					\
} while (0)

#define	_IF_DEQUEUE(ifq, m) do { 				\
	(m) = (ifq)->ifq_head; 					\
	if (m) { 						\
		if (((ifq)->ifq_head = (m)->m_nextpkt) == NULL)	\
			(ifq)->ifq_tail = NULL; 		\
		(m)->m_nextpkt = NULL; 				\
		(ifq)->ifq_len--; 				\
	} 							\
} while (0)

#define IF_DEQUEUE(ifq, m) do { 				\
	IF_LOCK(ifq); 						\
	_IF_DEQUEUE(ifq, m); 					\
	IF_UNLOCK(ifq); 					\
} while (0)

#define	_IF_POLL(ifq, m)	((m) = (ifq)->ifq_head)
#define	IF_POLL(ifq, m)		_IF_POLL(ifq, m)

#define _IF_DRAIN(ifq) do { 					\
	struct mbuf *m; 					\
	for (;;) { 						\
		_IF_DEQUEUE(ifq, m); 				\
		if (m == NULL) 					\
			break; 					\
		m_freem(m); 					\
	} 							\
} while (0)

#define IF_DRAIN(ifq) do {					\
	IF_LOCK(ifq);						\
	_IF_DRAIN(ifq);						\
	IF_UNLOCK(ifq);						\
} while(0)

#define	_IF_ENQUEUE(ifq, m) do { 				\
	(m)->m_nextpkt = NULL;					\
	if ((ifq)->ifq_tail == NULL) 				\
		(ifq)->ifq_head = m; 				\
	else 							\
		(ifq)->ifq_tail->m_nextpkt = m; 		\
        (ifq)->ifq_tail = m; 					\
	(ifq)->ifq_len++; 					\
} while(0)


struct ifaddr 
{
	struct sockaddr *ifa_addr; /* address of interface */
	struct ifnet *ifa_ifp;     /* back-pointer to interface */

};
struct ifqueue 
{
	struct mbuf *ifq_head;
	struct mbuf *ifq_tail;
	int ifq_len;
	int ifq_maxlen;
	int ifq_drops;
	spinlock_t ifq_lock;
};

/*
 * Structure defining a queue for a network interface.
 */
struct ifaltq 
{
	/* fields compatible with struct ifqueue */
	struct mbuf *ifq_head;
	struct mbuf *ifq_tail;
	int ifq_len;
	int ifq_maxlen;
	int ifq_drops;
	spinlock_t ifq_lock;

	/* driver owned queue (used for bulk dequeue and prepend) UNLOCKED */
	struct mbuf *ifq_drv_head;
	struct mbuf *ifq_drv_tail;
	int ifq_drv_len;
	int ifq_drv_maxlen;

	/* alternate queueing related fields */
	int altq_type;  /* discipline type */
	int altq_flags; /* flags (e.g. ready, in-use) */
	void *altq_disc; /* for discipline-specific use */
	struct ifnet *altq_ifp; /* back pointer to interface */

#if 0
	int (*altq_enqueue)(struct ifaltq *, struct mbuf *,
	                    struct altq_pktattr *);
	struct mbuf *(*altq_dequeue)(struct ifaltq *, int);
	int (*altq_request)(struct ifaltq *, int, void *);

	/* classifier fields */
	void *altq_clfier; /* classifier-specific use */
	void *(*altq_classify)(void *, struct mbuf *, int);

	/* token bucket regulator */
	struct tb_regulator *altq_tbr;

	/* input traffic conditioner (doesn't belong to the output queue...) */
	struct top_cdnr *altq_cdnr;
#endif
};


/*
 * Structure of a Link-Level sockaddr:
 */
struct sockaddr_dl 
{
	u_char  sdl_len;   /* Total length of sockaddr */
	u_char  sdl_family;/* AF_LINK */
	u_short sdl_index; /* if != 0, system given index for interface */
	u_char  sdl_type;  /* interface type */
	u_char  sdl_nlen;  /* interface name length, no trailing 0 reqd. */
	u_char  sdl_alen;  /* link level address length */
	u_char  sdl_slen;  /* link layer selector length */
	char    sdl_data[46];/* minimum work area, can be larger;
	                           contains both if name and ll address */
};

/*
 * Multicast address structure.  This is analogous to the ifaddr
 * structure except that it keeps track of multicast addresses.
 */
struct ifmultiaddr 
{
	TAILQ_ENTRY(ifmultiaddr) ifma_link; /* queue macro glue */
	struct sockaddr *ifma_addr;         /* address this membership is for */
	struct sockaddr *ifma_lladdr;       /* link-layer translation, if any */
	struct ifnet *ifma_ifp;             /* back-pointer to interface */
	u_int  ifma_refcount;               /* reference count */
	void   *ifma_protospec;             /* protocol-specific state, if any */
	struct ifmultiaddr *ifma_llifma;    /* pointer to ifma for ifma_lladdr */
};

struct ifmediareq 
{
	char ifm_name[IFNAMSIZ];        /* if name, e.g. "en0" */
	int ifm_current;                        /* current media options */
	int ifm_mask;                   /* don't care mask */
	int ifm_status;                 /* media status */
	int ifm_active;                 /* active options */
	int ifm_count;                  /* # entries in ifm_ulist array */
	int __user *ifm_ulist;          /* media words */
};

struct ieee80211vap;
#define bpf_if ieee80211vap

#if 0
/*
 * Descriptor associated with each attached hardware interface.
 */
struct bpf_if 
{
	LIST_ENTRY(bpf_if)      bif_next;       /* list of all interfaces */
	LIST_HEAD_DECLARE(, bpf_d)      bif_dlist;      /* descriptor list */
	u_int bif_dlt;                          /* link layer type */
	u_int bif_hdrlen;               /* length of header (with padding) */
	struct ifnet *bif_ifp;          /* corresponding interface */
	struct mutex bif_mtx;        /* mutex for interface */
};
#endif

struct ip 
{

//#if BYTE_ORDER == LITTLE_ENDIAN
	u_int ip_hl:4,/* header length */
	      ip_v:4; /* version */
//#endif
//#if BYTE_ORDER == BIG_ENDIAN
//u_int ip_v:4, /* version */
//ip_hl:4;/* header length */
//#endif
	u_char  ip_tos;/* type of service */
	u_short ip_len;/* total length */
	u_short ip_id; /* identification */
	u_short ip_off;/* fragment offset field */
#define IP_RF 0x8000/* reserved fragment flag */
#define IP_DF 0x4000/* dont fragment flag */
#define IP_MF 0x2000/* more fragments flag */
#define IP_OFFMASK 0x1fff/* mask for fragmenting bits */
	u_char ip_ttl;/* time to live */
	u_char ip_p;  /* protocol */
	u_short ip_sum;/* checksum */
	struct in_addr ip_src,ip_dst;/* source and dest address */
};

#ifdef INET6
struct ip6_hdr
{
	union
	{
		struct ip6_hdrctl
		{
			uint32_t ip6_un1_flow;   /* 4 bits version, 8 bits TC,
										20 bits flow-ID */
			uint16_t ip6_un1_plen;   /* payload length */
			uint8_t  ip6_un1_nxt;    /* next header */
			uint8_t  ip6_un1_hlim;   /* hop limit */
		} ip6_un1;
		uint8_t ip6_un2_vfc;       /* 4 bits version, top 4 bits tclass */
	} ip6_ctlun;
	struct in6_addr ip6_src;      /* source address */
	struct in6_addr ip6_dst;      /* destination address */
};

#define ip6_vfc   ip6_ctlun.ip6_un2_vfc
#define ip6_flow  ip6_ctlun.ip6_un1.ip6_un1_flow
#define ip6_plen  ip6_ctlun.ip6_un1.ip6_un1_plen
#define ip6_nxt   ip6_ctlun.ip6_un1.ip6_un1_nxt
#define ip6_hlim  ip6_ctlun.ip6_un1.ip6_un1_hlim
#define ip6_hops  ip6_ctlun.ip6_un1.ip6_un1_hlim

#endif


/* Function Prototypes */
void bpf_mtap(struct ifnet *ifp, struct mbuf *m);
void bpf_mtap2(struct bpf_if *bp, void *d,int l,struct mbuf *m, int tx_path);
int ether_ioctl(struct ifnet *ifp,unsigned long command, struct ifreq* data);
void if_initname(struct ifnet *ifp,const char *name,int unit);
void set_tx_queue_len(struct ifnet *ifp,int max_len);
void if_attach(struct ifnet *ifp);
void if_detach(struct ifnet *ifp);
void if_start(struct ifnet *ifp);
int ether_ifattach(struct ifnet *ifp, const u_int8_t *lla);
void if_free(struct ifnet *ifp, uint8_t wait_for_lock);
struct ifnet * ifunit(const char *name);
void if_delallmulti(struct ifnet *ifp);
int if_delmulti_locked(struct ifnet *ifp,
            struct ifmultiaddr *ifma, int detaching);
void if_freemulti(struct ifmultiaddr *ifma);

#ifdef __KERNEL__

/*
 * Driver callbacks for media status and change requests.
 */
struct net_device;
typedef int (*ifm_change_cb_t)(struct net_device *);
typedef void (*ifm_stat_cb_t)(struct net_device *, struct ifmediareq *);

/*
 * In-kernel representation of a single supported media type.
 */
struct ifmedia_entry 
{
	LIST_ENTRY(ifmedia_entry) ifm_list;
	int ifm_media;          /* description of this media attachment */
	int ifm_data;           /* for driver-specific use */
	void *ifm_aux;          /* for driver-specific use */
};

/*
 * One of these goes into a network interface's softc structure.
 * It is used to keep general media state.
 */
struct ifmedia 
{
	int ifm_mask;                   /* mask of changes we don't care about */
	int ifm_media;                  /* current user-set media word */
	struct ifmedia_entry *ifm_cur;  /* currently selected media */
	LIST_HEAD_DECLARE(, ifmedia_entry) ifm_list; /* list of all supported media */
	ifm_change_cb_t ifm_change;     /* media change driver callback */
	ifm_stat_cb_t ifm_status;       /* media status driver callback */
};

/* Initialize an interface's struct if_media field. */
void ifmedia_init(struct ifmedia *, int, ifm_change_cb_t, ifm_stat_cb_t);

/* Remove all mediums from a struct ifmedia.  */
void ifmedia_removeall(struct ifmedia *);

/* Add one supported medium to a struct ifmedia. */
void ifmedia_add(struct ifmedia *, int, int, void *);

/* Add an array (of ifmedia_entry) media to a struct ifmedia. */
void ifmedia_list_add(struct ifmedia *, struct ifmedia_entry *, int);

/* Set default media type on initialization. */
void ifmedia_set(struct ifmedia *, int);

/* Common ioctl function for getting/setting media, called by driver. */
int ifmedia_ioctl(struct net_device *, struct ifreq *, struct ifmedia *, u_long);
struct net_device* if_alloc(int type);
int if_transmit_pkt(struct ifnet *ifp, struct mbuf *m);
void indicate_pkt_to_os(struct ifnet *ifp, struct mbuf *m);
uint16_t onebox_type_trans(struct sk_buff *skb, struct net_device *dev);
#endif /*_KERNEL */

int send_data_pkt_to_drv(struct ifnet *ifp, struct mbuf *m, struct ieee80211_node *ni);
/*
 * if_media Options word:
 *      Bits    Use
 *      ----    -------
 *      0-4     Media variant
 *      5-7     Media type
 *      8-15    Type specific options
 *      16-18   Mode (for multi-mode devices)
 *      19      RFU
 *      20-27   Shared (global) options
 *      28-31   Instance
 */

/*
 * Ethernet
 */
#define IFM_ETHER       0x00000020
#define IFM_10_T        3               /* 10BaseT - RJ45 */
#define IFM_10_2        4               /* 10Base2 - Thinnet */
#define IFM_10_5        5               /* 10Base5 - AUI */
#define IFM_100_TX      6               /* 100BaseTX - RJ45 */
#define IFM_100_FX      7               /* 100BaseFX - Fiber */
#define IFM_100_T4      8               /* 100BaseT4 - 4 pair cat 3 */
#define IFM_100_VG      9               /* 100VG-AnyLAN */
#define IFM_100_T2      10              /* 100BaseT2 */
#define IFM_1000_SX     11              /* 1000BaseSX - multi-mode fiber */
#define IFM_10_STP      12              /* 10BaseT over shielded TP */
#define IFM_10_FL       13              /* 10BaseFL - Fiber */
#define IFM_1000_LX     14              /* 1000baseLX - single-mode fiber */
#define IFM_1000_CX     15              /* 1000baseCX - 150ohm STP */
#define IFM_1000_T      16              /* 1000baseT - 4 pair cat 5 */
#define IFM_HPNA_1      17              /* HomePNA 1.0 (1Mb/s) */
/* note 31 is the max! */

#define IFM_ETH_MASTER  0x00000100      /* master mode (1000baseT) */

/*
 * Token ring
 */
#define IFM_TOKEN       0x00000040
#define IFM_TOK_STP4    3               /* Shielded twisted pair 4m - DB9 */
#define IFM_TOK_STP16   4               /* Shielded twisted pair 16m - DB9 */
#define IFM_TOK_UTP4    5               /* Unshielded twisted pair 4m - RJ45 */
#define IFM_TOK_UTP16   6               /* Unshielded twisted pair 16m - RJ45 */
#define IFM_TOK_STP100  7               /* Shielded twisted pair 100m - DB9 */
#define IFM_TOK_UTP100  8               /* Unshielded twisted pair 100m - RJ45 */
#define IFM_TOK_ETR     0x00000200      /* Early token release */
#define IFM_TOK_SRCRT   0x00000400      /* Enable source routing features */
#define IFM_TOK_ALLR    0x00000800      /* All routes / Single route bcast */
#define IFM_TOK_DTR     0x00002000      /* Dedicated token ring */
#define IFM_TOK_CLASSIC 0x00004000      /* Classic token ring */
#define IFM_TOK_AUTO    0x00008000      /* Automatic Dedicate/Classic token ring */

/*
 * FDDI
 */
#define IFM_FDDI        0x00000060
#define IFM_FDDI_SMF    3               /* Single-mode fiber */
#define IFM_FDDI_MMF    4               /* Multi-mode fiber */
#define IFM_FDDI_UTP    5               /* CDDI / UTP */
#define IFM_FDDI_DA     0x00000100      /* Dual attach / single attach */

/*
 * IEEE 802.11 Wireless
 */
#define IFM_IEEE80211   0x00000080
/* NB: 0,1,2 are auto, manual, none defined below */
#define IFM_IEEE80211_FH1       3       /* Frequency Hopping 1Mbps */
#define IFM_IEEE80211_FH2       4       /* Frequency Hopping 2Mbps */
#define IFM_IEEE80211_DS1       5       /* Direct Sequence 1Mbps */
#define IFM_IEEE80211_DS2       6       /* Direct Sequence 2Mbps */
#define IFM_IEEE80211_DS5       7       /* Direct Sequence 5.5Mbps */
#define IFM_IEEE80211_DS11      8       /* Direct Sequence 11Mbps */
#define IFM_IEEE80211_DS22      9       /* Direct Sequence 22Mbps */
#define IFM_IEEE80211_MCS0      24      /* HT MCS rate */
#define IFM_IEEE80211_MCS1      25      /* HT MCS rate */
#define IFM_IEEE80211_MCS2      26      /* HT MCS rate */
#define IFM_IEEE80211_MCS3      27      /* HT MCS rate */
#define IFM_IEEE80211_MCS4      28      /* HT MCS rate */
#define IFM_IEEE80211_MCS5      29      /* HT MCS rate */
#define IFM_IEEE80211_MCS6      30      /* HT MCS rate */
#define IFM_IEEE80211_MCS7      31      /* HT MCS rate */
#define IFM_IEEE80211_MCS8      32      /* HT MCS rate */
#define IFM_IEEE80211_MCS9      33      /* HT MCS rate */
#define IFM_IEEE80211_MCS10     34      /* HT MCS rate */
#define IFM_IEEE80211_MCS11     35      /* HT MCS rate */
#define IFM_IEEE80211_MCS12     36      /* HT MCS rate */
#define IFM_IEEE80211_MCS13     37      /* HT MCS rate */
#define IFM_IEEE80211_MCS14     38      /* HT MCS rate */
#define IFM_IEEE80211_MCS15     39      /* HT MCS rate */
#define IFM_IEEE80211_MCS16     40     /* HT MCS rate */
#define IFM_IEEE80211_MCS17     41      /* HT MCS rate */
#define IFM_IEEE80211_MCS18     42      /* HT MCS rate */
#define IFM_IEEE80211_MCS19     43      /* HT MCS rate */
#define IFM_IEEE80211_MCS20     44      /* HT MCS rate */
#define IFM_IEEE80211_MCS21     45      /* HT MCS rate */
#define IFM_IEEE80211_MCS22     46      /* HT MCS rate */
#define IFM_IEEE80211_MCS23     47      /* HT MCS rate */
#define IFM_IEEE80211_MCS24     48      /* HT MCS rate */
#define IFM_IEEE80211_MCS25     49      /* HT MCS rate */
#define IFM_IEEE80211_MCS26     50      /* HT MCS rate */
#define IFM_IEEE80211_MCS27     51      /* HT MCS rate */
#define IFM_IEEE80211_MCS28     52      /* HT MCS rate */
#define IFM_IEEE80211_MCS29     53      /* HT MCS rate */
#define IFM_IEEE80211_MCS30     54      /* HT MCS rate */
#define IFM_IEEE80211_MCS31     55      /* HT MCS rate */
#define IFM_IEEE80211_MCS32     56      /* HT MCS rate */
#define IFM_IEEE80211_MCS33     57      /* HT MCS rate */
#define IFM_IEEE80211_MCS34     58      /* HT MCS rate */
#define IFM_IEEE80211_MCS35     59      /* HT MCS rate */
#define IFM_IEEE80211_MCS36     60      /* HT MCS rate */
#define IFM_IEEE80211_MCS37     61      /* HT MCS rate */
#define IFM_IEEE80211_MCS38     62      /* HT MCS rate */
/* NB: OFDM72 doesn't really exist so we don't handle it */
#if 0
#define IFM_IEEE80211_OFDM72    24      /* OFDM 72Mbps */
#endif

#define IFM_IEEE80211_ADHOC     0x00000100      /* Operate in Adhoc mode */
#define IFM_IEEE80211_HOSTAP    0x00000200      /* Operate in Host AP mode */
#define IFM_IEEE80211_IBSS      0x00000400      /* Operate in IBSS mode */
#define IFM_IEEE80211_WDS       0x00000800      /* Operate in WDS mode */
#define IFM_IEEE80211_TURBO     0x00001000      /* Operate in turbo mode */
#define IFM_IEEE80211_MONITOR   0x00002000      /* Operate in monitor mode */

/*
 * Shared media sub-types
 */
#define IFM_AUTO        0               /* Autoselect best media */
#define IFM_MANUAL      1               /* Jumper/dipswitch selects media */
#define IFM_NONE        2               /* Deselect all media */

/*
 * Shared options
 */
#define IFM_FDX         0x00100000      /* Force full duplex */
#define IFM_HDX         0x00200000      /* Force half duplex */
#define IFM_FLAG0       0x01000000      /* Driver defined flag */
#define IFM_FLAG1       0x02000000      /* Driver defined flag */
#define IFM_FLAG2       0x04000000      /* Driver defined flag */
#define IFM_LOOP        0x08000000      /* Put hardware in loopback */

/*
 * Status bits
 */
#define IFM_AVALID      0x00000001      /* Active bit valid */
#define IFM_ACTIVE      0x00000002      /* Interface attached to working net */

/*
 * Macros to extract various bits of information from the media word.
 */
#define IFM_TYPE(x)             ((x) & IFM_NMASK)
#define IFM_SUBTYPE(x)          ((x) & IFM_TMASK)
#define IFM_TYPE_OPTIONS(x)     ((x) & IFM_OMASK)
#define IFM_INST(x)             (((x) & IFM_IMASK) >> IFM_ISHIFT)
#define IFM_OPTIONS(x)          ((x) & (IFM_OMASK|IFM_GMASK))
#define IFM_MODE(x)             ((x) & IFM_MMASK)

#define IFM_INST_MAX            IFM_INST(IFM_IMASK)

/*
 * Macro to create a media word.
 */
#define IFM_MAKEWORD(type, subtype, options, instance)                  \
        ((type) | (subtype) | (options) | ((instance) << IFM_ISHIFT))
#define IFM_MAKEMODE(mode) \
        (((mode) << IFM_MSHIFT) & IFM_MMASK)

/*
 * NetBSD extension not defined in the BSDI API.  This is used in various
 * places to get the canonical description for a given type/subtype.
 *
 * NOTE: all but the top-level type descriptions must contain NO whitespace!
 * Otherwise, parsing these in ifconfig(8) would be a nightmare.
 */
struct ifmedia_description 
{
	int     ifmt_word;              /* word value; may be masked */
	const char *ifmt_string;        /* description */
};

#define IFM_TYPE_DESCRIPTIONS {                                         \
	{ IFM_ETHER,            "Ethernet" },                           \
	{ IFM_TOKEN,            "Token ring" },                         \
	{ IFM_FDDI,             "FDDI" },                               \
	{ IFM_IEEE80211,        "IEEE 802.11 Wireless Ethernet" },      \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_ETHERNET_DESCRIPTIONS {                             \
	{ IFM_10_T,     "10baseT/UTP" },                                \
	{ IFM_10_2,     "10base2/BNC" },                                \
	{ IFM_10_5,     "10base5/AUI" },                                \
	{ IFM_100_TX,   "100baseTX" },                                  \
	{ IFM_100_FX,   "100baseFX" },                                  \
	{ IFM_100_T4,   "100baseT4" },                                  \
	{ IFM_100_VG,   "100baseVG" },                                  \
	{ IFM_100_T2,   "100baseT2" },                                  \
	{ IFM_10_STP,   "10baseSTP" },                                  \
	{ IFM_10_FL,    "10baseFL" },                                   \
	{ IFM_1000_SX,  "1000baseSX" },                                 \
	{ IFM_1000_LX,  "1000baseLX" },                                 \
	{ IFM_1000_CX,  "1000baseCX" },                                 \
	{ IFM_1000_T,   "1000baseTX" },                                 \
	{ IFM_1000_T,   "1000baseT" },                                  \
	{ IFM_HPNA_1,   "homePNA" },                                    \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_ETHERNET_ALIASES {                                  \
	{ IFM_10_T,     "UTP" },                                        \
	{ IFM_10_T,     "10UTP" },                                      \
	{ IFM_10_2,     "BNC" },                                        \
	{ IFM_10_2,     "10BNC" },                                      \
	{ IFM_10_5,     "AUI" },                                        \
	{ IFM_10_5,     "10AUI" },                                      \
	{ IFM_100_TX,   "100TX" },                                      \
	{ IFM_100_T4,   "100T4" },                                      \
	{ IFM_100_VG,   "100VG" },                                      \
	{ IFM_100_T2,   "100T2" },                                      \
	{ IFM_10_STP,   "10STP" },                                      \
	{ IFM_10_FL,    "10FL" },                                       \
	{ IFM_1000_SX,  "1000SX" },                                     \
	{ IFM_1000_LX,  "1000LX" },                                     \
	{ IFM_1000_CX,  "1000CX" },                                     \
	{ IFM_1000_T,   "1000TX" },                                     \
	{ IFM_1000_T,   "1000T" },                                      \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_ETHERNET_OPTION_DESCRIPTIONS {                      \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_TOKENRING_DESCRIPTIONS {                            \
	{ IFM_TOK_STP4, "DB9/4Mbit" },                                  \
	{ IFM_TOK_STP16, "DB9/16Mbit" },                                \
	{ IFM_TOK_UTP4, "UTP/4Mbit" },                                  \
	{ IFM_TOK_UTP16, "UTP/16Mbit" },                                \
	{ IFM_TOK_STP100, "STP/100Mbit" },                              \
	{ IFM_TOK_UTP100, "UTP/100Mbit" },                              \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_TOKENRING_ALIASES {                                 \
	{ IFM_TOK_STP4, "4STP" },                                       \
	{ IFM_TOK_STP16, "16STP" },                                     \
	{ IFM_TOK_UTP4, "4UTP" },                                       \
	{ IFM_TOK_UTP16, "16UTP" },                                     \
	{ IFM_TOK_STP100, "100STP" },                                   \
	{ IFM_TOK_UTP100, "100UTP" },                                   \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_TOKENRING_OPTION_DESCRIPTIONS {                     \
	{ IFM_TOK_ETR,  "EarlyTokenRelease" },                          \
	{ IFM_TOK_SRCRT, "SourceRouting" },                             \
	{ IFM_TOK_ALLR,  "AllRoutes" },                                 \
	{ IFM_TOK_DTR,   "Dedicated" },                                 \
	{ IFM_TOK_CLASSIC, "Classic" },                                 \
	{ IFM_TOK_AUTO, " " },                                          \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_FDDI_DESCRIPTIONS {                                 \
	{ IFM_FDDI_SMF, "Single-mode" },                                \
	{ IFM_FDDI_MMF, "Multi-mode" },                                 \
	{ IFM_FDDI_UTP, "UTP" },                                        \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_FDDI_ALIASES {                                      \
	{ IFM_FDDI_SMF, "SMF" },                                        \
	{ IFM_FDDI_MMF, "MMF" },                                        \
	{ IFM_FDDI_UTP, "CDDI" },                                       \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_FDDI_OPTION_DESCRIPTIONS {                          \
	{ IFM_FDDI_DA, "Dual-attach" },                                 \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_IEEE80211_DESCRIPTIONS {                            \
	{ IFM_IEEE80211_FH1, "FH/1Mbps" },                              \
	{ IFM_IEEE80211_FH2, "FH/2Mbps" },                              \
	{ IFM_IEEE80211_DS1, "DS/1Mbps" },                              \
	{ IFM_IEEE80211_DS2, "DS/2Mbps" },                              \
	{ IFM_IEEE80211_DS5, "DS/5.5Mbps" },                            \
	{ IFM_IEEE80211_DS11, "DS/11Mbps" },                            \
	{ IFM_IEEE80211_DS22, "DS/22Mbps" },                            \
	{ IFM_IEEE80211_OFDM1_50, "OFDM/1.50Mbps" },                    \
	{ IFM_IEEE80211_OFDM2_25, "OFDM/2.25Mbps" },                    \
	{ IFM_IEEE80211_OFDM3, "OFDM/3Mbps" },                          \
	{ IFM_IEEE80211_OFDM4_50, "OFDM/4.5Mbps" },                     \
	{ IFM_IEEE80211_OFDM6, "OFDM/6Mbps" },                          \
	{ IFM_IEEE80211_OFDM9, "OFDM/9Mbps" },                          \
	{ IFM_IEEE80211_OFDM12, "OFDM/12Mbps" },                        \
	{ IFM_IEEE80211_OFDM13_5, "OFDM/13.5Mbps" },                    \
	{ IFM_IEEE80211_OFDM18, "OFDM/18Mbps" },                        \
	{ IFM_IEEE80211_OFDM24, "OFDM/24Mbps" },                        \
	{ IFM_IEEE80211_OFDM27, "OFDM/27Mbps" },                        \
	{ IFM_IEEE80211_OFDM36, "OFDM/36Mbps" },                        \
	{ IFM_IEEE80211_OFDM48, "OFDM/48Mbps" },                        \
	{ IFM_IEEE80211_OFDM54, "OFDM/54Mbps" },                        \
	{ IFM_IEEE80211_OFDM72, "OFDM/72Mbps" },                        \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_IEEE80211_ALIASES {                                 \
	{ IFM_IEEE80211_FH1, "FH1" },                                   \
	{ IFM_IEEE80211_FH2, "FH2" },                                   \
	{ IFM_IEEE80211_FH1, "FrequencyHopping/1Mbps" },                \
	{ IFM_IEEE80211_FH2, "FrequencyHopping/2Mbps" },                \
	{ IFM_IEEE80211_DS1, "DS1" },                                   \
	{ IFM_IEEE80211_DS2, "DS2" },                                   \
	{ IFM_IEEE80211_DS5, "DS5.5" },                                 \
	{ IFM_IEEE80211_DS11, "DS11" },                                 \
	{ IFM_IEEE80211_DS22, "DS22" },                                 \
	{ IFM_IEEE80211_DS1, "DirectSequence/1Mbps" },                  \
	{ IFM_IEEE80211_DS2, "DirectSequence/2Mbps" },                  \
	{ IFM_IEEE80211_DS5, "DirectSequence/5.5Mbps" },                \
	{ IFM_IEEE80211_DS11, "DirectSequence/11Mbps" },                \
	{ IFM_IEEE80211_DS22, "DirectSequence/22Mbps" },                \
	{ IFM_IEEE80211_OFDM1_50, "OFDM1.50Mpbs" },                     \
	{ IFM_IEEE80211_OFDM2_25, "OFDM2.25Mbps" },                     \
	{ IFM_IEEE80211_OFDM3, "OFDM3Mbps" },                           \
	{ IFM_IEEE80211_OFDM4_50, "OFDM4.5Mbps" },                      \
	{ IFM_IEEE80211_OFDM6, "OFDM6" },                               \
	{ IFM_IEEE80211_OFDM9, "OFDM9" },                               \
	{ IFM_IEEE80211_OFDM12, "OFDM12" },                             \
	{ IFM_IEEE80211_OFDM13_5, "OFDM13.5Mbps" },                     \
	{ IFM_IEEE80211_OFDM18, "OFDM18" },                             \
	{ IFM_IEEE80211_OFDM24, "OFDM24" },                             \
	{ IFM_IEEE80211_OFDM27, "OFDM27" },                             \
	{ IFM_IEEE80211_OFDM36, "OFDM36" },                             \
	{ IFM_IEEE80211_OFDM48, "OFDM48" },                             \
	{ IFM_IEEE80211_OFDM54, "OFDM54" },                             \
	{ IFM_IEEE80211_OFDM72, "OFDM72" },                             \
	{ IFM_IEEE80211_DS1, "CCK1" },                                  \
	{ IFM_IEEE80211_DS2, "CCK2" },                                  \
	{ IFM_IEEE80211_DS5, "CCK5.5" },                                \
	{ IFM_IEEE80211_DS11, "CCK11" },                                \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_IEEE80211_OPTION_DESCRIPTIONS {                     \
	{ IFM_IEEE80211_ADHOC, "adhoc" },                               \
	{ IFM_IEEE80211_HOSTAP, "hostap" },                             \
	{ IFM_IEEE80211_IBSS, "ibss" },                                 \
	{ IFM_IEEE80211_WDS, "wds" },                                   \
	{ IFM_IEEE80211_TURBO, "turbo" },                               \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_IEEE80211_MODE_DESCRIPTIONS {                       \
	{ IFM_IEEE80211_11A, "11a" },                                   \
	{ IFM_IEEE80211_11B, "11b" },                                   \
	{ IFM_IEEE80211_11G, "11g" },                                   \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_SHARED_DESCRIPTIONS {                               \
	{ IFM_AUTO,     "autoselect" },                                 \
	{ IFM_MANUAL,   "manual" },                                     \
	{ IFM_NONE,     "none" },                                       \
	{ 0, NULL },                                                    \
}

#define IFM_SUBTYPE_SHARED_ALIASES {                                    \
	{ IFM_AUTO,     "auto" },                                       \
	{ 0, NULL },                                                    \
}

#define IFM_SHARED_OPTION_DESCRIPTIONS {                                \
	{ IFM_FDX,      "full-duplex" },                                \
	{ IFM_HDX,      "half-duplex" },                                \
	{ IFM_FLAG0,    "flag0" },                                      \
	{ IFM_FLAG1,    "flag1" },                                      \
	{ IFM_FLAG2,    "flag2" },                                      \
	{ IFM_LOOP,     "hw-loopback" },                                \
	{ 0, NULL },                                                    \
}

#endif  //End of _NET_IF_NDIFACE_H_
