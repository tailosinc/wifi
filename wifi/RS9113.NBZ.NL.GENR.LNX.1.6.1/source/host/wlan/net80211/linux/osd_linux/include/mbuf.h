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

#ifndef _NET_MBUF_H
#define _NET_MBUF_H

#include <linux/netdevice.h> 
#include <linux/skbuff.h>
#include "common.h"

#define mbuf   netbuf_ctrl_block_m_s
#define M_HDR_PAD    2
#define MLEN            (MSIZE - sizeof(struct m_hdr))  /* normal data len */
#define MHLEN           (MLEN - sizeof(struct pkthdr))  /* data len w/pkthdr */
#define MSIZE 256

//Max buffer size if reduced to 1500 ?? check the scenario in defragmentation case
#define MAX_BUFFER_SIZE  1602

struct pkthdr 
{
	struct net_device  *rcvif;         /* rcv interface */
	/* variables for ip and tcp reassembly */
	void               *header;        /* pointer to packet header */
	int                len;           /* total packet length */
	uint32_t           flowid;        /* packet's 4-tuple system 
	                                   * flow identifier
	                                   */
	/* variables for hardware checksum */
	int                csum_flags;    /* flags regarding checksum */
	int                csum_data;     /* data field used by csum routines */
	u_int16_t          tso_segsz;     /* TSO segment size */
	union 
	{
		u_int16_t  vt_vtag;      /* Ethernet 802.1p+q vlan tag */
		u_int16_t  vt_nrecs;     /* # of IGMPv3 records in this chain */
	} PH_vt;
	SLIST_HEAD(packet_tags, m_tag) tags; /* list of packet tags */
};

struct m_ext 
{
	caddr_t          ext_buf;       /* start of buffer */
	void            (*ext_free)     /* free routine if not the usual */
	(void *, void *);
	void            *ext_arg1;      /* optional argument pointer */
	void            *ext_arg2;      /* optional argument pointer */
	uint32_t            ext_size;      /* size of buffer, for ext_free */
	volatile uint32_t  *ref_cnt;       /* pointer to ref count info */
	int              ext_type;      /* type of external storage */
};

struct m_hdr 
{
	struct mbuf     *mh_next;       /* next buffer in chain */
	struct mbuf     *mh_nextpkt;    /* next chain in queue/record */
	caddr_t          mh_data;       /* location of data */
	int              mh_len;        /* amount of data in this mbuf */
	int              mh_flags;      /* flags; see below */
	short            mh_type;       /* type of data in this mbuf */
	uint8_t          pad[M_HDR_PAD];/* word align                  */
};

enum EDCA_QUEUE
{
	BK_Q_STA = 0,
	BE_Q_STA,
	VI_Q_STA,
	VO_Q_STA,
	BK_Q_AP,
	BE_Q_AP,
	VI_Q_AP,
	VO_Q_AP,
};


/* Size of this structure should not exceed 48 bytes in case
 * if we are using skb->cb memory for this structure 
 */
#define M_PKTHDR        0x00000002 /* start of record */
typedef struct netbuf_ctrl_block_m_s 
{
	struct m_hdr    m_hdr;
	union 
	{
		struct 
		{
			struct pkthdr   MH_pkthdr;      /* M_PKTHDR set */
			union 
			{
				struct m_ext    MH_ext; /* M_EXT set */
				char            MH_databuf[MHLEN];
			} MH_dat;
		} MH;
		char    M_databuf[MLEN];                /* !M_PKTHDR, !M_EXT */
	} M_dat;

	uint8_t          vap[20];        // Pointer to vap structure reserved
	struct netbuf_ctrl_block_m_s *next; // next ptr
	uint32_t        *pkt_addr;
	void           *head;          // head address
	uint8_t          mac_hdr_len;    // MAC header length(80211)
	uint8_t          hdr_pad_len;    // Header padding length
	uint8_t          sta_id;         // Station id
	uint8_t          tid;            // Traffic identifier
	uint16_t         aggr_len;       // Total aggregation length
	uint32_t         lmac_tsf;      // Time stamp of the packet 
	uint8_t          priority;      
	uint8_t          skb_priority;
	unsigned short          flags;          // Flags
	void            *dev;
	uint64_t	pn;
	uint8_t		pn_valid;
}netbuf_ctrl_block_m_t;

#define m_next          m_hdr.mh_next
#define m_len           m_hdr.mh_len
#define m_data          m_hdr.mh_data
#define m_type          m_hdr.mh_type
#define m_flags         m_hdr.mh_flags
#define m_nextpkt       m_hdr.mh_nextpkt
#define m_act           m_nextpkt
#define m_pkthdr        M_dat.MH.MH_pkthdr
#define m_ext           M_dat.MH.MH_dat.MH_ext
#define m_pktdat        M_dat.MH.MH_dat.MH_databuf
#define m_dat           M_dat.M_databuf 

#define m_dup          netbuf_copy
#define m_freem        netbuf_free
#define mtod(m, t)     ((t)((m)->m_data))
#define m_pullup       netbuf_pullup
#define m_adj          netbuf_adj
#define M_PREPEND      netbuf_prepend
#define m_append       netbuf_append
#define m_split        netbuf_split
#define m_cat          netbuf_cat
#define m_unshare      netbuf_unshare
#define m_align        netbuf_align
#define m_copypacket   netbuf_copy
#define m_gethdr netbuf_gethdr
#define m_getjcl NULL
#define m_move_pkthdr(a,b) null_function()
#define ONEBOX_NEEDED_HEADROOM 234 //(USB_TX_HEADROOM 128 + Frame Desc 16 + D-word Allignment 64 + Extended Desc 26)
/*
 * Store WME access control bits in the vlan tag.
 * This is safe since it's done after the packet is classified
 * (where we use any previous tag) and because it's passed
 * directly in to the driver and there's no chance someone
 * else will clobber them on us.
 */
/* Need to be reviewed */
#define M_WME_SETAC(m, ac) \
	((m)->m_pkthdr.ether_vtag = (ac))
#define M_WME_SETTID(m, tid) \
	((m)->tid = (tid))
/*
 * Mbufs on the power save queue are tagged with an age and
 * timed out.  We reuse the hardware checksum field in the
 * mbuf packet header to store this data.
 */
/* Need to be reviewed */
#define M_AGE_SET(m,v) (m->m_pkthdr.csum_data = v)
#define M_AGE_GET(m)   (m->m_pkthdr.csum_data)
#define M_AGE_SUB(m,adj)(m->m_pkthdr.csum_data -= adj)
#define ether_vtag      PH_vt.vt_vtag
#define MT_DATA         1       /* dynamic (data) allocation */
#define M_PROTO5        0x00000100 /* protocol-specific */
#define M_MORE_DATA     M_PROTO5                /* more data frames to follow */
#define M_EXT           0x00000001 /* has associated external storage */

#define M_RDONLY        0x00000008 /* associated data is marked read-only */
#define M_EOSP        0x00000008 /* protocol-specific */

/*
 * Evaluate TRUE if it's safe to write to the mbuf m's data region (this can
 * be both the local data payload, or an external buffer area, depending on
 * whether M_EXT is set).
 */
#define M_WRITABLE(m)   (!((m)->m_flags & M_RDONLY)) 

#define M_TRAILINGSPACE(m)                                              \
	((m)->m_flags & M_EXT ?                                         \
	 (M_WRITABLE(m) ? (m)->m_ext.ext_buf + (m)->m_ext.ext_size   \
	  - ((m)->m_data + (m)->m_len) : 0) :                     \
	 &(m)->m_dat[MLEN] - ((m)->m_data + (m)->m_len))

#define M_LEADINGSPACE(m) get_headroom_of_m(m)

#define PSKB struct sk_buff *
#ifndef MCLSHIFT
#define MCLSHIFT        11              /* convert bytes to mbuf clusters */
#endif  /* MCLSHIFT */

#define MCLBYTES        (1 << MCLSHIFT) /* size of an mbuf cluster */

/* Store the sequence number.
*/
#define M_SEQNO_SET(m, seqno) \
	((m)->m_pkthdr.tso_segsz = (seqno))


struct ieee80211_node;
/* Function Prototypes */
int m_append(struct netbuf_ctrl_block_m_s *netbuf_cb, int len, const char *cp);
netbuf_ctrl_block_m_t*
netbuf_prepend(struct netbuf_ctrl_block_m_s *netbuf_cb, int len, int how);
void m_align(struct mbuf *m, int len) ;
int ieee80211_add_callback(struct mbuf *m,
                           void (*func)(struct ieee80211_node *, void *, int), void *arg);
netbuf_ctrl_block_m_t *netbuf_pullup(struct  netbuf_ctrl_block_m_s *netbuf_cb, int len);
struct netbuf_ctrl_block_m_s *netbuf_split(netbuf_ctrl_block_m_t *netbuf_cb, int len0, int wait);
void netbuf_cat(netbuf_ctrl_block_m_t *m, netbuf_ctrl_block_m_t *n);
void netbuf_adj(struct netbuf_ctrl_block_m_s *netbuf_cb, int len);
void m_copydata(const struct mbuf *m, int off, int len, caddr_t cp);
struct netbuf_ctrl_block_m_s * netbuf_unshare(struct netbuf_ctrl_block_m_s *m0, int how);
netbuf_ctrl_block_m_t *netbuf_copy (netbuf_ctrl_block_m_t *netbuf_cb, int malloc_flags);
netbuf_ctrl_block_m_t *netbuf_gethdr (int wait,   int type);
struct mbuf * onebox_translate_skb_to_netcb_m(struct sk_buff *skb);
netbuf_ctrl_block_m_t *  onebox_translate_netbuf_to_mbuf(netbuf_ctrl_block_t *netbuf_cb);
netbuf_ctrl_block_t *  onebox_translate_mbuf_to_netbuf(netbuf_ctrl_block_m_t *netbuf_cb);
void netbuf_free(netbuf_ctrl_block_m_t *netbuf_cb);
struct mbuf* m_getcl(int how,short type,int flags);
struct sk_buff *onebox_translate_mbuf_to_skb(struct mbuf *mbuf);
int get_headroom_of_m(netbuf_ctrl_block_m_t *m);
#endif // End of _NET_MBUF_H
