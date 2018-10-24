/*-
 * Copyright (c) 2001 Atsushi Onoe
 * Copyright (c) 2002-2009 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifdef __FREEBSD__
#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/net80211/ieee80211_output.c,v 1.108 2011/03/13 12:56:46 bschmidt Exp $");

#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/systm.h> 
#include <sys/mbuf.h>   
#include <sys/kernel.h>
#include <sys/endian.h>

#include <sys/socket.h>
 
#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_llc.h>
#include <net/if_media.h>
#include <net/if_vlan_var.h>
#ifdef INET
#include <netinet/in.h> 
#include <netinet/if_ether.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#endif
#ifdef INET6
#include <netinet/ip6.h>
#endif

#include <security/mac/mac_framework.h>
#endif

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_regdomain.h>
#ifdef IEEE80211_SUPPORT_SUPERG
#include <net80211/ieee80211_superg.h>
#endif
#ifdef IEEE80211_SUPPORT_TDMA
#include <net80211/ieee80211_tdma.h>
#endif
#include <net80211/ieee80211_wds.h>
#include <net80211/ieee80211_mesh.h>
#define INET

#define	ETHER_HEADER_COPY(dst, src) \
	memcpy(dst, src, sizeof(struct ether_header))

/* unalligned little endian access */     
#define LE_WRITE_2(p, v) do {				\
	((uint8_t *)(p))[0] = (v) & 0xff;		\
	((uint8_t *)(p))[1] = ((v) >> 8) & 0xff;	\
} while (0)
#define LE_WRITE_4(p, v) do {				\
	((uint8_t *)(p))[0] = (v) & 0xff;		\
	((uint8_t *)(p))[1] = ((v) >> 8) & 0xff;	\
	((uint8_t *)(p))[2] = ((v) >> 16) & 0xff;	\
	((uint8_t *)(p))[3] = ((v) >> 24) & 0xff;	\
} while (0)

static int ieee80211_fragment(struct ieee80211vap *, struct mbuf *,
	u_int hdrsize, u_int ciphdrsize, u_int mtu);
static	void ieee80211_tx_mgt_cb(struct ieee80211_node *, void *, int);

#ifdef IEEE80211_DEBUG
/*
 * Decide if an outbound management frame should be
 * printed when debugging is enabled.  This filters some
 * of the less interesting frames that come frequently
 * (e.g. beacons).
 */
static __inline int
doprint(struct ieee80211vap *vap, int subtype)
{
	switch (subtype) {
	case IEEE80211_FC0_SUBTYPE_PROBE_RESP:
		return (vap->iv_opmode == IEEE80211_M_IBSS);
	}
	return 1;
}
#endif

/*
 * Start method for vap's.  All packets from the stack come
 * through here.  We handle common processing of the packets
 * before dispatching them to the underlying device.
 */
void
ieee80211_start(struct ifnet *ifp)
{
#define	IS_DWDS(vap) \
	(vap->iv_opmode == IEEE80211_M_WDS && \
	 (vap->iv_flags_ext & IEEE80211_FEXT_WDSLEGACY) == 0)

#ifdef __FreeBSD__
	struct ieee80211vap *vap = ifp->if_softc;
#elif __LINUX__ 
  struct ifaltq *if_snd;
  #if KERNEL_VERSION_BTWN_2_6_(18,22)
	struct ieee80211vap *vap = ifp->priv;
  #else
	struct ieee80211vap *vap = netdev_priv(ifp);
  #endif
	struct ieee80211_frame *wh;
#endif
	struct ieee80211com *ic = vap->iv_ic;
	struct ifnet *parent = ic->ic_ifp;
	struct ieee80211_node *ni;
	struct mbuf *m;
	struct ether_header *eh;
	int error;

	/* NB: parent must be up and running */
	if (!IFNET_IS_UP_RUNNING(parent)) {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_OUTPUT,
		    "%s: ignore queue, parent %s not up+running\n",
		    __func__, parent->if_xname);
		/* XXX stat */
		return;
	}
	if (vap->iv_state == IEEE80211_S_SLEEP) {
		/*
		 * In power save, wakeup device for transmit.
		 */
		ieee80211_new_state(vap, IEEE80211_S_RUN, 0);
		return;
	}
	/*
	 * No data frames go out unless we're running.
	 * Note in particular this covers CAC and CSA
	 * states (though maybe we should check muting
	 * for CSA).
	 */
	if (vap->iv_state != IEEE80211_S_RUN) {
		IEEE80211_LOCK(ic);
		/* re-check under the com lock to avoid races */
		if (vap->iv_state != IEEE80211_S_RUN) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_OUTPUT,
			    "%s: ignore queue, in %s state\n",
			    __func__, ieee80211_state_name[vap->iv_state]);
			vap->iv_stats.is_tx_badstate++;
			IEEE80211_UNLOCK(ic);
#ifdef __FREEBSD__
			IFQ_LOCK(&ifp->if_snd);
#elif __LINUX__
			if_snd = (struct ifaltq *)ifp->ml_priv;
			IFQ_LOCK(if_snd);
#endif
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
#ifdef __FREEBSD__
			IFQ_UNLOCK(&ifp->if_snd);
#elif __LINUX__
			IFQ_UNLOCK(if_snd);
#endif
			return;
		}
		IEEE80211_UNLOCK(ic);
	}
	for (;;) {
	#ifdef __FreeBSD__
		IFQ_DEQUEUE(&ifp->if_snd, m);
    #elif __LINUX__
	    if_snd = (struct ifaltq *)ifp->ml_priv;
		IFQ_DEQUEUE(if_snd, m);
	#endif
		if (m == NULL)
			break;
		/*
		 * Sanitize mbuf flags for net80211 use.  We cannot
		 * clear M_PWR_SAV or M_MORE_DATA because these may
		 * be set for frames that are re-submitted from the
		 * power save queue.
		 *
		 * NB: This must be done before ieee80211_classify as
		 *     it marks EAPOL in frames with M_EAPOL.
		 */
		m->m_flags &= ~(M_80211_TX - M_PWR_SAV - M_MORE_DATA);
		/*
		 * Cancel any background scan.
		 */
		if (ic->ic_flags & IEEE80211_F_SCAN)
			ieee80211_cancel_anyscan(vap);
		/* 
		 * Find the node for the destination so we can do
		 * things like power save and fast frames aggregation.
		 *
		 * NB: past this point various code assumes the first
		 *     mbuf has the 802.3 header present (and contiguous).
		 */
		ni = NULL;
		if (m->m_len < sizeof(struct ether_header) &&
		   (m = m_pullup(m, sizeof(struct ether_header))) == NULL) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_OUTPUT,
			    "discard frame, %s\n", "m_pullup failed");
			vap->iv_stats.is_tx_nobuf++;	/* XXX */
#ifdef  __FreeBSD__
    	ifp->if_oerrors++;
#elif __LINUX__
        ifp->stats.tx_errors++;
#endif
			continue;
		}
		eh = mtod(m, struct ether_header *);
		if (ETHER_IS_MULTICAST(eh->ether_dhost)) {
			if (IS_DWDS(vap)) {
				/*
				 * Only unicast frames from the above go out
				 * DWDS vaps; multicast frames are handled by
				 * dispatching the frame as it comes through
				 * the AP vap (see below).
				 */
				IEEE80211_DISCARD_MAC(vap, IEEE80211_MSG_WDS,
				    eh->ether_dhost, "mcast", "%s", "on DWDS");
				vap->iv_stats.is_dwds_mcast++;
				m_freem(m);
				continue;
			}
			if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
				/*
				 * Spam DWDS vap's w/ multicast traffic.
				 */
				/* XXX only if dwds in use? */
				ieee80211_dwds_mcast(vap, m);
			}
		}
#ifdef IEEE80211_SUPPORT_MESH
		if (vap->iv_opmode != IEEE80211_M_MBSS) {
#endif
			ni = ieee80211_find_txnode(vap, eh->ether_dhost);
			if (ni == NULL) {
				/* NB: ieee80211_find_txnode does stat+msg */
#ifdef  __FreeBSD__
        	ifp->if_oerrors++;
#elif __LINUX__
        ifp->stats.tx_errors++;
#endif
			m_freem(m);
				continue;
			}

			if ((ni->ni_associd == 0 &&
			    (ni->ni_flags & IEEE80211_NODE_ASSOCID))
					|| (!(ni->ni_flags & IEEE80211_NODE_AUTH) 
					 && !(eh->ether_type == htons(ETHERTYPE_PAE)))) {
				IEEE80211_DISCARD_MAC(vap, IEEE80211_MSG_OUTPUT,
				    eh->ether_dhost, NULL,
				    "sta not associated (type 0x%04x)",
				    htons(eh->ether_type));
				vap->iv_stats.is_tx_notassoc++;
#ifdef  __FreeBSD__
            	ifp->if_oerrors++;
#elif __LINUX__
        ifp->stats.tx_errors++;
#endif
				m_freem(m);
				ieee80211_free_node(ni);
				continue;
			}
#ifdef IEEE80211_SUPPORT_MESH
		} else {
			if (!IEEE80211_ADDR_EQ(eh->ether_shost, vap->iv_myaddr)) {
				/*
				 * Proxy station only if configured.
				 */
				if (!ieee80211_mesh_isproxyena(vap)) {
					IEEE80211_DISCARD_MAC(vap,
					    IEEE80211_MSG_OUTPUT |
						IEEE80211_MSG_MESH,
					    eh->ether_dhost, NULL,
					    "%s", "proxy not enabled");
					vap->iv_stats.is_mesh_notproxy++;
	#ifdef  __FreeBSD__
                	ifp->if_oerrors++;
    #elif __LINUX__
          ifp->stats.tx_errors++;
    #endif
					m_freem(m);
					continue;
				}
				ieee80211_mesh_proxy_check(vap, eh->ether_shost);
			}
			ni = ieee80211_mesh_discover(vap, eh->ether_dhost, m);
			if (ni == NULL) {
				/*
				 * NB: ieee80211_mesh_discover holds/disposes
				 * frame (e.g. queueing on path discovery).
				 */

          #ifdef  __FreeBSD__
                 ifp->if_oerrors++;
          #elif __LINUX__
                 ifp->stats.tx_errors++;
          #endif
				continue;
			}
		}
#endif
		if ((ni->ni_flags & IEEE80211_NODE_PWR_MGT) &&
		    (m->m_flags & M_PWR_SAV) == 0) {
			/*
			 * Station in power save mode; pass the frame
			 * to the 802.11 layer and continue.  We'll get
			 * the frame back when the time is right.
			 * XXX lose WDS vap linkage?
			 */
			(void) ieee80211_pwrsave(ni, m);
			ieee80211_free_node(ni);
			continue;
		}
		/* calculate priority so drivers can find the tx queue */
		if (ieee80211_classify(ni, m)) {
			IEEE80211_DISCARD_MAC(vap, IEEE80211_MSG_OUTPUT,
			    eh->ether_dhost, NULL,
			    "%s", "classification failure");
			vap->iv_stats.is_tx_classify++;
      #ifdef  __FreeBSD__
            ifp->if_oerrors++;
      #elif __LINUX__
            ifp->stats.tx_errors++;
      #endif
			m_freem(m);
			ieee80211_free_node(ni);
			continue;
		}
		if(m->m_flags & M_EAPOL)
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, ("EAPOL Recvd\n"));
		/*
		 * Stash the node pointer.  Note that we do this after
		 * any call to ieee80211_dwds_mcast because that code
		 * uses any existing value for rcvif to identify the
		 * interface it (might have been) received on.
		 */
		m->m_pkthdr.rcvif = (void *)ni;

		BPF_MTAP(ifp, m);		/* 802.3 tx */
 
		/*
		 * Check if A-MPDU tx aggregation is setup or if we
		 * should try to enable it.  The sta must be associated
		 * with HT and A-MPDU enabled for use.  When the policy
		 * routine decides we should enable A-MPDU we issue an
		 * ADDBA request and wait for a reply.  The frame being
		 * encapsulated will go out w/o using A-MPDU, or possibly
		 * it might be collected by the driver and held/retransmit.
		 * The default ic_ampdu_enable routine handles staggering
		 * ADDBA requests in case the receiver NAK's us or we are
		 * otherwise unable to establish a BA stream.
		 */
//	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("AMPDU: ni_flags %x and iv_flags_ht %x mflags =%x\n", ni->ni_flags, vap->iv_flags_ht, m->m_flags));
		if (((ni->ni_flags & IEEE80211_NODE_AMPDU_TX) && (ni->ni_htrates.rs_nrates)) &&
		    (vap->iv_flags_ht & IEEE80211_FHT_AMPDU_TX) &&
		    (m->m_flags & M_EAPOL) == 0) {
		//	const int ac = M_WME_GETAC(m);
			int tid = M_WME_GETTID(m);
			struct ieee80211_tx_ampdu *tap = &ni->ni_tx_ampdu[tid];

			ieee80211_txampdu_count_packet(tap);
			//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Ampdu:Before checking ampdu status ac=%d tap->flags =%x\n", ac, tap->txa_flags));
			if (IEEE80211_AMPDU_RUNNING(tap)) {
				/*
				 * Operational, mark frame for aggregation.
				 *
				 * XXX do tx aggregation here
				 */
				//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Ampdu: Ampdu already enabled\n"));
				m->m_flags |= M_AMPDU_MPDU;
			} else if (!IEEE80211_AMPDU_REQUESTED(tap) &&
			    ic->ic_ampdu_enable(ni, tap)) {
				/*
				 * Not negotiated yet, request service.
				 */
#ifdef CONFIG_11W
				if (vap->iv_opmode == IEEE80211_M_STA) {
				    if (vap->iv_rsn_ie) {
				    	if (!vap->hal_priv_vap->conn_in_prog) {
#endif
				ieee80211_ampdu_request(ni, tap);
#ifdef CONFIG_11W
					}
				    } else 
					ieee80211_ampdu_request(ni, tap);
				}

				if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
				    if (vap->iv_rsn_ie) {
				    	if (ni) {
						if(!ni->hal_priv_node.conn_in_prog) {
						ieee80211_ampdu_request(ni, tap);
						}
					}
				    } else 
				ieee80211_ampdu_request(ni, tap);
				}
#endif
			}
		}
#ifdef IEEE80211_SUPPORT_SUPERG
		else if (IEEE80211_ATH_CAP(vap, ni, IEEE80211_NODE_FF)) {
			m = ieee80211_ff_check(ni, m);
			if (m == NULL) {
				/* NB: any ni ref held on stageq */
				continue;
			}
		}
#endif /* IEEE80211_SUPPORT_SUPERG */
		if (__predict_true((vap->iv_caps & IEEE80211_C_8023ENCAP) == 0)) {
			/*
			 * Encapsulate the packet in prep for transmission.
			 */
			m = ieee80211_encap(vap, ni, m);
			if (m == NULL) {
				/* NB: stat+msg handled in ieee80211_encap */
				ieee80211_free_node(ni);
				continue;
			}
		}

       #ifdef __FreeBSD__
        error = parent->if_transmit(parent, m);
	   #elif __LINUX__
	   if (__predict_true((vap->iv_caps & IEEE80211_C_8023ENCAP) == 0)) 
	   {
			wh = mtod(m, struct ieee80211_frame *);
			if (wh->i_fc[1] & IEEE80211_FC1_WEP) 
			{
				ieee80211_crypto_encap(ni, m);
	    	}
	   }
	   error = send_data_pkt_to_drv(parent, m, ni);
       #endif

	   if (error != 0) {
		   /* NB: IFQ_HANDOFF reclaims mbuf */
		   ieee80211_free_node(ni);
	   } else {
#ifdef  __FreeBSD__
		   ifp->if_opackets++;
#elif __LINUX__
		   ifp->stats.tx_packets++;
		   ifp->stats.tx_bytes += m->m_len;
#endif
	   }
	   ic->ic_lastdata = ticks;
	}
#undef IS_DWDS
}

/*
 * 802.11 output routine. This is (currently) used only to
 * connect bpf write calls to the 802.11 layer for injecting
 * raw 802.11 frames.
 */
#ifdef __FREEBSD__
int
ieee80211_output(struct ifnet *ifp, struct mbuf *m,
	struct sockaddr *dst, struct route *ro)
#elif __LINUX__
int ieee80211_output(struct sk_buff *skb, struct ifnet *ifp)
#endif
{
#define senderr(e) do { error = (e); goto bad;} while (0)
	struct ieee80211_node *ni = NULL;
	struct ieee80211vap *vap;
	struct ieee80211_frame *wh;
	int error;
#ifdef __LINUX__
	struct mbuf *m = onebox_translate_skb_to_netcb_m(skb);
	struct ifaltq *if_snd;
#endif


#ifdef __FREEBSD__
	IFQ_LOCK(&ifp->if_snd);
#elif __LINUX__
	if_snd = (struct ifaltq *)ifp->ml_priv;
	IFQ_LOCK(if_snd);
#endif
	if (ifp->if_drv_flags & IFF_DRV_OACTIVE) {
#ifdef __FREEBSD__
		IFQ_UNLOCK(&ifp->if_snd);
#elif __LINUX__
		IFQ_UNLOCK_EARLY(if_snd);
#endif
		/*
		 * Short-circuit requests if the vap is marked OACTIVE
		 * as this can happen because a packet came down through
		 * ieee80211_start before the vap entered RUN state in
		 * which case it's ok to just drop the frame.  This
		 * should not be necessary but callers of if_output don't
		 * check OACTIVE.
		 */
		senderr(ENETDOWN);
	}
#ifdef __FREEBSD__
	IFQ_UNLOCK(&ifp->if_snd);
#elif __LINUX__
	IFQ_UNLOCK(if_snd);
#endif
#ifdef __FREEBSD__
	vap = ifp->if_softc;
#elif __LINUX__ 
  #if KERNEL_VERSION_BTWN_2_6_(18,22)
	vap = ifp->priv;
  #else
	vap = netdev_priv(ifp);
  #endif
#endif
	/*
	 * Hand to the 802.3 code if not tagged as
	 * a raw 802.11 frame.
	 */
#ifdef __FREEBSD__
	if (dst->sa_family != AF_IEEE80211)
		return vap->iv_output(ifp, m, dst, ro);
#endif
#ifdef MAC
	error = mac_ifnet_check_transmit(ifp, m);
	if (error)
		senderr(error);
#endif
	if (ifp->if_flags & IFF_MONITOR)
		senderr(ENETDOWN);
	if (!IFNET_IS_UP_RUNNING(ifp))
		senderr(ENETDOWN);
	if (vap->iv_state == IEEE80211_S_CAC) {
		IEEE80211_DPRINTF(vap,
		    IEEE80211_MSG_OUTPUT | IEEE80211_MSG_DOTH,
		    "block %s frame in CAC state\n", "raw data");
		vap->iv_stats.is_tx_badstate++;
		senderr(EIO);		/* XXX */
	} else if (vap->iv_state == IEEE80211_S_SCAN)
		senderr(EIO);
	/* XXX bypass bridge, pfil, carp, etc. */

	if (m->m_pkthdr.len < sizeof(struct ieee80211_frame_ack))
		senderr(EIO);	/* XXX */
	wh = mtod(m, struct ieee80211_frame *);
	if ((wh->i_fc[0] & IEEE80211_FC0_VERSION_MASK) !=
	    IEEE80211_FC0_VERSION_0)
		senderr(EIO);	/* XXX */

	/* locate destination node */
	switch (wh->i_fc[1] & IEEE80211_FC1_DIR_MASK) {
	case IEEE80211_FC1_DIR_NODS:
	case IEEE80211_FC1_DIR_FROMDS:
		ni = ieee80211_find_txnode(vap, wh->i_addr1);
		break;
	case IEEE80211_FC1_DIR_TODS:
	case IEEE80211_FC1_DIR_DSTODS:
		if (m->m_pkthdr.len < sizeof(struct ieee80211_frame))
			senderr(EIO);	/* XXX */
		ni = ieee80211_find_txnode(vap, wh->i_addr3);
		break;
	}
	if (ni == NULL) {
		/*
		 * Permit packets w/ bpf params through regardless
		 * (see below about sa_len).
		 */
#ifdef __FREEBSD__
		if (dst->sa_len == 0)
			senderr(EHOSTUNREACH);
#endif
		ni = ieee80211_ref_node(vap->iv_bss);
	}

	/*
	 * Sanitize mbuf for net80211 flags leaked from above.
	 *
	 * NB: This must be done before ieee80211_classify as
	 *     it marks EAPOL in frames with M_EAPOL.
	 */
	m->m_flags &= ~M_80211_TX;

	/* calculate priority so drivers can find the tx queue */
	/* XXX assumes an 802.3 frame */
	if (ieee80211_classify(ni, m))
		senderr(EIO);		/* XXX */

     #ifdef  __FreeBSD__
            ifp->if_opackets++;
     #elif __LINUX__
            ifp->stats.tx_packets++;
            ifp->stats.tx_bytes += m->m_len;
     #endif
	IEEE80211_NODE_STAT(ni, tx_data);
	if (IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		IEEE80211_NODE_STAT(ni, tx_mcast);
		m->m_flags |= M_MCAST;
	} else
		IEEE80211_NODE_STAT(ni, tx_ucast);
	/* NB: ieee80211_encap does not include 802.11 header */
	IEEE80211_NODE_STAT_ADD(ni, tx_bytes, m->m_pkthdr.len);

	/*
	 * NB: DLT_IEEE802_11_RADIO identifies the parameters are
	 * present by setting the sa_len field of the sockaddr (yes,
	 * this is a hack).
	 * NB: we assume sa_data is suitably aligned to cast.
	 */
#ifdef __FREEBSD__
	return vap->iv_ic->ic_raw_xmit(ni, m,
	    (const struct ieee80211_bpf_params *)(dst->sa_len ?
		dst->sa_data : NULL));
#elif __LINUX__
     return vap->iv_ic->ic_raw_xmit(ni,m,NULL);
#endif
bad:
	if (m != NULL)
		m_freem(m);
	if (ni != NULL)
		ieee80211_free_node(ni);
#ifdef  __FreeBSD__
    	ifp->if_oerrors++;
#elif __LINUX__
      ifp->stats.tx_errors++;
#endif

	return error;
#undef senderr
}

/*
 * Set the direction field and address fields of an outgoing
 * frame.  Note this should be called early on in constructing
 * a frame as it sets i_fc[1]; other bits can then be or'd in.
 */
void
ieee80211_send_setup(
	struct ieee80211_node *ni,
	struct mbuf *m,
	int type, int tid,
	const uint8_t sa[IEEE80211_ADDR_LEN],
	const uint8_t da[IEEE80211_ADDR_LEN],
	const uint8_t bssid[IEEE80211_ADDR_LEN])
{
#define	WH4(wh)	((struct ieee80211_frame_addr4 *)wh)
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_tx_ampdu *tap;
	struct ieee80211_frame *wh = mtod(m, struct ieee80211_frame *);
	ieee80211_seq seqno;

	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | type;
	if ((type & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA) {
		switch (vap->iv_opmode) {
		case IEEE80211_M_STA:
			wh->i_fc[1] = IEEE80211_FC1_DIR_TODS;
			IEEE80211_ADDR_COPY(wh->i_addr1, bssid);
			IEEE80211_ADDR_COPY(wh->i_addr2, sa);
			IEEE80211_ADDR_COPY(wh->i_addr3, da);
			break;
		case IEEE80211_M_IBSS:
		case IEEE80211_M_AHDEMO:
			wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
			IEEE80211_ADDR_COPY(wh->i_addr1, da);
			IEEE80211_ADDR_COPY(wh->i_addr2, sa);
			IEEE80211_ADDR_COPY(wh->i_addr3, bssid);
			break;
		case IEEE80211_M_HOSTAP:
			wh->i_fc[1] = IEEE80211_FC1_DIR_FROMDS;
			IEEE80211_ADDR_COPY(wh->i_addr1, da);
			IEEE80211_ADDR_COPY(wh->i_addr2, bssid);
			IEEE80211_ADDR_COPY(wh->i_addr3, sa);
			break;
		case IEEE80211_M_WDS:
			wh->i_fc[1] = IEEE80211_FC1_DIR_DSTODS;
			IEEE80211_ADDR_COPY(wh->i_addr1, da);
			IEEE80211_ADDR_COPY(wh->i_addr2, vap->iv_myaddr);
			IEEE80211_ADDR_COPY(wh->i_addr3, da);
			IEEE80211_ADDR_COPY(WH4(wh)->i_addr4, sa);
			break;
		case IEEE80211_M_MBSS:
#ifdef IEEE80211_SUPPORT_MESH
			/* XXX add support for proxied addresses */
			if (IEEE80211_IS_MULTICAST(da)) {
				wh->i_fc[1] = IEEE80211_FC1_DIR_FROMDS;
				/* XXX next hop */
				IEEE80211_ADDR_COPY(wh->i_addr1, da);
				IEEE80211_ADDR_COPY(wh->i_addr2,
				    vap->iv_myaddr);
			} else {
				wh->i_fc[1] = IEEE80211_FC1_DIR_DSTODS;
				IEEE80211_ADDR_COPY(wh->i_addr1, da);
				IEEE80211_ADDR_COPY(wh->i_addr2,
				    vap->iv_myaddr);
				IEEE80211_ADDR_COPY(wh->i_addr3, da);
				IEEE80211_ADDR_COPY(WH4(wh)->i_addr4, sa);
			}
#endif
			break;
		case IEEE80211_M_MONITOR:	/* NB: to quiet compiler */
		case IEEE80211_M_P2P:		/* NB: to quiet compiler */
			break;
		}
	} else {
		wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
		IEEE80211_ADDR_COPY(wh->i_addr1, da);
		IEEE80211_ADDR_COPY(wh->i_addr2, sa);
#ifdef IEEE80211_SUPPORT_MESH
		if (vap->iv_opmode == IEEE80211_M_MBSS)
			IEEE80211_ADDR_COPY(wh->i_addr3, sa);
		else
#endif
			IEEE80211_ADDR_COPY(wh->i_addr3, bssid);
	}
	*(uint16_t *)&wh->i_dur[0] = 0;

	tap = &ni->ni_tx_ampdu[tid];
	if (tid != IEEE80211_NONQOS_TID && IEEE80211_AMPDU_RUNNING(tap))
		m->m_flags |= M_AMPDU_MPDU;
	else {
		seqno = ni->ni_txseqs[tid]++;
		*(uint16_t *)&wh->i_seq[0] =
		    htole16(seqno << IEEE80211_SEQ_SEQ_SHIFT);
		M_SEQNO_SET(m, seqno);
	}

	if (IEEE80211_IS_MULTICAST(wh->i_addr1))
		m->m_flags |= M_MCAST;
#undef WH4
}
#ifdef CONFIG_11W
static inline int ieee80211_is_action(__le16 fc)
{                  
        int b = (fc & cpu_to_le16(IEEE80211_FC0_TYPE_MASK | IEEE80211_FC0_SUBTYPE_MASK)) ==
               cpu_to_le16(IEEE80211_FC0_SUBTYPE_ACTION);
	return b;
}                

void onebox_prints_dump(int zone, unsigned char *vdata, int len)
{       
        unsigned short ii;
        
        for (ii=0; ii< len; ii++) {
                if (!(ii % 16) && ii) {
                        printk("\n");
                }
                printk("%02x ", vdata[ii]);
        }
        printk("\n");
}

bool ieee80211_is_robust_mgmt_frame(struct ieee80211com *ic, int type, struct mbuf *m)
{
	if ((type == IEEE80211_FC0_SUBTYPE_DEAUTH) ||
	    (type == IEEE80211_FC0_SUBTYPE_DISASSOC)) {
		return true;
	}

	onebox_prints_dump(1, m->m_data, m->m_len);

        if (type == IEEE80211_FC0_SUBTYPE_ACTION) {
		struct ieee80211_frame *wh = mtod(m, struct ieee80211_frame *);
		u8 hdrspace = ieee80211_hdrspace(ic, wh);
                u8 category = m->m_data[hdrspace];

                /*
                 * Action frames, excluding Public Action frames, are Robust
                 * Management Frames. However, if we are looking at a Protected
                 * frame, skip the check since the data may be encrypted and
                 * the frame has already been found to be a Robust Management
                 * Frame (by the other end).
                 */
                return category != IEEE80211_ACTION_CAT_HT &&
                        category != IEEE80211_ACTION_CAT_VENDOR;
        }

        return false;
}
#endif
#ifdef CONFIG_11W
void dump(int zone,unsigned char *vdata, int len )
{
        unsigned short ii;

        for(ii=0; ii< len; ii++)
        {
                if(!(ii % 16))
                {
                        printk("\n%04d: ", ii);
                }
                printk("%02x ",vdata[ii]);
        }
        printk("\n");
}
#endif
/*
 * Send a management frame to the specified node.  The node pointer
 * must have a reference as the pointer will be passed to the driver
 * and potentially held for a long time.  If the frame is successfully
 * dispatched to the driver, then it is responsible for freeing the
 * reference (and potentially free'ing up any associated storage);
 * otherwise deal with reclaiming any reference (on error).
 */
int
ieee80211_mgmt_output(struct ieee80211_node *ni, struct mbuf *m, int type,
	struct ieee80211_bpf_params *params)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct ieee80211_frame *wh;
#ifdef IEEE80211_DEBUG
	uint8_t ether_buf[18];
#endif

	KASSERT(ni != NULL, ("null node"));

	if (vap->iv_state == IEEE80211_S_CAC) {
		IEEE80211_NOTE(vap, IEEE80211_MSG_OUTPUT | IEEE80211_MSG_DOTH,
		    ni, "block %s frame in CAC state",
			ieee80211_mgt_subtype_name[
			    (type & IEEE80211_FC0_SUBTYPE_MASK) >>
				IEEE80211_FC0_SUBTYPE_SHIFT]);
		vap->iv_stats.is_tx_badstate++;
		ieee80211_free_node(ni);
		m_freem(m);
		return EIO;		/* XXX */
	}

	M_PREPEND(m, sizeof(struct ieee80211_frame), M_DONTWAIT);
	if (m == NULL) {
		ieee80211_free_node(ni);
		return ENOMEM;
	}

	wh = mtod(m, struct ieee80211_frame *);
	if(type == 0xc0 && vap->hal_priv_vap->roam_ind)
	{
		ieee80211_send_setup(ni, m,
	  	   IEEE80211_FC0_TYPE_MGT | type, IEEE80211_NONQOS_TID,
		    	vap->iv_myaddr, vap->hal_priv_vap->iv_bss_roam->ni_macaddr, vap->hal_priv_vap->iv_bss_roam->ni_macaddr);
	}
	else
	{
		ieee80211_send_setup(ni, m,
	  	   IEEE80211_FC0_TYPE_MGT | type, IEEE80211_NONQOS_TID,
	   		  vap->iv_myaddr, ni->ni_macaddr, ni->ni_bssid);
	}
#ifdef CONFIG_11W
	dump(0, m->m_data, m->m_len);

	if (ieee80211_is_robust_mgmt_frame(ic, type, m)) {
		struct ieee80211vap *vap = ni->ni_vap;

		if (vap->iv_opmode == IEEE80211_M_STA) {
		if ((!vap->hal_priv_vap->conn_in_prog) &&
		/* Connected mode is WPA2 and key has been installed + MFP is enabled */	
		    (vap->hal_priv_vap->mfp_enabled))  {
			params->ibp_flags |= IEEE80211_BPF_CRYPTO;
		} } 

		if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
			/* FIXME - Add code for broadcast frames encryption when mfp_enabled = 0 but per node mfp_enabled = 1 */
			if (!IEEE80211_ADDR_MBCAST(wh->i_addr1)) {
				if ((ni->hal_priv_node.mfp_enabled) && 
				    (!ni->hal_priv_node.conn_in_prog)) {
					params->ibp_flags |= IEEE80211_BPF_CRYPTO;
				}
			} else {
				if (vap->hal_priv_vap->mfp_enabled) {
					params->ibp_flags |= IEEE80211_BPF_CRYPTO;
				}
			}
		}
	}
#endif


	if (params->ibp_flags & IEEE80211_BPF_CRYPTO) {
		IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_AUTH, wh->i_addr1,
		    "encrypting frame (%s)", __func__);
		wh->i_fc[1] |= IEEE80211_FC1_WEP;
		ieee80211_crypto_encap(ni, m);
	}
	m->m_flags |= M_ENCAP;		/* mark encapsulated */

#ifdef CONFIG_11W
	if (vap->iv_opmode == IEEE80211_M_STA) {
		if ((type == IEEE80211_FC0_SUBTYPE_DEAUTH) ||
	    	    (type == IEEE80211_FC0_SUBTYPE_DISASSOC)) {
			vap->iv_igtk_keyix = IEEE80211_KEYIX_NONE;
			vap->iv_def_txkey = IEEE80211_KEYIX_NONE;
			ieee80211_crypto_delglobalkeys(vap);
			vap->hal_priv_vap->mfp_enabled = 0;
			vap->iv_bss->hal_priv_node.mfp_enabled = 0;
		}
	}
#endif
	KASSERT(type != IEEE80211_FC0_SUBTYPE_PROBE_RESP, ("probe response?"));
	M_WME_SETAC(m, params->ibp_pri);

#ifdef IEEE80211_DEBUG
	/* avoid printing too many frames */
	if ((ieee80211_msg_debug(vap) && doprint(vap, type)) ||
	    ieee80211_msg_dumppkts(vap)) {
		printf("[%s] send %s on channel %u\n",
		    ether_sprintf(wh->i_addr1, ether_buf),
		    ieee80211_mgt_subtype_name[
			(type & IEEE80211_FC0_SUBTYPE_MASK) >>
				IEEE80211_FC0_SUBTYPE_SHIFT],
		    ieee80211_chan2ieee(ic, ic->ic_curchan));
	}
#endif
	IEEE80211_NODE_STAT(ni, tx_mgmt);

	return ic->ic_raw_xmit(ni, m, params);
}

/*
 * Send a null data frame to the specified node.  If the station
 * is setup for QoS then a QoS Null Data frame is constructed.
 * If this is a WDS station then a 4-address frame is constructed.
 *
 * NB: the caller is assumed to have setup a node reference
 *     for use; this is necessary to deal with a race condition
 *     when probing for inactive stations.  Like ieee80211_mgmt_output
 *     we must cleanup any node reference on error;  however we
 *     can safely just unref it as we know it will never be the
 *     last reference to the node.
 */
int
ieee80211_send_nulldata(struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct mbuf *m;
	struct ieee80211_frame *wh;
	int hdrlen;
	uint8_t *frm;

	if (vap->iv_state == IEEE80211_S_CAC) {
		IEEE80211_NOTE(vap, IEEE80211_MSG_OUTPUT | IEEE80211_MSG_DOTH,
		    ni, "block %s frame in CAC state", "null data");
		ieee80211_unref_node(&ni);
		vap->iv_stats.is_tx_badstate++;
		return EIO;		/* XXX */
	}

	if (ni->ni_flags & (IEEE80211_NODE_QOS|IEEE80211_NODE_HT))
		hdrlen = sizeof(struct ieee80211_qosframe);
	else
		hdrlen = sizeof(struct ieee80211_frame);
	/* NB: only WDS vap's get 4-address frames */
	if (vap->iv_opmode == IEEE80211_M_WDS)
		hdrlen += IEEE80211_ADDR_LEN;
	if (ic->ic_flags & IEEE80211_F_DATAPAD)
		hdrlen = roundup(hdrlen, sizeof(uint32_t));

	m = ieee80211_getmgtframe(&frm, ic->ic_headroom + hdrlen, 0);
	if (m == NULL) {
		/* XXX debug msg */
		ieee80211_unref_node(&ni);
		vap->iv_stats.is_tx_nobuf++;
		return ENOMEM;
	}
	KASSERT(M_LEADINGSPACE(m) >= hdrlen,
	    ("leading space %zd", M_LEADINGSPACE(m)));
	if (!(M_PREPEND(m, hdrlen, M_DONTWAIT))) {
		m_freem(m);
		/* NB: cannot happen */
		ieee80211_free_node(ni);
		return ENOMEM;
	}

	wh = mtod(m, struct ieee80211_frame *);		/* NB: a little lie */
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, 
			("Seq is %02x %02x \n", wh->i_seq[0], wh->i_seq[1]));
	//wh->i_seq[0]=0;
	//wh->i_seq[1]=0;
	if (ni->ni_flags & IEEE80211_NODE_QOS) {
		const int tid = WME_AC_TO_TID(WME_AC_BE);
		uint8_t *qos;

		if(vap->hal_priv_vap->roam_ind)
		{
			ieee80211_send_setup(ni, m,
		  	  IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS_NULL,
		   		 tid, vap->iv_myaddr, vap->hal_priv_vap->iv_bss_roam->ni_macaddr, vap->hal_priv_vap->iv_bss_roam->ni_macaddr);
		}
		else
			ieee80211_send_setup(ni, m,
		  	  IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS_NULL,
		   	 tid, vap->iv_myaddr, ni->ni_macaddr, ni->ni_bssid);

		if (vap->iv_opmode == IEEE80211_M_WDS)
			qos = ((struct ieee80211_qosframe_addr4 *) wh)->i_qos;
		else
			qos = ((struct ieee80211_qosframe *) wh)->i_qos;
		qos[0] = tid & IEEE80211_QOS_TID;
		if (ic->ic_wme.wme_wmeChanParams.cap_wmeParams[WME_AC_BE].wmep_noackPolicy)
			qos[0] |= IEEE80211_QOS_ACKPOLICY_NOACK;
		qos[1] = 0;
		if(ni->uapsd.uapsd_flags && 
		   (ni->ni_flags & IEEE80211_NODE_PWR_MGT)) {
			qos[0] |= IEEE80211_QOS_EOSP;
			m->m_flags |= M_EOSP;
			ni->uapsd.eosp_triggered = 1;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Indicating EOSP in NULL data\n"));
		}
	} 
	else 
	{
		if(vap->hal_priv_vap->roam_ind)
		{
			//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
			//("In %s and %d ni flags=%x ni->ni_mac=%02x%02x%02x\n", __func__, __LINE__, ni->ni_flags, ni->ni_macaddr[0], ni->ni_macaddr[1], ni->ni_macaddr[5]));
			ieee80211_send_setup(ni, m,
					IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_NODATA,
					IEEE80211_NONQOS_TID,
					vap->iv_myaddr, vap->hal_priv_vap->iv_bss_roam->ni_macaddr, vap->hal_priv_vap->iv_bss_roam->ni_macaddr);
			wh->i_fc[1] |= IEEE80211_FC1_PWR_MGT;
		}
		else
		{
			ieee80211_send_setup(ni, m,
					IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_NODATA,
					IEEE80211_NONQOS_TID,
					vap->iv_myaddr, ni->ni_macaddr, ni->ni_bssid);
		}
	}
	if (vap->iv_opmode != IEEE80211_M_WDS) {
		/* NB: power management bit is never sent by an AP */
		if ((ni->ni_flags & IEEE80211_NODE_PWR_MGT) &&
		    vap->iv_opmode != IEEE80211_M_HOSTAP)
			wh->i_fc[1] |= IEEE80211_FC1_PWR_MGT;
	}
	m->m_len = m->m_pkthdr.len = hdrlen;
	m->m_flags |= M_ENCAP;		/* mark encapsulated */
	M_WME_SETAC(m, WME_AC_BE);

	IEEE80211_NODE_STAT(ni, tx_data);

	IEEE80211_NOTE(vap, IEEE80211_MSG_DEBUG | IEEE80211_MSG_DUMPPKTS, ni,
	    "send %snull data frame on channel %u, pwr mgt %s",
	    ni->ni_flags & IEEE80211_NODE_QOS ? "QoS " : "",
	    ieee80211_chan2ieee(ic, ic->ic_curchan),
	    wh->i_fc[1] & IEEE80211_FC1_PWR_MGT ? "ena" : "dis");

	return ic->ic_raw_xmit(ni, m, NULL);
}

/* 
 * Assign priority to a frame based on any vlan tag assigned
 * to the station and/or any Diffserv setting in an IP header.
 * Finally, if an ACM policy is setup (in station mode) it's
 * applied.
 */
int
ieee80211_classify(struct ieee80211_node *ni, struct mbuf *m)
{
	const struct ether_header *eh = mtod(m, struct ether_header *);
	int v_wme_ac, d_wme_ac, ac;
	int tid = 0;

	/*
	 * Always promote PAE/EAPOL frames to high priority.
	 */
	if (eh->ether_type == htons(ETHERTYPE_PAE)) {
		/* NB: mark so others don't need to check header */
		m->m_flags |= M_EAPOL;
		ac = WME_AC_VO;
		tid = 6;
		goto done;
	}
	/*
	 * Non-qos traffic goes to BE.
	 */
	if ((ni->ni_flags & IEEE80211_NODE_QOS) == 0) {
		ac = WME_AC_BE;
		tid = 0;
		goto done;
	}

	/* 
	 * If node has a vlan tag then all traffic
	 * to it must have a matching tag.
	 */
	v_wme_ac = 0;
	if (ni->ni_vlan != 0) {
		 if ((m->m_flags & M_VLANTAG) == 0) {
			IEEE80211_NODE_STAT(ni, tx_novlantag);
			return 1;
		}
		if (EVL_VLANOFTAG(m->m_pkthdr.ether_vtag) !=
		    EVL_VLANOFTAG(ni->ni_vlan)) {
			IEEE80211_NODE_STAT(ni, tx_vlanmismatch);
			return 1;
		}
		/* map vlan priority to AC */
		v_wme_ac = TID_TO_WME_AC(EVL_PRIOFTAG(ni->ni_vlan));
		tid = EVL_PRIOFTAG(ni->ni_vlan);
	}

	/* XXX m_copydata may be too slow for fast path */
#ifdef INET
	if (eh->ether_type == htons(ETHERTYPE_IP)) {
		uint8_t tos;
		/*
		 * IP frame, map the DSCP bits from the TOS field.
		 */
		/* NB: ip header may not be in first mbuf */
		m_copydata(m, sizeof(struct ether_header) +
		    offsetof(struct ip, ip_tos), sizeof(tos), &tos);
		tos >>= 5;		/* NB: ECN + low 3 bits of DSCP */
		d_wme_ac = TID_TO_WME_AC(tos);
		tid = tos;
	} else {
#endif /* INET */
#ifdef INET6
	if (eh->ether_type == htons(ETHERTYPE_IPV6)) {
		uint32_t flow;
		uint8_t tos;
		/*
		 * IPv6 frame, map the DSCP bits from the traffic class field.
		 */
		m_copydata(m, sizeof(struct ether_header) +
		    offsetof(struct ip6_hdr, ip6_flow), sizeof(flow),
		    (caddr_t) &flow);
		tos = (uint8_t)(ntohl(flow) >> 20);
		tos >>= 5;		/* NB: ECN + low 3 bits of DSCP */
		d_wme_ac = TID_TO_WME_AC(tos);
		tid = tos;
	} else {
#endif /* INET6 */
		d_wme_ac = WME_AC_BE;
		tid = 0;
#ifdef INET6
	}
#endif
#ifdef INET
	}
#endif
	/*
	 * Use highest priority AC.
	 */
	if (v_wme_ac > d_wme_ac)
		ac = v_wme_ac;
	else
		ac = d_wme_ac;

	/*
	 * Apply ACM policy.
	 */
	if (ni->ni_vap->iv_opmode == IEEE80211_M_STA) {
		const int acmap[4] = {
			WME_AC_BK,	/* WME_AC_BE */
			WME_AC_BK,	/* WME_AC_BK */
			WME_AC_BE,	/* WME_AC_VI */
			WME_AC_VI,	/* WME_AC_VO */
		};
		struct ieee80211com *ic = ni->ni_ic;

		while (ac != WME_AC_BK &&
		    ic->ic_wme.wme_wmeBssChanParams.cap_wmeParams[ac].wmep_acm)
			ac = acmap[ac];
	}
done:
	M_WME_SETAC(m, ac);
	M_WME_SETTID(m, tid);
	return 0;
}

/*
 * Insure there is sufficient contiguous space to encapsulate the
 * 802.11 data frame.  If room isn't already there, arrange for it.
 * Drivers and cipher modules assume we have done the necessary work
 * and fail rudely if they don't find the space they need.
 */
struct mbuf *
ieee80211_mbuf_adjust(struct ieee80211vap *vap, int hdrsize,
	struct ieee80211_key *key, struct mbuf *m)
{
#define	TO_BE_RECLAIMED	(sizeof(struct ether_header) - sizeof(struct llc))
	int needed_space = vap->iv_ic->ic_headroom + hdrsize;

	if (key != NULL) {
		/* XXX belongs in crypto code? */
		needed_space += key->wk_cipher->ic_header;
		/* XXX frags */
		/*
		 * When crypto is being done in the host we must insure
		 * the data are writable for the cipher routines; clone
		 * a writable mbuf chain.
		 * XXX handle SWMIC specially
		 */
		if (key->wk_flags & (IEEE80211_KEY_SWENCRYPT|IEEE80211_KEY_SWENMIC)) {
			m = m_unshare(m, M_NOWAIT);
			if (m == NULL) {
				IEEE80211_DPRINTF(vap, IEEE80211_MSG_OUTPUT,
				    "%s: cannot get writable mbuf\n", __func__);
				vap->iv_stats.is_tx_nobuf++; /* XXX new stat */
				return NULL;
			}
		}
	}
	/*
	 * We know we are called just before stripping an Ethernet
	 * header and prepending an LLC header.  This means we know
	 * there will be
	 *	sizeof(struct ether_header) - sizeof(struct llc)
	 * bytes recovered to which we need additional space for the
	 * 802.11 header and any crypto header.
	 */
	/* XXX check trailing space and copy instead? */
	if (M_LEADINGSPACE(m) < needed_space - TO_BE_RECLAIMED) {
		struct mbuf *n = m_gethdr(M_NOWAIT, m->m_type);
		if (n == NULL) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_OUTPUT,
			    "%s: cannot expand storage\n", __func__);
			vap->iv_stats.is_tx_nobuf++;
			m_freem(m);
			return NULL;
		}
		KASSERT(needed_space <= MHLEN,
		    ("not enough room, need %u got %zu\n", needed_space, MHLEN));
		/*
		 * Setup new mbuf to have leading space to prepend the
		 * 802.11 header and any crypto header bits that are
		 * required (the latter are added when the driver calls
		 * back to ieee80211_crypto_encap to do crypto encapsulation).
		 */
		/* NB: must be first 'cuz it clobbers m_data */
		m_move_pkthdr(n, m);
		n->m_len = 0;			/* NB: m_gethdr does not set */
		n->m_data += needed_space;
		/*
		 * Pull up Ethernet header to create the expected layout.
		 * We could use m_pullup but that's overkill (i.e. we don't
		 * need the actual data) and it cannot fail so do it inline
		 * for speed.
		 */
		/* NB: struct ether_header is known to be contiguous */
		n->m_len += sizeof(struct ether_header);
		m->m_len -= sizeof(struct ether_header);
		m->m_data += sizeof(struct ether_header);
		/*
		 * Replace the head of the chain.
		 */
		n->m_next = m;
		m = n;
	}
	return m;
#undef TO_BE_RECLAIMED
}

/*
 * Return the transmit key to use in sending a unicast frame.
 * If a unicast key is set we use that.  When no unicast key is set
 * we fall back to the default transmit key.
 */ 
static __inline struct ieee80211_key *
ieee80211_crypto_getucastkey(struct ieee80211vap *vap,
	struct ieee80211_node *ni)
{
	if (IEEE80211_KEY_UNDEFINED(&ni->ni_ucastkey)) {
		if (vap->iv_def_txkey == IEEE80211_KEYIX_NONE ||
		    IEEE80211_KEY_UNDEFINED(&vap->iv_nw_keys[vap->iv_def_txkey]))
			return NULL;
		return &vap->iv_nw_keys[vap->iv_def_txkey];
	} else {
		return &ni->ni_ucastkey;
	}
}

/*
 * Return the transmit key to use in sending a multicast frame.
 * Multicast traffic always uses the group key which is installed as
 * the default tx key.
 */ 
static __inline struct ieee80211_key *
ieee80211_crypto_getmcastkey(struct ieee80211vap *vap,
	struct ieee80211_node *ni)
{
	if (vap->iv_def_txkey == IEEE80211_KEYIX_NONE ||
	    IEEE80211_KEY_UNDEFINED(&vap->iv_nw_keys[vap->iv_def_txkey]))
		return NULL;
	return &vap->iv_nw_keys[vap->iv_def_txkey];
}

/*
 * Encapsulate an outbound data frame.  The mbuf chain is updated.
 * If an error is encountered NULL is returned.  The caller is required
 * to provide a node reference and pullup the ethernet header in the
 * first mbuf.
 *
 * NB: Packet is assumed to be processed by ieee80211_classify which
 *     marked EAPOL frames w/ M_EAPOL.
 */
struct mbuf *
ieee80211_encap(struct ieee80211vap *vap, struct ieee80211_node *ni,
    struct mbuf *m)
{
#define	WH4(wh)	((struct ieee80211_frame_addr4 *)(wh))
	struct ieee80211com *ic = ni->ni_ic;
#ifdef IEEE80211_SUPPORT_MESH
	struct ieee80211_mesh_state *ms = vap->iv_mesh;
	struct ieee80211_meshcntl_ae10 *mc;
#endif
	struct ether_header eh;
	struct ieee80211_frame *wh;
	struct ieee80211_key *key;
	struct llc *llc;
	int hdrsize, hdrspace, datalen, addqos, txfrag, is4addr;
	//ieee80211_seq seqno;
	int meshhdrsize, meshae;
	uint8_t *qos;

	/*
	 * Copy existing Ethernet header to a safe place.  The
	 * rest of the code assumes it's ok to strip it when
	 * reorganizing state for the final encapsulation.
	 */
	KASSERT(m->m_len >= sizeof(eh), ("no ethernet header!"));
	ETHER_HEADER_COPY(&eh, mtod(m, caddr_t));

	/*
	 * Insure space for additional headers.  First identify
	 * transmit key to use in calculating any buffer adjustments
	 * required.  This is also used below to do privacy
	 * encapsulation work.  Then calculate the 802.11 header
	 * size and any padding required by the driver.
	 *
	 * Note key may be NULL if we fall back to the default
	 * transmit key and that is not set.  In that case the
	 * buffer may not be expanded as needed by the cipher
	 * routines, but they will/should discard it.
	 */
	if (vap->iv_flags & IEEE80211_F_PRIVACY) {
		if (vap->iv_opmode == IEEE80211_M_STA ||
		    !IEEE80211_IS_MULTICAST(eh.ether_dhost) ||
		    (vap->iv_opmode == IEEE80211_M_WDS &&
		     (vap->iv_flags_ext & IEEE80211_FEXT_WDSLEGACY)))
			key = ieee80211_crypto_getucastkey(vap, ni);
		else
			key = ieee80211_crypto_getmcastkey(vap, ni);
		if (key == NULL && (m->m_flags & M_EAPOL) == 0) {
			IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_CRYPTO,
			    eh.ether_dhost,
			    "no default transmit key (%s) deftxkey %u",
			    __func__, vap->iv_def_txkey);
			vap->iv_stats.is_tx_nodefkey++;
			goto bad;
		}
	} else
		key = NULL;
	/*
	 * XXX Some ap's don't handle QoS-encapsulated EAPOL
	 * frames so suppress use.  This may be an issue if other
	 * ap's require all data frames to be QoS-encapsulated
	 * once negotiated in which case we'll need to make this
	 * configurable.
	 * NB: mesh data frames are QoS.
	 */
	addqos = ((ni->ni_flags & (IEEE80211_NODE_QOS|IEEE80211_NODE_HT)) ||
	    (vap->iv_opmode == IEEE80211_M_MBSS)) &&
	    (m->m_flags & M_EAPOL) == 0;
	if (addqos)
		hdrsize = sizeof(struct ieee80211_qosframe);
	else
		hdrsize = sizeof(struct ieee80211_frame);
#ifdef IEEE80211_SUPPORT_MESH
	if (vap->iv_opmode == IEEE80211_M_MBSS) {
		/*
		 * Mesh data frames are encapsulated according to the
		 * rules of Section 11B.8.5 (p.139 of D3.0 spec).
		 * o Group Addressed data (aka multicast) originating
		 *   at the local sta are sent w/ 3-address format and
		 *   address extension mode 00
		 * o Individually Addressed data (aka unicast) originating
		 *   at the local sta are sent w/ 4-address format and
		 *   address extension mode 00
		 * o Group Addressed data forwarded from a non-mesh sta are
		 *   sent w/ 3-address format and address extension mode 01
		 * o Individually Address data from another sta are sent
		 *   w/ 4-address format and address extension mode 10
		 */
		is4addr = 0;		/* NB: don't use, disable */
		if (!IEEE80211_IS_MULTICAST(eh.ether_dhost))
			hdrsize += IEEE80211_ADDR_LEN;	/* unicast are 4-addr */
		meshhdrsize = sizeof(struct ieee80211_meshcntl);
		/* XXX defines for AE modes */
		if (IEEE80211_ADDR_EQ(eh.ether_shost, vap->iv_myaddr)) {
			if (!IEEE80211_IS_MULTICAST(eh.ether_dhost))
				meshae = 0;
			else
				meshae = 4;		/* NB: pseudo */
		} else if (IEEE80211_IS_MULTICAST(eh.ether_dhost)) {
			meshae = 1;
			meshhdrsize += 1*IEEE80211_ADDR_LEN;
		} else {
			meshae = 2;
			meshhdrsize += 2*IEEE80211_ADDR_LEN;
		}
	} else {
#endif
		/*
		 * 4-address frames need to be generated for:
		 * o packets sent through a WDS vap (IEEE80211_M_WDS)
		 * o packets sent through a vap marked for relaying
		 *   (e.g. a station operating with dynamic WDS)
		 */
		is4addr = vap->iv_opmode == IEEE80211_M_WDS ||
		    ((vap->iv_flags_ext & IEEE80211_FEXT_4ADDR) &&
		     !IEEE80211_ADDR_EQ(eh.ether_shost, vap->iv_myaddr));
		if (is4addr)
			hdrsize += IEEE80211_ADDR_LEN;
		meshhdrsize = meshae = 0;
#ifdef IEEE80211_SUPPORT_MESH
	}
#endif
	/*
	 * Honor driver DATAPAD requirement.
	 */
	if (ic->ic_flags & IEEE80211_F_DATAPAD)
		hdrspace = roundup(hdrsize, sizeof(uint32_t));
	else
		hdrspace = hdrsize;

	if (__predict_true((m->m_flags & M_FF) == 0)) {
		/*
		 * Normal frame.
		 */
		m = ieee80211_mbuf_adjust(vap, hdrspace + meshhdrsize, key, m);
		if (m == NULL) {
			/* NB: ieee80211_mbuf_adjust handles msgs+statistics */
			goto bad;
		}
		/* NB: this could be optimized 'cuz of ieee80211_mbuf_adjust */
		m_adj(m, sizeof(struct ether_header) - sizeof(struct llc));
		llc = mtod(m, struct llc *);
		llc->llc_dsap = llc->llc_ssap = LLC_SNAP_LSAP;
		llc->llc_control = LLC_UI;
		llc->llc_snap.org_code[0] = 0;
		llc->llc_snap.org_code[1] = 0;
		llc->llc_snap.org_code[2] = 0;
		llc->llc_snap.ether_type = eh.ether_type;
	} else {
#ifdef IEEE80211_SUPPORT_SUPERG
		/*
		 * Aggregated frame.
		 */
		m = ieee80211_ff_encap(vap, m, hdrspace + meshhdrsize, key);
		if (m == NULL)
#endif
			goto bad;
	}
	datalen = m->m_pkthdr.len;		/* NB: w/o 802.11 header */

	if (!(M_PREPEND(m, hdrspace + meshhdrsize, M_DONTWAIT))) {
		m_freem(m);
		vap->iv_stats.is_tx_nobuf++;
		goto bad;
	}
	wh = mtod(m, struct ieee80211_frame *);
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_DATA;
	*(uint16_t *)wh->i_dur = 0;
	qos = NULL;	/* NB: quiet compiler */
	if (is4addr) {
		wh->i_fc[1] = IEEE80211_FC1_DIR_DSTODS;
		IEEE80211_ADDR_COPY(wh->i_addr1, ni->ni_macaddr);
		IEEE80211_ADDR_COPY(wh->i_addr2, vap->iv_myaddr);
		IEEE80211_ADDR_COPY(wh->i_addr3, eh.ether_dhost);
		IEEE80211_ADDR_COPY(WH4(wh)->i_addr4, eh.ether_shost);
	} else switch (vap->iv_opmode) {
	case IEEE80211_M_STA:
		wh->i_fc[1] = IEEE80211_FC1_DIR_TODS;
		IEEE80211_ADDR_COPY(wh->i_addr1, ni->ni_bssid);
		IEEE80211_ADDR_COPY(wh->i_addr2, eh.ether_shost);
		IEEE80211_ADDR_COPY(wh->i_addr3, eh.ether_dhost);
		break;
	case IEEE80211_M_IBSS:
	case IEEE80211_M_AHDEMO:
		wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
		IEEE80211_ADDR_COPY(wh->i_addr1, eh.ether_dhost);
		IEEE80211_ADDR_COPY(wh->i_addr2, eh.ether_shost);
		/*
		 * NB: always use the bssid from iv_bss as the
		 *     neighbor's may be stale after an ibss merge
		 */
		IEEE80211_ADDR_COPY(wh->i_addr3, vap->iv_bss->ni_bssid);
		break;
	case IEEE80211_M_HOSTAP:
		wh->i_fc[1] = IEEE80211_FC1_DIR_FROMDS;
		IEEE80211_ADDR_COPY(wh->i_addr1, eh.ether_dhost);
		IEEE80211_ADDR_COPY(wh->i_addr2, ni->ni_bssid);
		IEEE80211_ADDR_COPY(wh->i_addr3, eh.ether_shost);
		break;
#ifdef IEEE80211_SUPPORT_MESH
	case IEEE80211_M_MBSS:
		/* NB: offset by hdrspace to deal with DATAPAD */
		mc = (struct ieee80211_meshcntl_ae10 *)
		     (mtod(m, uint8_t *) + hdrspace);
		switch (meshae) {
		case 0:			/* ucast, no proxy */
			wh->i_fc[1] = IEEE80211_FC1_DIR_DSTODS;
			IEEE80211_ADDR_COPY(wh->i_addr1, ni->ni_macaddr);
			IEEE80211_ADDR_COPY(wh->i_addr2, vap->iv_myaddr);
			IEEE80211_ADDR_COPY(wh->i_addr3, eh.ether_dhost);
			IEEE80211_ADDR_COPY(WH4(wh)->i_addr4, eh.ether_shost);
			mc->mc_flags = 0;
			qos = ((struct ieee80211_qosframe_addr4 *) wh)->i_qos;
			break;
		case 4:			/* mcast, no proxy */
			wh->i_fc[1] = IEEE80211_FC1_DIR_FROMDS;
			IEEE80211_ADDR_COPY(wh->i_addr1, eh.ether_dhost);
			IEEE80211_ADDR_COPY(wh->i_addr2, vap->iv_myaddr);
			IEEE80211_ADDR_COPY(wh->i_addr3, eh.ether_shost);
			mc->mc_flags = 0;		/* NB: AE is really 0 */
			qos = ((struct ieee80211_qosframe *) wh)->i_qos;
			break;
		case 1:			/* mcast, proxy */
			wh->i_fc[1] = IEEE80211_FC1_DIR_FROMDS;
			IEEE80211_ADDR_COPY(wh->i_addr1, eh.ether_dhost);
			IEEE80211_ADDR_COPY(wh->i_addr2, vap->iv_myaddr);
			IEEE80211_ADDR_COPY(wh->i_addr3, vap->iv_myaddr);
			mc->mc_flags = 1;
			IEEE80211_ADDR_COPY(mc->mc_addr4, eh.ether_shost);
			qos = ((struct ieee80211_qosframe *) wh)->i_qos;
			break;
		case 2:			/* ucast, proxy */
			wh->i_fc[1] = IEEE80211_FC1_DIR_DSTODS;
			IEEE80211_ADDR_COPY(wh->i_addr1, ni->ni_macaddr);
			IEEE80211_ADDR_COPY(wh->i_addr2, vap->iv_myaddr);
			/* XXX not right, need MeshDA */
			IEEE80211_ADDR_COPY(wh->i_addr3, eh.ether_dhost);
			/* XXX assume are MeshSA */
			IEEE80211_ADDR_COPY(WH4(wh)->i_addr4, vap->iv_myaddr);
			mc->mc_flags = 2;
			IEEE80211_ADDR_COPY(mc->mc_addr4, eh.ether_dhost);
			IEEE80211_ADDR_COPY(mc->mc_addr5, eh.ether_shost);
			qos = ((struct ieee80211_qosframe_addr4 *) wh)->i_qos;
			break;
		default:
			KASSERT(0, ("meshae %d", meshae));
			break;
		}
		mc->mc_ttl = ms->ms_ttl;
		ms->ms_seq++;
		LE_WRITE_4(mc->mc_seq, ms->ms_seq);
		break;
#endif
	case IEEE80211_M_WDS:		/* NB: is4addr should always be true */
	default:
		goto bad;
	}
	if (m->m_flags & M_MORE_DATA)
		wh->i_fc[1] |= IEEE80211_FC1_MORE_DATA;
	if (addqos) {
		int ac, tid;

		if (is4addr) {
			qos = ((struct ieee80211_qosframe_addr4 *) wh)->i_qos;
		/* NB: mesh case handled earlier */
		} else if (vap->iv_opmode != IEEE80211_M_MBSS)
			qos = ((struct ieee80211_qosframe *) wh)->i_qos;
		ac = M_WME_GETAC(m);
		/* map from access class/queue to 11e header priorty value */
		tid = WME_AC_TO_TID(ac);
		qos[0] = tid & IEEE80211_QOS_TID;
		if (ic->ic_wme.wme_wmeChanParams.cap_wmeParams[ac].wmep_noackPolicy)
			qos[0] |= IEEE80211_QOS_ACKPOLICY_NOACK;
#ifdef IEEE80211_SUPPORT_MESH
		if (vap->iv_opmode == IEEE80211_M_MBSS) {
			qos[1] |= IEEE80211_QOS_MC;
		} else
#endif
			qos[1] = 0;
		wh->i_fc[0] |= IEEE80211_FC0_SUBTYPE_QOS;

		if ((m->m_flags & M_AMPDU_MPDU) == 0) {
			/*
			 * NB: don't assign a sequence # to potential
			 * aggregates; we expect this happens at the
			 * point the frame comes off any aggregation q
			 * as otherwise we may introduce holes in the
			 * BA sequence space and/or make window accouting
			 * more difficult.
			 *
			 * XXX may want to control this with a driver
			 * capability; this may also change when we pull
			 * aggregation up into net80211
			 */
			//seqno = ni->ni_txseqs[tid]++;
			//*(uint16_t *)wh->i_seq =
			    //htole16(seqno << IEEE80211_SEQ_SEQ_SHIFT);
			//M_SEQNO_SET(m, seqno);
		}
	} else {
		//seqno = ni->ni_txseqs[IEEE80211_NONQOS_TID]++;
		//*(uint16_t *)wh->i_seq =
		    //htole16(seqno << IEEE80211_SEQ_SEQ_SHIFT);
		//M_SEQNO_SET(m, seqno);
	}


	/* check if xmit fragmentation is required */
	txfrag = (m->m_pkthdr.len > vap->iv_fragthreshold &&
	    !IEEE80211_IS_MULTICAST(wh->i_addr1) &&
	    (vap->iv_caps & IEEE80211_C_TXFRAG) &&
	    (m->m_flags & (M_FF | M_AMPDU_MPDU)) == 0);
	if (key != NULL) {
		/*
		 * IEEE 802.1X: send EAPOL frames always in the clear.
		 * WPA/WPA2: encrypt EAPOL keys when pairwise keys are set.
		 */
		if ((m->m_flags & M_EAPOL) == 0 ||
		    ((vap->iv_flags & IEEE80211_F_WPA) &&
		     (vap->iv_opmode == IEEE80211_M_STA ?
		      !IEEE80211_KEY_UNDEFINED(key) :
		      !IEEE80211_KEY_UNDEFINED(&ni->ni_ucastkey)))) {
			wh->i_fc[1] |= IEEE80211_FC1_WEP;
			if (!ieee80211_crypto_enmic(vap, key, m, txfrag)) {
				IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_OUTPUT,
				    eh.ether_dhost,
				    "%s", "enmic failed, discard frame");
				vap->iv_stats.is_crypto_enmicfail++;
				goto bad;
			}
		}
	}
	if (txfrag && !ieee80211_fragment(vap, m, hdrsize,
	    key != NULL ? key->wk_cipher->ic_header : 0, vap->iv_fragthreshold))
		goto bad;

	m->m_flags |= M_ENCAP;		/* mark encapsulated */

	IEEE80211_NODE_STAT(ni, tx_data);
	if (IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		IEEE80211_NODE_STAT(ni, tx_mcast);
		m->m_flags |= M_MCAST;
	} else
		IEEE80211_NODE_STAT(ni, tx_ucast);
	IEEE80211_NODE_STAT_ADD(ni, tx_bytes, datalen);

	return m;
bad:
	if (m != NULL)
		m_freem(m);
	return NULL;
#undef WH4
}

/*
 * Fragment the frame according to the specified mtu.
 * The size of the 802.11 header (w/o padding) is provided
 * so we don't need to recalculate it.  We create a new
 * mbuf for each fragment and chain it through m_nextpkt;
 * we might be able to optimize this by reusing the original
 * packet's mbufs but that is significantly more complicated.
 */
static int
ieee80211_fragment(struct ieee80211vap *vap, struct mbuf *m0,
	u_int hdrsize, u_int ciphdrsize, u_int mtu)
{
	struct ieee80211_frame *wh, *whf;
	struct mbuf *m, *prev, *next;
	u_int totalhdrsize, fragno, fragsize, off, remainder, payload;

	KASSERT(m0->m_nextpkt == NULL, ("mbuf already chained?"));
	KASSERT(m0->m_pkthdr.len > mtu,
		("pktlen %u mtu %u", m0->m_pkthdr.len, mtu));

	wh = mtod(m0, struct ieee80211_frame *);
	/* NB: mark the first frag; it will be propagated below */
	wh->i_fc[1] |= IEEE80211_FC1_MORE_FRAG;
	totalhdrsize = hdrsize + ciphdrsize;
	fragno = 1;
	off = mtu - ciphdrsize;
	remainder = m0->m_pkthdr.len - off;
	prev = m0;
	do {
		fragsize = totalhdrsize + remainder;
		if (fragsize > mtu)
			fragsize = mtu;
		/* XXX fragsize can be >2048! */
		KASSERT(fragsize < MCLBYTES,
			("fragment size %u too big!", fragsize));
		if (fragsize > MHLEN)
			m = m_getcl(M_DONTWAIT, MT_DATA, M_PKTHDR);
		else
			m = m_gethdr(M_DONTWAIT, MT_DATA);
		if (m == NULL)
			goto bad;
		/* leave room to prepend any cipher header */
		m_align(m, fragsize - ciphdrsize);

		/*
		 * Form the header in the fragment.  Note that since
		 * we mark the first fragment with the MORE_FRAG bit
		 * it automatically is propagated to each fragment; we
		 * need only clear it on the last fragment (done below).
		 * NB: frag 1+ dont have Mesh Control field present.
		 */
		whf = mtod(m, struct ieee80211_frame *);
		memcpy(whf, wh, hdrsize);
#ifdef IEEE80211_SUPPORT_MESH
		if (vap->iv_opmode == IEEE80211_M_MBSS) {
			if (IEEE80211_IS_DSTODS(wh))
				((struct ieee80211_qosframe_addr4 *)
				    whf)->i_qos[1] &= ~IEEE80211_QOS_MC;
			else
				((struct ieee80211_qosframe *)
				    whf)->i_qos[1] &= ~IEEE80211_QOS_MC;
		}
#endif
		*(uint16_t *)&whf->i_seq[0] |= htole16(
			(fragno & IEEE80211_SEQ_FRAG_MASK) <<
				IEEE80211_SEQ_FRAG_SHIFT);
		fragno++;

		payload = fragsize - totalhdrsize;
		/* NB: destination is known to be contiguous */
		m_copydata(m0, off, payload, mtod(m, uint8_t *) + hdrsize);
		m->m_len = hdrsize + payload;
		m->m_pkthdr.len = hdrsize + payload;
		m->m_flags |= M_FRAG;

		/* chain up the fragment */
		prev->m_nextpkt = m;
		prev = m;

		/* deduct fragment just formed */
		remainder -= payload;
		off += payload;
	} while (remainder != 0);

	/* set the last fragment */
	m->m_flags |= M_LASTFRAG;
	whf->i_fc[1] &= ~IEEE80211_FC1_MORE_FRAG;

	/* strip first mbuf now that everything has been copied */
	m_adj(m0, -(m0->m_pkthdr.len - (mtu - ciphdrsize)));
	m0->m_flags |= M_FIRSTFRAG | M_FRAG;

	vap->iv_stats.is_tx_fragframes++;
	vap->iv_stats.is_tx_frags += fragno-1;

	return 1;
bad:
	/* reclaim fragments but leave original frame for caller to free */
	for (m = m0->m_nextpkt; m != NULL; m = next) {
		next = m->m_nextpkt;
		m->m_nextpkt = NULL;		/* XXX paranoid */
		m_freem(m);
	}
	m0->m_nextpkt = NULL;
	return 0;
}

#ifdef ENABLE_P2P_SUPPORT
/*
 * Add a supported rates element id to a frame.
 */
static uint8_t *
ieee80211_add_only_11g_rates(uint8_t *frm, const struct ieee80211_rateset *rs)
{ /* This should be done when assigning rates to rs itself */ 
	int nrates, i;
	uint8_t *len_loc;

	*frm++ = IEEE80211_ELEMID_RATES;
	len_loc = frm++; /* Leave one byte for length */
	nrates = rs->rs_nrates;

	for(i = 0; i < nrates; i++)
	{
		uint8_t rate = rs->rs_rates[i];

		if(((rate & IEEE80211_RATE_VAL) == 2) || 
		   ((rate & IEEE80211_RATE_VAL) == 4) || 
		   ((rate & IEEE80211_RATE_VAL) == 11) || 
		   ((rate & IEEE80211_RATE_VAL) == 22))
		{ /* Dont add if 11b rate */
			continue;
		}
		if(((rate & IEEE80211_RATE_VAL) == 12) || 
		   ((rate & IEEE80211_RATE_VAL) == 24) || 
		   ((rate & IEEE80211_RATE_VAL) == 48))
		{ /* for now indicating 12, 24, 48 as basic rates */
			*frm++ = rate | IEEE80211_RATE_BASIC;
		}
		else
		{
			*frm++ = rate;
		}
		if ((frm - len_loc) > IEEE80211_RATE_SIZE)
		{
			break;
		}
	}
	*len_loc = (uint8_t)(frm - len_loc - 1);
	return frm;
}
#endif



static uint8_t *
ieee80211_add_only_11b_rates(uint8_t *frm, const struct ieee80211_rateset *rs)
{ /* This should be done when assigning rates to rs itself */ 
	int nrates, i;
	uint8_t *len_loc;

	*frm++ = IEEE80211_ELEMID_RATES;
	len_loc = frm++; /* Leave one byte for length */
	nrates = rs->rs_nrates;

	for(i = 0; i < nrates; i++)
	{
		uint8_t rate = rs->rs_rates[i];

		if(((rate & IEEE80211_RATE_VAL) == 2) || 
		   ((rate & IEEE80211_RATE_VAL) == 4) || 
		   ((rate & IEEE80211_RATE_VAL) == 11) || 
		   ((rate & IEEE80211_RATE_VAL) == 22))
		{/*Adding only 11b rates in basic rate set*/ 
			*frm++ = rate | IEEE80211_RATE_BASIC;
		}
	
		if ((frm - len_loc) > IEEE80211_RATE_SIZE)
		{
			break;
		}
	}
	*len_loc = (uint8_t)(frm - len_loc - 1);
	return frm;
}



/*
 * Add a supported rates element id to a frame.
 */
uint8_t *
ieee80211_add_rates(uint8_t *frm, const struct ieee80211_rateset *rs)
{
	int nrates;

	*frm++ = IEEE80211_ELEMID_RATES;
	nrates = rs->rs_nrates;
	if (nrates > IEEE80211_RATE_SIZE)
		nrates = IEEE80211_RATE_SIZE;
	*frm++ = nrates;
	memcpy(frm, rs->rs_rates, nrates);
	return frm + nrates;
}

#ifdef CONFIG_11W
#define ADDWORD(frm, v) do {		\
	LE_WRITE_4(frm, v);		\
	frm += 4;			\
} while (0)


/*
 * Add a supported rates element id to a frame.
 */
uint8_t *
ieee80211_add_timeout_ie(uint8_t *frm)
{
	uint32_t timeout = 1000;

	*frm++ = IEEE80211_ELEMID_TIMEOUT_INTERVAL;
	*frm++ = 5;
	*frm++ = IEEE80211_TIMEOUT_ASSOC_COMEBACK;
	ADDWORD(frm, timeout);	
	/* FIXME */
	return frm;
}
#endif
/*
 * Add an extended supported rates element id to a frame.
 */
uint8_t *
ieee80211_add_xrates(uint8_t *frm, const struct ieee80211_rateset *rs)
{
	/*
	 * Add an extended supported rates element if operating in 11g mode.
	 */
	if (rs->rs_nrates > IEEE80211_RATE_SIZE) {
		int nrates = rs->rs_nrates - IEEE80211_RATE_SIZE;
		*frm++ = IEEE80211_ELEMID_XRATES;
		*frm++ = nrates;
		memcpy(frm, rs->rs_rates + IEEE80211_RATE_SIZE, nrates);
		frm += nrates;
	}
	return frm;
}

/* 
 * Add an ssid element to a frame.
 */
static uint8_t *
ieee80211_add_ssid(uint8_t *frm, const uint8_t *ssid, u_int len)
{
	*frm++ = IEEE80211_ELEMID_SSID;
	*frm++ = len;
	memcpy(frm, ssid, len);
	return frm + len;
}

#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
/* 
 * Add an mdie element to a frame.
 */
static uint8_t *
ieee80211_add_mdie(uint8_t *frm, const uint8_t *mdie, u_int len)
{
	*frm++ = IEEE80211_ELEMID_MDIE;
	*frm++ = len;
	memcpy(frm, mdie, len);
	return frm + len;
}
#endif //CONFIG_11R

/*
 * Add an erp element to a frame.
 */
static uint8_t *
ieee80211_add_erp(uint8_t *frm, struct ieee80211com *ic)
{
	uint8_t erp;

	*frm++ = IEEE80211_ELEMID_ERP;
	*frm++ = 1;
	erp = 0;
	if (ic->ic_nonerpsta != 0)
		erp |= IEEE80211_ERP_NON_ERP_PRESENT;
	if (ic->ic_flags & IEEE80211_F_USEPROT)
		erp |= IEEE80211_ERP_USE_PROTECTION;
	if (ic->ic_flags & IEEE80211_F_USEBARKER)
		erp |= IEEE80211_ERP_LONG_PREAMBLE;
	*frm++ = erp;
	return frm;
}

/*
 * Add a CFParams element to a frame.
 */
static uint8_t *
ieee80211_add_cfparms(uint8_t *frm, struct ieee80211com *ic)
{
#define	ADDSHORT(frm, v) do {	\
	LE_WRITE_2(frm, v);	\
	frm += 2;		\
} while (0)
	*frm++ = IEEE80211_ELEMID_CFPARMS;
	*frm++ = 6;
	*frm++ = 0;		/* CFP count */
	*frm++ = 2;		/* CFP period */
	ADDSHORT(frm, 0);	/* CFP MaxDuration (TU) */
	ADDSHORT(frm, 0);	/* CFP CurRemaining (TU) */
	return frm;
#undef ADDSHORT
}

static __inline uint8_t *
add_appie(uint8_t *frm, const struct ieee80211_appie *ie)
{
	if(frm == NULL)
  	{
		return NULL;
	}
	memcpy(frm, ie->ie_data, ie->ie_len);
	return frm + ie->ie_len;
}

static __inline uint8_t *
add_proberesp_appie(uint8_t *frm, const struct ieee80211_appie *ie, int len)
{
	memcpy(frm, ie->ie_data, len);
	return frm + len;
}

static __inline uint8_t *
add_ie(uint8_t *frm, const uint8_t *ie)
{
	memcpy(frm, ie, 2 + ie[1]);
	return frm + 2 + ie[1];
}

#define	WME_OUI_BYTES		0x00, 0x50, 0xf2
/*
 * Add a WME information element to a frame.
 */
static uint8_t *
ieee80211_add_wme_info(uint8_t *frm, struct ieee80211_wme_state *wme)
{
	const struct ieee80211_wme_info info = {
		.wme_id		= IEEE80211_ELEMID_VENDOR,
		.wme_len	= sizeof(struct ieee80211_wme_info) - 2,
		.wme_oui	= { WME_OUI_BYTES },
		.wme_type	= WME_OUI_TYPE,
		.wme_subtype	= WME_INFO_OUI_SUBTYPE,
		.wme_version	= WME_VERSION,
		.wme_info	= 0,
	};
	memcpy(frm, &info, sizeof(info));
	return frm + sizeof(info); 
}

/*
 * Add a WME parameters element to a frame.
 */
static uint8_t *
ieee80211_add_wme_param(uint8_t *frm, struct ieee80211_wme_state *wme)
{
#define	SM(_v, _f)	(((_v) << _f##_S) & _f)
#define	ADDSHORT(frm, v) do {	\
	LE_WRITE_2(frm, v);	\
	frm += 2;		\
} while (0)
	/* NB: this works 'cuz a param has an info at the front */
	const struct ieee80211_wme_info param = {
		.wme_id		= IEEE80211_ELEMID_VENDOR,
		.wme_len	= sizeof(struct ieee80211_wme_param) - 2,
		.wme_oui	= { WME_OUI_BYTES },
		.wme_type	= WME_OUI_TYPE,
		.wme_subtype	= WME_PARAM_OUI_SUBTYPE,
		.wme_version	= WME_VERSION,
	};
	int i;

	memcpy(frm, &param, sizeof(param));
	frm += __offsetof(struct ieee80211_wme_info, wme_info);
	*frm++ = (wme->wme_bssChanParams.cap_info | 0xFF <<4);	/* AC info */
	*frm++ = 0;					/* reserved field */
	for (i = 0; i < WME_NUM_AC; i++) {
		const struct wmeParams *ac =
		       &wme->wme_bssChanParams.cap_wmeParams[i];
		*frm++ = SM(i, WME_PARAM_ACI)
		       | SM(ac->wmep_acm, WME_PARAM_ACM)
		       | SM(ac->wmep_aifsn, WME_PARAM_AIFSN)
		       ;
		*frm++ = SM(ac->wmep_logcwmax, WME_PARAM_LOGCWMAX)
		       | SM(ac->wmep_logcwmin, WME_PARAM_LOGCWMIN)
		       ;
		ADDSHORT(frm, ac->wmep_txopLimit); /*params to fill in; now all 0's are being filled */
	}
	return frm;
#undef SM
#undef ADDSHORT
}
#undef WME_OUI_BYTES

#ifdef RSI_CCX

/*
 * Adding a CCX Information (Re)Assoc Request 
 */
#define	CCX_OUI_BYTES		0x00, 0x40, 0x96
#define CCX_VERSION_IE 		0x03
static uint8_t *
ieee80211_add_ccx_info(uint8_t *frm)
{
	const struct ieee80211_ccx_info info = {
		.ccx_id		= IEEE80211_ELEMID_VENDOR,
		.ccx_len	= sizeof(struct ieee80211_ccx_info) - 2,
		.ccx_oui	= { CCX_OUI_BYTES },
		.version	= CCX_VERSION_IE,
		.ccx_version	= 0x02, //5
	};
	memcpy(frm, &info, sizeof(info));
	return frm + sizeof(info); 
}
/*
 * Adding Aironet Cisco Proprietary IE Captured from Probe response
 */
static uint8_t *
ieee80211_add_aironet_ie(uint8_t *frm,struct ieee80211_aironet_ie *ni_ies)
{
	memcpy(frm, ni_ies, sizeof(struct ieee80211_aironet_ie));
	return frm + sizeof(struct ieee80211_aironet_ie); 
}

/*
 * Adding Aironet CCX CCKM IE 
 */
#define	CCX_CCKM_IE		156
static uint8_t *
ieee80211_add_cckm_info(uint8_t *frm, struct ieee80211vap *vap) 
{
	uint8_t aironet_cckm_oui[] = {0x0, 0x40, 0x96};
	frm[0] = CCX_CCKM_IE;
	frm[1] = ((sizeof(struct ieee80211_aironet_cckm_ie))-2);
	memcpy(&frm[2], aironet_cckm_oui, sizeof(aironet_cckm_oui));
	frm[5] = 0;
	*(uint64_t *)&frm[6] = vap->cckm_ie.aironet_cckm_tsf;
	*(uint32_t *)&frm[14] = vap->cckm_ie.aironet_cckm_rn;
	*(uint64_t *)&frm[18] = vap->cckm_ie.aironet_cckm_mic;
	return frm+sizeof(struct ieee80211_aironet_cckm_ie);

}

/*
 * Adding Aironet Radio Cap-Management IE 
 */
static uint8_t *
ieee80211_add_ccx_radio_cap_ie(uint8_t *frm, struct ccx_radio_cap_ie *ni_ccx_radio_cap_ie)
{
	memcpy(frm, ni_ccx_radio_cap_ie, sizeof(struct ccx_radio_cap_ie));
	return frm +sizeof(struct ccx_radio_cap_ie); 
}

/*
 * Adding Aironet AP IP IE 
 */
#define CCX_AP_IP_IE  149 
static uint8_t *
ieee80211_add_aironet_ap_ip_ie(uint8_t *frm,struct ieee80211_aironet_ap_ip_ie *ni_ies)
{
	uint8_t aironet_ie[] = {0x00, 0x40, 0x96};
	frm[0] = CCX_AP_IP_IE;
	frm[1] = (sizeof(struct ieee80211_aironet_ap_ip_ie) -2);
	memcpy(&frm[2], aironet_ie, sizeof(aironet_ie) );
	return frm+12;
}

#endif

/*
 * Add an 11h Power Constraint element to a frame.
 */
static uint8_t *
ieee80211_add_powerconstraint(uint8_t *frm, struct ieee80211vap *vap)
{
	const struct ieee80211_channel *c = vap->iv_bss->ni_chan;
	/* XXX per-vap tx power limit? */
	int8_t limit = vap->iv_ic->ic_txpowlimit / 2;

	frm[0] = IEEE80211_ELEMID_PWRCNSTR;
	frm[1] = 1;
	frm[2] = c->ic_maxregpower > limit ?  c->ic_maxregpower - limit : 0;
	return frm + 3;
}

/*
 * Add an 11h Power Capability element to a frame.
 */
static uint8_t *
ieee80211_add_powercapability(uint8_t *frm, const struct ieee80211_channel *c)
{
	frm[0] = IEEE80211_ELEMID_PWRCAP;
	frm[1] = 2;
	frm[2] = c->ic_minpower;
	frm[3] = c->ic_maxregpower;
	return frm + 4;
}

/*
 * Add an 11h Supported Channels element to a frame.
 */
static uint8_t *
ieee80211_add_supportedchannels(uint8_t *frm, struct ieee80211com *ic)
{
	if (ic->ic_supported_chan == NULL) {
		/*
		 * Handle lazy construction of ie.  This is done on
		 * first use and after a channel change that requires
		 * re-calculation.
		 */
		if (ic->ic_supported_chan != NULL)
			free(ic->ic_supported_chan, M_80211_NODE_IE);
		ic->ic_supported_chan = ieee80211_alloc_supported_chan(ic);
		if (ic->ic_supported_chan == NULL)
			return frm;
	}
	return add_appie(frm, ic->ic_supported_chan);
}

/*
 * Add an 11h Quiet time element to a frame.
 */
static uint8_t *
ieee80211_add_quiet(uint8_t *frm, struct ieee80211vap *vap)
{
	struct ieee80211_quiet_ie *quiet = (struct ieee80211_quiet_ie *) frm;

	quiet->quiet_ie = IEEE80211_ELEMID_QUIET;
	quiet->len = 6;
	if (vap->iv_quiet_count_value == 1)
		vap->iv_quiet_count_value = vap->iv_quiet_count;
	else if (vap->iv_quiet_count_value > 1)
		vap->iv_quiet_count_value--;

	if (vap->iv_quiet_count_value == 0) {
		/* value 0 is reserved as per 802.11h standerd */
		vap->iv_quiet_count_value = 1;
	}

	quiet->tbttcount = vap->iv_quiet_count_value;
	quiet->period = vap->iv_quiet_period;
	quiet->duration = htole16(vap->iv_quiet_duration);
	quiet->offset = htole16(vap->iv_quiet_offset);
	return frm + sizeof(*quiet);
}

/*
 * Add an 11h Channel Switch Announcement element to a frame.
 * Note that we use the per-vap CSA count to adjust the global
 * counter so we can use this routine to form probe response
 * frames and get the current count.
 */
static uint8_t *
ieee80211_add_csa(uint8_t *frm, struct ieee80211vap *vap)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_csa_ie *csa = (struct ieee80211_csa_ie *) frm;

	csa->csa_ie = IEEE80211_ELEMID_CSA;
	csa->csa_len = 3;
	csa->csa_mode = 1;		/* XXX force quiet on channel */
	csa->csa_newchan = ieee80211_chan2ieee(ic, ic->ic_csa_newchan);
	csa->csa_count = ic->ic_csa_count - vap->iv_csa_count;
	return frm + sizeof(*csa);
}

/*
 * Add an 11h country information element to a frame.
 */
static uint8_t *
ieee80211_add_countryie(uint8_t *frm, struct ieee80211com *ic)
{

	if (ic->ic_countryie == NULL ||
	    ic->ic_countryie_chan != ic->ic_bsschan) {
		/*
		 * Handle lazy construction of ie.  This is done on
		 * first use and after a channel change that requires
		 * re-calculation.
		 */
		if (ic->ic_countryie != NULL)
			free(ic->ic_countryie, M_80211_NODE_IE);
		ic->ic_countryie = ieee80211_alloc_countryie(ic);
		if (ic->ic_countryie == NULL)
			return frm;
		ic->ic_countryie_chan = ic->ic_bsschan;
	}
	return add_appie(frm, ic->ic_countryie);
}

/*
 * Send a probe request frame with the specified ssid
 * and any optional information element data.
 */
int
ieee80211_send_probereq(struct ieee80211_node *ni,
	const uint8_t sa[IEEE80211_ADDR_LEN],
	const uint8_t da[IEEE80211_ADDR_LEN],
	const uint8_t bssid[IEEE80211_ADDR_LEN],
	const uint8_t *ssid, size_t ssidlen)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	const struct ieee80211_txparam *tp;
	struct ieee80211_bpf_params params;
	struct ieee80211_frame *wh;
	const struct ieee80211_rateset *rs;
	struct mbuf *m;
	uint8_t *frm;
#ifdef IEEE80211_DEBUG
	uint8_t ether_buf[18];
#endif

	if (vap->iv_state == IEEE80211_S_CAC) {
		IEEE80211_NOTE(vap, IEEE80211_MSG_OUTPUT, ni,
		    "block %s frame in CAC state", "probe request");
		vap->iv_stats.is_tx_badstate++;
		return EIO;		/* XXX */
	}

	/*
	 * Hold a reference on the node so it doesn't go away until after
	 * the xmit is complete all the way in the driver.  On error we
	 * will remove our reference.
	 */
	IEEE80211_DPRINTF(vap, IEEE80211_MSG_NODE,
		"ieee80211_ref_node (%s:%u) %p<%s> refcnt %d\n",
		__func__, __LINE__,
		ni, ether_sprintf(ni->ni_macaddr, ether_buf),
		ieee80211_node_refcnt(ni)+1);
	ieee80211_ref_node(ni);

	/*
	 * prreq frame format
	 *	[tlv] ssid
     *	[tlv] DS
	 *	[tlv] supported rates
	 *	[tlv] RSN (optional)
	 *	[tlv] extended supported rates
	 *	[tlv] WPA (optional)
	 *	[tlv] user-specified ie's
	 */
	m = ieee80211_getmgtframe(&frm,
		 ic->ic_headroom + sizeof(struct ieee80211_frame),
	       	 2 + IEEE80211_NWID_LEN
	       + 2 + IEEE80211_RATE_SIZE
           + sizeof(struct ieee80211_ie_dsss)
	       + sizeof(struct ieee80211_ie_wpa)
	       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
	       + sizeof(struct ieee80211_ie_wpa)
	       + (vap->iv_appie_probereq != NULL ?
		   vap->iv_appie_probereq->ie_len : 0)
	);
	if (m == NULL) {
		vap->iv_stats.is_tx_nobuf++;
		ieee80211_free_node(ni);
		return ENOMEM;
	}

	frm = ieee80211_add_ssid(frm, ssid, ssidlen);

	/* RSC-1888, CR-1121 - Added DS Parameter set in probe request in 2GHZ scanning. */
	if (IEEE80211_IS_CHAN_2GHZ(ic->ic_curchan)) {
		*frm++ = IEEE80211_ELEMID_DSPARMS;
		*frm++ = 1;
		*frm++ = ic->ic_curchan->ic_ieee;
	}

	rs = ieee80211_get_suprates(ic, ic->ic_curchan);
#ifdef ENABLE_P2P_SUPPORT
	if((vap->p2p != NULL) && (vap->p2p_enable))
	{
		frm = ieee80211_add_only_11g_rates(frm, rs);
	}
	else
#endif
	{
		frm = ieee80211_add_rates(frm, rs);
	}
#if 0
	/*commenting as security ie's are not necessary in probe request*/
	if (vap->iv_flags & IEEE80211_F_WPA2) {
		if (vap->iv_rsn_ie != NULL)
			frm = add_ie(frm, vap->iv_rsn_ie);
		/* XXX else complain? */
	}
#endif
#ifdef ENABLE_P2P_SUPPORT
	if((vap->p2p == NULL) || (vap->p2p_enable == 0))
#endif
	{
		frm = ieee80211_add_xrates(frm, rs);
	}
#if 0
	/*commenting as security ie's are not necessary in probe request*/
	if (vap->iv_flags & IEEE80211_F_WPA1) {
		if (vap->iv_wpa_ie != NULL)
			frm = add_ie(frm, vap->iv_wpa_ie);
		/* XXX else complain? */
	}
#endif
	if (vap->iv_appie_probereq != NULL)
		frm = add_appie(frm, vap->iv_appie_probereq);
	m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);

	KASSERT(M_LEADINGSPACE(m) >= sizeof(struct ieee80211_frame),
	    ("leading space %zd", M_LEADINGSPACE(m)));
	if (!(M_PREPEND(m, sizeof(struct ieee80211_frame), M_DONTWAIT))) {
		m_freem(m);
		/* NB: cannot happen */
		ieee80211_free_node(ni);
		return ENOMEM;
	}

	wh = mtod(m, struct ieee80211_frame *);
	ieee80211_send_setup(ni, m,
	     IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_PROBE_REQ,
	     IEEE80211_NONQOS_TID, sa, da, bssid);
	/* XXX power management? */
	m->m_flags |= M_ENCAP;		/* mark encapsulated */

	M_WME_SETAC(m, WME_AC_BE);

	IEEE80211_NODE_STAT(ni, tx_probereq);
	IEEE80211_NODE_STAT(ni, tx_mgmt);

	IEEE80211_DPRINTF(vap, IEEE80211_MSG_DEBUG | IEEE80211_MSG_DUMPPKTS,
	    "send probe req on channel %u bssid %s ssid \"%.*s\"\n",
	    ieee80211_chan2ieee(ic, ic->ic_curchan), ether_sprintf(bssid, ether_buf),
	    ssidlen, ssid);

	memset(&params, 0, sizeof(params));
	params.ibp_pri = M_WME_GETAC(m);
	tp = &vap->iv_txparms[ieee80211_chan2mode(ic->ic_curchan)];
	params.ibp_rate0 = tp->mgmtrate;
	if (IEEE80211_IS_MULTICAST(da)) {
		params.ibp_flags |= IEEE80211_BPF_NOACK;
		params.ibp_try0 = 1;
	} else
		params.ibp_try0 = tp->maxretry;
	params.ibp_power = ni->ni_txpower;
	return ic->ic_raw_xmit(ni, m, &params);
}

/*
 * Calculate capability information for mgt frames.
 */
uint16_t
ieee80211_getcapinfo(struct ieee80211vap *vap, struct ieee80211_channel *chan)
{
	struct ieee80211com *ic = vap->iv_ic;
	uint16_t capinfo;

#ifdef ENABLE_P2P_SUPPORT
	if(vap->p2p_enable == 0)
#endif
	KASSERT(vap->iv_opmode != IEEE80211_M_STA, ("station mode"));

	if (vap->iv_opmode == IEEE80211_M_HOSTAP)
		capinfo = IEEE80211_CAPINFO_ESS;
	else if (vap->iv_opmode == IEEE80211_M_IBSS)
		capinfo = IEEE80211_CAPINFO_IBSS;
	else
		capinfo = 0;
	if (vap->iv_flags & IEEE80211_F_PRIVACY)
		capinfo |= IEEE80211_CAPINFO_PRIVACY;
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    IEEE80211_IS_CHAN_2GHZ(chan))
		capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
	if (IEEE80211_IS_CHAN_5GHZ(chan) && (vap->iv_flags & IEEE80211_F_DOTH))
		capinfo |= IEEE80211_CAPINFO_SPECTRUM_MGMT;

  /** Privacy bit is Setting Again Unconditionally in Beacon and Probe Resp
    * as we have seen some rare scenarios, where it is disabled */ 
#ifdef ENABLE_P2P_SUPPORT
	if(vap->p2p_enable)
  {
		capinfo |= IEEE80211_CAPINFO_PRIVACY;
  }
#endif
	return capinfo;
}

/*
 * Send a management frame.  The node is for the destination (or ic_bss
 * when in station mode).  Nodes other than ic_bss have their reference
 * count bumped to reflect our use for an indeterminant time.
 */
int
ieee80211_send_mgmt(struct ieee80211_node *ni, int type, int arg)
{
#define	HTFLAGS (IEEE80211_NODE_HT | IEEE80211_NODE_HTCOMPAT)
#define	senderr(_x, _v)	do { vap->iv_stats._v++; ret = _x; goto bad; } while (0)
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct ieee80211_node *bss = vap->iv_bss;
	struct ieee80211_bpf_params params;
	struct mbuf *m;
	uint8_t *frm;
	uint16_t capinfo;
	int has_challenge, is_shared_key, ret = -1, status;
	uint8_t *qos_ptr; 
	struct ieee80211_wme_info *info;
#ifdef IEEE80211_DEBUG
	uint8_t ether_buf[18];
#endif
	KASSERT(ni != NULL, ("null node"));

	/*
	 * Hold a reference on the node so it doesn't go away until after
	 * the xmit is complete all the way in the driver.  On error we
	 * will remove our reference.
	 */
	IEEE80211_DPRINTF(vap, IEEE80211_MSG_NODE,
		"ieee80211_ref_node (%s:%u) %p<%s> refcnt %d\n",
		__func__, __LINE__,
		ni, ether_sprintf(ni->ni_macaddr, ether_buf),
		ieee80211_node_refcnt(ni)+1);
	ieee80211_ref_node(ni);

	memset(&params, 0, sizeof(params));
	switch (type) {

	case IEEE80211_FC0_SUBTYPE_AUTH:
#ifdef CONFIG_11W
		if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
			if (!ni->ni_associd) {
				ni->hal_priv_node.conn_in_prog = 1;
			}
		}
#else
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, 
				("<<<<<<<<< SENDING AUTHENTICATION REQUEST >>>>>>>>>>>>>\n"));
			vap->hal_priv_vap->conn_in_prog = 1;
#endif
		status = arg >> 16;
		arg &= 0xffff;
		has_challenge = ((arg == IEEE80211_AUTH_SHARED_CHALLENGE ||
		    arg == IEEE80211_AUTH_SHARED_RESPONSE) &&
		    ni->ni_challenge != NULL);

		/*
		 * Deduce whether we're doing open authentication or
		 * shared key authentication.  We do the latter if
		 * we're in the middle of a shared key authentication
		 * handshake or if we're initiating an authentication
		 * request and configured to use shared key.
		 */
		is_shared_key = has_challenge ||
		     arg >= IEEE80211_AUTH_SHARED_RESPONSE ||
		     (arg == IEEE80211_AUTH_SHARED_REQUEST &&
		      bss->ni_authmode == IEEE80211_AUTH_SHARED);

		m = ieee80211_getmgtframe(&frm,
			  ic->ic_headroom + sizeof(struct ieee80211_frame),
			  3 * sizeof(uint16_t)
			+ (has_challenge && status == IEEE80211_STATUS_SUCCESS ?
				 sizeof(uint16_t)+IEEE80211_CHALLENGE_LEN : 0)
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
			+ (vap->iv_appie_auth != NULL ?
                                vap->iv_appie_auth->ie_len : 0)
#endif //CONFIG_11R
		);
		if (m == NULL)
			senderr(ENOMEM, is_tx_nobuf);
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
		if(vap->iv_appie_auth != NULL && vap->hal_priv_vap->roam_ind == 1){
			((uint16_t *)frm)[0] = htole16(IEEE80211_AUTH_ALG_FT);
		}else
#endif //CONFIG_11R
		((uint16_t *)frm)[0] =
		    (is_shared_key) ? htole16(IEEE80211_AUTH_ALG_SHARED)
		                    : htole16(IEEE80211_AUTH_ALG_OPEN);
		((uint16_t *)frm)[1] = htole16(arg);	/* sequence number */
		((uint16_t *)frm)[2] = htole16(status);/* status */

		if (has_challenge && status == IEEE80211_STATUS_SUCCESS) {
			((uint16_t *)frm)[3] =
			    htole16((IEEE80211_CHALLENGE_LEN << 8) |
			    IEEE80211_ELEMID_CHALLENGE);
			memcpy(&((uint16_t *)frm)[4], ni->ni_challenge,
			    IEEE80211_CHALLENGE_LEN);
			m->m_pkthdr.len = m->m_len =
				4 * sizeof(uint16_t) + IEEE80211_CHALLENGE_LEN;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, 
					("SHARED : In %s and %d pkthdr_len %d \n", __func__, __LINE__, m->m_pkthdr.len));
			if (arg == IEEE80211_AUTH_SHARED_RESPONSE) {
				IEEE80211_NOTE(vap, IEEE80211_MSG_AUTH, ni,
				    "request encrypt frame (%s)", __func__);
				/* mark frame for encryption */
				params.ibp_flags |= IEEE80211_BPF_CRYPTO;
			}
		} else
			m->m_pkthdr.len = m->m_len = 3 * sizeof(uint16_t);
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
		if (vap->iv_appie_auth != NULL){
			memcpy(&((uint16_t *)frm)[3], vap->iv_appie_auth->ie_data, vap->iv_appie_auth->ie_len);
			m->m_pkthdr.len = m->m_len = m->m_len + vap->iv_appie_auth->ie_len;
		}
#endif //CONFIG_11R
		/* XXX not right for shared key */
		if (status == IEEE80211_STATUS_SUCCESS)
			IEEE80211_NODE_STAT(ni, tx_auth);
		else
			IEEE80211_NODE_STAT(ni, tx_auth_fail);

		if (vap->iv_opmode == IEEE80211_M_STA)
			ieee80211_add_callback(m, ieee80211_tx_mgt_cb,
				(void *) vap->iv_state);
		break;

	case IEEE80211_FC0_SUBTYPE_DEAUTH:
#if 0
		if(vap->hal_priv_vap->roam_ind)
		{
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("changing destination addr \n"));
				IEEE80211_ADDR_COPY(frm, vap->hal_priv_vap->iv_bss_roam->ni_macaddr);
		}
#endif
			/* Block the data packets */
		if(!vap->hal_priv_vap->sta_data_block && vap->iv_opmode == IEEE80211_M_STA)
		{
				vap->iv_block(vap, STA_DISCONNECTED, 0);
		}
		if(vap->hal_priv_vap->roam_ind)
		{
			if((!(vap->hal_priv_vap->roam_ioctl)))
			{
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
						("Sending NULL data instead of DEAUTH \n"));
				ieee80211_send_nulldata(ni);
#ifdef RSI_CCX
                                vap->aironet_iapp_pkt.aironet_disconect_at = jiffies; /*Getting the time of dissassociation with the AP */
#endif
				return 0;
			}

		}
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, 
				("<<<<<<<<< SENDING DEAUTHENTICATION REQUEST >>>>>>>>>>>>>\n"));
		IEEE80211_NOTE(vap, IEEE80211_MSG_AUTH, ni,
		    "send station deauthenticate (reason %d)", arg);
		m = ieee80211_getmgtframe(&frm,
			ic->ic_headroom + sizeof(struct ieee80211_frame),
			sizeof(uint16_t));
		if (m == NULL)
			senderr(ENOMEM, is_tx_nobuf);
		*(uint16_t *)frm = htole16(arg);	/* reason */
		m->m_pkthdr.len = m->m_len = sizeof(uint16_t);

		IEEE80211_NODE_STAT(ni, tx_deauth);
		IEEE80211_NODE_STAT_SET(ni, tx_deauth_code, arg);

		ieee80211_node_unauthorize(ni);		/* port closed */
		break;

	case IEEE80211_FC0_SUBTYPE_ASSOC_REQ:
	case IEEE80211_FC0_SUBTYPE_REASSOC_REQ:
		/*
		 * asreq frame format
		 *	[2] capability information
		 *	[2] listen interval
		 *	[6*] current AP address (reassoc only)
		 *	[tlv] ssid
		 *	[tlv] supported rates
		 *	[tlv] extended supported rates
		 *	[4] power capability (optional)
		 *	[28] supported channels (optional)
		 *	[tlv] HT capabilities
		 *	[tlv] WME (optional)
		 *	[tlv] Vendor OUI HT capabilities (optional)
		 *	[tlv] Atheros capabilities (if negotiated)
		 *	[tlv] AppIE's (optional)
		 */
		vap->hal_priv_vap->assoc_req = 1;
		m = ieee80211_getmgtframe(&frm,
			 ic->ic_headroom + sizeof(struct ieee80211_frame),
			 sizeof(uint16_t)
		       + sizeof(uint16_t)
		       + IEEE80211_ADDR_LEN
		       + 2 + IEEE80211_NWID_LEN
		       + 2 + IEEE80211_RATE_SIZE
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
		       + 4
		       + 2 + 26
		       + sizeof(struct ieee80211_wme_info)
#ifdef RSI_CCX
		       + sizeof(struct ieee80211_aironet_cckm_ie)
		       + sizeof(struct ieee80211_aironet_ie)
		       + sizeof(struct ieee80211_ccx_info)
		       + sizeof(struct ccx_radio_cap_ie)
		       + sizeof(struct ieee80211_aironet_ap_ip_ie)
#endif
		       + sizeof(struct ieee80211_ie_htcap)
		       + 4 + sizeof(struct ieee80211_ie_htcap)
#ifdef IEEE80211_SUPPORT_SUPERG
		       + sizeof(struct ieee80211_ath_ie)
#endif
		       + (vap->iv_appie_wpa != NULL ?
				vap->iv_appie_wpa->ie_len : 0)
		       + (vap->iv_appie_assocreq != NULL ?
				vap->iv_appie_assocreq->ie_len : 0)
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
			   + (vap->iv_appie_auth != NULL ?
				vap->iv_appie_auth->ie_len : 0)
			   + (vap->iv_appie_mdft != NULL ?
				vap->iv_appie_mdft->ie_len : 0)
#endif //CONFIG_11R
		);
		if (m == NULL)
			senderr(ENOMEM, is_tx_nobuf);

		KASSERT(vap->iv_opmode == IEEE80211_M_STA,
		    ("wrong mode %u", vap->iv_opmode));
		capinfo = IEEE80211_CAPINFO_ESS;
		if (vap->iv_flags & IEEE80211_F_PRIVACY)
			capinfo |= IEEE80211_CAPINFO_PRIVACY;
		/*
		 * NB: Some 11a AP's reject the request when
		 *     short premable is set.
		 */
		if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
		    IEEE80211_IS_CHAN_2GHZ(ic->ic_curchan))
			capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
		if (IEEE80211_IS_CHAN_ANYG(ic->ic_curchan) &&
		    (ic->ic_caps & IEEE80211_C_SHSLOT))
			capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
		if ((ni->ni_capinfo & IEEE80211_CAPINFO_SPECTRUM_MGMT) &&
		    (vap->iv_flags & IEEE80211_F_DOTH))
			capinfo |= IEEE80211_CAPINFO_SPECTRUM_MGMT;
		*(uint16_t *)frm = htole16(capinfo);
		frm += 2;

		KASSERT(bss->ni_intval != 0, ("beacon interval is zero!"));
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, 
				("listen interval in %s assoc req =%di ni_inval=%d\n", __func__, ic->ic_lintval, bss->ni_intval));
		*(uint16_t *)frm = 8;
	//	*(uint16_t *)frm = htole16(howmany(ic->ic_lintval,
	//					    bss->ni_intval));
		frm += 2;
		if (type == IEEE80211_FC0_SUBTYPE_REASSOC_REQ) 
		{
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
						("SENDING ORIGINAL REASSOCIATION REQUEST : In %s and %d type %x \n", __func__, __LINE__, type));
				IEEE80211_ADDR_COPY(frm, bss->ni_bssid);
				frm += IEEE80211_ADDR_LEN;
		}
		//else if ((type == IEEE80211_FC0_SUBTYPE_ASSOC_REQ) && (vap->hal_priv_vap->roam_ind)) 
		else if ((type == IEEE80211_FC0_SUBTYPE_ASSOC_REQ) && (vap->hal_priv_vap->reassoc_ind)) 
		{
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
					("SENDING REASSOCIATION REQUEST : type %x \n", type));
			type = IEEE80211_FC0_SUBTYPE_REASSOC_REQ;
			if(vap->hal_priv_vap->roam_ind)
				IEEE80211_ADDR_COPY(frm, vap->hal_priv_vap->iv_bss_roam->ni_bssid);
			else
				IEEE80211_ADDR_COPY(frm, vap->iv_bss->ni_bssid);
			frm += IEEE80211_ADDR_LEN;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
					("<<<<< Resetting Roam Indication flag >>>>>\n"));

#if 0
			if(!(vap->iv_flags & IEEE80211_F_PRIVACY))
			{
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
						("Resetting Roaming Indication flag in Open mode\n"));
				vap->hal_priv_vap->roam_ind = 0;
			}
#endif
		}
//		vap->hal_priv_vap->roam_ind = 0;
		
		frm = ieee80211_add_ssid(frm, ni->ni_essid, ni->ni_esslen);
#ifdef ENABLE_P2P_SUPPORT
		if((vap->p2p != NULL) && (vap->p2p_enable))
		{
			frm = ieee80211_add_only_11g_rates(frm, &ni->ni_rates);
		}
		else
#endif
		{
			frm = ieee80211_add_rates(frm, &ni->ni_rates);
		}
		if (vap->iv_flags & IEEE80211_F_WPA2) {
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
			if(vap->iv_appie_auth != NULL && vap->hal_priv_vap->roam_ind == 1){
				frm = add_appie(frm, vap->iv_appie_auth);
			}else
#endif //CONFIG_11R
			{
			 if (vap->iv_rsn_ie != NULL)
				frm = add_ie(frm, vap->iv_rsn_ie);
			/* XXX else complain? */
		}
   }
#ifdef ENABLE_P2P_SUPPORT
		if((vap->p2p == NULL) || (vap->p2p_enable == 0))
#endif
		{
			frm = ieee80211_add_xrates(frm, &ni->ni_rates);
		}
		if (capinfo & IEEE80211_CAPINFO_SPECTRUM_MGMT) {
			frm = ieee80211_add_powercapability(frm,
					ic->ic_curchan);
			frm = ieee80211_add_supportedchannels(frm, ic);
		}
#ifdef RSI_CCX
		else if(ni->ni_ies.ccx_ver_ie && (ic->ic_curchan->ic_flags & IEEE80211_CHAN_A))
		{
			frm = ieee80211_add_powercapability(frm,ic->ic_curchan);
		}
# endif

		if ((vap->iv_flags_ht & IEEE80211_FHT_HT) &&
		    ni->ni_ies.htcap_ie != NULL &&
		    ni->ni_ies.htcap_ie[0] == IEEE80211_ELEMID_HTCAP) 
		{
			frm = ieee80211_add_htcap(frm, ni);
		}
		if (vap->iv_flags & IEEE80211_F_WPA1) {
			if (vap->iv_wpa_ie != NULL)
				frm = add_ie(frm, vap->iv_wpa_ie);
			/* XXX else complain */
		}
		if(1)
		{
			memcpy(&vap->hal_priv_vap->uapsd_params_updated, &vap->hal_priv_vap->uapsd_params_ioctl, sizeof(vap->hal_priv_vap->uapsd_params_ioctl));
		}
		if ((ic->ic_flags & IEEE80211_F_WME) &&
					ni->ni_ies.wme_ie != NULL){
				frm = ieee80211_add_wme_info(frm, &ic->ic_wme);
				qos_ptr = frm - sizeof(struct ieee80211_wme_info);
				info = (struct ieee80211_wme_info *)qos_ptr;
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, 
						(" wmm Ie %02x %02x %02x %02x \n", ni->ni_ies.wme_ie[7], ni->ni_ies.wme_ie[8], ni->ni_ies.wme_ie[9], ni->ni_ies.wme_ie[10])); 
				/* If UAPSD ioctl is issued in between the connection and user 
				 * doesnt gave reset adapter has connection has not completed succesfully
				 * updating the IOCTL values as per user request
				 */
				//if((vap->hal_priv_vap->uapsd_params_ioctl.uapsd_acs != vap->hal_priv_vap->uapsd_params_updated.uapsd_acs)
				//		||(vap->hal_priv_vap->uapsd_params_updated.uapsd_wakeup_period != vap->hal_priv_vap->uapsd_params_ioctl.uapsd_wakeup_period))
				/* Checking UAPSD support in wme_info 
				 * parsed from beacon
				 */
				if(ni->ni_ies.wme_ie[8] & BIT(7))
				{
					IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, ("Ap supports UAPSD \n"));
					IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, 
							("acs %02x wakeup %02x \n", vap->hal_priv_vap->uapsd_params_ioctl.uapsd_acs, vap->hal_priv_vap->uapsd_params_ioctl.uapsd_wakeup_period));
								//memcpy(&adapter->uapsd_params_updated, wrq->u.data.pointer, sizeof(adapter->uapsd_params_ioctl));
						
						if(vap->hal_priv_vap->uapsd_params_updated.uapsd_acs)
						{
							info->wme_info = vap->hal_priv_vap->uapsd_params_updated.uapsd_acs;
							IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, 
									("USer needs UAPSD and AP also supports UAPSD\n"));
						}
				}
				else
				{
						info->wme_info = 0; 
						if(vap->hal_priv_vap->uapsd_params_updated.uapsd_acs)
						{
							IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, ("Mimic support is enabled\n"));
							vap->hal_priv_vap->uapsd_params_updated.mimic_support = 1;
						}

				}
			}
			else if(vap->hal_priv_vap->uapsd_params_updated.uapsd_acs)
			{
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
						("Mimic support is enabled in %s Line %d\n", __func__, __LINE__));
				vap->hal_priv_vap->uapsd_params_updated.mimic_support = 1;
			}
#ifdef RSI_CCX
		if(ni->ni_ies.ccx_ver_ie ){
			frm = ieee80211_add_ccx_info(frm);
		}

		if(vap->hal_priv_vap->roam_ind && vap->cckm_ie.aironet_cckm_rn  ) {
			frm = ieee80211_add_cckm_info(frm, vap);
		}
		if(ni->ni_ies.aironet_ie){
			frm = ieee80211_add_aironet_ie(frm,(struct ieee80211_aironet_ie *)ni->ni_ies.aironet_ie);
		}	

		if(ni->ni_ies.aironet_ie){
			frm = ieee80211_add_aironet_ap_ip_ie(frm,(struct ieee80211_aironet_ap_ip_ie *)ni->ni_ies.aironet_rsd_ie);
		}	

		if(ni->ni_ies.ccx_radio_cap_ie){
			frm = ieee80211_add_ccx_radio_cap_ie(frm,(struct ccx_radio_cap_ie *)ni->ni_ies.ccx_radio_cap_ie);
		}	
#endif
		vap->hal_priv_vap->roam_ind = 0;

		if ((vap->iv_flags_ht & IEEE80211_FHT_HT) &&
		    ni->ni_ies.htcap_ie != NULL &&
		    ni->ni_ies.htcap_ie[0] == IEEE80211_ELEMID_VENDOR)
			frm = ieee80211_add_htcap_vendor(frm, ni);
#ifdef IEEE80211_SUPPORT_SUPERG
		if (IEEE80211_ATH_CAP(vap, ni, IEEE80211_F_ATHEROS)) {
			frm = ieee80211_add_ath(frm, 
				IEEE80211_ATH_CAP(vap, ni, IEEE80211_F_ATHEROS),
				((vap->iv_flags & IEEE80211_F_WPA) == 0 &&
				 ni->ni_authmode != IEEE80211_AUTH_8021X) ?
				vap->iv_def_txkey : IEEE80211_KEYIX_NONE);
		}
#endif /* IEEE80211_SUPPORT_SUPERG */
		if (vap->iv_appie_assocreq != NULL)
			frm = add_appie(frm, vap->iv_appie_assocreq);
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
		if(vap->iv_appie_mdft != NULL){
			frm = ieee80211_add_mdie(frm, vap->iv_appie_mdft->ie_data, vap->iv_appie_mdft->ie_len);
		}
#endif //CONFIG_11R
		m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
#ifdef CONFIG_11W
		vap->hal_priv_vap->assoc_retry = 0;
#endif

		ieee80211_add_callback(m, ieee80211_tx_mgt_cb,
			(void *) vap->iv_state);
		break;

	case IEEE80211_FC0_SUBTYPE_ASSOC_RESP:
	case IEEE80211_FC0_SUBTYPE_REASSOC_RESP:
		/*
		 * asresp frame format
		 *	[2] capability information
		 *	[2] status
		 *	[2] association ID
		 *	[tlv] supported rates
		 *	[tlv] extended supported rates
		 *	[tlv] HT capabilities (standard, if STA enabled)
		 *	[tlv] HT information (standard, if STA enabled)
		 *	[tlv] WME (if configured and STA enabled)
		 *	[tlv] HT capabilities (vendor OUI, if STA enabled)
		 *	[tlv] HT information (vendor OUI, if STA enabled)
		 *	[tlv] Atheros capabilities (if STA enabled)
		 *	[tlv] AppIE's (optional)
		 */
		m = ieee80211_getmgtframe(&frm,
			 ic->ic_headroom + sizeof(struct ieee80211_frame),
			 sizeof(uint16_t)
		       + sizeof(uint16_t)
		       + sizeof(uint16_t)
		       + 2 + IEEE80211_RATE_SIZE
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
		       + sizeof(struct ieee80211_ie_htcap) + 4
		       + sizeof(struct ieee80211_ie_htinfo) + 4
		       + sizeof(struct ieee80211_wme_param)
#ifdef IEEE80211_SUPPORT_SUPERG
		       + sizeof(struct ieee80211_ath_ie)
#endif
		       + (vap->iv_appie_assocresp != NULL ?
				vap->iv_appie_assocresp->ie_len : 0)
		);
		if (m == NULL)
			senderr(ENOMEM, is_tx_nobuf);

		capinfo = ieee80211_getcapinfo(vap, bss->ni_chan);
		*(uint16_t *)frm = htole16(capinfo);
		frm += 2;

		*(uint16_t *)frm = htole16(arg);	/* status */
		frm += 2;

		if (arg == IEEE80211_STATUS_SUCCESS) {
			*(uint16_t *)frm = htole16(ni->ni_associd);
			IEEE80211_NODE_STAT(ni, tx_assoc);
		} else {
#ifdef CONFIG_11W
			*(uint16_t *)frm = 0;
#endif
			IEEE80211_NODE_STAT(ni, tx_assoc_fail);
		}
		frm += 2;

#ifdef ENABLE_P2P_SUPPORT
	if((vap->p2p != NULL) && (vap->p2p_enable))
	{
		frm = ieee80211_add_only_11g_rates(frm, &ni->ni_rates);
	}
	else
#endif
	{
		frm = ieee80211_add_rates(frm, &ni->ni_rates);
	}
#ifdef ENABLE_P2P_SUPPORT
	if((vap->p2p == NULL) || (vap->p2p_enable == 0))
#endif
	{
		frm = ieee80211_add_xrates(frm, &ni->ni_rates);
	}
		/* NB: respond according to what we received */
		if ((ni->ni_flags & HTFLAGS) == IEEE80211_NODE_HT) {
			frm = ieee80211_add_htcap(frm, ni);
			frm = ieee80211_add_htinfo(frm, ni);
		}
		if ((vap->iv_flags & IEEE80211_F_WME) &&
		    ni->ni_ies.wme_ie != NULL)
			frm = ieee80211_add_wme_param(frm, &ic->ic_wme);
		if ((ni->ni_flags & HTFLAGS) == HTFLAGS) {
			frm = ieee80211_add_htcap_vendor(frm, ni);
			frm = ieee80211_add_htinfo_vendor(frm, ni);
		}
#ifdef IEEE80211_SUPPORT_SUPERG
		if (IEEE80211_ATH_CAP(vap, ni, IEEE80211_F_ATHEROS))
			frm = ieee80211_add_ath(frm, 
				IEEE80211_ATH_CAP(vap, ni, IEEE80211_F_ATHEROS),
				((vap->iv_flags & IEEE80211_F_WPA) == 0 &&
				 ni->ni_authmode != IEEE80211_AUTH_8021X) ?
				vap->iv_def_txkey : IEEE80211_KEYIX_NONE);
#endif /* IEEE80211_SUPPORT_SUPERG */
		if (vap->iv_appie_assocresp != NULL)
			frm = add_appie(frm, vap->iv_appie_assocresp);
#ifdef CONFIG_11W

		if (arg == IEEE80211_STATUS_REJECTED_TEMPORARILY)
			frm = ieee80211_add_timeout_ie(frm);

#endif
		m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
		break;

	case IEEE80211_FC0_SUBTYPE_DISASSOC:
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, 
				("SENDING DISASSOCIATION : reason code %d\n", arg));
    vap->hal_priv_vap->reassoc_ind = 0;
		IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC, ni,
		    "send station disassociate (reason %d)", arg);
		m = ieee80211_getmgtframe(&frm,
			ic->ic_headroom + sizeof(struct ieee80211_frame),
			sizeof(uint16_t));
		if (m == NULL)
			senderr(ENOMEM, is_tx_nobuf);
		*(uint16_t *)frm = htole16(arg);	/* reason */
		m->m_pkthdr.len = m->m_len = sizeof(uint16_t);

		IEEE80211_NODE_STAT(ni, tx_disassoc);
		IEEE80211_NODE_STAT_SET(ni, tx_disassoc_code, arg);
		break;

	default:
		IEEE80211_NOTE(vap, IEEE80211_MSG_ANY, ni,
		    "invalid mgmt frame type %u", type);
		senderr(EINVAL, is_tx_unknownmgt);
		/* NOTREACHED */
	}

	/* NB: force non-ProbeResp frames to the highest queue */
	params.ibp_pri = WME_AC_VO;
	params.ibp_rate0 = bss->ni_txparms->mgmtrate;
	/* NB: we know all frames are unicast */
	params.ibp_try0 = bss->ni_txparms->maxretry;
	params.ibp_power = bss->ni_txpower;
#ifdef __LINUX__
	status = ieee80211_mgmt_output(ni, m, type, &params);
	if (!status)
	{
		if (m->m_flags & M_TXCB)
		{
			ieee80211_tx_mgt_cb(ni, (void *)vap->iv_state, status);
		}
	}
#else
	return ieee80211_mgmt_output(ni, m, type, &params);
#endif
bad:
	ieee80211_free_node(ni);
	return ret;
#undef senderr
#undef HTFLAGS
}

/*
 * Return an mbuf with a probe response frame in it.
 * Space is left to prepend and 802.11 header at the
 * front but it's left to the caller to fill in.
 */
struct mbuf *
ieee80211_alloc_proberesp(struct ieee80211_node *bss, int legacy, int p2p_ie_found)
{
	struct ieee80211vap *vap = bss->ni_vap;
	struct ieee80211com *ic = bss->ni_ic;
	const struct ieee80211_rateset *rs;
	struct mbuf *m;
	uint16_t capinfo;
	uint8_t *frm;
	uint8_t bint = 100;
	int proberesp_ie_len = 0;

	/*
	 * probe response frame format
	 *	[8] time stamp
	 *	[2] beacon interval
	 *	[2] cabability information
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 *	[tlv] parameter set (FH/DS)
	 *	[tlv] parameter set (IBSS)
	 *	[tlv] country (optional)
	 *	[3] power control (optional)
	 *	[5] channel switch announcement (CSA) (optional)
	 *	[tlv] extended rate phy (ERP)
	 *	[tlv] extended supported rates
	 *	[tlv] RSN (optional)
	 *	[tlv] HT capabilities
	 *	[tlv] HT information
	 *	[tlv] WPA (optional)
	 *	[tlv] WME (optional)
	 *	[tlv] Vendor OUI HT capabilities (optional)
	 *	[tlv] Vendor OUI HT information (optional)
	 *	[tlv] Atheros capabilities
	 *	[tlv] AppIE's (optional)
	 *	[tlv] Mesh ID (MBSS)
	 *	[tlv] Mesh Conf (MBSS)
	 */
	if(vap->iv_appie_proberesp != NULL)
	{
		proberesp_ie_len = vap->iv_appie_proberesp->ie_len;
#ifdef ENABLE_P2P_SUPPORT
		if((!p2p_ie_found) && (vap->p2p_enable))
		{
			proberesp_ie_len = vap->iv_appie_proberesp->ie_data[1] + 2 ;
		}
#endif
	}
	m = ieee80211_getmgtframe(&frm,
		 ic->ic_headroom + sizeof(struct ieee80211_frame),
		 8
	       + sizeof(uint16_t)
	       + sizeof(uint16_t)
	       + 2 + IEEE80211_NWID_LEN
	       + 2 + IEEE80211_RATE_SIZE
	       + 7	/* max(7,3) */
	       + IEEE80211_COUNTRY_MAX_SIZE
	       + 3
	       + sizeof(struct ieee80211_csa_ie)
	       + sizeof(struct ieee80211_quiet_ie)
	       + 3
	       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
	       + sizeof(struct ieee80211_ie_wpa)
	       + sizeof(struct ieee80211_ie_htcap)
	       + sizeof(struct ieee80211_ie_htinfo)
	       + sizeof(struct ieee80211_ie_wpa)
	       + sizeof(struct ieee80211_wme_param)
	       + 4 + sizeof(struct ieee80211_ie_htcap)
	       + 4 + sizeof(struct ieee80211_ie_htinfo)
#ifdef IEEE80211_SUPPORT_SUPERG
	       + sizeof(struct ieee80211_ath_ie)
#endif
#ifdef IEEE80211_SUPPORT_MESH
	       + 2 + IEEE80211_MESHID_LEN
	       + sizeof(struct ieee80211_meshconf_ie)
#endif
	       + (vap->iv_appie_proberesp != NULL ?
			//vap->iv_appie_proberesp->ie_len : 0)
			proberesp_ie_len : 0)
		+ (ic->ic_apple_ie.add_apple_ie == true ?
			ic->ic_apple_ie.ie_len : 0)
	);

	if (m == NULL) {
		vap->iv_stats.is_tx_nobuf++;
		return NULL;
	}

	memset(frm, 0, 8);	/* timestamp should be filled later */
	frm += 8;

#ifdef ENABLE_P2P_SUPPORT
	if(vap->p2p_enable && vap->p2p && (vap->p2p_mode == IEEE80211_P2P_DEVICE))
	{
		*(uint16_t *)frm = (bint);
	}
	else
#endif
	{
		*(uint16_t *)frm = (bss->ni_intval);
	}
	frm += 2;

#ifdef ENABLE_P2P_SUPPORT
	if(vap->p2p_enable && vap->p2p && (vap->p2p_mode == IEEE80211_P2P_DEVICE))
	{
		capinfo = ieee80211_getcapinfo(vap, vap->iv_ic->ic_curchan);
	}
	else
#endif
	{
		capinfo = ieee80211_getcapinfo(vap, bss->ni_chan);
	}
	*(uint16_t *)frm = htole16(capinfo);
	frm += 2;

#ifdef ENABLE_P2P_SUPPORT
	if(vap->p2p_enable && vap->p2p && (vap->p2p_mode == IEEE80211_P2P_DEVICE))
	{
		frm = ieee80211_add_ssid(frm, vap->p2p->p2p_wildcard_essid, vap->p2p->p2p_wildcard_essid_len);
	}
	else
#endif
	frm = ieee80211_add_ssid(frm, bss->ni_essid, bss->ni_esslen);

#ifdef ENABLE_P2P_SUPPORT
	if(vap->p2p_enable && vap->p2p && (vap->p2p_mode == IEEE80211_P2P_DEVICE))
	{
		rs = ieee80211_get_suprates(ic, bss->ni_chan);
	}
	else
#endif
	{
		rs = &bss->ni_rates;
	}

#ifdef ENABLE_P2P_SUPPORT
	if((vap->p2p != NULL) && (vap->p2p_enable))
	{
		frm = ieee80211_add_only_11g_rates(frm, rs);
	}
	else
#endif
	{
		if(ic->ic_curchan->ic_ieee == 14) {
			frm = ieee80211_add_only_11b_rates(frm, rs);
		}
		else {
			frm = ieee80211_add_rates(frm, rs);
		}
	}

	if (IEEE80211_IS_CHAN_FHSS(bss->ni_chan)) {
		*frm++ = IEEE80211_ELEMID_FHPARMS;
		*frm++ = 5;
		*frm++ = bss->ni_fhdwell & 0x00ff;
		*frm++ = (bss->ni_fhdwell >> 8) & 0x00ff;
		*frm++ = IEEE80211_FH_CHANSET(
		    ieee80211_chan2ieee(ic, bss->ni_chan));
		*frm++ = IEEE80211_FH_CHANPAT(
		    ieee80211_chan2ieee(ic, bss->ni_chan));
		*frm++ = bss->ni_fhindex;
	} else {
		*frm++ = IEEE80211_ELEMID_DSPARMS;
		*frm++ = 1;
		//*frm++ = ieee80211_chan2ieee(ic, bss->ni_chan);
		*frm++ = ic->ic_curchan->ic_ieee;
	}

	if (vap->iv_opmode == IEEE80211_M_IBSS) {
		*frm++ = IEEE80211_ELEMID_IBSSPARMS;
		*frm++ = 2;
		*frm++ = 0; *frm++ = 0;		/* ATIM window */
	}
	if ((vap->iv_flags & IEEE80211_F_DOTH) ||
	    (vap->iv_flags_ext & IEEE80211_FEXT_DOTD))
		frm = ieee80211_add_countryie(frm, ic);
	if (vap->iv_flags & IEEE80211_F_DOTH) {
		if (IEEE80211_IS_CHAN_5GHZ(bss->ni_chan))
			frm = ieee80211_add_powerconstraint(frm, vap);
		if (ic->ic_flags & IEEE80211_F_CSAPENDING)
			frm = ieee80211_add_csa(frm, vap);
	}
	if (vap->iv_flags & IEEE80211_F_DOTH) {
		if (IEEE80211_IS_CHAN_DFS(ic->ic_bsschan) &&
		    (vap->iv_flags_ext & IEEE80211_FEXT_DFS)) {
			if (vap->iv_quiet)
				frm = ieee80211_add_quiet(frm, vap);
		}
	}
	if (IEEE80211_IS_CHAN_ANYG(bss->ni_chan))
		frm = ieee80211_add_erp(frm, ic);
	if(ic->ic_curchan->ic_ieee != 14)
	{
#ifdef ENABLE_P2P_SUPPORT
		if((vap->p2p == NULL) || (vap->p2p_enable == 0))
#endif
		{
			frm = ieee80211_add_xrates(frm, rs);
		}
	}
	if (vap->iv_flags & IEEE80211_F_WPA2) {
		if (vap->iv_rsn_ie != NULL)
			frm = add_ie(frm, vap->iv_rsn_ie);
		/* XXX else complain? */
	}
	/*
	 * NB: legacy 11b clients do not get certain ie's.
	 *     The caller identifies such clients by passing
	 *     a token in legacy to us.  Could expand this to be
	 *     any legacy client for stuff like HT ie's.
	 */
	if(ic->ic_curchan->ic_ieee != 14)
	{
		if (IEEE80211_IS_CHAN_HT(bss->ni_chan) &&
	    		legacy != IEEE80211_SEND_LEGACY_11B) {
				frm = ieee80211_add_htcap(frm, bss);
				frm = ieee80211_add_htinfo(frm, bss);
		}
	}
	if (vap->iv_flags & IEEE80211_F_WPA1) {
		if (vap->iv_wpa_ie != NULL)
			frm = add_ie(frm, vap->iv_wpa_ie);
		/* XXX else complain? */
	}
	if(ic->ic_curchan->ic_ieee != 14)
	{
		if (vap->iv_flags & IEEE80211_F_WME)
			frm = ieee80211_add_wme_param(frm, &ic->ic_wme);
	}
#if 0
	if (IEEE80211_IS_CHAN_HT(bss->ni_chan) &&
	    (vap->iv_flags_ht & IEEE80211_FHT_HTCOMPAT) &&
	    legacy != IEEE80211_SEND_LEGACY_11B) {
		frm = ieee80211_add_htcap_vendor(frm, bss);
		frm = ieee80211_add_htinfo_vendor(frm, bss);
	}
#endif
#ifdef IEEE80211_SUPPORT_SUPERG
	if ((vap->iv_flags & IEEE80211_F_ATHEROS) &&
	    legacy != IEEE80211_SEND_LEGACY_11B)
		frm = ieee80211_add_athcaps(frm, bss);
#endif
	if (ic->ic_apple_ie.add_apple_ie) {
		memcpy(frm, ic->ic_apple_ie.ie_data, ic->ic_apple_ie.ie_len);
		frm = (frm + ic->ic_apple_ie.ie_len);
	}

	if (vap->iv_appie_proberesp != NULL)
		//frm = add_appie(frm, vap->iv_appie_proberesp);
		frm = add_proberesp_appie(frm, vap->iv_appie_proberesp, proberesp_ie_len);
#ifdef IEEE80211_SUPPORT_MESH
	if (vap->iv_opmode == IEEE80211_M_MBSS) {
		frm = ieee80211_add_meshid(frm, vap);
		frm = ieee80211_add_meshconf(frm, vap);
	}
#endif
	m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);

	return m;
}

/*
 * Send a probe response frame to the specified mac address.
 * This does not go through the normal mgt frame api so we
 * can specify the destination address and re-use the bss node
 * for the sta reference.
 */
int
ieee80211_send_proberesp(struct ieee80211vap *vap,
	const uint8_t da[IEEE80211_ADDR_LEN], int legacy, int p2p_ie_found)
{
	struct ieee80211_node *bss = vap->iv_bss;
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_frame *wh;
	struct mbuf *m;
#ifdef IEEE80211_DEBUG
	uint8_t ether_buf[18];
#endif

	if (vap->iv_state == IEEE80211_S_CAC) {
		IEEE80211_NOTE(vap, IEEE80211_MSG_OUTPUT, bss,
		    "block %s frame in CAC state", "probe response");
		vap->iv_stats.is_tx_badstate++;
		return EIO;		/* XXX */
	}

	/*
	 * Hold a reference on the node so it doesn't go away until after
	 * the xmit is complete all the way in the driver.  On error we
	 * will remove our reference.
	 */
	IEEE80211_DPRINTF(vap, IEEE80211_MSG_NODE,
	    "ieee80211_ref_node (%s:%u) %p<%s> refcnt %d\n",
	    __func__, __LINE__, bss, ether_sprintf(bss->ni_macaddr, ether_buf),
	    ieee80211_node_refcnt(bss)+1);
	ieee80211_ref_node(bss);

	m = ieee80211_alloc_proberesp(bss, legacy, p2p_ie_found);
	if (m == NULL) {
		ieee80211_free_node(bss);
		return ENOMEM;
	}

	M_PREPEND(m, sizeof(struct ieee80211_frame), M_DONTWAIT);
	KASSERT(m != NULL, ("no room for header"));

	wh = mtod(m, struct ieee80211_frame *);
	ieee80211_send_setup(bss, m,
	     IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_PROBE_RESP,
	     IEEE80211_NONQOS_TID, vap->iv_myaddr, da, bss->ni_bssid);
	/* XXX power management? */
	m->m_flags |= M_ENCAP;		/* mark encapsulated */

	M_WME_SETAC(m, WME_AC_BE);

	IEEE80211_DPRINTF(vap, IEEE80211_MSG_DEBUG | IEEE80211_MSG_DUMPPKTS,
	    "send probe resp on channel %u to %s%s\n",
	    ieee80211_chan2ieee(ic, ic->ic_curchan), ether_sprintf(da, ether_buf),
	    legacy ? " <legacy>" : "");
	IEEE80211_NODE_STAT(bss, tx_mgmt);

	return ic->ic_raw_xmit(bss, m, NULL);
}

/*
 * Allocate and build a RTS (Request To Send) control frame.
 */
struct mbuf *
ieee80211_alloc_rts(struct ieee80211com *ic,
	const uint8_t ra[IEEE80211_ADDR_LEN],
	const uint8_t ta[IEEE80211_ADDR_LEN],
	uint16_t dur)
{
	struct ieee80211_frame_rts *rts;
	struct mbuf *m;

	/* XXX honor ic_headroom */
	m = m_gethdr(M_DONTWAIT, MT_DATA);
	if (m != NULL) {
		rts = mtod(m, struct ieee80211_frame_rts *);
		rts->i_fc[0] = IEEE80211_FC0_VERSION_0 |
			IEEE80211_FC0_TYPE_CTL | IEEE80211_FC0_SUBTYPE_RTS;
		rts->i_fc[1] = IEEE80211_FC1_DIR_NODS;
		*(u_int16_t *)rts->i_dur = htole16(dur);
		IEEE80211_ADDR_COPY(rts->i_ra, ra);
		IEEE80211_ADDR_COPY(rts->i_ta, ta);

		m->m_pkthdr.len = m->m_len = sizeof(struct ieee80211_frame_rts);
	}
	return m;
}

/*
 * Allocate and build a CTS (Clear To Send) control frame.
 */
struct mbuf *
ieee80211_alloc_cts(struct ieee80211com *ic,
	const uint8_t ra[IEEE80211_ADDR_LEN], uint16_t dur)
{
	struct ieee80211_frame_cts *cts;
	struct mbuf *m;

	/* XXX honor ic_headroom */
	m = m_gethdr(M_DONTWAIT, MT_DATA);
	if (m != NULL) {
		cts = mtod(m, struct ieee80211_frame_cts *);
		cts->i_fc[0] = IEEE80211_FC0_VERSION_0 |
			IEEE80211_FC0_TYPE_CTL | IEEE80211_FC0_SUBTYPE_CTS;
		cts->i_fc[1] = IEEE80211_FC1_DIR_NODS;
		*(u_int16_t *)cts->i_dur = htole16(dur);
		IEEE80211_ADDR_COPY(cts->i_ra, ra);

		m->m_pkthdr.len = m->m_len = sizeof(struct ieee80211_frame_cts);
	}
	return m;
}

#ifdef __LINUX__
static void
ieee80211_tx_mgt_timeout(unsigned long arg)
#else
ieee80211_tx_mgt_timeout(void *arg)
#endif
{
	struct ieee80211vap *vap = (struct ieee80211vap *)arg;
	struct ieee80211_node *ni = vap->iv_bss;

	//ni->ni_essid[0] = '\0';
	if(vap->hal_priv_vap->assoc_req)
	{
    		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, 
					("Sending peer notify with delete event  due to timeout\n"));
		vap->iv_ic->ic_send_station_info(ni, STA_DISCONNECTED);
		vap->hal_priv_vap->conn_in_prog = 0;
		ieee80211_notify_node_leave(ni);
		vap->hal_priv_vap->assoc_req = 0;
	}
	if (vap->iv_state != IEEE80211_S_INIT &&
	    (vap->iv_ic->ic_flags & IEEE80211_F_SCAN) == 0) {

		if(vap->iv_state == IEEE80211_S_AUTH){
			if(!vap->hal_priv_vap->reauth_state){
				vap->hal_priv_vap->reauth_state = 1;
				IEEE80211_SEND_MGMT(ni,
			   		 IEEE80211_FC0_SUBTYPE_AUTH, 1);
				return;
			}
			else{
				vap->hal_priv_vap->reauth_state = 0;
				vap->hal_priv_vap->conn_in_prog = 0;
			}
		}
		/*
		 * NB: it's safe to specify a timeout as the reason here;
		 *     it'll only be used in the right state.
		 */
		ieee80211_new_state(vap, IEEE80211_S_SCAN,
			IEEE80211_SCAN_FAIL_TIMEOUT);
	}
}

static void
ieee80211_tx_mgt_cb(struct ieee80211_node *ni, void *arg, int status)
{
	struct ieee80211vap *vap = ni->ni_vap;
	enum ieee80211_state ostate = (enum ieee80211_state) arg;

	/*
	 * Frame transmit completed; arrange timer callback.  If
	 * transmit was successfuly we wait for response.  Otherwise
	 * we arrange an immediate callback instead of doing the
	 * callback directly since we don't know what state the driver
	 * is in (e.g. what locks it is holding).  This work should
	 * not be too time-critical and not happen too often so the
	 * added overhead is acceptable.
	 *
	 * XXX what happens if !acked but response shows up before callback?
	 */
	/* Below arg value is changed to vap instead on ni, as ni may change
	 * while connecting to differnt AP. So at the time of expiry of timer 
	 * we get old AP's ni due to this we may try to connect to the older one
	 * which user doesnt need.
	 */
	if (vap->iv_state == ostate)
		callout_reset(&vap->iv_mgtsend,
			status == 0 ? msecs_to_jiffies(IEEE80211_TRANS_WAIT * 1000) : 0,
			ieee80211_tx_mgt_timeout, vap);
}

static void
ieee80211_beacon_construct(struct mbuf *m, uint8_t *frm,
	struct ieee80211_beacon_offsets *bo, struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct ieee80211_rateset *rs = &ni->ni_rates;
	uint16_t capinfo;

	/*
	 * beacon frame format
	 *	[8] time stamp
	 *	[2] beacon interval
	 *	[2] cabability information
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 *	[3] parameter set (DS)
	 *	[8] CF parameter set (optional)
	 *	[tlv] parameter set (IBSS/TIM)
	 *	[tlv] country (optional)
	 *	[3] power control (optional)
	 *	[5] channel switch announcement (CSA) (optional)
	 *	[tlv] extended rate phy (ERP)
	 *	[tlv] extended supported rates
	 *	[tlv] RSN parameters
	 *	[tlv] HT capabilities
	 *	[tlv] HT information
	 * XXX Vendor-specific OIDs (e.g. Atheros)
	 *	[tlv] WPA parameters
	 *	[tlv] WME parameters
	 *	[tlv] Vendor OUI HT capabilities (optional)
	 *	[tlv] Vendor OUI HT information (optional)
	 *	[tlv] Atheros capabilities (optional)
	 *	[tlv] TDMA parameters (optional)
	 *	[tlv] Mesh ID (MBSS)
	 *	[tlv] Mesh Conf (MBSS)
	 *	[tlv] application data (optional)
	 */

	memset(bo, 0, sizeof(*bo));

	memset(frm, 0, 8);	/* XXX timestamp is set by hardware/driver */
	frm += 8;
	*(uint16_t *)frm = ni->ni_intval;
	frm += 2;
	capinfo = ieee80211_getcapinfo(vap, ni->ni_chan);
	bo->bo_caps = (uint16_t *)frm;
	*(uint16_t *)frm = htole16(capinfo);
	frm += 2;
	*frm++ = IEEE80211_ELEMID_SSID;
	if ((vap->iv_flags & IEEE80211_F_HIDESSID) == 0) {
		*frm++ = ni->ni_esslen;
		memcpy(frm, ni->ni_essid, ni->ni_esslen);
		frm += ni->ni_esslen;
	} else
		*frm++ = 0;
#ifdef ENABLE_P2P_SUPPORT
	if((vap->p2p != NULL) && (vap->p2p_enable))
	{
		frm = ieee80211_add_only_11g_rates(frm, rs);
	}
	else
#endif
	{
		if(ic->ic_curchan->ic_ieee == 14) {
			frm = ieee80211_add_only_11b_rates(frm, rs);
		}
		else {
			frm = ieee80211_add_rates(frm, rs);
		}
	}
	if (!IEEE80211_IS_CHAN_FHSS(ni->ni_chan)) {
		*frm++ = IEEE80211_ELEMID_DSPARMS;
		*frm++ = 1;
		//*frm++ = ieee80211_chan2ieee(ic, ni->ni_chan);
		*frm++ = ic->ic_curchan->ic_ieee;
	}
	if (ic->ic_flags & IEEE80211_F_PCF) {
		bo->bo_cfp = frm;
		frm = ieee80211_add_cfparms(frm, ic);
	}
	bo->bo_tim = frm;
	if (vap->iv_opmode == IEEE80211_M_IBSS) {
		*frm++ = IEEE80211_ELEMID_IBSSPARMS;
		*frm++ = 2;
		*frm++ = 0; *frm++ = 0;		/* ATIM window */
		bo->bo_tim_len = 0;
	} else if (vap->iv_opmode == IEEE80211_M_HOSTAP ||
	    vap->iv_opmode == IEEE80211_M_MBSS
#ifdef ENABLE_P2P_SUPPORT
	    || vap->iv_opmode == IEEE80211_M_P2P
#endif
	) {
		/* TIM IE is the same for Mesh and Hostap */
		struct ieee80211_tim_ie *tie = (struct ieee80211_tim_ie *) frm;

		tie->tim_ie = IEEE80211_ELEMID_TIM;
		tie->tim_len = 4;	/* length */
		tie->tim_count = 0;	/* DTIM count */ 
		tie->tim_period = vap->iv_dtim_period;	/* DTIM period */
		tie->tim_bitctl = 0;	/* bitmap control */
		tie->tim_bitmap[0] = 0;	/* Partial Virtual Bitmap */
		frm += sizeof(struct ieee80211_tim_ie);
		bo->bo_tim_len = 1;
	}
	bo->bo_tim_trailer = frm;
	if (((vap->iv_flags & IEEE80211_F_DOTH) ||
	    (vap->iv_flags_ext & IEEE80211_FEXT_DOTD)) 
      && (!((ic->ic_regdomain.isocc[0] == 'W') && (ic->ic_regdomain.isocc[1] == 'R'))))
		frm = ieee80211_add_countryie(frm, ic);
	if (vap->iv_flags & IEEE80211_F_DOTH) {
		if (IEEE80211_IS_CHAN_5GHZ(ni->ni_chan))
			frm = ieee80211_add_powerconstraint(frm, vap);
		bo->bo_csa = frm;
		if (ic->ic_flags & IEEE80211_F_CSAPENDING)
			frm = ieee80211_add_csa(frm, vap);
	} else
		bo->bo_csa = frm;

	if (vap->iv_flags & IEEE80211_F_DOTH) {
		bo->bo_quiet = frm;
		if (IEEE80211_IS_CHAN_DFS(ic->ic_bsschan) &&
		    (vap->iv_flags_ext & IEEE80211_FEXT_DFS)) {
			if (vap->iv_quiet)
				frm = ieee80211_add_quiet(frm,vap);
		}
	} else
		bo->bo_quiet = frm;

	if (IEEE80211_IS_CHAN_ANYG(ni->ni_chan)) {
		bo->bo_erp = frm;
		frm = ieee80211_add_erp(frm, ic);
	}
	if(ic->ic_curchan->ic_ieee != 14)
	{
#ifdef ENABLE_P2P_SUPPORT
		if((vap->p2p == NULL) || (vap->p2p_enable == 0))
#endif
		{
			frm = ieee80211_add_xrates(frm, rs);
		}
	}

	if(ic->ic_curchan->ic_ieee != 14)
	{
		if (vap->iv_flags & IEEE80211_F_WME) {
			bo->bo_wme = frm;
			frm = ieee80211_add_wme_param(frm, &ic->ic_wme);
		}
	}

	if (vap->iv_flags & IEEE80211_F_WPA2) {
		if (vap->iv_rsn_ie != NULL)
			frm = add_ie(frm, vap->iv_rsn_ie);
		/* XXX else complain */
	}
	if(ic->ic_curchan->ic_ieee != 14)
	{
		if (IEEE80211_IS_CHAN_HT(ni->ni_chan)) {
			frm = ieee80211_add_htcap(frm, ni);
			bo->bo_htinfo = frm;
			frm = ieee80211_add_htinfo(frm, ni);
		}
	}

	if (vap->iv_flags & IEEE80211_F_WPA1) {
		if (vap->iv_wpa_ie != NULL)
			frm = add_ie(frm, vap->iv_wpa_ie);
		/* XXX else complain */
	}
	#if 0
	if (IEEE80211_IS_CHAN_HT(ni->ni_chan) &&
	    (vap->iv_flags_ht & IEEE80211_FHT_HTCOMPAT)) {
		frm = ieee80211_add_htcap_vendor(frm, ni);
		frm = ieee80211_add_htinfo_vendor(frm, ni);
	}
	#endif
#ifdef IEEE80211_SUPPORT_SUPERG
	if (vap->iv_flags & IEEE80211_F_ATHEROS) {
		bo->bo_ath = frm;
		frm = ieee80211_add_athcaps(frm, ni);
	}
#endif
#ifdef IEEE80211_SUPPORT_TDMA
	if (vap->iv_caps & IEEE80211_C_TDMA) {
		bo->bo_tdma = frm;
		frm = ieee80211_add_tdma(frm, vap);
	}
#endif

	if (ic->ic_apple_ie.add_apple_ie) {
		memcpy(frm, ic->ic_apple_ie.ie_data, ic->ic_apple_ie.ie_len);
		frm = (frm + ic->ic_apple_ie.ie_len);
	}
	if (vap->iv_appie_beacon != NULL) {
		bo->bo_appie = frm;
		bo->bo_appie_len = vap->iv_appie_beacon->ie_len;
		frm = add_appie(frm, vap->iv_appie_beacon);
	}

#ifdef IEEE80211_SUPPORT_MESH
	if (vap->iv_opmode == IEEE80211_M_MBSS) {
		frm = ieee80211_add_meshid(frm, vap);
		bo->bo_meshconf = frm;
		frm = ieee80211_add_meshconf(frm, vap);
	}
#endif
	bo->bo_tim_trailer_len = frm - bo->bo_tim_trailer;
	bo->bo_csa_trailer_len = frm - bo->bo_csa;
	m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
}

/*
 * Allocate a beacon frame and fillin the appropriate bits.
 */
struct mbuf *
ieee80211_beacon_alloc(struct ieee80211_node *ni,
	struct ieee80211_beacon_offsets *bo)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct ifnet *ifp = vap->iv_ifp;
	struct ieee80211_frame *wh;
	struct mbuf *m;
	int pktlen;
	uint8_t *frm;

	/*
	 * beacon frame format
	 *	[8] time stamp
	 *	[2] beacon interval
	 *	[2] cabability information
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 *	[3] parameter set (DS)
	 *	[8] CF parameter set (optional)
	 *	[tlv] parameter set (IBSS/TIM)
	 *	[tlv] country (optional)
	 *	[3] power control (optional)
	 *	[5] channel switch announcement (CSA) (optional)
	 *	[tlv] extended rate phy (ERP)
	 *	[tlv] extended supported rates
	 *	[tlv] RSN parameters
	 *	[tlv] HT capabilities
	 *	[tlv] HT information
	 *	[tlv] Vendor OUI HT capabilities (optional)
	 *	[tlv] Vendor OUI HT information (optional)
	 * XXX Vendor-specific OIDs (e.g. Atheros)
	 *	[tlv] WPA parameters
	 *	[tlv] WME parameters
	 *	[tlv] TDMA parameters (optional)
	 *	[tlv] Mesh ID (MBSS)
	 *	[tlv] Mesh Conf (MBSS)
	 *	[tlv] application data (optional)
	 * NB: we allocate the max space required for the TIM bitmap.
	 * XXX how big is this?
	 */
	pktlen =   8					/* time stamp */
		 + sizeof(uint16_t)			/* beacon interval */
		 + sizeof(uint16_t)			/* capabilities */
		 + 2 + ni->ni_esslen			/* ssid */
	         + 2 + IEEE80211_RATE_SIZE		/* supported rates */
	         + 2 + 1				/* DS parameters */
		 + 2 + 6				/* CF parameters */
		 + 2 + 4 + vap->iv_tim_len		/* DTIM/IBSSPARMS */
		 + IEEE80211_COUNTRY_MAX_SIZE		/* country */
		 + 2 + 1				/* power control */
		 + sizeof(struct ieee80211_csa_ie)	/* CSA */
		 + sizeof(struct ieee80211_quiet_ie)	/* Quiet */
		 + 2 + 1				/* ERP */
	         + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
		 + (vap->iv_caps & IEEE80211_C_WPA ?	/* WPA 1+2 */
			2*sizeof(struct ieee80211_ie_wpa) : 0)
		 /* XXX conditional? */
		 + 4+2*sizeof(struct ieee80211_ie_htcap)/* HT caps */
		 + 4+2*sizeof(struct ieee80211_ie_htinfo)/* HT info */
		 + (vap->iv_caps & IEEE80211_C_WME ?	/* WME */
			sizeof(struct ieee80211_wme_param) : 0)
#ifdef IEEE80211_SUPPORT_SUPERG
		 + sizeof(struct ieee80211_ath_ie)	/* ATH */
#endif
#ifdef IEEE80211_SUPPORT_TDMA
		 + (vap->iv_caps & IEEE80211_C_TDMA ?	/* TDMA */
			sizeof(struct ieee80211_tdma_param) : 0)
#endif
#ifdef IEEE80211_SUPPORT_MESH
		 + 2 + ni->ni_meshidlen
		 + sizeof(struct ieee80211_meshconf_ie)
#endif
		 + IEEE80211_MAX_APPIE
		 ;

	if (ic->ic_apple_ie.add_apple_ie) {
		ic->ic_apple_ie.jiffies = jiffies;
		pktlen += ic->ic_apple_ie.ie_len;
	}

	m = ieee80211_getmgtframe(&frm,
		ic->ic_headroom + sizeof(struct ieee80211_frame), pktlen);
	if (m == NULL) {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_ANY,
			"%s: cannot get buf; size %u\n", __func__, pktlen);
		vap->iv_stats.is_tx_nobuf++;
		return NULL;
	}
	ieee80211_beacon_construct(m, frm, bo, ni);

	M_PREPEND(m, sizeof(struct ieee80211_frame), M_DONTWAIT);
	KASSERT(m != NULL, ("no space for 802.11 header?"));
	wh = mtod(m, struct ieee80211_frame *);
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT |
	    IEEE80211_FC0_SUBTYPE_BEACON;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	*(uint16_t *)wh->i_dur = 0;
	IEEE80211_ADDR_COPY(wh->i_addr1, ifp->if_broadcastaddr);
	IEEE80211_ADDR_COPY(wh->i_addr2, vap->iv_myaddr);
	IEEE80211_ADDR_COPY(wh->i_addr3, ni->ni_bssid);
	*(uint16_t *)wh->i_seq = 0;

	return m;
}

/*
 * Update the dynamic parts of a beacon frame based on the current state.
 */
int
ieee80211_beacon_update(struct ieee80211_node *ni,
	struct ieee80211_beacon_offsets *bo, struct mbuf *m, int mcast)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	int len_changed = 0;
	uint16_t capinfo;
	struct ieee80211_frame *wh;
	ieee80211_seq seqno;

	IEEE80211_LOCK(ic);
	/*
	 * Handle 11h channel change when we've reached the count.
	 * We must recalculate the beacon frame contents to account
	 * for the new channel.  Note we do this only for the first
	 * vap that reaches this point; subsequent vaps just update
	 * their beacon state to reflect the recalculated channel.
	 */
	if (isset(bo->bo_flags, IEEE80211_BEACON_CSA) &&
	    vap->iv_csa_count == ic->ic_csa_count) {
		vap->iv_csa_count = 0;
		/*
		 * Effect channel change before reconstructing the beacon
		 * frame contents as many places reference ni_chan.
		 */
		if (ic->ic_csa_newchan != NULL)
			ieee80211_csa_completeswitch(ic);
		/*
		 * NB: ieee80211_beacon_construct clears all pending
		 * updates in bo_flags so we don't need to explicitly
		 * clear IEEE80211_BEACON_CSA.
		 */
		ieee80211_beacon_construct(m,
		    mtod(m, uint8_t*) + sizeof(struct ieee80211_frame), bo, ni);

		/* XXX do WME aggressive mode processing? */
		IEEE80211_UNLOCK(ic);
		return 1;		/* just assume length changed */
	}

	wh = mtod(m, struct ieee80211_frame *);
	seqno = ni->ni_txseqs[IEEE80211_NONQOS_TID]++;
	*(uint16_t *)&wh->i_seq[0] =
		htole16(seqno << IEEE80211_SEQ_SEQ_SHIFT);
	M_SEQNO_SET(m, seqno);

	/* XXX faster to recalculate entirely or just changes? */
	capinfo = ieee80211_getcapinfo(vap, ni->ni_chan);
	*bo->bo_caps = htole16(capinfo);

	if(ic->ic_curchan->ic_ieee != 14)
	{
		if (vap->iv_flags & IEEE80211_F_WME) {
			struct ieee80211_wme_state *wme = &ic->ic_wme;

			/*
			 * Check for agressive mode change.  When there is
			 * significant high priority traffic in the BSS
			 * throttle back BE traffic by using conservative
			 * parameters.  Otherwise BE uses agressive params
			 * to optimize performance of legacy/non-QoS traffic.
			 */
			if (wme->wme_flags & WME_F_AGGRMODE) {
				if (wme->wme_hipri_traffic >
						wme->wme_hipri_switch_thresh) {
					IEEE80211_DPRINTF(vap, IEEE80211_MSG_WME,
							"%s: traffic %u, disable aggressive mode\n",
							__func__, wme->wme_hipri_traffic);
					wme->wme_flags &= ~WME_F_AGGRMODE;
					ieee80211_wme_updateparams_locked(vap, 0);
					wme->wme_hipri_traffic =
						wme->wme_hipri_switch_hysteresis;
				} else
					wme->wme_hipri_traffic = 0;
			} else {
				if (wme->wme_hipri_traffic <=
						wme->wme_hipri_switch_thresh) {
					IEEE80211_DPRINTF(vap, IEEE80211_MSG_WME,
							"%s: traffic %u, enable aggressive mode\n",
							__func__, wme->wme_hipri_traffic);
					wme->wme_flags |= WME_F_AGGRMODE;
					ieee80211_wme_updateparams_locked(vap, 0);
					wme->wme_hipri_traffic = 0;
				} else
					wme->wme_hipri_traffic =
						wme->wme_hipri_switch_hysteresis;
			}
			if (isset(bo->bo_flags, IEEE80211_BEACON_WME)) {
				(void) ieee80211_add_wme_param(bo->bo_wme, wme);
				clrbit(bo->bo_flags, IEEE80211_BEACON_WME);
			}
		}

		if (isset(bo->bo_flags,  IEEE80211_BEACON_HTINFO)) {
			ieee80211_ht_update_beacon(vap, bo);
			clrbit(bo->bo_flags, IEEE80211_BEACON_HTINFO);
		}
	}
#ifdef IEEE80211_SUPPORT_TDMA
	if (vap->iv_caps & IEEE80211_C_TDMA) {
		/*
		 * NB: the beacon is potentially updated every TBTT.
		 */
		ieee80211_tdma_update_beacon(vap, bo);
	}
#endif
#ifdef IEEE80211_SUPPORT_MESH
	if (vap->iv_opmode == IEEE80211_M_MBSS)
		ieee80211_mesh_update_beacon(vap, bo);
#endif

	if (vap->iv_opmode == IEEE80211_M_HOSTAP ||
	    vap->iv_opmode == IEEE80211_M_MBSS
#ifdef ENABLE_P2P_SUPPORT
	    || vap->iv_opmode == IEEE80211_M_P2P
#endif
	) {	/* NB: no IBSS support*/
		struct ieee80211_tim_ie *tie =
			(struct ieee80211_tim_ie *) bo->bo_tim;
		if (isset(bo->bo_flags, IEEE80211_BEACON_TIM)) {
			u_int timlen, timoff, i;
			/* 
			 * ATIM/DTIM needs updating.  If it fits in the
			 * current space allocated then just copy in the
			 * new bits.  Otherwise we need to move any trailing
			 * data to make room.  Note that we know there is
			 * contiguous space because ieee80211_beacon_allocate
			 * insures there is space in the mbuf to write a
			 * maximal-size virtual bitmap (based on iv_max_aid).
			 */
			/*
			 * Calculate the bitmap size and offset, copy any
			 * trailer out of the way, and then copy in the
			 * new bitmap and update the information element.
			 * Note that the tim bitmap must contain at least
			 * one byte and any offset must be even.
			 */
			if (vap->iv_ps_pending != 0) {
				timoff = 128;		/* impossibly large */
				for (i = 0; i < vap->iv_tim_len; i++)
					if (vap->iv_tim_bitmap[i]) {
						timoff = i &~ 1;
						break;
					}
				KASSERT(timoff != 128, ("tim bitmap empty!"));
				for (i = vap->iv_tim_len-1; i >= timoff; i--)
					if (vap->iv_tim_bitmap[i])
						break;
				timlen = 1 + (i - timoff);
			} else {
				timoff = 0;
				timlen = 1;
			}
			if (timlen != bo->bo_tim_len) {
				/* copy up/down trailer */
				int adjust = tie->tim_bitmap+timlen
					   - bo->bo_tim_trailer;
				ovbcopy(bo->bo_tim_trailer,
				    bo->bo_tim_trailer+adjust,
				    bo->bo_tim_trailer_len);
				bo->bo_tim_trailer += adjust;
				bo->bo_erp += adjust;
				bo->bo_htinfo += adjust;
#ifdef IEEE80211_SUPPORT_SUPERG
				bo->bo_ath += adjust;
#endif
#ifdef IEEE80211_SUPPORT_TDMA
				bo->bo_tdma += adjust;
#endif
#ifdef IEEE80211_SUPPORT_MESH
				bo->bo_meshconf += adjust;
#endif
				bo->bo_appie += adjust;
				bo->bo_wme += adjust;
				bo->bo_csa += adjust;
				bo->bo_quiet += adjust;
				bo->bo_tim_len = timlen;

				/* update information element */
				tie->tim_len = 3 + timlen;
				tie->tim_bitctl = timoff;
				len_changed = 1;
			}
			memcpy(tie->tim_bitmap, vap->iv_tim_bitmap + timoff,
				bo->bo_tim_len);

			clrbit(bo->bo_flags, IEEE80211_BEACON_TIM);

			IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER,
				"%s: TIM updated, pending %u, off %u, len %u\n",
				__func__, vap->iv_ps_pending, timoff, timlen);
		}
		/* count down DTIM period */
		if (tie->tim_count == 0)
			tie->tim_count = tie->tim_period - 1;
		else
			tie->tim_count--;
		/* update state for buffered multicast frames on DTIM */
		if (mcast && tie->tim_count == 0)
			tie->tim_bitctl |= 1;
		else
			tie->tim_bitctl &= ~1;

		if (isset(bo->bo_flags, IEEE80211_BEACON_CSA)) {
			struct ieee80211_csa_ie *csa =
			    (struct ieee80211_csa_ie *) bo->bo_csa;

			/*
			 * Insert or update CSA ie.  If we're just starting
			 * to count down to the channel switch then we need
			 * to insert the CSA ie.  Otherwise we just need to
			 * drop the count.  The actual change happens above
			 * when the vap's count reaches the target count.
			 */
			if (vap->iv_csa_count == 0) {
				memmove(&csa[1], csa, bo->bo_csa_trailer_len);
				bo->bo_erp += sizeof(*csa);
				bo->bo_htinfo += sizeof(*csa);
				bo->bo_wme += sizeof(*csa);
#ifdef IEEE80211_SUPPORT_SUPERG
				bo->bo_ath += sizeof(*csa);
#endif
#ifdef IEEE80211_SUPPORT_TDMA
				bo->bo_tdma += sizeof(*csa);
#endif
#ifdef IEEE80211_SUPPORT_MESH
				bo->bo_meshconf += sizeof(*csa);
#endif
				bo->bo_appie += sizeof(*csa);
				bo->bo_csa_trailer_len += sizeof(*csa);
				bo->bo_quiet += sizeof(*csa);
				bo->bo_tim_trailer_len += sizeof(*csa);
				m->m_len += sizeof(*csa);
				m->m_pkthdr.len += sizeof(*csa);

				ieee80211_add_csa(bo->bo_csa, vap);
			} else
				csa->csa_count--;
			vap->iv_csa_count++;
			/* NB: don't clear IEEE80211_BEACON_CSA */
		}
		if (IEEE80211_IS_CHAN_DFS(ic->ic_bsschan) &&
		    (vap->iv_flags_ext & IEEE80211_FEXT_DFS) ){
			if (vap->iv_quiet)
				ieee80211_add_quiet(bo->bo_quiet, vap);
		}
		if (isset(bo->bo_flags, IEEE80211_BEACON_ERP)) {
			/*
			 * ERP element needs updating.
			 */
			(void) ieee80211_add_erp(bo->bo_erp, ic);
			clrbit(bo->bo_flags, IEEE80211_BEACON_ERP);
		}
#ifdef IEEE80211_SUPPORT_SUPERG
		if (isset(bo->bo_flags,  IEEE80211_BEACON_ATH)) {
			ieee80211_add_athcaps(bo->bo_ath, ni);
			clrbit(bo->bo_flags, IEEE80211_BEACON_ATH);
		}
#endif
	}
	if (isset(bo->bo_flags, IEEE80211_BEACON_APPIE)) {
		const struct ieee80211_appie *aie = vap->iv_appie_beacon;
		int aielen;
		uint8_t *frm;

		aielen = 0;
		if (aie != NULL)
			aielen += aie->ie_len;
		if (aielen != bo->bo_appie_len) {
			/* copy up/down trailer */
			int adjust = aielen - bo->bo_appie_len;
#if 0
			ovbcopy(bo->bo_tim_trailer, bo->bo_tim_trailer+adjust,
				bo->bo_tim_trailer_len);
			bo->bo_tim_trailer += adjust;
			bo->bo_appie += adjust;
#endif
			bo->bo_appie_len = aielen;
			m->m_len += adjust;

			len_changed = 1;
		}
		frm = bo->bo_appie;
		if (aie != NULL)
			frm  = add_appie(frm, aie);
		clrbit(bo->bo_flags, IEEE80211_BEACON_APPIE);
	}
	IEEE80211_UNLOCK(ic);

	return len_changed;
}
EXPORT_SYMBOL(ieee80211_beacon_alloc);
EXPORT_SYMBOL(ieee80211_beacon_update);
EXPORT_SYMBOL(ieee80211_encap);
EXPORT_SYMBOL(ieee80211_add_rates);
EXPORT_SYMBOL(ieee80211_add_xrates);
