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

#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif
#include "ieee80211_linux.h"
#if KERNEL_VERSION_BTWN_2_6_(30,35)
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#ifndef ifr_media
#define ifr_media ifr_ifru.ifru_ivalue
#endif
#include <asm/uaccess.h>
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>
#include "ndiface.h"
#include "channel_utilization.h"


void null_function(void)
{
	return;
}

void bpf_mtap(struct ifnet *ifp, struct mbuf *m)
{
	return;
}

uint8_t ieee80211_check_queue_status(struct ifnet *ifp, struct ieee80211vap *vap)
{
#ifdef USE_SUBQUEUES
	if (vap->iv_opmode == IEEE80211_M_STA) {
		if (vap->hal_priv_vap->stop_per_q[BE_Q_STA])
			return 1;
	} else {
		if (vap->hal_priv_vap->stop_per_q[BE_Q_AP])
			return 1;
	}
#else
	if (vap->iv_opmode == IEEE80211_M_STA) {
		if (vap->hal_priv_vap->stop_per_q[BE_Q_STA] || vap->hal_priv_vap->stop_per_q[BK_Q_STA] ||
		    vap->hal_priv_vap->stop_per_q[VO_Q_STA] || vap->hal_priv_vap->stop_per_q[VI_Q_STA]) {
			return 1;
    }
	} else {
		if (vap->hal_priv_vap->stop_per_q[BE_Q_AP] || vap->hal_priv_vap->stop_per_q[BK_Q_AP] ||
		    vap->hal_priv_vap->stop_per_q[VO_Q_AP] || vap->hal_priv_vap->stop_per_q[VI_Q_AP])
			return 1;
	}
#endif
	return 0;
}


void bpf_mtap2(struct bpf_if *vap, void *d,int l,struct mbuf *m, int tx_path)
{
	struct mbuf *m_copy = NULL;
	struct sk_buff *skb;
	struct radiotap_header *th;
	uint64_t timestamp;
	uint32_t status_word;
	uint8_t crc, band_width, short_gi, ht, mcs, ch_num;
	int rssi, rate;

	m_copy = m_dup(m, 0);
	if (m_copy == NULL)
		return;

	skb = onebox_translate_mbuf_to_skb(m_copy);
	skb->dev = vap->iv_ifp;

	rssi 				= skb->data[0];
	rate 				= (*(int16_t *)&skb->data[2]) & 0x01ff;
	timestamp 	= *(int64_t *)&skb->data[4];
	crc  				= ((skb->data[15] >> 5) & 1);
	status_word = *(int32_t *)&skb->data[12];
	band_width  = (*(int8_t *)&skb->data[3]) & 0x30;
	short_gi    = (*(int8_t *)&skb->data[3]) & 0x06;
	ht			    = (*(int8_t *)&skb->data[3]) & 0x01;
	mcs			    = (*(int8_t *)&skb->data[2]) & 0x0f;
	ch_num    	= *(int8_t *)&skb->data[16];

	if (skb_headroom(skb) < sizeof(struct radiotap_header))
	{
		dev_kfree_skb(skb);
		kfree(m_copy);
		skb = NULL;
		return;
	}
	else 
	{
		if( vap->iv_ic->driver_mode == 7 )
		{
			skb_pull(skb, vap->hal_priv_vap->extended_desc_size);
		}
		th = (struct radiotap_header *) skb_push(skb, sizeof(struct radiotap_header));
		memcpy(th, d, l);
	}

    if( vap->iv_ic->driver_mode == 7 ) /* Checking if the driver mode is sniffer mode */
	{
		th->wt_ihdr.it_len = cpu_to_le16(sizeof(struct radiotap_header));
		th->wt_ihdr.it_present = BIT(IEEE80211_RADIOTAP_RATE) 
			| BIT(IEEE80211_RADIOTAP_DBM_ANTSIGNAL) 
			| BIT(IEEE80211_RADIOTAP_TSFT) 
			| BIT(IEEE80211_RADIOTAP_CHANNEL) 
			| BIT(IEEE80211_RADIOTAP_FLAGS)
			| BIT(IEEE80211_RADIOTAP_RX_FLAGS);

		switch(rate)
		{
			case 0   :
				th->wt_rate = 2;
				break;
			case 0x2 :
				th->wt_rate = 4;
				break;
			case 0x4 :
				th->wt_rate = 11;
				break;
			case 0x6 :
				th->wt_rate = 22;
				break;
			case 0x8b :
				th->wt_rate = 12;
				break;
			case 0x8f :
				th->wt_rate = 18;
				break;
			case 0x8a :
				th->wt_rate = 24;
				break;
			case 0x8e :
				th->wt_rate = 36;
				break;
			case 0x89 :
				th->wt_rate = 48;
				break;
			case 0x8d :
				th->wt_rate = 72;
				break;
			case 0x88 :
				th->wt_rate = 96;
				break;
			case 0x8c :
				th->wt_rate = 108;
				break;
			default 	:
				if(ht)
					th->wt_rate = 0;
				else
					IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Invalid Rate \n"));
				break;
		}

		th->wt_tsft = timestamp;
		if(short_gi)
		{
			th->wt_flags  |= IEEE80211_RADIOTAP_F_SHORTGI;
			th->mcs_flags |= IEEE80211_MCS_SGI;	
		}
		if(crc << 6)
		{
			th->wt_flags  |= IEEE80211_RADIOTAP_F_BADFCS;
		}

		if(band_width)
		{
			th->mcs_flags |= IEEE80211_RADIOTAP_MCS_BW_40;
		}

		if(ht)
		{
			th->wt_ihdr.it_present |= BIT(IEEE80211_RADIOTAP_MCS);
			th->mcs_present |= IEEE80211_RADIOTAP_MCS_HAVE_MCS;
		}

		th->mcs_present |= IEEE80211_RADIOTAP_MCS_HAVE_BW;
		th->mcs_present |= IEEE80211_RADIOTAP_MCS_HAVE_GI;
		th->mcs_index    = mcs;

		th->wt_flags 		|= IEEE80211_RADIOTAP_F_FCS ;
		th->wt_chan_freq = vap->iv_ic->ic_curchan->ic_freq;

		if((th->wt_chan_freq) >= 2412 && (th->wt_chan_freq) <= 2484)
		{
			th->wt_chan_flags |= IEEE80211_CHAN_2GHZ;
		}
		else
		{
			th->wt_chan_flags |= (IEEE80211_CHAN_5GHZ) | (IEEE80211_CHAN_OFDM);
		}

		th->wt_dbm_antsignal = rssi;

		if(ch_num != vap->iv_ic->ic_curchan->ic_ieee)
		{
			dev_kfree_skb(skb);
			kfree(m_copy);
			return;
		}

	}
	skb_reset_mac_header(skb);
	skb->protocol = ETH_P_80211_RAW;
	netif_rx(skb);  
	kfree(m_copy);
	return;
}

int ether_ioctl(struct ifnet *ifp,unsigned long command, struct ifreq *ptr)
{
	return 0;
}
void if_initname(struct ifnet *ifp,const char *name,int unit)
{
	int err;
	if (name != NULL)
	{
		if (strchr(name, '%'))
		{
			if ((err = dev_alloc_name(ifp, name)) < 0) 
			{
				//return err;
				return;
			}
		}
		else
		{
			strncpy(ifp->name, name, sizeof(ifp->name));
		}
	}
	ifp->ifindex = unit;
	return;
}

void set_tx_queue_len(struct ifnet *ifp,int max_len)
{
	ifp->tx_queue_len = max_len;
	return;
}

extern struct iw_handler_def ieee80211_iw_handler_def;
struct net_device* if_alloc(int opmode)
{
	struct net_device *dev = NULL;

#if KERNEL_VERSION_GREATER_THAN_2_6_(27)
	static struct net_device_ops net80211_netdev_ops =
	{
		.ndo_open           = ieee80211_init,
		.ndo_stop           = ieee80211_ifstop,
		.ndo_start_xmit     = ieee80211_hardstart,
		.ndo_do_ioctl       = ieee80211_ioctl,
	};
#endif

	if(opmode == IEEE80211_M_MONITOR)
	{
		net80211_netdev_ops.ndo_start_xmit = ieee80211_null_transmit;
	}
	/* Allocate memory for the netdevice */
#ifndef USE_SUBQUEUES
	dev = alloc_etherdev(sizeof(struct ieee80211vap));
#else
	dev = alloc_etherdev_mq(sizeof(struct ieee80211vap), 4);
#endif
	if (dev == NULL) 
	{
		printk("Unable to create ether dev \n");
		return NULL;
	}	
#if KERNEL_VERSION_BTWN_2_6_(18,27)
	dev->hard_start_xmit = ieee80211_hardstart;
	dev->if_stop  = ieee80211_ifstop;
	dev->if_ioctl = ieee80211_ioctl;
	dev->if_init = ieee80211_init;
#endif

/* Register the netdevice operations */
#if KERNEL_VERSION_GREATER_THAN_2_6_(27)
	dev->netdev_ops = &net80211_netdev_ops;	 
#endif

#ifdef CONFIG_WIRELESS_EXT
	dev->wireless_handlers = &ieee80211_iw_handler_def;
#endif
	return dev;
}

void if_attach(struct ifnet *ifp)
{
	return;
}

void if_detach(struct ifnet *ifp)
{
	return;
}

#if 0
void if_start(struct ifnet *ifp)
{
	ifp->if_start(NULL, ifp);
}
#endif

int ieee80211_null_transmit(struct sk_buff *skb, struct ifnet *ifp)
{
	dev_kfree_skb(skb);
	ifp->stats.tx_errors++;
	return 0;
}

int ieee80211_hardstart(struct sk_buff *skb, struct ifnet *ifp)
{
	uint8_t eapol_pkt = 0;
	netbuf_ctrl_block_m_t *netbuf_cb_m;
	struct ifaltq *if_snd_q = ifp->ml_priv;
	struct ieee80211_node *ni = NULL;
#if KERNEL_VERSION_BTWN_2_6_(18,22)
	struct ieee80211vap *vap = ifp->priv;
#else
	struct ieee80211vap *vap = netdev_priv(ifp);
#endif

	ni = vap->iv_bss;
	if(vap->hal_priv_vap->stop_tx_q) {
		
		if_printf(ifp, "%s %d: Dropping pkts as queue is stopped\n", __func__, __LINE__);
	}
	if ((skb->data[ETH_PROTOCOL_OFFSET] == 0x00) && (skb->data[ETH_PROTOCOL_OFFSET+1] == 0x60))
	{
		if_printf(ifp, "%s:Dropping XNS protocol Packets\n", __func__);
		goto xmit_fail;
	}

	/*Dropping pkts other than EAPOL's */
	if (ieee80211_check_queue_status(ifp, vap))
	{
		if ((skb->data[ETH_PROTOCOL_OFFSET] == 0x88) && (skb->data[ETH_PROTOCOL_OFFSET + 1] == 0x8e)) {
			eapol_pkt = 1;
		} else {
			goto xmit_fail;
		}
	}
#ifndef INET6
	/* To Avoid IPV6 Packets
	 * Now dropping - To be added in default queue */
	if ((skb->data[ETH_PROTOCOL_OFFSET] == 0x86) && 
	    (skb->data[ETH_PROTOCOL_OFFSET + 1] == 0xdd)) 
	{
		if_printf(ifp, "%s: IPv6 Pkt\n", __func__);
		goto xmit_fail;
	}
#endif

#ifdef PWR_SAVE_SUPPORT
	if((vap->iv_opmode == IEEE80211_M_STA) && (TRAFFIC_PS_EN) 
			&& (ps_params_def.ps_en))
	{
		/* acquire semaphore lock */
		vap->check_traffic(vap, 1, skb->len); //1 ->indicates tx path
		/* Release semaphore */
	}
#endif
	if(vap->iv_bss)
	{
		ni->ni_stats.ns_tx_data++;
		ni->ni_stats.ns_tx_bytes += skb->len;
	}
	if(skb_headroom(skb) < ONEBOX_NEEDED_HEADROOM)
	{
		struct sk_buff *new_skb;
		new_skb= dev_alloc_skb(skb->len + ONEBOX_NEEDED_HEADROOM);
		skb_reserve(new_skb,ONEBOX_NEEDED_HEADROOM); //Extra 2 bytes is to align the finally packet to dword as it is giving problem with
		// Few embedded platforms which use DMA for SDIO transfers if not aligned to dword
		// The extra 2bytes here will be compensated by the 6 bytes done while removing ethernet
		// header and inserting LLC header
		skb_put(new_skb, skb->len);
		memcpy(new_skb->data, skb->data, skb->len);
		memcpy(new_skb->cb, skb->cb, 48);
		dev_kfree_skb(skb);
		skb = new_skb;
	}

	if (skb->data[ETH_HDR_OFFSET] & 0x1)
	{
		SKB_CB(skb)->flags |= ONEBOX_MULTICAST;
	}
	else if (skb->data[ETH_HDR_OFFSET] == 0xFF)
	{
		SKB_CB(skb)->flags |= ONEBOX_BROADCAST;
	}
#if 0
	else
	{
		if(vap->hal_priv_vap->stop_udp_pkts)
		{
			struct iphdr *ip = (struct iphdr *)&skb->data[IP_HDR_OFFSET];
			/* Drop unicast UDP packets if stop is set */
			if((*(uint16_t *)&skb->data[ETH_PROTOCOL_OFFSET] == cpu_to_be16(0x0800)) && /*wether IP packet*/
			   (ip->protocol == 0x11)) /* wether UDP packet */
			{
				udp_pkts_dropped++;
				if(udp_pkts_dropped && !(udp_pkts_dropped % 10000))
				{
					//printk("no of udp_pkts_dropped = %d\n", udp_pkts_dropped);
				}
				goto xmit_fail;
			}
		}
	}
#endif


#if 0
	int i = 0;
	printk("Pkt dump in %s %d skb->len=%d\n", __func__, __LINE__, skb->len);
	for(i=0; i< 120; i++)
	{
		if(i && ((i % 16) == 0))
			printk("\n");
		printk("%02x ", skb->data[i]);
	}
	printk("\n");
#endif
	netbuf_cb_m = onebox_translate_skb_to_netcb_m(skb);
	netbuf_cb_m->skb_priority = skb_get_queue_mapping(skb);

	if (eapol_pkt) {
		IF_PREPEND(if_snd_q, netbuf_cb_m);
	} else {
		IF_ENQUEUE(if_snd_q, netbuf_cb_m);
	}

	ieee80211_start(ifp);
	return 0;

xmit_fail:
	if (skb)
	{
		dev_kfree_skb(skb);
	}
	return 0;
}


void indicate_pkt_to_os(struct ifnet *ifp, struct mbuf *m)
{
	struct sk_buff *skb;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0))
	int err;
#endif
	uint8_t *pkt;
	pkt = m->m_data;
	if((pkt[0] == (IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS_NULL))
		 || (pkt[0] == (IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_NODATA)))
	{
		/* Drop QOS null data / Null data packets without indicating to OS */
		dev_kfree_skb((struct sk_buff *)m->pkt_addr);
		kfree(m);
		return;
	}
	skb = onebox_translate_mbuf_to_skb(m);
	skb->dev = ifp;
	skb->protocol = onebox_type_trans(skb, ifp);
	memset(skb->cb, 0, sizeof(skb->cb));

	/* Function is present only in kernels 2.4 and above */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,0))
	//netif_receive_skb(skb); /* Better to check the return value */
	//netif_rx(skb);
	netif_rx_ni(skb);
#else
	err = netif_rx(skb);
	
#endif

	kfree(m);
}
/*
 * The kernel version of this function alters the skb in a manner
 * inconsistent with dev->hard_header_len header reservation. This 
 * is a rewrite of the portion of eth_type_trans() that we need.
 */
uint16_t onebox_type_trans(struct sk_buff *skb, struct net_device *dev)
{
	struct ethhdr *eth;

	skb_reset_mac_header(skb);
	skb_pull(skb, ETH_HLEN);
	eth = (struct ethhdr *)skb_mac_header(skb);

	if (*eth->h_dest & 1)
	{
		if (memcmp(eth->h_dest, dev->broadcast, ETH_ALEN) == 0)
		{
			skb->pkt_type = PACKET_BROADCAST;
		}
		else
		{
			skb->pkt_type = PACKET_MULTICAST;
		}
	}
	else if (memcmp(eth->h_dest, dev->dev_addr, ETH_ALEN))
	{
		skb->pkt_type = PACKET_OTHERHOST;
	}
	if (ntohs(eth->h_proto) >= 1536)
	{
		return eth->h_proto;
	}
	return htons(ETH_P_802_2);
	//return 0;
}


/*
 * Perform common duties while attaching to interface list
 */
int ether_ifattach(struct ifnet *ifp, const u_int8_t *lla)
{
	//memcpy(ifp->dev_addr, ifp->priv->iv_myaddr,ETHER_ADDR_LEN);
#if KERNEL_VERSION_LESS_THAN_4_11_(9)
	ifp->destructor = free_netdev;
#else
	ifp->priv_destructor = free_netdev;
#endif
	if(register_netdevice(ifp))
	{
		printk("onebox: failed in registering interface %s\n",ifp->name);
		return -1;
	}
	return 0;
}

/*
 * Perform common duties while detaching an Ethernet interface
 */
void if_free(struct ifnet *ifp, uint8_t wait_for_lock)
{
#if KERNEL_VERSION_BTWN_2_6_(18,22)
	struct ieee80211vap *vap = ifp->priv;
#else
	struct ieee80211vap *vap = netdev_priv(ifp);
#endif

	kfree(ifp->ml_priv);
	/* Let's do the freeing first */
	kfree(vap->hal_priv_vap->core_vp);
	kfree(vap->hal_priv_vap);
	vap->hal_priv_vap = NULL; 

	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, 
	    ("rtnl_lock status is %d \n",rtnl_is_locked()));

#if 1
	if (ifp->reg_state == NETREG_REGISTERED) {
		if (wait_for_lock) {
			//unregister_netdevice(ifp);
			unregister_netdev(ifp);
			//rtnl_unlock();
		} else
			unregister_netdevice(ifp);
	}
#endif

//#ifdef ONEBOX_CONFIG_CFG80211
//	cfg80211_free_wdev(ifp->ieee80211_ptr);
//#endif

}

/*
 * Defined in freebsd.c and not being called from other functions
   Used to get the device pointer out of the parent device.
 */
struct ifnet * ifunit(const char *name)
{
	return NULL;
}

/*
 * Delete all multicast group membership for an interface.
 * Should be used to quickly flush all multicast filters.
 */
void
if_delallmulti(struct ifnet *ifp)
{
#if 0
	struct ifmultiaddr *ifma;
	struct ifmultiaddr *next;
	IF_ADDR_LOCK(ifp);
	TAILQ_FOREACH_SAFE(ifma, &ifp->if_multiaddrs, ifma_link, next)
		if_delmulti_locked(ifp, ifma, 0);
	IF_ADDR_UNLOCK(ifp);
#endif
}

/*
 * Perform deletion of network-layer and/or link-layer multicast address.
 *
 * Return 0 if the reference count was decremented.
 * Return 1 if the final reference was released, indicating that the
 * hardware hash filter should be reprogrammed.
 */
int
if_delmulti_locked(struct ifnet *ifp, struct ifmultiaddr *ifma, int detaching)
{
	struct ifmultiaddr *ll_ifma;

	if (ifp != NULL && ifma->ifma_ifp != NULL)
	{
		KASSERT(ifma->ifma_ifp == ifp,
		        ("%s: inconsistent ifp %p", __func__, ifp));
		//IF_ADDR_LOCK_ASSERT(ifp); 
	}

	ifp = ifma->ifma_ifp;

	/*
	 * If the ifnet is detaching, null out references to ifnet,
	 * so that upper protocol layers will notice, and not attempt
	 * to obtain locks for an ifnet which no longer exists. The
	 * routing socket announcement must happen before the ifnet
	 * instance is detached from the system.
	 */
	if (detaching) 
	{
		#ifdef DIAGNOSTIC
		printf("%s: detaching ifnet instance %p\n", __func__, ifp);
		#endif
		/*
		 * ifp may already be nulled out if we are being reentered
		 * to delete the ll_ifma.
		 */
		if (ifp != NULL) 
		{
			rt_newmaddrmsg(RTM_DELMADDR, ifma);
			ifma->ifma_ifp = NULL;
		}
	}

	if (--ifma->ifma_refcount > 0)
	{
		return 0;
	}
	/*
	 * If this ifma is a network-layer ifma, a link-layer ifma may
	 * have been associated with it. Release it first if so.
	 */
	ll_ifma = ifma->ifma_llifma;
	if (ll_ifma != NULL) 
	{
		KASSERT(ifma->ifma_lladdr != NULL,
		        ("%s: llifma w/o lladdr", __func__));
		if (detaching)
		{
			ll_ifma->ifma_ifp = NULL;	/* XXX */
		}
		if (--ll_ifma->ifma_refcount == 0) 
		{
			if (ifp != NULL)
			{
				//TAILQ_REMOVE(&ifp->if_multiaddrs, ll_ifma,
				  //  ifma_link); 
			}
			if_freemulti(ll_ifma);
		}
	}

#if 0 
	if (ifp != NULL)
		TAILQ_REMOVE(&ifp->if_multiaddrs, ifma, ifma_link);
#endif 

	if_freemulti(ifma);

	/*
	 * The last reference to this instance of struct ifmultiaddr
	 * was released; the hardware should be notified of this change.
	 */
	return 1;
}

/*
 * if_freemulti: free ifmultiaddr structure and possibly attached related
 * addresses.  The caller is responsible for implementing reference
 * counting, notifying the driver, handling routing messages, and releasing
 * any dependent link layer state.
 */
void
if_freemulti(struct ifmultiaddr *ifma)
{

	KASSERT(ifma->ifma_refcount == 0, ("if_freemulti: refcount %d",
	    ifma->ifma_refcount));
	KASSERT(ifma->ifma_protospec == NULL,
	    ("if_freemulti: protospec not NULL"));

	if (ifma->ifma_lladdr != NULL)
	{
		free(ifma->ifma_lladdr, M_IFMADDR);
	}
	free(ifma->ifma_addr, M_IFMADDR);
	free(ifma, M_IFMADDR);
}

/*
 * Compile-time options:
 * IFMEDIA_DEBUG:
 *	turn on implementation-level debug printfs.
 * 	Useful for debugging newly-ported  drivers.
 */

struct ifmedia_entry *ifmedia_match(struct ifmedia *, int, int);

#ifdef IFMEDIA_DEBUG
int ifmedia_debug = 0;
static void ifmedia_printword(int);
#endif

/*
 * Initialize if_media struct for a specific interface instance.
 */
void
ifmedia_init(struct ifmedia *ifm, 
             int dontcare_mask,
             ifm_change_cb_t change_callback, 
             ifm_stat_cb_t status_callback)
{
	LIST_INIT(&ifm->ifm_list);
	ifm->ifm_cur = NULL;
	ifm->ifm_media = 0;
	ifm->ifm_mask = dontcare_mask;        /* IF don't-care bits */
	ifm->ifm_change = change_callback;
	ifm->ifm_status = status_callback;
}

void
ifmedia_removeall(struct ifmedia *ifm)
{
	struct ifmedia_entry *entry;

	for (entry = LIST_FIRST(&ifm->ifm_list); entry;
	     entry = LIST_FIRST(&ifm->ifm_list))
	{
		LIST_REMOVE(entry, ifm_list);
		kfree(entry);
	}
}

/*
 * Add a media configuration to the list of supported media
 * for a specific interface instance.
 */
void
ifmedia_add(struct ifmedia *ifm, int mword, int data, void *aux)
{
	register struct ifmedia_entry *entry;

#ifdef IFMEDIA_DEBUG
	if (ifmedia_debug) 
	{
		if (ifm == NULL) 
		{
			printk(KERN_ERR "ifmedia_add: null ifm\n");
			return;
		}
		//printk(KERN_INFO "Adding entry for ");
		ifmedia_printword(mword);
	}
#endif

	entry = kmalloc(sizeof(*entry), GFP_KERNEL);
	if (entry == NULL)
	{
		panic("ifmedia_add: can't malloc entry");
	}
	entry->ifm_media = mword;
	entry->ifm_data = data;
	entry->ifm_aux = aux;

	LIST_INSERT_HEAD(&ifm->ifm_list, entry, ifm_list);
}

/*
 * Add an array of media configurations to the list of
 * supported media for a specific interface instance.
 */
void
ifmedia_list_add(struct ifmedia *ifm, struct ifmedia_entry *lp, int count)
{
	int i;

	for (i = 0; i < count; i++)
	{
		ifmedia_add(ifm, lp[i].ifm_media, lp[i].ifm_data, lp[i].ifm_aux);
	}
}



/*
 * Set the default active media. 
 *
 * Called by device-specific code which is assumed to have already
 * selected the default media in hardware.  We do _not_ call the
 * media-change callback.
 */
void
ifmedia_set(struct ifmedia *ifm, int target)
{
	struct ifmedia_entry *match;

	match = ifmedia_match(ifm, target, ifm->ifm_mask);

	if (match == NULL) 
	{
		printk(KERN_ERR "ifmedia_set: no match for 0x%x/0x%x\n",
		       target, ~ifm->ifm_mask);
		panic("ifmedia_set");
	}
	ifm->ifm_cur = match;

#ifdef IFMEDIA_DEBUG
	if (ifmedia_debug) 
	{
		printk(KERN_INFO "ifmedia_set: target ");
		ifmedia_printword(target);
		printk(KERN_INFO "ifmedia_set: setting to ");
		ifmedia_printword(ifm->ifm_cur->ifm_media);
	}
#endif
}

/*
 * Device-independent media ioctl support function.
 */
int
ifmedia_ioctl(struct net_device *dev, struct ifreq *ifr,
              struct ifmedia *ifm, u_long cmd)
{
	struct ifmedia_entry *match;
	struct ifmediareq *ifmr;
	int error = 0, sticky;

	if (dev == NULL || ifr == NULL || ifm == NULL)
	{
		return -EINVAL;
	}

	ifmr = kmalloc(sizeof(struct ifmediareq), GFP_KERNEL);
	if(ifmr == NULL)
	{
		return -1;
	}
	memset(ifmr, 0, sizeof(struct ifmediareq));
	switch (cmd) 
	{
		/*
		 * Set the current media.
		 */
		case  SIOCSIFMEDIA:
		{
			struct ifmedia_entry *oldentry;
			int oldmedia;
			int newmedia = ifr->ifr_media;
		
			match = ifmedia_match(ifm, newmedia, ifm->ifm_mask);
			if (match == NULL) 
			{
#ifdef IFMEDIA_DEBUG 
				if (ifmedia_debug) 
				{
					printk("ifmedia_ioctl: no media found for 0x%x\n",
					newmedia);
				}
#endif
				error = -ENXIO;
				break;
			}
		
			/*
			 * If no change, we're done.
			 * XXX Automedia may invole software intervention.
			 *     Keep going in case the the connected media changed.
			 *     Similarly, if best match changed (kernel debugger?).
			 */
			if ((IFM_SUBTYPE(newmedia) != IFM_AUTO) &&
			    (newmedia == ifm->ifm_media) &&
			    (match == ifm->ifm_cur))
			{
				break;
			}
			/*
			 * We found a match, now make the driver switch to it.
			 * Make sure to preserve our old media type in case the
			 * driver can't switch.
			 */
#ifdef IFMEDIA_DEBUG
			if (ifmedia_debug)
			{
				printk("ifmedia_ioctl: switching %s to ", dev->name);
				ifmedia_printword(match->ifm_media);
			}
#endif
			oldentry = ifm->ifm_cur;
			oldmedia = ifm->ifm_media;
			ifm->ifm_cur = match;
			ifm->ifm_media = newmedia;
			error = (*ifm->ifm_change)(dev);
			if ((error < 0) && (error != -ENETRESET)) 
			{
				ifm->ifm_cur = oldentry;
				ifm->ifm_media = oldmedia;
			}
			break;
		}
		
		/*
		 * Get list of available media and current media on interface.
		 */
		case  SIOCGIFMEDIA:
		{
			struct ifmedia_entry *ep;
			int *kptr, count;
			int usermax; /* user requested max */
		
			kptr = NULL; /* XXX gcc */
		
			ifmr->ifm_active = ifmr->ifm_current = ifm->ifm_cur ?
			                   ifm->ifm_cur->ifm_media : IFM_NONE;
			ifmr->ifm_mask = ifm->ifm_mask;
			ifmr->ifm_status = 0;
			(*ifm->ifm_status)(dev, ifmr);
		
			count = 0;
			usermax = 0;
		
			/*
			 * If there are more interfaces on the list, count
			 * them.  This allows the caller to set ifmr->ifm_count
			 * to 0 on the first call to know how much space to
			 * allocate.
			 */
			LIST_FOREACH(ep, &ifm->ifm_list, ifm_list)
				usermax++;
			/*
			 * Don't allow the user to ask for too many
			 * or a negative number.
			 */
			if (ifmr->ifm_count > usermax)
			{
				ifmr->ifm_count = usermax;
			}
			else if (ifmr->ifm_count < 0)
			{
				error = (-EINVAL);
				break;
			}
		
			if (ifmr->ifm_count != 0) 
			{
				kptr = (int *)kmalloc(ifmr->ifm_count * sizeof(int),
				                      GFP_KERNEL);
		
				if (kptr == NULL)
				{
					error = (-ENOMEM);
					break;
				}
				/*
				 * Get the media words from the interface's list.
				 */
				ep = LIST_FIRST(&ifm->ifm_list);
				for (; ep != NULL && count < ifmr->ifm_count;
				    ep = LIST_NEXT(ep, ifm_list), count++)
					kptr[count] = ep->ifm_media;
		
				if (ep != NULL)
					error = -E2BIG;	/* oops! */
			}
			else
				count = usermax;
		
			/*
			 * We do the copyout on E2BIG, because that's
			 * just our way of telling userland that there
			 * are more.  This is the behavior I've observed
			 * under BSD/OS 3.0
			 */
			sticky = error;
			if ((error == 0 || error == -E2BIG) && ifmr->ifm_count != 0) 
			{
				error = copy_to_user(ifmr->ifm_ulist,
				                      kptr, ifmr->ifm_count * sizeof(int));
			}
		
			if (error == 0)
			{
				error = sticky;
			}
			if (ifmr->ifm_count != 0)
			{
				kfree(kptr);
			}
			ifmr->ifm_count = count;
			memcpy(ifr, ifmr, sizeof(struct ifreq));
			break;
		}
		
		default:
			error = -EINVAL;
			break;
	}
	kfree(ifmr);
	return error;
}

/*
 * Find media entry matching a given ifm word.
 *
 */
struct ifmedia_entry *
ifmedia_match(struct ifmedia *ifm, int target, int mask)
{
	struct ifmedia_entry *match, *next;

	match = NULL;
	mask = ~mask;

	LIST_FOREACH(next, &ifm->ifm_list, ifm_list) 
	{
		if ((next->ifm_media & mask) == (target & mask))
		{
#if defined(IFMEDIA_DEBUG) || defined(DIAGNOSTIC)
			if (match)
			{
				printk("ifmedia_match: multiple match for "
				        "0x%x/0x%x\n", target, mask);
			}
#endif
			match = next;
		}
		//printk("In %s ifm_media = %0x, target = %0x\n", __func__, next->ifm_media, target);
	}

	return match;
}

#ifdef IFMEDIA_DEBUG
struct ifmedia_description ifm_type_descriptions[] =
	IFM_TYPE_DESCRIPTIONS;

struct ifmedia_description ifm_subtype_ethernet_descriptions[] =
	IFM_SUBTYPE_ETHERNET_DESCRIPTIONS;

struct ifmedia_description ifm_subtype_ethernet_option_descriptions[] =
	IFM_SUBTYPE_ETHERNET_OPTION_DESCRIPTIONS;

struct ifmedia_description ifm_subtype_tokenring_descriptions[] =
	IFM_SUBTYPE_TOKENRING_DESCRIPTIONS;

struct ifmedia_description ifm_subtype_tokenring_option_descriptions[] =
	IFM_SUBTYPE_TOKENRING_OPTION_DESCRIPTIONS;

struct ifmedia_description ifm_subtype_fddi_descriptions[] =
	IFM_SUBTYPE_FDDI_DESCRIPTIONS;

struct ifmedia_description ifm_subtype_fddi_option_descriptions[] =
	IFM_SUBTYPE_FDDI_OPTION_DESCRIPTIONS;

struct ifmedia_description ifm_subtype_ieee80211_descriptions[] =
	IFM_SUBTYPE_IEEE80211_DESCRIPTIONS;

struct ifmedia_description ifm_subtype_ieee80211_option_descriptions[] =
	IFM_SUBTYPE_IEEE80211_OPTION_DESCRIPTIONS;

struct ifmedia_description ifm_subtype_ieee80211_mode_descriptions[] =
	IFM_SUBTYPE_IEEE80211_MODE_DESCRIPTIONS;

struct ifmedia_description ifm_subtype_shared_descriptions[] =
	IFM_SUBTYPE_SHARED_DESCRIPTIONS;

struct ifmedia_description ifm_shared_option_descriptions[] =
	IFM_SHARED_OPTION_DESCRIPTIONS;

struct ifmedia_type_to_subtype {           /* XXX: right place for declaration? */
	struct ifmedia_description *subtypes;
	struct ifmedia_description *options;
	struct ifmedia_description *modes;
};

/* must be in the same order as IFM_TYPE_DESCRIPTIONS */
struct ifmedia_type_to_subtype ifmedia_types_to_subtypes[] = {
	{ &ifm_subtype_ethernet_descriptions[0],
	  &ifm_subtype_ethernet_option_descriptions[0],
	  NULL, },
	{ &ifm_subtype_tokenring_descriptions[0],
	  &ifm_subtype_tokenring_option_descriptions[0],
	  NULL, },
	{ &ifm_subtype_fddi_descriptions[0],
	  &ifm_subtype_fddi_option_descriptions[0],
	  NULL, },
	{ &ifm_subtype_ieee80211_descriptions[0],
	  &ifm_subtype_ieee80211_option_descriptions[0],
	  &ifm_subtype_ieee80211_mode_descriptions[0] },
};

/*
 * print a media word.
 */
static void
ifmedia_printword(int ifmw)
{
	struct ifmedia_description *desc;
	struct ifmedia_type_to_subtype *ttos;
	int seen_option = 0;

	/* Find the top-level interface type. */
	for (desc = ifm_type_descriptions, ttos = ifmedia_types_to_subtypes;
	    desc->ifmt_string != NULL; desc++, ttos++)
	{
		if (IFM_TYPE(ifmw) == desc->ifmt_word)
		{
			break;
		}
	}
	if (desc->ifmt_string == NULL) 
	{
		printk("<unknown type>\n");
		return;
	}
	printk(desc->ifmt_string);

	/* Any mode. */
	for (desc = ttos->modes; desc && desc->ifmt_string != NULL; desc++)
	{
		if (IFM_MODE(ifmw) == desc->ifmt_word) 
		{
			if (desc->ifmt_string != NULL)
			{
				printk(" mode %s", desc->ifmt_string);
			}
			break;
		}
	}
	/*
	 * Check for the shared subtype descriptions first, then the
	 * type-specific ones.
	 */
	for (desc = ifm_subtype_shared_descriptions;
	    desc->ifmt_string != NULL; desc++)
	{
		if (IFM_SUBTYPE(ifmw) == desc->ifmt_word)
		{
			goto got_subtype;
		}
	}
	for (desc = ttos->subtypes; desc->ifmt_string != NULL; desc++)
	{
		if (IFM_SUBTYPE(ifmw) == desc->ifmt_word)
		{
			break;
		}
	}
	if (desc->ifmt_string == NULL)
	{
		printk(" <unknown subtype>\n");
		return;
	}

got_subtype:
	printk(" %s", desc->ifmt_string);

	/*
	 * Look for shared options.
	 */
	for (desc = ifm_shared_option_descriptions;
	    desc->ifmt_string != NULL; desc++)
	{
		if (ifmw & desc->ifmt_word)
		{
			if (seen_option == 0)
			{
				printk(" <");
			}
			printk("%s%s", seen_option++ ? "," : "",
			    desc->ifmt_string);
		}
	}

	/*
	 * Look for subtype-specific options.
	 */
	for (desc = ttos->options; desc->ifmt_string != NULL; desc++) 
	{
		if (ifmw & desc->ifmt_word) 
		{
			if (seen_option == 0)
			{
				printk(" <");
			}
			printk("%s%s", seen_option++ ? "," : "",
			    desc->ifmt_string);
		}
	}
	printk("%s\n", seen_option ? ">" : "");
}
#endif /* IFMEDIA_DEBUG */

int send_data_pkt_to_drv(struct ifnet *ifp, netbuf_ctrl_block_m_t *m, struct ieee80211_node *ni)
{
	int status = 0;
	struct sk_buff *skb;
	skb = (struct sk_buff *)m->pkt_addr;

	skb->dev = ifp;
	skb->len = m->m_len;
	SKB_CB(skb)->ni = ni;
	skb->priority = m->m_pkthdr.ether_vtag;
	SKB_CB(skb)->flags |= m->m_flags;
	//printk(" %s %d skb->flags=%x\n", __func__, __LINE__, SKB_CB(skb)->flags);

	kfree(m);
	//status = dev_queue_xmit(skb);
#if KERNEL_VERSION_BTWN_2_6_(18, 27)
	ifp->hard_start_xmit(skb, ifp);
#else
	ifp->netdev_ops->ndo_start_xmit(skb, ifp);
#endif
	return status;
}

/* Will be used for vap->ifp->if_transmit */
int if_transmit_pkt(struct ifnet *ifp, struct mbuf *m)
{
	struct ifaltq *if_snd;
	if_snd = (struct ifaltq *)ifp->ml_priv;
	IF_ENQUEUE(if_snd,m);
	ieee80211_start(ifp);
	return 0;
}

int ieee80211_ifstop(struct ifnet *ifp)
{
	/* Added for handling close call from hostapd/ wpa supplicant*/ 
	struct ieee80211vap *vap;
	struct ieee80211com *ic;
	struct ifnet *parent;
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	vap = ifp->priv;
#else  
	vap = netdev_priv(ifp);
#endif
	ic = vap->iv_ic;
	parent = ic->ic_ifp;

	IEEE80211_DPRINTF(vap, IEEE80211_MSG_STATE | IEEE80211_MSG_DEBUG,
			"%s\n", __func__);
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR,
		    ("down interface %s\n", ifp->if_xname));

	if( !vap )
		return -1;
#ifdef ONEBOX_CONFIG_CFG80211
	ieee80211_notify_scan_done(vap, true);
#endif
	ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
	return 0;
}

EXPORT_SYMBOL(ifmedia_ioctl);
EXPORT_SYMBOL(null_function);
