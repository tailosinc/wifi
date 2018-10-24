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

#include <net80211/ieee80211_var.h>
#include <linux/version.h>
#include <linux/hardirq.h>

#define IEEE80211_TRANS_WAIT 2                   /* mgt frame tx timer (secs) */



/* Append len bytes at the end of the buffer netbuf from the address cp */
int m_append(struct netbuf_ctrl_block_m_s *netbuf_cb, int len, c_caddr_t cp)
{
	struct sk_buff *skb = (struct sk_buff *)netbuf_cb->pkt_addr;
#ifdef CONFIG_11W
	struct sk_buff *new_skb = NULL;
#endif
	uint8_t *data_ptr = NULL;
#ifdef CONFIG_11W
	int i=0, length = 0;
#else
	int i=0;
#endif

	/* checking for the buffer available at the end */
	if (skb_shinfo(skb)->nr_frags == 0)
	{
		if (len > (skb->end - skb->tail)) 
//		Temporary fix for handling additional 2 bytes
		{
#ifdef CONFIG_11W
			new_skb = dev_alloc_skb(skb->len + len + 156);
			skb_reserve(new_skb, 156);
			length = skb->len + len;
			memcpy(new_skb->data, skb->data, skb->len);
			data_ptr = skb_put(new_skb, (skb->len));
			memcpy(new_skb->cb, skb->cb, 48);
			dev_kfree_skb(skb);
			skb = new_skb;
			netbuf_cb->pkt_addr = (void *)skb;
			*(unsigned long *)skb->cb = (unsigned long)netbuf_cb;
#else
			return 0;
#endif
		}
		data_ptr = skb_put(skb, len);
	} 
	else
		return 0;

	for (i = 0; i < len; i++)
	{
		data_ptr[i] = cp[i];
	}
	netbuf_cb->m_len = skb->len;
#ifdef CONFIG_11W
	netbuf_cb->m_pkthdr.len = skb->len;
#endif
	return 1;

}

int get_headroom_of_m(netbuf_ctrl_block_m_t *m)
{
	struct sk_buff *skb = (struct sk_buff *)m->pkt_addr;

	skb->len = m->m_len;
	skb->data = m->m_data;
	return skb_headroom(skb);
}
/* Pushes ths skb->data pointer by len size*/
netbuf_ctrl_block_m_t*
netbuf_prepend(struct netbuf_ctrl_block_m_s *netbuf_cb, int len, int how)
{
	struct sk_buff *skb = (struct sk_buff *)netbuf_cb->pkt_addr;

	skb->len =netbuf_cb->m_len;
	if (skb_headroom(skb) < len)
	{
		return NULL;
	}

	if (skb_shinfo(skb)->nr_frags == 0)
	{
		netbuf_cb->m_data = skb_push(skb, len); //The pointer is pulled now to point to the new data
		 netbuf_cb->m_len  = skb->len;
#ifdef CONFIG_11W
		netbuf_cb->m_pkthdr.len = skb->len;
#endif
	}
	return netbuf_cb;
}

void
m_align(struct mbuf *m, int len) 
{
	int adjust;
	if (m->m_flags & M_PKTHDR)
	{
		adjust = MHLEN - len; 
	}
	else
	{ 
		adjust = MLEN - len; 
	}
	m->m_data += adjust &~ (sizeof(long)-1);
}

int
ieee80211_add_callback(struct mbuf *m,
                       void (*func)(struct ieee80211_node *, void *, int),
                       void *arg)
{
	m->m_flags |= M_TXCB;
#if  0
  struct ieee80211vap *vap = arg; 
	int timer = IEEE80211_TRANS_WAIT;
  mod_timer(&vap->iv_mgtsend, jiffies + msecs_to_jiffies(timer));
#endif
	return 0;
}

/* In free bsd this is to make sure that the buf in netbuf_cb has len number 
 * bytes of contiguous memory, if no it frees the m*/
netbuf_ctrl_block_m_t *netbuf_pullup(struct  netbuf_ctrl_block_m_s *netbuf_cb_m, int len)
{
	struct sk_buff *skb = (struct sk_buff *)netbuf_cb_m->pkt_addr;

	if (skb->len < len)
		goto out;
	else if (!skb_shinfo(skb)->nr_frags)
		return netbuf_cb_m;

out:
	netbuf_free(netbuf_cb_m);
	return NULL;
}

/* split the packet contained in one skb to two skbs 
 * len0 bytes will be there in the first mbuf and the rest of them in the second mbuf*/
struct netbuf_ctrl_block_m_s *netbuf_split(netbuf_ctrl_block_m_t *netbuf_cb, int len0, int wait)
{
	struct sk_buff *original_skb = (struct sk_buff *)netbuf_cb->pkt_addr;
	struct sk_buff *new_skb = NULL;
	netbuf_ctrl_block_m_t *new_netbuf_cb;

	if (wait)
	{
		new_netbuf_cb = kmalloc (sizeof(netbuf_ctrl_block_m_t), GFP_KERNEL);
	}
	else
	{
		new_netbuf_cb = kmalloc (sizeof(netbuf_ctrl_block_m_t), GFP_KERNEL | GFP_ATOMIC);
	}
	/* allocating new skb with length remaining length*/
	new_skb = dev_alloc_skb(original_skb->len - len0);
	/* see if we need to reserve any head room here */
	/* prepare the skb to copy the second part of the data packet */
	skb_put(new_skb, (original_skb->len - len0));
	/* copying the second part of the data to new skb */
	memcpy(new_skb->data, (&original_skb->data[len0]), (original_skb->len - len0));
	/* assign the new lengths */
	new_skb->len = original_skb->len - len0;
	original_skb->len -= len0;

	/* updating netbuf_cbs*/
	netbuf_cb->m_len = original_skb->len;
	new_netbuf_cb->m_len = new_skb->len;
	new_netbuf_cb->m_data = new_skb->data;
	new_netbuf_cb->pkt_addr = (uint32_t *)new_skb;

	return new_netbuf_cb;
}

void netbuf_cat(netbuf_ctrl_block_m_t *m, netbuf_ctrl_block_m_t *n)
{
	struct sk_buff *skb1 = (struct sk_buff *)m->pkt_addr;
	struct sk_buff *skb2 = (struct sk_buff *)n->pkt_addr;
	struct sk_buff *new_skb = NULL;
	if(skb_tailroom(skb1) >= skb2->len)
	{
		/* Available tailroom in the first buffer so copying the data */
		memcpy((skb1->data + skb1->len), skb2->data, skb2->len);
		skb1->len += skb2->len;
		skb_set_tail_pointer(skb1, skb1->len);
		m->m_len = skb1->len;
		m->m_data = skb1->data;
	}
	else
	{
		if((skb1->len + skb2->len) >= MAX_BUFFER_SIZE)
		{
			new_skb = dev_alloc_skb(skb1->len + skb2->len);
		}
		else
		{
			new_skb = dev_alloc_skb(MAX_BUFFER_SIZE);
			
		}
		if(new_skb == NULL)
		{
			printk("Failed to allocate memory in %s\n", __func__);
			return;
		}
		skb_reserve(new_skb, ONEBOX_NEEDED_HEADROOM);
		skb_put(new_skb, (skb1->len + skb2->len));
		memcpy(new_skb->data, skb1->data, skb1->len);
		memcpy((new_skb->data + skb1->len), skb2->data, skb2->len);
		m->m_data = new_skb->data;
		m->m_len = new_skb->len;
		m->pkt_addr = (uint32_t *)new_skb;
		m->m_pkthdr.len = new_skb->len;
		m->head = new_skb->head;
		m->m_ext.ext_buf  = (new_skb->head + NET_SKB_PAD);
		m->m_ext.ext_size = new_skb->len;
		dev_kfree_skb(skb1);
	}
	netbuf_free(n);
}

/* Pulls ths skb->data pointer by len size*/
void netbuf_adj(struct netbuf_ctrl_block_m_s *netbuf_cb_m, int len)
{
	struct sk_buff *skb = (struct sk_buff *)netbuf_cb_m->pkt_addr;
	
	skb->len = netbuf_cb_m->m_len;
	skb->data = netbuf_cb_m->m_data;
	if (len > 0)
	{
		netbuf_cb_m->m_data = skb_pull(skb, len); //The pointer is pulled now to point to the new data
	}
#if 0
	else
		skb_trim(skb, -len);
#endif
	netbuf_cb_m->m_len = skb->len;
	return;
}

/* This is for copying len number of bytes from skb m starting from off bytes of offset to cp */
/* review this one more time */
void
m_copydata(const struct mbuf *m, int off, int len, caddr_t cp)
{
	u_int count;

	KASSERT(off >= 0, ("m_copydata, negative off %d", off));
	KASSERT(len >= 0, ("m_copydata, negative len %d", len));
	while (off > 0)
	{
		KASSERT(m != NULL, ("m_copydata, offset > size of mbuf chain"));
		if (off < m->m_len)
		{
			break;
		}
		off -= m->m_len;
		m = m->m_next;
	}
	while (len > 0)
	{
		KASSERT(m != NULL, ("m_copydata, length > size of mbuf chain"));
		count = min(m->m_len - off, len);
		bcopy(mtod(m, caddr_t) + off, cp, count);
		len -= count;
		cp += count;
		off = 0;
		 //m = m->m_next;
	}
}

/* This is to create a clone buffer which can be writable in case of security to write cipher */
/* Nothing needs to be done in linux as the buffer is already writable */
struct netbuf_ctrl_block_m_s * netbuf_unshare(struct netbuf_ctrl_block_m_s *m0, int how)
{
	return m0;
}

/* This function copies skb to a new skb */
//Take care of the ref count also
netbuf_ctrl_block_m_t *netbuf_copy (netbuf_ctrl_block_m_t *netbuf_cb, int malloc_flags)
{
	netbuf_ctrl_block_m_t *new_netbuf_cb;
	struct sk_buff *skb = (struct sk_buff *)netbuf_cb->pkt_addr;
	struct sk_buff *new_skb;

	new_skb= dev_alloc_skb(skb->len + ONEBOX_NEEDED_HEADROOM);
	if(new_skb == NULL)
	{
		return NULL;
	}
	skb_reserve(new_skb,ONEBOX_NEEDED_HEADROOM - 2); //Extra 2 bytes is to align the finally packet to dword as it is giving problem with
	                                                // Few embedded platforms which use DMA for SDIO transfers if not aligned to dword
	                                                // The extra 2bytes here will be compensated by the 6 bytes done while removing ethernet
	                                                // header and inserting LLC header
	skb_put(new_skb, skb->len);
	memcpy(new_skb->data, skb->data, skb->len);
	memcpy(new_skb->cb, skb->cb, 48); //Define this function once after netbuf_ctrl_block_m_t structure is freezed

	new_netbuf_cb = onebox_translate_skb_to_netcb_m(new_skb);
	new_netbuf_cb->m_flags = netbuf_cb->m_flags;
	/* copying the cb structure */
	return new_netbuf_cb;
}

netbuf_ctrl_block_m_t *netbuf_gethdr (int wait,   int type)
{
	return NULL;
/*
	struct sk_buff *skb = (struct sk_buff *)m->pkt_addr;
	if (!skb_shinfo(skb)->nr_frags)
	{
	  return NULL;
	} 
*/
}

/* This function maps the skb to netbuf control block structure */
netbuf_ctrl_block_m_t * onebox_translate_skb_to_netcb_m(struct sk_buff *skb)
{
	netbuf_ctrl_block_m_t *netbuf_cb_m = NULL;
	netbuf_cb_m = kzalloc(sizeof(netbuf_ctrl_block_m_t), GFP_ATOMIC);
	
	netbuf_cb_m->m_data = skb->data;
	netbuf_cb_m->m_len = skb->len;
	netbuf_cb_m->pkt_addr = (uint32_t *)skb;
	netbuf_cb_m->m_flags = M_EXT;
	netbuf_cb_m->flags = SKB_CB(skb)->flags; 
	netbuf_cb_m->m_pkthdr.len = skb->len;
	//  netbuf_cb->next = skb->next; 
	//  netbuf_cb_m->head = skb->head;
	netbuf_cb_m->m_ext.ext_buf  = (skb->head + NET_SKB_PAD);
	netbuf_cb_m->m_ext.ext_size = skb->len;
	return netbuf_cb_m;
}

netbuf_ctrl_block_m_t * onebox_translate_netbuf_to_mbuf(netbuf_ctrl_block_t *netbuf_cb)
{
	netbuf_ctrl_block_m_t *new_netbuf_cb_m = NULL;
	if (in_atomic())
	{
		new_netbuf_cb_m = kmalloc(sizeof(netbuf_ctrl_block_m_t), GFP_ATOMIC);
	}
	else
	{
		new_netbuf_cb_m = kmalloc(sizeof(netbuf_ctrl_block_m_t), GFP_KERNEL);
	}
	if(new_netbuf_cb_m == NULL)
	{
		printk("ERROR while allocating memory in %s\n", __func__);
		dev_kfree_skb(netbuf_cb->pkt_addr);
		kfree(netbuf_cb);
		return NULL;
	}
	new_netbuf_cb_m->m_data    = netbuf_cb->data;
	new_netbuf_cb_m->pkt_addr  = netbuf_cb->pkt_addr;
	new_netbuf_cb_m->m_len     = netbuf_cb->len;
	new_netbuf_cb_m->m_pkthdr.len     = netbuf_cb->len;
	new_netbuf_cb_m->m_flags   = M_EXT;
	new_netbuf_cb_m->m_type    = MT_DATA;
	new_netbuf_cb_m->m_nextpkt = NULL;
	new_netbuf_cb_m->m_next    = NULL;
	kfree(netbuf_cb);
	return new_netbuf_cb_m;
}

netbuf_ctrl_block_t *onebox_translate_mbuf_to_netbuf(netbuf_ctrl_block_m_t *netbuf_cb_m)
{
	netbuf_ctrl_block_t *netbuf_cb;
	struct sk_buff * skb = (struct sk_buff *)netbuf_cb_m->pkt_addr;

	netbuf_cb = kmalloc(sizeof(netbuf_ctrl_block_t), GFP_ATOMIC);
	if (netbuf_cb == NULL)
		return NULL;
	netbuf_cb->len   = netbuf_cb_m->m_len;
	netbuf_cb->data   = netbuf_cb_m->m_data;
	netbuf_cb->pkt_addr   = netbuf_cb_m->pkt_addr;
	netbuf_cb->flags = netbuf_cb_m->flags;
	*((unsigned long int*)skb->cb) = (unsigned long)netbuf_cb;

	return netbuf_cb;
}

void netbuf_free(netbuf_ctrl_block_m_t *netbuf_cb_m)
{
	dev_kfree_skb((struct sk_buff *)netbuf_cb_m->pkt_addr);
	kfree(netbuf_cb_m);
}

struct mbuf* m_getcl(int how,short type,int flags)
{
	return NULL;
}

struct sk_buff *onebox_translate_mbuf_to_skb(struct mbuf *mbuf)
{
	struct sk_buff *skb ;
	skb = (struct sk_buff *)mbuf->pkt_addr;
	skb->len = mbuf->m_len;
	skb->data = mbuf->m_data;
	skb->ip_summed = CHECKSUM_NONE;
	return skb;
}

EXPORT_SYMBOL(m_copydata);
EXPORT_SYMBOL(netbuf_prepend);
EXPORT_SYMBOL(netbuf_append);
EXPORT_SYMBOL(netbuf_adj);
#ifdef CONFIG_11W
EXPORT_SYMBOL(netbuf_free);
#endif
EXPORT_SYMBOL(onebox_translate_netbuf_to_mbuf);
EXPORT_SYMBOL(onebox_translate_mbuf_to_netbuf);
