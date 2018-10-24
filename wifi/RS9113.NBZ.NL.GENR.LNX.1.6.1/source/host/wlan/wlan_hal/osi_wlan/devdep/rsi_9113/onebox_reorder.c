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

#include "wlan_common.h"

#ifdef BYPASS_RX_DATA_PATH

static uint64_t
read_6(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5) 
{
        uint32_t iv32 = (b0 << 0) | (b1 << 8) | (b2 << 16) | (b3 << 24);
        uint16_t iv16 = (b4 << 0) | (b5 << 8); 
        return (((uint64_t)iv16) << 32) | iv32;
}

void onebox_reorder_pkt(WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb)
{
	uint8 tid = 0, vap_id = 0; 
	uint8 *msg = netbuf_cb->data;
	uint8 pad_bytes = msg[4];
	uint16 seqno;
	uint8 status;
	int32 msg_len;
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211vap *vap = NULL;
	struct ieee80211_node *ni = NULL;
	netbuf_ctrl_block_m_t *netbuf_cb_m = NULL;
	uint8 qos_pkt = 0, pn_valid = 0;
	uint64_t pn;

	msg_len  = (ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[0])& 0x0fff);
	msg_len -= pad_bytes;

	vap_id = ((msg[14] & 0xf0) >> 4);	    
	qos_pkt = (msg[7] & BIT(0));
	pn_valid = (msg[7] & BIT(1));
	tid = (msg[14] & 0x0f);	

	w_adapter->os_intf_ops->onebox_acquire_sem(&w_adapter->ic_lock_vap, 0);
	w_adapter->total_data_rx_done_intr++;

	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
	{
		if(vap->hal_priv_vap->vap_id == vap_id)
		{
			break;
		}
	}

	if(!vap)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Vap Id %d \n"), vap_id));
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Pkt recvd is \n")));
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, msg, 32);
		w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
		w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);
		return;

	}
	//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("NETBUF DUMP IN REORDER FUNCTION\n")));
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, netbuf_cb->data, netbuf_cb->len);

	w_adapter->os_intf_ops->onebox_netbuf_adj(netbuf_cb, (FRAME_DESC_SZ + pad_bytes));
	//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("NETBUF DUMP AFTER ADJUST\n")));
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, netbuf_cb->data, netbuf_cb->len);

	netbuf_cb_m = onebox_translate_netbuf_to_mbuf(netbuf_cb);

	if (netbuf_cb_m == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to alloc mem %s %d\n"), __func__, __LINE__));
		w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);
		return;
	}

	/* 
	 * We need to fill the packet details which we recieved 
	 * from the descriptor in Device decap mode
	 */
	if(netbuf_cb_m->m_len < 14){
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("@@Alert: Data Packet less than Ethernet HDR size received\n")));	
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)netbuf_cb_m->m_data, netbuf_cb_m->m_len);
	}

	if(vap->iv_opmode == IEEE80211_M_HOSTAP)
	{
		ni = w_adapter->net80211_ops->onebox_find_node(&vap->iv_ic->ic_sta, &netbuf_cb_m->m_data[6] );
	}
	else
	{
		ni = w_adapter->net80211_ops->onebox_find_node(&vap->iv_ic->ic_sta, (uint8 *)&vap->iv_bss->ni_macaddr );
	}

	if(!ni) {
		/* what should I do AS this should not happen for data pkts?????*/
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Giving pkt directly to hostap In %s Line %d \n"), __func__, __LINE__));
		vap->iv_deliver_data(vap, vap->iv_bss, netbuf_cb_m);
		w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);
		return;
	}

	/*
	 * checking pn for both unicast and multicast packet. 
	 * mcast packet will be dropped in ieee80211_deliver_data func
	 * if pn number doesn't match.
	 */
	if (pn_valid && (vap->iv_opmode == IEEE80211_M_STA)) {
		pn = read_6(msg[8],  msg[9],  msg[10], 
			    msg[11], msg[12], msg[13]);
		memcpy(&netbuf_cb_m->pn, &pn, 8);
		netbuf_cb_m->pn_valid = pn_valid;
	}

	/* PULL this into some structure and map accordingly*/
	ni->ni_avgrssi =  ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[16]);
	ni->ni_rxrate = ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[18]);
	vap->iv_bss->ni_txpower = ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[36]);
	if(vap->iv_opmode == IEEE80211_M_STA)
	{
        ni->ni_txrate = ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[34]);
    }
	else if (vap->iv_opmode == IEEE80211_M_HOSTAP)
    {
        vap->cur_rate = ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[34]);
	}
	if((*(uint8_t *)&netbuf_cb_m->m_data[12] == 0x88) && (*(uint8_t *)&netbuf_cb_m->m_data[13] == 0x8e)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<==== Recvd EAPOL from %02x:%02x:%02x:%02x:%2x:%02x ====>\n"), ni->ni_macaddr[0], ni->ni_macaddr[1],
						ni->ni_macaddr[2], ni->ni_macaddr[3], ni->ni_macaddr[4], ni->ni_macaddr[5]));
		if(netbuf_cb_m->m_len == 113)
		{
			ni->ni_jointime = jiffies_to_msecs((jiffies - vap->iv_jointime));		
		}
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)netbuf_cb_m->m_data, netbuf_cb_m->m_len);		
	}
	/* Indicates the Packet has been decapsulted*/
	netbuf_cb_m->m_flags |= M_DECAP;

#ifdef PWR_SAVE_SUPPORT
	if((vap->iv_opmode == IEEE80211_M_STA)
			&& (TRAFFIC_PS_EN)
			&& (ps_params_def.ps_en)
			&& !(netbuf_cb_m->m_data[0] & 0x01)
			)
	{
		vap->check_traffic(vap, 0, netbuf_cb_m->m_len);
	}
#endif

	if(!qos_pkt)
	{
		//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Recvd non qos pkt seq no %d tid %d \n", (((*(uint16 *)&msg[5]) >> 4) & 0xfff), (msg[14] & 0x0f))));
		vap->iv_deliver_data(vap, vap->iv_bss, netbuf_cb_m);
		w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);
		return;
	}

	if(tid > 8)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Pkt with unkown tid is \n")));
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, msg, 32);
		vap->iv_deliver_data(vap, vap->iv_bss, netbuf_cb_m);
		w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);
		return;
	}

	seqno = ((ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[5]) >> 4) & 0xfff); 

	netbuf_cb_m->tid = tid;
	netbuf_cb_m->aggr_len = seqno;

	if(ni->ni_flags & IEEE80211_NODE_HT)
	{
		if (tid < WME_NUM_TID)
		{
			if (ni->ni_rx_ampdu[tid].rxa_flags & IEEE80211_AGGR_RUNNING)
			{
				netbuf_cb_m->m_flags |= M_AMPDU;
			}
		}
	}
	status = vap->iv_input(ni, netbuf_cb_m, 0, 0);	
	w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);
	return;

}

#endif
