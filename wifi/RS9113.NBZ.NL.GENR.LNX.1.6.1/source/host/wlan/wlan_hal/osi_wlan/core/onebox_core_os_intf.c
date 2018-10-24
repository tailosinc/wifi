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

#include "onebox_wlan_core.h"
#include "mbuf.h"


static void
onebox_completion_handler(WLAN_ADAPTER w_adapter,
                          netbuf_ctrl_block_t   *netbuf_cb )
{
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211vap *vap = NULL;

	netbuf_ctrl_block_t *netbuf_cb_dup;
	netbuf_cb_dup = w_adapter->os_intf_ops->onebox_alloc_skb(netbuf_cb->len);
	w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb_dup, netbuf_cb->len);
	w_adapter->os_intf_ops->onebox_memcpy((uint8 *)netbuf_cb_dup->data, (uint8 *)netbuf_cb->data, netbuf_cb->len);
	w_adapter->os_intf_ops->onebox_netbuf_adj(netbuf_cb_dup, FRAME_DESC_SZ + netbuf_cb_dup->data[OFFSET_EXT_DESC_SIZE]);

	if (netbuf_cb) {
		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) { 
			if (vap->iv_opmode == IEEE80211_M_MONITOR)
				break;
		}
		w_adapter->core_ops->onebox_core_radiotap_tx(w_adapter, vap, netbuf_cb_dup);
	}
	return;
}


/**
 * This function preapres the frame_desc.
 *
 * @param       Pointer to private driver structure.
 * @param       Pointer to netbuf control block structure.
 * @return      ONEBOX_STATUS_SUCCESS on success else ONEBOX_STATUS_FAILURE.
 */
ONEBOX_STATUS prepare_onair_data_pkt_desc(WLAN_ADAPTER w_adapter, 
                                  netbuf_ctrl_block_t *netbuf_cb,
                                  int8 q_num)
{
		uint16 *frame_desc;
#ifndef BYPASS_TX_DATA_PATH
		uint16 seq_num = 0 ;
		struct ieee80211_frame *tmp_hdr = NULL;
		uint8 hdr_size = MIN_802_11_HDR_LEN;
		uint8 ieee80211_size = hdr_size;
		const struct ieee80211_cipher *cip = NULL;
#endif
		uint8 addqos;
		struct ieee80211_node *ni = NULL;
		struct ieee80211vap *vap = NULL;
		struct ieee80211com *ic = &w_adapter->vap_com;
		uint8 security_enabled = 0;
		uint8 extnd_size = 0;
		xtended_desc_t *xtend_desc_pkt = NULL;
	struct chanAccParams *wme_params_sta;
	
		wme_params_sta = &ic->ic_wme.wme_wmeChanParams;

		//: Change label from fail to data fail
		FUNCTION_ENTRY(ONEBOX_ZONE_DATA_SEND);
		ni = (struct ieee80211_node *)netbuf_cb->ni;/* NB: always passed down by 802.11 layer */
		if (ni == NULL)
		{
				/* NB: this happens if someone marks the underlying device up */
				ONEBOX_DEBUG(ONEBOX_ZONE_DATA_SEND,
								(TEXT( "Dropping; No node in skb control block!\n")));
				goto fail;
		}
		vap = ni->ni_vap;

		if(vap == NULL) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERR: Freeing the buffer as VAP has been freed\n")));
				goto fail;
		}


#ifndef BYPASS_TX_DATA_PATH
		tmp_hdr = (struct ieee80211_frame *)&netbuf_cb->data[0];
		seq_num = ONEBOX_CPU_TO_LE16( *( (uint16 *)&(tmp_hdr->i_seq[0])));
		seq_num = seq_num >> 4;
#endif

		/* Get HdrSize Dynamically */
		/* If addr is unaligned, aligning
		 * it to dword and keeping extra bytes
		 * in extended descriptor
		 */
		w_adapter->os_intf_ops->onebox_change_hdr_size(netbuf_cb, (FRAME_DESC_SZ + sizeof(xtended_desc_t)));
		extnd_size = ((unsigned long )netbuf_cb->data & 0x3f);/* To make it 64 bit aligned buffer, to avoid mmc timeout interrupt */
		w_adapter->os_intf_ops->onebox_change_hdr_size(netbuf_cb, extnd_size);
		w_adapter->os_intf_ops->onebox_memset(netbuf_cb->data,0,(FRAME_DESC_SZ + extnd_size + sizeof(xtended_desc_t)));
		xtend_desc_pkt = (xtended_desc_t *)((uint8 *)netbuf_cb->data + FRAME_DESC_SZ);
		frame_desc = (uint16 *)&netbuf_cb->data[0];

		frame_desc[2] = ONEBOX_CPU_TO_LE16(extnd_size + sizeof(xtended_desc_t));//Extended desc size 

#ifndef BYPASS_TX_DATA_PATH
		if (tmp_hdr->i_fc[0] & 0x80) //QOS Support
		{
				hdr_size += 2;
				ieee80211_size = hdr_size;
		}

		if (tmp_hdr->i_fc[1] & IEEE80211_FC1_WEP)
		{
				struct ieee80211_key *ikey = NULL;
				if ((netbuf_cb->flags & ONEBOX_BROADCAST) ||
								(netbuf_cb->flags & ONEBOX_MULTICAST) || 
								(IEEE80211_KEY_UNDEFINED(&ni->ni_ucastkey))) 
				{
						ikey = &vap->iv_nw_keys[vap->iv_def_txkey];
				}
				else
				{
						ikey = &ni->ni_ucastkey;
				}
				cip = ikey->wk_cipher;
				if(cip == NULL)
				{
						goto fail;
				}
				if (cip->ic_miclen)
				{
						if (hdr_size == MIN_802_11_HDR_LEN)
			{
				hdr_size += cip->ic_header + 20;  
			}
			else
			{
				hdr_size += cip->ic_header + 18;  
			}
		}
		else
		{
			hdr_size += cip->ic_header;  
		}
		ieee80211_size += cip->ic_header;
	}
#endif
	/*  Need to set encrypt bit if keys are loaded.  Need to check through Vap->iv_flags 
	 *whether security is enabled
	 */
	if(vap->iv_flags & IEEE80211_F_PRIVACY)
	{
		security_enabled = 1;
		/* Bit 15 indicates encryption has to be taken care by firmware */
		/* Firmware needs this flags as we are queuing bcast pkts in beacon queue and to encrypt bcast pkts only */
    		if((ni->ni_flags & IEEE80211_NODE_ENCRYPT_ENBL) || (netbuf_cb->flags & ONEBOX_BROADCAST) || (netbuf_cb->flags & ONEBOX_MULTICAST))
		{
			frame_desc[6] |= ONEBOX_CPU_TO_LE16(ONEBOX_BIT(15));
		}
	}


	/* Fill the descriptor */
	frame_desc[0] = ONEBOX_CPU_TO_LE16((netbuf_cb->len - (FRAME_DESC_SZ)) | (ONEBOX_WIFI_DATA_Q << 12));
	netbuf_cb->tx_pkt_type = WLAN_TX_D_Q;
	
	addqos = (ni->ni_flags & (IEEE80211_NODE_QOS|IEEE80211_NODE_HT));
	
	if(addqos) {
		frame_desc[6] |= ONEBOX_CPU_TO_LE16(ONEBOX_BIT(12));
	}
#ifdef BYPASS_TX_DATA_PATH
 /* Doing encap in firmware indicating in fd[1]*/
	frame_desc[1] = ONEBOX_CPU_TO_LE16(1);
#else
	frame_desc[2] |= ONEBOX_CPU_TO_LE16((ieee80211_size) << 8);
#endif

	if(vap->hal_priv_vap->fixed_rate_enable)
	{
		frame_desc[3]  = ONEBOX_CPU_TO_LE16(RATE_INFO_ENABLE);
		/* Fill data rate in case of fixed rate */
		frame_desc[4] = ONEBOX_CPU_TO_LE16(vap->hal_priv_vap->rate_hix);

		//: Uncomment the below check when we support shortgi 40
		if((vap->iv_flags_ht & IEEE80211_FHT_SHORTGI20) || (vap->iv_flags_ht & IEEE80211_FHT_SHORTGI40))
		{
			/*checking whether the connected node supports SHORT GI*/
			if ((ni->ni_htcap & IEEE80211_HTCAP_SHORTGI40) || (ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20))
			{
				if(vap->hal_priv_vap->rate_hix & 0x100) /*check this for MCS rates only */
				{
					frame_desc[4] |= ONEBOX_CPU_TO_LE16(ENABLE_SHORTGI_RATE);;/* Indicates shortGi HT Rate */
				}			
			}			
		}

		if(ic->band_flags & IEEE80211_CHAN_HT40) {
			if (ni->ni_htcap & IEEE80211_HTCAP_CHWIDTH40) 
			{
				frame_desc[5] = ONEBOX_CPU_TO_LE16(FULL_40M_ENABLE);
			} else if(ic->band_flags & IEEE80211_CHAN_HT40U) {
				/* Primary Channel is Below secondary Channel. So send data in
				 * lower 20 band 
				 */
				frame_desc[5] = ONEBOX_CPU_TO_LE16(LOWER_20_ENABLE); //full 40
				frame_desc[5] |= ONEBOX_CPU_TO_LE16(LOWER_20_ENABLE >> 12); //full 40

			} else {
				/* Primary Channel is above secondary Channel. So send data in
				 * uppper 20 band 
				 */
				frame_desc[5] = ONEBOX_CPU_TO_LE16(UPPER_20_ENABLE); //full 40
				frame_desc[5] |= ONEBOX_CPU_TO_LE16(UPPER_20_ENABLE >> 12); //full 40
			}
		}
	}
#ifdef ACM_NO_TSPEC_CNFM
	if(vap->iv_opmode == IEEE80211_M_STA) {
		if(wme_params_sta->cap_wmeParams[w_adapter->selected_qnum].wmep_acm) {
			netbuf_cb->tid = 1;
			q_num = BK_Q_STA;
		}
	}
#endif

	frame_desc[7] = ONEBOX_CPU_TO_LE16(((netbuf_cb->tid & 0xf) << 4) | (q_num & 0xf) | (netbuf_cb->sta_id << 8));
	if((vap->iv_opmode == IEEE80211_M_HOSTAP) && (netbuf_cb->flags & MORE_DATA) && (q_num != BEACON_HW_Q) && (q_num != BROADCAST_HW_Q))
	{
		frame_desc[3] |= (MORE_DATA_PRESENT);
	}
	if((vap->iv_opmode == IEEE80211_M_HOSTAP) && (netbuf_cb->flags & M_EOSP) && (q_num != BEACON_HW_Q) && (q_num != BROADCAST_HW_Q))
	{
		//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("indicating EOSP %s Line %d sta_id %d\n", __func__, __LINE__, netbuf_cb->sta_id)));
		frame_desc[1] |= ONEBOX_CPU_TO_LE16(REQUIRE_TSF_SYNC_CONFIRM);  /* Expecting Confirm for this packet */
		frame_desc[1] |= ONEBOX_CPU_TO_LE16(EOSP_INDICATION);  
	}
	if((q_num == BROADCAST_HW_Q) || (q_num == BEACON_HW_Q))
	{
		frame_desc[3] |= (INSERT_SEQ_NO);
	}
#if defined(RSI_CCX) || defined(CONFIG_11R)
	if(w_adapter->IAPP_PKT)
	{ 
		/*Indication to firmware for IAPP Packet ie setting 6th bit in mac_info filed */
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Setting IAPP Pkt info to the firmware\n")));
		frame_desc[3] |= ONEBOX_CPU_TO_LE16(1 << 6); 
	}
#endif
        
	if ((netbuf_cb->flags & ONEBOX_BROADCAST) || (netbuf_cb->flags & ONEBOX_MULTICAST))
	{
		frame_desc[3] |= ONEBOX_CPU_TO_LE16(ONEBOX_BROADCAST_PKT | RATE_INFO_ENABLE);

		/* : DO we need to handle host encap/decap separately */
		if(netbuf_cb->flags & FIRST_BCAST)
		{
			/*first broad cast packet set beacon gated frame */
			//frame_desc[3] |= (DTIM_BEACON_GATED_FRAME );
		}

		if(1) /* In case of AP mode use the following rates for txn of broadcast pkts */
		{
			if (!vap->hal_priv_vap->mgmt_rate_enable) {
				if((w_adapter->operating_band != BAND_2_4GHZ)
#ifdef ENABLE_P2P_SUPPORT
						|| (vap->p2p_enable)
#endif
					)
				{	
					/*Fill default rate for broad cast pkts in case of fixed rate */
					frame_desc[4] = ONEBOX_CPU_TO_LE16(0xB | ONEBOX_11G_MODE); // 6mbps 11a mode
				}
				else
				{
					/*Fill default rate for broad cast pkts in case of fixed rate */
					frame_desc[4] = ONEBOX_CPU_TO_LE16(0 | ONEBOX_11B_MODE); //1mbps  11b mode
				}
			} else {
					frame_desc[4] = ONEBOX_CPU_TO_LE16(vap->hal_priv_vap->default_mgmt_rate); 
			}

		}
		frame_desc[7] = ONEBOX_CPU_TO_LE16(((netbuf_cb->tid & 0xf) << 4) | (q_num & 0xf) | (vap->hal_priv_vap->vap_id << 8));
	}
	
	frame_desc[4] |= ONEBOX_CPU_TO_LE16((vap->hal_priv_vap->vap_id << 14));
	
	/*Fill bbp info */
	/*frame_desc[5] = 0;*/

#ifndef BYPASS_TX_DATA_PATH
	/*Fill sequence number */
	frame_desc[6] |= ONEBOX_CPU_TO_LE16((seq_num) & 0xfff);
#endif
//	frame_desc[6] |= ONEBOX_CPU_TO_LE16(key_idx << 13);


#ifdef BYPASS_TX_DATA_PATH
	if(netbuf_cb->data[16+12+extnd_size] == 0x88 && netbuf_cb->data[16+12+extnd_size+1] == 0x8e)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<==== Sending EAPOL to %02x:%02x:%02x:%02x:%02x:%02x ====>\n"),
				netbuf_cb->data[16 + extnd_size], netbuf_cb->data[17 + extnd_size], netbuf_cb->data[18 + extnd_size], 
				netbuf_cb->data[19 + extnd_size], netbuf_cb->data[20 + extnd_size], netbuf_cb->data[21 + extnd_size]));
		frame_desc[6] |= ONEBOX_CPU_TO_LE16(BIT(13));//This bit indicates firmware to add this pkt to sw_q head
		frame_desc[1] |= ONEBOX_CPU_TO_LE16(FETCH_RETRY_CNT_FRM_HST);  
		xtend_desc_pkt->retry_cnt = EAPOL_RETRY_CNT;
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (uint8 *)netbuf_cb->data, netbuf_cb->len);
	}
#endif

	/* Assign desc_ptr, pkt_len values */
	if(netbuf_cb->len <= 1594)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Data pkt QUEUED \n")));
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DATA_SEND, (uint8 *)netbuf_cb->data, netbuf_cb->len);

		/* Perform some processing here */
		if (w_adapter->sc_nmonvaps) 
		{
			//w_adapter->os_intf_ops->onebox_netbuf_adj(netbuf_cb, FRAME_DESC_SZ + netbuf_cb->data[OFFSET_EXT_DESC_SIZE]);
#ifndef BYPASS_TX_DATA_PATH
			if (security_enabled)
			{
				if((cip->ic_cipher == IEEE80211_CIPHER_AES_CCM) || (cip->ic_cipher == IEEE80211_CIPHER_WEP))
				{
					w_adapter->os_intf_ops->onebox_memmove(&netbuf_cb->data[MIN_802_11_HDR_LEN + 2 /*qos*/ - cip->ic_header], 
					                                     &netbuf_cb->data[0],
					                                     (ieee80211_size)); 
					w_adapter->os_intf_ops->onebox_netbuf_adj(netbuf_cb, (MIN_802_11_HDR_LEN + 2 - cip->ic_header));
				}
				else if( cip->ic_cipher == IEEE80211_CIPHER_TKIP)
				{
					w_adapter->os_intf_ops->onebox_memmove(&netbuf_cb->data[MIN_802_11_HDR_LEN + 2 /*qos*/ - 
					                                     (cip->ic_header + cip->ic_miclen + 6/* IV*/ + 4 /*ICV */)], 
					                                     &netbuf_cb->data[0],
					                                     (ieee80211_size)); 
					w_adapter->os_intf_ops->onebox_netbuf_adj(netbuf_cb, 
					                                        (MIN_802_11_HDR_LEN + 2 /*qos*/ - 
					                                        (cip->ic_header + cip->ic_miclen + 6/* IV*/ + 4 /*ICV */))); 
				}

			}
			else
#endif
			onebox_completion_handler(w_adapter, netbuf_cb);
		}
	}
	else 
	{ 
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT 
		             ("Bigger Packet > 1594 Bytes, Not Transmitting pkt_len = %d\n"), netbuf_cb->len));
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (uint8 *)netbuf_cb->data, netbuf_cb->len);
	}
#ifdef THROUGHPUT_DEBUG
	if(( !w_adapter->prev_jiffies) || (jiffies_to_msecs(jiffies - w_adapter->prev_jiffies) > 1000))
	{
			w_adapter->prev_jiffies = jiffies;
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In cur sec VO sent = %d VI=%d BE=%d BK=%d\n", (w_adapter->total_data_vo_pkt_send - w_adapter->prev_sec_data_vo_pkt_send),
												(w_adapter->total_data_vi_pkt_send - w_adapter->prev_sec_data_vi_pkt_send),
												(w_adapter->total_data_be_pkt_send - w_adapter->prev_sec_data_be_pkt_send),
												(w_adapter->total_data_bk_pkt_send - w_adapter->prev_sec_data_bk_pkt_send))));
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Dropped pkts =%d\n",w_adapter->stats.tx_dropped -w_adapter->prev_sec_dropped)));

			w_adapter->prev_sec_data_vo_pkt_send = w_adapter->total_data_vo_pkt_send;
			w_adapter->prev_sec_data_vi_pkt_send = w_adapter->total_data_vi_pkt_send;
			w_adapter->prev_sec_data_be_pkt_send = w_adapter->total_data_be_pkt_send;
			w_adapter->prev_sec_data_bk_pkt_send = w_adapter->total_data_bk_pkt_send;
			w_adapter->prev_sec_dropped = w_adapter->stats.tx_dropped;	
		
	}
#endif

	
	return ONEBOX_STATUS_SUCCESS;
fail:
	w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
	return ONEBOX_STATUS_FAILURE;
}

ONEBOX_STATUS prepare_onair_mgmt_pkt_desc(WLAN_ADAPTER w_adapter, 
                                  netbuf_ctrl_block_t *netbuf_cb)
{
	uint16 *msg = NULL;
	int msg_len = 0;
	ONEBOX_STATUS status;
	struct ieee80211_node *ni;
	struct ieee80211vap *vap = NULL;
	struct ieee80211_frame *wh = NULL;
	struct ieee80211com *ic = &w_adapter->vap_com;
	uint8 extnd_size = 0;
	uint8 ii, jj = 0;
#ifdef BYPASS_TX_DATA_PATH
	uint8 addqos = 0;
#endif
	xtended_desc_t *xtend_desc_pkt = NULL;

	
	if(netbuf_cb->flags & M_EAPOL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND, (TEXT("EAPOL len %d \n"), netbuf_cb->len));
	}
	ni = (struct ieee80211_node *)netbuf_cb->ni;
	if(ni == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERR: Freeing the skb as node(ni) has been freed\n")));
		goto err;
	}
	vap = ni->ni_vap;
	
	if(vap == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERR: Freeing the buffer as VAP has been freed\n")));
		goto err;
	}

	//ic = vap->iv_ic;
	/* hdr = (struct onebox_80211_hdr_4addr *)netbuf_cb->data; */
	wh = (struct ieee80211_frame *)&netbuf_cb->data[0];
	//:header size should be pulled with extended desc size
	if( (netbuf_cb->data[12] == 0x88) && (netbuf_cb->data[13] == 0x8e) && (netbuf_cb->flags & M_EAPOL) )
	{
		addqos = ((ni->ni_flags & (IEEE80211_NODE_QOS|IEEE80211_NODE_HT)));
		//if(((vap->iv_state == IEEE80211_S_RUN) && !(vap->hal_priv_vap->conn_in_prog)) || (vap->)) {
		if((((vap->iv_state == IEEE80211_S_RUN) 
						&& !(vap->hal_priv_vap->conn_in_prog) 
						&& (vap->iv_opmode == IEEE80211_M_STA))) 
				|| ((vap->iv_opmode == IEEE80211_M_HOSTAP) 
					&& (ni->ni_flags & IEEE80211_NODE_AUTH))) {
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Change the Host_q to DATA_Q as it is rekeying\n")));
			netbuf_cb->tid = 7;
			if(vap->iv_opmode == IEEE80211_M_STA)
				netbuf_cb->priority = VO_Q_STA;
			else
				netbuf_cb->priority = VO_Q_AP;
			
			status = prepare_onair_data_pkt_desc(w_adapter, netbuf_cb, netbuf_cb->priority);
			if(status != ONEBOX_STATUS_SUCCESS) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERR: Sending EAPOL is failed\n")));
			}
			return status;
		}
	}
	w_adapter->os_intf_ops->onebox_change_hdr_size(netbuf_cb, (FRAME_DESC_SZ + sizeof(xtended_desc_t))); // + TX_VECTOR_SZ));
	extnd_size = ((unsigned long )netbuf_cb->data & 0x3f);/* To make it 64 bit aligned buffer, to avoid mmc timeout interrupt */
	w_adapter->os_intf_ops->onebox_change_hdr_size(netbuf_cb, extnd_size); // + TX_VECTOR_SZ)); 
	w_adapter->os_intf_ops->onebox_memset(netbuf_cb->data,0,(FRAME_DESC_SZ + extnd_size + sizeof(xtended_desc_t)));// + TX_VECTOR_SZ));
	msg = (uint16 *)netbuf_cb->data;
	xtend_desc_pkt = (xtended_desc_t *)((uint8 *)netbuf_cb->data + FRAME_DESC_SZ);
	msg_len = netbuf_cb->len;


	//if(!((*(uint8 *)&msg[28/2 ] == 0x88) && (*((uint8 *)&msg[29/2] + 1) == 0x8e)))
	//if(!(ONEBOX_CPU_TO_BE16(msg[14]) == 0x888e))
	if(!(netbuf_cb->flags & M_EAPOL))
	{
		/* Except EAP packets remaining Mgmt packets with 
 		 * size more than 512 are dropped here */
		
		if(msg_len > (MAX_MGMT_PKT_SIZE)) 
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Management packet can't be more than 512 bytes\n")));
			w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (uint8 *)msg, msg_len);
			goto err;
		}
	}

	msg[0] = ONEBOX_CPU_TO_LE16((msg_len - FRAME_DESC_SZ) | (ONEBOX_WIFI_MGMT_Q << 12));
	msg[2] = ONEBOX_CPU_TO_LE16(extnd_size + sizeof(xtended_desc_t));
	msg[1] = ONEBOX_CPU_TO_LE16(TX_DOT11_MGMT);

	netbuf_cb->tx_pkt_type = WLAN_TX_M_Q;

	//if(!(ONEBOX_CPU_TO_BE16(msg[14]) == 0x888e))
	if(!(netbuf_cb->flags & M_EAPOL))
	{
		msg[3] = ONEBOX_CPU_TO_LE16(RATE_INFO_ENABLE | INSERT_SEQ_NO);
	}
	else
	{
		msg[3] = ONEBOX_CPU_TO_LE16(RATE_INFO_ENABLE);
	}

	if(wh->i_addr1[0] & 0x1)	
	{
		/*Set Broadcast bit flag for onair mgmt frames */
		msg[3] |= ONEBOX_CPU_TO_LE16(ONEBOX_BROADCAST_PKT);
	}
	
	if (!vap->hal_priv_vap->mgmt_rate_enable) {
		if((w_adapter->operating_band != BAND_2_4GHZ)
#ifdef ENABLE_P2P_SUPPORT
				|| (vap->p2p_enable)
#endif
			)
		{	
			/*Fill default rate for broad cast pkts in case of fixed rate */
			msg[4] = ONEBOX_CPU_TO_LE16(0xB | ONEBOX_11G_MODE); // 6mbps 11a mode
		}
		else
		{
			/*Fill default rate for broad cast pkts in case of fixed rate */
			msg[4] = ONEBOX_CPU_TO_LE16(0 | ONEBOX_11B_MODE); //1mbps  11b mode
		}
	} else {
		msg[4] = ONEBOX_CPU_TO_LE16(vap->hal_priv_vap->default_mgmt_rate);
	}


	if(ic->band_flags & IEEE80211_CHAN_HT40)
	{

					if (ic->band_flags & IEEE80211_CHAN_HT40U) {
									/* Primary Channel is Below secondary Channel. So send data in
									 * lower 20 band 
									 */
									msg[5] = ONEBOX_CPU_TO_LE16(LOWER_20_ENABLE); //full 40
									msg[5] |= ONEBOX_CPU_TO_LE16(LOWER_20_ENABLE >> 12); //full 40
					} else {
									/* Primary Channel is above secondary Channel. So send data in
									 * upper 20 band 
									 */
									msg[5] = ONEBOX_CPU_TO_LE16(UPPER_20_ENABLE); //full 40
									msg[5] |= ONEBOX_CPU_TO_LE16(UPPER_20_ENABLE >> 12); //full 40
					}
	}

	if((vap->iv_state > IEEE80211_S_SCAN) && (vap->iv_opmode == IEEE80211_M_STA) 
			&&  !(ic->ic_flags & IEEE80211_F_SCAN) && (w_adapter->operating_band == BAND_2_4GHZ)) {
		for(ii = 0; jj< ni->ni_rates.rs_nrates; ii++) {
			if(ni->ni_rates.rs_rates[ii] & IEEE80211_RATE_BASIC) {
				for(jj = 0 ; jj < MAX_BG_SUPP_RATES; jj++) {
					if((w_adapter->std_rates[jj]) == (ni->ni_rates.rs_rates[ii] & IEEE80211_RATE_VAL)) {
						msg[4] = w_adapter->rps_rates[jj];
						break;
					}
				}
				break;
			}
		}
	}
	/*Fill sequence number */
	msg[6] = ONEBOX_CPU_TO_LE16((*(uint16 *)&wh->i_seq[0]) >> 4);

#ifdef CONFIG_11W
 if(netbuf_cb->data[FRAME_DESC_SZ + extnd_size+ sizeof(xtended_desc_t) + 1] & 0x40 )
 {
         printk("<==== PROTECTED FRAME ENCRYPTION ====>\n");
         msg[2] |= ONEBOX_CPU_TO_LE16((MIN_802_11_HDR_LEN_MFP << 8));
         msg[6] = ONEBOX_CPU_TO_LE16( ONEBOX_BIT(12) | ONEBOX_BIT(15));
			   msg[3] &= ~ONEBOX_CPU_TO_LE16(INSERT_SEQ_NO);
 } else {
         msg[2] |= ONEBOX_CPU_TO_LE16((MIN_802_11_HDR_LEN << 8));
 }
#else
	msg[2] |= ONEBOX_CPU_TO_LE16((MIN_802_11_HDR_LEN << 8));
#endif


#ifdef BYPASS_TX_DATA_PATH
	if((netbuf_cb->data[extnd_size + FRAME_DESC_SZ + 12+ sizeof(xtended_desc_t)] == 0x88) 
					&& (netbuf_cb->data[extnd_size + FRAME_DESC_SZ + 13+ sizeof(xtended_desc_t)] == 0x8e) 
					&& (netbuf_cb->flags & M_EAPOL))
	{
			addqos = ((ni->ni_flags & (IEEE80211_NODE_QOS|IEEE80211_NODE_HT)));
			if(addqos){
					msg[6] |= ONEBOX_CPU_TO_LE16(ONEBOX_BIT(12));//enable qos
					msg[7] |= ONEBOX_CPU_TO_LE16(((7) << 4) | (netbuf_cb->sta_id << 8) ); //Tid: 7 indicates VOICE traffic
					ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND, (TEXT("In %s Line %d sta_id %d\n"), __func__, __LINE__, netbuf_cb->sta_id));
			}

			if((netbuf_cb->len == (EAPOL_4_LEN + FRAME_DESC_SZ + extnd_size + sizeof(xtended_desc_t))) 
							&& (vap->hal_priv_vap->conn_in_prog))
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND, (TEXT("<==== Sending EAPOL_4 Packet ====> \n")));
				//msg[1] |= ONEBOX_CPU_TO_LE16(1 << 10);  /* Expecting Confirm for this packet */
				if(vap->iv_opmode == IEEE80211_M_STA) {
					msg[1] |= ONEBOX_CPU_TO_LE16(REQUIRE_CONFIRM_TO_HOST);  /* Expecting Confirm for this packet */
					//msg[7] |= ONEBOX_CPU_TO_LE16(EAPOL4_CONFIRM ); /* Expecting confirm for eapol4 pkt */ 
					xtend_desc_pkt->confirm_frame_type = EAPOL4_CONFIRM;
				}
			}
			/* Encapsulate the pkt in TA */
			msg[1] |= ONEBOX_CPU_TO_LE16(1 << 15);
			msg[3] &= ~ONEBOX_CPU_TO_LE16(RATE_INFO_ENABLE);
			msg[3] &= ~ONEBOX_CPU_TO_LE16(ONEBOX_BROADCAST_PKT);
			msg[1] |= ONEBOX_CPU_TO_LE16(FETCH_RETRY_CNT_FRM_HST);  
			xtend_desc_pkt->retry_cnt = EAPOL_RETRY_CNT;
			ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND, (TEXT("<==== Sending EAPOL PKT IN MGMT QUEUE ====>\n")));
		}
#endif
	if(!(netbuf_cb->flags & M_EAPOL))
	{
		if(netbuf_cb->data[16 + extnd_size + sizeof(xtended_desc_t)] == 0x40)
		{
#ifndef WIFI_ALLIANCE
		if(vap->hal_priv_vap->probe_req_bcast)
		{
#endif
				msg[1] |= ONEBOX_CPU_TO_LE16(REQUIRE_CONFIRM_TO_HOST);  /* Expecting Confirm for this packet */
				xtend_desc_pkt->confirm_frame_type = PROBEREQ_CONFIRM;
				vap->hal_priv_vap->probe_req_bcast = 0;
#ifndef WIFI_ALLIANCE
		}
#endif
	}else if(netbuf_cb->data[16 + extnd_size + sizeof(xtended_desc_t)] == IEEE80211_FC0_SUBTYPE_PROBE_RESP) {
		msg[1] |= ONEBOX_CPU_TO_LE16(ADD_DELTA_TSF_VAP_ID);  
		msg[1] |= ONEBOX_CPU_TO_LE16(FETCH_RETRY_CNT_FRM_HST);  
		xtend_desc_pkt->retry_cnt = PROBE_RESP_RETRY_CNT;
	}

	if(vap->iv_opmode == IEEE80211_M_HOSTAP)
	{
		if((netbuf_cb->data[16 + extnd_size+ sizeof(xtended_desc_t)] == (IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS_NULL)) 
				|| (netbuf_cb->data[16 + extnd_size+ sizeof(xtended_desc_t)] == (IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_NODATA))) 
		{	
			if(IEEE80211_ADDR_EQ(&netbuf_cb->data[20 + extnd_size+ sizeof(xtended_desc_t)] ,ni->ni_macaddr))
			{
					msg[1] |= ONEBOX_CPU_TO_LE16(REQUIRE_CONFIRM_TO_HOST);  /* Expecting Confirm for this packet */
					msg[7] = ONEBOX_CPU_TO_LE16((ni->hal_priv_node.sta_id & 0xff) << 8);
					xtend_desc_pkt->confirm_frame_type = NULLDATA_CONFIRM;
			}
		}

		if((netbuf_cb->flags & M_EOSP))
		{
			if(ni->uapsd.uapsd_flags) {
				//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("indicating EOSP %s Line %d sta_id %d\n", __func__, __LINE__, netbuf_cb->sta_id)));
        msg[1] |= ONEBOX_CPU_TO_LE16(EOSP_INDICATION);  
			}
			msg[1] |= ONEBOX_CPU_TO_LE16(REQUIRE_TSF_SYNC_CONFIRM);  /* Expecting TSF for this packet */
		}
	}
#ifdef BYPASS_TX_DATA_PATH
	if(netbuf_cb->data[ETH_HDR_LEN + 2 + extnd_size+ sizeof(xtended_desc_t)] == 0x50)
	{
		/* Inserting TSF for probe response frames */
		msg[3] |= ONEBOX_CPU_TO_LE16(INSERT_TSF);
	}
#endif
	}
	msg[4] |= ONEBOX_CPU_TO_LE16((vap->hal_priv_vap->vap_id << 14));
	if(!(netbuf_cb->flags & M_EAPOL))
	{

		if(netbuf_cb->data[FRAME_DESC_SZ + extnd_size+ sizeof(xtended_desc_t)] == 0XD0)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND, (TEXT("<==== Sending ACTION FRAME %d====>\n"), __LINE__));
			msg[7] |= ONEBOX_CPU_TO_LE16((ni->hal_priv_node.sta_id << 8) ); //Tid: 7 indicates VOICE traffic
			ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND, (TEXT("Sta_id is %d\n"), ni->hal_priv_node.sta_id));
		}
	}

	if (w_adapter->sc_nmonvaps) 
	{
		onebox_completion_handler(w_adapter, netbuf_cb);
	}

	if(ni->ni_flags & IEEE80211_NODE_TMP)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" %s %d  Freeing  temp node in Hal\n"), __func__, __LINE__));
		w_adapter->net80211_ops->onebox_free_node(ni);
	}

	return ONEBOX_STATUS_SUCCESS;
err:
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(" %s %d  Freeing skb in error case\n"), __func__, __LINE__));
	w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb,0);
	return ONEBOX_STATUS_FAILURE;
}
/**
 * This function transmits the packets.
 *
 * @param Pointer to the private driver structure.
 * @param Pointer to netbuf control block structure.
 * @return ONEBOX_STATUS_SUCCESS on success else negative number on failure.
 *
 */
int core_xmit( WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb )
{
	struct ieee80211vap *vap = NULL;
	uint16 pkt_priority = BE_Q_STA;
	struct ieee80211_node *ni = NULL;
#ifndef BYPASS_TX_DATA_PATH
	uint8 qos_pkt = 0;
	uint16 seqno;
#endif
#ifdef BYPASS_TX_DATA_PATH
	uint8 tos;
#endif
	struct core_vap *core_vp;

	/* Drop Zero Length Packets */
	if (!netbuf_cb->len)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Zero Length packet\n"),__func__));
		w_adapter->stats.tx_dropped++;
		goto xmit_fail;
	}

	/* Drop Packets if FSM state is not open */
	if (w_adapter->fsm_state != FSM_MAC_INIT_DONE)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: FSM state not open\n"),__func__));
		w_adapter->stats.tx_dropped++;
		goto xmit_fail;
	}

	netbuf_cb->aggr_pcnt = 0;
//	netbuf_cb->retry_count = 0;
	netbuf_cb->aggr_flag = 0;
	netbuf_cb->aggr_len = 0;

	/* Get node for this SKB */
	ni = (struct ieee80211_node *)netbuf_cb->ni; 
	if (ni == NULL)
	{
		/* This happens if someone marks the underlying device up */
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s:Dropping; No node in netbuf control block!\n"),__func__));
		goto xmit_fail;
	}

	/* Get the vap pointer */
	vap = ni->ni_vap;
#ifndef BYPASS_TX_DATA_PATH
	if (netbuf_cb->data[MAC_80211_HDR_FRAME_CONTROL] & 0x80)
	{
		qos_pkt = 1;
		netbuf_cb->tid = netbuf_cb->data[24] & IEEE80211_QOS_TID;
	}
	else
	{
		netbuf_cb->tid = IEEE80211_NONQOS_TID;
	}
#else


		/* Incase of device encap pick the tos from the IP hdr */
	if (ONEBOX_CPU_TO_LE16(*(uint16 *)&netbuf_cb->data[12]) == htons(ETHERTYPE_IPV6)) {
		tos = (uint8_t)(ntohl(ONEBOX_CPU_TO_LE32(*(uint32 *)&netbuf_cb->data[14])) >> 20);
		tos >>= 5;
	} else {
		tos = netbuf_cb->data[15] >> 5;
	}


	netbuf_cb->tid = tos; 
	netbuf_cb->priority = TID_TO_WME_AC(tos);

#endif

	if ((netbuf_cb->flags & ONEBOX_MULTICAST) || (netbuf_cb->flags & ONEBOX_BROADCAST))
	{
		if(vap->iv_opmode == IEEE80211_M_HOSTAP)
		{
			core_vp = vap->hal_priv_vap->core_vp;
			w_adapter->stats.multicast++;
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("BROADCAST/MULTICAST PKTS \n")));
			if ((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&core_vp->rv_mcastq)) > MULTICAST_WATER_MARK)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(" MULTICAST Queue is Full\n")));
				goto xmit_fail;
			}
#ifndef BYPASS_TX_DATA_PATH
			//Filling seq no for broadcast pkts
			if (netbuf_cb->data[MAC_80211_HDR_FRAME_CONTROL] & 0x80)
			{
				seqno = ni->ni_txseqs[netbuf_cb->tid]++;
			}
			else
			{
				seqno = ni->ni_txseqs[IEEE80211_NONQOS_TID]++;
			}
			*(uint16_t *)((struct ieee80211_frame *)(netbuf_cb->data))->i_seq = ONEBOX_CPU_TO_LE16(seqno << IEEE80211_SEQ_SEQ_SHIFT);
#endif
			/* Queue broadcast/multicast packets */
			//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, netbuf_cb->data, 40);
			w_adapter->os_intf_ops->onebox_netbuf_queue_tail(&core_vp->rv_mcastq, netbuf_cb->pkt_addr);
			return ONEBOX_STATUS_SUCCESS;
		}
		else
		{
			netbuf_cb->flags &= ~ONEBOX_BROADCAST;
			netbuf_cb->flags &= ~ONEBOX_MULTICAST;
		}
	}
	else
	{
		w_adapter->stats.tx_packets++;
		w_adapter->stats.tx_bytes += netbuf_cb->len;
	}

	/* Assigning sta id */
	netbuf_cb->sta_id = ni->hal_priv_node.sta_id;

	/* Checking for WMM */
	if (w_adapter->vap_com.ic_flags & IEEE80211_F_WME) 
	{
		switch (netbuf_cb->priority) 
		{
			case WME_AC_VO:
				pkt_priority = VO_Q_STA;
				break;
			case WME_AC_VI:
				pkt_priority = VI_Q_STA;
				break;
			case WME_AC_BE:
				pkt_priority = BE_Q_STA;
				break;
			case WME_AC_BK:
				pkt_priority = BK_Q_STA;
				break;
			default:
				/* Should not come here */
				pkt_priority = BE_Q_STA;
		}
	} 
	else 
	{
		pkt_priority = BE_Q_STA;
	}

	if(vap->iv_opmode == IEEE80211_M_HOSTAP)
	{
		pkt_priority += 4;
	}

	if((vap->iv_opmode == IEEE80211_M_HOSTAP) && 
			(vap->iv_flags & IEEE80211_F_PRIVACY) && 
			!(ni->ni_flags & IEEE80211_NODE_ENCRYPT_ENBL) &&
			!((netbuf_cb->flags & M_EAPOL) && (netbuf_cb->data[12] == 0x88) && 
				(netbuf_cb->data[13] == 0x8e))
			)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Dropping pkts as key is not installed opmode %d sta_id %d\n"), 
						__func__, vap->iv_opmode, ni->hal_priv_node.sta_id));
		goto xmit_fail;

	}

	if((vap->iv_opmode == IEEE80211_M_HOSTAP) && 
			//!(ni->ni_flags & IEEE80211_NODE_AUTH) &&
			((netbuf_cb->flags & M_EAPOL) && (netbuf_cb->data[12] == 0x88) && (netbuf_cb->data[13] == 0x8e)))
	{
		pkt_priority = MGMT_SOFT_Q;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("<==== EAPOLPKT priority changed to MGMT_QUEUE in core_xmit Line %d ====>\n"),__LINE__));

	}
	if((vap->iv_opmode == IEEE80211_M_STA) 
        	//&&(vap->hal_priv_vap->conn_in_prog) 
                && ( (netbuf_cb->data[12] == 0x88) && (netbuf_cb->data[13] == 0x8e))) 
	{
		pkt_priority = MGMT_SOFT_Q;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("<==== EAPOLPKT priority changed to MGMT_QUEUE in core_xmit ====>\n")));
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("Pkt Priority = %d\n"), pkt_priority));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("Soft queue length before queueing %d\n"),
	w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[pkt_priority])));

	if((pkt_priority != MGMT_SOFT_Q ) && (pkt_priority != BROADCAST_HW_Q)) {
					/* Dropping Pkts as MAX QUEUE water mark level is reached */
					
					if((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[pkt_priority]) + 1 ) >= w_adapter->host_txq_maxlen[pkt_priority]) {
													vap->hal_priv_vap->stop_per_q[pkt_priority] = 1;
					
					}
	}

	if(vap->hal_priv_vap->stop_per_q[pkt_priority]) {

		if ((vap->iv_opmode == IEEE80211_M_STA) && 
		    (vap->hal_priv_vap->conn_in_prog) && 
				((vap->iv_rsn_ie) || (vap->iv_wpa_ie))) {
			// If connection is in progress and mode is WPA/2 modes, then we don't stop the queues 
			w_adapter->devdep_ops->onebox_schedule_pkt_for_tx(w_adapter);
		} else {
#ifndef USE_SUBQUEUES	
		if(!w_adapter->os_intf_ops->onebox_is_ifp_txq_stopped(vap->iv_ifp))
#else
		if(!w_adapter->os_intf_ops->onebox_is_sub_txq_stopped(vap->iv_ifp, netbuf_cb->skb_priority))
#endif
		{
#ifndef USE_SUBQUEUES	
			w_adapter->os_intf_ops->onebox_stop_ifp_txq(vap->iv_ifp);
#else
			w_adapter->os_intf_ops->onebox_stop_sub_txq(vap->iv_ifp, netbuf_cb->skb_priority);
#endif
			/* Has to be taken care for multiple vaps case in a different way */
		}
			w_adapter->devdep_ops->onebox_schedule_pkt_for_tx(w_adapter);
			goto xmit_fail;
		}
	}
		
	if ( ((pkt_priority == VO_Q_STA) || (pkt_priority == VO_Q_AP)) && 
	     ((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[pkt_priority]) + 1 ) >= VO_DATA_QUEUE_WATER_MARK )) {
			vap->hal_priv_vap->stop_udp_pkts[pkt_priority] = 1;
	}	else if (((pkt_priority == VI_Q_STA) || (pkt_priority == VI_Q_AP)) && 
	     ((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[pkt_priority]) + 1 ) >= VI_DATA_QUEUE_WATER_MARK )) {
			vap->hal_priv_vap->stop_udp_pkts[pkt_priority] = 1;
	}	else if ( ((pkt_priority == BE_Q_STA) || (pkt_priority == BE_Q_AP)) && 
	     ((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[pkt_priority]) + 1 ) >= BE_DATA_QUEUE_WATER_MARK ))	{
			vap->hal_priv_vap->stop_udp_pkts[pkt_priority] = 1;
	}	else if (((pkt_priority == BK_Q_STA)|| (pkt_priority == BK_Q_AP)) && 
	     ((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[pkt_priority]) + 1 ) >= BK_DATA_QUEUE_WATER_MARK )) {
			vap->hal_priv_vap->stop_udp_pkts[pkt_priority] = 1;
	}

	if(vap->hal_priv_vap->stop_udp_pkts[pkt_priority]) 
					w_adapter->devdep_ops->onebox_schedule_pkt_for_tx(w_adapter);

	/* Dropping UDP packets if the correspondig network queue is full */
#ifndef BYPASS_TX_DATA_PATH
	if(vap->hal_priv_vap->stop_udp_pkts[pkt_priority])
	{
		const struct ieee80211_cipher *cip;
		struct ieee80211_key *ikey;
		struct ieee80211_frame *wh;
		struct llc *llc;
		struct iphdr * ip;
		uint8 snap_offset;

		wh = (struct ieee80211_frame *)&netbuf_cb->data[0];
		if(qos_pkt)
		{
			snap_offset = MIN_802_11_HDR_LEN + 2;
		}
		else
		{
			snap_offset = MIN_802_11_HDR_LEN;
		}

		if (wh->i_fc[1] & IEEE80211_FC1_WEP) 
		{
			ikey = &ni->ni_ucastkey;
			cip = ikey->wk_cipher;
			if(cip == NULL)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s in %d xmit failed \n"),__func__,__LINE__));
				goto xmit_fail;
			}

			if( cip->ic_cipher == IEEE80211_CIPHER_TKIP)
			{
				/* iv + keyid + extiv + tx mic */
				snap_offset  += cip->ic_header + cip->ic_miclen + 6 /*IV*/ + 4 /*ICV*/;
			}
			else if((cip->ic_cipher == IEEE80211_CIPHER_AES_CCM) || (cip->ic_cipher == IEEE80211_CIPHER_WEP))
			{
				snap_offset += cip->ic_header;
			}
		}

		/* Extract the llc and ip offsets */
		llc = (struct llc *)&netbuf_cb->data[snap_offset];
		ip = (struct iphdr *)&netbuf_cb->data[ snap_offset + sizeof(struct llc)]; /*  yet to Handle the VLAN tagging case */
		//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ether type =%04x and ip proto=%04x\n",llc->llc_snap.ether_type, ip->protocol)));
		if((llc->llc_snap.ether_type == cpu_to_be16(0x0800)) && /* whether IP packet */
		   (ip->protocol == UDP_TYPE)) /* whether UDP packet */
		{
			udp_pkts_dropped++;
			if(udp_pkts_dropped && !(udp_pkts_dropped % 10000))
			{
			//	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("no of udp_pkts_dropped = %d\n", udp_pkts_dropped)));
			}
			goto xmit_fail;
		}
	}
#else //Device encap mode
	if(vap->hal_priv_vap->stop_udp_pkts[pkt_priority])
	{
		struct iphdr * ip;

		/* Extract the llc and ip offsets */
		ip = (struct iphdr *)&netbuf_cb->data[ ETH_HDR_LEN]; /*  yet to Handle the VLAN tagging case */
		if((netbuf_cb->data[ETH_PROTOCOL_OFFSET] == cpu_to_be16(0x0800)) && /* whether IP packet */
		   (ip->protocol == UDP_TYPE)) /* whether UDP packet */
		{
			w_adapter->udp_pkts_dropped++;
			if(w_adapter->udp_pkts_dropped && !(w_adapter->udp_pkts_dropped % 10000))
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("no of udp_pkts_dropped = %d\n"), w_adapter->udp_pkts_dropped));
			}
			goto xmit_fail;
		}
	}
#endif

#ifndef BYPASS_TX_DATA_PATH
	if(qos_pkt)
	{
		seqno = ni->ni_txseqs[netbuf_cb->tid]++;
	}
	else
	{
		seqno = ni->ni_txseqs[IEEE80211_NONQOS_TID]++;
	}
	*(uint16_t *)((struct ieee80211_frame *)(netbuf_cb->data))->i_seq = ONEBOX_CPU_TO_LE16(seqno << IEEE80211_SEQ_SEQ_SHIFT);
#endif

	//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Identifier is %x\n",*(uint16_t *)&netbuf_cb->data[38])));
	//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Sequence number is %d\n", seqno)));

	netbuf_cb->priority = (uint8)pkt_priority;
	//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s %d: Queueing pkts to %d\n", __func__, __LINE__, pkt_priority)));
	if((pkt_priority != MGMT_SOFT_Q)) {

			if(prepare_onair_data_pkt_desc(w_adapter, netbuf_cb, (pkt_priority))
							== ONEBOX_STATUS_SUCCESS){ 
					//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Queueing data pkts\n")));
					core_queue_pkt(w_adapter, netbuf_cb, (pkt_priority));

					/* Schedule the data transmit task */
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s: ===> Scheduling TX tasklet <===\n"),__func__));
					w_adapter->devdep_ops->onebox_schedule_pkt_for_tx(w_adapter);

					return ONEBOX_STATUS_SUCCESS;
			}
	} else if(prepare_onair_mgmt_pkt_desc(w_adapter, netbuf_cb) == ONEBOX_STATUS_SUCCESS) {
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Queuing onair mgmt pkt\n")));
			core_queue_pkt(w_adapter, netbuf_cb, (pkt_priority));

			/* Schedule the data transmit task */
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s: ===> Scheduling TX tasklet <===\n"),__func__));
			w_adapter->devdep_ops->onebox_schedule_pkt_for_tx(w_adapter);
			return ONEBOX_STATUS_SUCCESS;
	}
	

xmit_fail:	
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s:Failed to xmit packet\n"),__func__));
	if (netbuf_cb) 
	{
		w_adapter->stats.tx_dropped++;
		w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb,0);
	}
	w_adapter->total_tx_data_dropped[pkt_priority]++;

	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This functions adds the packet to socket buffer queue.
 *
 * @param Pointer to the private driver structure.
 * @param Pointer to netbuf control block structure.
 * @param queue number.
 *
 * @return none.
 */
inline void core_queue_pkt(WLAN_ADAPTER w_adapter, 
                           netbuf_ctrl_block_t *netbuf_cb, uint8 q_num)
{

	if (q_num > NUM_SOFT_QUEUES) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Invalid Queue Number: q_num = %d\n"), __func__, q_num));
		return;
	}

	w_adapter->os_intf_ops->onebox_netbuf_queue_tail(&w_adapter->host_tx_queue[q_num], netbuf_cb->pkt_addr);
	return;
}

/**
 * This functions deletes a packet FROM socket buffer queue.
 *
 * @param Pointer to the private driver structure.
 * @param queue number.
 * @return pointer to  netbuf_ctrl_block structure.
 */
netbuf_ctrl_block_t* core_dequeue_pkt(WLAN_ADAPTER w_adapter, uint8 q_num)
{
	netbuf_ctrl_block_t *netbuf_cb;

	if (q_num > NUM_SOFT_QUEUES)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Invalid Queue Number\n"),__func__));
		return NULL;
	}
	/* Dequeue the packet from the soft queue specified */
	{
		netbuf_cb = w_adapter->os_intf_ops->onebox_dequeue_pkt((void *)&w_adapter->host_tx_queue[q_num]);
	}
	return netbuf_cb;
}

/**
 * This function sends the management packets.
 *
 * @param       Pointer to node structure.
 * @param       Pointer to the netbuf control block structure.
 * @param       Pointer to the 
 * @return ONEBOX_STATUS_SUCCESS on success else negative number on failure.
 */
int onebox_send_mgmt_pkt(struct ieee80211_node *ni, netbuf_ctrl_block_m_t *netbuf_cb_m, const struct ieee80211_bpf_params *bpf_params)
{
	struct ieee80211com *ic;
	struct ieee80211vap *vap;
	struct onebox_os_intf_operations *os_intf_ops;
	WLAN_ADAPTER w_adapter;
	netbuf_ctrl_block_t *netbuf_cb;
	uint16 node_id;
	/* Convention followed in station mode, vap_id is zero */
	uint8 vap_id = 0;

	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	ic = ni->ni_ic;
	vap = ni->ni_vap;
	vap_id = vap->hal_priv_vap->vap_id;
	w_adapter = os_intf_ops->onebox_get_priv((void *)ic->ic_ifp);

	netbuf_cb = onebox_translate_mbuf_to_netbuf(netbuf_cb_m);
	/* Free netbuf_cb_m irrespective of the status from above function */
	if(netbuf_cb != NULL)
	{
		netbuf_cb->flags |= netbuf_cb_m->m_hdr.mh_flags;
	}
	kfree(netbuf_cb_m);

	node_id = ni?ni->hal_priv_node.sta_id:0xff;

	if (!netbuf_cb->len)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%sZero Length mgmt packet\n"),__func__));
		goto send_mgmt_fail;
	}

	if (w_adapter->fsm_state != FSM_MAC_INIT_DONE)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: FSM state is not open \n"),__func__));
		goto send_mgmt_fail;
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND,(TEXT("%s: Mgmt Packet Length = %d\n"), __func__,netbuf_cb->len));

	netbuf_cb->ni = ni;

	if (w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[MGMT_SOFT_Q]) > 200) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Management Soft queue is full\n"),__func__));
		goto send_mgmt_fail;
	}

	netbuf_cb->mac_hdr_len = MIN_802_11_HDR_LEN;

	/* Adding The Packet On The Skb Queue */
	ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND,(TEXT("%s: Adding to soft queue %d\n"),__func__, MGMT_SOFT_Q));
	if ((netbuf_cb->data[0] == 0x00) || (netbuf_cb->data[0] == 0x20)) 
	{
		if(vap->iv_opmode == IEEE80211_M_STA)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND, (TEXT("Sending peer notify \n")));
			core_send_station_info(ni, STA_CONNECTED);
		}
	}
	if(prepare_onair_mgmt_pkt_desc(w_adapter, netbuf_cb) == ONEBOX_STATUS_SUCCESS){
	core_queue_pkt(w_adapter, netbuf_cb, MGMT_SOFT_Q);

	/* Schedule the data transmit task */
	ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND,(TEXT("Scheduling the qos_processor tasklet\n")));
	w_adapter->devdep_ops->onebox_schedule_pkt_for_tx(w_adapter);

	return ONEBOX_STATUS_SUCCESS;
	}

send_mgmt_fail:
	if (netbuf_cb) 
	{
		w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb,0);
	}

	return ONEBOX_STATUS_SUCCESS;
}
