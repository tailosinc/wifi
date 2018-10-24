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
#include "onebox_wlan_pktpro.h"
#include "onebox_autorate.h"


/**
 * This function sends the recieved data packet onto air.
 *
 * @param       Pointer to private driver structure.
 * @param       Pointer to netbuf control block structure.
 * @return      ONEBOX_STATUS_SUCCESS on success else ONEBOX_STATUS_FAILURE.
 */
ONEBOX_STATUS send_onair_data_pkt(WLAN_ADAPTER w_adapter, 
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
	uint32 status;
	uint8 security_enabled = 0;
	uint8 extnd_size = 0;
	xtended_desc_t *xtend_desc_pkt = NULL;
	struct driver_assets *d_assets = w_adapter->d_assets;

	//: Change label from fail to data fail
	FUNCTION_ENTRY(ONEBOX_ZONE_DATA_SEND);
	ni = (struct ieee80211_node *)netbuf_cb->ni;/* NB: always passed down by 802.11 layer */
	if (ni == NULL)
	{
		/* NB: this happens if someone marks the underlying device up */
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
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
	extnd_size = ((unsigned long )netbuf_cb->data & 0x3f);  /* To make it 64bit aligned buffer, to avoid mmc timeout interrupt */
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
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERR: In %s in %d Failed \n"),__func__,__LINE__));
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
				if(((ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20))  && (vap->iv_flags_ht & IEEE80211_FHT_SHORTGI20))
				{
					if(vap->hal_priv_vap->rate_hix & 0x100) /*check this for MCS rates only */
					{
						frame_desc[4] |= ENABLE_SHORTGI_RATE;;/* Indicates shortGi HT Rate */
					}			
				}			
		}
	}
	frame_desc[7] = ONEBOX_CPU_TO_LE16(((TID_MAPPING_AC(netbuf_cb->tid & 0xf)) << 4) | (q_num & 0xf) | (netbuf_cb->sta_id << 8));
	if((vap->iv_opmode == IEEE80211_M_HOSTAP) && (netbuf_cb->flags & MORE_DATA) && (q_num != BEACON_HW_Q) && (q_num != BROADCAST_HW_Q))
	{
		frame_desc[3] |= (MORE_DATA_PRESENT);
	}
	if((q_num == BROADCAST_HW_Q) || (q_num == BEACON_HW_Q))
	{
		frame_desc[3] |= (INSERT_SEQ_NO);
	}
	if ((netbuf_cb->flags & ONEBOX_BROADCAST) || (netbuf_cb->flags & ONEBOX_MULTICAST))
	{
		frame_desc[3] |= ONEBOX_CPU_TO_LE16(ONEBOX_BROADCAST_PKT | RATE_INFO_ENABLE);

		/* : DO we need to handle host encap/decap separately */
		if(netbuf_cb->flags & FIRST_BCAST)
		{
			/*first broad cast packet set beacon gated frame */
			//frame_desc[3] |= (DTIM_BEACON_GATED_FRAME );
		}

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
					frame_desc[4] = ONEBOX_CPU_TO_LE16(vap->hal_priv_vap->default_mgmt_rate); //1mbps  11b mode
			}

		frame_desc[7] = ONEBOX_CPU_TO_LE16(((TID_MAPPING_AC(netbuf_cb->tid & 0xf)) << 4) | (q_num & 0xf) | (vap->hal_priv_vap->vap_id << 8));
	}
	
	frame_desc[4] |= ONEBOX_CPU_TO_LE16((vap->hal_priv_vap->vap_id << 14));
	
	if (vap->iv_flags_ht & IEEE80211_FHT_USEHT40) {
			if(ic->band_flags & IEEE80211_CHAN_HT40) {
					if(ic->band_flags & IEEE80211_CHAN_HT40U) {
							frame_desc[5] = ONEBOX_CPU_TO_LE16(LOWER_20_ENABLE); //full 40
							frame_desc[5] |= ONEBOX_CPU_TO_LE16(LOWER_20_ENABLE >> 12); //full 40

					} else {
							frame_desc[5] = ONEBOX_CPU_TO_LE16(UPPER_20_ENABLE); //full 40
							frame_desc[5] |= ONEBOX_CPU_TO_LE16(UPPER_20_ENABLE >> 12); //full 40

					}
			}
	}
	/*Fill bbp info */
	/*frame_desc[5] = 0;*/

#ifndef BYPASS_TX_DATA_PATH
	/*Fill sequence number */
	frame_desc[6] |= ONEBOX_CPU_TO_LE16((seq_num) & 0xfff);
#endif
//	frame_desc[6] |= ONEBOX_CPU_TO_LE16(key_idx << 13);


#ifdef BYPASS_TX_DATA_PATH
	if(netbuf_cb->data[28] == 0x88 && netbuf_cb->data[29] == 0x8e)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<==== Sending EAPOL to %02x:%02x:%02x:%02x:%02x:%02x ====>\n"),
				netbuf_cb->data[16], netbuf_cb->data[17], netbuf_cb->data[18], 
				netbuf_cb->data[19], netbuf_cb->data[20], netbuf_cb->data[21]));
		frame_desc[6] |= ONEBOX_CPU_TO_LE16(BIT(13));//This bit indicates firmware to add this pkt to sw_q head
		frame_desc[1] |= ONEBOX_CPU_TO_LE16(FETCH_RETRY_CNT_FRM_HST);  
		xtend_desc_pkt->retry_cnt = EAPOL_RETRY_CNT;
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (uint8 *)netbuf_cb->data, netbuf_cb->len);
	}
#endif

	/* Assign desc_ptr, pkt_len values */
	if(netbuf_cb->len <= 1594)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_DATA_SEND, (TEXT("Data pkt dump\n")));
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DATA_SEND, (uint8 *)netbuf_cb->data, netbuf_cb->len);

		status = w_adapter->onebox_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, WLAN_Q);
		if (status != ONEBOX_STATUS_SUCCESS) 
		{ 
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT 
			             ("%s: Failed To Write The Packet\n"),__func__));
		}
		/* Perform some processing here */
		if (w_adapter->sc_nmonvaps) 
		{
			w_adapter->os_intf_ops->onebox_netbuf_adj(netbuf_cb, FRAME_DESC_SZ + netbuf_cb->data[OFFSET_EXT_DESC_SIZE]);
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
			{
#if 0
				w_adapter->os_intf_ops->onebox_memmove(&netbuf_cb->data[ieee80211_size], 
				                                     &netbuf_cb->data[0],
				                                     ieee80211_size); 
				w_adapter->os_intf_ops->onebox_netbuf_adj(netbuf_cb, ieee80211_size);
#endif
			}
			//onebox_completion_handler(w_adapter, netbuf_cb);
		}
	}
	else 
	{ 
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT 
		             ("Bigger Packet > 1594 Bytes, Not Transmitting pkt_len = %d\n"), netbuf_cb->len));
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (uint8 *)netbuf_cb->data, netbuf_cb->len);
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

	if (d_assets->host_intf_type == HOST_INTF_SDIO)
		w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
	FUNCTION_EXIT(ONEBOX_ZONE_DATA_SEND);
	return ONEBOX_STATUS_SUCCESS;
fail:
	w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
	return ONEBOX_STATUS_FAILURE;
}


/**
 * This function prepares the beacon packet and sends it onto air.
 *
 * @param       Pointer to private driver's structure.
 * @param       Pointer to network buffer control block structure.
 * @param       Pointer to the core vap structure.
 * @param       dtim period value.
 * @return      ONEBOX_STATUS_SUCCESS on success else ONEBOX_STATUS_FAILURE.
 */
ONEBOX_STATUS send_beacon(WLAN_ADAPTER w_adapter, 
                          netbuf_ctrl_block_t *netbuf_cb,
                          struct core_vap *core_vp,
                          int8 dtim_beacon)
{

	ONEBOX_STATUS status;
	onebox_mac_frame_t *frame_desc;
	netbuf_ctrl_block_t *mcast_netbuf = NULL;
	uint8 i = 0;
	netbuf_ctrl_block_t *beacon_buf = NULL;
	uint8 extnd_size;
	struct ieee80211_node *ni = (struct ieee80211_node *)netbuf_cb->ni;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = &w_adapter->vap_com;
	uint32 counter = 0;
	unsigned long spin_lock_flags;
	uint8 device_buf_status = 0;
	struct driver_assets *d_assets = w_adapter->d_assets;

	if (netbuf_cb->len > 506)// 6 bytes will be used to give addr length in words and length in bytes
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("beacon length > 506\n")));
		return ONEBOX_STATUS_FAILURE;
	}

	if(w_adapter->mgmt_buffer_full)
	{
		return ONEBOX_STATUS_FAILURE;
	}

	/* If there are minimum two broadcast packets dequeue the first one and place it in the queue */

	/* Preparing the Packet */
	beacon_buf = w_adapter->os_intf_ops->onebox_alloc_skb(netbuf_cb->len + FRAME_DESC_SZ);
	if(beacon_buf == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Line %d Unable to allocate skb for beacon \n"), __func__, __LINE__));
	 	return ONEBOX_STATUS_FAILURE;
	}
	w_adapter->os_intf_ops->onebox_add_data_to_skb(beacon_buf, netbuf_cb->len + FRAME_DESC_SZ);

	extnd_size = ((unsigned long )beacon_buf->data & 0x3f);

	w_adapter->os_intf_ops->onebox_change_hdr_size(beacon_buf, (extnd_size));

	frame_desc = (onebox_mac_frame_t *)beacon_buf->data;
	w_adapter->os_intf_ops->onebox_memset((uint8 *)frame_desc, 0, FRAME_DESC_SZ);

	/* Copy beacon data from netbuf_cb->data to beacon_buf*/
	w_adapter->os_intf_ops->onebox_memcpy(((uint8 *)beacon_buf->data + FRAME_DESC_SZ + extnd_size), (uint8 *)netbuf_cb->data, netbuf_cb->len);

	frame_desc->desc_word[0] = ONEBOX_CPU_TO_LE16((beacon_buf->len - FRAME_DESC_SZ) | (ONEBOX_WIFI_DATA_Q << 12));
	beacon_buf->tx_pkt_type = WLAN_TX_D_Q;
	frame_desc->desc_word[1] = ONEBOX_CPU_TO_LE16(0);		//encap in umac
	// (Mac hdr len in MSB) && extended desc size
	frame_desc->desc_word[2] = ONEBOX_CPU_TO_LE16((MIN_802_11_HDR_LEN << 8) | (extnd_size));		
	/* Fill mac information */ 
	frame_desc->desc_word[3] |= ONEBOX_CPU_TO_LE16(MAC_BBP_INFO | NO_ACK_IND | (BEACON_FRAME)| INSERT_TSF | INSERT_SEQ_NO);
		frame_desc->desc_word[3]  |= ONEBOX_CPU_TO_LE16(RATE_INFO_ENABLE);
	frame_desc->desc_word[7] = ONEBOX_CPU_TO_LE16(BEACON_HW_Q);
	frame_desc->desc_word[4] |= ONEBOX_CPU_TO_LE16((vap->hal_priv_vap->vap_id << 14));
  
	if(ic->band_flags & IEEE80211_CHAN_HT40) {
			if(ic->band_flags & IEEE80211_CHAN_HT40U) {
					/*Secondary channel is upper the primary channel */
					frame_desc->desc_word[5] = ONEBOX_CPU_TO_LE16(LOWER_20_ENABLE); 
					frame_desc->desc_word[5] |= ONEBOX_CPU_TO_LE16(LOWER_20_ENABLE >> 12);

			}
			else {
					frame_desc->desc_word[5] = ONEBOX_CPU_TO_LE16(UPPER_20_ENABLE); 
					frame_desc->desc_word[5] |= ONEBOX_CPU_TO_LE16(UPPER_20_ENABLE >> 12);

			}
	}

	if (!vap->hal_priv_vap->mgmt_rate_enable) {
		if((w_adapter->operating_band != BAND_2_4GHZ)
#ifdef ENABLE_P2P_SUPPORT
				|| (vap->p2p_enable)
#endif
			)
		{	
			/*Fill default rate for broad cast pkts in case of fixed rate */
			frame_desc->desc_word[4] |= ONEBOX_CPU_TO_LE16(0xB | ONEBOX_11G_MODE); // 6mbps 11a mode
		}
		else
		{
			/*Fill default rate for broad cast pkts in case of fixed rate */
			frame_desc->desc_word[4] |= ONEBOX_CPU_TO_LE16(0 | ONEBOX_11B_MODE); //1mbps  11b mode
		}
	} else {
			frame_desc->desc_word[4] |= ONEBOX_CPU_TO_LE16(vap->hal_priv_vap->default_mgmt_rate);
	}

	if(dtim_beacon)
	{
		frame_desc->desc_word[3] |= ONEBOX_CPU_TO_LE16(DTIM_BEACON);
	}

	if (beacon_buf->len > (512))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(KERN_ERR "MGMT_LEN > 512 Bytes, Drop it\n")));
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(KERN_ERR "Beacon Length greater than 512\n")));
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, beacon_buf->data, beacon_buf->len);
		/* Freeing memory allocated for beacon */
		w_adapter->os_intf_ops->onebox_free_pkt(beacon_buf, 0);
		return ONEBOX_STATUS_FAILURE;
	}

	status = w_adapter->onebox_send_pkt_to_coex(w_adapter->d_assets, beacon_buf, WLAN_Q);

	if(status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("load beacon, onebox_card_write faile\n")));
	}
	else
	{
		if (w_adapter->sc_nmonvaps) 
		{
			//onebox_completion_handler(w_adapter, netbuf_cb);
		}
		w_adapter->total_beacon_count++;
	}
	/* Freeing memory allocated for beacon */
	if (d_assets->host_intf_type == HOST_INTF_SDIO)
		w_adapter->os_intf_ops->onebox_free_pkt(beacon_buf, 0);
	if(dtim_beacon)
	{
		while(((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&core_vp->rv_mcastq))) > 0)
		{
			if (i > 3)
				break;
			i++;

	if(d_assets->host_intf_type == HOST_INTF_USB) {
			counter = d_assets->update_usb_buf_status(d_assets->global_priv);
			if(counter >= 15) {
				counter = 0;
				w_adapter->os_intf_ops->onebox_acquire_spinlock(&d_assets->usb_wlan_coex_write_pending, &spin_lock_flags);
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("CORE_MSG: BUFFER FULL Queueing data packets till BUFFER_FREE COMES\n")));
				atomic_set(&d_assets->techs[WLAN_ID].pkt_write_pending, 1);
				w_adapter->os_intf_ops->onebox_release_spinlock(&d_assets->usb_wlan_coex_write_pending, spin_lock_flags);
				break;
			}
	} else {
		status = d_assets->onebox_common_read_register(d_assets,
				w_adapter->buffer_status_register,
				&device_buf_status); /* Buffer */

		if (status != ONEBOX_STATUS_SUCCESS)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Failed to Buffer Status Register\n"),__func__));
			break;
		}
		/* Updating The buffer_full Status In w_adapter Member */
		if(device_buf_status & (ONEBOX_BIT(SD_PKT_MGMT_BUFF_FULL)))
		{
			w_adapter->mgmt_buffer_full = 1;
		}
		else
		{
			w_adapter->mgmt_buffer_full = 0;
		}
		if(device_buf_status & (ONEBOX_BIT(SD_PKT_BUFF_FULL)))
		{
			w_adapter->buffer_full = 1;
		}
		else
		{
			w_adapter->buffer_full = 0;
		}
		if(device_buf_status & (ONEBOX_BIT(SD_PKT_BUFF_SEMI_FULL)))
		{
			w_adapter->semi_buffer_full = 1;
		}
		else
		{
			w_adapter->semi_buffer_full = 0;
		}
		if (w_adapter->device_model == RSI_DEV_9116) {
			if(w_adapter->semi_buffer_full) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: DATA buffer full, Queing Multicast packet unitl buffer free\n"),__func__));
				break;
			}
		}
		else if(w_adapter->buffer_full)	{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: DATA buffer full, Queing Multicast packet unitl buffer free\n"),__func__));
			break;
		}
	}
	mcast_netbuf = w_adapter->os_intf_ops->onebox_dequeue_pkt((void *)&core_vp->rv_mcastq);
	if(mcast_netbuf == NULL) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("mcast netbuf NULL, stop tx of mcast pkts\n")));
		return ONEBOX_STATUS_FAILURE; 
	}

	if (((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&core_vp->rv_mcastq)) - 1) > 0)
	{
		/* More data is present so indicate it to fw */
		mcast_netbuf->flags |= MORE_DATA;
	}
	else
	{
		mcast_netbuf->flags &= ~MORE_DATA;
		mcast_netbuf->flags |= LAST_BCAST;
	}
	mcast_netbuf->priority = MCAST_QNUM;
	/* send the multicast/broadcast traffic after dtim beacon */
#ifdef SEPERATE_BROADCAST_QUEUE
	send_onair_data_pkt(w_adapter, mcast_netbuf, BROADCAST_HW_Q);
#else
	send_onair_data_pkt(w_adapter, mcast_netbuf, BEACON_HW_Q);
#endif
		}
	}
#ifdef P2P_ENABLE
	/*Scan the devices when we are Group Owner mode */
	if(vap->p2p_scan_chan)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Doing Device discovery in Group operation state in GO mode\n")));
		vap->p2p_prev_state = vap->p2p_cur_state;
		vap->p2p_cur_state = EVENT_SEARCH;
		vap->iv_ic->ic_p2p_state_controller(vap,NULL);
	}
	/* Initiating the Invitation req after beacon transmission */
	if((vap->p2p_inv_node != NULL) /*&& (vap->listen_mode == 0)*/)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Initiating Invitation Request to "MAC_FMT"\n"),MAC_ADDR(ni->ni_macaddr)));
		vap->p2p_prev_state = vap->p2p_cur_state;
		vap->p2p_cur_state  = EVENT_P2P_INV; 
		p2p_state_controller(vap,vap->p2p_inv_node);
		vap->p2p_inv_node = NULL;
	}
#endif
	return ONEBOX_STATUS_SUCCESS;
} /* send_beacon */


void onebox_internal_pkt_dump(WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Ifname is %s\n"), w_adapter->vap_com.ic_ifp->name));
	switch(netbuf_cb->data[2])
	{
		case RESET_MAC_REQ:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING RESET_MAC_REQUEST FRAME =====>\n")));
			break;
		case RADIO_CAPABILITIES:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING RADIO_CAPABILITIES =====>\n")));
			break;
		case BB_PROG_VALUES_REQUEST:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING BB_PROG_VALUES_REQUEST FRAME =====>\n")));
			break;
		case RF_PROG_VALUES_REQUEST:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING RF_PROG_VALUES_REQUEST FRAME =====>\n")));
			break;
		case WAKEUP_SLEEP_REQUEST:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING WAKEUP_SLEEP_REQUEST FRAME =====>\n")));
			break;
		case SCAN_REQUEST:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING SCAN_REQUEST FRAME =====>\n")));
			break;
		case TSF_UPDATE:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING TSF_UPDATE FRAME =====>\n")));
			break;
		case PEER_NOTIFY:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING PEER_NOTIFY FRAME =====>\n")));
			break;
		case BLOCK_UNBLOCK:
			if(netbuf_cb->data[8] == 0xf){
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== BLOCK DATA QUEUES =====>\n")));
			}
			else{
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== UNBLOCK DATA QUEUES =====>\n")));
			}
			break;
		case SET_KEY:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING SET_KEY FRAME =====>\n")));
			break;
		case AUTO_RATE_IND:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING AUTO_RATE_IND FRAME =====>\n")));
			break;
		case BOOTUP_PARAMS_REQUEST:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING BOOTUP_PARAMS_REQUEST FRAME =====>\n")));
			break;
		case VAP_CAPABILITIES:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING VAP_CAPABILITIES FRAME =====>\n")));
			break;
		case VAP_DYNAMIC_UPDATE:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING VAP_DYNAMIC_UPDATE_FRAME =====>\n")));
			break;
		case EEPROM_READ_TYPE:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING EEPROM_READ_TYPE FRAME =====>\n")));
			break;
		case EEPROM_WRITE:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING EEPROM_WRITE FRAME =====>\n")));
			break;
		case GPIO_PIN_CONFIG:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING GPIO_PIN_CONFIG FRAME =====>\n")));
			break;
		case SET_RX_FILTER:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING SET_RX_FILTER FRAME =====>\n")));
			break;
		case AMPDU_IND:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING AMPDU_IND FRAME =====>\n")));
			break;
		case STATS_REQUEST_FRAME:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING STATS_REQUEST_FRAME FRAME =====>\n")));
			break;
		case BB_BUF_PROG_VALUES_REQ:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING BB_BUF_PROG_VALUES_REQ FRAME =====>\n")));
			break;
		case BBP_PROG_IN_TA:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING BBP_PROG_IN_TA FRAME =====>\n")));
			break;
		case BG_SCAN_PARAMS:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING BG_SCAN_PARAMS FRAME =====>\n")));
			break;
		case BG_SCAN_PROBE_REQ:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING BG_SCAN_PROBE_REQ FRAME =====>\n")));
			break;
		case CW_MODE_REQ:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING CW_MODE_REQ FRAME =====>\n")));
			break;
		case RADIO_PARAMS_UPDATE:
      ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,("<===== RADIO PARAMS UPDATE =====>\n" ));
			break;
		case PER_CMD_PKT:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING PER_CMD_PKT FRAME =====>\n")));
			break;
		case DEV_SLEEP_REQUEST:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING DEV_SLEEP_REQUEST FRAME =====>\n")));
			break;
		case DEV_WAKEUP_CNF:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING DEV_WAKEUP_CNF FRAME =====>\n")));
			break;
		case ANT_SEL_FRAME:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING ANT_SEL FRAME =====>\n")));
			break;
		case CONFIRM:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING CONFIRM FRAME =====>\n")));
			break;
		case DEBUG_FRAME:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING DEBUG FRAME =====>\n")));
			break;
			//case HW_BEACON_MISS_HANDLE:
			//		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("HW_BEACON_MISS_HANDLE FRAME\n")));
			//		break;
		case WOWLAN_CONFIG_PARAMS:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING WOWLAN_CONFIG FRAME =====>\n")));
			break;
		case FEATURES_ENABLE:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING FEATURES_ENABLE FRAME =====>\n")));
      break;
		case LOG_STRUCT_PRGMG:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING STRUCT PROGRAMMING REQ FRAME =====>\n")));
      break;
    case COEX_CONFIGURATIONS:  //COEX SLOTTING CONTROL
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING COEX CONFIGURATION  FRAME =====>\n")));
      break;
    case DISABLE_PROGRAMMING:  //DISABLE_PROGRAMMING
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING DISABLE_PROGRAMMING REQ  FRAME =====>\n")));
      break;
    case WLAN_IQ_STATS_REQ:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING WLAN_IQ_STATS_REQ  FRAME =====>\n")));
      break;
    case GTK_REKEY_DATA:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING GTK_REKEY_DATA FRAME =====>\n")));
      break;
		default:
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<===== SENDING UNKNOWN FRAME =====>\n")));
			break;
	}
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)netbuf_cb->data, netbuf_cb->len);
}

void onair_mgmt_dump( WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb, uint8 extnd_size)
{
		switch(netbuf_cb->data[FRAME_DESC_SZ + extnd_size])
		{
			case IEEE80211_FC0_SUBTYPE_AUTH:
#ifdef BYPASS_TX_DATA_PATH
				if((netbuf_cb->data[FRAME_DESC_SZ + extnd_size + 12] != 0x88) 
								&&(netbuf_cb->data[FRAME_DESC_SZ + extnd_size + 13] != 0x8e))
#endif
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Sending AUTHENTICATION Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_ASSOC_REQ:
#ifdef BYPASS_TX_DATA_PATH
				if((netbuf_cb->data[FRAME_DESC_SZ + extnd_size + 12] != 0x88) 
								&&(netbuf_cb->data[FRAME_DESC_SZ + extnd_size + 13] != 0x8e))
#endif
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Sending ASSOCIATION REQUEST Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_REASSOC_REQ:
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Sending RE-ASSOCIATION RESPONSE Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_ASSOC_RESP:
#ifdef BYPASS_TX_DATA_PATH
				if((netbuf_cb->data[FRAME_DESC_SZ + extnd_size + 12] != 0x88) 
								&&(netbuf_cb->data[FRAME_DESC_SZ + extnd_size + 13] != 0x8e))
#endif
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Sending ASSOCIATION RESPONSE Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_PROBE_REQ:
					ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Sending Probe Request Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_DISASSOC:
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Sending Disassociation Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_DEAUTH:
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Sending Deauthentication Packet ====>\n")));
				break;
			default:
				return;
		}
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (uint8 *)netbuf_cb->data, netbuf_cb->len);
}

void recv_onair_dump(WLAN_ADAPTER w_adapter, uint8 *buffer, uint32 len)
{
		switch(buffer[0])
		{
			case IEEE80211_FC0_SUBTYPE_AUTH:
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Recvd AUTHENTICATION Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_REASSOC_REQ:
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Recvd REASSOCIATION REQUEST Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_ASSOC_REQ:
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Recvd ASSOCIATION REQUEST Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_ASSOC_RESP:
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Recvd ASSOCIATION RESPONSE Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_REASSOC_RESP:
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Recvd ASSOCIATION RESPONSE Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_DISASSOC:
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Recvd Disassociation Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_DEAUTH:
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Recvd Deauthentication Packet ====>\n")));
				break;
			case IEEE80211_FC0_SUBTYPE_ACTION:
			default:
				return;
		}
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, buffer, len);
}

