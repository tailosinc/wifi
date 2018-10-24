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
/**
 * This function is used to send a frame to the PPE for requesting Stats.
 *
 * @param Pointer to the w_adapter structure.
 * return ONEBOX_STATUS_SUCCESS on success else ONEBOX_STATUS_FAILURE.
 */
ONEBOX_STATUS stats_frame(WLAN_ADAPTER w_adapter)
{

	ONEBOX_STATUS status = 0;
	onebox_mac_frame_t *mgmt_frame;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("===> Sending PER STATS REQUEST FRAME <===\n")));

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);
	
	/* Do not fill with the Zero length, eventhough there is no framebody  */
	/* If you fill Zero length, DMA will stuck*/
//	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(5);
	/* FrameType*/
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(STATS_REQUEST_FRAME);
	mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(w_adapter->ch_util_start_flag);
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(w_adapter->stats_interval);
	mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(w_adapter->false_cca_rssi_threshold);

	/* Indication to PPE to request statistics */
//	mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(0x1);
	mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12);
	if (w_adapter->recv_stop)
	{
		// this is reserved field
		//set this value to Indicate firmware to stop sending stats
		mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(1 << 8);
	}    
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (FRAME_DESC_SZ));
	return status;
}


/**
 * This function starts/stops the PER transmission mode (Burst/Continuous).
 *
 * @param Pointer to the w_adapter structure.
 * @return status 0 on success else -1 on failure.
 */
int start_per_tx(WLAN_ADAPTER w_adapter)
{
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS ;

	if (w_adapter->endpoint_params.enable)
	{  
		w_adapter->per_flag = 0;
		if (w_adapter->tx_running)
		{
			return ONEBOX_STATUS_FAILURE;
		}	

		if(w_adapter->fsm_state == FSM_MAC_INIT_DONE )
		{
      if (w_adapter->device_model == RSI_DEV_9116) {
        if(w_adapter->d_assets->disable_programming){
          goto  SEND_PER_CMD;
        } else {
                    w_adapter->devdep_ops->onebox_band_check(w_adapter);
          set_per_configurations (w_adapter); 
        }
      }

			if (w_adapter->endpoint_params.channel == 0xFF) {
				if(w_adapter->devdep_ops->onebox_mgmt_send_bb_reset_req(w_adapter) != ONEBOX_STATUS_SUCCESS) {
					return ONEBOX_STATUS_FAILURE;
				}
			} else if(w_adapter->endpoint_params.channel) {
				w_adapter->devdep_ops->onebox_band_check(w_adapter);
				w_adapter->fsm_state = FSM_SCAN_CFM;		
				w_adapter->devdep_ops->onebox_set_channel(w_adapter, w_adapter->endpoint_params.channel);
				if(w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->bb_rf_event), EVENT_WAIT_FOREVER) < 0 ) {
					ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed\n", __func__, __LINE__));
					return ONEBOX_STATUS_FAILURE;
				}
				w_adapter->fsm_state = FSM_MAC_INIT_DONE;
				w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->bb_rf_event));
			}
		}
SEND_PER_CMD:
		if ( (!w_adapter->endpoint_params.length))
		{    
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ONEBOX_set_per_tx_mode: Continuous TX len not supported%d\n"),
			                                       w_adapter->endpoint_params.length));
			return ONEBOX_STATUS_FAILURE;
		}    
    
		if (w_adapter->endpoint_params.mode == PER_CONT_MODE)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_OID, (TEXT(
			          "onebox_set_per_tx_mode: Enabling the PER continuous mode\n")));
			if (do_continuous_send(w_adapter))
			{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(
													"Cant send as do_continuous failed : PER\n")));
					return ONEBOX_STATUS_FAILURE;
			}
			w_adapter->tx_running = CONTINUOUS_RUNNING; //indicating PER_CONT_MODE
		}


		if (w_adapter->endpoint_params.mode == PER_BURST_MODE)
		{   
			onebox_send_per_frame(w_adapter,PER_BURST_MODE);
			if(w_adapter->endpoint_params.aggr_enable == 1)
			{
				send_per_ampdu_indiaction_frame(w_adapter);
				w_adapter->fsm_state = FSM_AMPDU_IND_SENT;
			}
			else
			{
				if (w_adapter->fsm_state == FSM_MAC_INIT_DONE)
				{	
					ONEBOX_DEBUG(ONEBOX_ZONE_OID, (TEXT("onebox_set_per_tx_mode: Enabling the PER burst mode\n")));
					if (w_adapter->os_intf_ops->onebox_init_thread(&(w_adapter->sdio_scheduler_thread_handle_per), 
                                                                   "PER THREAD", 
                                                                   0, 
                                                                   w_adapter->wlan_osd_ops->onebox_per_thread, 
                                                                   w_adapter) != 0)
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("onebox_set_per_tx_mode: Unable to initialize thread\n")));
						return -EFAULT;
					}
					w_adapter->os_intf_ops->onebox_start_thread(&(w_adapter->sdio_scheduler_thread_handle_per));
					w_adapter->tx_running = BURST_RUNNING;//indicating PER_BURST_MODE
				}
				else
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(
									"Driver is not in MAC_INIT_DONE STATE\n")));
					return -EFAULT;
				}	
			}
			return 0;
		}
	}

	else
	{
		if(w_adapter->tx_running == BURST_RUNNING)
		{
			onebox_send_per_frame(w_adapter, PER_BURST_MODE);
			w_adapter->wlan_osd_ops->onebox_kill_per_thread(w_adapter);
		}
		else if(w_adapter->tx_running == CONTINUOUS_RUNNING)
		{
			onebox_send_per_frame(w_adapter, PER_CONT_MODE);
		}
		else
		{
			return ONEBOX_STATUS_FAILURE;
		}
		w_adapter->tx_running = 0;
		if(w_adapter->netbuf_cb_per)
		{
//			w_adapter->os_intf_ops->onebox_free_pkt(w_adapter->netbuf_cb_per, 0);
			w_adapter->total_per_pkt_sent = 0;
			w_adapter->netbuf_cb_per = NULL;
		}
		else
		{
			return ONEBOX_STATUS_FAILURE;
		}
	}
	return status;
}

/**
 * This function is used to start the burst mode.
 *
 * @param Pointer to the w_adapter structure.
 * @return ONEBOX_STATUS_SUCCESS on success else ONEBOX_STATUS_FAILURE.
 */
ONEBOX_STATUS start_per_burst(WLAN_ADAPTER w_adapter)
{
	if (w_adapter->endpoint_params.enable)
	{
		if (do_continuous_send(w_adapter))
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("onebox_do_continuous_send failed\n")));
			return ONEBOX_STATUS_FAILURE;
		}
	}
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function transmits the prepared PER packet to the PPE.
 *
 * @param Pointer to the w_adapter structure.
 * return 0 on success else -1 on failure.
 */
int do_continuous_send(WLAN_ADAPTER w_adapter)
{
	int status;
	uint16 seq_num;
	uint8 device_buf_status = 0;
	uint8 extended_desc = 4;
	uint16 length;
	struct driver_assets *d_assets = w_adapter->d_assets;
	
	if(w_adapter->endpoint_params.aggr_enable == 1)
	{
		if(w_adapter->no_of_per_fragments != 1)
		{
			length = PER_AGGR_LIMIT_PER_PKT;
		}
		else
		{
			length = (w_adapter->endpoint_params.length - ((w_adapter->endpoint_params.aggr_count-1)*PER_AGGR_LIMIT_PER_PKT));
		}
                w_adapter->no_of_per_fragments--;

	}
	else
		length = (w_adapter->endpoint_params.length > 1536 ) ? 1536 : w_adapter->endpoint_params.length;
	if(1)//(!w_adapter->per_flag)
	{
		w_adapter->netbuf_cb_per = w_adapter->os_intf_ops->onebox_alloc_skb(length + FRAME_DESC_SZ + extended_desc - 4 /* CRC */);
		if (w_adapter->netbuf_cb_per == NULL) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("unable to allocate buffer for tx\n")));
			return ONEBOX_STATUS_FAILURE;
		}
		w_adapter->os_intf_ops->onebox_memset(w_adapter->netbuf_cb_per->data, 0, ((length + FRAME_DESC_SZ + extended_desc - 4 /* CRC */)));
		w_adapter->os_intf_ops->onebox_add_data_to_skb(w_adapter->netbuf_cb_per, (length + FRAME_DESC_SZ + extended_desc - 4 /* CRC */));
		prepare_per_pkt(w_adapter, w_adapter->netbuf_cb_per);
		w_adapter->per_flag = 1;
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)w_adapter->netbuf_cb_per->data, (FRAME_DESC_SZ + extended_desc));
	}

	status = d_assets->onebox_common_read_register(d_assets,
																								 w_adapter->buffer_status_register,
																								 &device_buf_status); /* Buffer */

	if (status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                     (TEXT("%s: Failed to Read Intr Status Register\n"),__func__));
		return ONEBOX_STATUS_FAILURE;
	}

	if(device_buf_status & (1 << SD_PKT_BUFF_FULL))
	{
		if(!w_adapter->buffer_full)
		{
			w_adapter->buf_full_counter++;
		}
		w_adapter->buffer_full = 1;
		ONEBOX_DEBUG(ONEBOX_ZONE_DATA_DUMP, (TEXT( "PER B\n")));
		return ONEBOX_STATUS_SUCCESS;
	}
	else
	{
		w_adapter->buffer_full = 0;

	}
	if(w_adapter->netbuf_cb_per->len > (FRAME_DESC_SZ + MIN_802_11_HDR_LEN + extended_desc)) 
	{
		seq_num = ONEBOX_CPU_TO_LE16(*(uint16 *)(&w_adapter->netbuf_cb_per->data[FRAME_DESC_SZ + MIN_802_11_HDR_LEN  + extended_desc - 2])) >> 4;
		seq_num = ((seq_num + 1) % 4096);

		w_adapter->netbuf_cb_per->data[FRAME_DESC_SZ + MIN_802_11_HDR_LEN + extended_desc - 2] = (seq_num << 4) & 0xff;
		w_adapter->netbuf_cb_per->data[FRAME_DESC_SZ + MIN_802_11_HDR_LEN + extended_desc - 1] = ((seq_num << 4) >> 8) ;
	}
	if(w_adapter->per_packet.enable)
		w_adapter->os_intf_ops->onebox_memcpy(&w_adapter->netbuf_cb_per->data[FRAME_DESC_SZ + extended_desc], &w_adapter->per_packet.packet[0], \
                                                   ((w_adapter->per_packet.length)>w_adapter->endpoint_params.length)?(w_adapter->endpoint_params.length):(w_adapter->per_packet.length));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)w_adapter->netbuf_cb_per->data,(w_adapter->netbuf_cb_per->len));

	w_adapter->netbuf_cb_per->tx_pkt_type = WLAN_TX_D_Q;
	w_adapter->netbuf_cb_per->data[1] |= BIT(7);/* Immediate Wakeup bit*/
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)w_adapter->netbuf_cb_per->data,(w_adapter->netbuf_cb_per->len));
	w_adapter->os_intf_ops->onebox_netbuf_queue_tail(&w_adapter->host_tx_queue[BE_Q_STA], w_adapter->netbuf_cb_per->pkt_addr);
	w_adapter->devdep_ops->onebox_schedule_pkt_for_tx(w_adapter);
#if 0
	status = onebox_send_internal_mgmt_frame(w_adapter, 
			(uint16 *)w_adapter->netbuf_cb_per->data, 
			(w_adapter->netbuf_cb_per->len));
#endif
	if (status != ONEBOX_STATUS_SUCCESS) 
	{ 
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Failed To Write The Packet\n"),__func__));
		return ONEBOX_STATUS_FAILURE;
	}
	w_adapter->total_per_pkt_sent++;
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function is used to prepare a packet for transmission.
 *
 * @param Pointer to the w_adapter structure.
 * @param Pointer to the netbuf control block structure.
 * @return ONEBOX_STATUS_SUCCESS.
 */
ONEBOX_STATUS prepare_per_pkt(WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb)
{
	uint32 *frame_desc,temp_word;
	struct ieee80211_frame *tmp_hdr = NULL;
	uint8  size_of_hdr; // frame_desc + MAC_HDR + Extended_desc   16 + 24 + 4
	uint8 intrnl_hdr;
	uint8 actual_len;
	/* For future use */
	uint8 extended_desc = 4;

	uint8 mac_addr[6] = {0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5};
	
	uint8 offset = 0, min_len = 0;
	uint32 rate_flags, greenfield, ch_bw, i = 0;  
	size_of_hdr = FRAME_DESC_SZ + extended_desc + MIN_802_11_HDR_LEN;

#if 1 // Below code is not required as data area of SKB has enough space already to hold FRAME_DESC_SZ and extended_desc.
	if (netbuf_cb->len < size_of_hdr) // frame_desc + MAC_HDR + Extended_desc   16 + 24 + 4
	{
		min_len = (netbuf_cb->len  - FRAME_DESC_SZ - extended_desc);
	}
#endif
	frame_desc = (uint32 *)&netbuf_cb->data[0];
	w_adapter->os_intf_ops->onebox_memset((uint8 *)frame_desc, 0, (FRAME_DESC_SZ + extended_desc));

	temp_word = (netbuf_cb->len - FRAME_DESC_SZ) & 0xffff;
	frame_desc[0] = ONEBOX_CPU_TO_LE32(temp_word);
	if (netbuf_cb->len >= (MIN_802_11_HDR_LEN + FRAME_DESC_SZ + extended_desc))
	{
		frame_desc[1] = ONEBOX_CPU_TO_LE32( MIN_802_11_HDR_LEN << 8);
	}
	else
	{
		frame_desc[1] = ONEBOX_CPU_TO_LE32( w_adapter->per_packet.length << 8);
	} 
	if((w_adapter->per_packet.enable && !w_adapter->per_packet.insert_seq) || !w_adapter->per_packet.enable)
		frame_desc[1] |= (BIT(2) << 16);  //insert seq_no
	if(w_adapter->endpoint_params.aggr_enable) 
	{
		temp_word = (( ENABLE_MAC_INFO) << 16); //In mac_info set bit0 and bit9 for bcast pkt
		frame_desc[3] |= ONEBOX_CPU_TO_LE32(QOS_EN); // Indicating of QOS in mac_flags
	}
	else
	temp_word = ((BROADCAST_IND | ENABLE_MAC_INFO) << 16); //In mac_info set bit0 and bit9 for bcast pkt
	rate_flags = (w_adapter->endpoint_params.rate_flags << 2);
	/*: Put macros for these mask values */
	ch_bw	= (rate_flags & 0x00F0);
	if( ch_bw & BIT(4) )
		ch_bw = 0;
	greenfield  = (rate_flags & 0x0008);
	greenfield  = (greenfield << 17);
	if (rate_flags & 0x0004)//checking short_GI
	{
		w_adapter->endpoint_params.rate |= BIT(9);
	}  
	if (w_adapter->endpoint_params.mode)
	{
		rate_flags |= CONTINUOUS_MODE;
	} 

  /* Rates are sent in word 5 and channel bandwidth in word 6
   * */
	frame_desc[1] |= ONEBOX_CPU_TO_LE32(temp_word | extended_desc);
	frame_desc[2] = ONEBOX_CPU_TO_LE32((w_adapter->endpoint_params.rate & 0x3ff) | (ch_bw  << 12) | greenfield);
	frame_desc[4] = ONEBOX_CPU_TO_LE32(w_adapter->endpoint_params.power ) & 0xff ;
	if (w_adapter->endpoint_params.mode == PER_CONT_MODE)
	{	
		frame_desc[4] |= ONEBOX_CPU_TO_LE32(3 << 8);
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" CONTINUOUS PER MODE STARTED\n")));
	}
	offset += FRAME_DESC_SZ + extended_desc;
	/* Form the Mac header for the PER Packet to be transmitted */
	tmp_hdr = (struct ieee80211_frame *)&netbuf_cb->data[offset];

	if (netbuf_cb->len >= size_of_hdr)
	{
		tmp_hdr->i_fc[0]=0x8;
		tmp_hdr->i_fc[1]=0x01;
		tmp_hdr->i_dur[0]=0x00;
		tmp_hdr->i_dur[1]=0x00;

		w_adapter->os_intf_ops->onebox_memcpy(tmp_hdr->i_addr1, mac_addr, MAC_ADDR_LEN);
		w_adapter->os_intf_ops->onebox_memcpy(tmp_hdr->i_addr2, w_adapter->mac_addr, MAC_ADDR_LEN);
		w_adapter->os_intf_ops->onebox_memcpy(tmp_hdr->i_addr3, w_adapter->mac_addr, MAC_ADDR_LEN);

		offset += MIN_802_11_HDR_LEN;

		/*Fill the PER packet with dummy data */
		//w_adapter->os_intf_ops->onebox_memset(&netbuf_cb->data[offset], 0xdd, (netbuf_cb->len - offset));
		for (i = offset; i <= (netbuf_cb->len); i += 4)
		{
			netbuf_cb->data[i] = 0xff;
			netbuf_cb->data[i + 1] = 0x00;
			netbuf_cb->data[i + 2] = 0xbb;
			netbuf_cb->data[i + 3] = 0x55;
		}
		if (((netbuf_cb->len - offset)%4))
			w_adapter->os_intf_ops->onebox_memset(&netbuf_cb->data[i -4], 0xdd, ((netbuf_cb->len - offset)%4));
	}
	else
	{
		intrnl_hdr = netbuf_cb->len + extended_desc; // frame_decriptor_size  + extended_descriptor_size
		actual_len = intrnl_hdr + w_adapter->per_packet.length; //actual len = intrnl_hdr + len of per pkt from transmit.c
		for (i = offset; i<= actual_len; i++){
		netbuf_cb->data[i] = 0xff;
		}
	}
	netbuf_cb->data[1] |= (ONEBOX_WIFI_DATA_Q << 4); 
	if(w_adapter->endpoint_params.aggr_enable) /*  why if and else cases are same */
		netbuf_cb->data[14] |= 1;// send pkts in queue no: 3
	else
		netbuf_cb->data[14] |= 1;// send pkts in queue no: 5 
	return ONEBOX_STATUS_SUCCESS;
}


/**
 * This function is used to send AMPDU Indication frame in PER mode.
 *
 * @param Pointer to the w_adapter structure.
 * @return ONEBOX_STATUS_SUCCESS on success else ONEBOX_STATUS_FAILURE.
 */
ONEBOX_STATUS send_per_ampdu_indiaction_frame(WLAN_ADAPTER w_adapter)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[FRAME_DESC_SZ];

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("===> Frame to Send AMPDU IndiCATIONframe in PER <===\n")));

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

	/* FrameType*/
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((sizeof(mgmt_frame->u.ampdu_ind)) | ONEBOX_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(AMPDU_IND); 
	//mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(ni->hal_priv_node.tid[tidno].seq_start); 
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(0); //tid
	mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(w_adapter->endpoint_params.aggr_count); //baw_size 
	mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(0 | (START_AMPDU_AGGR << 4)| ( 0 << 8)); 

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(
					" ampdu indication frame sent in %s \n"), __func__));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, FRAME_DESC_SZ);
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (FRAME_DESC_SZ));
	return status;


}


/**
 * This function is used to send PER frame for Burst/Continuous mode.
 *
 * @param Pointer to the w_adapter structure.
 * @return ONEBOX_STATUS_SUCCESS on success else ONEBOX_STATUS_FAILURE.
 */
ONEBOX_STATUS onebox_send_per_frame(WLAN_ADAPTER w_adapter,uint8 mode)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("===> Frame to Send PER Frame <===\n")));
	
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

	/* FrameType*/
	mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12 | sizeof(struct per_params_s));
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(PER_CMD_PKT);
	mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(mode);
	memcpy(&pkt_buffer[16],(uint8 *)(&w_adapter->endpoint_params),sizeof(per_params));

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(
					 " Sending PER frame \n")));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)mgmt_frame, FRAME_DESC_SZ);
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (FRAME_DESC_SZ + sizeof(per_params)));
	return status;
}

ONEBOX_STATUS onebox_send_w_9116_features(WLAN_ADAPTER w_adapter)
{
	onebox_mac_frame_t *mgmt_frame;
  ONEBOX_STATUS status;
  features_enable_t features_enable_frame;
  uint8 len = (FRAME_DESC_SZ + sizeof(features_enable_t));
	uint8  pkt_buffer[len];
  struct driver_assets *d_assets = w_adapter->d_assets;
	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, len);
	w_adapter->os_intf_ops->onebox_memset(&features_enable_frame, 0, sizeof(features_enable_t));
  features_enable_frame.pll_mode =  d_assets->w_9116_features.pll_mode;
  features_enable_frame.rf_type = d_assets->w_9116_features.rf_type;
  features_enable_frame.wireless_mode =d_assets->w_9116_features.wireless_mode;
  features_enable_frame.enable_ppp = d_assets->w_9116_features.enable_ppe;
  features_enable_frame.afe_type =d_assets->w_9116_features.afe_type;
    if(d_assets->w_9116_features.dpd) {
    features_enable_frame.feature_enable |= DPD;
    }
    if(d_assets->w_9116_features.SIFSTransmitenable) {
    features_enable_frame.feature_enable |= SIFSTRANSMITENABLE;
    }
    if(d_assets->w_9116_features.pwrsave_options & DUTY_CYCLING) {
    features_enable_frame.feature_enable |= DUTY_CYCLING;
    }
    if(d_assets->w_9116_features.pwrsave_options & END_OF_FRAME) {
    features_enable_frame.feature_enable |= END_OF_FRAME;
    }
    features_enable_frame.feature_enable |= d_assets->wlan_pwrsave_options;
    if(features_enable_frame.feature_enable & LMAC_BEACON_DROP) {
      features_enable_frame.feature_enable |= DROP_BYTES_FEATURE;
      //features_enable_frame.feature_enable |= PPE_DMEM_WRITE_FEATURE;//WILL BE USED LATER
      features_enable_frame.feature_enable |= (d_assets->lmac_bcon_en_dis_threshold << 28);
    }
	mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12 | sizeof(features_enable_t));
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(FEATURES_ENABLE);
	memcpy(&pkt_buffer[16],(uint8 *)(&features_enable_frame),sizeof(features_enable_t));
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame,len);
	return status;
}
EXPORT_SYMBOL (onebox_send_w_9116_features);
ONEBOX_STATUS onebox_send_structure_prog_stats_request(WLAN_ADAPTER w_adapter, programming_stats_t *programming_stats)
{
	netbuf_ctrl_block_t *netbuf_cb = NULL;
  onebox_mac_frame_t *mgmt_frame;
  ONEBOX_STATUS status;
  uint8 pkt_len = FRAME_DESC_SZ;
  FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);
	netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(pkt_len);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;
	}
	w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);
  mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
  w_adapter->os_intf_ops->onebox_memset(mgmt_frame,0x0,pkt_len);
  mgmt_frame->desc_word[0] = (COEX_TX_Q << 12 );
  mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(LOG_STRUCT_PRGMG_D);
  mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16( programming_stats->start_stop << 8);
  mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE32( programming_stats->interval);
	netbuf_cb->tx_pkt_type = COEX_Q;
    memcpy(&netbuf_cb->data[16],(uint8 *)(programming_stats),sizeof(programming_stats_t));
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("<======SENDING STRUCUTE LOGGING REQ ========>\n")));
    w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, pkt_len);
	 status = w_adapter->d_assets->common_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, COEX_Q);
  return status;
}
EXPORT_SYMBOL (onebox_send_structure_prog_stats_request);
ONEBOX_STATUS onebox_send_disable_programming(WLAN_ADAPTER w_adapter,unsigned char buffer)
{
	netbuf_ctrl_block_t *netbuf_cb = NULL;
  onebox_mac_frame_t *mgmt_frame;
  ONEBOX_STATUS status;
  FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);
	netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(FRAME_DESC_SZ);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;
	}
	w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, FRAME_DESC_SZ);
  mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
  w_adapter->os_intf_ops->onebox_memset(mgmt_frame,0x0,FRAME_DESC_SZ);
  mgmt_frame->desc_word[0] = (COEX_TX_Q << 12 );
  mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(DISABLE_PROGRAMMING_D);
  mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(buffer << 8);
	netbuf_cb->tx_pkt_type = COEX_Q;
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("<======SENDING DISABLE_PROGRAMMING REQ ========>\n")));
  w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, FRAME_DESC_SZ);
	 status = w_adapter->d_assets->common_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, COEX_Q);
  return status;
}
EXPORT_SYMBOL (onebox_send_disable_programming);
ONEBOX_STATUS onebox_send_programming_structs(WLAN_ADAPTER w_adapter,prog_structure_t* prog_structure_p)
{
	netbuf_ctrl_block_t *netbuf_cb = NULL;
  onebox_mac_frame_t *mgmt_frame;
  ONEBOX_STATUS status;
  uint len;
  //uint8  pkt_buffer[pkt_len];
  FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

  len = (FRAME_DESC_SZ + prog_structure_p->len);

	netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(len );
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;

	}

	w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);

  mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
  w_adapter->os_intf_ops->onebox_memset(mgmt_frame,0x0,len);

  /* FrameType*/
  mgmt_frame->desc_word[0] = (COEX_TX_Q << 12 |prog_structure_p->len);
  mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(PROG_STRUCTURE_D);
  mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(prog_structure_p->prog_type << 8);
  mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(prog_structure_p->structure_present);
  *((uint32*)(&(mgmt_frame->desc_word[4]))) = ONEBOX_CPU_TO_LE32(prog_structure_p->TA_RAM_ADDRESS);
  mgmt_frame->desc_word[6] = ONEBOX_CPU_TO_LE32(prog_structure_p->bb_rf_flags);

	netbuf_cb->tx_pkt_type = COEX_Q;

	memcpy(&netbuf_cb->data[16],(uint8 *)(prog_structure_p) + FRAME_DESC_SZ ,prog_structure_p->len);

		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("<======SENDING PROGRAMMING STRUCT'S REQ ========>\n")));
  w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, len );
  //status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame,pkt_len);
	 status = w_adapter->d_assets->common_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, COEX_Q);
  return status;
}
EXPORT_SYMBOL (onebox_send_programming_structs);
ONEBOX_STATUS onebox_send_wlan_iq_capture_request(WLAN_ADAPTER w_adapter)
{
	onebox_mac_frame_t *mgmt_frame;
  ONEBOX_STATUS status;
	uint8  pkt_buffer[FRAME_DESC_SZ];
	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("===> WLAN IQ Capture Req Frame <===\n")));
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);
	mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12 );
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(WLAN_IQ_STATS_REQ);
  mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(w_adapter->wlan_iqs_stats.no_of_samples);
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)mgmt_frame, FRAME_DESC_SZ);
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame,FRAME_DESC_SZ);
	return status;
}
EXPORT_SYMBOL (onebox_send_wlan_iq_capture_request);
