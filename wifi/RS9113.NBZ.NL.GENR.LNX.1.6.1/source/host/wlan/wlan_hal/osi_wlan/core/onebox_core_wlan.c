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

ONEBOX_STATUS onebox_mgmt_send_bb_reset_req(WLAN_ADAPTER w_adapter);

/** This function is used to determine the wmm queue based on the backoff procedure.
 * Data packets are dequeued from the selected hal queue and sent to the below layers.
 * @param  pointer to the driver private data structure
 * @return void
 */
void core_qos_processor(WLAN_ADAPTER w_adapter)
{
	netbuf_ctrl_block_t *netbuf_cb;
	uint8 q_num;
	uint8 counter = 0;
	uint8 device_buf_status = 0;
	uint8 status;
	struct ieee80211com *ic;
	struct ieee80211vap *vap = NULL;
	uint32 vap_id = 0;
	uint32 sta_id = 0;
	struct ieee80211_node *ni = NULL;
	struct driver_assets *d_assets = w_adapter->d_assets;
  uint64 spin_lock_flags; 

	ic = &w_adapter->vap_com;
	
  //ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Liien %d %p adap %p\n", __func__, __LINE__, w_adapter, d_assets->techs[WLAN_ID].priv)));
	if(!d_assets->techs[WLAN_ID].tx_access) {
	    //check whther we have tx_access or not
	    d_assets->techs[WLAN_ID].tx_intention = 1;
	    d_assets->update_tx_status(d_assets, WLAN_ID);
	    if(!d_assets->techs[WLAN_ID].tx_access) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Waiting for tx_acces from common hal cmntx %d\n"), d_assets->common_hal_tx_access));
		d_assets->techs[WLAN_ID].deregister_flags = 1;
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Waiting evbent %s Line %d\n"), __func__, __LINE__));
		if (wait_event_timeout((d_assets->techs[WLAN_ID].deregister_event), (d_assets->techs[WLAN_ID].deregister_flags == 0), msecs_to_jiffies(6000) )) {
			if(!d_assets->techs[WLAN_ID].tx_access) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d unable to get access \n"), __func__, __LINE__));
				return;
			}
		} else {
		    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERR: In %s Line %d Initialization of WLAN Failed as Wlan TX access is not granted from Common Hal \n"), __func__, __LINE__));
		    return;
		}
	  }
	}

	while (1) 
	{
		if(w_adapter->beacon_event) /* Check for beacon event set and break from here */
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			             (TEXT("CORE_MSG: BEACON EVENT HAS HIGHEST PRIORITY\n")));
			break;
		}
		if(w_adapter->fsm_state == FSM_MAC_INIT_DONE)
		{
			TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
			{ 
				if(vap->iv_opmode == IEEE80211_M_STA){
					break;
				}
			}
			if(vap && (PS_STATE == PS_EN_REQ_SENT) && (vap->iv_state != IEEE80211_S_RUN))
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Breaking the schedule has  PS is disabled vap_state %d PS_STATE %d\n"), vap->iv_state, PS_STATE));
				break;	
			}
		}

		q_num = core_determine_hal_queue(w_adapter);
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		             (TEXT("\nCORE_MSG: qos_processor: Queue number = %d\n"), q_num));

		if (q_num == INVALID_QUEUE) 
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\nCORE_MSG: qos_pro: No More Pkt Due to invalid queueno\n")));
			break;
		}
		if(w_adapter->sta_data_block && (q_num < NO_STA_DATA_QUEUES))
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("CORE_MSG: Data block is set, So don't dequeue pkts\n")));
			break;
		}

		if (w_adapter->block_ap_queues && 
		    (((q_num > NO_STA_DATA_QUEUES) 
				  && (q_num < MGMT_SOFT_Q)) || (q_num == BEACON_HW_Q))
				) {
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("CORE_MSG: AP Data block is set, So don't dequeue pkts\n")));
				break;
		}

		if(d_assets->host_intf_type == HOST_INTF_SDIO) {

		if(w_adapter->buf_status_updated ||(!counter))	
		{ /* Polling buffer full status for every 4 packets */
                        /* Shouldn't change this place of updation */
			w_adapter->buf_status_updated = 0;
			status = d_assets->onebox_common_read_register(d_assets,
																										 w_adapter->buffer_status_register,
																										 &device_buf_status); /* Buffer */
	
			if (status != ONEBOX_STATUS_SUCCESS)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			             (TEXT("%s: Failed to Read Intr Status Register\n"),__func__));
				break;
			}
			if(device_buf_status & (ONEBOX_BIT(SD_PKT_MGMT_BUFF_FULL)))
			{
				if(!w_adapter->mgmt_buffer_full)
				{
			 		w_adapter->mgmt_buf_full_counter++;
				}
				w_adapter->mgmt_buffer_full = 1;
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("CORE_MSG: MGMT BUFFER FULL Queueing data packets till BUFFER_FREE COMES\n")));
				break;
			}
			else
			{
				w_adapter->mgmt_buffer_full = 0;
			}

			if((!(device_buf_status & (ONEBOX_BIT(SD_PKT_BUFF_FULL)))) && 
				(!(device_buf_status & (ONEBOX_BIT(SD_PKT_BUFF_SEMI_FULL))))) {

				w_adapter->no_buffer_fulls++;
			}
			if(device_buf_status & (ONEBOX_BIT(SD_PKT_BUFF_FULL)))
			{
				if(!w_adapter->buffer_full)
				{
			 		w_adapter->buf_full_counter++;
				}
				w_adapter->buffer_full = 1;
				//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("CORE_MSG: BUFFER FULL Queueing data packets till BUFFER_FREE COMES\n")));
			}
			else
			{
				w_adapter->buffer_full = 0;
			}

			if(device_buf_status & (ONEBOX_BIT(SD_PKT_BUFF_SEMI_FULL)))
			{
				if(!w_adapter->semi_buffer_full)
				{
			 		w_adapter->buf_semi_full_counter++;
				}
				w_adapter->semi_buffer_full = 1;
			}
			else
			{
				w_adapter->semi_buffer_full = 0;
			}

			((w_adapter->semi_buffer_full == 0) ? (counter = 4) : (counter = 1));
		}

		if((q_num == MGMT_SOFT_Q) && w_adapter->mgmt_buffer_full) /* Check mgmtbuffer full for mgmt packets */
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("CORE_MSG: MGMT BUFFER FULL Queueing data packets till BUFFER_FREE COMES\n")));
			break;
		}
		else if((w_adapter->buffer_full) && (q_num != MGMT_SOFT_Q))
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("CORE_MSG: BUFFER FULL Queueing data packets till BUFFER_FREE COMES\n")));
			break;
		}

		if(w_adapter->fsm_state == FSM_MAC_INIT_DONE)
		{
			TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
			{ 
				if((vap->iv_opmode == IEEE80211_M_STA) 
								&& (((PS_STATE == DEV_IN_PWR_SVE) && (vap->iv_state != IEEE80211_S_RUN)))) {
								//|| ((driver_ps.delay_pwr_sve_decision_flag) && ((vap->iv_state != IEEE80211_S_RUN)))){
					update_pwr_save_status(vap, PS_DISABLE, MGMT_PENDING_PATH);
					break;
				}
			}
		}
		/* Removing A Packet From The netbuf Queue */
		netbuf_cb = core_dequeue_pkt(w_adapter, q_num);
		counter--;
		} else {
#if 0
    if(atomic_read(&d_assets->techs[WLAN_ID].pkt_write_pending)) {
      break;
    }
#endif
    counter = d_assets->update_usb_buf_status(d_assets->global_priv);
    if(counter >= 15) {
      counter = 0;
      w_adapter->os_intf_ops->onebox_acquire_spinlock(&d_assets->usb_wlan_coex_write_pending, &spin_lock_flags);
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("CORE_MSG: BUFFER FULL Queueing data packets till BUFFER_FREE COMES\n")));
      atomic_set(&d_assets->techs[WLAN_ID].pkt_write_pending, 1);
      w_adapter->os_intf_ops->onebox_release_spinlock(&d_assets->usb_wlan_coex_write_pending, spin_lock_flags);
      break;
    }

		if(w_adapter->fsm_state == FSM_MAC_INIT_DONE)
		{
			TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
			{ 
				if((vap->iv_opmode == IEEE80211_M_STA) 
								&& (((PS_STATE == DEV_IN_PWR_SVE) && (vap->iv_state != IEEE80211_S_RUN)))) {
								//|| ((driver_ps.delay_pwr_sve_decision_flag) && ((vap->iv_state != IEEE80211_S_RUN)))){
					update_pwr_save_status(vap, PS_DISABLE, MGMT_PENDING_PATH);
					break;
				}
			}
		}
    /* Removing A Packet From The netbuf Queue */
    netbuf_cb = core_dequeue_pkt(w_adapter, q_num);
		}

		if (netbuf_cb == NULL) 
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\nCORE_MSG:NETBUF is NULL qnum = %d\n"),q_num));
			break;
		}
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("CORE_MSG: qos_pro: Sending Packet To HAL\n")));

    if(q_num == MGMT_SOFT_Q)
    {
      if(netbuf_cb->flags & INTERNAL_MGMT_PKT)
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Internal mgmt pkt dump\n")));
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("CORE_MSG: Mgmt PKT calling on air mgmt\n"))); 
        //w_adapter->devdep_ops->onebox_snd_mgmt_pkt(w_adapter,netbuf_cb);
        onebox_internal_pkt_dump(w_adapter, netbuf_cb);	

        if(netbuf_cb->data[2] == WAKEUP_SLEEP_REQUEST) 
        {
          TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
          { 
            if(vap->iv_opmode == IEEE80211_M_STA){
              break;
            }
          }
          if( (netbuf_cb->data[16] == PS_DISABLE)&& (netbuf_cb->data[18] == DEEP_SLEEP))
          {
          }
          else if(vap && (PS_STATE == PS_EN_REQ_QUEUED))
          {
            PS_STATE = PS_EN_REQ_SENT;
            ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (TEXT("In %s line %d mvng pwr save state to %d\n"), __func__, __LINE__, PS_STATE));
          }
        }
	
	/* Observed Rx done handling happening before PS State being changed.
         * Hence writing the packet after state is changed in USB case
	 */
		status = w_adapter->onebox_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, WLAN_Q);
	if(d_assets->host_intf_type == HOST_INTF_SDIO)
		w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
        continue;
      }
    }
		
		if(q_num < MAX_HW_QUEUES)
						w_adapter->total_tx_data_sent[q_num]++;

		if(q_num == MGMT_SOFT_Q)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("CORE_MSG: Mgmt PKT\n"))); 
			//w_adapter->devdep_ops->onebox_snd_mgmt_pkt(w_adapter,netbuf_cb);
			onair_mgmt_dump(w_adapter, netbuf_cb, netbuf_cb->data[4]);
			status = w_adapter->onebox_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, WLAN_Q);
				if(d_assets->host_intf_type == HOST_INTF_SDIO)
			w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
		} 
		else
		{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("CORE_MSG: Data PKT \n")));
				vap_id = netbuf_cb->data[9] >> 6;
				sta_id = netbuf_cb->data[15];
				ni = (struct ieee80211_node *)netbuf_cb->ni;

				w_adapter->os_intf_ops->onebox_acquire_sem(&w_adapter->ic_lock_vap, 0);
				if (!w_adapter->hal_vap[vap_id].vap_in_use) {
					if (w_adapter->Driver_Mode == 2/*PER MODE*/ ) {
						status = w_adapter->onebox_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, WLAN_Q);
						w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);
						continue;
					}
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("CORE_MSG:Dropping pkts as vap is deleted vap id %d \n"), vap_id));
					w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
					w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);
					continue;
				}

				vap = ni->ni_vap;
				/* Has to be taken care for multiple vaps case in a different way */
				if(!vap){
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to find VAP node\n")));
						w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
						w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);
						continue;

				}

				if ((w_adapter->hal_vap[vap_id].vap  == vap) && 
				    (vap->hal_priv_vap)) {
						if ((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[q_num])) < (w_adapter->host_txq_maxlen[q_num] - 500)) {
#ifndef USE_SUBQUEUES	
								if(w_adapter->os_intf_ops->onebox_is_ifp_txq_stopped(vap->iv_ifp))
#else
													if(w_adapter->os_intf_ops->onebox_is_sub_txq_stopped(vap->iv_ifp, netbuf_cb->skb_priority))
#endif
													{
#ifndef USE_SUBQUEUES	
																			w_adapter->os_intf_ops->onebox_start_ifp_txq(vap->iv_ifp);
#else
																			w_adapter->os_intf_ops->onebox_start_sub_txq(vap->iv_ifp, netbuf_cb->skb_priority);
#endif
										}
								vap->hal_priv_vap->stop_udp_pkts[q_num] = 0;
								vap->hal_priv_vap->stop_per_q[q_num] = 0;
								}
				}	else {
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid VAP Ptr, We are seeing pkts on old vap ptr\n")));
				}

				if(!((netbuf_cb->flags & ONEBOX_BROADCAST_PKT) && (vap->iv_opmode == IEEE80211_M_HOSTAP)))
				{
						if(sta_id >= w_adapter->max_stations_supported) {
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("CORE_MSG:Dropping pkts as sta_id is greater than max supported stations%d \n"), sta_id));
						w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
						w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);
						continue;
						
						}

						if(!(w_adapter->sta_connected_bitmap[sta_id/32] & BIT(sta_id % 32))) /* each bitmap is uint32 so diving by 32*/
						{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("CORE_MSG:Dropping pkts as station is deleted sta id %d \n"), sta_id));
								w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
								w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);
								continue;
						}
				}
				w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);

				status = w_adapter->onebox_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, WLAN_Q);
        /* In USB mode pkt gets freed in done handler */
		if(d_assets->host_intf_type == HOST_INTF_SDIO) 
				w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
		}
	}

	//if(vap && (vap->iv_opmode == IEEE80211_M_STA) && (ps_params_def.ps_en)) {
	if(vap && (vap->iv_opmode == IEEE80211_M_STA) && (PS_STATE == DEV_IN_PWR_SVE)) {

		d_assets->techs[WLAN_ID].tx_intention = 0;
		d_assets->techs[WLAN_ID].tx_access = 0;
		d_assets->update_tx_status(d_assets, WLAN_ID);
	}

	return;
}

/**
 * This function indicates the received packet to net80211.
 * @param  Pointer to the w_adapter structure
 * @param  pointer to the netbuf control block structure.
 * @return ONEBOX_STATUS_SUCCESS on success else negative number on failure.
 */ 
uint32 wlan_core_pkt_recv(WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb, int8 rs_rssi, int8 chno)
{
	int16 nf=0, status,rs_tstamp=0;
	struct ieee80211_node *ni;
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211vap *vap =NULL;
	netbuf_ctrl_block_m_t *netbuf_cb_m;
	uint8_t tid;
	/*
	 * No key index or no entry, do a lookup and
	 * add the node to the mapping table if possible.
	 * 
	 */
	ni = w_adapter->net80211_ops->onebox_find_rxnode(ic,
	                                               (const struct ieee80211_frame_min *)netbuf_cb->data);
	netbuf_cb_m = onebox_translate_netbuf_to_mbuf(netbuf_cb);

	if (netbuf_cb_m == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to alloc mem %s %d\n"), __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}

	if (ni != NULL) 
	{
		vap = ni->ni_vap;

		/* set the M_AMPDU FLAG if addba setup is done for receiving tid for now.
		 *  Ideally this should be taken from PPE */
		if(ni->ni_flags & IEEE80211_NODE_HT)
		{
			tid = ieee80211_gettid((const struct ieee80211_frame *)netbuf_cb_m->m_data);
			if (tid < WME_NUM_TID)
			{
				if (ni->ni_rx_ampdu[tid].rxa_flags & IEEE80211_AGGR_RUNNING)
				{
					netbuf_cb_m->m_flags |= M_AMPDU;
				}
			}
		}
		ni->hal_priv_node.chno = chno;
		ni->ni_avgrssi = rs_rssi;
		//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("SCAN: In %s Line %d chno = %02x \n", __func__, __LINE__, ni->hal_priv_node.chno)));
		status = vap->iv_input(ni, netbuf_cb_m, rs_rssi, nf);	
		/*
		 * If the station has a key cache slot assigned
		 * update the key->node mapping table.
		 */
	}
	else
	{
		status = w_adapter->net80211_ops->onebox_input_all(ic, netbuf_cb_m, rs_rssi, rs_tstamp, chno);
	}

	if(!status)
	{
		return ONEBOX_STATUS_SUCCESS;
	}
	else
	{
		return ONEBOX_STATUS_FAILURE;
	}
}

#ifdef BYPASS_RX_DATA_PATH
uint8 bypass_data_pkt(WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb)
{
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211vap *vap =NULL;
	
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
	{
		netbuf_cb->dev = vap->iv_ifp;
		w_adapter->os_intf_ops->onebox_indicate_pkt_to_os(vap->iv_ifp, netbuf_cb);
	}
	return 0;
}
#endif
/**
 * This function sends the beacon packet.
 *
 * @param  Pointer to ieee80211vap structure.
 * @param  pointer to the netbuf control block structure.
 * @return ONEBOX_STATUS_SUCCESS on success else negative number on failure.
 */
uint32 core_send_beacon(WLAN_ADAPTER w_adapter,netbuf_ctrl_block_t *netbuf_cb, struct core_vap *core_vp)
{
	uint16 i;
	uint8 dtim_beacon = 0;

	/* Drop Zero Length Beacon */
	if (!(netbuf_cb->len)) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\nCORE_MSG: ### Zero Length Beacon ###")));
		return ONEBOX_STATUS_FAILURE;
	}

	/* Drop if FSM state is not open */
	if (w_adapter->fsm_state != FSM_MAC_INIT_DONE) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\nCORE_MSG: ### FSM state is not open ###\n")));
		return ONEBOX_STATUS_FAILURE;
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("CORE_MSG:===> Sending beacon to packet processor <===\n")));    
	if (1) 
	{
#define BEACON_PARSING_OFFSET  36
		/* Parsing the beacon content to identify dtim beacons */
		for( i = BEACON_PARSING_OFFSET; i < netbuf_cb->len ;) /* start beacon parsing from first element */ 
		/*Fixed elements are of length 12 and mac header length is 24 */
		{
			if(netbuf_cb->data[i] == IEEE80211_ELEMID_TIM) /* Checking for TIM Element Value is 0x5*/
			{
				if(netbuf_cb->data[i+2] == 0) /*tim count field */
				{
					dtim_beacon = 1;
				}
				break;
			}
			i += (netbuf_cb->data[i+1] + 2);
		}
		w_adapter->devdep_ops->onebox_send_beacon(w_adapter, netbuf_cb, core_vp, dtim_beacon);
	}
	else 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("\nCORE_MSG: Beacon HAL queue is full\n")));
		return ONEBOX_STATUS_FAILURE;
	}

	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function gives the packet to the radiotap module.
 *
 * @param  Pointer to ieee80211vap structure.
 * @param  Pointer to the netbuf control block structure.
 * @return none. 
 */
void core_radiotap_tx(WLAN_ADAPTER w_adapter, struct ieee80211vap *vap, netbuf_ctrl_block_t *netbuf_cb)
{
	netbuf_ctrl_block_m_t *netbuf_cb_m = NULL;
	netbuf_cb_m = onebox_translate_netbuf_to_mbuf(netbuf_cb);
	if (netbuf_cb_m == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to alloc mem %s %d\n"), __func__, __LINE__));
		return;
	}
	w_adapter->net80211_ops->onebox_radiotap_tx(vap,netbuf_cb_m);
	w_adapter->os_intf_ops->onebox_mem_free(netbuf_cb_m);
	return;
}

/**
 * This routine dump the given data through the debugger..
 *
 * @param  Debug zone.  
 * @param  Pointer to data that has to dump.  
 * @param  Length of the data to dump.  
 * @return none. 
 */
VOID onebox_print_dump(int32 zone,UCHAR *vdata, int32 len )
{
	uint16 ii;

	if(!zone)
	{
		return;
	}

	for(ii=0; ii< len; ii++)
	{
		if(!(ii % 16))
		{
			ONEBOX_DEBUG(zone, (TEXT("\n%04d: "), ii));
		}
		ONEBOX_DEBUG(zone,(TEXT("%02x "),(vdata[ii])));
	}
	ONEBOX_DEBUG(zone, (TEXT("\n")));
}

/**
 * This routine indicates mic failure to net80211..
 *
 * @param  Pointer to the driver private structure.  
 * @param  pointer to the packet indicating mic failure.  
 * @return none. 
 */

void core_michael_failure(WLAN_ADAPTER w_adapter, uint8 *msg)
{
	struct ieee80211_node *ni = NULL;
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211vap *vap =NULL;
	struct ieee80211_frame frame;
	u_int keyix;
	uint8 status_word;

	frame.i_fc[0] = 0;
	frame.i_fc[1] = 0;
	w_adapter->os_intf_ops->onebox_memcpy(frame.i_addr2, (msg + 6), IEEE80211_ADDR_LEN);
	/**/
	status_word = msg[12];
	keyix = 1;
	ni = w_adapter->net80211_ops->onebox_find_rxnode(ic, (struct ieee80211_frame_min *)&frame);
	if(!(ni)){
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s line %d: Node is null \n"), __func__, __LINE__));
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, msg, 16);
		return;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("The mac addres of the node is %02x %02x %02x %02x %02x %02x\n"), ni->ni_macaddr[0], ni->ni_macaddr[1],
			ni->ni_macaddr[2],ni->ni_macaddr[3],ni->ni_macaddr[4],ni->ni_macaddr[5]));
	vap = ni->ni_vap;
	if(status_word & CIPHER_MISMATCH) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Received Cipher Mismatch Ind\n")));
	}else if(status_word & ICV_ERROR) {
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("Received ICV Failure Indication\n")));
	} else if((status_word & MIC_FAILURE) || (status_word & BCAST_MIC)) {
		if( vap->hal_priv_vap->cip_type == IEEE80211_CIPHER_TKIP ) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Received MIC Failure Indication\n")));
			w_adapter->net80211_ops->onebox_rxmic_failure(vap, (struct ieee80211_frame *)&frame, keyix);
		}
	}

}

