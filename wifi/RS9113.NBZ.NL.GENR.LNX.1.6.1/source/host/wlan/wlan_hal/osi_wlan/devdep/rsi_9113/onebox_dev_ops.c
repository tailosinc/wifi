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

/* include files */
#include "wlan_common.h"
#include "onebox_hal.h"
#include "onebox_linux.h"
#include "onebox_wlan_pktpro.h"

/**
 * This function prepares the netbuf control block
 *
 * @param 
 * w_adapter pointer to the driver private structure
 * @param 
 * buffer pointer to the packet data
 * @param 
 * len length of the packet 
 * @return .This function returns ONEBOX_STATUS_SUCCESS.
 */

#if 0
static netbuf_ctrl_block_t * prepare_netbuf_cb(WLAN_ADAPTER w_adapter, uint8 *buffer,
                      uint32 pkt_len, uint8 extended_desc)
{
	netbuf_ctrl_block_t *netbuf_cb  = NULL;
	uint8 payload_offset;
	pkt_len -= extended_desc;
#ifdef BYPASS_RX_DATA_PATH
	netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(pkt_len + FRAME_DESC_SZ);
#else
	netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(pkt_len);
#endif
	payload_offset = extended_desc + FRAME_DESC_SZ;
	if(netbuf_cb == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("@@Error while allocating skb in %s:\n"), __func__));
		return NULL;
	}
	/* Preparing The Skb To Indicate To core */
	netbuf_cb->dev = w_adapter->dev;
	w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);
	//(*(uint16 *)(netbuf_cb->data)) = (*((uint16 *)buffer));
#ifdef BYPASS_RX_DATA_PATH
	netbuf_cb->len = pkt_len + FRAME_DESC_SZ;
	w_adapter->os_intf_ops->onebox_memcpy((netbuf_cb->data), (buffer), FRAME_DESC_SZ);
	w_adapter->os_intf_ops->onebox_memcpy((netbuf_cb->data) + FRAME_DESC_SZ, (buffer + payload_offset), netbuf_cb->len - FRAME_DESC_SZ);
#else
	netbuf_cb->len = pkt_len;
	w_adapter->os_intf_ops->onebox_memcpy((netbuf_cb->data), (buffer + payload_offset), netbuf_cb->len);
#endif
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXTtk("Pkt to be indicated to Net80211 layer is len =%d\n", netbuf_cb->len)));
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, netbuf_cb->data, netbuf_cb->len);
	return netbuf_cb;
}
#endif

/**
 * This function read frames from the SD card.
 *
 * @param  Pointer to driver w_adapter structure.  
 * @param  Pointer to received packet.  
 * @param  Pointer to length of the received packet.  
 * @return 0 if success else -1. 
 */
ONEBOX_STATUS wlan_read_pkt(WLAN_ADAPTER w_adapter, 
				netbuf_ctrl_block_t *netbuf_cb)
{
	uint32 queueno;
	uint8 extended_desc;
//	uint32 pkt_len;
	uint8 *frame_desc_addr = netbuf_cb->data;
//	uint8 *frame_desc_addr = msg;
	uint32 length = 0;
	uint16 offset =0;
#ifndef BYPASS_RX_DATA_PATH
	//netbuf_ctrl_block_t *netbuf_cb = NULL;
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211vap *vap = NULL;
	uint8 vap_id;
#endif

	FUNCTION_ENTRY(ONEBOX_ZONE_DATA_RCV);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("w_adapter = %p\n"),
	             w_adapter));
	queueno = (uint32)((ONEBOX_CPU_TO_LE16(*(uint16 *)&frame_desc_addr[offset]) & 0x7000) >> 12);
	length   = (ONEBOX_CPU_TO_LE16(*(uint16 *)&frame_desc_addr[offset]) & 0x0fff);
	extended_desc   = (*(uint8 *)&frame_desc_addr[offset + 4] & 0x00ff);
	ONEBOX_DEBUG(ONEBOX_ZONE_DATA_RCV, (TEXT("###Received in QNumber:%d Len=%d###!!!\n"), queueno, length));
	w_adapter->stats.rx_packets++;	
	switch(queueno)
	{
		case ONEBOX_WIFI_DATA_Q:
		{
			/* Check if aggregation enabled */
			if (length > (ONEBOX_RCV_BUFFER_LEN * 4 ))
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					  (TEXT("%s: Toooo big packet %d\n"), __func__,length));	    
				length = ONEBOX_RCV_BUFFER_LEN * 4 ;
			}
			if ((length < ONEBOX_HEADER_SIZE) || (length < MIN_802_11_HDR_LEN))
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: Too small packet %d\n"), __func__, length));
				w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, frame_desc_addr, length);
			}
			
			if (!length)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Dummy pkt has comes in\n"), __func__));
			}
			//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("***WLAN RECEIVED DATA PKT ***\n")));
			ONEBOX_DEBUG(ONEBOX_ZONE_DATA_RCV,(TEXT("@@@ Rcvd Data Pkt b4 netbuf_cb preparation:\n"))); 
			w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DATA_RCV, (frame_desc_addr + offset), (length + extended_desc));
#if 0
#ifndef BYPASS_RX_DATA_PATH
			netbuf_cb = prepare_netbuf_cb(w_adapter, (frame_desc_addr + offset), length, extended_desc);
			if(netbuf_cb == NULL)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_DATA_RCV,(TEXT("@@@ Error in preparing the rcv packet:\n"))); 
				return ONEBOX_STATUS_FAILURE;
			}
#endif
#endif
#ifdef BYPASS_RX_DATA_PATH
			onebox_reorder_pkt(w_adapter, netbuf_cb); /* coex */
			//onebox_reorder_pkt(w_adapter, (frame_desc_addr + offset)); /* coex */
			//onebox_reorder_pkt(w_adapter, (frame_desc_addr + offset));
#else
			w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DATA_RCV, netbuf_cb->data, netbuf_cb->len);
#ifdef PWR_SAVE_SUPPORT
				TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
				{
					/* vap_id and check only if that vap_id matches and vap is of STA mode*/
					vap_id = ((netbuf_cb->data[14] & 0xf0) >> 4);	    
					if(vap && (vap->hal_priv_vap->vap_id == vap_id) 
							&& (vap->iv_opmode == IEEE80211_M_STA) 
							&& (TRAFFIC_PS_EN) 
							&& (ps_params_def.ps_en))
					{
						vap->check_traffic(vap, 0, netbuf_cb->len);
						break;
					}
				}
#endif
			/* As these are data pkts we are not sending rssi and chno vales */
			w_adapter->core_ops->onebox_indicate_pkt_to_net80211(w_adapter,
						   (netbuf_ctrl_block_t *)netbuf_cb, 0, 0);
#endif
		}
		break;

		case ONEBOX_WIFI_MGMT_Q:
		{
			if (onebox_mgmt_pkt_recv(w_adapter, netbuf_cb))
				w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
		}  
		break;

		default:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: pkt from invalid queue\n"), __func__)); 
			w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
		}
		break;
	} /* End switch */      

	FUNCTION_EXIT(ONEBOX_ZONE_INFO);

	return ONEBOX_STATUS_SUCCESS;
}
EXPORT_SYMBOL(wlan_read_pkt);

/**
 * This function set the AHB master access MS word in the SDIO slave registers.
 *
 * @param  Pointer to the driver w_adapter structure. 
 * @param  ms word need to be initialized.
 * @return ONEBOX_STATUS_SUCCESS on success and ONEBOX_STATUS_FAILURE on failure. 
 */
ONEBOX_STATUS onebox_wlan_sdio_master_access_msword(WLAN_ADAPTER w_adapter,
                                               uint16 ms_word)
{
	UCHAR byte;
	uint8 reg_dmn;
	ONEBOX_STATUS  status=ONEBOX_STATUS_SUCCESS;
	struct driver_assets *d_assets = w_adapter->d_assets;

	reg_dmn = 0; //TA domain
	/* Initialize master address MS word register with given value*/
	byte=(UCHAR)(ms_word&0x00FF);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("%s: MASTER_ACCESS_MSBYTE:0x%x\n"), __func__,byte));
	status = d_assets->onebox_common_write_register(d_assets,
																									reg_dmn,
																									SDIO_MASTER_ACCESS_MSBYTE,
																									&byte);             
	if(status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: fail to access MASTER_ACCESS_MSBYTE\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}
	byte=(UCHAR)(ms_word >>8);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("%s:MASTER_ACCESS_LSBYTE:0x%x\n"), __func__,byte));
	status = d_assets->onebox_common_write_register(d_assets,
																									reg_dmn,
																									SDIO_MASTER_ACCESS_LSBYTE,
																									&byte);
	if(status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: fail to access MASTER_ACCESS_LSBYTE\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}
	return ONEBOX_STATUS_SUCCESS;
} /* End <onebox_sdio_master_access_msword */

/**
 * This function schedules the packet for transmission.
 *
 * @param  pointer to the w_adapter structure .  
 */
void schedule_pkt_for_tx(WLAN_ADAPTER w_adapter)
{
		w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->sdio_scheduler_event));
}

static int wlan_thread_waiting_for_event(WLAN_ADAPTER w_adapter)
{
	int32 status = 0;
	if ((w_adapter->buffer_full) || (w_adapter->beacon_event && w_adapter->mgmt_buffer_full))
	{
		/* Wait for 2ms, by the time firmware clears the buffer full interrupt */
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s: Event wait for 2ms \n"), __func__));
		status = w_adapter->os_intf_ops->onebox_wait_event(&w_adapter->sdio_scheduler_event, 2);
		if (status < 0) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d: sdio_sched wait event failed %d\n", __func__, __LINE__, status));
		}
		
	}
	else
	{
		status = w_adapter->os_intf_ops->onebox_wait_event(&w_adapter->sdio_scheduler_event, EVENT_WAIT_FOREVER);
		if (status < 0) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : sdio_sched wait event failed %d\n", __func__, __LINE__, status));
		}
	}
	w_adapter->os_intf_ops->onebox_reset_event(&w_adapter->sdio_scheduler_event);
	w_adapter->sdio_thread_counter++;
	return status;
}
/**
 * This is a kernel thread to send the packets to the device
 *
 * @param
 *  data Pointer to driver's private data
 * @return
 *  None
 */
void sdio_scheduler_thread(void *data)
{
	WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)data;
//	onebox_thread_handle_t *handle = &w_adapter->sdio_scheduler_thread_handle;
	int32 rc = 0;
	uint32 counter;
	struct driver_assets *d_assets = w_adapter->d_assets;
	unsigned long spin_lock_flags;

	FUNCTION_ENTRY(ONEBOX_ZONE_DEBUG);

	do 
	{
		if (w_adapter->beacon_event)
		{

			if (d_assets->host_intf_type == HOST_INTF_USB) {
				counter = d_assets->update_usb_buf_status(d_assets->global_priv);
				if(counter >= 15) {
					counter = 0;
					w_adapter->os_intf_ops->onebox_acquire_spinlock(&d_assets->usb_wlan_coex_write_pending, &spin_lock_flags);
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("CORE_MSG: BUFFER FULL Queueing data packets till BUFFER_FREE COMES\n")));
					atomic_set(&d_assets->techs[WLAN_ID].pkt_write_pending, 1);
					w_adapter->os_intf_ops->onebox_release_spinlock(&d_assets->usb_wlan_coex_write_pending, spin_lock_flags);
					goto WAIT_EVENT;
				} else {

				}
			}
				if(w_adapter->mgmt_buffer_full) {
					//break;
				}else {
					w_adapter->beacon_event = 0;

					if (w_adapter->hal_vap[w_adapter->beacon_event_vap_id].vap != NULL) 
					{
						if (IEEE80211_IS_MODE_BEACON(w_adapter->hal_vap[w_adapter->beacon_event_vap_id].vap->iv_opmode)) 
						{
							//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Calling vap_load_beacon\n")));
							w_adapter->core_ops->onebox_vap_load_beacon(w_adapter, w_adapter->beacon_event_vap_id);
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT ("!!Enabling Beacon path!!!!!!!!!!!\n")));
						}
					}
				}
			}

			if(w_adapter->core_init_done)
			{
				w_adapter->core_ops->onebox_core_qos_processor(w_adapter);
			}

WAIT_EVENT:

			rc = wlan_thread_waiting_for_event(w_adapter);
			if (rc < 0) {
				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wlan_thread_waiting_for_event failed %d\n", __func__, __LINE__, rc));
				//handle->kill_thread = 1;
				break;
			}
			
			/* If control is coming up to end schedule again */

		} 
#if KERNEL_VERSION_BTWN_2_6_(18,26)
		while(!onebox_signal_pending());
#elif KERNEL_VERSION_GREATER_THAN_2_6_(27)
		while(w_adapter->os_intf_ops->onebox_atomic_read(&w_adapter->txThreadDone) == 0); 
		ONEBOX_DEBUG(ONEBOX_ZONE_WARN, 
				(TEXT("sdio_scheduler_thread: Came out of do while loop\n")));
		w_adapter->os_intf_ops->onebox_completion_event(&w_adapter->txThreadComplete,0);
		ONEBOX_DEBUG(ONEBOX_ZONE_WARN, 
				(TEXT("sdio_scheduler_thread: Completed onebox_completion_event \n")));
#endif

		FUNCTION_EXIT(ONEBOX_ZONE_DEBUG);
	}


uint16* hw_queue_status(WLAN_ADAPTER w_adapter)
{

	return NULL;
}
