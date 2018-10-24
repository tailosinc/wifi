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

#include "onebox_common.h"
#include "onebox_sdio_intf.h"
#include "onebox_core.h"
#include "onebox_host_intf_ops.h"
#include "onebox_zone.h"

extern METADATA metadata_flash_content[];
extern METADATA metadata_swbl_with_mbr;

static uint8 proto_id_to_pkttype(uint8 pkt_type)
{
	uint8 invalid_id = 0xff;

	switch(pkt_type) {

		case ZIGB_TX_Q:
			return ZB_ID;

		case BT_TX_Q:
			return BT_ID;

		case WLAN_TX_M_Q:
		case WLAN_TX_D_Q:
		case COEX_Q:
			return WLAN_ID;

		default:
			return invalid_id;
	}
}

ONEBOX_STATUS onebox_common_read_register(struct driver_assets *d_assets,
																					uint32 Addr, uint8 *data)
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;
	return adapter->osd_host_intf_ops->onebox_read_register(adapter, Addr, data);
}
EXPORT_SYMBOL(onebox_common_read_register);
 
ONEBOX_STATUS onebox_common_write_register(struct driver_assets *d_assets, uint8 reg_dmn,
														 							 uint32 Addr, uint8 *data) 
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;
	return adapter->osd_host_intf_ops->onebox_write_register(adapter, reg_dmn, Addr, data);
}
EXPORT_SYMBOL(onebox_common_write_register);

ONEBOX_STATUS onebox_common_read_multiple(struct driver_assets *d_assets,
																					uint32 Addr, uint32 Count, 
																					uint8 *data )
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;
	return adapter->osd_host_intf_ops->onebox_read_multiple(adapter, Addr, Count, data);
}
EXPORT_SYMBOL(onebox_common_read_multiple);


ONEBOX_STATUS onebox_common_write_multiple(struct driver_assets *d_assets,
																					 uint32 Addr, 
																					 uint8 *data,
																					 uint32 Count,
																					 netbuf_ctrl_block_t *netbuf_cb)
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;
	return adapter->osd_host_intf_ops->onebox_write_multiple(adapter, Addr,
																													 data, Count,
																													 netbuf_cb);
}
EXPORT_SYMBOL(onebox_common_write_multiple);

ONEBOX_STATUS onebox_common_ta_read_multiple(struct driver_assets *d_assets,
																						 uint32 Addr, 
																						 uint8 *data,
																						 uint32 Count)
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;
	return adapter->osd_host_intf_ops->onebox_ta_read_multiple(adapter, Addr, data, Count);
}
EXPORT_SYMBOL(onebox_common_ta_read_multiple);

ONEBOX_STATUS onebox_common_ta_write_multiple(struct driver_assets *d_assets,
																							uint32 Addr,
																							uint8 *data,
																							uint32 Count)
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;
	return adapter->osd_host_intf_ops->onebox_ta_write_multiple(adapter, Addr, data, Count);
}
EXPORT_SYMBOL(onebox_common_ta_write_multiple);

ONEBOX_STATUS onebox_common_master_reg_read(struct driver_assets *d_assets,
																						uint32 addr,
																						uint32 *data,
																						uint16 size) 
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;
	return adapter->osd_host_intf_ops->onebox_master_reg_read(adapter, addr, data, size);
}
EXPORT_SYMBOL(onebox_common_master_reg_read);

ONEBOX_STATUS onebox_common_master_reg_write(struct driver_assets *d_assets,
																						 unsigned long addr,
																						 unsigned long data,
																						 uint16 size) 
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;
	return adapter->osd_host_intf_ops->onebox_master_reg_write(adapter, addr, data, size);
}
EXPORT_SYMBOL(onebox_common_master_reg_write);

void onebox_send_coex_configuration(struct driver_assets *d_assets,coex_cmd_t *coex_cmd_p )
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;
  adapter->coex_osi_ops->send_coex_configuration(adapter,coex_cmd_p);
  return;
}
EXPORT_SYMBOL(onebox_send_coex_configuration);
ONEBOX_STATUS send_pkt_to_coex(struct driver_assets *d_assets,
															 netbuf_ctrl_block_t* netbuf_cb,
															 uint8 hal_queue)
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	uint8 pkt_type;
	uint16 type = 0;
	
	if(!adapter) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s:%d Adapter is null\n"),__func__,__LINE__));
		return ONEBOX_STATUS_FAILURE;
	}

	if(hal_queue != WLAN_Q && hal_queue != VIP_Q) 
	{
	//	adapter->os_intf_ops->onebox_acquire_sem(&adapter->transmit_lock, 0);
		adapter->os_intf_ops->onebox_netbuf_queue_tail(&adapter->coex_queues[hal_queue], netbuf_cb->pkt_addr);
		adapter->tot_pkts_qed[hal_queue]++;
	//	adapter->os_intf_ops->onebox_release_sem(&adapter->transmit_lock);
        	adapter->coex_osi_ops->onebox_coex_sched_pkt_xmit(adapter);
		return status;
	}

	adapter->os_intf_ops->onebox_acquire_sem(&adapter->transmit_lock, 0);
	if(adapter->Driver_Mode == WIFI_MODE_ON && !adapter->flashing_mode_on) { 
		pkt_type  = proto_id_to_pkttype(netbuf_cb->tx_pkt_type);
		if(pkt_type == 0xff) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
						 ("%s: %d  Invalid pkt type \n"),__func__, __LINE__));
			adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, (uint8 *)netbuf_cb->data, FRAME_DESC_SZ);
				status = ONEBOX_STATUS_FAILURE;
				goto fail;
			} else if(d_assets->techs[pkt_type].fw_state != FW_ACTIVE) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
						 ("%s: FW is not active for this protocol %d \n"),__func__,
											netbuf_cb->tx_pkt_type));
				status = ONEBOX_STATUS_FAILURE;
				goto fail;
				
			} else if((!d_assets->techs[pkt_type].tx_access) || (!d_assets->common_hal_tx_access)) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
						 ("%s %d : Protocol/COMMON_HAL Has no TX access proto_id %d and acess %d common_hal access %d\n"),__func__,
											netbuf_cb->tx_pkt_type, pkt_type, d_assets->techs[pkt_type].tx_access, d_assets->common_hal_tx_access));
				dump_stack();
			
				adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, (uint8 *)netbuf_cb->data, FRAME_DESC_SZ);
				if (d_assets->host_intf_type == HOST_INTF_USB)
					adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
				goto fail;
			}
	}

	if((hal_queue == WLAN_Q))	{
		type = ONEBOX_CPU_TO_LE16(*(uint16 *)(&netbuf_cb->data[0]));
		netbuf_cb->tx_pkt_type = ((type & 0xffff)>> 12)  ;
    
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("type is %x and pkt_type is %d\n",type, netbuf_cb->tx_pkt_type));
	}
	status = adapter->osi_host_intf_ops->onebox_host_intf_write_pkt(adapter,
									&netbuf_cb->data[0],
									netbuf_cb->len,
									netbuf_cb->tx_pkt_type, netbuf_cb);
	if (status != ONEBOX_STATUS_SUCCESS)                                 
	{    
 		adapter->tot_pkts_dropped[hal_queue]++;
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(
					"%s: Failed To Write The Packet\n"),__func__));
	}                         
	adapter->tot_pkts_sentout[hal_queue]++;
fail:
	adapter->os_intf_ops->onebox_release_sem(&adapter->transmit_lock);
	return status;
}/*End <send_pkt_to_coex>*/
EXPORT_SYMBOL(send_pkt_to_coex);


/**
 * This function schedules the packet for transmission.
 *
 */
void coex_sched_pkt_xmit(PONEBOX_ADAPTER adapter)
{
	adapter->os_intf_ops->onebox_set_event(&(adapter->sdio_scheduler_thread_handle.thread_event));
}

/**
 * This is a kernel thread to send the packets to the device
 *
 * @param
 *  data Pointer to driver's private data
 * @return
 *  None
 */
void coex_transmit_thread(void *data)
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)data;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	onebox_thread_handle_t *handle = &adapter->sdio_scheduler_thread_handle;
	int32 rt = 0;
	

	FUNCTION_ENTRY(ONEBOX_ZONE_DEBUG);
	do
	{
		if (adapter->flashing_started)
		{
			adapter->fsm_state = FSM_FLASH_BURN; 
			if(adapter->Driver_Mode == QSPI_FLASHING) {
				adapter->qspi_flashing = 1;

				if(adapter->flashing_mode == QSPI_FLASHING||
						adapter->flashing_mode == QSPI_UPDATE) {
				if (adapter->device_model == RSI_DEV_9116)
					status = auto_load_flash_content(adapter, metadata_flash_content[0]); //called only  to load the flash
				else
					status = manual_load_flash_content(adapter, metadata_flash_content[adapter->coex_mode]); //called only  to load the flash
				
				} else if(adapter->flashing_mode == SWBL_FLASHING_NOSBL ||
						adapter->flashing_mode == SWBL_FLASHING_NOSBL_FILE) {
					status = manual_load_flash_content(adapter, metadata_swbl_with_mbr); //called only  to load the swbl
				}
			} else {
				status = auto_load_flash_content(adapter, metadata_flash_content[adapter->coex_mode]); //called only  to load the flash
			}

			if (status != ONEBOX_STATUS_SUCCESS)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: FIRMWARE UPGRADATION FAILED \n"), __FUNCTION__));
				adapter->fsm_state = FSM_CARD_NOT_READY; 
			}
			else
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT ("%s: FIRMWARE UPGRADED TO FLASH SUCCESSFULLY \n"), __FUNCTION__));
				if(adapter->Driver_Mode == WIFI_MODE_ON)
					send_flashing_status(adapter, ONEBOX_TRUE);
				adapter->fsm_state = FSM_MAC_INIT_DONE; 
			}
			adapter->Driver_Mode = WIFI_MODE_ON;
			adapter->flashing_started = 0;
			adapter->flashing_mode_on = 0;
		} 

		rt = adapter->os_intf_ops->onebox_wait_event(&adapter->sdio_scheduler_thread_handle.thread_event, EVENT_WAIT_FOREVER);
		if (rt) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d kill_thread %d \n",__func__, __LINE__, rt, handle->kill_thread));
			handle->kill_thread = 1;
			break;
		}
		adapter->os_intf_ops->onebox_reset_event(&adapter->sdio_scheduler_thread_handle.thread_event);
		adapter->sdio_thread_counter++;
		adapter->coex_osi_ops->onebox_coex_pkt_processor(adapter);

	} 
#if KERNEL_VERSION_BTWN_2_6_(18,26)
	while(!onebox_signal_pending());
#elif KERNEL_VERSION_GREATER_THAN_2_6_(27)
	while(adapter->os_intf_ops->onebox_atomic_read(&adapter->sdio_scheduler_thread_handle.thread_done) == 0); 
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, 
	             (TEXT("%s: Came out of do while loop\n"), __func__));
	adapter->os_intf_ops->onebox_completion_event(&adapter->sdio_scheduler_thread_handle.thread_complete,0);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, 
	             (TEXT("%s: Completed onebox_completion_event \n"), __func__));
#endif
  
	FUNCTION_EXIT(ONEBOX_ZONE_DEBUG);
}

/** This function is used to determine the wmm queue based on the backoff procedure.
 * Data packets are dequeued from the selected hal queue and sent to the below layers.
 * @param  pointer to the driver private data structure
 * @return void
 */
void coex_pkt_processor(PONEBOX_ADAPTER adapter)
{
	netbuf_ctrl_block_t *netbuf_cb;
	uint8 q_num;
	uint8 status;
	uint8 pkt_type;
	uint32_t ulp_sleep_ack = 0;
	struct driver_assets *d_assets = adapter->d_assets;

	while (1) 
	{
		q_num = coex_get_queue_num(adapter);
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		             (TEXT("\n HAL CORE_MSG: qos_processor: Queue number = %d\n"), q_num));

		if (q_num == INVALID_QUEUE) 
		{
				/* The below should be done only if the BT and ZB protocols have power save enabled */
				if(/*Pwr save enabled*/d_assets->techs[BT_ID].default_ps_en) {
					d_assets->techs[BT_ID].tx_intention = 0;
					d_assets->techs[BT_ID].tx_access = 0;
					d_assets->update_tx_status(d_assets, BT_ID);
				}

        if(/*Pwr save enabled*/d_assets->techs[ZB_ID].default_ps_en) {
          d_assets->techs[ZB_ID].tx_intention = 0;
          d_assets->techs[ZB_ID].tx_access = 0;
          d_assets->update_tx_status(d_assets, ZB_ID);
        }
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\nHAL CORE_MSG: qos_pro: No More Pkt Due to invalid queueno\n")));
				break;
		}

	if(q_num == BT_Q){
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Waiting for tx_acces from common hal\n"));
		if(!d_assets->techs[BT_ID].tx_access) {
			d_assets->techs[BT_ID].tx_intention = 1;
			d_assets->update_tx_status(d_assets, BT_ID);
			if(!d_assets->techs[BT_ID].tx_access) {
				//adapter->devdep_ops->onebox_schedule_pkt_for_tx(adapter);
		d_assets->techs[BT_ID].deregister_flags = 1;
		if (wait_event_timeout((d_assets->techs[BT_ID].deregister_event), (d_assets->techs[BT_ID].deregister_flags == 0), msecs_to_jiffies(3000) )) {
		} else {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("ERR: In %s Line %d Initialization of BT Failed as BT TX access is not granted from Common Hal \n", __func__, __LINE__));
			return;
		}
			}
			
		} else {
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("HAS tx_acces from common hal\n"));
		}
	}
	else if(q_num == ZIGB_Q){
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Waiting for tx_acces from common hal\n"));
			if(!d_assets->techs[ZB_ID].tx_access) {
					d_assets->techs[ZB_ID].tx_intention = 1;
					d_assets->update_tx_status(d_assets, ZB_ID);
					if(!d_assets->techs[ZB_ID].tx_access) {
							d_assets->techs[ZB_ID].deregister_flags = 1;
							if (wait_event_timeout((d_assets->techs[ZB_ID].deregister_event), (d_assets->techs[ZB_ID].deregister_flags == 0), msecs_to_jiffies(3000) )) {
							} else {
									ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("ERR: In %s Line %d Initialization of BT Failed as BT TX access is not granted from Common Hal \n", __func__, __LINE__));
									return;
							}
					}else {
							ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("In %s Line %d HAS tx_acces from common hal \n", __func__, __LINE__));
					}
			} 	
	} 	

		/* Removing A Packet From The netbuf Queue */
		netbuf_cb = coex_dequeue_pkt(adapter, q_num);

		if (netbuf_cb == NULL) 
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\nCORE_MSG:NETBUF is NULL qnum = %d\n"),q_num));
			break;
		}
		adapter->os_intf_ops->onebox_acquire_sem(&adapter->transmit_lock, 0);
		if(adapter->Driver_Mode == WIFI_MODE_ON && !adapter->flashing_mode_on) { 
			pkt_type = proto_id_to_pkttype(netbuf_cb->tx_pkt_type);
			if(pkt_type == 0xff) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
							("%s: Invalid pkt type \n"),__func__));
				status = ONEBOX_STATUS_FAILURE;
				goto fail;
			} else if((netbuf_cb->tx_pkt_type == COEX_Q) && 
					(d_assets->common_hal_fw_state != FW_ACTIVE)) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
							("%s: FW is not active for this protocol %d \n"),__func__,
							netbuf_cb->tx_pkt_type));
				status = ONEBOX_STATUS_FAILURE;
				goto fail;
			}
		}

		if((netbuf_cb->data[2] == CONFIRM) && (netbuf_cb->data[15] == ULP_SLEEP_NOTIFY)) {
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("ULP SLEEP ACK\n"));
			ulp_sleep_ack = 1;
			adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_PWR_SAVE, (uint8 *)netbuf_cb->data, netbuf_cb->len);

		} else {
			netbuf_cb->data[1] |= BIT(7);/* IMMEDIATE WAKEUP*/
		}
		status = adapter->osi_host_intf_ops->onebox_host_intf_write_pkt(adapter,
						&netbuf_cb->data[0],
						netbuf_cb->len,
						netbuf_cb->tx_pkt_type,
            netbuf_cb);
	adapter->os_intf_ops->onebox_acquire_sem(&d_assets->tx_access_lock, 0);
		if(ulp_sleep_ack && d_assets->sleep_entry_recvd) {
			d_assets->ulp_sleep_ack_sent = true;
			d_assets->common_hal_tx_access = false;
			ONEBOX_DEBUG( ONEBOX_ZONE_PWR_SAVE, ("ULP SLEEP ACK SENT\n"));
		} 
	adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);

		if (status != ONEBOX_STATUS_SUCCESS)                                 
		{    
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
						("%s: Failed To Write The Packet\n"),__func__));
		}
		adapter->tot_pkts_sentout[q_num]++;
fail:
		adapter->os_intf_ops->onebox_release_sem(&adapter->transmit_lock);
#ifdef USE_USB_INTF
    if(netbuf_cb->tx_pkt_type != COEX_Q) 
#endif
		adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);

	}
//	adapter->os_intf_ops->onebox_release_sem(&adapter->transmit_lock);
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
 * This functions adds the packet to socket buffer queue.
 *
 * @param Pointer to the private driver structure.
 * @param Pointer to netbuf control block structure.
 * @param queue number.
 *
 * @return none.
 */
inline void coex_queue_pkt(PONEBOX_ADAPTER adapter, 
                           netbuf_ctrl_block_t *netbuf_cb, uint8 q_num)
{

	if (q_num > COEX_SOFT_QUEUES) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Invalid Queue Number: q_num = %d\n"), __func__, q_num));
		return;
	}
	
	adapter->os_intf_ops->onebox_netbuf_queue_tail(&adapter->coex_queues[q_num], netbuf_cb->pkt_addr);
	return;
}

/**
 * This functions deletes a packet FROM socket buffer queue.
 *
 * @param Pointer to the private driver structure.
 * @param queue number.
 * @return pointer to  netbuf_ctrl_block structure.
 */
inline netbuf_ctrl_block_t* coex_dequeue_pkt(PONEBOX_ADAPTER adapter, uint8 q_num)
{
	netbuf_ctrl_block_t *netbuf_cb;

	if (q_num > COEX_SOFT_QUEUES)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Invalid Queue Number\n"),__func__));
		return NULL;
	}
	/* Dequeue the packet from the soft queue specified */
	{
		netbuf_cb = adapter->os_intf_ops->onebox_dequeue_pkt((void *)&adapter->coex_queues[q_num]);
	}
	return netbuf_cb;
}

/**
 * This function determines the HAL queue from which packets has to be dequeued while transmission.
 *
 * @param Pointer to the driver's private structure .
 * @return ONEBOX_STATUS_SUCCESS on success else ONEBOX_STATUS_FAILURE.
 */
uint8 coex_get_queue_num(PONEBOX_ADAPTER adapter)
{
	uint8 q_num = INVALID_QUEUE;

	if (adapter->os_intf_ops->onebox_netbuf_queue_len(&adapter->coex_queues[VIP_Q]))
	{
		q_num = VIP_Q;
		return q_num;
	}   
	if (adapter->os_intf_ops->onebox_netbuf_queue_len(&adapter->coex_queues[COEX_Q]))
	{
		q_num = COEX_Q;
		return q_num;
	}   
	if (adapter->os_intf_ops->onebox_netbuf_queue_len(&adapter->coex_queues[BT_Q]))
	{
		q_num = BT_Q;
		return q_num;
	}
	if (adapter->os_intf_ops->onebox_netbuf_queue_len(&adapter->coex_queues[ZIGB_Q]))
	{
		q_num = ZIGB_Q;
		return q_num;
	}
	if (adapter->os_intf_ops->onebox_netbuf_queue_len(&adapter->coex_queues[WLAN_Q]))
	{
		q_num = WLAN_Q;
		return q_num;
	}
	return INVALID_QUEUE;
}

ONEBOX_STATUS onebox_coex_mgmt_frame(PONEBOX_ADAPTER adapter, uint16 *addr, uint16 len)
{
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;

	FUNCTION_ENTRY(ONEBOX_ZONE_MGMT_SEND);

	netbuf_cb = adapter->os_intf_ops->onebox_alloc_skb(len);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;

	}
	adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);
	/*copy the internal mgmt frame to netbuf and queue the pkt */
	adapter->os_intf_ops->onebox_memcpy((uint8 *)netbuf_cb->data, (uint8 *)addr, len);
	netbuf_cb->flags |= INTERNAL_MGMT_PKT;	
	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_MGMT_SEND, (uint8 *)netbuf_cb->data, netbuf_cb->len);
	adapter->os_intf_ops->onebox_netbuf_queue_tail(&adapter->coex_queues[COEX_Q], netbuf_cb->pkt_addr);
	adapter->coex_osi_ops->onebox_coex_sched_pkt_xmit(adapter);
	return status;
}
void send_coex_configuration(PONEBOX_ADAPTER adapter,coex_cmd_t *coex_cmd_p )
{
	onebox_mac_frame_t *mgmt_frame;
	netbuf_ctrl_block_t *netbuf_cb = NULL;
  uint8 len = (FRAME_DESC_SZ +sizeof(coex_cmd_t));
  int status = 0;
	struct driver_assets *d_assets = adapter->d_assets;
	FUNCTION_ENTRY(ONEBOX_ZONE_MGMT_SEND);
	netbuf_cb = adapter->os_intf_ops->onebox_alloc_skb(len);
	if(netbuf_cb == NULL)
	{            
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		return;
	}    
	adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);
	mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
	adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, len);
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((len - FRAME_DESC_SZ)|COEX_Q << 12);
	netbuf_cb->tx_pkt_type = COEX_Q;
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(COEX_CONFIGURATIONS); 	
	adapter->os_intf_ops->onebox_memcpy((uint8 *)(netbuf_cb->data+FRAME_DESC_SZ), (uint8 *)coex_cmd_p, sizeof(coex_cmd_t));
	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, (uint8 *)mgmt_frame, len);
	status = send_pkt_to_coex(d_assets, netbuf_cb, COEX_Q);
	if (status != ONEBOX_STATUS_SUCCESS)                                 
	{    
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
					("%s: Failed To send coex Configuration\n"),__func__));
	}
	return;
}
EXPORT_SYMBOL(send_coex_configuration);

ONEBOX_STATUS onebox_wait_for_rf_prog_frame_rsp(struct driver_assets *d_assets) 
{

    PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;
    ONEBOX_STATUS status_l = 0;

    ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("===> WAITING FOR RF RESPONE <===\n")));
    status_l = adapter->os_intf_ops->onebox_wait_event(&(adapter->common_bb_rf_event),EVENT_WAIT_FOREVER );
    adapter->os_intf_ops->onebox_reset_event(&(adapter->common_bb_rf_event));

    if(status_l < 0) {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                (TEXT ("%s: TIMEOUT IN RF RESPONE \n"), __FUNCTION__));
        return ONEBOX_STATUS_FAILURE;	
    }
    return status_l;
} 
EXPORT_SYMBOL(onebox_wait_for_rf_prog_frame_rsp);

