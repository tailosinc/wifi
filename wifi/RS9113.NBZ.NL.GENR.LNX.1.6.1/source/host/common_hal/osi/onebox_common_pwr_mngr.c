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
#include "onebox_common.h"
#include "onebox_hal.h"
#include "onebox_linux.h"
#include "onebox_pktpro.h"
#include "onebox_intf_ops.h"
#include "onebox_sdio_intf.h"
#include "onebox_zone.h"

int protocol_tx_access(struct driver_assets *d_assets)
{
	uint8 ii = 0;
	for (ii = 0; ii < MAX_IDS; ii++) {
		if (d_assets->techs[ii].drv_state == MODULE_ACTIVE &&
				d_assets->techs[ii].tx_intention)
			return 1;
	}
	return 0;
}


#ifdef GPIO_HANDSHAKE

/**
 * gpio_host_tx_intention() - This function is called when the host GPIO value has changed.
 *
 * @gpio_value: The value of the host gpio.
 *
 * Return: None.
 */
void gpio_host_tx_intention(PONEBOX_ADAPTER adapter,bool gpio_value)
{
	struct driver_assets *d_assets = adapter->d_assets;
#ifdef USE_SDIO_INTF
	struct mmc_host *host ;
#endif

	int counter = 0;

#ifdef USE_SDIO_INTF
	if (adapter->host_intf_type == HOST_INTF_SDIO)
		host = adapter->sdio_pfunction->card->host;
#endif

	if (gpio_value) {
		ONEBOX_DEBUG( ONEBOX_ZONE_PWR_SAVE, intk("In %s %d setting host gpio high\n", __func__, __LINE__));		
		adapter->os_intf_ops->onebox_set_host_status(gpio_value);
REPEAT_CHECK:
		if (!adapter->os_intf_ops->onebox_get_device_status()) { /* Device GPIO is low */
			while(1) {
				msleep(1);
				if (adapter->os_intf_ops->onebox_get_device_status()) {
					/* Once device GPIO is high, wait for 2 more milliseconds and
					 * again read the status, if it is high then give tx_access 
					 */
					msleep(2);
					if (adapter->os_intf_ops->onebox_get_device_status()) {

#ifdef USE_SDIO_INTF
						if (adapter->host_intf_type == HOST_INTF_SDIO)
						  host->ops->set_ios (host, &host->ios);
#endif
						d_assets->common_hal_tx_access = true;
//						ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("In %s Line %d cmn tx is %d\n", __func__, __LINE__, d_assets->common_hal_tx_access));
						break;

					}else
						continue;
					//add BT and ZB tx_access	

				}

				if(counter++ > 30) { 
					ONEBOX_DEBUG( ONEBOX_ZONE_PWR_SAVE, ("Device GPIO not high after 30ms\n"));
					ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("ERR: Stopping driver ops\n"));
					break;
				}
			}
		} else { /* Device GPIO status is high */
			msleep(2);
			if (adapter->os_intf_ops->onebox_get_device_status()) {
				d_assets->common_hal_tx_access = true;
//				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("In %s Line %d cmn tx is %d\n", __func__, __LINE__, d_assets->common_hal_tx_access));
			} else {
//				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("In %s Line %d cmn tx is %d\n", __func__, __LINE__, d_assets->common_hal_tx_access));
				goto REPEAT_CHECK;
			}
		}
	} else {
		adapter->os_intf_ops->onebox_set_host_status(gpio_value);
//		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("In %s %d setting host gpio low\n", __func__, __LINE__));		
	}
}

#endif

static void send_ulp_sleep_ack(PONEBOX_ADAPTER adapter)
{
	onebox_mac_frame_t *mgmt_frame;
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	//ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	struct driver_assets *d_assets = adapter->d_assets;

	FUNCTION_ENTRY(ONEBOX_ZONE_MGMT_SEND);

	netbuf_cb = adapter->os_intf_ops->onebox_alloc_skb(FRAME_DESC_SZ);
	if(netbuf_cb == NULL)
	{            
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		return;

	}    
	adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, FRAME_DESC_SZ);

	mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
	adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

	ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (TEXT("In %s line %d \n"), __func__, __LINE__));
	if (adapter->ulp_sleep_token == 0xABCD) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("error \n"));
	}
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(0 << 12);
	netbuf_cb->tx_pkt_type = COEX_Q;
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(CONFIRM); 
	mgmt_frame->desc_word[6] = adapter->ulp_sleep_token;
	mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(ULP_SLEEP_NOTIFY << 8);

	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_PWR_SAVE, (uint8 *)mgmt_frame, FRAME_DESC_SZ);
	d_assets->ulp_sleep_ack_sent = true;
	send_pkt_to_coex(d_assets, netbuf_cb, COEX_Q);
#if 0
	status = adapter->osi_host_intf_ops->onebox_host_intf_write_pkt(adapter,
		                                                 netbuf_cb->data, 
	                                                         netbuf_cb->len,
	                                                         netbuf_cb->tx_pkt_type, netbuf_cb);
	if (status != ONEBOX_STATUS_SUCCESS)                                 
	{    
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
					("%s: Failed To Write The Packet\n"),__func__));
	}
	adapter->tot_pkts_sentout[COEX_Q]++;
	adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
#endif
	return;
}
/**
 * sleep_entry_recvd() - This function is called when a sleep entry frame is recvd from fw.
 *
 * Return: None.
 */
void sleep_entry_recvd(PONEBOX_ADAPTER adapter)
{
	struct driver_assets *d_assets = adapter->d_assets;
	uint8 proto_id = 0;


	ONEBOX_DEBUG( ONEBOX_ZONE_PWR_SAVE, ("In %s %d\n", __func__, __LINE__));
	adapter->os_intf_ops->onebox_acquire_sem(&d_assets->tx_access_lock, 0);
	d_assets->sleep_entry_recvd = 1;
	if (!protocol_tx_access(d_assets)) { /* checks if all active protocols have tx_access */
		//d_assets->common_hal_tx_access = false;
		ONEBOX_DEBUG( ONEBOX_ZONE_PWR_SAVE, ("In %s Line %d cmn tx is %d\n", __func__, __LINE__, d_assets->common_hal_tx_access));
		send_ulp_sleep_ack(adapter);
	} else {
			ONEBOX_DEBUG( ONEBOX_ZONE_PWR_SAVE, ("One of the protcol requested tx_acess\n"));
			d_assets->common_hal_tx_access = true;
			d_assets->ulp_sleep_ack_sent = false;
			while(proto_id < MAX_IDS) {
					if (d_assets->techs[proto_id].deregister_flags 
									&& (d_assets->techs[proto_id].drv_state == MODULE_ACTIVE)) {
							d_assets->techs[proto_id].deregister_flags = 0;
							ONEBOX_DEBUG( ONEBOX_ZONE_PWR_SAVE, ("Waking up an event \n"));
							d_assets->techs[proto_id].tx_access = 1;
							wake_up(&d_assets->techs[proto_id].deregister_event);
					}
					proto_id++;
			}
	}

	adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
	return;
}

/**
 * sleep_exit_recvd() - This function is called when a sleep exit frame is recvd from fw.
 *
 * Return: None.
 */
void sleep_exit_recvd(PONEBOX_ADAPTER adapter)
{
	struct driver_assets *d_assets = adapter->d_assets;
	uint8 proto_id = 0;

	//acquire_lock();//add mutex	
	adapter->os_intf_ops->onebox_acquire_sem(&d_assets->tx_access_lock, 0);
	d_assets->sleep_entry_recvd = 0;
	d_assets->common_hal_tx_access = true;
	d_assets->ulp_sleep_ack_sent = false;
	adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
	while(proto_id < MAX_IDS) {
		if (d_assets->techs[proto_id].deregister_flags 
		    && (d_assets->techs[proto_id].drv_state == MODULE_ACTIVE)) {
			d_assets->techs[proto_id].deregister_flags = 0;
			ONEBOX_DEBUG( ONEBOX_ZONE_PWR_SAVE, ("Waking up an event \n"));
			d_assets->techs[proto_id].tx_access = 1;
			wake_up(&d_assets->techs[proto_id].deregister_event);
		}
		proto_id++;
	}
	//release lock
	return;
}


/**
 * update_tx_status() - This function is used to update the various sleep states
 * 			based on the tx state of the all the active protocols with
 * 			ps enabled.
 *
 * @proto_id: ID of the protocol that calls the function.
 *
 * Return: None.
 */
void update_tx_status(struct driver_assets *d_assets, uint8 proto_id)
{
	//struct driver_assets *d_assets = onebox_get_driver_asset();
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)d_assets->global_priv;

	if (proto_id > 2) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("ERR: invalid protocol id in %s %d\n", __func__, __LINE__));
		return;
	}

	if (d_assets->techs[proto_id].drv_state == MODULE_REMOVED) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("ERR: MODULE_REMOVED state %s %d proto_id %d\n", __func__, __LINE__, proto_id));
		return;
	}

	if (d_assets->techs[proto_id].tx_intention) {
		//acquire lock
		adapter->os_intf_ops->onebox_acquire_sem(&d_assets->tx_access_lock, 0);
		//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("In %s Line %d cmn tx is %d\n", __func__, __LINE__, d_assets->common_hal_tx_access));
		if (d_assets->common_hal_tx_access) {
			if(d_assets->ulp_sleep_ack_sent) {
				//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("**Sleep ack is queued cant give access\n"));
				adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
				return;
				
			}
			d_assets->techs[proto_id].tx_access = 1;
			//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Setting tx_acces for %d %s %d\n", proto_id, __func__, __LINE__));
		} else {

#ifdef GPIO_HANDSHAKE
			if(d_assets->lp_ps_handshake_mode == GPIO_HAND_SHAKE || d_assets->ulp_ps_handshake_mode == GPIO_HAND_SHAKE){
				gpio_host_tx_intention(adapter,GPIO_HIGH);
				/* giving tx_access in gpio_host_tx_intention is not good b'se gpio fn doesnt
				 * know who has requested the acess.
				 * So returning from abve function if common hal has tx_access the give
				 * access to requested protocol.
				 */ 
				if (d_assets->common_hal_tx_access) {
					d_assets->techs[proto_id].tx_access = true;
				}
			}
#else
			if (adapter->host_intf_type == HOST_INTF_USB) {

				if(adapter->usb_intf_suspend) {
					adapter->os_intf_ops->onebox_mod_timer(&adapter->usb_resume_timer, msecs_to_jiffies(1));
				}
			}
#endif

		}
		adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
		return;
	} else {

#ifdef GPIO_HANDSHAKE
		if(d_assets->lp_ps_handshake_mode == GPIO_HAND_SHAKE || d_assets->ulp_ps_handshake_mode == GPIO_HAND_SHAKE){
			//acquire_lock
			adapter->os_intf_ops->onebox_acquire_sem(&d_assets->tx_access_lock, 0);
			if (!protocol_tx_access(d_assets)) {
				d_assets->common_hal_tx_access = 0;
				gpio_host_tx_intention(adapter,GPIO_LOW);
			}
			//release lock
			adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
			return;
		}
#else
		adapter->os_intf_ops->onebox_acquire_sem(&d_assets->tx_access_lock, 0);
		if (d_assets->sleep_entry_recvd && !d_assets->ulp_sleep_ack_sent) {
			if (!protocol_tx_access(d_assets)) {
				//d_assets->common_hal_tx_access = 0;
				ONEBOX_DEBUG( ONEBOX_ZONE_PWR_SAVE, ("In %s Line %d sending ULP sleep_ack\n", __func__, __LINE__));
				send_ulp_sleep_ack(adapter);
			} 
		}
		adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
		return;
#endif

	}
}

EXPORT_SYMBOL(sleep_entry_recvd);
EXPORT_SYMBOL(sleep_exit_recvd);
EXPORT_SYMBOL(update_tx_status);
EXPORT_SYMBOL(protocol_tx_access);
