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
#include "bt_common.h"
#include "onebox_linux.h"
#include "onebox_sdio_intf.h"

static ONEBOX_STATUS bt_gpl_read_pkt(struct driver_assets *d_assets,
				     netbuf_ctrl_block_t *netbuf_cb)
{
	BT_ADAPTER bt_adapter = (BT_ADAPTER)d_assets->techs[BT_ID].priv;

	bt_adapter->os_intf_ops->onebox_acquire_sem(&bt_adapter->bt_gpl_lock, 0);

	if (d_assets->techs[BT_ID].drv_state == MODULE_ACTIVE) {
		bt_read_pkt(bt_adapter, netbuf_cb);
	} else {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("WLAN is being removed.. Dropping Pkt\n")));
		bt_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
		netbuf_cb = NULL;
	}

	bt_adapter->os_intf_ops->onebox_release_sem(&bt_adapter->bt_gpl_lock);

	return ONEBOX_STATUS_SUCCESS;
}

/*
 * This function deregisters BT firmware
 * @param  Pointer to bt_adapter structure.  
 * @return 0 if success else -1. 
 */

static ONEBOX_STATUS bt_deregister_fw(BT_ADAPTER bt_adapter)
{
	uint16 *frame_desc;
	uint16 pkt_len;
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	struct driver_assets *d_assets = bt_adapter->d_assets;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("===> Deregister BT FW <===\n")));

	netbuf_cb = bt_adapter->os_intf_ops->onebox_alloc_skb(FRAME_DESC_SZ);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;

	}
	bt_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, FRAME_DESC_SZ);

	bt_adapter->os_intf_ops->onebox_memset(netbuf_cb->data, 0, FRAME_DESC_SZ);
	
	frame_desc = (uint16 *)netbuf_cb->data;

	/* packet length without descriptor */
	pkt_len = netbuf_cb->len - FRAME_DESC_SZ;

	/* Assigning packet length */
	frame_desc[0] = pkt_len & 0xFFF;

	/* Assigning queue number */
	frame_desc[0] |= (ONEBOX_CPU_TO_LE16(BT_INT_MGMT_Q) & 0x7) << 12;

	/* Assigning packet type in Last word */
	frame_desc[7] = ONEBOX_CPU_TO_LE16(BT_DEREGISTER);

	netbuf_cb->tx_pkt_type = BT_TX_Q;
	
	bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO, netbuf_cb->data, FRAME_DESC_SZ);
	d_assets->bt_deregister_sent = 1;
	status = bt_adapter->onebox_send_pkt_to_coex(bt_adapter->d_assets, netbuf_cb, VIP_Q);
	if (status != ONEBOX_STATUS_SUCCESS) 
	{ 
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT 
			     ("%s: Failed To Write The Packet\n"),__func__));
	}
	BT_TECH.tx_intention = 0;
	BT_TECH.tx_access = 0;
	d_assets->update_tx_status(d_assets, BT_ID);
	d_assets->techs[BT_ID].fw_state = FW_INACTIVE;
	bt_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
  	return status;
}

/**
 * This function is triggered whenever BT module is 
 * inserted. Links BT module with common hal module 
 * @params void
 * @return ONEBOX_STATUS_SUCCESS on success else ONEBOX_STATUS_FAILURE
 */
int32 bt_insert(struct driver_assets *d_assets)
{
	int32 rc = 0;
	BT_ADAPTER bt_adapter;
	struct onebox_os_intf_operations *os_intf_ops = 
		onebox_get_os_intf_operations_from_origin();
	struct onebox_bt_osd_operations *osd_bt_ops = 
		onebox_get_bt_osd_operations_from_origin();
	struct onebox_osi_bt_ops *osi_bt_ops = 
		onebox_get_osi_bt_ops();

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Initialization function called\n"), __func__));

	os_intf_ops->onebox_acquire_sem(&d_assets->bt_init_lock, 0);

	if(d_assets->techs[BT_ID].drv_state == MODULE_ACTIVE) {
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d BT Module is already initialized\n"), __func__, __LINE__));
		os_intf_ops->onebox_release_sem(&d_assets->bt_init_lock);
		return ONEBOX_STATUS_SUCCESS;
	}

	if ((sizeof(struct bt_priv) % 32))
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("size of onebox bt_adapter is not 32 byte aligned\n")));

	bt_adapter = os_intf_ops->onebox_mem_zalloc(sizeof(struct bt_priv), GFP_KERNEL);
	if (!bt_adapter) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s:Memory allocation for bt_adapter failed\n"), __func__));   
		goto nomem;
	}

	os_intf_ops->onebox_memset(bt_adapter, 0, sizeof(struct bt_priv));

	/* Initialise the Core and device dependent operations */
	//bt_adapter->osd_host_intf_ops = osd_host_intf_ops;
	//bt_adapter->osi_host_intf_ops = osi_host_intf_ops;

	bt_adapter->os_intf_ops = os_intf_ops;
	bt_adapter->osd_bt_ops  = osd_bt_ops;
	bt_adapter->osi_bt_ops  = osi_bt_ops;
	bt_adapter->d_assets = d_assets;

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Mutex init successfull\n"), __func__));

	os_intf_ops->onebox_init_dyn_mutex(&bt_adapter->bt_gpl_lock);

	d_assets->techs[BT_ID].priv = (void *)bt_adapter;
	bt_adapter->onebox_send_pkt_to_coex = d_assets->common_send_pkt_to_coex;

	os_intf_ops->onebox_init_event(&(bt_adapter->bt_per_event));


	os_intf_ops->onebox_strcpy(bt_adapter->name, "onebox_bt"); 

	bt_adapter->fsm_state = FSM_DEVICE_READY;


	bt_adapter->driver_mode = d_assets->asset_role;
	bt_adapter->coex_mode   = d_assets->coex_mode;
	bt_adapter->oper_mode	 = d_assets->oper_mode;

	d_assets->techs[BT_ID].drv_state = MODULE_ACTIVE;
	init_waitqueue_head(&d_assets->techs[BT_ID].deregister_event);
	bt_adapter->osi_bt_ops->onebox_bt_read_reg_params(bt_adapter);	
	rc = bt_adapter->osi_bt_ops->onebox_core_init(bt_adapter);
	if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: BT core init failed\n"), __func__));
		goto nocoreinit;	
	}

	d_assets->techs[BT_ID].default_ps_en = 1;

	if (setup_bt_procfs(bt_adapter))
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Failed to setup BT procfs entry\n"), __func__));

	os_intf_ops->onebox_release_sem(&d_assets->bt_init_lock);
	return ONEBOX_STATUS_SUCCESS;

nocoreinit:
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Failed to initialize BT error[%d]\n"), __func__, rc));  
	d_assets->techs[BT_ID].drv_state = MODULE_INSERTED;
	os_intf_ops->onebox_mem_free(bt_adapter);
nomem:
	os_intf_ops->onebox_release_sem(&d_assets->bt_init_lock);
	return ONEBOX_STATUS_FAILURE;
}/* End <bt_insert> */

/**
 * This function removes the wlan module safely..
 *
 * @param  Pointer to sdio_func structure.  
 * @param  Pointer to sdio_device_id structure.  
 * @return VOID. 
 */
int32 bt_remove(struct driver_assets *d_assets)
{
	int32 rc = 0;
	struct onebox_os_intf_operations *os_intf_ops = onebox_get_os_intf_operations_from_origin();
	struct wireless_techs *bt_d;
	BT_ADAPTER bt_adapter = d_assets->techs[BT_ID].priv;

	bt_adapter->os_intf_ops->onebox_acquire_sem(&bt_adapter->bt_gpl_lock, 0);

	FUNCTION_ENTRY(ONEBOX_ZONE_INFO);   
	if (d_assets->techs[BT_ID].drv_state != MODULE_ACTIVE) {
		return ONEBOX_STATUS_SUCCESS;
	}

	if (d_assets->card_state != GS_CARD_DETACH) {
		bt_d = &d_assets->techs[BT_ID];
		bt_d->tx_intention = 1;
		d_assets->update_tx_status(d_assets, BT_ID);
		if(!bt_d->tx_access) {
			d_assets->techs[BT_ID].deregister_flags = 1;
			if(wait_event_timeout((bt_d->deregister_event), 
						(d_assets->techs[BT_ID].deregister_flags == 0),
						msecs_to_jiffies(6000))) {
				bt_deregister_fw(bt_adapter);	
			}	else {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Failed to get sleep exit\n")));
			}
		} else 
			bt_deregister_fw(bt_adapter);	
	}


	destroy_bt_procfs(bt_adapter);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: calling core deinitialization\n"), __func__));   
	rc = bt_adapter->osi_bt_ops->onebox_core_deinit(bt_adapter);
	if (rc)
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: failed to deinit BT, error[%d]\n"), __func__, rc));   

	bt_adapter->os_intf_ops->onebox_release_sem(&bt_adapter->bt_gpl_lock);

	os_intf_ops->onebox_mem_free(bt_adapter);
	d_assets->techs[BT_ID].drv_state = MODULE_INSERTED;
	d_assets->techs[BT_ID].priv = NULL;


	FUNCTION_EXIT(ONEBOX_ZONE_INFO);
	return ONEBOX_STATUS_SUCCESS;
}/* End <bt_remove> */

ONEBOX_STATUS bt_module_init(struct driver_assets *d_assets)
{
	int32 rc = 0;
	d_assets->techs[BT_ID].drv_state = MODULE_INSERTED;

	d_assets->techs[BT_ID].inaugurate = bt_insert;
	d_assets->techs[BT_ID].disconnect = bt_remove;
	d_assets->techs[BT_ID].onebox_get_pkt_from_coex = bt_gpl_read_pkt;

#if 1
	/** If WLAN is not enabled then initialize bt */
	if ((d_assets->card_state == GS_CARD_ABOARD) && 
			!(d_assets->oper_mode & (OP_WLAN_AP_MODE | OP_WLAN_STA_MODE))) { 
		if(d_assets->techs[BT_ID].fw_state == FW_ACTIVE) {
			rc = bt_insert(d_assets);
			if (rc) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: failed to insert "
					"bt error[%d]\n"),__func__, rc)); 
				return 0;
			}
		}
	}	
#endif

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("bt_module_init called\n")));
	return 0;

}

ONEBOX_STATUS bt_module_deinit(struct driver_assets *d_assets)
{
	if (d_assets->techs[BT_ID].drv_state == MODULE_ACTIVE)
		bt_remove(d_assets);

	d_assets->techs[BT_ID].drv_state = MODULE_REMOVED;
	
	d_assets->techs[BT_ID].inaugurate = NULL;
	d_assets->techs[BT_ID].disconnect = NULL;
	d_assets->techs[BT_ID].onebox_get_pkt_from_coex = NULL;

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("bt_module_deinit called and unregistering the gpl driver\n")));
	return 0;

}

ONEBOX_STATIC int32 onebox_bt_gpl_module_init(VOID)
{
	return 0;
}

ONEBOX_STATIC VOID onebox_bt_gpl_module_exit(VOID)
{
	return; 
}

EXPORT_SYMBOL(bt_module_init);
EXPORT_SYMBOL(bt_module_deinit);

module_init(onebox_bt_gpl_module_init);
module_exit(onebox_bt_gpl_module_exit);
MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("Redpine Signals, Inc.");
MODULE_DESCRIPTION("Driver for Redpine Signals' RS9113 module based USB/SDIO cards.");
MODULE_SUPPORTED_DEVICE("Redpine Signals' RS9113 module based USB/SDIO cards.");
