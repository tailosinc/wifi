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

#include <linux/module.h>
#include <linux/kernel.h>
#include "onebox_datatypes.h"
#include "bt_common.h"

uint32 onebox_bt_zone_enabled = ONEBOX_ZONE_INFO |
				ONEBOX_ZONE_INIT |
				ONEBOX_ZONE_OID |
				ONEBOX_ZONE_MGMT_SEND |
				ONEBOX_ZONE_MGMT_RCV |
				ONEBOX_ZONE_DATA_SEND |
				ONEBOX_ZONE_DATA_RCV |
				ONEBOX_ZONE_FSM | 
				ONEBOX_ZONE_ISR |
				ONEBOX_ZONE_MGMT_DUMP |
				ONEBOX_ZONE_DATA_DUMP |
				ONEBOX_ZONE_DEBUG |
				ONEBOX_ZONE_AUTORATE |
				ONEBOX_ZONE_PWR_SAVE |
				ONEBOX_ZONE_ERROR |
				0;  
static ushort bt_rf_tx_power_mode = 0x00;
static ushort bt_rf_rx_power_mode = 0x00;

static ushort bt_rf_type = 0;
static ushort ble_tx_pwr_inx = 0;
static ushort ble_pwr_save_options = 0;

EXPORT_SYMBOL(onebox_bt_zone_enabled);


module_param(bt_rf_tx_power_mode,ushort,0);
MODULE_PARM_DESC(bt_rf_tx_power_mode, "Power modes for Tx");

module_param(bt_rf_type, ushort, 0);
MODULE_PARM_DESC(bt_rf_type, "bt rf type");
module_param(ble_tx_pwr_inx, ushort, 0);
MODULE_PARM_DESC(ble_tx_pwr_inx, "ble_tx_pwr_inx");
module_param(ble_pwr_save_options, ushort, 0);
MODULE_PARM_DESC(ble_pwr_save_options, "ble_pwr_save_options");
module_param(bt_rf_rx_power_mode,ushort,0);
MODULE_PARM_DESC(bt_rf_rx_power_mode, "Power modes for Rx");

int32 bt_read_reg_params (BT_ADAPTER bt_adapter)
{
	uint8 len, status;
	struct driver_assets *d_assets;
	
	netbuf_ctrl_block_t *netbuf_cb;
	uint16 *frame_desc;

  d_assets = bt_adapter->d_assets;

  if(d_assets->device_model == RSI_DEV_9116)
  {
    len = (FRAME_DESC_SZ + 5); /* Payload is 5 bytes and Framedesc sz is 16 bytes */

    bt_adapter->bt_rf_type = (bt_rf_type & 0x0F);
    bt_adapter->ble_tx_pwr_inx = ble_tx_pwr_inx;
    bt_adapter->ble_pwr_save_options = (ble_pwr_save_options & 0x0F);
  }
  else
  {
    len = (FRAME_DESC_SZ + 2); /* Payload is 2bytes and Framedesc sz is 16 bytes */
    bt_adapter->bt_rf_power_mode  = (bt_rf_tx_power_mode & TX_POWER_MODE_MASK); 
    bt_adapter->bt_rf_power_mode |= ((bt_rf_rx_power_mode << 4) & RX_POWER_MODE_MASK); 
  }

	

	netbuf_cb = bt_adapter->os_intf_ops->onebox_alloc_skb(len);
	if (!netbuf_cb) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s  Line %d Error allocating skb\n"), __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}

	//bt_adapter->os_intf_ops->onebox_reserve_data(netbuf_cb, 
	//		REQUIRED_HEADROOM_FOR_BT_HAL);

	bt_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);
	frame_desc = (uint16 *)netbuf_cb->data;

	/* Assigning packet length */
	frame_desc[0] = (netbuf_cb->len - FRAME_DESC_SZ) & 0xFFF;

	/* Assigning queue number */
	frame_desc[0] |= (ONEBOX_CPU_TO_LE16(BT_INT_MGMT_Q) & 0x7) << 12;

	netbuf_cb->bt_pkt_type = BT_RF_POWER_MODE_REQ;

	/* Assigning packet type in first word */
	frame_desc[7] = ONEBOX_CPU_TO_LE16(netbuf_cb->bt_pkt_type);

	netbuf_cb->tx_pkt_type = BT_TX_Q;

	/* Filling payload */
	//frame_desc[8] = bt_adapter->bt_rf_power_mode;
	bt_adapter->os_intf_ops->onebox_memcpy((netbuf_cb->data + FRAME_DESC_SZ), &bt_adapter->bt_rf_power_mode, (netbuf_cb->len - FRAME_DESC_SZ));

  if(d_assets->device_model == RSI_DEV_9116)
  {
    bt_adapter->os_intf_ops->onebox_memcpy((netbuf_cb->data + FRAME_DESC_SZ + 2 ), &bt_adapter->bt_rf_type, sizeof(bt_adapter->bt_rf_type));
    bt_adapter->os_intf_ops->onebox_memcpy((netbuf_cb->data + FRAME_DESC_SZ + 3 ), &bt_adapter->ble_tx_pwr_inx, sizeof(bt_adapter->ble_tx_pwr_inx));
    bt_adapter->os_intf_ops->onebox_memcpy((netbuf_cb->data + FRAME_DESC_SZ + 4 ), &bt_adapter->ble_pwr_save_options, sizeof(bt_adapter->ble_pwr_save_options));
  }

  
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s  %d Line \n"),__func__,__LINE__));
	bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO, netbuf_cb->data, netbuf_cb->len);

	status = bt_adapter->onebox_send_pkt_to_coex(bt_adapter->d_assets, netbuf_cb, BT_Q);
	if(status) 
	{ 
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Failed To Write The Packet\n"),__func__));
		return status;
	}
	return ONEBOX_STATUS_SUCCESS;
}

ONEBOX_STATIC int32 onebox_bt_nongpl_module_init(VOID)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("onebox_nongpl_module_init called and registering the nongpl driver\n")));
	return 0;
}

ONEBOX_STATIC VOID onebox_bt_nongpl_module_exit(VOID)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("onebox_nongpl_module_exit called and unregistering the nongpl driver\n")));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("BT : NONGPL module exit\n")));
	return;
}

module_init(onebox_bt_nongpl_module_init);
module_exit(onebox_bt_nongpl_module_exit);

MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("Redpine Signals, Inc.");
MODULE_DESCRIPTION("Driver for Redpine Signals' RS9113 module based USB/SDIO cards.");
MODULE_SUPPORTED_DEVICE("Redpine Signals' RS9113 module based USB/SDIO cards.");
