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

#include "onebox_host_intf_ops.h"
#include "onebox_intf_ops.h"
#include "onebox_sdio_intf.h"
#include "onebox_zone.h"
#include "onebox_cm_pwr_mgr.h"
#include <linux/usb.h>

ONEBOX_EXTERN int onebox_register_os_intf_operations(struct onebox_os_intf_operations *os_intf_ops);
ONEBOX_STATUS onebox_usb_sg_card_write(PONEBOX_ADAPTER adapter, void *buf,uint16 len,uint8 ep_num);
ONEBOX_STATUS onebox_usb_gspi_init(PONEBOX_ADAPTER adapter);
int onebox_usb_probe (struct usb_interface *pfunction,
                  const struct usb_device_id *id);
VOID onebox_usb_disconnect (struct usb_interface *pfunction);
uint32 onebox_find_bulkInAndOutEndpoints (struct usb_interface *interface, PONEBOX_ADAPTER rsi_dev);
ONEBOX_STATUS onebox_usb_card_write(PONEBOX_ADAPTER adapter, void *buf, uint16 len, uint8 ep_num);

static const struct usb_device_id onebox_usb_IdTable[] = 
{
//	{ USB_DEVICE(0x0303, 0x0100) },
//	{ USB_DEVICE(0x041B, 0x0301) },
//	{ USB_DEVICE(0x041B, 0x0201) },
//	{ USB_DEVICE(0x041B, 0x9113) },
	{ USB_DEVICE(0x1618, 0x9113) },
	{ USB_DEVICE(0x1618, 0x9116) },
#ifdef GDVR_DRV
	{ USB_DEVICE(0x041B, 0x9330) },
#endif
	{ /* Blank */},
};

void onebox_cancel_rx_urbs(PONEBOX_ADAPTER adapter)
{
	ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Killing RX URBS\n"));
	if (adapter->rx_usb_urb[0]) { 
		usb_kill_urb(adapter->rx_usb_urb[0]);
	}

	if (adapter->rx_usb_urb[1]) { 
		usb_kill_urb(adapter->rx_usb_urb[1]);
	}

	if (adapter->rx_usb_urb[2]) { 
		usb_kill_urb(adapter->rx_usb_urb[2]);
	}

	if (adapter->rx_usb_urb[3]) { 
		usb_kill_urb(adapter->rx_usb_urb[3]);
	}

	if (adapter->rx_usb_urb[4]) { 
		usb_kill_urb(adapter->rx_usb_urb[4]);
	}
}


#ifdef CONFIG_PM
void onebox_resume_usb_intf_timer(PONEBOX_ADAPTER adapter)
{
	struct driver_assets *d_assets = adapter->d_assets;

	ONEBOX_DEBUG(ONEBOX_ZONE_USB_DEBUG_INFO, ("Resume Timer Expired\n"));

	if(adapter->usb_intf_suspend) {
		if (adapter->usb_in_deep_ps) {
			/*Check tx access */
			if(!protocol_tx_access(d_assets)) { 
				/*If no acess found confgure timer and return*/
				adapter->os_intf_ops->onebox_mod_timer(&adapter->usb_resume_timer, msecs_to_jiffies(1000));
				return;
			}
		}

		ONEBOX_DEBUG(ONEBOX_ZONE_USB_DEBUG_INFO, ("Resuming usb Interface\n"));
		usb_autopm_get_interface_async(adapter->usb_pfunction);
		if (atomic_read(&adapter->usb_pfunction->pm_usage_cnt) > 0) {
			usb_autopm_put_interface_async(adapter->usb_pfunction);
		}
	}
}

static int onebox_usb_suspend(struct usb_interface *iface, pm_message_t pm_msg)
{
	PONEBOX_ADAPTER adapter = usb_get_intfdata(iface);
	struct driver_assets *d_assets = adapter->d_assets;
	uint8 *temp_buf;
	uint16 len = 6;
	ONEBOX_STATUS status_l;
	uint32 suspend_time = 0;
	struct usb_device *usbdev = adapter->usbdev;
	
	temp_buf = kzalloc(6, GFP_KERNEL);
	if(!temp_buf) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("Can't Allocate Memory\n"));
		return -ENOMEM;
	}
	 	

	adapter->os_intf_ops->onebox_acquire_sem(&d_assets->tx_access_lock, 0);
	if ((!protocol_tx_access(d_assets)) 
			&& (!atomic_read(&adapter->tx_pending_urb_cnt))) {


		status_l = usb_control_msg (usbdev,
				usb_rcvctrlpipe (usbdev, 0),
				USB_VENDOR_PS_STATUS_READ,
				USB_TYPE_VENDOR | USB_DIR_IN | USB_RECIP_DEVICE,
				0 , 0,
				(void *) temp_buf, len, 3000);

		if (status_l < 0)
		{
			ONEBOX_DEBUG (ONEBOX_ZONE_ERROR,
					("%s Line %d :REG READ FAILED WITH ERROR CODE :%010x \n", __func__, __LINE__, status_l));
			adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
			kfree(temp_buf);
			return -EBUSY;
		}

		ONEBOX_DEBUG(ONEBOX_ZONE_USB_DEBUG_INFO, ("Status of FW is %d\n", temp_buf[0]));

		if (!temp_buf[0]) {
			ONEBOX_DEBUG(ONEBOX_ZONE_USB_DEBUG_INFO, ("Dont Process suspend\n"));
			adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
			kfree(temp_buf);
			return -EBUSY;
		}
		if((*(uint32 *)&temp_buf[2])) {
			suspend_time = ((*(uint32 *)&temp_buf[2]) /1000); 
			adapter->usb_in_deep_ps = 0;
		} else {
			suspend_time = 1000; 
			adapter->usb_in_deep_ps = 1;
		}

		ONEBOX_DEBUG(ONEBOX_ZONE_USB_DEBUG_INFO, ("Suspend time is %d\n", suspend_time));

		if (suspend_time < 5) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("Dont Process suspend as suspend time is less than 5ms \n"));
			adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
			kfree(temp_buf);
			return -EBUSY;
		}

		d_assets->common_hal_tx_access = false; //ABT to suspend the USB state hence remove the access to common hal
		onebox_cancel_rx_urbs(adapter);

		adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);

		adapter->usb_intf_suspend = 1;

		if (!adapter->usb_resume_timer.function) {
			adapter->os_intf_ops->onebox_init_sw_timer(&adapter->usb_resume_timer, (unsigned long)adapter,
					(void *)&onebox_resume_usb_intf_timer, msecs_to_jiffies(suspend_time));
		} else 
			adapter->os_intf_ops->onebox_mod_timer(&adapter->usb_resume_timer, msecs_to_jiffies(suspend_time));

		kfree(temp_buf);
		ONEBOX_DEBUG(ONEBOX_ZONE_USB_DEBUG_INFO, ("Sucessfully Suspended\n"));
		return 0;
	}
	else {
		adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
		kfree(temp_buf);
		return -EBUSY;
	}
}

static int onebox_usb_resume(struct usb_interface *iface)
{
	PONEBOX_ADAPTER adapter = usb_get_intfdata(iface);
	struct driver_assets *d_assets = adapter->d_assets;
	uint8 *temp_buf;
	uint16 len = 6;
	ONEBOX_STATUS status_l;
	struct usb_device *usbdev = adapter->usbdev;

	temp_buf = kzalloc(6, GFP_KERNEL);
	if(!temp_buf) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("Can't Allocate Memory\n"));
		return -ENOMEM;
	}
	 	
	adapter->os_intf_ops->onebox_acquire_sem(&d_assets->tx_access_lock, 0);
	ONEBOX_DEBUG(ONEBOX_ZONE_USB_DEBUG_INFO, ("Interface Resumed\n"));

	status_l = usb_control_msg (usbdev,
			usb_rcvctrlpipe (usbdev, 0),
			USB_VENDOR_PS_STATUS_READ,
			USB_TYPE_VENDOR | USB_DIR_IN | USB_RECIP_DEVICE,
			1 , 0,
			(void *) temp_buf, len, 3000);

	if (status_l < 0)
	{
		ONEBOX_DEBUG (ONEBOX_ZONE_ERROR,
				("%s Line %d :REG READ FAILED WITH ERROR CODE :%010x \n", __func__, __LINE__, status_l));
		adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
		kfree(temp_buf);
		return status_l;
	}
	adapter->osd_host_intf_ops->onebox_rcv_urb_submit(adapter,
			adapter->rx_usb_urb[0], 1); 

	adapter->osd_host_intf_ops->onebox_rcv_urb_submit(adapter,
			adapter->rx_usb_urb[2], 1); 

	adapter->osd_host_intf_ops->onebox_rcv_urb_submit(adapter,
			adapter->rx_usb_urb[3], 1); 

	adapter->osd_host_intf_ops->onebox_rcv_urb_submit(adapter,
			adapter->rx_usb_urb[4], 1); 

	adapter->osd_host_intf_ops->onebox_rcv_urb_submit(adapter,
			adapter->rx_usb_urb[1], 2); 

	adapter->usb_intf_suspend = 0;
	adapter->os_intf_ops->onebox_release_sem(&d_assets->tx_access_lock);
	sleep_exit_recvd(adapter);
	kfree(temp_buf);
	return 0;
}
#endif

ONEBOX_STATIC struct usb_driver onebox_usb_driver = 
{
	.name       = "Onebox-USB",
	.probe      = onebox_usb_probe,
	.disconnect     = onebox_usb_disconnect,
	.id_table   = onebox_usb_IdTable,
#ifdef CONFIG_PM
	.suspend    = onebox_usb_suspend,
	.resume	    = onebox_usb_resume,
	.supports_autosuspend = 1,
#endif
};


static ONEBOX_STATUS usb_ulp_read_write(PONEBOX_ADAPTER adapter, uint16 addr, uint16 *data, uint16 len_in_bits)
{
	if((usb_master_reg_write(adapter, GSPI_DATA_REG1, ((addr << 6) | (data[1] & 0x3f)), 2) < 0)) {
		goto fail;
	}

	if((usb_master_reg_write(adapter, GSPI_DATA_REG0, ONEBOX_CPU_TO_LE16(*(uint16 *)&data[0]), 2)) < 0) {
		goto fail;
	}

	if((onebox_usb_gspi_init(adapter)) < 0) {
		goto fail;
	}

	if((usb_master_reg_write(adapter, GSPI_CTRL_REG1, ((len_in_bits - 1) | GSPI_TRIG), 2)) < 0) {
		goto fail;
	}

 	msleep(10);
	return ONEBOX_STATUS_SUCCESS;
fail:
	return ONEBOX_STATUS_FAILURE;
}

ONEBOX_STATUS onebox_usb_gspi_init(PONEBOX_ADAPTER adapter)
{
	uint32 gspi_ctrl_reg0_val;
	//! RF control reg 
	//! clk_ratio [3:0] 
	/* Programming gspi frequency = soc_frequency / 2 */
	/* Warning : ULP seemed to be not working
	 * well at high frequencies. Modify accordingly */
	gspi_ctrl_reg0_val = 0x4;
	//! csb_setup_time [5:4] 
	gspi_ctrl_reg0_val |= 0x10; 
	//! csb_hold_time [7:6] 
	gspi_ctrl_reg0_val |= 0x40; 
	//! csb_high_time [9:8] 
	gspi_ctrl_reg0_val |= 0x100; 
	//! spi_mode [10] 
	gspi_ctrl_reg0_val |= 0x000; 
	//! clock_phase [11] 
	gspi_ctrl_reg0_val |= 0x000; 
	/* Initializing GSPI for ULP read/writes */
	return usb_master_reg_write(adapter, GSPI_CTRL_REG0, gspi_ctrl_reg0_val, 2);
}

/**
 * This function resets and re-initializes the card.
 *
 * @param  Pointer to usb_func.  
 * @VOID 
 */
static void onebox_reset_card(PONEBOX_ADAPTER adapter)
{
	uint16 temp[4] = {0};
	if((adapter->osd_host_intf_ops->onebox_master_reg_write(adapter, 
					SWBL_REGOUT, FW_WDT_DISABLE_REQ, 2)) < 0) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d FW WDT Disabling failed..\n",__func__,
					__LINE__));
		goto fail;
	}

#define TA_HOLD_REG 0x22000844
	if(adapter->d_assets->asset_role == 3 && adapter->host_intf_type == HOST_INTF_USB)
		usb_master_reg_write(adapter, TA_HOLD_REG, 0xB, 4); 
	else
		usb_master_reg_write(adapter, TA_HOLD_REG, 0xE, 4); 
	msleep(100);
	if (!(adapter->device_model == RSI_DEV_9116)) {
		*(uint32 *)temp = 2;
		if((usb_ulp_read_write(adapter, WATCH_DOG_TIMER_1, &temp[0], 32)) < 0) {
			goto fail;
		}

		*(uint32 *)temp = 0;
		if((usb_ulp_read_write(adapter, WATCH_DOG_TIMER_2, temp, 32)) < 0) {
			goto fail;
		}

		*(uint32 *)temp = 50;
		if((usb_ulp_read_write(adapter, WATCH_DOG_DELAY_TIMER_1, temp, 32)) < 0) {
			goto fail;
		}

		*(uint32 *)temp = 0;
		if((usb_ulp_read_write(adapter, WATCH_DOG_DELAY_TIMER_2, temp, 32)) < 0) {
			goto fail;
		}

		*(uint32 *)temp = ((0xaa000) | RESTART_WDT | BYPASS_ULP_ON_WDT);
		if((usb_ulp_read_write(adapter, WATCH_DOG_TIMER_ENABLE, temp, 32)) < 0) {
			goto fail;
		}
	}
	else {
		if((usb_master_reg_write(adapter, NWP_WWD_INTERRUPT_TIMER, 5, 4)) < 0) {
			goto fail;
		}
		if((usb_master_reg_write(adapter, NWP_WWD_SYSTEM_RESET_TIMER, 8, 4)) < 0) {
			goto fail;
		}
		if((usb_master_reg_write(adapter, NWP_WWD_MODE_AND_RSTART, 0xAA0001, 4)) < 0) {
			goto fail;
		}
	}
	return;
fail:
	ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Reset card Failed\n"));
	return;
}

/**
 * This function is called by kernel when the driver provided 
 * Vendor and device IDs are matched. All the initialization 
 * work is done here.
 *
 * @param  Pointer to usb_func structure.  
 * @param  Pointer to usb_device_id structure.  
 * @return SD_SUCCESS in case of successful initialization or 
 *         a negative error code signifying failure. 
 */
int onebox_usb_probe(struct usb_interface *pfunction, const struct usb_device_id *id)
{
	struct onebox_coex_osi_operations *coex_osi_ops = onebox_get_coex_osi_operations();
	struct onebox_osi_host_intf_operations *osi_host_intf_ops = onebox_get_usb_osi_host_intf_operations();
	struct onebox_osd_host_intf_operations *osd_host_intf_ops = onebox_get_usb_osd_host_intf_operations();
	struct onebox_os_intf_operations *os_intf_ops = onebox_get_os_intf_operations_from_origin();
	struct driver_assets *d_assets;
	uint8 ii;
	PONEBOX_ADAPTER adapter;

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		(TEXT("%s: driver initialization ...\n"), __func__));

	/* Register gpl related os interface operations */
	onebox_register_os_intf_operations(os_intf_ops);

	d_assets = os_intf_ops->onebox_mem_zalloc(sizeof(struct driver_assets),GFP_KERNEL);
	if (d_assets == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Memory allocation for d_assets failed\n"), __func__));   
		return ONEBOX_TRUE;
	}
	os_intf_ops->onebox_memset(d_assets, 0, sizeof(struct driver_assets));

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Initialization function called\n"), __func__));

	adapter = os_intf_ops->onebox_mem_zalloc(sizeof(ONEBOX_ADAPTER),GFP_KERNEL);
	if(adapter==NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Memory allocation for adapter failed\n"), __func__));   
		goto fail;
	}
	os_intf_ops->onebox_memset(adapter, 0, sizeof(ONEBOX_ADAPTER));

	d_assets->global_priv = (void *)adapter;

	adapter->d_assets = d_assets;
	
	adapter->host_intf_type = HOST_INTF_USB;
	d_assets->host_intf_type = HOST_INTF_USB;

	adapter->segment = (uint8 *)kmalloc(2048, GFP_ATOMIC);
	if (adapter->segment == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Memory alloc for segment failed\n"), __func__));   
		goto fail;
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
                          (TEXT("Initialized HAL/CORE/DEV_DEPENDENT Operations\n")));   

	//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("PROBE: In %s Line %d adapter = %p usbdev = %p\n", __func__, __LINE__, adapter, adapter->usbdev));

	adapter->usbdev = interface_to_usbdev(pfunction);
	//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("PROBE: In %s Line %d usbdev = %p\n", __func__, __LINE__, adapter->usbdev));
	if(adapter->usbdev == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Memory allocation for usb failed \n"), __func__));   
		return ONEBOX_TRUE;
	}

	if (onebox_find_bulkInAndOutEndpoints (pfunction, adapter))
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR,  ("Error2\n"));
		goto fail;
	}
	//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("PROBE: In %s Line %d adapter = %p usbdev = %p\n", __func__, __LINE__, adapter, adapter->usbdev));

	/* storing our device information in interface device for future refrences */	
	usb_set_intfdata(pfunction, adapter);
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Enabled the interface\n"), __func__));
	/* Initialise the Core and device dependent operations */
	adapter->coex_osi_ops = coex_osi_ops;
	adapter->osd_host_intf_ops = osd_host_intf_ops;
	adapter->osi_host_intf_ops = osi_host_intf_ops;
	adapter->os_intf_ops = os_intf_ops;

	adapter->usb_pfunction = pfunction;
	//d_assets->global_priv = (void *)adapter;
	d_assets->pfunc = (void *)pfunction;

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Product ID : 0x%x "
		"Vendor ID : 0x%x \n"),__func__, id->idProduct, 
		id->idVendor));
	if(id && (id->idProduct == 0x9113)){
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: 9113 MODULE IS CONNECTED\n"),__func__));
		adapter->device_model = RSI_DEV_9113;
	}
	else  if(id && (id->idProduct == 0x9116)){
		adapter->device_model = RSI_DEV_9116;
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: 9116 MODULE IS CONNECTED\n"),__func__));
	} else {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("*** Invalid Device ID ***\n")));
		goto fail;
	}

	d_assets->device_model = adapter->device_model;

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Context setting suceeded\n"), __func__));   
#if 1
	adapter->DataRcvPacket = (uint8*)os_intf_ops->onebox_mem_zalloc((ONEBOX_RCV_BUFFER_LEN * 4),
	                                                                GFP_KERNEL);
	if(adapter->DataRcvPacket==NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Memory allocation for receive buffer failed\n"), __func__));   
		goto fail;
	}
#endif	
	os_intf_ops->onebox_init_dyn_mutex(&adapter->transmit_lock);
	os_intf_ops->onebox_init_spinlock(&d_assets->usb_wlan_coex_write_pending);

	/* coex */
	if(adapter->coex_osi_ops->onebox_common_hal_init(d_assets, adapter)  != ONEBOX_STATUS_SUCCESS) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s Line %d: Failed to initialize common HAL\n"), __func__, __LINE__));
		goto fail;
	}


	/*** Receive thread */
	adapter->rx_usb_urb[0]    =  usb_alloc_urb(0, GFP_KERNEL);
	adapter->rx_usb_urb[1]    =  usb_alloc_urb(0, GFP_KERNEL);
	adapter->rx_usb_urb[2]    =  usb_alloc_urb(0, GFP_KERNEL);
	adapter->rx_usb_urb[3]    =  usb_alloc_urb(0, GFP_KERNEL);
	adapter->rx_usb_urb[4]    =  usb_alloc_urb(0, GFP_KERNEL);
	
	for(ii = 0; ii < MAX_TX_URB; ii ++) 
	{
		adapter->tx_usb_urb[ii]    =  usb_alloc_urb(0, GFP_KERNEL);
		adapter->context_urb[ii] = adapter->os_intf_ops->onebox_mem_zalloc(sizeof(urb_context_t), GFP_ATOMIC);
	}
	adapter->os_intf_ops->onebox_init_event(&(adapter->usb_tx_event));
	adapter->tx_coex_usb_urb[0]    =  usb_alloc_urb(0, GFP_KERNEL);
	adapter->tx_coex_usb_urb[1]    =  usb_alloc_urb(0, GFP_KERNEL);

	//adapter->rx_usb_urb[0]->transfer_buffer = adapter->DataRcvPacket;
	adapter->TransmitBlockSize = 252;
	adapter->ReceiveBlockSize  = 252;

	coex_osi_ops->onebox_read_reg_params(adapter);
	/*To verify whether Fw is already downloaded */
	usb_master_reg_read(adapter, MISC_REG, &adapter->fw, 2);//read 12 register
	adapter->fw &= 1;
	if (!(adapter->fw)) 
	{ 
		if((adapter->fw_load_mode == FLASH_RAM_NO_SBL) ||
				(adapter->fw_load_mode == FW_LOAD_WITH_DESC)) {
			if((usb_master_reg_write (adapter,MISC_REG,0x0,2)) < 0) {
				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Writing into address MISC_REG addr 0x41050012 Failed\n"));
				goto fail;
			}
		}
		if(coex_osi_ops->onebox_device_init(adapter, 1)) /* Load firmware */
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Failed in device initialization\n"), __func__));
			goto fail;
		}
		if((adapter->fw_load_mode == FLASH_RAM_NO_SBL) ||
				(adapter->fw_load_mode == FW_LOAD_WITH_DESC)) {
			if (adapter->device_model == RSI_DEV_9116) {
				/* Do nothing */
			}
			else {
				if((usb_master_reg_write (adapter,ROM_PC_RESET_ADDRESS,0xab,1)) < 0) {
					ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Resetting ROM PC Failed\n"));
					goto fail;
				}
			}
		}
	}

	adapter->osd_host_intf_ops->onebox_rcv_urb_submit(adapter,
			adapter->rx_usb_urb[0], 1); 

	adapter->osd_host_intf_ops->onebox_rcv_urb_submit(adapter,
			adapter->rx_usb_urb[1], 2); 

	adapter->osd_host_intf_ops->onebox_rcv_urb_submit(adapter,
			adapter->rx_usb_urb[2], 1); 

	adapter->osd_host_intf_ops->onebox_rcv_urb_submit(adapter,
			adapter->rx_usb_urb[3], 1); 

	adapter->osd_host_intf_ops->onebox_rcv_urb_submit(adapter,
			adapter->rx_usb_urb[4], 1); 
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: usb_master_reg_write done\n"), __func__));

	d_assets->card_state = GS_CARD_ABOARD;
	d_assets->techs[WLAN_ID].drv_state = MODULE_REMOVED;
	d_assets->techs[WLAN_ID].fw_state = FW_INACTIVE;
	d_assets->techs[BT_ID].drv_state = MODULE_REMOVED;
	d_assets->techs[BT_ID].fw_state = FW_INACTIVE;
	d_assets->techs[ZB_ID].drv_state = MODULE_REMOVED;
	d_assets->techs[ZB_ID].fw_state = FW_INACTIVE;

#ifdef CONFIG_PM
	usb_enable_autosuspend(adapter->usbdev);
#endif

	return 0;    
fail:
	d_assets->card_state = GS_CARD_DETACH;
	d_assets->techs[WLAN_ID].drv_state = MODULE_REMOVED;
	d_assets->techs[WLAN_ID].fw_state = FW_INACTIVE;
	d_assets->techs[BT_ID].drv_state = MODULE_REMOVED;
	d_assets->techs[BT_ID].fw_state = FW_INACTIVE;
	d_assets->techs[ZB_ID].drv_state = MODULE_REMOVED;
	d_assets->techs[ZB_ID].fw_state = FW_INACTIVE;
	if (adapter->sdio_scheduler_thread_handle.function_ptr)
		adapter->os_intf_ops->onebox_kill_thread(&adapter->sdio_scheduler_thread_handle); 
	if (adapter->onebox_entry)
		adapter->os_intf_ops->onebox_remove_proc_entry(adapter->onebox_entry, adapter->proc_name);
	if (adapter->segment)
		adapter->os_intf_ops->onebox_mem_free(adapter->segment);
	if (adapter)
		kfree(adapter);
	if (d_assets)
		kfree(d_assets);

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Failed to initialize...Exiting\n"), __func__));  
		
	return -ENODEV;
}/* End <onebox_usb_probe> */


/**
 * This function performs the reverse of the probe function..
 *
 * @param  Pointer to usb_func structure.  
 * @param  Pointer to usb_device_id structure.  
 * @return VOID. 
 */
VOID onebox_usb_disconnect(struct usb_interface *pfunction)
{
	struct onebox_os_intf_operations *os_intf_ops;
	struct driver_assets *d_assets ;
	uint8 ii;

	PONEBOX_ADAPTER adapter = usb_get_intfdata(pfunction);

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
	    ("%s: device deinitialization\n", __func__));

	if (!adapter) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(" Adapter is NULL \n"));
		return;
	}

	d_assets = adapter->d_assets;
	if (!d_assets) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(" d_assets is NULL \n"));
		return;
	}

	adapter->os_intf_ops->onebox_remove_timer(&adapter->usb_resume_timer);
	adapter->coex_osi_ops->onebox_common_hal_deinit(d_assets, adapter);
	//onebox_cancel_rx_urbs(adapter);
	onebox_reset_card(adapter);
	os_intf_ops = onebox_get_os_intf_operations_from_origin(); 
	os_intf_ops->onebox_set_event(&adapter->usb_tx_event);
	os_intf_ops->onebox_delete_event(&(adapter->usb_tx_event));
#ifdef CONFIG_PM
	usb_disable_autosuspend(adapter->usbdev);
#endif

	/* Release the interrupt handler */
	adapter->stop_card_write = 2; //stopping all writes after deinit
	usb_free_urb(adapter->rx_usb_urb[0]);
	usb_free_urb(adapter->rx_usb_urb[1]);
	usb_free_urb(adapter->rx_usb_urb[2]);
	usb_free_urb(adapter->rx_usb_urb[3]);
	usb_free_urb(adapter->rx_usb_urb[4]);
	for (ii = 0; ii < MAX_TX_URB; ii ++) {
		usb_free_urb(adapter->tx_usb_urb[ii]);
		adapter->os_intf_ops->onebox_mem_free(adapter->context_urb[ii]);
	}
	
	usb_free_urb(adapter->tx_coex_usb_urb[0]);
	usb_free_urb(adapter->tx_coex_usb_urb[1]);
	kfree(adapter->segment);

	if (adapter->write_fail) /* is this even meaningful? */
		adapter->write_fail = 0;

	os_intf_ops->onebox_mem_free(d_assets);
	os_intf_ops->onebox_mem_free(adapter);

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
	    ("%s: device disconnected.\n", __func__));

	return;
}/* End <onebox_usb_disconnect> */

/**
 * This function reads one byte of information from a register.
 *
 * @param  Pointer to Driver adapter structure.  
 * @param  Function Number.  
 * @param  Address of the register.  
 * @param  Pointer to the data that stores the data read.  
 * @return On success ONEBOX_STATUS_SUCCESS else ONEBOX_STATUS_FAILURE. 
 */
ONEBOX_STATUS usb_read_register(PONEBOX_ADAPTER ext_adapter, uint32 Addr,
                            uint8 *data)  
{
#if ((defined USE_USB_INTF) && (defined USB_BUFFER_STATUS_INDI))
	uint16 data1 = 0;
	usb_master_reg_read(ext_adapter->usbdev, Addr, &data1, 2);//read single byte from register
	*data = *(uint8 *)&data1;
#endif
	return ONEBOX_STATUS_SUCCESS;
}/* End <usb_read_register> */

/**
 * This function writes one byte of information into a register.
 *
 * @param  Pointer to Driver adapter structure.  
 * @param  Function Number.  
 * @param  Address of the register.  
 * @param  Pointer to the data tha has to be written.  
 * @return On success ONEBOX_STATUS_SUCCESS else ONEBOX_STATUS_FAILURE. 
 */
int usb_write_register(PONEBOX_ADAPTER adapter,uint8 reg_dmn,uint32 Addr,uint8 *data)
{
	return ONEBOX_STATUS_SUCCESS;
}/* End <usb_write_register> */

/**
 * This function is a dummy stub in case of USB interface
 *
 * @param  Pointer to Driver adapter structure.  
 * @param  Function Number.  
 * @param  Address of the register.  
 * @param  Length of the data to be read.  
 * @param  Pointer to the read data.  
 * @return On success ONEBOX_STATUS_SUCCESS else ONEBOX_STATUS_FAILURE. 
 */
ONEBOX_STATUS usb_read_register_multiple(PONEBOX_ADAPTER adapter, 
                                     uint32 Addr,
                                     uint32 Count,
                                     uint8 *data )
{
	return ONEBOX_STATUS_SUCCESS;
}/* End <usb_read_register_multiple> */  

void wlan_usb_tx_done_handler(struct urb *urb) 
{

	urb_context_t *context_ptr = urb->context;
	PONEBOX_ADAPTER adapter = context_ptr->adapter;
	struct driver_assets *d_assets = adapter->d_assets;
	netbuf_ctrl_block_t *netbuf_recv_pkt = context_ptr->netbuf_addr;
	uint64 status = 0 ;

	context_ptr->urb_in_use = 0;
  
	if (urb->status) 
	{
		ONEBOX_DEBUG (ONEBOX_ZONE_ERROR, 
			      ("USB CARD WRITE FAILED WITH ERROR CODE :%10d  length %d\n",
			       urb->status, urb->actual_length));
		ONEBOX_DEBUG (ONEBOX_ZONE_DEBUG, ("PKT Tried to Write is \n"));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, urb->transfer_buffer, urb->actual_length);
		adapter->os_intf_ops->onebox_free_pkt(netbuf_recv_pkt, 0);
		adapter->write_fail = 1;
		return;
	}

	atomic_dec(&adapter->tx_pending_urb_cnt);
	adapter->os_intf_ops->onebox_free_pkt(netbuf_recv_pkt, 0);

	/**Handle below cases for BT and ZB*/
	adapter->os_intf_ops->onebox_acquire_spinlock(&d_assets->usb_wlan_coex_write_pending, &status);
	if(atomic_read(&d_assets->techs[WLAN_ID].pkt_write_pending)) {
		if(atomic_read(&adapter->tx_pending_urb_cnt) <= (MAX_TX_URB - 6)) {
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("Clearing Buffer full interput %s Line %d\n"), __func__, __LINE__));
			atomic_set(&d_assets->techs[WLAN_ID].pkt_write_pending, 0);
	//struct onebox_os_intf_operations *os_intf_ops = onebox_get_os_intf_operations_from_origin();
			d_assets->techs[WLAN_ID].schedule_pkt_tx(d_assets);
		}
	}
	adapter->os_intf_ops->onebox_release_spinlock(&d_assets->usb_wlan_coex_write_pending,status);
#if 0
	if((!d_assets->common_hal_tx_access) && 
			(d_assets->ulp_sleep_ack_sent)   && 
			(!atomic_read(&adapter->tx_pending_urb_cnt))) {
		if(!d_assets->common_hal_tx_access) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Suspend now\n"));
			adapter->osd_host_intf_ops->onebox_schedule_suspend();	
		}
	}
#endif
}

void zb_bt_tx_done_handler(struct urb *urb) 
{
	PONEBOX_ADAPTER adapter = urb->context;

	if (urb->status) 
	{
		ONEBOX_DEBUG (ONEBOX_ZONE_ERROR, 
				("USB CARD WRITE FAILED WITH ERROR CODE :%10d  length %d\n", urb->status, urb->actual_length));
		ONEBOX_DEBUG (ONEBOX_ZONE_DEBUG, ("PKT Tried to Write is \n"));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, urb->transfer_buffer, urb->actual_length);
		adapter->write_fail = 1;
		return;
	}
	adapter->os_intf_ops->onebox_set_event(&adapter->usb_tx_event);
}

ONEBOX_STATUS wlan_usb_card_write(PONEBOX_ADAPTER adapter, netbuf_ctrl_block_t *netbuf_cb, uint16 len, uint8 ep_num)
{
	ONEBOX_STATUS status_l;
	uint8 ii;
	struct urb *urb;
	uint32 found = 0;
	urb_context_t *context_ptr = NULL;

	adapter->write_urb = ((adapter->write_urb + 1) & (MAX_TX_URB - 1));
	context_ptr = adapter->context_urb[adapter->write_urb];	

	if(context_ptr->urb_in_use) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Urb over write issue occured\n")));
		for(ii = (adapter->write_urb + 1); ii < (MAX_TX_URB - 1); ii++) {
			context_ptr = adapter->context_urb[ii];	
			if(!context_ptr->urb_in_use) {
				adapter->write_urb = ii;
				found = 1;
				break;
			}
		}
		if(!found ) {
			return ONEBOX_STATUS_FAILURE;
		}
	}

	if(adapter->usb_intf_suspend) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s Line %d WHY U came to write in suspended state\n", __func__, __LINE__));
		dump_stack();
	}

	if(context_ptr == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to fetch the URB context ptr\n")));
		return ONEBOX_STATUS_FAILURE;
	}

	context_ptr->urb_in_use = 1;
	context_ptr->adapter = adapter;
	context_ptr->netbuf_addr = netbuf_cb;
	urb = adapter->tx_usb_urb[adapter->write_urb]; 
	urb->transfer_buffer = netbuf_cb->data;
	atomic_inc(&adapter->tx_pending_urb_cnt);
	urb->transfer_flags |= URB_ZERO_PACKET;

	usb_fill_bulk_urb (urb,
			adapter->usbdev,
			usb_sndbulkpipe (adapter->usbdev,
				adapter->bulk_out_endpointAddr[ep_num - 1]),
			(void *)netbuf_cb->data, len, wlan_usb_tx_done_handler,
			context_ptr);

	//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Submitiing urb\n"));
	status_l = usb_submit_urb(urb, GFP_KERNEL); 

	if (status_l < 0) 
	{
		ONEBOX_DEBUG (ONEBOX_ZONE_ERROR, ("Failed To Submit URB Status :%10d \n", status_l));
		adapter->write_fail = 1;
		return status_l;
	}

	return ONEBOX_STATUS_SUCCESS;
}

ONEBOX_STATUS onebox_wlan_usb_card_write(PONEBOX_ADAPTER adapter, uint8 *buf, uint16 len, uint8 ep_num, netbuf_ctrl_block_t *netbuf_cb)
{
	ONEBOX_STATUS status_l = 0;
	uint8 status;

	if (adapter->write_fail)
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s: Unable to write, device not responding endpoint:%d\n", __func__, ep_num));
		return status_l;
	}

	//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Changing HDR SIZE ************\n"));
	status = adapter->os_intf_ops->onebox_change_hdr_size(netbuf_cb, ONEBOX_USB_TX_HEAD_ROOM);
	//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Status is %d\n", status));
	if((status != 0) && (status < 0)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("NOT ENOUGH HEADROOM\n")));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, netbuf_cb->data, netbuf_cb->len);
		adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
		return ONEBOX_STATUS_FAILURE;
	}
	adapter->os_intf_ops->onebox_memset(netbuf_cb->data,0,ONEBOX_USB_TX_HEAD_ROOM);

	status_l = wlan_usb_card_write(adapter, netbuf_cb, netbuf_cb->len, ep_num);
	return status_l;
}

/**
 * This function writes multiple bytes of information to the SD card.
 *
 * @param  Pointer to Driver adapter structure.  
 * @param  Function Number.  
 * @param  Address of the register.  
 * @param  Length of the data.  
 * @param  Pointer to the data that has to be written.  
 * @return On success ONEBOX_STATUS_SUCCESS else ONEBOX_STATUS_FAILURE. 
 */
ONEBOX_STATUS usb_write_register_multiple(PONEBOX_ADAPTER adapter,
                                      uint32 Addr,
                                      uint8 *data,
                                      uint32 Count,
                                      netbuf_ctrl_block_t *netbuf_cb
                                     )
{
	ONEBOX_STATUS status_l;

	status_l = 0;
	if (adapter->write_fail) 
	{
		return status_l;
	}
	if(adapter == NULL || Addr == 0 )
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("PROBE: In %s Line %d unable to card write check\n", __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}
  
	if(Addr == 1) {
		//status_l = onebox_wlan_usb_card_write(adapter, data, Count,  Addr);
		status_l = onebox_wlan_usb_card_write(adapter, data, Count,  Addr, netbuf_cb);
	} else {
		status_l = onebox_usb_sg_card_write(adapter, data, Count,  Addr);
	}

	return status_l;
} /* End <usb_write_register_multiple> */

ONEBOX_STATUS onebox_usb_sg_card_write(PONEBOX_ADAPTER adapter, void *buf, uint16 len, uint8 ep_num)
{
	ONEBOX_STATUS status_l;
	uint8 *seg;

	status_l = 0;
	if (adapter->write_fail)
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s: Unable to write, device not responding endpoint:%d\n", __func__, ep_num));
		return status_l;
	}
	if ((adapter->coex_mode == WIFI_ZIGBEE) && 
	    (((ep_num == 3) && (adapter->device_model == RSI_DEV_9116)) || 
	     ((!(adapter->device_model == RSI_DEV_9116)) && (ep_num == 2)))) {

		  seg = adapter->segment;
		  memcpy(seg, buf, len);
	} else {
		seg = adapter->segment;
		memset(seg, 0, ONEBOX_USB_TX_HEAD_ROOM);
		memcpy(seg + ONEBOX_USB_TX_HEAD_ROOM, buf, len);
		len += ONEBOX_USB_TX_HEAD_ROOM;
	}

	status_l = onebox_usb_card_write(adapter, seg, len, ep_num);
	return status_l;
}



ONEBOX_STATUS onebox_usb_card_write(PONEBOX_ADAPTER adapter, void *buf, uint16 len, uint8 ep_num)
{
	ONEBOX_STATUS status_l;
  	int32 rt = 0;
	

	if(adapter->usb_intf_suspend) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s Line %d WHY U came to write in suspended state\n", __func__, __LINE__));
		dump_stack();
	}
	adapter->tx_coex_usb_urb[0]->transfer_flags |= URB_ZERO_PACKET;

	usb_fill_bulk_urb (adapter->tx_coex_usb_urb[0],
			adapter->usbdev,
			usb_sndbulkpipe (adapter->usbdev,
				adapter->bulk_out_endpointAddr[ep_num - 1]),
			(void *)buf, len, zb_bt_tx_done_handler,
			adapter);

	status_l = usb_submit_urb(adapter->tx_coex_usb_urb[0], GFP_KERNEL); 

	if (status_l < 0) 
	{
		ONEBOX_DEBUG (ONEBOX_ZONE_ERROR, ("Failed To Submit URB Status :%10d \n", status_l));
		adapter->write_fail = 1;
		return status_l;
	}

  	rt = adapter->os_intf_ops->onebox_wait_event(&adapter->usb_tx_event, 60*HZ);
  	if (rt < 0) {
  	  ONEBOX_DEBUG (ONEBOX_ZONE_ERROR, ("%s line no %d: wait on usb_tx_event failed %d\n", __func__, __LINE__, rt));
  	  goto fail;
  	} else if (rt > 0) {
  	  adapter->os_intf_ops->onebox_reset_event(&adapter->usb_tx_event);
  	  return ONEBOX_STATUS_SUCCESS;
  	}

fail:
	ONEBOX_DEBUG (ONEBOX_ZONE_ERROR, ("USB CARD WRITE FAILED WITH ERROR CODE :%10d \n", status_l));
	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, 
			adapter->tx_coex_usb_urb[0]->transfer_buffer, 
			adapter->tx_coex_usb_urb[0]->actual_length);
	adapter->os_intf_ops->onebox_reset_event(&adapter->usb_tx_event);
	adapter->write_fail = 1;
	return ONEBOX_STATUS_FAILURE;
}

/**
 * This function registers the client driver.
 *
 * @param  VOID.  
 * @return 0 if success else a negative number. 
 */
int register_usb_driver(void)
{
	return (usb_register(&onebox_usb_driver));
}

/**
 * This function unregisters the client driver.
 *
 * @param  VOID.  
 * @return VOID. 
 */
void unregister_usb_driver(void)
{
	usb_deregister(&onebox_usb_driver);
}


/*FUNCTION*********************************************************************
Function Name:  onebox_remove
Description:    Dummy for linux usb
Returned Value: None
Parameters: 
----------------------------+-----+-----+-----+------------------------------
Name                        | I/P | O/P | I/O | Purpose
----------------------------+-----+-----+-----+------------------------------
None
 ******************************************************************************/
ONEBOX_STATUS usb_remove(void)
{
	/**Dummy for Linux*/
	return 0;
}

/**
 * This function initializes the bulk endpoints 
 * to the device.
 *
 * @param
 *      interface       Interface descriptor
 * @param
 *      rsi_dev         driver control block
 * @return         
 *      0 on success and -1 on failure 
 */
uint32 onebox_find_bulkInAndOutEndpoints (struct usb_interface *interface, PONEBOX_ADAPTER rsi_dev)
{
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int buffer_size;
	int ret_val = -ENOMEM, i, bep_found = 0, binep_found = 0;
	uint8 ep_num = 0;
	uint32 pipe = 0;

	ONEBOX_DEBUG( ONEBOX_ZONE_INIT,  ("{%s+}\n", __FUNCTION__));

	/* set up the endpoint information */
	/* check out the endpoints */
	/* use only the first bulk-in and bulk-out endpoints */

	if((interface == NULL) || (rsi_dev == NULL)) 
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR,  ("Null pointer in {%s}\n", __func__));
		return ret_val;
	}

	iface_desc = &(interface->altsetting[0]);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             ("bNumEndpoints :%08x \n", iface_desc->desc.bNumEndpoints));
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) 
	{
		endpoint = &(iface_desc->endpoint[i].desc);

		ONEBOX_DEBUG (ONEBOX_ZONE_INFO, ("IN LOOP :%08x \n", bep_found));
#if 0		
		if ((!(rsi_dev->bulk_in_endpointAddr)) && 
		      (endpoint->bEndpointAddress & USB_DIR_IN) &&	/* Direction */
		      ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK)) 
		{
			/* we found a bulk in endpoint */
			ONEBOX_DEBUG (ONEBOX_ZONE_INFO, ("IN EP :%08x \n", i));
			buffer_size = endpoint->wMaxPacketSize;
			//buffer_size = MAX_RX_PKT_SZ;
			rsi_dev->bulk_in_size = buffer_size;
			rsi_dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			ep_num = endpoint->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("IN ep num is %d\n",ep_num));
			pipe = usb_rcvbulkpipe(adapter->usbdev, ep_num);
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("IN pipe num is %8x\n",pipe));
			pipe = usb_rcvbulkpipe(adapter->usbdev, endpoint->bEndpointAddress);
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("IN pipe num 2 is %8x\n",pipe));
		}
#else		
		if ((!(rsi_dev->bulk_in_endpointAddr[binep_found])) && 
		      (endpoint->bEndpointAddress & USB_DIR_IN) &&	/* Direction */
		      ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK)) 
		{
			/* we found a bulk in endpoint */
			ONEBOX_DEBUG (ONEBOX_ZONE_INFO, ("IN EP :%08x \n", i));
			buffer_size = endpoint->wMaxPacketSize;
			//buffer_size = MAX_RX_PKT_SZ;
			rsi_dev->bulk_in_endpointAddr[binep_found] = endpoint->bEndpointAddress;
			ep_num = endpoint->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("IN ep num is %d\n",ep_num));
			pipe = usb_rcvbulkpipe(rsi_dev->usbdev, ep_num);
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("IN pipe num is %8x\n",pipe));
			pipe = usb_rcvbulkpipe(rsi_dev->usbdev, endpoint->bEndpointAddress);
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("IN pipe num 2 is %8x\n",pipe));
			/* on some platforms using this kind of buffer alloc
			 * call eliminates a dma "bounce buffer".
			 *
			 * NOTE: you'd normally want i/o buffers that hold
			 * more than one packet, so that i/o delays between
			 * packets don't hurt throughput.
			 */
			buffer_size = endpoint->wMaxPacketSize;
			rsi_dev->bulk_in_size[binep_found] = buffer_size;
			//rsi_dev->tx_urb->transfer_flags = (URB_NO_TRANSFER_DMA_MAP );
			binep_found++;
		}
#endif		
		if (!rsi_dev->bulk_out_endpointAddr[bep_found] &&
		    !(endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK)) 
		{
			ONEBOX_DEBUG (ONEBOX_ZONE_INFO, ("OUT EP :%08x \n", i));
			/* we found a bulk out endpoint */
			rsi_dev->bulk_out_endpointAddr[bep_found] = endpoint->bEndpointAddress;
			ep_num = endpoint->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("OUT ep num is %d\n",ep_num));
			pipe = usb_sndbulkpipe(rsi_dev->usbdev, ep_num);
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("OUT pipe num is %8x\n",pipe));
			pipe = usb_sndbulkpipe(rsi_dev->usbdev, endpoint->bEndpointAddress);
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("OUT pipe num 2 is %8x\n",pipe));
			/* on some platforms using this kind of buffer alloc
			 * call eliminates a dma "bounce buffer".
			 *
			 * NOTE: you'd normally want i/o buffers that hold
			 * more than one packet, so that i/o delays between
			 * packets don't hurt throughput.
			 */
			buffer_size = endpoint->wMaxPacketSize;
			rsi_dev->bulk_out_size[bep_found] = buffer_size;
			//rsi_dev->tx_urb->transfer_flags = (URB_NO_TRANSFER_DMA_MAP );
			bep_found++;
		}
		if ((bep_found >= MAX_BULK_EP) || (binep_found >= MAX_BULK_EP))
		{
			break;
		}
	}

	if (!(rsi_dev->bulk_in_endpointAddr[0] && rsi_dev->bulk_out_endpointAddr[0])) 
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR,  ("Couldn't find both bulk-in and bulk-out endpoints"));
		return ret_val;
	} else 
	{
		ONEBOX_DEBUG (ONEBOX_ZONE_INIT, ("EP INIT SUCESS \n"));
	}

	return 0;
}

/* This function reads the data from given register address 
 *
 * @param
 *      Adapter         Pointer to driver's private data area.
 * @param
 *      reg             Address of the register
 * @param
 *      value           Value to write
 * @param
 *      len             Number of bytes to write
 * @return         
 *      none
 */
ONEBOX_STATUS usb_master_reg_read (PONEBOX_ADAPTER adapter, uint32 reg, uint32 * value, uint16 len)
{
	uint8 *read_buf;
	ONEBOX_STATUS status_l;
	struct usb_device *usbdev = adapter->usbdev;
	status_l = ONEBOX_STATUS_SUCCESS;
	len = 2;
	//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("arguments to ctrl msg usbdev = %p value = %p\n", usbdev, value ));
	
	read_buf = kzalloc(4, GFP_KERNEL);
	if(!read_buf) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("Can't Allocate Memory\n"));
		return -ENOMEM;
	}
	 	
	status_l = usb_control_msg (usbdev,
	                            usb_rcvctrlpipe (usbdev, 0),
	                            USB_VENDOR_REGISTER_READ,
	                            USB_TYPE_VENDOR | USB_DIR_IN | USB_RECIP_DEVICE,
	                            ((reg & 0xffff0000) >> 16), (reg & 0xffff),
	                            (void *) read_buf, len, HZ * 5);

	*value = (read_buf[0] | (read_buf[1] << 8));
	if (status_l < 0)
	{
		ONEBOX_DEBUG (ONEBOX_ZONE_ERROR,
		              ("REG READ FAILED WITH ERROR CODE :%010x \n", status_l));
		kfree(read_buf);
		return status_l;
	}
	ONEBOX_DEBUG (ONEBOX_ZONE_INFO, ("USB REG READ VALUE :%10x \n", *value));
	kfree(read_buf);
	return status_l;
}

/**
 * This function writes the given data into the given register address 
 *
 * @param
 *      Adapter         Pointer to driver's private data area.
 * @param
 *      reg             Address of the register
 * @param
 *      value           Value to write
 * @param
 *      len             Number of bytes to write
 * @return         
 *      none
 */
ONEBOX_STATUS usb_master_reg_write (PONEBOX_ADAPTER adapter, unsigned long reg,
    unsigned long value, uint16 len)
{
	uint8 *usb_reg_buf;
	ONEBOX_STATUS status_l;
	struct usb_device *usbdev = adapter->usbdev;

	usb_reg_buf = kzalloc(4, GFP_KERNEL);
	if(!usb_reg_buf) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("Can't Allocate Memory\n"));
		return -ENOMEM;
	}
	status_l = ONEBOX_STATUS_SUCCESS;
	usb_reg_buf[0] = (value & 0x000000ff);
	usb_reg_buf[1] = (value & 0x0000ff00) >> 8;
	usb_reg_buf[2] = (value & 0x00ff0000) >> 16; 
	usb_reg_buf[3] = (value & 0xff000000) >> 24;

	ONEBOX_DEBUG (ONEBOX_ZONE_INFO, ("USB REG WRITE VALUE :%lu \n", value));
	status_l = usb_control_msg (usbdev,
	                            usb_sndctrlpipe (usbdev, 0),
	                            USB_VENDOR_REGISTER_WRITE, USB_TYPE_VENDOR,
	                            ((reg & 0xffff0000) >> 16), (reg & 0xffff),
	                            (void *) usb_reg_buf, len, HZ * 5);
	if (status_l < 0)
	{
		ONEBOX_DEBUG (ONEBOX_ZONE_ERROR,
		              ("REG WRITE FAILED WITH ERROR CODE :%10d \n", status_l));
	}
	kfree(usb_reg_buf);

	return status_l;
}

/**
 * This function submits the given URB to the USB stack 
 *
 * @param
 *      Adapter         Pointer to driver's private data area.
 * @param
 *      URB             URB to submit
 * @return         
 *      none
 */
void onebox_rx_urb_submit (PONEBOX_ADAPTER adapter, struct urb *urb, uint8 ep_num)
{
	void (*done_handler) (struct urb *);
	urb_context_t *context_ptr = NULL;
	uint32_t total_len = 0;

	context_ptr = adapter->os_intf_ops->onebox_mem_zalloc(sizeof(urb_context_t), 
				GFP_ATOMIC);
	if (!context_ptr) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			("`context_ptr' memory allocation failed\n"));
		return;
	}

	context_ptr->adapter = adapter;
	
	if(adapter->Driver_Mode == SNIFFER_MODE)
		total_len = ONEBOX_RCV_BUFFER_LEN*4;
	else
		total_len = ONEBOX_RCV_BUFFER_LEN;

	context_ptr->netbuf_addr = adapter->os_intf_ops->onebox_alloc_skb(total_len);
	if (!context_ptr->netbuf_addr) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			("`netbuf_addr' memory allocation failed\n"));
		return;
	}

	adapter->os_intf_ops->onebox_add_data_to_skb(context_ptr->netbuf_addr, total_len);
	context_ptr->ep_num = ep_num;
	urb->transfer_buffer = context_ptr->netbuf_addr->data;
	adapter->total_usb_rx_urbs_submitted++;

	done_handler = onebox_rx_done_handler;

	usb_fill_bulk_urb (urb,
	                   adapter->usbdev,
	                   usb_rcvbulkpipe (adapter->usbdev,
				   adapter->bulk_in_endpointAddr[ep_num - 1]),
	                   urb->transfer_buffer, total_len, done_handler,
	                   context_ptr);

	if (usb_submit_urb(urb, GFP_ATOMIC)) 
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR,  ("submit(bulk rx_urb)"));
	}
}

/**
 * This function is called when a packet is received from 
 * the stack. This is rx done callback.
 *
 * @param
 *      URB             Received URB
 * @return         
 *      none
 */
void onebox_rx_done_handler (struct urb *urb)
{
	uint8 pkt_type = 0;
	urb_context_t *context_ptr = urb->context;
	PONEBOX_ADAPTER adapter = context_ptr->adapter;
	netbuf_ctrl_block_t *netbuf_recv_pkt = context_ptr->netbuf_addr;
 	uint8 status = ONEBOX_STATUS_FAILURE;

	switch (urb->status) {
		case 0:
			break;
		case -ECONNRESET:
		case -ENODEV:
		case -ESHUTDOWN:
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("rx_done, status %d, length %d \n", urb->status, urb->actual_length));
			adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, urb->transfer_buffer, urb->actual_length);
		case -ENOENT:
		default:
			adapter->os_intf_ops->onebox_free_pkt(netbuf_recv_pkt, 0);
			adapter->os_intf_ops->onebox_mem_free(context_ptr);
			return;
	}

	adapter->total_usb_rx_urbs_done++;
	if (urb->actual_length == 0)
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR,  ("==> ERROR: ZERO LENGTH <==\n"));
		goto resubmit;
	}

	adapter->os_intf_ops->onebox_netbuf_trim(netbuf_recv_pkt, urb->actual_length);

	if (adapter->Driver_Mode == QSPI_FLASHING ||
			adapter->flashing_mode_on) {
		pkt_type = COEX_PKT;
	} else {
		if (context_ptr->ep_num == 1) {
			pkt_type = WLAN_PKT;	
		} else if (context_ptr->ep_num == 2) {
			if (adapter->coex_mode == WIFI_BT_LE || adapter->coex_mode == WIFI_BT_CLASSIC)
				pkt_type = BT_PKT;
			else if (adapter->coex_mode == WIFI_ZIGBEE)
				pkt_type = ZIGB_PKT;
			else
				pkt_type = WLAN_PKT;
		}
	}

	netbuf_recv_pkt->rx_pkt_type = pkt_type;

	status = process_usb_rcv_pkt(adapter, netbuf_recv_pkt, pkt_type);

	if(status == ONEBOX_STATUS_FAILURE) {
		ONEBOX_DEBUG (ONEBOX_ZONE_ERROR, (TEXT("ERROR : Error in processing the received Packet\n")));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, netbuf_recv_pkt->data, netbuf_recv_pkt->len);
		adapter->os_intf_ops->onebox_mem_free(context_ptr);
		adapter->os_intf_ops->onebox_free_pkt(netbuf_recv_pkt, 0);
		return ; 
	}

	onebox_rx_urb_submit(adapter, urb, context_ptr->ep_num);
	adapter->os_intf_ops->onebox_mem_free(context_ptr);
	return;

resubmit:
	onebox_rx_urb_submit(adapter, urb, context_ptr->ep_num);
	adapter->os_intf_ops->onebox_mem_free(context_ptr);
	adapter->os_intf_ops->onebox_free_pkt(netbuf_recv_pkt, 0);
	return;
}

/**
 * This function writes multiple bytes of information to the SD card.
 *
 * @param  Pointer to Driver adapter structure.  
 * @param  Function Number.  
 * @param  Address of the register.  
 * @param  Length of the data.  
 * @param  Pointer to the data that has to be written.  
 * @return On success ONEBOX_STATUS_SUCCESS else ONEBOX_STATUS_FAILURE. 
 */
ONEBOX_STATUS usb_write_ta_register_multiple(PONEBOX_ADAPTER adapter,
                                      uint32 Addr,
                                      uint8 *data,
                                      uint32 Count
                                     )
{
	ONEBOX_STATUS status_l;
	uint8 *buf;
	uint32 transfer;
	
	//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("PROBE: In %s Line %d\n", __func__, __LINE__));
	buf = kzalloc(4096, GFP_KERNEL);
	if (!buf)
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s: Failed to allocate memory\n",__func__));
		return -ENOMEM;
	}
	
	status_l = ONEBOX_STATUS_SUCCESS;
	
	while (Count) 
	{
		transfer = min_t(int, Count, 4096);

		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("In %s Line %d transfer %d\n ",__func__,__LINE__,transfer));
		memcpy(buf, data, transfer);
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("ctrl pipe number is %8x\n",usb_sndctrlpipe(adapter->usbdev, 0)));
	
		status_l = usb_control_msg (adapter->usbdev,
		                            usb_sndctrlpipe (adapter->usbdev, 0),
		                            USB_VENDOR_REGISTER_WRITE, USB_TYPE_VENDOR,
		                            ((Addr & 0xffff0000) >> 16), (Addr & 0xffff),
		                            (void *) buf, transfer, HZ * 5);
		if (status_l < 0) 
		{
			ONEBOX_DEBUG (ONEBOX_ZONE_ERROR,
			              ("REG WRITE FAILED WITH ERROR CODE :%10d \n", status_l));
			kfree(buf);
			return status_l;
		}
		else 
		{
			Count -= transfer;
			data += transfer;
			Addr += transfer;
		}
	}
	ONEBOX_DEBUG (ONEBOX_ZONE_INFO,
	              ("REG WRITE WAS SUCCESSFUL :%10d \n", status_l));
	kfree(buf);
	return 0;
} /* End <usb_write_register_multiple> */

/**
 * This function reads multiple bytes of information to the SD card.
 *
 * @param  Pointer to Driver adapter structure.  
 * @param  Function Number.  
 * @param  Address of the register.  
 * @param  Length of the data.  
 * @param  Pointer to the data that has to be written.  
 * @return On success ONEBOX_STATUS_SUCCESS else ONEBOX_STATUS_FAILURE. 
 */
ONEBOX_STATUS usb_read_ta_register_multiple(PONEBOX_ADAPTER adapter,
                                      uint32 Addr,
                                      uint8 *data,
                                      uint32 Count
                                     )
{
	ONEBOX_STATUS status_l;
	uint8 *buf;
	uint32 transfer;
	
	//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("PROBE: In %s Line %d\n", __func__, __LINE__));
	buf = kzalloc(4096, GFP_KERNEL);
	if (!buf)
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("PROBE: %s %d Failed to allocate memory\n", __func__, __LINE__));
		return -ENOMEM;
	}
	
	status_l = ONEBOX_STATUS_SUCCESS;
	while (Count) 
	{
		transfer = min_t(int, Count, 4096);

		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("In %s Line %d transfer %d count %d \n ",__func__,__LINE__,transfer,Count));
		status_l = usb_control_msg (adapter->usbdev,
		                            usb_rcvctrlpipe (adapter->usbdev, 0),
		                            USB_VENDOR_REGISTER_READ,
	                            	    USB_TYPE_VENDOR | USB_DIR_IN | USB_RECIP_DEVICE,
		                            ((Addr & 0xffff0000) >> 16), (Addr & 0xffff),
		                            (void *) buf, transfer, HZ * 5);
		memcpy(data, buf, transfer);
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, buf, transfer);
		if (status_l < 0) 
		{
			ONEBOX_DEBUG (ONEBOX_ZONE_ERROR,
			              ("REG WRITE FAILED WITH ERROR CODE :%10d \n", status_l));
			kfree(buf);
			return status_l;
		}
		else 
		{
			Count -= transfer;
			data += transfer;
			Addr += transfer;
		}
	}
	ONEBOX_DEBUG (ONEBOX_ZONE_INFO,
	              ("MASTER READ IS SUCCESSFUL :%10d \n", status_l));
	kfree(buf);
	return 0;
} /* End <usb_write_register_multiple> */
EXPORT_SYMBOL(onebox_rx_urb_submit);
