/**
 * @file onebox_osd_main.c
 * @author 
 * @version 1.0
 *
 * @section LICENSE
 *
 * This software embodies materials and concepts that are confidential to Redpine
 * Signals and is made available solely pursuant to the terms of a written license
 * agreement with Redpine Signals
 *
 * @section DESCRIPTION
 *
 * This file contains all the Linux network device specific code.
 */
#include "zb_common.h"
#include "onebox_linux.h"
#include "onebox_sdio_intf.h"
#include "onebox_zigb_ioctl.h"

static uint8 device_mac_addr[6] = {0x00, 0x23, 0xa7, 0x27, 0x03, 0x91};

/**
 * Allocate & initialize the network device.This function
 * allocates memory for the network device & initializes 
 * it with ethernet generic values.
 *
 * @param  size of the priv area to be allocated.  
 * @return Pointer to the network device structure. 
 */
static struct net_device* onebox_allocdev(int32 sizeof_priv)
{
	struct net_device *dev = NULL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,17,0))
	dev = alloc_netdev(sizeof_priv, "zigb%d", NET_NAME_UNKNOWN, ether_setup);
#else
	dev = alloc_netdev(sizeof_priv, "zigb%d", ether_setup);
#endif
	return dev;
}/* End of <onebox_allocdev> */

/**
 * This function register the network device.
 *
 * @param  Pointer to our network device structure.  
 * @return On success 0 is returned else a negative value. 
 */
static int32 onebox_registerdev(struct net_device *dev)
{
	if (rtnl_is_locked())
		return register_netdevice(dev);
	else 
		return register_netdev(dev);
}/* End of <onebox_registerdev> */

/**
 * This function unregisters the network device & returns it 
 * back to the kernel.
 *
 * @param  Pointer to our network device structure.  
 * @return VOID. 
 */
VOID unregister_dev(struct net_device *dev)
{

	if (rtnl_is_locked())
		unregister_netdevice(dev);
	else {
		unregister_netdev(dev);
		free_netdev(dev);
	}
	return;
} /* End of <onebox_unregisterdev> */

/**
 * This function gives the statistical information regarding 
 * the interface.
 *
 * @param  Pointer to our network device strucure.  
 * @return Pointer to the net_device_stats structure . 
 */
static struct net_device_stats* onebox_get_stats(struct net_device *dev)
{
	ZB_ADAPTER Adapter = netdev_priv(dev);
	return &Adapter->stats;
} /* End of <onebox_get_stats> */

/**
 * This function performs all net device related operations
 * like allocating,initializing and registering the netdevice.
 *
 * @param  VOID.  
 * @return On success 0 is returned else a negative value. 
 */
struct net_device* zigb_netdevice_op(VOID)
{    
	struct net_device *dev; 

#if KERNEL_VERSION_BTWN_2_6_(18, 27)
	/*Allocate & initialize the network device structure*/  
	dev = onebox_allocdev(sizeof(struct zb_priv));    
	if (dev == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s:Failure in allocation of net-device\n"),__func__));
		return dev;
	}

	dev->open                 = device_open;
	dev->stop                 = device_close;
	dev->hard_start_xmit      = zigb_xmit;
	dev->get_stats            = onebox_get_stats;   
	dev->do_ioctl             = zigb_ioctl;
	//dev->wireless_handlers    = &onebox_handler_def;  
	dev->hard_header_len      = FRAME_DESC_SZ + RSI_DESCRIPTOR_SZ; /* 64 + 16 */

	if (tcp_csumoffld_enable)
	{
		dev->features |= NETIF_F_IP_CSUM;
	}
	if (onebox_registerdev(dev) != 0)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Registration of net-device failed\n"), __func__));
		free_netdev(dev);
		return NULL;      
	}
#elif KERNEL_VERSION_GREATER_THAN_2_6_(27)
	static const struct net_device_ops zigb_netdev_ops =
	{
		.ndo_open           = device_open,
		.ndo_stop           = device_close,
		.ndo_start_xmit     = zigb_xmit,
		.ndo_get_stats      = onebox_get_stats,
		.ndo_do_ioctl       = zigb_ioctl,
	};
	
	/*Allocate & initialize the network device structure*/
	dev = onebox_allocdev(sizeof(struct zb_priv));    
	if (dev == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Failure in allocation of net-device\n"),
		              __func__));
		return dev;
	}

	//dev->wireless_handlers    = &onebox_handler_def;
	dev->hard_header_len    = FRAME_DESC_SZ + ONEBOX_DESCRIPTOR_SZ; /* 64 + 16 */
	dev->netdev_ops         = &zigb_netdev_ops;

	if (onebox_registerdev(dev) != 0)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Registration of net-device failed\n"),
		              __func__));
		free_netdev(dev);
		return NULL;
	}
#endif
	return dev;
}/* End of <zigb_netdevice_op> */

static ONEBOX_STATUS zigb_gpl_read_pkt(struct driver_assets *d_assets,
				       netbuf_ctrl_block_t *netbuf_cb)
{
	ZB_ADAPTER z_adapter = (ZB_ADAPTER)d_assets->techs[ZB_ID].priv;

	z_adapter->os_intf_ops->onebox_acquire_sem(&z_adapter->zigb_gpl_lock, 0);
	if (d_assets->techs[ZB_ID].drv_state == MODULE_ACTIVE) {
		zigb_read_pkt(z_adapter, netbuf_cb);
	} else {
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ZIGB is being removed.. Dropping Pkt\n")));
		z_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
		netbuf_cb = NULL;
	}

	z_adapter->os_intf_ops->onebox_release_sem(&z_adapter->zigb_gpl_lock);
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function is called by OS when we UP the
 * interface.
 *
 * @param  Pointer to network device
 * @return Success 
 */
int device_open(struct net_device *dev)
{
	//netif_start_queue(dev);
	dev->flags |=  IFF_RUNNING;
	return 0;
}

/**
 * This function is called by OS when the interface
 * status changed to DOWN.
 *
 * @param  dev  Pointer to network device
 */
int device_close(struct net_device *dev)
{
	if (!netif_queue_stopped(dev))
	{
		netif_stop_queue(dev);
	}
	return 0;
}

/**
 * Os sends packet to driver using this function.
 * @param  Pointer to struct sk_buff containing the payload
 *      to be transmitted.
 * @param  Pointer to network device
 */
int zigb_xmit(struct sk_buff *skb, struct net_device *dev)
{
	/* Dummy for Zigbee currently */
	return ONEBOX_STATUS_SUCCESS;
}

/*
 * This function deregisters ZIGB firmware
 * @param  Pointer to adapter structure.  
 * @return 0 if success else -1. 
 */

static ONEBOX_STATUS zigb_deregister_fw(ZB_ADAPTER zb_adapter)
{
	uint8 *frame_desc;
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	struct driver_assets *d_assets = zb_adapter->d_assets;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("===> Deregister ZIGB FW <===\n")));

	netbuf_cb = zb_adapter->os_intf_ops->onebox_alloc_skb(FRAME_DESC_SZ);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;

	}
	zb_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, FRAME_DESC_SZ);

	zb_adapter->os_intf_ops->onebox_memset(netbuf_cb->data, 0, FRAME_DESC_SZ);
	
	frame_desc = (uint8 *)netbuf_cb->data;

	frame_desc[12] = 0x0; /* Sequence Number */
	frame_desc[13] = 0x1; /* Direction */
	frame_desc[14] = 0x7; /* Interface */
	frame_desc[15] = ZIGB_DEREGISTER; /* Packet Type */
	netbuf_cb->tx_pkt_type = ZIGB_TX_Q;
	status = zb_adapter->onebox_send_pkt_to_coex(zb_adapter->d_assets, netbuf_cb, VIP_Q);
	if (status != ONEBOX_STATUS_SUCCESS) 
	{ 
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT 
			     ("%s: Failed To Write The Packet\n"),__func__));
	}
	ZB_TECH.tx_intention = 0;
	ZB_TECH.tx_access = 0;
	d_assets->update_tx_status(d_assets, ZB_ID);
	d_assets->techs[ZB_ID].fw_state = FW_INACTIVE;
	zb_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
  	return status;
}

/**
 * This function is triggered whenever zigb module is 
 * inserted. Links zigb module with hal module 
 * work is done here.
 *
 * @return 
 */
int32 zigb_insert(struct driver_assets *d_assets)
{
	int32 rc = 0;
	struct net_device *dev = NULL;
	ZB_ADAPTER zb_adapter;

	//struct onebox_osi_host_intf_operations *osi_host_intf_ops = onebox_get_osi_host_intf_operations();
	//struct onebox_osd_host_intf_operations *osd_host_intf_ops = onebox_get_osd_host_intf_operations();
	struct onebox_os_intf_operations *os_intf_ops = onebox_get_os_intf_operations_from_origin();
	struct onebox_zigb_osd_operations *osd_zigb_ops = onebox_get_zigb_osd_ops();
	struct onebox_osi_zigb_ops *osi_zigb_ops = onebox_get_osi_zigb_ops();

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Initialization function called\n"), __func__));
	os_intf_ops->onebox_acquire_sem(&d_assets->zigbee_init_lock, 0);
	if(d_assets->techs[ZB_ID].drv_state == MODULE_ACTIVE) {
		
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d Zigbee Module is already initialized\n"), __func__, __LINE__));
	os_intf_ops->onebox_release_sem(&d_assets->zigbee_init_lock);
	return ONEBOX_STATUS_SUCCESS;
	}

	dev = zigb_netdevice_op();
	if (!dev) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Failed to perform netdevice operations\n"), __func__));
		goto nodev;
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
	            (TEXT("%s: Net device operations suceeded\n"), __func__));
	zb_adapter = os_intf_ops->onebox_get_priv(dev); /*we can also use dev->priv;*/

	os_intf_ops->onebox_memset(zb_adapter, 0, sizeof(struct zb_priv));

	/* Initialise the Core and device dependent operations */
	zb_adapter->osi_zigb_ops      = osi_zigb_ops;
	zb_adapter->zigb_osd_ops      = osd_zigb_ops;
	//zb_adapter->osd_host_intf_ops = osd_host_intf_ops;
	//zb_adapter->osi_host_intf_ops = osi_host_intf_ops;
	zb_adapter->os_intf_ops       = os_intf_ops;
	zb_adapter->d_assets       = d_assets;

	os_intf_ops->onebox_init_dyn_mutex(&zb_adapter->zigb_gpl_lock);

	zb_adapter->dev = dev;

	zb_adapter->os_intf_ops->onebox_memcpy(zb_adapter->mac_addr, device_mac_addr, ETH_ALEN);
	zb_adapter->os_intf_ops->onebox_memcpy(zb_adapter->dev->dev_addr, zb_adapter->mac_addr, ETH_ALEN);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Mutex init successfull\n"), __func__));

	d_assets->techs[ZB_ID].priv = (void *)zb_adapter;
	zb_adapter->onebox_send_pkt_to_coex = d_assets->common_send_pkt_to_coex;

  	os_intf_ops->onebox_init_event(&(zb_adapter->zb_per_event));

	os_intf_ops->onebox_strcpy(zb_adapter->name, "onebox_zigb"); 

	zb_adapter->fsm_state = FSM_DEVICE_READY;

  	zb_adapter->driver_mode = d_assets->asset_role;

	init_waitqueue_head(&d_assets->techs[ZB_ID].deregister_event);
	rc = zb_adapter->osi_zigb_ops->onebox_core_init(zb_adapter);
	if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: failed to init ZIGBEE, error[%d]\n"), __func__, rc));
		goto nocoreinit;
	}

	d_assets->techs[ZB_ID].drv_state = MODULE_ACTIVE;
	d_assets->techs[ZB_ID].default_ps_en = 1;

	rc = setup_zigb_procfs(zb_adapter);
	if (rc) 
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: failed to init zigbee procfs entry, error[%d]\n"), __func__, rc));

	os_intf_ops->onebox_release_sem(&d_assets->zigbee_init_lock);
	return ONEBOX_STATUS_SUCCESS;    

nocoreinit:
	if (dev)
		unregister_dev(dev);
nodev:
	os_intf_ops->onebox_release_sem(&d_assets->zigbee_init_lock);
	return -ENOMEM;    
} /* End <zigb_insert> */

/**
 * This function removes the zigb module safely..
 *
 * @param  Pointer to sdio_func structure.  
 * @param  Pointer to sdio_device_id structure.  
 * @return VOID. 
 */
int32 zigb_remove(struct driver_assets *d_assets)
{
	ZB_ADAPTER zb_adapter = d_assets->techs[ZB_ID].priv;
	struct net_device *dev = zb_adapter->dev;
	struct wireless_techs *zb_d;

	zb_adapter->os_intf_ops->onebox_acquire_sem(&zb_adapter->zigb_gpl_lock, 0);

	if (d_assets->card_state != GS_CARD_DETACH) {
		zb_d = &d_assets->techs[ZB_ID];
		zb_d->tx_intention = 1;
		d_assets->update_tx_status(d_assets, ZB_ID);
		if(!zb_d->tx_access) {
			d_assets->techs[ZB_ID].deregister_flags = 1;
			if(wait_event_timeout((zb_d->deregister_event), 
						(d_assets->techs[ZB_ID].deregister_flags == 0),
						msecs_to_jiffies(6000))) {
				zigb_deregister_fw(zb_adapter);	
			}	else {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Failed to get sleep exit\n")));
			}
		} else
			zigb_deregister_fw(zb_adapter);	
	}


	destroy_zigb_procfs(zb_adapter);	

	zb_adapter->osi_zigb_ops->onebox_core_deinit(zb_adapter);
	zb_adapter->os_intf_ops->onebox_release_sem(&zb_adapter->zigb_gpl_lock);
        /*Return the network device to the kernel*/
	unregister_dev(dev);
	d_assets->techs[ZB_ID].drv_state = MODULE_INSERTED;

	return ONEBOX_STATUS_SUCCESS;
}/* End <zigb_remove> */

ONEBOX_STATUS zb_module_init(struct driver_assets *d_assets)
{
	int rc = 0;
	d_assets->techs[ZB_ID].drv_state = MODULE_INSERTED;
	d_assets->techs[ZB_ID].inaugurate = zigb_insert;
	d_assets->techs[ZB_ID].disconnect = zigb_remove;
	d_assets->techs[ZB_ID].onebox_get_pkt_from_coex = zigb_gpl_read_pkt;
	
#if 1
	/** If WLAN is not enabled then initialize ZIGB */
	if ((d_assets->card_state == GS_CARD_ABOARD) && 
			!(d_assets->oper_mode & (3/*OP_WLAN_STA_MODE | OP_WLAN_AP_MODE*/))) {
		if(d_assets->techs[ZB_ID].fw_state == FW_ACTIVE) {
			rc = zigb_insert(d_assets);
			if (rc) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: failed to insert "
								"zigb error[%d]\n"),__func__, rc)); 
				return 0;
			}
		}
	}	
#endif
	
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ZIGB : Zigb gpl installed\n ")));
	return 0;
}

ONEBOX_STATUS zb_module_deinit(struct driver_assets *d_assets)
{
	if (d_assets->techs[ZB_ID].drv_state == MODULE_ACTIVE) {
		//d_assets->techs[ZB_ID].drv_state = MODULE_REMOVED;
		zigb_remove(d_assets);
	} else {
		d_assets->techs[ZB_ID].drv_state = MODULE_REMOVED;
	}
	
	d_assets->techs[ZB_ID].inaugurate = NULL;
	d_assets->techs[ZB_ID].disconnect = NULL;
	d_assets->techs[ZB_ID].onebox_get_pkt_from_coex = NULL;

	return 0;
}

ONEBOX_STATIC int32 onebox_zigbgpl_module_init(VOID)
{
	return 0;
}

ONEBOX_STATIC VOID onebox_zigbgpl_module_exit(VOID)
{
	return;
}
EXPORT_SYMBOL(zb_module_init);
EXPORT_SYMBOL(zb_module_deinit);

module_init(onebox_zigbgpl_module_init);
module_exit(onebox_zigbgpl_module_exit);

MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("Redpine Signals, Inc.");
MODULE_DESCRIPTION("Driver for Redpine Signals' RS9113 module based USB/SDIO cards.");
MODULE_SUPPORTED_DEVICE("Redpine Signals' RS9113 module based USB/SDIO cards.");
