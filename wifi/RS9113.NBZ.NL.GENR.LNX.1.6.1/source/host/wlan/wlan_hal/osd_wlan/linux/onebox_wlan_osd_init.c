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
#include "onebox_linux.h"
#include "onebox_sdio_intf.h"

static uint32 tcp_csumoffld_enable =0;

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

#if ((LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)))
	dev = alloc_netdev(sizeof_priv, "rpine%d", ether_setup);
#else
	dev = alloc_netdev(sizeof_priv, "rpine%d", NET_NAME_UNKNOWN, ether_setup);
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

	if (dev->reg_state == NETREG_REGISTERED) {
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
	WLAN_ADAPTER Adapter = netdev_priv(dev);
	return &Adapter->stats;
} /* End of <onebox_get_stats> */

#define ONEBOX_IOCTL_PROTO	SIOCIWFIRSTPRIV + 21
#define ONEBOX_IOCTL_WATCH	SIOCIWLASTPRIV - 0xd
#define ONEBOX_IOCTL_VAPCREATE	SIOCIWLASTPRIV - 0xf
#define ONEBOX_IOCTL_VAPDELETE	SIOCIWLASTPRIV - 0x10
#define ONEBOX_IOCTL_WLAN_STATS	SIOCIWLASTPRIV - 0xc
#define ONEBOX_IOCTL_HOST	SIOCIWLASTPRIV - 0x0B
#define ONEBOX_IOCTL_SET_BBRF	SIOCIWLASTPRIV - 0x08
#define ONEBOX_IOCTL_SET_CWMODE	SIOCIWLASTPRIV - 0x05
static int
onebox_ioctl_proto(struct net_device *dev,
                       struct iw_request_info *info,
                       struct iw_point *w, 
                       char *extra)
{
	struct iwreq frq;
	frq.u.data.flags = w->flags;
	frq.u.data.length = w->length;
	if(w->pointer)
		frq.u.data.pointer = w->pointer;

	return onebox_ioctl (dev, (struct ifreq *)&frq, ONEBOX_IOCTL_PROTO);
}

static int
onebox_ioctl_watch(struct net_device *dev,
                       struct iw_request_info *info,
                       struct iw_point *w, 
                       char *extra)
{
	struct iwreq frq;
	frq.u.data.flags = w->flags;
	frq.u.data.length = w->length;
	return onebox_ioctl (dev, (struct ifreq *)&frq, ONEBOX_IOCTL_WATCH);
}

static int
onebox_ioctl_vap_create(struct net_device *dev,
                       struct iw_request_info *info,
                       struct iw_point *w, 
                       char *extra)
{
	struct iwreq frq;
	frq.u.data.pointer = w->pointer;
	return onebox_ioctl (dev, (struct ifreq *)&frq, ONEBOX_IOCTL_VAPCREATE);
}

static int
onebox_ioctl_vap_delete(struct net_device *dev,
                       struct iw_request_info *info,
                       struct iw_point *w, 
                       char *extra)
{
	struct iwreq frq;
	frq.u.data.pointer = w->pointer;
	return onebox_ioctl (dev, (struct ifreq *)&frq, ONEBOX_IOCTL_VAPDELETE);
}

static int
onebox_ioctl_wlan_stats(struct net_device *dev,
                       struct iw_request_info *info,
                       struct iw_point *w, 
                       char *extra)
{
	struct iwreq frq;
	frq.u.data.flags = w->flags;
	frq.u.data.length = w->length;
	return onebox_ioctl (dev, (struct ifreq *)&frq, ONEBOX_IOCTL_WLAN_STATS);
}

static int
onebox_ioctl_host(struct net_device *dev,
                       struct iw_request_info *info,
                       struct iw_point *w, 
                       char *extra)
{
	struct iwreq frq;
	frq.u.data.flags = w->flags;
	if(w->pointer)
		frq.u.data.pointer = w->pointer;
	if(w->length)
		frq.u.data.length = w->length;
	return onebox_ioctl (dev, (struct ifreq *)&frq, ONEBOX_IOCTL_HOST);
}

static int
onebox_ioctl_set_bbrf(struct net_device *dev,
                       struct iw_request_info *info,
                       struct iw_point *w, 
                       char *extra)
{
	struct iwreq frq;
	frq.u.data.pointer = w->pointer;
	frq.u.data.length = w->length;
	if(w->flags)
		frq.u.data.flags = w->flags;
	return onebox_ioctl (dev, (struct ifreq *)&frq, ONEBOX_IOCTL_SET_BBRF);
}

static int
onebox_ioctl_set_cw_mode(struct net_device *dev,
                       struct iw_request_info *info,
                       struct iw_point *w, 
                       char *extra)
{
	struct iwreq frq;
	frq.u.data.flags = w->flags;
	return onebox_ioctl (dev, (struct ifreq *)&frq, ONEBOX_IOCTL_SET_CWMODE);
}

#define set_priv(x,f) [x - SIOCIWFIRSTPRIV] = (iw_handler) f
static const iw_handler onebox_priv_handlers[] = 
{
        set_priv(ONEBOX_IOCTL_PROTO, onebox_ioctl_proto),
        set_priv(ONEBOX_IOCTL_WATCH, onebox_ioctl_watch),
        set_priv(ONEBOX_IOCTL_VAPCREATE, onebox_ioctl_vap_create),
        set_priv(ONEBOX_IOCTL_VAPDELETE, onebox_ioctl_vap_delete),
        set_priv(ONEBOX_IOCTL_WLAN_STATS, onebox_ioctl_wlan_stats),
        set_priv(ONEBOX_IOCTL_HOST, onebox_ioctl_host),
        set_priv(ONEBOX_IOCTL_SET_BBRF, onebox_ioctl_set_bbrf),
        set_priv(ONEBOX_IOCTL_SET_CWMODE, onebox_ioctl_set_cw_mode),
};

static const struct iw_priv_args onebox_priv_args[] =
{
        /* NB: setoptie & getoptie are !IW_PRIV_SIZE_FIXED */
        { ONEBOX_IOCTL_PROTO,
          0X6000 | 0x0800 | 1, 0,"proto" },
	{ ONEBOX_IOCTL_WATCH,
          0X6000 | 0x0800 | 1, 0,"watch" },
	{ ONEBOX_IOCTL_VAPCREATE,
          0X6000 | 0x0800 | 1, 0,"vap_create" },
	{ ONEBOX_IOCTL_VAPDELETE,
          0X6000 | 0x0800 | 1, 0,"vap_delete" },
	{ ONEBOX_IOCTL_WLAN_STATS,
          0X6000 | 0x0800 | 1, 0,"wlan_iq_stats" },
	{ ONEBOX_IOCTL_HOST,
          0X6000 | 0x0800 | 1, 0,"host" },
	{ ONEBOX_IOCTL_SET_BBRF,
          0X6000 | 0x0800 | 1, 0,"set_bb_rf" },
	{ ONEBOX_IOCTL_SET_CWMODE,
          0X6000 | 0x0800 | 1, 0,"set_cw_mode" },
};

struct iw_handler_def onebox_iw_handler_def = 
{
#ifdef CONFIG_WEXT_PRIV
        .private = (iw_handler *) onebox_priv_handlers,
        .num_private = ARRAY_SIZE(onebox_priv_handlers),
        .private_args = (struct iw_priv_args *) onebox_priv_args,
        .num_private_args = ARRAY_SIZE(onebox_priv_args),
#endif
};

/**
 * This function performs all net device related operations
 * like allocating,initializing and registering the netdevice.
 *
 * @param  VOID.  
 * @return On success 0 is returned else a negative value. 
 */

struct net_device* wlan_netdevice_op(VOID)
{    
	struct net_device *dev; 

#if KERNEL_VERSION_BTWN_2_6_(18, 27)
	/*Allocate & initialize the network device structure*/  
	dev = onebox_allocdev(sizeof(struct wlan_priv));    
	if (dev == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s:Failure in allocation of net-device\n"),__func__));
		return dev;
	}

	dev->open                 = device_open;
	dev->stop                 = device_close;
	dev->hard_start_xmit      = onebox_xmit;
	dev->get_stats            = onebox_get_stats;   
	dev->do_ioctl             = onebox_ioctl;
#ifdef CONFIG_WIRELESS_EXT
	dev->wireless_handlers    = &onebox_iw_handler_def;
#endif
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
	static const struct net_device_ops onebox_netdev_ops =
	{
		.ndo_open           = device_open,
		.ndo_stop           = device_close,
		.ndo_start_xmit     = onebox_xmit,
		.ndo_get_stats      = onebox_get_stats,
		.ndo_do_ioctl       = onebox_ioctl,
	};
	
	/*Allocate & initialize the network device structure*/
	dev = onebox_allocdev(sizeof(struct wlan_priv));    
	if (dev == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Failure in allocation of net-device\n"),
		              __func__));
		goto noalloc;
	}
#ifdef CONFIG_WIRELESS_EXT
	dev->wireless_handlers    = &onebox_iw_handler_def;
#endif

	//dev->wireless_handlers    = &onebox_handler_def;
	dev->hard_header_len    = FRAME_DESC_SZ + ONEBOX_DESCRIPTOR_SZ; /* 64 + 16 */
	dev->netdev_ops         = &onebox_netdev_ops;

	dev->if_flags |= IFF_SLAVE;

	if (tcp_csumoffld_enable)
	{
		dev->features          |= NETIF_F_IP_CSUM;
	}

	if (onebox_registerdev(dev) != 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Registration of net-device failed\n"),
		              __func__));
		goto nodev;
	}
	return dev;
nodev:
	free_netdev(dev);
	dev = NULL;
noalloc:
	return dev;
#endif
}/* End of <wlan_netdevice_op> */

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

static ONEBOX_STATUS wlan_gpl_read_pkt(struct driver_assets *d_assets, netbuf_ctrl_block_t *netbuf_cb)
{
	WLAN_ADAPTER w_adapter;

	w_adapter = (WLAN_ADAPTER)d_assets->techs[WLAN_ID].priv;

	w_adapter->os_intf_ops->onebox_acquire_sem(&w_adapter->wlan_gpl_lock, 0);
	if (d_assets->techs[WLAN_ID].drv_state == MODULE_ACTIVE) {
		wlan_read_pkt(w_adapter, netbuf_cb);
	} else {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("WLAN is being removed.. Dropping Pkt\n")));
		w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
	}
	w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->wlan_gpl_lock);
	return ONEBOX_STATUS_SUCCESS;
}

static void dump_wlan_mgmt_pending_pkts(struct driver_assets *d_assets)
{

	netbuf_ctrl_block_t *netbuf_cb = NULL;
	WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)d_assets->techs[WLAN_ID].priv;

	for(;;)
	{	
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
					(TEXT("%s:%d ==> PKT IN MGMT QUEUE count %d  <==\n"), __func__, __LINE__, w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[MGMT_SOFT_Q])));
		if (w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[MGMT_SOFT_Q]))
		{
			//netbuf_cb = core_dequeue_pkt(w_adapter, MGMT_SOFT_Q);
			netbuf_cb = w_adapter->os_intf_ops->onebox_dequeue_pkt((void *)&w_adapter->host_tx_queue[MGMT_SOFT_Q]);
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
					(TEXT("%s:%d ==> PKT IN MGMT QUEUE <==\n"), __func__, __LINE__));
			if(netbuf_cb){
				w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (uint8 *)netbuf_cb->data, netbuf_cb->len);
				w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
			}
		}
		else{
			break;
		}
	}

}

static ONEBOX_STATUS wlan_update_buf_status(uint8 device_buf_status, struct driver_assets *d_assets)
{
	WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)d_assets->techs[WLAN_ID].priv;

	if (d_assets->techs[WLAN_ID].drv_state == MODULE_ACTIVE) {
		
		//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("_UBS_")));
		w_adapter->buf_status_updated = 1;
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
			w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->sdio_scheduler_event));
			/* This Interrupt Is Recvd When The Hardware Buffer Is Empty */
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, 
				(TEXT("%s: ==> BUFFER_AVILABLE <==\n"), __func__));
			w_adapter->buf_avilable_counter++;
	} else {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("WLAN not active and hence no buffer updates\n")));
	}
	return ONEBOX_STATUS_SUCCESS;
}
/**
 * Os sends packet to driver using this function.
 * @param  Pointer to struct sk_buff containing the payload
 *      to be transmitted.
 * @param  Pointer to network device
 */
int onebox_xmit(struct sk_buff *skb, struct net_device *dev)
{
	void * ni;
	uint16 flags;
	WLAN_ADAPTER w_adapter;
	netbuf_ctrl_block_t *netbuf_cb;
	w_adapter = netdev_priv(dev);

	FUNCTION_ENTRY(ONEBOX_ZONE_DATA_SEND);


	/* Store the required skb->cb content temporarily before modifying netbuf_cb
	 * because both of them have the same memory */

	ni =(void *)SKB_CB(skb)->ni;
	flags =(uint16)SKB_CB(skb)->flags;

	/* Now assign the required fields of netbuf_cb for hal operations */
	netbuf_cb = kmalloc(sizeof(netbuf_ctrl_block_t), GFP_ATOMIC);
	netbuf_cb->ni = ni;
	netbuf_cb->flags = flags;

	netbuf_cb->skb_priority = skb_get_queue_mapping(skb);

	netbuf_cb->pkt_addr = (void *)skb;
	netbuf_cb->len = skb->len;
	netbuf_cb->data = skb->data;
	netbuf_cb->priority = skb->priority;
	*((unsigned long int *)skb->cb) = (unsigned long)netbuf_cb;
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DATA_SEND, netbuf_cb->data, netbuf_cb->len);
	w_adapter->core_ops->onebox_core_xmit(w_adapter, netbuf_cb);
	return ONEBOX_STATUS_SUCCESS;
}

/*
 * This function deregisters WLAN firmware
 * @param  Pointer to w_adapter structure.  
 * @return 0 if success else -1. 
 */

static ONEBOX_STATUS wlan_deregister_fw(WLAN_ADAPTER w_adapter)
{
	onebox_mac_frame_t *mgmt_frame;
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("===> Deregister WLAN FW <===\n")));

	netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(FRAME_DESC_SZ);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;

	}
	w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, FRAME_DESC_SZ);
	mgmt_frame = (onebox_mac_frame_t *)netbuf_cb->data;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

	/* FrameType*/
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(WLAN_DE_REGISTER);
#define IMMEDIATE_WAKEUP 1
	mgmt_frame->desc_word[0] = ((ONEBOX_WIFI_MGMT_Q << 12)| (IMMEDIATE_WAKEUP << 15));
	netbuf_cb->tx_pkt_type = WLAN_TX_M_Q;

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<==== DEREGISTER FRAME ====>\n")));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)mgmt_frame, FRAME_DESC_SZ);
	status = w_adapter->onebox_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, WLAN_Q);
	if (status != ONEBOX_STATUS_SUCCESS) 
	{ 
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT 
			     ("%s: %d Failed To Write The Packet\n"),__func__, __LINE__));
	}
#undef IMMEDIATE_WAKEUP
	if(w_adapter->d_assets->host_intf_type == HOST_INTF_SDIO)
		w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
	return status;
}

/**
 * This function is triggered whenever wlan module is 
 * inserted. Links wlan module with hal module 
 * work is done here.
 *
 * @return 
 */
void wlan_schedule_pkt_for_tx(struct driver_assets *d_assets)
{
  WLAN_ADAPTER w_adapter = d_assets->techs[WLAN_ID].priv; /*we can also use dev->priv;*/
  
  w_adapter->devdep_ops->onebox_schedule_pkt_for_tx(w_adapter);
}

int32 wlan_insert(struct driver_assets *d_assets)
{
	struct net_device *dev = NULL;
	uint32 count =0;
	WLAN_ADAPTER w_adapter = NULL;

	struct onebox_core_operations *core_ops = onebox_get_core_wlan_operations();
	struct onebox_net80211_operations *net80211_ops = onebox_get_net80211_operations();
	struct onebox_devdep_operations *devdep_ops = onebox_get_devdep_wlan_operations();
	struct onebox_os_intf_operations *os_intf_ops = onebox_get_os_intf_operations_from_origin();
	struct ieee80211_rate_ops *core_rate_ops = onebox_get_ieee80211_rate_ops();
	struct onebox_wlan_osd_operations *wlan_osd_ops = onebox_get_wlan_osd_operations_from_origin();

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("In %s Line %d: initializing WLAN Layer\n"),__func__, __LINE__));

	os_intf_ops->onebox_acquire_sem(&d_assets->wlan_init_lock, 0);

	if(d_assets->techs[WLAN_ID].drv_state == MODULE_ACTIVE) {
		
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d WLAN Module is already initialized\n"), __func__, __LINE__));
	os_intf_ops->onebox_release_sem(&d_assets->wlan_init_lock);
	return ONEBOX_STATUS_FAILURE;
	}
	dev = wlan_netdevice_op();
	if (!dev) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Failed to initialize a network device\n"), __func__));
		goto nodev;
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
	             (TEXT("%s: Net device operations suceeded\n"), __func__));
	w_adapter = os_intf_ops->onebox_get_priv(dev); /*we can also use dev->priv;*/

	os_intf_ops->onebox_memset(w_adapter, 0, sizeof(struct wlan_priv));

	/* Initialise the Core and device dependent operations */
	w_adapter->core_ops          = core_ops;
	w_adapter->net80211_ops      = net80211_ops;
	w_adapter->devdep_ops        = devdep_ops;
	w_adapter->os_intf_ops       = os_intf_ops;
	w_adapter->core_rate_ops     = core_rate_ops;
	w_adapter->wlan_osd_ops      = wlan_osd_ops;

	w_adapter->dev = dev;
	w_adapter->d_assets = d_assets;
	w_adapter->device_model = d_assets->device_model;

#ifdef RADAR_AUTO
	os_intf_ops->onebox_init_spinlock(&w_adapter->radar_lock);
#endif
	os_intf_ops->onebox_init_dyn_mutex(&w_adapter->wlan_gpl_lock);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Mutex init successfull\n"), __func__));

	os_intf_ops->onebox_init_event(&(w_adapter->sdio_scheduler_event));
	os_intf_ops->onebox_init_event(&(w_adapter->stats_event));
	os_intf_ops->onebox_init_event(&(w_adapter->bb_rf_event));
	os_intf_ops->onebox_init_event(&(w_adapter->scan_check_event));
  os_intf_ops->onebox_init_event(&(w_adapter->wlan_iqs_event));
#ifdef ONEBOX_CONFIG_PUF
	os_intf_ops->onebox_init_event(&(w_adapter->d_assets->puf_event));
#endif

	d_assets->techs[WLAN_ID].priv = (void *)w_adapter;
	if (d_assets->host_intf_type == HOST_INTF_SDIO) {
	w_adapter->sdio_pfunction              = (struct sdio_func *)d_assets->pfunc;
	w_adapter->buffer_status_register = ONEBOX_DEVICE_BUFFER_STATUS_REGISTER; 
	} else {
	w_adapter->usb_pfunction            = (struct usb_interface *)d_assets->pfunc;
	w_adapter->buffer_status_register = d_assets->techs[WLAN_ID].buffer_status_reg_addr; 
	}

	if (d_assets->host_intf_type ==  HOST_INTF_SDIO)
		w_adapter->device = &w_adapter->sdio_pfunction->dev;
	else
		w_adapter->device = &w_adapter->usb_pfunction->dev;

	w_adapter->onebox_send_pkt_to_coex = d_assets->common_send_pkt_to_coex;

	if (wlan_osd_ops->onebox_init_wlan_thread(&(w_adapter->sdio_scheduler_thread_handle),
					          "WLAN-Thread",
					          0,
					          devdep_ops->onebox_sdio_scheduler_thread, 
					          w_adapter) != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Unable to initialize wlan thread\n"), __func__));
		goto fail;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Initialized wlan thread & Event\n"), __func__));

	/* start the transmit thread */
	os_intf_ops->onebox_start_thread( &(w_adapter->sdio_scheduler_thread_handle));

	os_intf_ops->onebox_strcpy(w_adapter->name, "onebox-mobile"); 
	os_intf_ops->onebox_start_netq(w_adapter->dev);
	init_waitqueue_head(&d_assets->techs[WLAN_ID].deregister_event);
	
	/*: why is read reg parameters not called from here?? */	
	//read_reg_parameters(w_adapter);
	w_adapter->recv_channel = 1;
	w_adapter->RFType = ONEBOX_RF_8111;
	w_adapter->def_chwidth = BW_20Mhz;
  /*The operating band will be later initialized based on the band from eeprom reads */
	w_adapter->operating_band = BAND_2_4GHZ;
	w_adapter->operating_chwidth = BW_20Mhz;
	w_adapter->calib_mode  = 0;
	w_adapter->per_lpbk_mode  = 0;
	if(d_assets->coex_mode == 1/*WIFI_ALONE*/) {
		w_adapter->aggr_limit.tx_limit = 8;
		w_adapter->aggr_limit.rx_limit = 8;
	} else {
		w_adapter->aggr_limit.tx_limit = 5;
		w_adapter->aggr_limit.rx_limit = 8;
	}
	w_adapter->Driver_Mode = d_assets->asset_role;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Driver mode in WLAN is %d\n"), w_adapter->Driver_Mode));
	if (w_adapter->Driver_Mode == RF_EVAL_LPBK_CALIB)
	{
		/*: Try to optimize these conditions */
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: RF Eval LPBK CALIB mode on\n"), __func__));    
		w_adapter->Driver_Mode = RF_EVAL_MODE_ON; /* RF EVAL mode */
		w_adapter->calib_mode  = 1;
		w_adapter->per_lpbk_mode  = 1;
	} 
	else if (w_adapter->Driver_Mode == RF_EVAL_LPBK)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: RF Eval LPBK mode on\n"), __func__));    
		w_adapter->Driver_Mode = RF_EVAL_MODE_ON; /* RF EVAL mode */
		w_adapter->per_lpbk_mode  = 1;
	} 
	w_adapter->PassiveScanEnable = ONEBOX_FALSE;
	w_adapter->config_params.BT_coexistence = 0;
	w_adapter->isMobileDevice = ONEBOX_TRUE;
	w_adapter->max_stations_supported  = MAX_STATIONS_SUPPORTED;

	if (d_assets->antenna_diversity) {
		w_adapter->antenna_diversity = true;
		if (d_assets->obm_ant_sel_val)
			w_adapter->antenna_in_use = d_assets->obm_ant_sel_val;
		else
			w_adapter->antenna_in_use = RSI_INTERNAL_ANTENNA;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ANTENNA DIVERSITY is enabled\n")));
	}

	//w_adapter->os_intf_ops->onebox_memcpy(w_adapter->mac_addr, device_mac_addr, ETH_ALEN);
	//w_adapter->os_intf_ops->onebox_memcpy(w_adapter->dev->dev_addr, w_adapter->mac_addr, ETH_ALEN);
	
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" %s:Initialising wlan tx soft queues\n"), __func__));
	/* initializing tx soft queues */
	for (count = 0; count < NUM_SOFT_QUEUES; count++) 
		w_adapter->os_intf_ops->onebox_netbuf_queue_init(&w_adapter->host_tx_queue[count]);

	if (onebox_umac_init_done(w_adapter)!= ONEBOX_STATUS_SUCCESS)
		goto fail;

	w_adapter->os_intf_ops->onebox_init_event(&(d_assets->iap_event));
	d_assets->techs[WLAN_ID].drv_state = MODULE_ACTIVE;
#if 0
	struct wireless_techs *wlan_d;
	wlan_d = &d_assets->techs[WLAN_ID];
	wlan_d->tx_intention = 1;
	while(1) {
		msleep(1);
		d_assets->update_tx_status(d_assets, WLAN_ID);
		if(wlan_d->tx_access)
			break;
		count++;
		if(count > 500)
			break;
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Waiting for TX access\n")));
	}

	if (!wlan_d->tx_access) {
		d_assets->techs[WLAN_ID].deregister_flags = 1;
		if (wait_event_timeout((wlan_d->deregister_event), (d_assets->techs[WLAN_ID].deregister_flags == 0), msecs_to_jiffies(6000) )) {
		} else {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERR: In %s Line %d Initialization of WLAN Failed as Wlan TX access is not granted from Common Hal \n"), __func__, __LINE__));
			goto fail;
		}
	}
#endif

  d_assets->techs[WLAN_ID].schedule_pkt_tx = &wlan_schedule_pkt_for_tx;
	w_adapter->fsm_state = FSM_FW_LOADED;
#if 0
	if (onebox_load_bootup_params(w_adapter) != ONEBOX_STATUS_SUCCESS) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: Failed to load bootup parameters\n"), 
			     __func__));
		goto fail;
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_FSM,
		     (TEXT("%s: BOOTUP Parameters loaded successfully\n"),__func__));

	w_adapter->fsm_state = FSM_LOAD_BOOTUP_PARAMS ;
#endif


	os_intf_ops->onebox_init_dyn_mutex(&w_adapter->ic_lock_vap);
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("wlan hal: Init proc entry call\n")));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("dev name is %s\n"), dev->name));
	if (setup_wlan_procfs(w_adapter))
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: Failed to setup wlan procfs entry\n"),__func__)); 

	os_intf_ops->onebox_release_sem(&d_assets->wlan_init_lock);
	//Initializing Queue Depths
	w_adapter->host_txq_maxlen[BK_Q_STA] = w_adapter->host_txq_maxlen[BK_Q_AP] = MAX_BK_QUEUE_LEVEL; 
	w_adapter->host_txq_maxlen[BE_Q_STA] = w_adapter->host_txq_maxlen[BE_Q_AP] = MAX_BE_QUEUE_LEVEL; 
	w_adapter->host_txq_maxlen[VI_Q_STA] = w_adapter->host_txq_maxlen[VI_Q_AP] = MAX_VI_QUEUE_LEVEL; 
	w_adapter->host_txq_maxlen[VO_Q_STA] = w_adapter->host_txq_maxlen[VO_Q_AP] = MAX_VO_QUEUE_LEVEL; 
	return ONEBOX_STATUS_SUCCESS;    

fail:
	if (dev) {
		unregister_dev(dev);
		w_adapter->dev = NULL;
	}
nodev:
	os_intf_ops->onebox_release_sem(&d_assets->wlan_init_lock);
	return -ENOMEM;    
}/* End <wlan_insert> */

/**
 * This function removes the wlan module safely..
 *
 * @param  Pointer to sdio_func structure.  
 * @param  Pointer to sdio_device_id structure.  
 * @return VOID. 
 */
int32 wlan_remove(struct driver_assets *d_assets)
{
	struct net_device *dev = NULL;
	struct onebox_os_intf_operations *os_intf_ops = onebox_get_os_intf_operations_from_origin();
	struct onebox_wlan_osd_operations *wlan_osd_ops = onebox_get_wlan_osd_operations_from_origin();
	struct wireless_techs *wlan_d = &d_assets->techs[WLAN_ID];
	WLAN_ADAPTER w_adapter = NULL;
	
	w_adapter = d_assets->techs[WLAN_ID].priv;
	if (w_adapter == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s %d WLAN Adapter is NULL\n"), __func__, __LINE__));
		return -1;
	}
	
	w_adapter->os_intf_ops->onebox_acquire_sem(&w_adapter->wlan_gpl_lock, 0);
	dev = w_adapter->dev;

	FUNCTION_ENTRY(ONEBOX_ZONE_INFO);   


	w_adapter->recv_stop = 1;
	os_intf_ops->onebox_set_event(&(w_adapter->stats_event));
	os_intf_ops->onebox_delete_event(&(w_adapter->stats_event));
	os_intf_ops->onebox_set_event(&(w_adapter->wlan_iqs_event));
	os_intf_ops->onebox_delete_event(&(w_adapter->wlan_iqs_event));
	w_adapter->os_intf_ops->onebox_set_event(&(d_assets->iap_event));
	if(d_assets->mfi_signature)
		os_intf_ops->onebox_mem_free(d_assets->mfi_signature);
	if(d_assets->mfi_certificate)
		os_intf_ops->onebox_mem_free(d_assets->mfi_certificate);

	/* Removing channel_util_timeout timer here */
	if ((w_adapter->channel_util_timeout.function != NULL) &&
	    (w_adapter->os_intf_ops->onebox_sw_timer_pending(&w_adapter->channel_util_timeout))) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d channel utilization timer was pending\n"), __func__, __LINE__));
		w_adapter->os_intf_ops->onebox_remove_timer(&w_adapter->channel_util_timeout);
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d Removed channel utilization timer\n"), __func__, __LINE__));
	}

	if ((w_adapter->long_pulse_timer.function != NULL) &&
	    (w_adapter->os_intf_ops->onebox_sw_timer_pending(&w_adapter->long_pulse_timer))) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d channel utilization timer was pending\n"), __func__, __LINE__));
		w_adapter->os_intf_ops->onebox_remove_timer(&w_adapter->long_pulse_timer);
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d Removed long pulse timer \n"), __func__, __LINE__));
	}

		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Below are WLAN STATS: \n"), __func__));
	printk("total beacon send : %d\n",
		   w_adapter->total_beacon_count);
	printk("total beacon Interrupts Received : %d\n",
		   w_adapter->beacon_interrupt);

	printk("total_mgmt_pkt_send : %d\n",
		   w_adapter->total_tx_data_sent[MGMT_SOFT_Q]);

	/* RX Path Stats */
	printk("total_mgmt_rx       : %d\n",
		   w_adapter->total_mgmt_rx_done_intr);
	printk("total_onair_mgmt       : %d\n",
		   w_adapter->onair_mgmt_pkts);
	printk("total_data_rx       : %d\n",
		   w_adapter->total_data_rx_done_intr);

	printk("BUFFER FULL COUNTER  : %d\n",
		   w_adapter->buf_full_counter);
	printk("BUFFER SEMI FULL COUNTER  : %d\n",
		   w_adapter->buf_semi_full_counter);
	printk("MGMT BUFFER FULL COUNTER  : %d\n",
		   w_adapter->mgmt_buf_full_counter);


	w_adapter->core_ops->onebox_core_deinit(w_adapter);
	d_assets->techs[WLAN_ID].tx_intention = 1;
	d_assets->update_tx_status(d_assets,WLAN_ID);
	
	if(!d_assets->techs[WLAN_ID].tx_access) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Waiting for tx_acces from common hal cmntx %d\n"), d_assets->common_hal_tx_access));
		d_assets->techs[WLAN_ID].deregister_flags = 1;
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Waiting event %s Line %d\n"), __func__, __LINE__));
		if (wait_event_timeout((d_assets->techs[WLAN_ID].deregister_event), (d_assets->techs[WLAN_ID].deregister_flags == 0), msecs_to_jiffies(6000) )) {
			if(!d_assets->techs[WLAN_ID].tx_access) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d unable to get access \n"), __func__, __LINE__));
				return -1;
			}
		} else {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERR: In %s Line %d Initialization of WLAN Failed as Wlan TX access is not granted from Common Hal \n"), __func__, __LINE__));
			return -1;
		}
	}

	if(wlan_d->tx_access && (d_assets->dreg_frame == 0) && 
	    (d_assets->protocol_enabled & WLAN_PROTOCOL) && (d_assets->card_state != GS_CARD_DETACH)) {
		wlan_deregister_fw(w_adapter);	
	}

	WLAN_TECH.tx_intention = 0;
	WLAN_TECH.tx_access = 0;
	d_assets->update_tx_status(d_assets, WLAN_ID);

	d_assets->techs[WLAN_ID].fw_state = FW_INACTIVE;

	w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->wlan_gpl_lock);
  /*Return the network device to the kernel*/
	destroy_wlan_procfs(w_adapter);
#if KERNEL_VERSION_BTWN_2_6_(18, 26)
	wlan_osd_ops->onebox_kill_wlan_thread(&w_adapter->sdio_scheduler_thread_handle); 
#elif KERNEL_VERSION_GREATER_THAN_2_6_(27)
	wlan_osd_ops->onebox_kill_wlan_thread(w_adapter); 
#endif
	if(w_adapter->dev != NULL) {
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("IN %s Line %d Unregistering netdev \n"), __func__, __LINE__));
          unregister_dev(dev);
	}

	FUNCTION_EXIT(ONEBOX_ZONE_INFO);

	return ONEBOX_STATUS_SUCCESS;
}/* End <onebox_disconnect> */

ONEBOX_STATUS wlan_module_init(struct driver_assets *d_assets)
{
	uint32 rc = 0;
	WLAN_ADAPTER w_adapter = NULL;
	if (d_assets->techs[WLAN_ID].drv_state != MODULE_ACTIVE) {
		d_assets->techs[WLAN_ID].drv_state = MODULE_INSERTED;
		d_assets->techs[WLAN_ID].inaugurate = wlan_insert;
		d_assets->techs[WLAN_ID].disconnect = wlan_remove;
		d_assets->techs[WLAN_ID].onebox_get_pkt_from_coex = wlan_gpl_read_pkt;
		d_assets->techs[WLAN_ID].onebox_get_buf_status = wlan_update_buf_status;
		d_assets->techs[WLAN_ID].wlan_dump_mgmt_pending = dump_wlan_mgmt_pending_pkts;

		if (d_assets->card_state == GS_CARD_ABOARD) {
			if ((d_assets->techs[WLAN_ID].fw_state == FW_ACTIVE) &&
					(d_assets->techs[WLAN_ID].drv_state != MODULE_ACTIVE)) {
				rc = wlan_insert(d_assets);
				if (rc) {
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: failed to initialize "
									"wlan error[%d]\n"),__func__, rc)); 
					return ONEBOX_STATUS_FAILURE;
				}
			}
		}	
	} else {
		w_adapter = (WLAN_ADAPTER)d_assets->techs[WLAN_ID].priv;	
		w_adapter->fsm_state = FSM_FW_LOADED;
	}
	
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("WLAN : Wlan Module installed\n ")));
	return 0;
}

ONEBOX_STATUS wlan_module_deinit(struct driver_assets *d_assets)
{
	struct wireless_techs *wlan_d;


	wlan_d = &d_assets->techs[WLAN_ID];
	if(!wlan_d->tx_access) {
		wlan_d->tx_intention = 1;
		d_assets->update_tx_status(d_assets, WLAN_ID);
		if(!wlan_d->tx_access) {
			d_assets->techs[WLAN_ID].deregister_flags = 1;
			wait_event_timeout((wlan_d->deregister_event), (d_assets->techs[WLAN_ID].deregister_flags == 0), msecs_to_jiffies(6000));
		}
	}

	if (d_assets->techs[WLAN_ID].drv_state == MODULE_ACTIVE) {
		d_assets->techs[WLAN_ID].drv_state = MODULE_REMOVED;
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d Calling Wlan_remove \n"), __func__, __LINE__));
		wlan_remove(d_assets);
	}

	d_assets->techs[WLAN_ID].inaugurate = NULL;
	d_assets->techs[WLAN_ID].disconnect = NULL;
	d_assets->techs[WLAN_ID].onebox_get_pkt_from_coex = NULL;
	d_assets->techs[WLAN_ID].onebox_get_buf_status = NULL;

	return 0;
}

ONEBOX_STATIC int32 onebox_wlangpl_module_init(VOID)
{
	
	return 0;

}


ONEBOX_STATIC VOID onebox_wlangpl_module_exit(VOID)
{

	return ;
}

EXPORT_SYMBOL(wlan_module_init);
EXPORT_SYMBOL(wlan_module_deinit);

module_init(onebox_wlangpl_module_init);
module_exit(onebox_wlangpl_module_exit);

MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("Redpine Signals, Inc.");
MODULE_DESCRIPTION("Driver for Redpine Signals' RS9113 module based USB/SDIO cards.");
MODULE_SUPPORTED_DEVICE("Redpine Signals' RS9113 module based USB/SDIO cards.");
