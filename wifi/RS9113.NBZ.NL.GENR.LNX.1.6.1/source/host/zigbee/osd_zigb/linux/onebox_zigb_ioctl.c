/**
 * @file onebox_hal_ioctl.c
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
 * This file contians the code for handling ioctls.
 */

#include "zb_common.h"
#include "onebox_zigb_ioctl.h"

static ONEBOX_STATUS zigb_tx_pkt(ZB_ADAPTER zb_adapter, uint8 *addr, uint8 len)
{
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;

	netbuf_cb = zb_adapter->os_intf_ops->onebox_alloc_skb(len);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;

	}
	zb_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);
	/*copy the internal mgmt frame to netbuf and queue the pkt */
	if(copy_from_user((uint8 *)netbuf_cb->data,(const void __user *) addr, len))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
			(TEXT("%s: Unable to perform copy operation\n"), __func__));
		return -EFAULT;
	}
#if 0		
	zb_adapter->os_intf_ops->onebox_netbuf_queue_tail(&zb_adapter->host_tx_queue[MGMT_SOFT_Q], netbuf_cb->pkt_addr);
	zb_adapter->osi_zigb_ops->onebox_schedule_pkt_for_tx(zb_adapter);
#endif
	zb_adapter->osi_zigb_ops->onebox_send_pkt(zb_adapter, netbuf_cb);
	return status;
}

static int zigb_recv_pkt(ZB_ADAPTER zb_adapter, uint8 *msg, uint16 *len)
{
	netbuf_ctrl_block_t *netbuf_cb;

	if(zb_adapter->os_intf_ops->onebox_netbuf_queue_len(&zb_adapter->zigb_rx_queue))
	{
		netbuf_cb = zb_adapter->os_intf_ops->onebox_dequeue_pkt((void *)&zb_adapter->zigb_rx_queue);
		if(netbuf_cb == NULL)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("queue is empty but length is not zero in %s"), __func__));
			return ONEBOX_STATUS_FAILURE;
		}
		if(copy_to_user((void __user *)msg, netbuf_cb->data, netbuf_cb->len))
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
				(TEXT("%s: Unable to perform copy operation\n"), __func__));
			return -EFAULT;
		}
		*len = netbuf_cb->len;
		zb_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb,0);
		return ONEBOX_STATUS_SUCCESS;	
	}
	return ONEBOX_STATUS_FAILURE;
}

/**
 *  Calls the corresponding (Private) IOCTL functions
 *
 * @param  pointer to the net_device
 * @param  pointer to the ifreq
 * @param  value of the ioctl command, input to this function
 * @return returns 0 on success otherwise returns the corresponding 
 * error code for failure
 */
int zigb_ioctl(struct net_device *dev,struct ifreq *ifr, int cmd)
{
	ZB_ADAPTER zb_adapter = (ZB_ADAPTER)netdev_priv(dev);
 	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	uint8 len = 0, *msg = NULL;
	uint16 recv_len =0;
 	struct iwreq *wrq = (struct iwreq *)ifr;

	/* Check device is present or not */
	if (!netif_device_present(dev))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR , (TEXT(" %s : DEVICE not present\n"), __func__));
		return -ENODEV;
	}
	msg = wrq->u.data.pointer;
	len = wrq->u.data.length;

	switch(cmd)
	{
		case ONEBOX_ZIGB_SEND:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO , (TEXT("***********************************\n")));
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("**ZIGBEE SEND IOCTL RECEIVED*******\n")));
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO , (TEXT("***********************************\n")));

			if(!msg || !len)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("Address or length is invalid\n"));
				return -EINVAL;	
			}
			status = zigb_tx_pkt(zb_adapter, msg, len);
			return status;
		}
		case ONEBOX_ZIGB_RECV:
		{

			status = zigb_recv_pkt(zb_adapter, msg, &recv_len);
			wrq->u.data.length = recv_len;
			if(zb_adapter->os_intf_ops->onebox_netbuf_queue_len(&zb_adapter->zigb_rx_queue))
				wrq->u.data.flags = 1;
			else
				wrq->u.data.flags = 0;
			return status;
		}
		default:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR , (TEXT(" %s :Received invalid ioctl\n"), __func__));
			return -EOPNOTSUPP;
		}
		
	}
	return ONEBOX_STATUS_SUCCESS;
}
