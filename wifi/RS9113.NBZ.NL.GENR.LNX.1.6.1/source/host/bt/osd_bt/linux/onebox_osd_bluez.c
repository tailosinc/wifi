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
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/ioctl.h>
#include <linux/skbuff.h>

#ifdef USE_BLUEZ_BT_STACK

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include "bt_common.h"
#include "onebox_linux.h"
#define VERSION "2.2"


/**
 * callback function for `hdev->open'
 *
 * @hdev - pointer to `struct hci_dev' data
 * @return - 0 on success
 */
static int onebox_hdev_open(struct hci_dev *hdev)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
		(TEXT("%s: `%s' open\n"), __func__, hdev->name));

	if(test_and_set_bit(HCI_RUNNING, &hdev->flags))
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("%s: device `%s' already running\n"),
			 __func__, hdev->name));

	return 0;
}

/**
 * callback function for `hdev->close'
 *
 * @hdev - pointer to `struct hci_dev' data
 * @return - 0 on success
 */
static int onebox_hdev_close(struct hci_dev *hdev)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
		(TEXT("%s: `%s' close\n"), __func__, hdev->name));

	if(!test_and_clear_bit(HCI_RUNNING, &hdev->flags))
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("%s: device `%s' not running\n"),
			 __func__, hdev->name));

	return 0;
}

/**
 * callback function for `hdev->flush'
 *
 * @hdev - pointer to `struct hci_dev' data
 * @return - 0 on success
 */
static int onebox_hdev_flush(struct hci_dev *hdev)
{
	BT_ADAPTER bt_adapter;

	if (!(bt_adapter = get_hci_drvdata(hdev)))
		return -EFAULT;

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
		(TEXT("%s: `%s' flush \n"), __func__, hdev->name));

	bt_adapter->os_intf_ops->onebox_queue_purge(&bt_adapter->bt_tx_queue);

	return 0;
}



/**
 * callback function for `hdev->send'
 *
 * @hdev - pointer to `struct hci_dev' data
 * @skb - pointer to the skb 
 * @return - 0 on success
 * 
 * `hci' sends a packet through this function.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
static int onebox_hdev_send_frame(struct sk_buff *skb)
#else
static int onebox_hdev_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
#endif
{
	BT_ADAPTER bt_adapter;
	netbuf_ctrl_block_t *netbuf_cb = (netbuf_ctrl_block_t *)skb->cb;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;
#endif

	if (!(bt_adapter = get_hci_drvdata(hdev)))
		return -EFAULT;

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return -EBUSY;

	switch (bt_cb(skb)->pkt_type) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;

	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;

	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;

	default:
		return -EILSEQ;
	}

	if(skb_headroom(skb) < REQUIRED_HEADROOM_FOR_BT_HAL)
	{
		/* Re-allocate one more skb with sufficent headroom make copy of input-skb to new one */
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s No sufficient head room\n"), __func__));

		/* Allocate new buffer with required headroom */
		netbuf_cb = bt_adapter->os_intf_ops->onebox_alloc_skb(skb->len + REQUIRED_HEADROOM_FOR_BT_HAL);
		/* Reserve the required headroom */
		bt_adapter->os_intf_ops->onebox_reserve_data(netbuf_cb, REQUIRED_HEADROOM_FOR_BT_HAL);
		/* Prepare the buffer to add data */
		bt_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, skb->len);
		/* copy the data from skb to new buffer */
		bt_adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, skb->data, skb->len);
		/* Assign the pkt type to new netbuf_cb */
		netbuf_cb->bt_pkt_type = bt_cb(skb)->pkt_type;
		/* Finally free the old skb */
		dev_kfree_skb(skb);
	}
	else
	{
		netbuf_cb->len = skb->len;
		netbuf_cb->pkt_addr = (VOID *)skb;
		netbuf_cb->data = skb->data;
		netbuf_cb->bt_pkt_type = bt_cb(skb)->pkt_type;
	}
	bt_adapter->osi_bt_ops->onebox_bt_xmit(bt_adapter, netbuf_cb);
	return 0;
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 3, 8)
/**
 * callback function for `hdev->destruct'
 *
 * @hdev - pointer to `struct hci_dev' data
 * @return - void
 */
static void onebox_hdev_destruct(struct hci_dev *hdev)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		(TEXT("%s: `%s' destruct \n"), __func__, hdev->name));
	return;
}
#endif

#ifdef RSI_CONFIG_ANDROID
static void rsi_enqueue(BT_ADAPTER bt_adapter, struct sk_buff *skb)
{
	if (bt_adapter->rsi_skb_queue_front == (bt_adapter->rsi_skb_queue_rear + 1) % QUEUE_SIZE) {
		/*
		 * If queue is full, current solution is to drop
		 * the following entries.
		 */
		kfree_skb(skb);
	} else {
		if (bt_adapter->rsi_skb_queue_front == -1) {
			bt_adapter->rsi_skb_queue_front = 0;
			bt_adapter->rsi_skb_queue_rear = 0;
		} else {
			bt_adapter->rsi_skb_queue_rear++;
			bt_adapter->rsi_skb_queue_rear %= QUEUE_SIZE;
		}

		bt_adapter->rsi_skb_queue[bt_adapter->rsi_skb_queue_rear] = skb;
	}
}

static void rsi_send_to_stack(BT_ADAPTER bt_adapter, struct sk_buff *skb)
{
	struct hci_dev *hdev = bt_adapter->hdev;
	struct sk_buff *rtk_skb_copy = NULL;

	if (!hdev) {
ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Frame for unknown HCI device")));
		return;
	}

	if (!test_bit(HCI_RUNNING, &hdev->flags)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("HCI not running")));
		return;
	}

	rtk_skb_copy = pskb_copy(skb, GFP_ATOMIC);
	if (!rtk_skb_copy) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Copy skb error")));
		return;
	}

	memcpy(skb_push(rtk_skb_copy, 1), &bt_cb(skb)->pkt_type, 1);
	rsi_enqueue(bt_adapter, rtk_skb_copy);

	/* Make sure bt char device existing before wakeup read queue */
	if (hdev) {
		wake_up_interruptible(&bt_adapter->rsi_btchr_read_wait);
	}

	return;
}
#endif

ONEBOX_STATUS send_pkt_to_bluez(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb)
{
 	ONEBOX_STATUS status;
	struct sk_buff *skb = netbuf_cb->pkt_addr;
	struct hci_dev *hdev = bt_adapter->hdev;
 
 	bt_adapter->hdev->stat.byte_rx += netbuf_cb->len;

 	skb->dev = (void *)hdev;
 	bt_cb(skb)->pkt_type = netbuf_cb->bt_pkt_type;

#ifdef RSI_CONFIG_ANDROID
	rsi_send_to_stack(bt_adapter, skb);
	kfree_skb(skb);
 	return 0;
#else
	status = onebox_hci_recv_frame(hdev, skb);
	if (status < 0)
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: packet to `%s' failed\n"),
				 __func__, hdev->name));
	if(netbuf_cb){
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
				(TEXT("%s: Freeing netbuf_cb \n"),__func__));
		kfree(netbuf_cb);
	}
	return status;
#endif
}

/**
 * registers with the bluetooth-hci(BlueZ) interface
 *
 * @bt_adapter - pointer to bluetooth asset's bt_adapter 
 * @return  - 0 on success 
 */
int bluez_init(BT_ADAPTER bt_adapter)
{
	int err;
	struct hci_dev *hdev;
	struct driver_assets *d_assets = bt_adapter->d_assets;

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
		(TEXT("%s: registering with `hci'\n"),__func__));
#ifdef RSI_CONFIG_ANDROID
	init_waitqueue_head(&bt_adapter->rsi_btchr_read_wait);
#endif
	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	if (d_assets->host_intf_type == HOST_INTF_SDIO)
		hdev->bus = HCI_SDIO;
	else
		hdev->bus = HCI_USB;

	set_hci_drvdata(hdev, bt_adapter);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,8,0)
	hdev->dev_type = HCI_PRIMARY;
#else
	hdev->dev_type = HCI_BREDR;
#endif

	bt_adapter->hdev = hdev;

	hdev->open     = onebox_hdev_open;
	hdev->close    = onebox_hdev_close;
	hdev->flush    = onebox_hdev_flush;
	hdev->send     = onebox_hdev_send_frame;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 3, 8)
	hdev->destruct = onebox_hdev_destruct;
	hdev->owner    = THIS_MODULE;
#endif

	err = hci_register_dev(hdev);
	if (err < 0) {
 		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("%s: hci registration failed %d\n"), __func__, err));
		hci_free_dev(hdev);
		bt_adapter->hdev = NULL;
		return err;
	}

 	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		(TEXT("%s: done registering `%s'\n"), __func__, hdev->name));
	
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * unregisters with the bluetooth-hci(BlueZ) interface
 *
 * @bt_adapter - pointer to bluetooth asset's bt_adapter 
 * @return  - 0 on success 
 */
int bluez_deinit(BT_ADAPTER bt_adapter)
{
	struct hci_dev *hdev;

	if (!(hdev = bt_adapter->hdev))
		return -EFAULT;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		(TEXT("%s: deregistering `%s'\n"), 
		__func__, hdev->name));

	onebox_hci_dev_hold(hdev);
	hci_unregister_dev(hdev);
	onebox_hci_dev_put(hdev);

	hci_free_dev(hdev);

	return ONEBOX_STATUS_SUCCESS;
}
#endif
