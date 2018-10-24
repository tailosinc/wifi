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

#include <linux/cdev.h>
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

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include "bt_common.h"
#include "onebox_linux.h"
#include "onebox_zone.h"

#define VERSION "2.2"
static dev_t bt_devid; /* bt char device number */
static struct cdev bt_char_dev; /* bt character device structure */
static struct class *bt_char_class; /* device class for usb char driver */
struct mutex btchr_mutex;
BT_ADAPTER gadapter = NULL;
#define BT_CHAR_DEVICE_NAME "rsi"

static int hci_device_close(BT_ADAPTER bt_adapter)
{
	struct hci_dev *hdev = bt_adapter->hdev;

	if (!hdev) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: failed to get hci dev[Null]"), __func__));
		return -ENODEV;
	}

	hdev->flush(hdev);
	hdev->close(hdev);
	/* Clear flags */
	hdev->flags = 0;

	return 0;
}

static int hci_device_open(BT_ADAPTER bt_adapter)
{
	struct hci_dev *hdev = bt_adapter->hdev;
	int ret = 0;

	hdev = bt_adapter->hdev;
	if (!hdev) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Failed to get hci dev[Null]"), __func__));
		return -ENODEV;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0)
	if (test_bit(HCI_UNREGISTER, hdev->dev_flags)) {
#else
	if (test_bit(HCI_UNREGISTER, &hdev->dev_flags)) {
#endif
		ret = -ENODEV;
		goto done;
	}

	if (test_bit(HCI_UP, &hdev->flags)) {
		ret = -EALREADY;
		goto done;
	}

	if (hdev->open(hdev)) {
		ret = -EIO;
		goto done;
	}

	set_bit(HCI_UP, &hdev->flags);
done:
	return ret;
}

static int rsi_btchr_open(struct inode *inode_p, struct file  *file_p)
{
	BT_ADAPTER bt_adapter = gadapter;
	struct hci_dev *hdev = bt_adapter->hdev;

	if (!hdev) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Failed to get hci dev[NULL]"), __func__));
		return -1;
	}
	//data = get_hci_drvdata(hdev);

	atomic_inc(&hdev->promisc);
	file_p->private_data = bt_adapter;

	hci_device_open(bt_adapter);

	return nonseekable_open(inode_p, file_p);
}

static int rsi_btchr_close(struct inode  *inode_p, struct file   *file_p)
{
	BT_ADAPTER bt_adapter;
	struct hci_dev *hdev;

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: BT usb char device is closing"), __func__));
	/* Not open unless wanna tracing log */
	/* trace_printk("%s: close....\n", __func__); */

	bt_adapter = file_p->private_data;

	hdev = bt_adapter->hdev;
	if (hdev) {
		atomic_set(&hdev->promisc, 0);
		hci_device_close(bt_adapter);
	}
	file_p->private_data = NULL;

	return 0;
}

static struct sk_buff *rsi_dequeue(BT_ADAPTER bt_adapter, unsigned int deq_len)
{
	struct sk_buff *skb;
	struct sk_buff *skb_copy;

	if (bt_adapter->rsi_skb_queue_front == -1) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Queue is empty"), __func__));
		return NULL;
	}

	skb = bt_adapter->rsi_skb_queue[bt_adapter->rsi_skb_queue_front];
	if (deq_len >= skb->len) {
		if (bt_adapter->rsi_skb_queue_front == bt_adapter->rsi_skb_queue_rear) {
			bt_adapter->rsi_skb_queue_front = -1;
			bt_adapter->rsi_skb_queue_rear = -1;
		} else {
			bt_adapter->rsi_skb_queue_front++;
			bt_adapter->rsi_skb_queue_front %= QUEUE_SIZE;
		}
		/*
		 * Return skb addr to be dequeued, and the caller
		 * should free the skb eventually.
		 */
		return skb;
	} else {
		skb_copy = pskb_copy(skb, GFP_ATOMIC);
		skb_pull(skb, deq_len);
		kfree_skb(skb);
		return skb_copy;
	}
}

int is_queue_empty(BT_ADAPTER bt_adapter)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Queue front value is: %d\n"),bt_adapter->rsi_skb_queue_front));
	return (bt_adapter->rsi_skb_queue_front == -1) ? 1 : 0;
}

static ssize_t rsi_btchr_read(struct file *file_p,
		char __user *buf_p,
		size_t count,
		loff_t *pos_p)
{
	BT_ADAPTER bt_adapter = file_p->private_data;
	struct hci_dev *hdev;
	struct sk_buff *skb;
	int ret = 0;
	int len;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("*************RSI BT READ*********************: %d\n"), count));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s: BT usb char device is reading"), __func__));

	while (count) {
		hdev = bt_adapter->hdev;
		if (!hdev) {
			/*
			 * Note: Only when BT device hotplugged out, we wil get
			 * into such situation. In order to keep the upper layer
			 * stack alive (blocking the read), we should never return
			 * EFAULT or break the loop.
			 */
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Failed to get hci dev[Null]"), __func__));
			break;
		}

		if (is_queue_empty(bt_adapter)) {
			ret = wait_event_interruptible(bt_adapter->rsi_btchr_read_wait, !is_queue_empty(bt_adapter));
			if (ret < 0) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: wait event is signaled %d"), __func__, ret));
				break;
			}
		}
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Trying to dequeue pkt here: %d\n"), count));

		skb = rsi_dequeue(bt_adapter, count);
		if (skb) {
			len = min_t(unsigned int, skb->len, count);

			if (copy_to_user(buf_p, skb->data, len)) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Failed to put data to user space"), __func__));
				kfree_skb(skb);
				break;
			}
			//kfree_skb(skb);
			return len;
		}
	}

	return 0;
}

static ssize_t rsi_btchr_write(struct file *file_p,
		const char __user *buf_p,
		size_t count,
		loff_t *pos_p)
{
	BT_ADAPTER bt_adapter = file_p->private_data;
	struct hci_dev *hdev;
	struct sk_buff *skb;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("*************RSI BT WRITE*********************\n")));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s: BT usb char device is writing"), __func__));

	hdev = bt_adapter->hdev;
	if (!hdev) {
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s: Failed to get hci dev[Null]"), __func__));
		/*
		 * Note: we bypass the data from the upper layer if bt device
		 * is hotplugged out. Fortunatelly, H4 or H5 HCI stack does
		 * NOT check rsi_btchr_write's return value. However, returning
		 * count instead of EFAULT is preferable.
		 */
		/* return -EFAULT; */
		return count;
	}

	/* Never trust on btusb_data, as bt device may be hotplugged out */
	//data = get_hci_drvdata(hdev);

	if (count > HCI_MAX_FRAME_SIZE)
		return -EINVAL;

	skb = bt_skb_alloc(count, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;
	skb_reserve(skb, -1); 

	if (copy_from_user(skb_put(skb, count), buf_p, count)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Failed to get data from user space"), __func__));
		kfree_skb(skb);
		return -EFAULT;
	}

	skb->dev = (void *)hdev;
	bt_cb(skb)->pkt_type = *((__u8 *)skb->data);
	skb_pull(skb, 1);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Sending pkt through HCI device\n")));
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
	hdev->send(hdev, skb);
#else
	hdev->send(skb);
#endif

	return count;
}

static unsigned int rsi_btchr_poll(struct file *file_p, poll_table *wait)
{
	BT_ADAPTER bt_adapter = file_p->private_data;
	struct hci_dev *hdev;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s: BT usb char device is polling"), __func__));

	hdev = bt_adapter->hdev;
	if (!hdev) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Failed to get hci dev[Null]"), __func__));
		//mdelay(URB_CANCELING_DELAY_MS);
		return POLLOUT | POLLWRNORM;
	}

	if (!is_queue_empty(bt_adapter)) {
		return POLLIN | POLLRDNORM;
	}

	poll_wait(file_p, &bt_adapter->rsi_btchr_read_wait, wait);

	if (!is_queue_empty(bt_adapter)) {
		return POLLIN | POLLRDNORM;

	} else {
		return POLLOUT | POLLWRNORM;
	}
}

static struct file_operations rsi_btdev_ops  = {
	open	:	rsi_btchr_open,
	release	:	rsi_btchr_close,
	read	:	rsi_btchr_read,
	write	:	rsi_btchr_write,
	poll	:	rsi_btchr_poll
};

/**
 * registers with the bluedroid interface
 *
 * @bt_adapter - pointer to bluetooth asset's bt_adapter 
 * @return  - 0 on success 
 */
int rsi_bdroid_init(BT_ADAPTER bt_adapter)
{
	int res = 0;
	struct device *dev;

	gadapter = bt_adapter;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Register usb char device interface for BT driver")));
	/*
	 * btchr mutex is used to sync between
	 * 1) downloading patch and opening bt char driver
	 * 2) the file operations of bt char driver
	 */
	//mutex_init(&btchr_mutex);

	//skb_queue_head_init(&btchr_readq);
	//init_waitqueue_head(&btchr_read_wait);

	bt_char_class = class_create(THIS_MODULE, BT_CHAR_DEVICE_NAME);
	if (IS_ERR(bt_char_class)) {
		return PTR_ERR(bt_char_class);
	}

	res = alloc_chrdev_region(&bt_devid, 0, 1, BT_CHAR_DEVICE_NAME);
	if (res < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Failed to allocate bt char device")));
		goto err_alloc;
	}

	dev = device_create(bt_char_class, NULL, bt_devid, NULL, BT_CHAR_DEVICE_NAME);
	if (IS_ERR(dev)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Failed to create bt char device")));
		res = PTR_ERR(dev);
		goto err_create;
	}

	bt_adapter->rsi_skb_queue_front = -1;
	bt_adapter->rsi_skb_queue_rear = -1;
	cdev_init(&bt_char_dev, &rsi_btdev_ops);
	res = cdev_add(&bt_char_dev, bt_devid, 1);
	if (res < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Failed to add bt char device")));
		goto err_add;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Finished bt char device initialization\n")));

	return 0;

err_add:
	device_destroy(bt_char_class, bt_devid);
err_create:
	unregister_chrdev_region(bt_devid, 1);
err_alloc:
	class_destroy(bt_char_class);
	return res;
}

/**
 * unregisters with the bluedroid interface
 *
 * @bt_adapter - pointer to bluetooth asset's bt_adapter 
 * @return  - 0 on success 
 */
void rsi_bdroid_deinit(BT_ADAPTER bt_adapter)
{
	device_destroy(bt_char_class, bt_devid);
	cdev_del(&bt_char_dev);
	unregister_chrdev_region(bt_devid, 1);
	class_destroy(bt_char_class);
	return;
}
