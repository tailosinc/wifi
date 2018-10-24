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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/version.h>
#include <net/iw_handler.h>     /* New driver API */
#include <linux/string.h>
#include <net/genetlink.h>
#include <net/sock.h>
#include <asm/string.h>

#include "onebox_common.h"
#include "onebox_linux.h"
#include "onebox_sdio_intf.h"
#include "onebox_zone.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
#include <linux/semaphore.h>
#endif

/**
 * This function initializes event.
 *
 * @param  Pointer to the event data structure.  
 * @return 0 . 
 */

 uint32 onebox_zone_enabled = /* ONEBOX_ZONE_INFO |
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
                             ONEBOX_ZONE_PWR_SAVE | */
                             ONEBOX_ZONE_ERROR |
                             0;
int32 init_event (ONEBOX_EVENT *pEvent)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Initialising the event %p addr\n"),pEvent));
	atomic_set(&pEvent->eventCondition, 1);
	init_waitqueue_head(&pEvent->eventQueue);
	return 0;
}

int32 delete_event(ONEBOX_EVENT *pEvent)
{
	/**Dummy for Linux*/
	return 0;
}
 
/**
 * This function initializes the spinlock.
 *
 * @param  lock pointer.  
 * @return void. 
 */
VOID spinlock_init(VOID *lock)
{
	spin_lock_init((onebox_spinlock_t *)lock);
}


/**
 * This function acquires the spinlock.
 *
 * @param  lock pointer
 * @param  flags to save the previous interrupt state  
 * @return returns 0 on success else an error code. 
 */
VOID acquire_spinlock(PVOID lock,unsigned long *flags)
__acquires((spinlock_t *)lock)
{
  spin_lock_irqsave((spinlock_t *)lock, *flags);

}

/**
 * This function releases the spinlock
 *
 * @param  lock pointer
 * @param  flags used to keep track of the interrupt state
 * @return returns 0 on success else an error code. 
 */
VOID release_spinlock(PVOID lock,unsigned long flags)
__releases((spinlock_t *)lock)
{
	spin_unlock_irqrestore((onebox_spinlock_t *)lock, flags);
}

/**
 * This function acquires the mutex.
 *
 * @param  mutex name.  
 * @return returns 0 on success else an error code. 
 */
int acquire_mutex(void* mutex)
{
	struct mutex *mlock;
	int status;
	mlock = (struct mutex *)&mutex;
	status = mutex_lock_interruptible(mlock);
	return status;
}

/**
 * This function unlocks the mutex
 * @param  Pointer to semaphore.  
 * @return void. 
 */
void release_mutex(void *mutex)
{
	struct mutex *mlock;
	mlock = (struct mutex *)&mutex;
	mutex_unlock(mlock);
	return;
}

/**
 * This function acquires the interruptable semaphore.
 *
 * @param  Pointer to semaphore.  
 * @param  Delay (Dummy for Linux) .  
 * @return 0 if success else a negative number. 
 */
int down_interruptible_sem(void* sem, int delay)
{
	return down_interruptible((struct semaphore*)sem);
}

/**
 * This function acquires the semaphore.
 *
 * @param  Pointer to semaphore.  
 * @param  Delay (Dummy for Linux) .  
 * @return 0 if success else a negative number. 
 */
ONEBOX_STATUS down_sem(void* sem, int delay)
{
	down((struct semaphore*)sem);
	return 0; 
}

/**
 * This function releases the semaphore.
 *
 * @param  Pointer to semaphore.  
 * @return TRUE if success else a FALSE. 
 */
BOOLEAN up_sem(void* sem)
{
	up((struct semaphore*)sem);
	return 0; 
}

/**
 * This function is used to put the current execution in a queue 
 * and reschedules itself for execution on "timeout" or when a 
 * wakeup is generated
 * 
 * @param Ptr to the event structure
 * @param Timeout value in msecs
 * @return Success or Failure status
 */
int wait_queue_event(ONEBOX_EVENT *event, uint32 timeOut)
{
	int32 Status = 0;
	if (!timeOut)
	{
		Status = wait_event_interruptible(event->eventQueue,
		                         (atomic_read(&event->eventCondition) == 0));
		if (Status) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s: line no %d wait event returned with error %d\n ", __func__, __LINE__, Status));
			return Status;
		}
		
	}
	else
	{
		Status = wait_event_interruptible_timeout(event->eventQueue,
		                                          (atomic_read(&event->eventCondition) == 0),
		                                          timeOut);
		if (Status < 0) {
			dump_stack();
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d: wait event timeout returned with error %d\n ", __func__, __LINE__, Status));
			return Status;
		}
		
	} /* End if <condition> */

	return Status;
}

/**
 * This function handle the set event functionality.
 *
 * @param  Pointer to event.  
 * @return VOID. 
 */
VOID rsi_set_event(ONEBOX_EVENT *event)
{
	atomic_set(&event->eventCondition, 0);
	wake_up_interruptible(&event->eventQueue);
}
EXPORT_SYMBOL(rsi_set_event);

/**
 * This function handle the reset event functionality.
 *
 * @param  Pointer to event.  
 * @return void. 
 */
void rsi_reset_event(ONEBOX_EVENT *event)
{
	atomic_set(&event->eventCondition, 1);
}

/** This function reads the atomic variable 
 * @param  pointer to atomic_t
 * @return  read
 */
int read_atomic_var(void *addr)
{
	atomic_t *v = (atomic_t *)addr;
	int val = atomic_read(v);
	return val;
}

/** This function is used to signal a completion event by calling complete
 * and calls exit for the current thread
 * @param 
 * @return VOID
 */
void completion_exit_event(void *addr,int val)
{
	struct completion *wait;
	wait = (struct completion *)addr;
	complete_and_exit(wait,val);
	return;
}


/**
 * This function starts the network Q.
 *
 * @param  Pointer to driver adapter structure.  
 * @return VOID. 
 */
void start_networkq(void *dev)
{
	netif_start_queue(dev);
}

/**
 * This function handle the starting of transmit Q.
 *
 * @param  Pointer to driver adapter structure.  
 * @return VOID. 
 */
void start_ifp_txq(void *dev)
{
	return netif_wake_queue(dev);
}

/**
 * This function handle the starting of an Access category Queue.
 *
 * @param  Pointer to driver adapter structure.  
 * @return VOID. 
 */
void start_sub_txq(void *dev, int ac)
{
	return netif_wake_subqueue(dev, ac);
}

/**
 * This function handle the stopping of transmit Q.
 *
 * @param  Pointer to driver adapter structure.  
 * @return VOID. 
 */
void stop_ifp_txq(void *dev)
{
	return netif_stop_queue(dev);
}

/* This function handle the stopping of an Access category Queue.
 *
 * @param  Pointer to driver adapter structure.  
 * @return VOID. 
 */
void stop_sub_txq(void *dev, int ac)
{
	return netif_stop_subqueue(dev, ac);
}

/**
 * This function handle the checking of transmitQ of an Access Category .
 *
 * @param  Pointer to driver adapter structure.  
 * @return true if the queue is stopped else false. 
 */
int txq_sub_stopped(void *dev, int ac)
{
	return __netif_subqueue_stopped(dev, ac);
}

/**
 * This function handle the checking of transmit Q.
 *
 * @param  Pointer to driver adapter structure.  
 * @return true if the queue is stopped else false. 
 */
int txq_ifp_stopped(void *dev)
{
	return netif_queue_stopped(dev);
}

/**
 * This function modifies the timer.
 *
 * @param  Pointer to timer to be modified.  
 * @param  timeout value.  
 * @return 0. 
 */
int modify_timer(void* timer, unsigned long expires)
{
	return mod_timer((struct timer_list*)timer, (jiffies+(expires)));
}

/**
 * This function handle returns the current timestamp
 *
 * @param  VOID.
 * @return unsigned long value of jiffies 
 */
unsigned long get_jiffies (void) 
{
	return jiffies;
}

/**
 * This function allows the micro seconds delay.
 *
 * @param  microseconds delay.  
 * @return VOID. 
 */
void usec_delay (unsigned long usecs)
{
	udelay((unsigned int)usecs);
}

/**
 * This function allows the milli seconds delay.
 *
 * @param  mseconds delay.  
 * @return VOID. 
 */
void msec_delay (unsigned int msecs)
{
	mdelay(msecs);
}

/**
 * Allocate an sk buff.
 * 
 * @param  size of the sk buff to allocate.  
 * @return pointer to the netbuf control block structure. 
 */
netbuf_ctrl_block_t* allocate_skb(int len)
{
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	struct sk_buff *skb=NULL;
#ifdef USE_USB_INTF
	skb = dev_alloc_skb(len + (ONEBOX_USB_TX_HEAD_ROOM + FRAME_DESC_SZ + 64 +26 /*extended desc sz is variable*/));
	if(!skb) {
		return NULL;
	}
	skb_reserve(skb, (ONEBOX_USB_TX_HEAD_ROOM + FRAME_DESC_SZ + 26));
#else
	skb = dev_alloc_skb(len + NET_IP_ALIGN + FRAME_DESC_SZ + 64/*64 bit dword align req*/ + 26 /*extended desc sz is variable*/);
	if (skb && NET_IP_ALIGN) {
		skb_reserve(skb, NET_IP_ALIGN);
	}
	skb_reserve(skb, ( FRAME_DESC_SZ + 26 + 64/*64 bit dword align req */));
#endif
	if (!skb) 
	{
		return NULL;
	} 
	else 
	{
		netbuf_cb = kmalloc(sizeof(netbuf_ctrl_block_t), GFP_ATOMIC);
		if (netbuf_cb == NULL) {
			dev_kfree_skb_any(skb);
			return NULL;
		}
		netbuf_cb->pkt_addr = (void *)skb;
		netbuf_cb->data = skb->data;
		netbuf_cb->len = len;
		*(unsigned long *)(skb->cb) = (unsigned long)netbuf_cb;
	}
	return netbuf_cb;
}

/**
 * This function puts the data to the begining of skb buffer
 * @param  Pointer to sk buff structure
 * @param  Len of data to be pulled
 * @return Pointer to UCHAR. 
 */
PUCHAR put_data(netbuf_ctrl_block_t * netbuf_cb, int len)
{
	struct sk_buff *skb = (struct sk_buff *)netbuf_cb->pkt_addr;
	uint32 dword_align_req_bytes = 0;

	netbuf_cb->data = skb_put(skb,len);
	netbuf_cb->len  = skb->len;
#if 0
if (((uint32) netbuf_cb->data & 0x3) < skb_headroom (skb))
  {
    skb_push (skb, ((uint32) netbuf_cb->data & 0x3));
    netbuf_cb->data = skb->data;
    netbuf_cb->len = skb->len;
  }
#endif
	dword_align_req_bytes = ((uint32)netbuf_cb->data & 0x3f);
	if (dword_align_req_bytes) {
		if (dword_align_req_bytes > skb_headroom(skb)) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT 
						("%s:  ERROR: Not Enough Head room: headroom = %d, len = %d\n"),__func__, skb_headroom(skb), dword_align_req_bytes));
			dump_stack();
		}
    skb_push (skb, dword_align_req_bytes);
		skb_trim (skb, skb->len-dword_align_req_bytes);
    netbuf_cb->data = skb->data;
    netbuf_cb->len = skb->len;
	}
	return netbuf_cb->data;
}

/**
 * This function queues a buffer at the list head
 * @param  Pointer to sk buff head structure
 * @return void 
 */
void get_skb_queue_head_init(void *addr)
{
	struct sk_buff_head *list;
	list = (struct sk_buff_head *)addr;
	return skb_queue_head_init(list);
}

/**
 * This function queues a buffer at the head of list 
 * @param  Pointer to sk buff head structure
 * @param  Pointer to sk buffer to be queued
 * @return returns queue length on success else an error code. 
 */
void skb_queue_head_add(void *addr, void *buffer)
{
	struct sk_buff *skb;
	struct sk_buff_head *list;
	skb = (struct sk_buff *)buffer;
	list = (struct sk_buff_head *)addr;
	return skb_queue_head(list,skb);
}
/**
 * This function queues a buffer at the list tail
 * @param  Pointer to sk buff head structure
 * @param  Pointer to sk buffer to be queued
 * @return returns queue length on success else an error code. 
 */
void get_skb_queue_tail(void *addr, void *buffer)
{
	struct sk_buff *skb;
	struct sk_buff_head *list;
	skb = (struct sk_buff *)buffer;
	list = (struct sk_buff_head *)addr;
	return skb_queue_tail(list,skb);
}

/**
 * This function gets the skb queue length
 * @param  Pointer to sk buff head structure
 * @return returns queue length on success else an error code. 
 */
int  get_skb_queue_len(void *addr)
{
	struct sk_buff_head *list;
	list = (struct sk_buff_head *)addr;
	return skb_queue_len(list);
}
EXPORT_SYMBOL(get_skb_queue_len);
/**
 * This function adds  the data to the sk buffer
 * @param  Pointer to sk buff structure
 * @param  Len of data to be pulled
 * @return pointer to the first byte of extra data is returned 
 */
ONEBOX_STATUS push_data(netbuf_ctrl_block_t *netbuf_cb, uint16 len)
{
	PUCHAR status;
	struct sk_buff *skb = (struct sk_buff *)netbuf_cb->pkt_addr;

	if( len > skb_headroom(skb))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT 
		             ("%s:  ERROR: Not Enough Head room: headroom = %d, len = %d\n"),__func__, skb_headroom(skb), len));
		return -1;
	}
	status = skb_push(skb,len);
	netbuf_cb->data = skb->data;
	netbuf_cb->len  = skb->len;
	return 0;
}

void reserve_data(netbuf_ctrl_block_t *netbuf_cb, int reserve_len)
{
	struct sk_buff *skb = netbuf_cb->pkt_addr;

	skb_reserve(skb, reserve_len);
	netbuf_cb->data = skb->data;
}

/**
 * This function handles the dequeing of packet from a skb list
 *
 * @param  Pointer to driver adapter structure.  
 * @param  Q number.  
 * @return Pointer to netbuf_ctrl_block_t. 
 */
netbuf_ctrl_block_t* dequeue_pkt(void *addr)
{
	netbuf_ctrl_block_t *netbuf_cb;
	struct sk_buff *rcvskb;  
	struct sk_buff_head *list = (struct sk_buff_head *)addr;

	rcvskb = skb_dequeue(list);
	if(rcvskb == NULL)
	{
		return NULL;
	}
	netbuf_cb = (netbuf_ctrl_block_t *)*((unsigned long int *)(rcvskb->cb));
	return netbuf_cb;
}

/**
 * This function frees the packet.
 *
 * @param  Pointer to driver adapter structure.  
 * @param  Pointer to netbuf_ctrl_block_t structure.  
 * @param  status.  
 * @return VOID. 
 */
void free_pkt(netbuf_ctrl_block_t *netbuf_cb,int status)
{
	dev_kfree_skb_any((struct sk_buff *)netbuf_cb->pkt_addr);
	kfree(netbuf_cb);
	return;
}

/**
 * This function handle the allocation of memory.
 *
 * @param  Pointer to Address to be allocated.  
 * @param  Length to be allocated.  
 * @param  Flags.  
 * @return VOID. 
 */
VOID mem_alloc(VOID **ptr,uint16 len,gfp_t flags)
{
	*ptr = kmalloc(len,flags);
}

/**
 * This function free the memory.
 *
 * @param  Pointer to Address to be freeed.  
 * @param  Length to be freeed.  
 * @param  Flags.  
 * @return VOID. 
 */
VOID freemem(VOID *ptr)
{
	kfree(ptr);
}

/**
 * This function free the virtual memory.
 *
 * @param  Pointer to Address to be freeed.  
 * @return VOID. 
 */
VOID vfreemem(VOID *ptr)
{
	vfree(ptr);
}

/**
 * This fucntion prepares the sk_buff_head for use by
 * other skb functions
 *
 * @param  Pointer to sk_buff_head structure.  
 * @return VOID. 
 */
void queue_head_init(struct sk_buff_head *list)
{
	skb_queue_head_init(list); 
}

/**
 * This function will empty a given list
 *
 * @param  Pointer to sk_buff_head structure.  
 * @return VOID. 
 */
void queue_purge(struct sk_buff_head *list)
{
	skb_queue_purge(list); 
}

/**
 * This function dynamically initializes the mutex at runtime.
 *
 * @param  Pointer to semaphore.  
 * @return VOID. 
 */
void init_dyn_mutex(struct semaphore *sem_name)
{
	sema_init(sem_name, 1);
}

/**
 * This function refers to the static initialization of mutex.
 *
 * @param  mutex type.  
 * @return VOID. 
 */
void init_static_mutex(void *mutex)
{
	mutex_init((struct mutex *)mutex);
	return;
}

/**
 * This function initializes the workq
 *
 * @param  Pointer to work_struct
 * @param  Pointer to callback function
 * @return VOID. 
 */
void init_workq(struct work_struct *work, void *function)
{
	INIT_WORK(work, function);
}

EXPORT_SYMBOL(init_workq);

void deinit_workq(struct workqueue_struct *work)
{
	flush_workqueue(work);
	destroy_workqueue(work);
}

/**
 * This function requests IRQ.
 *
 * @param  IRQ.  
 * @param  handler.  
 * @param  IRQ flags.  
 * @param  Pointer to device name.  
 * @param  Pointer to device id.  
 * @return 0. 
 */
int32 intr_request_irq (uint32 irq, void * handler, ulong irqflags, const char *devname,
                        void *dev_id)
{

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
	int ret;
	ret= request_irq(irq, (irq_handler_t)handler, irqflags, devname,
	                 (void *)dev_id);
	return ret;
#else
	return 0;
#endif
}

#if 0
/**
 * This function handle the copying of memory.
 *
 * @param  Pointer to memory1.  
 * @param  Pointer to memory2.  
 * @param  Length to be copied from m2 to m1.  
 * @return VOID. 
 */
uint8 extract_vap_id(const char *str)
{
	/* works for interface units {0..9}. Supported range for
	 * vapid is also {0...8}. This may not work for unit above 9.
	 */
	return str[strlen(str)-1] - '0';
}
#endif

/**
 * This function handle the copying of memory.
 *
 * @param  Pointer to memory1.  
 * @param  Pointer to memory2.  
 * @param  Length to be copied from m2 to m1.  
 * @return VOID. 
 */
void* rsi_memcpy(void *to, const void *from, int len)
{
	return memcpy(to, from, len);
}

/**
 * This function sets the memory to the given value.
 *
 * @param  Pointer to src.  
 * @param  Value to be filled in src.  
 * @param  length to be filled.  
 * @return VOID. 
 */
void* rsi_memset(void *src, int val, int len)
{
	return memset(src,val,len);
}

/**
 * This function gets a member called 'priv' in netdevice structure.
 *
 * @param  Pointer to device
 * @return VOID. 
 */
void* get_netdev_priv(void *addr)
{
	struct net_device *dev;
	dev = (struct net_device *)addr;
#if KERNEL_VERSION_GREATER_THAN_2_6_(27)
	return netdev_priv(dev);
#else
	return dev->priv;
#endif
}

/**
 * This function copys the string pointed by source to destination.
 *
 * @param  Pointer to destination address
 * @param  Pointer to source address
 * @return returns the pointer to the destination address
 */
uint8* rsi_strcpy(char *dest,const char *src)
{
	return strcpy(dest,src);
}

/**
 * This function compares the number of bytes pointed by
 * s1 to the num of bytes pointed by s2 
 *
 * @param  Pointer to block of memory
 * @param  Pointer to block of memory
 * @return Returns zero if they all match else a value different from zero
 */
int rsi_memcmp(const void *s1,const void *s2,int len)
{
	return memcmp(s1,s2,len);
}

/** This functions invokes the scheduler
 * @param   void
 * @return  void
 */
void rsi_schedule(void)
{
	schedule();
}

/** This function is used to trim the skb->data pointer to length size
 * by removing data from the end
 * @param   pointer to netbuf_ctrl_block structure
 * @param   length to which netbuf should be trimmed
 * @return void
 */

void netbuf_ctrl_block_t_trim(netbuf_ctrl_block_t *netbuf_cb_t, int len)
{
	struct sk_buff *skb = (struct sk_buff *)netbuf_cb_t->pkt_addr;
	skb->len = netbuf_cb_t->len;
	skb->data = netbuf_cb_t->data;
	if (len > 0)
	{
    	skb_trim(skb, len);
	}
	netbuf_cb_t->len = skb->len;
	return;
}

/** This function is used to pull the skb->data pointer by length size
 * @param   pointer to netbuf_ctrl_block structure
 * @param   amount of data to be removed
 * @return void
 */

void netbuf_ctrl_block_t_adj(netbuf_ctrl_block_t *netbuf_cb_t, int len)
{
	struct sk_buff *skb = (struct sk_buff *)netbuf_cb_t->pkt_addr;
	skb->len = netbuf_cb_t->len;
	skb->data = netbuf_cb_t->data;
	if (len > 0)
	{
		netbuf_cb_t->data = skb_pull(skb, len); //The pointer is pulled now to point to the new data
	}
#if 0
        else
                skb_trim(skb, -len);
#endif
	netbuf_cb_t->len = skb->len;
	return;
}

#ifdef BYPASS_RX_DATA_PATH
void indicate_hal_pkt_to_os(void *dev, netbuf_ctrl_block_t *netbuf_cb_t)
{
	struct sk_buff *skb = (struct sk_buff *)netbuf_cb_t->pkt_addr;
	if(netbuf_cb_t->data[28] == 0x88 && netbuf_cb_t->data[29] == 0x8e) /*EAPOL PKT INDICATE TO OS and update WINDOW accordingly*/
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("<<<<< Indicating EAPOL Pkt to OS and  seq_no %d >>>>>\n", (((*(uint16 *)&netbuf_cb_t->data[5]) >> 4) & 0xfff)));
	}

	skb->len = netbuf_cb_t->len;
	skb->data = netbuf_cb_t->data;

	skb->ip_summed = CHECKSUM_NONE; 
	skb->dev = dev;
	skb_pull(skb, FRAME_DESC_SZ);
	skb->protocol = eth_type_trans(skb, dev);
	skb->priority = (netbuf_cb_t->data[7] & 0x01); 
	netif_rx(skb);
 /* Freeing of skb will be taken care by os in case of RCV bypass mode */
}


void append_netbuf (netbuf_ctrl_block_t *dest, netbuf_ctrl_block_t *src	)
{
  struct sk_buff *skb  = dest->pkt_addr;
  struct sk_buff *skb1 = src->pkt_addr;
  struct sk_buff *ptr = skb;

  while (ptr->next != NULL)
  {
    ptr = skb->next;
  }
  ptr->next = skb1;
  skb1->next = NULL;
} 

netbuf_ctrl_block_t *get_last_netbuf(netbuf_ctrl_block_t *netbuf_cb)
{
  struct sk_buff *skb = netbuf_cb->pkt_addr;
  struct sk_buff *next_skb = skb;
  netbuf_ctrl_block_t *new_netbuf_cb = NULL;
  while (next_skb->next)
  {
	next_skb = next_skb->next;
  }
  new_netbuf_cb = (netbuf_ctrl_block_t *)next_skb->cb;
  return new_netbuf_cb;
}


#endif

/**
 * This function initializes the timer.
 *
 * @param  Pointer to timer.  
 * @param  expires.  
 * @return 0. 
 */
void initialize_timer(struct timer_list *timer, unsigned long data, void *function, unsigned long timeout)
{
  init_timer((struct timer_list*)timer);
	timer->data  = data;
	timer->function = function;
	timer->expires = timeout + jiffies;
	add_timer((struct timer_list *)timer);
	
}
/**
 * This function addesthe timer.
 *
 * @param  Pointer to timer.  
 * @param  expires.  
 * @return void. 
 */
void
onebox_add_timer(void* timer)
{
  add_timer((struct timer_list*)timer);
}

#if KERNEL_VERSION_LESS_THAN_3_6(0)
int interrupt_queue_work(struct workqueue_struct *wq, struct work_struct *work)
#else
bool interrupt_queue_work(struct workqueue_struct *wq,
			  struct work_struct *work)
#endif
{
	return queue_work(wq, work);
}


void remove_timer(void *timer)
{
  del_timer((struct timer_list*)timer);
}

int onebox_timer_pending(void *timer)
{
	return timer_pending((struct timer_list*)timer);
}

struct workqueue_struct *create_work_queue(uint8 *work_name)
{
	return create_workqueue(work_name);
}

/**
 * Extracts the payload from a nl-message received via a GENL Socket
 *
 * @gcb - Pointer to asset's specific genl data. 
 * @return - Pointer to the payload.
 * 	     NULL on error
 */
uint8 *onebox_genlrecv_handler(struct genl_cb *gcb)
{
	uint8 *data = NULL;
	struct nlattr *na;
	//struct genlmsghdr *ghdr;
	struct genl_info *info;

	if (!gcb)
		goto err;
	
	if (!(info = gcb->gc_info))
		goto err;

	if (!gcb->gc_done)
		goto err;

	//ghdr = info->genlhdr;

	/*
	 * pid should be updated, application has
	 * to send a data first.
	 */
	gcb->gc_pid = get_portid(info);
	gcb->gc_seq = info->snd_seq;

	na = info->attrs[RSI_USER_A_MSG];
	if (na) {
		data = (uint8 *)nla_data(na);
		if (!data) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			    (TEXT("genlrecv_handler: no data recevied "
			          "on family `%s' of asset `%s'\n"),
			     gcb->gc_name, ASSET_NAME(gcb->gc_assetid))); 
			goto err;
		}
	} else {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		    (TEXT("genlrecv_handler: `na' is NULL on "
			  "family `%s' of asset `%s'\n"),
		     gcb->gc_name, ASSET_NAME(gcb->gc_assetid))); 
		goto err;
	}

	return data;

err:
	return NULL;
}

/** 
 * Sends a message to the userspace via netlink socket
 *
 * @gcb         Pointer to asset's specific genl data.
 * @netbuf_cb   Pointer to the netbuf_ctrl_block contining the payload
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 */
int32 onebox_genlsend(struct genl_cb *gcb, netbuf_ctrl_block_t *netbuf_cb)
{
	uint8 *data;
	uint32 len;
	int32 rc;
	void *hdr;
	struct sk_buff *gskb;
	struct net *net = &init_net;

	if (!gcb || !netbuf_cb)
		return -EFAULT;

	if (!gcb->gc_done)
		return -ENODEV;	

	data = netbuf_cb->data;
	len  = netbuf_cb->len;

	if (!data || !len)
		return -ENODATA;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	   (TEXT("genlsend: sending data %p with len-%d to pid-%d \n from "
		 "asset-`%s' over family-`%s'\n"), data, len, gcb->gc_pid,
	   ASSET_NAME(gcb->gc_assetid), gcb->gc_name)); 

	gskb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (!gskb) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		    (TEXT("genlsend: 'genlmsg_new' failed"
			  " family `%s' asset `%s'\n"), gcb->gc_name,
		    ASSET_NAME(gcb->gc_assetid)));
		rc = -ENOMEM; 
		goto err;
	}

	hdr = genlmsg_put(gskb, 0, gcb->gc_seq + 1, 
              		  gcb->gc_family, 0, RSI_USER_C_CMD);
	if (!hdr) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		    (TEXT("genlsend: 'genlmsg_put' failed"
			  " family `%s' asset `%s'\n"), gcb->gc_name,
		    ASSET_NAME(gcb->gc_assetid)));
		rc = -EMSGSIZE;
		goto err_fill;
	}

	rc = nla_put(gskb, RSI_USER_A_MAX, len, data);
	if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		    (TEXT("genlsend: 'nla_put' fail(%d)\n"
			  " family `%s' asset `%s'\n"), rc, gcb->gc_name,
		    ASSET_NAME(gcb->gc_assetid)));
		goto err_fill;
	}

	genlmsg_end(gskb, hdr);
#ifndef RSI_IMX51
#if KERNEL_VERSION_BTWN_2_6_(30,31)
	rc = genlmsg_unicast(gskb, gcb->gc_pid);
#else
	rc = genlmsg_unicast(net, gskb, gcb->gc_pid);
#endif
	if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		   (TEXT("genlsend: 'genlmsg_unicast' failed(%d)\n"
			 " family `%s' asset `%s'\n"), rc, gcb->gc_name,
		    ASSET_NAME(gcb->gc_assetid)));
		goto err;
	}
#endif
	return rc;

err_fill:
	nlmsg_free(gskb);
err:
	return rc;
}

/**
 * Registers a genl family and operations
 *
 * @gcb         Pointer to asset's specific GENL data
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 */
int32 onebox_register_genl(struct genl_cb *gcb)
{
	int32 rc = -1;	
	struct genl_ops *ops;
	struct genl_family *family;

	if (!gcb)
		return -EFAULT;

	if (gcb->gc_done) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		    (TEXT("genl-register: nl_family `%s' asset "
			  "`%s' exist, fail\n"), gcb->gc_name,
		    ASSET_NAME(gcb->gc_assetid)));
		return -EEXIST;
	}
	
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	    (TEXT("genl-register: nl_family `%s' asset `%s'\n"),
	    gcb->gc_name, ASSET_NAME(gcb->gc_assetid)));

	family = gcb->gc_family;
	ops    = gcb->gc_ops;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
	family->ops   = gcb->gc_ops;
	family->n_ops = gcb->gc_n_ops;
#endif

	rc = genl_register_family(family);  
	if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		   (TEXT("genl-register: `genl_register_family' fail %d\n")
		   ,rc)); 
		return rc;
	}	

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 12, 34)
	rc = genl_register_ops(family, ops); 
	if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		  (TEXT("genl-register: `genl_register_ops' fail %d\n"), rc)); 
		genl_unregister_family(family);
		return rc;
	}
#endif
	gcb->gc_done = 1;

	return rc;
}

/*==============================================*/
/**
 * @fn          int onebox_unregister_genl(struct genl_cb *gcb)
 * @brief       Unregisters genl family and operations
 * @param[in]   struct genl_cb *gcb, pointer to asset's specific Genl data.
 * @param[out]  none
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 * @section description
 * This API is used to unregister genl related ops.
 *=======================================================*/

int32 onebox_unregister_genl(struct genl_cb *gcb)
{
	int32 rc = -1;	

	if (!gcb) 
		return -EFAULT;

	if (!gcb->gc_done) {
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	          (TEXT("genl-unregister: family `%s' asset `%s'"
		  " not registered yet\n"), gcb->gc_name, 
	          ASSET_NAME(gcb->gc_assetid)));
		return -ENODEV;
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	    (TEXT("genl-unregister: family `%s' asset `%s'\n"),
	    gcb->gc_name, ASSET_NAME(gcb->gc_assetid)));

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 12, 34)
	rc = genl_unregister_ops(gcb->gc_family, gcb->gc_ops);
	if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		    (TEXT("genl-unregister: `genl_unregister_ops' "
			  "family %s asset %s fail %d\n"),
		    gcb->gc_name, ASSET_NAME(gcb->gc_assetid), rc));
		return rc;
	}
#endif
	rc = genl_unregister_family(gcb->gc_family);
	if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		    (TEXT("genl-unregister: `genl_unregister_family' "
		  	  "family %s asset %s fail %d\n"),
		     gcb->gc_name, ASSET_NAME(gcb->gc_assetid), rc));
		return rc;
	}

	gcb->gc_done = 0;
	
	return rc;
}

ONEBOX_STATIC int32 common_module_init(VOID)
{
	return 0;
}

ONEBOX_STATIC void common_module_exit(VOID)
{
	return ;
}

EXPORT_SYMBOL(onebox_zone_enabled);
module_init(common_module_init);
module_exit(common_module_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Redpine Signals, Inc.");
MODULE_DESCRIPTION("Driver for Redpine Signals' RS9113 module based USB/SDIO cards.");
MODULE_SUPPORTED_DEVICE("Redpine Signals' RS9113 module based USB/SDIO cards.");
