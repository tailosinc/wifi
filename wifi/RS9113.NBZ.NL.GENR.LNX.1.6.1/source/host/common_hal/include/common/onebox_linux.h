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

#ifndef __ONEBOX_AP_LINUX_H__
#define __ONEBOX_AP_LINUX_H__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
//#include <linux/pci.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/rtnetlink.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/vermagic.h>
#include <asm/io.h>
//#include <asm/system.h>
#include <linux/mod_devicetable.h>
#include <linux/proc_fs.h>              /*** For Proc File System ***/
#include <linux/wireless.h>             /*** Wireless extensions  ***/
#include <net/iw_handler.h>             /*** Wireless handlers    ***/
#include <linux/workqueue.h>
#include <linux/version.h>
#include <asm/byteorder.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>

#include "onebox_common.h"
#include "onebox_thread.h"
#include "onebox_datatypes.h"


typedef struct sk_buff_head   onebox_netbuf_head_t;
typedef spinlock_t            onebox_spinlock_t; 


#define TEXT(arg) arg

#ifdef ONEBOX_DEBUG_ENABLE
#define ONEBOX_DEBUG(zone, x) \
       if((!zone) || (zone & onebox_zone_enabled)) printk x
#else
#define ONEBOX_DEBUG(zone, x)
#endif

/* FUNCTION PROTOTYPES */
ONEBOX_STATUS onebox_data_buff_alloc(uint32 *phy_addr,
                                     uint32 *netbuf_addr,
                                     uint16 pkt_len);
int device_open(struct net_device *dev);
int device_close(struct net_device *dev);
int onebox_xmit(struct sk_buff *skb, struct net_device *dev);
int zigb_xmit(struct sk_buff *skb, struct net_device *dev);
void mem_alloc(void **ptr, uint16 len, gfp_t flags);
void* mem_zalloc(int size,int flags);
VOID vfreemem(VOID *ptr);
VOID freemem(VOID *ptr);
VOID rsi_set_event(ONEBOX_EVENT *event);
int wait_queue_event(ONEBOX_EVENT *event_ptr, uint32 timeout);
VOID rsi_reset_event(ONEBOX_EVENT *event);
unsigned long get_jiffies (void);

int down_interruptible_sem(void* sem, int delay);
ONEBOX_STATUS down_sem(void* sem, int delay);
BOOLEAN up_sem(void* sem);
int modify_timer(void* timer,unsigned long expires);
void usec_delay (unsigned long usecs);
void msec_delay (unsigned int msecs);

int32 init_event(ONEBOX_EVENT *pEvent);
int32 delete_event(ONEBOX_EVENT *pEvent);
int32 wakeup_event(ONEBOX_EVENT *event_ptr);
void queue_head_init(struct sk_buff_head *list);
void queue_purge(struct sk_buff_head *list);
void proc_entry_remove(struct proc_dir_entry *onebox_entry, uint8 *proc_name);


int init_thread(onebox_thread_handle_t *handle, uint8 *name,
                uint32 priority, thread_function function_ptr,
                void *context);
int init_wlan_thread(onebox_thread_handle_t *handle, uint8 *name,
                     uint32 priority, thread_function function_ptr,
                     void *context);
void per_thread(void *pContext);
int kill_thread(onebox_thread_handle_t *handle);

int32 intr_request_irq (uint32 irq, void * handler, ulong irqflags, const char *devname,
                        void *dev_id);
void free_pkt(netbuf_ctrl_block_t *netbuf_cb,int status);

void start_networkq(void *dev);

void stop_ifp_txq(void *dev);
void stop_sub_txq(void *dev, int ac);
void start_ifp_txq(void *dev);
void start_sub_txq(void *dev, int ac);
int txq_ifp_stopped(void *dev);
int txq_sub_stopped(void *dev, int ac);

void stop_all_txq(void *dev);
void start_all_txq(void *dev);
int check_stop_all_txq(void *dev);

VOID spinlock_init(VOID *lock);
VOID acquire_spinlock(PVOID lock,unsigned long *flags);
VOID release_spinlock(PVOID lock,unsigned long flags);
void init_dyn_mutex(struct semaphore *sem_name);
void init_static_mutex(void *mutex);
int acquire_mutex(void* mutex);
void release_mutex(void *mutex);

void create_kernel_thread(struct work_struct *work);
netbuf_ctrl_block_t* allocate_skb(int len);
ONEBOX_STATUS push_data(netbuf_ctrl_block_t *netbuf, uint16 len);
PUCHAR put_data(netbuf_ctrl_block_t *netbuf_cb, int len);
void reserve_data(netbuf_ctrl_block_t *netbuf_cb, int reserve_len);


void init_workq(struct work_struct *work, void *function);
void deinit_workq(struct workqueue_struct *work);

netbuf_ctrl_block_t* dequeue_pkt(void *addr);

PUCHAR get_firmware (const int8 *fn, uint32 *read_length, uint32 type,
                     uint8 *fw_path);
int write_to_file (const int8 *fn, uint16 *dp, uint32 write_len, 
			uint32 type, uint8 *file_path);
void get_skb_queue_head_init(void *addr);
void get_skb_queue_tail(void *addr,void *buffer);
void skb_queue_head_add(void *addr,void *buffer);
struct workqueue_struct *create_work_queue(uint8 *work_name);
int get_skb_queue_len(void *addr);
void* rsi_memcpy(void *to, const void *from, int len);
void* rsi_memset(void *src, int val, int len);
int rsi_memcmp(const void *s1,const void *s2,int len);
void* get_netdev_priv(void *addr);
void  unregister_dev(struct net_device *dev);
struct net_device* wlan_netdevice_op(VOID);
struct net_device* zigb_netdevice_op(VOID);
uint8* rsi_strcpy(char *dest,const char *src);
void rsi_schedule(void);
uint8 extract_token ( struct file *fp, uint8 * token);

int read_atomic_var(void *addr);
void completion_exit_event(void *addr,int val);
void netbuf_ctrl_block_t_adj(netbuf_ctrl_block_t *netbuf_cb_t, int len);
void netbuf_ctrl_block_t_trim(netbuf_ctrl_block_t *netbuf_cb_t, int len);
#ifdef BYPASS_RX_DATA_PATH
void indicate_hal_pkt_to_os(void *dev, netbuf_ctrl_block_t *netbuf_cb_t);
void append_netbuf (netbuf_ctrl_block_t *dest, netbuf_ctrl_block_t *src	);
netbuf_ctrl_block_t *get_last_netbuf(netbuf_ctrl_block_t *netbuf_cb);
#endif
void initialize_timer(struct timer_list *timer, unsigned long data, void *funtion, unsigned long timeout);
void onebox_add_timer(void* timer);
void remove_timer(void *timer);
int onebox_timer_pending(void *timer);
#if KERNEL_VERSION_LESS_THAN_3_6(0)
int interrupt_queue_work(struct workqueue_struct *wq, struct work_struct *work);
#else
bool interrupt_queue_work(struct workqueue_struct *wq,
			  struct work_struct *work);
#endif

//uint8 extract_vap_id(const char *str);
int32 onebox_genlsend(struct genl_cb *gcb, netbuf_ctrl_block_t *netbuf_cb);
int32 onebox_register_genl(struct genl_cb *gcb);
int32 onebox_unregister_genl(struct genl_cb *gcb);
uint8 *onebox_genlrecv_handler(struct genl_cb *gcb);
#endif
