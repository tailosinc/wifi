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

#include "onebox_common.h"
#include "onebox_thread.h"
#include "onebox_linux.h"

#include <linux/usb.h>

/* OS_INTERFACE OPERATIONS */
struct onebox_os_intf_operations 
{
	uint8* (*onebox_strcpy)(char *s1, const char *s2);
	void (*onebox_mem_alloc)(void **ptr,uint16 len, gfp_t flags);
	void* (*onebox_mem_zalloc)(int size, gfp_t flags);
	void (*onebox_mem_free)(VOID *ptr);
	void  (*onebox_vmem_free)(VOID *ptr); 
	void* (*onebox_memcpy)(void *to, const void *from, int len);
	void* (*onebox_memset)(void *src, int val, int len);
	int   (*onebox_memcmp)(const void *s1,const void *s2,int len);
	void* (*onebox_memmove)(void *s1, const void *s2, int n);
	//uint8 (*onebox_extract_vap_id)(const char *str);
	int (*onebox_wait_event)(ONEBOX_EVENT *event,uint32 timeOut);
	void (*onebox_set_event)(ONEBOX_EVENT *event);
	void (*onebox_reset_event)(ONEBOX_EVENT *event);
	int  (*onebox_mod_timer)( void* timer,
	                          unsigned long expires);
	void  (*onebox_do_gettimeofday)(VOID *tv_now);
	void* (*onebox_vmalloc)(unsigned long size);
	//Fill sys_time_struct
	// void  (*onebox_fill_sys_time_struct)(SYS_TIME_STRUCT *ptr);
	void  (*onebox_usec_delay)(unsigned int usecs);
	void  (*onebox_msec_delay)(unsigned int msecs);  
	void   (*onebox_acquire_sem_interruptible)(PVOID sem,
	                                           int  delay_msec);
	ONEBOX_STATUS (*onebox_acquire_sem)(PVOID sem, int  delay_msec);
	BOOLEAN (*onebox_release_sem)(PVOID sem);
	int32 (*onebox_init_event)(ONEBOX_EVENT *pEvent);
	int32 (*onebox_delete_event)(ONEBOX_EVENT *pEvent);
	void (*onebox_queue_head_init)(struct sk_buff_head *list);
	void (*onebox_queue_purge)(struct sk_buff_head *list);
	void (*onebox_init_dyn_mutex)(struct semaphore *sem_name);
	void (*onebox_init_static_mutex)(void *mutex);
	int  (*onebox_acquire_mutex)(void *  sem);
	void (*onebox_release_mutex)(PVOID mutex);
	void (*onebox_init_spinlock)(void *lock);
	void (*onebox_acquire_spinlock)(void* lock,unsigned long *flags);
	void (*onebox_release_spinlock)(void* lock,unsigned long flags);
	void (*onebox_spin_lock_irqsave)(void *lock , int flags);
	void (*onebox_spin_lock_irqrestore)(void *lock , int flags);
	unsigned long (*onebox_get_jiffies) (void);
	void (*onebox_get_random_bytes)(void *buf, int nbytes);
	void (*onebox_free_pkt)(netbuf_ctrl_block_t* pkt,
	                        ONEBOX_STATUS Status);
	struct proc_dir_entry* (*onebox_init_proc)(void *adapter, uint8 *proc_name);
	void (*onebox_remove_proc_entry)(struct proc_dir_entry *onebox_entry, uint8 *proc_name);
	int (*onebox_start_thread)(onebox_thread_handle_t *handle);
	int (*onebox_init_thread)(onebox_thread_handle_t *handle, uint8 *name,
	                          uint32 priority, thread_function function_ptr,
	                          void *context);
	int (*onebox_kill_thread)(onebox_thread_handle_t *handle);
	void (*onebox_init_workq)(struct work_struct *work, void *function);
	void (*onebox_deinit_workq)(struct workqueue_struct *work);
	int32 (*onebox_request_irq)(unsigned int irq, void * handler,
	                            unsigned long  irqflags,
	                            const char *  devname,
	                            void *  dev_id);
	int  (*onebox_set_irq_type)(unsigned int irq,unsigned int type);
	struct workqueue_struct * (*onebox_create_singlethread_workqueue)(const char *name);
	PUCHAR (*onebox_get_firmware)(const int8 *fn,uint32 *read_length,
	                               uint32 type,uint8 *firmware_path);
	int(*onebox_write_to_file)(const int8 *fn, uint16 *dp, uint32 write_len, uint32 type,
				 uint8 *file_path);
	PUCHAR (*onebox_add_data_to_skb)(netbuf_ctrl_block_t *netbuf_cb, int len);
	netbuf_ctrl_block_t* (*onebox_alloc_skb)(int len);
	int (*onebox_is_ifp_txq_stopped)(void *dev);
	int (*onebox_is_sub_txq_stopped)(void *dev, int ac);
	void (*onebox_start_netq)(void *dev);
	void (*onebox_start_sub_txq)(void *dev, int ac);
	void (*onebox_start_ifp_txq)(void *dev);
	void (*onebox_stop_ifp_txq)(void *dev);
	void (*onebox_stop_sub_txq)(void *dev, int ac);
	void (*onebox_create_kthread)(struct work_struct *work);
	netbuf_ctrl_block_t* (*onebox_allocate_skb)(int len);
	PUCHAR (*onebox_push_data)(void *addr,int len);
  	void (*onebox_reserve_data)(netbuf_ctrl_block_t *netbuf_cb, int reserve_len);
	netbuf_ctrl_block_t* (*onebox_dequeue_pkt)(void * addr);
	ONEBOX_STATUS (*onebox_change_hdr_size)(netbuf_ctrl_block_t *netbuf_cb, uint16 len);
	void (*onebox_netbuf_queue_init)(void *addr);
	void (*onebox_netbuf_queue_tail)(void *addr,void *buffer);
	int (*onebox_netbuf_queue_len)(void *addr);
	void* (*onebox_get_priv)(void *addr);
	void (*onebox_unregisterdev)(struct net_device *dev);
	struct net_device * (*onebox_netdevice_op)(void);
	void (*onebox_schedule)(void);
	
	int (*onebox_atomic_read)(void *addr);
	void (*onebox_completion_event)(void *addr,int val );
	
	void (*onebox_netbuf_adj)(netbuf_ctrl_block_t *netbuf_cb_t, int len);
	void (*onebox_netbuf_trim)(netbuf_ctrl_block_t *netbuf_cb_t, int len);
#ifdef BYPASS_RX_DATA_PATH
	void (*onebox_indicate_pkt_to_os)(void *dev, netbuf_ctrl_block_t *netbuf_cb_t);

	void (*onebox_append_netbuf_cb)(netbuf_ctrl_block_t *dest, netbuf_ctrl_block_t *src );
	netbuf_ctrl_block_t * (*onebox_get_last_netbuf_cb)(netbuf_ctrl_block_t *netbuf_cb);
#endif
	void (*onebox_init_sw_timer)(struct timer_list *timer, unsigned long data, void *function, unsigned long timeout);
	void (*onebox_add_sw_timer)(void *timer);
	void (*onebox_remove_timer)(void *timer);
	int  (*onebox_sw_timer_pending)(void *timer);
#if KERNEL_VERSION_LESS_THAN_3_6(0)
	int (*onebox_queue_work)(struct workqueue_struct *wq, struct work_struct *work);
#else
	bool (*onebox_queue_work)(struct workqueue_struct *, struct work_struct *);
#endif
	void (*onebox_tasklet_sched)(struct tasklet_struct *);
	void (*onebox_netbuf_queue_head)(void *addr,void *buffer);
	struct workqueue_struct * (*onebox_create_work_queue)(uint8 *work_queue_name);
	int32 (*onebox_genl_init)(struct genl_cb * gcb);
	int32 (*onebox_genl_deinit)(struct genl_cb * gcb);
	int32 (*onebox_genl_app_send)(struct genl_cb *gcb,
				      netbuf_ctrl_block_t *netbuf_cb);
	uint8 *(*onebox_genl_recv_handle)(struct genl_cb *gcb);
#ifdef GPIO_HANDSHAKE
	void (*onebox_set_host_status)(int value);
	void (*onebox_gpio_init)(void);
	void (*onebox_gpio_deinit)(void);
	int32 (*onebox_get_device_status)(void );
#endif

};

/* EOF */
