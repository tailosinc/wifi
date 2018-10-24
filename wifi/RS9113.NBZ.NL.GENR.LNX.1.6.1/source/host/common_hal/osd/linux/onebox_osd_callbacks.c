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
#include <linux/sched.h>
#include <net/iw_handler.h>     /* New driver API */

#include <linux/string.h>
#include <asm/string.h>

#include "onebox_common.h"
#include "onebox_linux.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
#include <linux/semaphore.h>
#endif
#if((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))&& \
   (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)))
#include <linux/sdio/ctsystem.h>
#endif

/* Os_interface operations can be called by Core/HAL */
static struct onebox_os_intf_operations os_intf_ops = {
	.onebox_strcpy = rsi_strcpy, 
	.onebox_mem_alloc = mem_alloc,
	.onebox_mem_zalloc = (void *)kzalloc,
	.onebox_mem_free = freemem,
	.onebox_vmalloc = vmalloc,
	.onebox_vmem_free = vfreemem,
	.onebox_memcpy = (void *)rsi_memcpy,
	.onebox_memset = (void *)rsi_memset,
	.onebox_memcmp = (void *)rsi_memcmp,
	.onebox_memmove = (void *)memmove,
	//.onebox_extract_vap_id=extract_vap_id,
	.onebox_wait_event = wait_queue_event,
	.onebox_set_event = rsi_set_event,
	.onebox_reset_event = rsi_reset_event,
	.onebox_mod_timer = modify_timer,  
	.onebox_do_gettimeofday = (void *)do_gettimeofday,
	.onebox_usec_delay = (void *)usec_delay,
	.onebox_msec_delay = msec_delay,
	.onebox_acquire_sem_interruptible = (void *)down_interruptible_sem,  
	.onebox_acquire_sem = (void *)down_sem,
	.onebox_release_sem = up_sem,
	.onebox_init_event = init_event,
	.onebox_delete_event = delete_event,
	.onebox_queue_head_init = queue_head_init,
	.onebox_queue_purge = queue_purge,
	.onebox_init_dyn_mutex = init_dyn_mutex,
	.onebox_init_static_mutex = init_static_mutex,
	.onebox_init_proc = init_proc_fs,
	.onebox_remove_proc_entry = proc_entry_remove,
	.onebox_init_thread = init_thread,
	.onebox_create_kthread = create_kernel_thread,
	.onebox_start_thread = rsi_start_thread,
	.onebox_kill_thread = kill_thread,
	.onebox_init_workq = init_workq,
	.onebox_deinit_workq = deinit_workq,
	.onebox_request_irq = intr_request_irq,
	.onebox_get_jiffies = get_jiffies,
	.onebox_free_pkt = free_pkt, 
	.onebox_alloc_skb = allocate_skb, 
	.onebox_change_hdr_size = push_data,
	.onebox_add_data_to_skb = put_data,
	.onebox_reserve_data = reserve_data,
	.onebox_get_priv = get_netdev_priv,
	.onebox_dequeue_pkt = dequeue_pkt,
	.onebox_get_random_bytes = get_random_bytes,
	.onebox_get_firmware = get_firmware,
	.onebox_write_to_file = write_to_file,
	.onebox_acquire_mutex = acquire_mutex,
	.onebox_release_mutex = release_mutex,
	.onebox_init_spinlock = spinlock_init,
	.onebox_release_spinlock = release_spinlock,
	.onebox_acquire_spinlock = acquire_spinlock,
	.onebox_is_ifp_txq_stopped=txq_ifp_stopped,
	.onebox_is_sub_txq_stopped=txq_sub_stopped,
	.onebox_start_netq = start_networkq,
	.onebox_start_sub_txq = start_sub_txq,
	.onebox_start_ifp_txq = start_ifp_txq,
	.onebox_stop_ifp_txq = stop_ifp_txq,
	.onebox_stop_sub_txq = stop_sub_txq,
	.onebox_netbuf_queue_len = get_skb_queue_len,
	.onebox_netbuf_queue_tail = get_skb_queue_tail,
	.onebox_netbuf_queue_init = get_skb_queue_head_init, 
	.onebox_unregisterdev = NULL,//unregister_dev,
	.onebox_netdevice_op = NULL,//get_netdevice_op,
	.onebox_atomic_read = read_atomic_var,
	.onebox_completion_event = completion_exit_event,
	.onebox_netbuf_adj = netbuf_ctrl_block_t_adj,
	.onebox_netbuf_trim = netbuf_ctrl_block_t_trim,
#ifdef BYPASS_RX_DATA_PATH
	.onebox_indicate_pkt_to_os = indicate_hal_pkt_to_os,
	.onebox_append_netbuf_cb = append_netbuf,
	.onebox_get_last_netbuf_cb = get_last_netbuf,
#endif
	.onebox_init_sw_timer = initialize_timer,
	.onebox_add_sw_timer = onebox_add_timer,
	.onebox_remove_timer = remove_timer,
	.onebox_sw_timer_pending = onebox_timer_pending,
	.onebox_queue_work = interrupt_queue_work,
	.onebox_tasklet_sched = tasklet_schedule,
	.onebox_netbuf_queue_head = skb_queue_head_add,
	.onebox_create_work_queue = create_work_queue,

	.onebox_genl_init = onebox_register_genl,
	.onebox_genl_deinit = onebox_unregister_genl,
	.onebox_genl_app_send = onebox_genlsend,
	.onebox_genl_recv_handle = onebox_genlrecv_handler,
#ifdef GPIO_HANDSHAKE
	.onebox_gpio_init = gpio_init, // For GPIO Initialization
	.onebox_gpio_deinit = gpio_deinit,//For GPIO Deinitialization
	.onebox_set_host_status = set_host_status,// For GPIO Toggle
	.onebox_get_device_status = get_device_status,	// For Reading GPIO Status
#endif
};

/**
 * This function returns the pointer to the os interface operations structure
 * @param  void   
 * @return Returns address of the os_intf_operations structure. 
 */
struct onebox_os_intf_operations *onebox_get_os_intf_operations_from_origin(void)
{
	return &os_intf_ops;
}
EXPORT_SYMBOL(onebox_get_os_intf_operations_from_origin);
