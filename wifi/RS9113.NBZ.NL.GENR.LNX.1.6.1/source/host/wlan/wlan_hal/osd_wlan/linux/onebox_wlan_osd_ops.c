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

#include <linux/hrtimer.h>
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
#include <asm/string.h>
#include <linux/fs.h>
#include <linux/file.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
#include <linux/smp.h>
#include <linux/kthread.h>
#else
#include <linux/smp_lock.h>
#endif

#include "wlan_common.h"
#include "onebox_linux.h"
#include "onebox_sdio_intf.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
#include <linux/semaphore.h>
#endif



#if 0
/**
 * This function is a wrapper to be used for all Linux thread functions.
 *
 * @param  context
 * @return status
 */
static int onebox_internal_thread(void *context)
{
	onebox_thread_handle_t *handle = (onebox_thread_handle_t*)context;

#if 1 
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
	int status;
	status = mutex_lock_interruptible(&handle->thread_lock);
	if(status)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("In %s: mutex lock if failed with error status = %d\n"),__func__, status));
	}
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
 	lock_kernel();
#endif
#endif
	siginitsetinv(&current->blocked,
	              sigmask(SIGKILL)|sigmask(SIGINT)|sigmask(SIGTERM));
	strcpy(current->comm, handle->name);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
	daemonize(handle->name);
	allow_signal(SIGKILL);
#ifdef CONFIG_PM
	current->flags |= PF_NOFREEZE;
#endif
#else
	daemonize();
	reparent_to_init();
#endif
#if 1
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
	mutex_unlock(&handle->thread_lock);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
	unlock_kernel();
#endif
#endif
	up(&handle->sync_thread);
	do
	{
		handle->function_ptr(handle->context);
		if (onebox_signal_pending())
		{
			flush_signals(current);
		}
	}while (handle->kill_thread == 0);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
	current->io_context = NULL;
#endif
	complete_and_exit(&handle->completion, 0);
	return 0;
}

/**
 * This function creates a kernel thread
 *
 * @param Pointer to the work_struct
 * @return void
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
void create_kernel_thread(struct work_struct *work)
{  
	onebox_thread_handle_t *handle = container_of(work, onebox_thread_handle_t,
	                                              task_queue);
	handle->thread_id = kernel_thread(onebox_internal_thread, handle, 0);
	return;
}
#else
void create_kernel_thread(void *data)
{  
	onebox_thread_handle_t *handle = data;
	handle->thread_id = kernel_thread(onebox_internal_thread, handle, 0);
	return;
}
#endif
#endif

/**
 * This function creates a thread and initializes the thread
 * function and name.
 *
 * @param Pointer to onebox_thread_handle_t structure
 * @param Thread name                         
 * @param Thread priority         
 * @param Thread function name
 * @param Context to be passed to the thrd
 * @return Success or Failure status
 */
int init_wlan_thread(onebox_thread_handle_t *handle, uint8 *name,
                     uint32 priority, thread_function function_ptr,
                     void *context)
{
	struct onebox_os_intf_operations *os_intf_ops = onebox_get_os_intf_operations_from_origin();
	WLAN_ADAPTER  w_adapter;
	ONEBOX_ASSERT(function_ptr);
	ONEBOX_ASSERT(handle);
	ONEBOX_ASSERT(name);
	w_adapter = context;
	if (!(function_ptr && handle && name))
	{
		return ONEBOX_STATUS_FAILURE;
	}

	memset(handle, 0, sizeof(onebox_thread_handle_t));
	handle->function_ptr = function_ptr;
	handle->context = context;
	handle->kill_thread = 0;
	init_completion(&handle->completion);
	atomic_set(&w_adapter->txThreadDone,0);
	init_completion(&w_adapter->txThreadComplete);
	sema_init(&handle->sync_thread, 1);
	down(&handle->sync_thread);

	/* Make sure that the name does not exceed the length */
	strncpy(handle->name, name, ONEBOX_THREAD_NAME_LEN);
	handle->name[ONEBOX_THREAD_NAME_LEN] = '\0';
#if KERNEL_VERSION_GREATER_THAN_2_6_(35)
	mutex_init(&handle->thread_lock);
#endif
	/* Initialize Kernel Start Tasklet */
	//INIT_WORK(&handle->task_queue, create_kernel_thread);
	INIT_WORK(&handle->task_queue, os_intf_ops->onebox_create_kthread);
	return 0;
}

/**
 * This function kills the thread
 *
 * @param  Pointer to onebox_thread_handle_t structure
 * @return status
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 26)
int kill_wlan_thread(onebox_thread_handle_t *handle)
{
	int status;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("** Before locking kernel **\n")));
	lock_kernel();
#endif

	handle->kill_thread = 1;
	status = kill_proc((struct task_struct *)handle->thread_id, SIGKILL, 1);
	if (status)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             ("onebox_kill_wlan_thread: Unable to Kill Thread %s\n",
		             handle->name));
		return ONEBOX_STATUS_FAILURE;
	}
	else
	{
		wait_for_completion(&handle->completion);
	}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("*Before unlocking kernel **\n")));
	unlock_kernel();
#endif
	/* Cleanup Zombie Threads */
	kill_proc((struct task_struct *)2, SIGCHLD, 1);
	return 0;
}
#elif KERNEL_VERSION_GREATER_THAN_2_6_(27)
int kill_wlan_thread(WLAN_ADAPTER w_adapter)
{
	onebox_thread_handle_t *handle = &w_adapter->sdio_scheduler_thread_handle;
	struct onebox_os_intf_operations *os_intf_ops = onebox_get_os_intf_operations_from_origin();
	struct driver_assets *d_assets = w_adapter->d_assets;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("IN KILL WLAN THREAD %s %d\n"), __func__, __LINE__));

	if((handle->kill_thread) || (handle->function_ptr == NULL)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Ptr is NULL \n")));
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("IN KILL WLAN THREAD %s %d\n"), __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}
		if (d_assets->techs[WLAN_ID].deregister_flags) {
			d_assets->techs[WLAN_ID].deregister_flags = 0;
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("IN KILL WLAN THREAD %s %d\n"), __func__, __LINE__));
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Waking up an event \n")));
			wake_up(&d_assets->techs[WLAN_ID].deregister_event);
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("IN KILL WLAN THREAD %s %d\n"), __func__, __LINE__));
		}

	if (!handle->kill_thread) {
		handle->kill_thread = 1;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("In %s line no %d\n"),__func__, __LINE__));
		atomic_inc(&w_adapter->txThreadDone);
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("In %s line no %d Before setting event\n"),__func__, __LINE__));
		os_intf_ops->onebox_set_event(&(w_adapter->sdio_scheduler_event));
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
		//handle->thread_id = NULL;
		kthread_stop(handle->thread_id);
#else
		wait_for_completion(&w_adapter->txThreadComplete);
		handle->thread_id = 0;
#endif
	} 
	  
	return ONEBOX_STATUS_SUCCESS;
}
#endif


/** This function is used to get the skb control buffer 
 * @param  pointer to netbuf_ctrl_block_m_t structure
 * @return pointer to netbuf_ctrl_block structure
 */
netbuf_ctrl_block_t * get_skb_cb(netbuf_ctrl_block_m_t *netbuf_cb_m)
{
	struct sk_buff *skb;
	netbuf_ctrl_block_t *netbuf_cb;
	skb = (struct sk_buff *)netbuf_cb_m->pkt_addr;
	netbuf_cb = (netbuf_ctrl_block_t *)*((unsigned long int *)skb->cb);
	return netbuf_cb;
}

int kill_per_thread(WLAN_ADAPTER w_adapter)
{
	onebox_thread_handle_t *handle ;
	if(w_adapter->total_per_pkt_sent)
	{
		handle = &w_adapter->sdio_scheduler_thread_handle_per;
		handle->kill_thread = 1;
		wait_for_completion(&w_adapter->txThreadComplete);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
		handle->thread_id = NULL;
#else
		handle->thread_id = 0;
#endif
	}
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * PER Burst Mode
 */
void per_thread(void *pContext)
{
	WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)pContext;
	ONEBOX_STATUS status;
	int  no_of_fragments= 0; 
	no_of_fragments = w_adapter->no_of_per_fragments = w_adapter->endpoint_params.aggr_count;
	do  
	{  
		status = w_adapter->core_ops->onebox_start_per_burst(w_adapter);
                no_of_fragments--;
		//	} while (!handle->kill_thread && !status ); 
	} while (no_of_fragments && w_adapter->endpoint_params.aggr_enable); 
	complete_and_exit(&w_adapter->txThreadComplete, 0); 
}
