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

/* Include Files */
#include "onebox_common.h"
#include "onebox_linux.h"
#include "onebox_sdio_intf.h"
//#if KERNEL_VERSION_EQUALS_3_2_(14)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
#include <linux/smp.h>
#include <linux/kthread.h>
#else
#include <linux/smp_lock.h>
#endif
#include "onebox_zone.h"
#include <linux/signal.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
#include <linux/sched/signal.h>
#endif

/* Check again for higher versions*/
#if KERNEL_VERSION_GREATER_THAN_2_6_(28)
#define kill_proc(A, B, C) send_sig(B, A, C)
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28))
#define kill_proc(A, B, C) kill_proc(A, B, C)
#endif


/**
 * This function starts the thread
 *
 * @param  Pointer to onebox_thread_handle_t structure
 * @return status
 */
int rsi_start_thread(onebox_thread_handle_t *handle) 
{
	onebox_schedule_work(&handle->task_queue);
	down(&handle->sync_thread);
	return 0;
}

/**
 * This function kills the thread
 *
 * @param  Pointer to onebox_thread_handle_t structure
 * @return status
 */
int kill_thread(onebox_thread_handle_t *handle)
{
#if KERNEL_VERSION_BTWN_2_6_(18, 26)
	int status;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("** Before locking kernel **\n")));
	lock_kernel();
#endif

	if((handle->function_ptr == NULL) || (handle->kill_thread)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d Thread has been killed already \n"), __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}
	handle->kill_thread = 1;
	status = kill_proc((struct task_struct *)handle->thread_id, SIGKILL, 1);
	if (status)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             ("onebox_kill_thread: Unable to Kill Thread %s\n",
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
#elif KERNEL_VERSION_GREATER_THAN_2_6_(27)

	if((handle->function_ptr == NULL) || (handle->kill_thread)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d Thread has been killed already \n"), __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}

	if (!handle->kill_thread) {
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("%s line no %d: thread is not killed yet\n", __func__, __LINE__));
		handle->kill_thread = 1;
		atomic_inc(&handle->thread_done);
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("In %s line no %d Before setting event\n"),__func__, __LINE__));
		rsi_set_event(&handle->thread_event);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
		//handle->thread_id = NULL;
		kthread_stop(handle->thread_id);
#else
		wait_for_completion(&handle->thread_complete);
		handle->thread_id = 0;
#endif
	} else
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("%s line no %d: thread is already killed \n", __func__, __LINE__));
#endif
	return ONEBOX_STATUS_SUCCESS;
}


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

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0))
	daemonize(handle->name);
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
	allow_signal(SIGKILL);
#ifdef CONFIG_PM
	current->flags |= PF_NOFREEZE;
#endif
#else
	daemonize();
	reparent_to_init();
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
	mutex_unlock(&handle->thread_lock);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
	unlock_kernel();
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
void create_kernel_thread(struct work_struct *work)
{  
	onebox_thread_handle_t *handle = container_of(work, onebox_thread_handle_t,
	                                              task_queue);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
	handle->thread_id = kthread_run(onebox_internal_thread, handle, handle->name);
#else
	handle->thread_id = kernel_thread(onebox_internal_thread, handle, 0);
#endif
	return;
}

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
int init_thread(onebox_thread_handle_t *handle, uint8 *name,
                uint32 priority, thread_function function_ptr,
                void *context)
{
	struct onebox_os_intf_operations *os_intf_ops = onebox_get_os_intf_operations_from_origin();

	ONEBOX_ASSERT(function_ptr);
	ONEBOX_ASSERT(handle);
	ONEBOX_ASSERT(name);

	if (!(function_ptr && handle && name))
	    return ONEBOX_STATUS_FAILURE;
	
	memset(handle, 0, sizeof(onebox_thread_handle_t));
	handle->function_ptr = function_ptr;
	handle->context = context;
	handle->kill_thread = 0;
	init_completion(&handle->completion);
	atomic_set(&handle->thread_done,0);
	init_completion(&handle->thread_complete);
	sema_init(&handle->sync_thread, 1);
	down(&handle->sync_thread);

	/* Make sure that the name does not exceed the length */
	strncpy(handle->name, name, ONEBOX_THREAD_NAME_LEN);
	handle->name[ONEBOX_THREAD_NAME_LEN] = '\0';
#if KERNEL_VERSION_GREATER_THAN_2_6_(35)
	mutex_init(&handle->thread_lock);
#endif
	os_intf_ops->onebox_init_event(&handle->thread_event);

	/* Initialize Kernel Start Tasklet */
	init_workq(&handle->task_queue, create_kernel_thread);
	return 0;
}

