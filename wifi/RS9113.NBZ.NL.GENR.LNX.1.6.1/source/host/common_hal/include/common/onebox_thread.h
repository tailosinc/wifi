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

#ifndef __ONEBOX_THREAD_H__
#define __ONEBOX_THREAD_H__

#define ONEBOX_THREAD_NAME_LEN  15
#define EVENT_WAIT_FOREVER     (0x00)

#define onebox_signal_pending()       signal_pending(current) 

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
typedef  struct work_struct         ONEBOX_WORK_QUEUE;
#define  onebox_init_work_queue(a,b,c)  INIT_WORK(a,b)
#define  onebox_schedule_work(a)        schedule_work(a)
#else
typedef  struct tq_struct           ONEBOX_WORK_QUEUE;
#define  onebox_init_work_queue(a,b,c)  INIT_TQUEUE(a,b,c)
#define  onebox_schedule_work(a)        schedule_task(a)
#endif

#define onebox_schedule()        schedule()

typedef void (*thread_function)(void *);

typedef struct _onebox_event 
{  
	atomic_t        eventCondition;         
	wait_queue_head_t   eventQueue;            
}ONEBOX_EVENT, *PONEBOX_EVENT;

typedef struct semaphore ONEBOX_SEMAPHORE,*PONEBOX_SEMAPHORE;

typedef struct _onebox_thread_handle_t 
{
	uint8   name[ONEBOX_THREAD_NAME_LEN + 1]; /* Thread Name */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
	struct task_struct *thread_id;
#else
	int   thread_id; /* OS Specific ThreadId */
#endif
	int   kill_thread; /* Flag to indicate Thread Kill */
	thread_function function_ptr; /* Thread Function */
	void  *context; /* Argument to thread function */
	struct completion completion; /* Thread Compeletion event */
	ONEBOX_WORK_QUEUE task_queue;
	ONEBOX_SEMAPHORE sync_thread;
	struct mutex thread_lock;
	atomic_t thread_done;
	struct completion thread_complete;
	ONEBOX_EVENT thread_event;

}onebox_thread_handle_t;

int rsi_start_thread(onebox_thread_handle_t *handle); 
#endif
