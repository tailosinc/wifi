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

#ifndef __DEP_LINUX_H__
#define __DEP_LINUX_H__

/* #define ATH_DEBUG_SPINLOCKS */ /* announce before spinlocking */
#include <linux/module.h>
#include <linux/wireless.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/random.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/rtnetlink.h>
#include <linux/ip.h>

#define _KERNEL
#define __LINUX__ 1
#define __noinline  
#define KERNEL_VERSION_BTWN_2_6_(a,b) \
 ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,a)) && \
  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,b)))

#define KERNEL_VERSION_EQUALS_2_6_(a) \
 (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,a))

/* Kernel version greater than equals */
#define KERNEL_VERSION_GREATER_THAN_2_6_(a)  \
 (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,a)) 

#define KERNEL_VERSION_LESS_THAN_4_11_(a)  \
 (LINUX_VERSION_CODE < KERNEL_VERSION(4,11,a)) 

#include "onebox_netbuf.h"
#include "hal_var.h"
#include "mbuf.h"
#include "ndiface.h"
#include "sysctl.h"
#include "common.h"
#include "ioctl.h"

#ifdef ONEBOX_CONFIG_CFG80211
#include "cfg80211_ioctl.h"
#endif

struct ieee80211_csa_event 
{
	uint32_t        iev_flags;      /* channel flags */
	uint16_t        iev_freq;       /* setting in Mhz */
	uint8_t         iev_ieee;       /* IEEE channel number */
	uint8_t         iev_mode;       /* CSA mode */
	uint8_t         iev_count;      /* CSA count */
};

struct ieee80211_cac_event 
{
	uint32_t        iev_flags;      /* channel flags */
	uint16_t        iev_freq;       /* setting in Mhz */
	uint8_t         iev_ieee;       /* IEEE channel number */
	/* XXX timestamp? */
	uint8_t         iev_type;       /* IEEE80211_NOTIFY_CAC_* */
};

struct ieee80211_radar_event 
{
	uint32_t        iev_flags;      /* channel flags */
	uint16_t        iev_freq;       /* setting in Mhz */
	uint8_t         iev_ieee;       /* IEEE channel number */
	/* XXX timestamp? */
};

struct ieee80211_auth_event 
{
	uint8_t         iev_addr[6];
};

struct ieee80211_deauth_event 
{ 
	uint8_t         iev_addr[6];
};

struct ieee80211_country_event 
{
	uint8_t         iev_addr[6];
	uint8_t         iev_cc[2];      /* ISO country code */
};

struct ieee80211_radio_event 
{
	uint8_t         iev_state;      /* 1 on, 0 off */
};

void
if_printf(struct net_device *dev, const char *fmt, ...); 
void print_msg(const char *fmt, ...);
#define __FBSDID 
#define printf print_msg

#define _BYTE_ORDER 1
#define _BIG_ENDIAN 1
#define BAND_2_4_GHZ 0
#define BAND_5GHZ    1

#ifndef ALIGNED_POINTER
/*
 * ALIGNED_POINTER is a boolean macro that checks whether an address
 * is valid to fetch data elements of type t from on this architecture.
 * This does not reflect the optimal alignment, just the possibility
 * (within reasonable limits). 
 *
 */
#define ALIGNED_POINTER(p,t) 1
#endif

#define ticks jiffies
#define hz HZ
#define RELEASE_VERSION "trunk"
#define howmany(x, y) (((x)+((y)-1))/(y))
#define A_MAX(a,b) ((a) > (b) ? (a) : (b))

#define __predict_true likely
#if 0
#define KASSERT(exp, msg) do {                  \
       if (unlikely(!(exp))) {                 \
               printk msg;                     \
               BUG();                          \
        }                                       \
    } while (0)
#else
#define KASSERT(exp, msg) do {                  \
	if (unlikely(!(exp))) {                 \
		printk msg;                     \
		printk("In: BUG HERE in %s at %d \n", __func__, __LINE__);       \
	}                                       \
}while (0)
#endif

int copyin(void *a, void *b, size_t c);
int copyout(void *a, void *b, size_t c);

/* Task queue related information */

typedef void (*func)(struct work_struct *);
#define taskqueue workqueue_struct
#define taskqueue_enqueue_fn func

/*
 * Callout operations
 */
#define callout_init(a, b) init_timer(a)
#define callout_drain(a) del_timer_sync(a)
#define callout_pending(a) timer_pending(a)
#define callout_stop(a) del_timer(a)

static inline void callout_reset(struct timer_list *timer, 
                                 unsigned long expires,
                                 void (*fn)(unsigned long), 
                                 void *arg)
{
	if (!timer->function)
	{
		timer->data = (unsigned long)arg;
		timer->function = fn;
		timer->expires =  jiffies + expires;
		add_timer(timer);
	}
	else
	{
		mod_timer(timer, jiffies + expires);
	}
	return;
}

#define callout_schedule(a,b) mod_timer(a,b)
#define callout_init_mtx(a, b, c) init_timer(a)

#define EVENTHANDLER_REGISTER
#define EVENTHANDLER_DEREGISTER

#define CTASSERT(x)  __CTASSERT(x,__LINE__)
#define __CTASSERT(x, y) typedef char __assert ## y[(x) ? 1 : -1]
#define CURVNET_RESTORE  
#define CURVNET_SET    
#define CURVNET_SET_QUIET 

/*
 * mtx locking definitions
 */

#define mtx mutex
#define mtx_init(_as,name,type,opts)  \
	mutex_init((_as))
#define mtx_destroy(_as) \
	mutex_destroy((_as))
#define mtx_lock(_as)  \
	mutex_lock((_as))
#define mtx_unlock(_as) \
	mutex_unlock((_as))
/* mtx_assert- takes the type of the mutex as input (owned/recursed/not recursed)*/
#define mtx_assert(_as,what) \
	mutex_is_locked((_as))
#define mtx_owned(_as) \
	mutex_trylock(_as) /*Check it once -*/

#define IF_LOCK_INIT(ifq) spin_lock_init(&(ifq)->ifq_lock)
#define IF_LOCK(ifq) \
do { unsigned long __if_flags;\
	spin_lock_irqsave(&(ifq)->ifq_lock, __if_flags);
#define IF_UNLOCK(ifq) \
	spin_unlock_irqrestore(&(ifq)->ifq_lock,__if_flags); \
}while(0);

#define IF_UNLOCK_EARLY(ifq) \
	spin_unlock_irqrestore(&(ifq)->ifq_lock,__if_flags);

#define IF_LOCK_ASSERT(ifq) \
	KASSERT(spin_is_locked(&(ifq)->ifq_lock), ("IFQ not locked!"))

#define IFQ_LOCK IF_LOCK
#define IFQ_UNLOCK IF_UNLOCK
#define IFQ_UNLOCK_EARLY IF_UNLOCK_EARLY

/*
 * ACL locking definitions
 */
/* WHY MULTIPLE DECLARATIONS */
#define ACL_LOCK_INIT(_as, _name) spin_lock_init(&(_as)->as_lock)
#define ACL_LOCK_DESTROY(_as)
#define ACL_LOCK(_as) \
do { \
    unsigned long __acl_lockflags; \
    spin_lock_irqsave(&(_as)->as_lock, __acl_lockflags);
#define ACL_UNLOCK(_as) \
    spin_unlock_irqrestore((&(_as)->as_lock), __acl_lockflags); \
   } while(0);
#define ACL_UNLOCK_EARLY(_as) \
    spin_unlock_irqrestore((&(_as)->as_lock), __acl_lockflags);

#define ACL_LOCK_ASSERT(_as) \
	KASSERT(spin_is_locked(&(_as)->as_lock), ("ACL not locked!"))


/*
 * A "policy module" is an adjunct module to net80211 that provides
 * functionality that typically includes policy decisions.  This
 * modularity enables extensibility and vendor-supplied functionality.
 */
#define _IEEE80211_POLICY_MODULE(policy, name, version) \
typedef void (*policy##_setup)(int); \
SET_DECLARE(policy##_set, policy##_setup); \
static int \
wlan_##name##_modevent(module_t mod, int type, void *unused) \
{ \
	policy##_setup * const *iter, f; \
	switch (type) { \
	case MOD_LOAD: \
		SET_FOREACH(iter, policy##_set) { \
			f = (void*) *iter; \
			f(type); \
		} \
		return 0; \
	case MOD_UNLOAD: \
	case MOD_QUIESCE: \
		if (nrefs) { \
			printf("wlan_##name: still in use (%u dynamic refs)\n",\
				nrefs); \
			return EBUSY; \
		} \
		if (type == MOD_UNLOAD) { \
			SET_FOREACH(iter, policy##_set) { \
				f = (void*) *iter; \
				f(type); \
			} \
		} \
		return 0; \
	} \
	return EINVAL; \
} \
static moduledata_t name##_mod = { \
	"wlan_" #name, \
	wlan_##name##_modevent, \
	0 \
}; \
DECLARE_MODULE(wlan_##name, name##_mod, SI_SUB_DRIVERS, SI_ORDER_FIRST);\
MODULE_VERSION(wlan_##name, version); \
MODULE_DEPEND(wlan_##name, wlan, 1, 1, 1)

/*
 * Crypto modules implement cipher support.
 */
#define	IEEE80211_CRYPTO_MODULE(name, version) \
static int \
name##_module_init(void) \
{ \
	ieee80211_crypto_register(&name); \
	return 0; \
} \
static void name##_module_exit(void) \
{ \
	ieee80211_crypto_unregister(&name); \
} \
MODULE_LICENSE("Dual BSD/GPL");\
module_init(name##_module_init); \
module_exit(name##_module_exit); \
MODULE_VERSION("version##");

/*
 * Rate control modules provide tx rate control support.
 */
#define IEEE80211_RATECTL_MODULE(alg, version) \
	_IEEE80211_POLICY_MODULE(ratectl, alg, version); \

#define IEEE80211_RATECTL_ALG(name, alg, v) \
static void \
alg##_modevent(int type) \
{ \
	if (type == MOD_LOAD) \
		ieee80211_ratectl_register(alg, &v); \
	else \
		ieee80211_ratectl_unregister(alg); \
} \
TEXT_SET(ratectl##_set, alg##_modevent)

#define IEEE80211_NODE_ITERATE_LOCK_INIT(_nt, _name) do { \
	spin_lock_init(&(_nt)->nt_scanlock); \
} while (0)
#define IEEE80211_NODE_ITERATE_LOCK_OBJ(_nt) (&(_nt)->nt_scanlock)
#define IEEE80211_NODE_ITERATE_LOCK_DESTROY(_nt) 
#define IEEE80211_NODE_ITERATE_LOCK(_nt) \
do { \
	unsigned long __scan_lockflags; \
	spin_lock_irqsave((IEEE80211_NODE_ITERATE_LOCK_OBJ(_nt)), __scan_lockflags);
#define IEEE80211_NODE_ITERATE_UNLOCK(_nt) \
	spin_unlock_irqrestore((IEEE80211_NODE_ITERATE_LOCK_OBJ(_nt)), __scan_lockflags); \
} while(0);
/*
 * Power-save queue definitions. 
 */
#define IEEE80211_PSQ_INIT(_psq, _name) spin_lock_init(&(_psq)->psq_lock)
#define IEEE80211_PSQ_DESTROY(_psq) 
#define IEEE80211_PSQ_LOCK(_psq) \
do { \
unsigned long __psq_lockflags; \
spin_lock_irqsave((&(_psq)->psq_lock), __psq_lockflags);
#define IEEE80211_PSQ_UNLOCK(_psq) \
	spin_unlock_irqrestore((&(_psq)->psq_lock), __psq_lockflags); \
} while(0);
#define IEEE80211_PSQ_UNLOCK_EARLY(_psq) \
    spin_unlock_irqrestore((&(_psq)->psq_lock), __psq_lockflags);

/*
 * Scan table definitions.
 */
#define IEEE80211_SCAN_TABLE_LOCK_INIT(_st, _name) \
	spin_lock_init(&(_st)->st_lock)
#define IEEE80211_SCAN_TABLE_LOCK_DESTROY(_st)
#define IEEE80211_SCAN_TABLE_LOCK(_st) \
do { \
	unsigned long __stlockflags; \
	spin_lock_irqsave((&(_st)->st_lock), __stlockflags);
#define IEEE80211_SCAN_TABLE_UNLOCK(_st) \
	spin_unlock_irqrestore((&(_st)->st_lock), __stlockflags); \
} while(0);
#define IEEE80211_SCAN_TABLE_UNLOCK_EARLY(_st) \
	spin_unlock_irqrestore((&(_st)->st_lock), __stlockflags);

#define STAILQ_ENTRY(type)                                              \
struct {                                                                \
	struct type *stqe_next; /* next element */                      \
}

struct ieee80211_cb 
{
	u_int8_t vlan[8];           /* reserve for vlan tag info */
	struct ieee80211_node *ni;
	u_int32_t flags;
#define M_LINK0     0x01            /* frame needs WEP encryption */
#define M_FF        0x02            /* fast frame */
#define M_PWR_SAV   0x04            /* bypass power save handling */
#define M_UAPSD     0x08            /* frame flagged for u-apsd handling */
#define M_RAW           0x10
#ifdef IEEE80211_DEBUG_REFCNT
	int tracked;
	void   (*next_destructor)(struct sk_buff *skb);
#endif
	struct sk_buff *next;           /* fast frame sk_buf chain */
	u_int8_t auth_pkt;
};

#define ieee80211_node_initref(_ni)  atomic_set(&(_ni)->ni_refcnt, 1)

#define ieee80211_node_incref(_ni) atomic_inc(&(_ni)->ni_refcnt)
#define ieee80211_node_decref(_ni) atomic_dec(&(_ni)->ni_refcnt)
//#define ieee80211_node_dectestref(_ni) atomic_dec_and_test(&(_ni)->ni_refcnt)
#define ieee80211_node_refcnt(_ni)      atomic_read(&(_ni)->ni_refcnt)

#define __offsetof offsetof
#define arc4random() 4 
//get_random_int
#define getmicrouptime do_gettimeofday

#define MODULE_DEPEND(module, mdepend, vmin, vpref, vmax)       \
static struct mod_depend _##module##_depend_on_##mdepend = {    \
	vmin,                           \
	vpref,                          \
	vmax                            \
};                              \
MODULE_METADATA(_md_##module##_on_##mdepend, MDT_DEPEND,    \
    &_##module##_depend_on_##mdepend, #mdepend)

#ifndef MODULE_VERSION
#define MODULE_VERSION(module,version) MODULE_VERSION(version)
#endif

/* NEEDS TO BE REVIEWED */ 
#define mtx_destory(_as) mutex_destroy((_as))
#define mtx_lock(_as)  mutex_lock((_as))
#define mtx_unlock(_as) mutex_unlock((_as))
/* mtx_assert- takes the type of the mutex as input (owned/recursed/not recursed)*/
#define mtx_assert(_as,what) mutex_is_locked((_as))


#define M_NOWAIT 0x0001     /* do not block */
#define M_DONTWAIT M_NOWAIT
#define M_WAITOK 0x0002      /* ok to block */
#define M_ZERO 0x0100      /* bzero the allocation */

typedef struct 
{
	char name[16];   /* e.g. "ath0_com_lock" */
	spinlock_t lmtx;
	int16_t lock_count;
	struct task_struct *task;
}ieee80211_com_lock_t;

#define IEEE80211_LOCK_INIT(_ic, _name) do { \
	ieee80211_com_lock_t *cl = &(_ic)->ic_comlock; \
	snprintf(cl->name, sizeof(cl->name), "%s_com_lock",_name); \
	spin_lock_init(&cl->lmtx); \
	cl->lock_count = 0; \
} while (0)

#define IEEE80211_LOCK_OBJ(_ic) (&(_ic)->ic_comlock.lmtx)
#define IEEE80211_LOCK(_ic) \
	if (((_ic)->ic_comlock.task) != current) \
	{\
		spin_lock_irqsave((IEEE80211_LOCK_OBJ(_ic)), _ic->ilock_flags); \
		(_ic)->ic_comlock.task = current;\
	}\
	((_ic)->ic_comlock.lock_count)++;\
	if(!spin_is_locked(IEEE80211_LOCK_OBJ(_ic)) || (((_ic)->ic_comlock.task) != current))\
	{\
	}

#define IEEE80211_UNLOCK(_ic) \
	if ((_ic)->ic_comlock.task == current) \
	{\
		if(--((_ic)->ic_comlock.lock_count) == 0) \
		{\
			(_ic)->ic_comlock.task = NULL; \
			spin_unlock_irqrestore(IEEE80211_LOCK_OBJ(_ic) ,_ic->ilock_flags); \
		}\
	} \
	else\
	{\
		printk("In: %s %d ERROR ERROR This case should Never occur\n", __func__, __LINE__);\
	}\
	if(((_ic)->ic_comlock.lock_count) < 0) \
	{\
	}

#define IEEE80211_LOCK_DESTROY(_ic)   

#if (defined(CONFIG_SMP) || defined(CONFIG_DEBUG_SPINLOCK)) && defined(spin_is_locked)
#define IEEE80211_LOCK_ASSERT(_ic) \
	KASSERT(spin_is_locked(&(_ic)->ic_comlock.lmtx), ("ieee80211com not locked!")); 
#if (defined(ATH_DEBUG_SPINLOCKS))
#define IEEE80211_LOCK_CHECK(_ic) do { \
	if (spin_is_locked(&(_ic)->ic_comlock.lmtx)) \
		printk("%s:%d - about to block on ieee80211com lock!\n", __func__, __LINE__); \
} while(0)
#else /* #if (defined(ATH_DEBUG_SPINLOCKS)) */
#define IEEE80211_LOCK_CHECK(_ic)
#endif 
#else
#define IEEE80211_LOCK_ASSERT(_ic)
#define IEEE80211_LOCK_CHECK(_ic)
#endif


#define IEEE80211_NODE_LOCK_INIT(_nt, _name) \
do {\
	ieee80211_node_lock_t *nl = &(_nt)->nt_nodelock;    \
	spin_lock_init(&nl->lock); \
	nl->task = NULL; \
	nl->lock_count = 0; \
} while(0)

#define IEEE80211_NODE_LOCK_OBJ(_nt)  (&(_nt)->nt_nodelock.lock)
#define IEEE80211_NODE_LOCK_DESTROY(_nt) 
#define IEEE80211_NODE_LOCKING_CODE(_nt) \
	{\
		if (((_nt)->nt_nodelock.task) != current) \
		{\
			spin_lock_irqsave((IEEE80211_NODE_LOCK_OBJ(_nt)), __node_lockflags); \
			(_nt)->nt_nodelock.task = current;\
		}\
		((_nt)->nt_nodelock.lock_count)++;\
		if(!spin_is_locked(IEEE80211_NODE_LOCK_OBJ(_nt)) || (((_nt)->nt_nodelock.task) != current))\
		{\
		}\
	}
#define IEEE80211_NODE_UNLOCKING_CODE(_nt) \
	{\
		if ((_nt)->nt_nodelock.task == current) \
		{\
			if(--((_nt)->nt_nodelock.lock_count) == 0) \
			{\
				(_nt)->nt_nodelock.task = NULL; \
				spin_unlock_irqrestore(IEEE80211_NODE_LOCK_OBJ(_nt) ,__node_lockflags); \
			}\
		} \
		else\
		{\
			printk("In: %s %d ERROR ERROR This case should Never occur\n", __func__, __LINE__);\
		}\
		if(((_nt)->nt_nodelock.lock_count) < 0) \
		{\
		}\
	}

#define IEEE80211_NODE_LOCK(_nt) \
do {	\
    unsigned long __node_lockflags = 0; \
	IEEE80211_NODE_LOCKING_CODE((_nt))
	
#define IEEE80211_NODE_IS_LOCKED(_nt) \
	spin_is_locked(IEEE80211_NODE_LOCK_OBJ(_nt))

#define IEEE80211_NODE_UNLOCK(_nt) \
	IEEE80211_NODE_UNLOCKING_CODE((_nt)) \
  } while(0);

#define IEEE80211_NODE_UNLOCK_EARLY(_nt)  IEEE80211_NODE_UNLOCKING_CODE((_nt))

//#define IEEE80211_NODE_CONDITIONAL_LOCK(_nt) IEEE80211_NODE_LOCKING_CODE((_nt))
//#define IEEE80211_NODE_CONDITIONAL_UNLOCK(_nt) IEEE80211_NODE_UNLOCKING_CODE((_nt))


#if (defined(CONFIG_SMP) || defined(CONFIG_DEBUG_SPINLOCK)) && defined(spin_is_locked)
#define IEEE80211_NODE_LOCK_ASSERT(_ni) \
	KASSERT(spin_is_locked(&(_ni)->nt_nodelock.lock), \
		("802.11 node not locked!"))
#if (defined(ATH_DEBUG_SPINLOCKS))
#define IEEE80211_NODE_LOCK_CHECK(_ni) do { \
	if (spin_is_locked(&(_ni)->nt_nodelock)) \
		printk("%s:%d - about to block on node lock!\n", __func__, __LINE__); \
} while(0)
#else /* #if (defined(ATH_DEBUG_SPINLOCKS)) */
#define IEEE80211_NODE_LOCK_CHECK(_ni)
#endif /* #if (defined(ATH_DEBUG_SPINLOCKS)) */
#else
#define IEEE80211_NODE_LOCK_ASSERT(_ni)
#define IEEE80211_NODE_LOCK_CHECK(_ni)
#endif


#define IEEE80211_AGEQ_INIT(_aq, _name) \
	spin_lock_init(&(_aq)->aq_lock);
#define IEEE80211_AGEQ_DESTROY(_aq)
#define IEEE80211_AGEQ_LOCK(_aq) \
do { \
    unsigned long __ageq_lockflags; \
    spin_lock_irqsave(&(_aq)->aq_lock, __ageq_lockflags);
#define IEEE80211_AGEQ_UNLOCK(_aq) \
    spin_unlock_irqrestore(&(_aq)->aq_lock, __ageq_lockflags); \
   } while(0);

#define IEEE80211_AGEQ_LOCK_AT_START(_aq) \
    unsigned long __ageq_lockflags; \
    spin_lock_irqsave(&(_aq)->aq_lock, __ageq_lockflags);
#define IEEE80211_AGEQ_UNLOCK_EARLY(_aq) \
    spin_unlock_irqrestore(&(_aq)->aq_lock, __ageq_lockflags);

/*
 * A "policy module" is an adjunct module to net80211 that provides
 * functionality that typically includes policy decisions.  This
 * modularity enables extensibility and vendor-supplied functionality.
 */
#define _IEEE80211_POLICY_MODULE(policy, name, version) \
typedef void (*policy##_setup)(int); \
SET_DECLARE(policy##_set, policy##_setup); \
static int \
wlan_##name##_modevent(module_t mod, int type, void *unused) \
{ \
	policy##_setup * const *iter, f; \
	switch (type) { \
	case MOD_LOAD: \
		SET_FOREACH(iter, policy##_set) { \
			f = (void*) *iter; \
			f(type); \
		} \
		return 0; \
	case MOD_UNLOAD: \
	case MOD_QUIESCE: \
		if (nrefs) { \
			printf("wlan_##name: still in use (%u dynamic refs)\n",\
				nrefs); \
			return EBUSY; \
		} \
		if (type == MOD_UNLOAD) { \
			SET_FOREACH(iter, policy##_set) { \
				f = (void*) *iter; \
				f(type); \
			} \
		} \
		return 0; \
	} \
	return EINVAL; \
} \
static moduledata_t name##_mod = { \
	"wlan_" #name, \
	wlan_##name##_modevent, \
	0 \
}; \
DECLARE_MODULE(wlan_##name, name##_mod, SI_SUB_DRIVERS, SI_ORDER_FIRST);\
MODULE_VERSION(wlan_##name, version); \
MODULE_DEPEND(wlan_##name, wlan, 1, 1, 1)


/*
 * Rate control modules provide tx rate control support.
 */
#define IEEE80211_RATECTL_MODULE(alg, version) \
	_IEEE80211_POLICY_MODULE(ratectl, alg, version); \

#define IEEE80211_RATECTL_ALG(name, alg, v) \
static void \
alg##_modevent(int type) \
{ \
	if (type == MOD_LOAD) \
		ieee80211_ratectl_register(alg, &v); \
	else \
		ieee80211_ratectl_unregister(alg); \
} \
TEXT_SET(ratectl##_set, alg##_modevent)
#define RIJNDAEL_MAXNR  14
typedef struct 
{
	int     decrypt;
	int     Nr;             /* key-length-dependent number of rounds */
	uint32_t ek[4 * (RIJNDAEL_MAXNR + 1)];  /* encrypt key schedule */
	uint32_t dk[4 * (RIJNDAEL_MAXNR + 1)];  /* decrypt key schedule */
}rijndael_ctx;

void  rijndael_decrypt(const rijndael_ctx *, const u_char *, u_char *); 
void  rijndael_encrypt(const rijndael_ctx *, const u_char *, u_char *);
#ifndef CONFIG_11W 
//#define rijndael_encrypt(arg1,arg2,arg3)   null_function()
#endif
/* rijndael_encrypt */ 
/* rijndael_set_key */ 

/* ticks_to_msecs */
#define msecs_to_ticks(ms) msecs_to_jiffies(ms)
#define ticks_to_msecs(t) jiffies_to_msecs(t)
#define ticks_to_secs(t) (ticks_to_msecs(t) / 1000)
#define time_before(a,b) time_after(b,a)
#define time_before_eq(a,b) time_after_eq(b,a)

#define STAILQ_NEXT(elm, field) ((elm)->field.stqe_next)

#define STAILQ_LAST(head, type, field) \
	(STAILQ_EMPTY((head)) ? \
	NULL : \
	((struct type *)(void *)                \
	((char *)((head)->stqh_last) - __offsetof(struct type, field))))

#define STAILQ_INSERT_TAIL(head, elm, field) do {           \
    STAILQ_NEXT((elm), field) = NULL;               \
    *(head)->stqh_last = (elm);                 \
    (head)->stqh_last = &STAILQ_NEXT((elm), field);         \
} while (0)

/*
 * Node locking definitions.
 */
typedef spinlock_t ieee80211_scan_table_lock_t;

typedef struct spinlock_dup
{
	spinlock_t lock;
	int16_t lock_count;
	struct task_struct *task;
}ieee80211_node_lock_t;
/*
 * Node locking definitions.
 */

/*
 * Node table iteration locking definitions; this protects the
 * scan generation # used to iterate over the station table
 * while grabbing+releasing the node lock.
 */
typedef spinlock_t ieee80211_scan_lock_t;
/*
 * Power-save queue definitions. 
 */
typedef spinlock_t ieee80211_psq_lock_t;
/*
 * Age queue definitions.
 */
typedef spinlock_t ieee80211_ageq_lock_t;
/*
 * ACL locking definitions
 */
typedef spinlock_t acl_lock_t;

/* va_end */ 
/* va_start */
#if 0
#define RIJNDAEL_MAXNR  14

/* mtx_owned */  
typedef struct 
{
	int     decrypt;
	int     Nr;             /* key-length-dependent number of rounds */
	uint32_t ek[4 * (RIJNDAEL_MAXNR + 1)];  /* encrypt key schedule */
	uint32_t dk[4 * (RIJNDAEL_MAXNR + 1)];  /* decrypt key schedule */
}rijndael_ctx;
#endif
static __inline void *
ieee80211_malloc(size_t size, int flags)
{
	void *p = kmalloc(size, flags & M_NOWAIT ? GFP_ATOMIC : GFP_KERNEL);
	if (p && (flags & M_ZERO))
	{
		memset(p, 0, size);
	}
	return p;
}
/*
 * Linux has no equivalents to malloc types so null these out.
 */
#define MALLOC_DEFINE(type, shortdesc, longdesc)
#define MALLOC_DECLARE(type)
#define callout timer_list
#define malloc(_size, _type, _flags) \
        ieee80211_malloc(_size, _flags)
#define free(addr, type) kfree((addr))

/* bpf_peers_present */
void null_function(void);
void rijndaelEncrypt(const u32 rk[/*4*(Nr + 1)*/], int Nr, const u8 pt[16], u8 ct[16]);
int rijndaelKeySetupEnc(u32 rk[/*4*(Nr + 1)*/], const u8 cipherKey[], int keyBits);
int rijndaelKeySetupDec(u32 rk[/*4*(Nr + 1)*/], const u8 cipherKey[], int keyBits); 
void rijndael_set_key(rijndael_ctx *ctx, const u_char *key, int bits);
struct taskqueue *
taskqueue_create(const char *name, int mflags,
                 taskqueue_enqueue_fn enqueue, void *context);
void    
taskqueue_drain(struct taskqueue *queue, struct work_struct *task);

void taskqueue_enqueue(struct taskqueue *ic_tq, struct work_struct *task);

void taskqueue_unblock(struct taskqueue *ic_tq);
void taskqueue_block(struct taskqueue *ic_tq);
int taskqueue_start_threads(struct taskqueue** ic_tq, int flag1, int flag2,
                            char*str, char* if_xname);

void taskqueue_free(struct taskqueue* ic_tq);

void taskqueue_thread_enqueue(struct work_struct *work);
/* tx path usage */
#define M_ENCAP         M_PROTO1                /* 802.11 encap done */
#define M_PROTO8        0x00200000 /* protocol-specific */
#define M_AMPDU_MPDU    M_PROTO8                /* ok for A-MPDU aggregation */
#define M_PROTO1        0x00000010 /* protocol-specific */
#define M_PROTO2        0x00000020 /* protocol-specific */
#define M_AMPDU         M_PROTO1                /* A-MPDU subframe */
#define M_WEP           M_PROTO2                /* WEP done by hardware */
#define ETHERTYPE_PAE           0x888e  /* EAPOL PAE/802.1x */
#define M_BCAST         0x00000200 /* send/received as link-level broadcast */
#define M_DECAP        0x80000000 /* protocol-specific */

#define le16toh(_x) le16_to_cpu(_x)
#define htole16(_x) cpu_to_le16(_x)
#define le32toh(_x) le32_to_cpu(_x)
#define htole32(_x) cpu_to_le32(_x)
#define be32toh(_x) be32_to_cpu(_x)
#define htobe32(_x) cpu_to_be32(_x)
#define le64toh(_x) le64_to_cpu(_x)
#define htole64(_x) cpu_to_le64(_x)

/*
 * Structure of a 10Mb/s Ethernet header.
 */
#define ETHER_ADDR_LEN 6
struct ether_header 
{
        unsigned char  ether_dhost[ETHER_ADDR_LEN];
        unsigned char  ether_shost[ETHER_ADDR_LEN];
        unsigned short ether_type;
} __packed;
/*
 * This unlikely to be popular but it dramatically reduces diffs.
 */
#define IEEE80211_SCANNER_ALG(a,b,c)
#define IEEE80211_SCANNER_MODULE(name, version)                         \
static const struct ieee80211_scanner mesh_default; \
static const struct ieee80211_scanner sta_default; \
static const struct ieee80211_scanner adhoc_default; \
static const struct ieee80211_scanner ap_default; \
static int \
name##_module_init(void) \
{ \
	ieee80211_scanner_register(IEEE80211_M_STA, &sta_default); \
	ieee80211_scanner_register(IEEE80211_M_IBSS, &mesh_default); \
	ieee80211_scanner_register(IEEE80211_M_IBSS, &adhoc_default); \
	ieee80211_scanner_register(IEEE80211_M_AHDEMO, &adhoc_default); \
	ieee80211_scanner_register(IEEE80211_M_HOSTAP, &ap_default); \
	return 0; \
} \
static void name##_module_exit(void) \
{ \
	ieee80211_scanner_unregister(IEEE80211_M_STA, &sta_default); \
	ieee80211_scanner_unregister(IEEE80211_M_IBSS, &mesh_default); \
	ieee80211_scanner_unregister(IEEE80211_M_IBSS, &adhoc_default); \
	ieee80211_scanner_unregister(IEEE80211_M_AHDEMO, &adhoc_default); \
	ieee80211_scanner_unregister(IEEE80211_M_HOSTAP, &ap_default); \
} \
MODULE_LICENSE("Dual BSD/GPL");\
module_init(name##_module_init); \
module_exit(name##_module_exit); \
MODULE_VERSION("version##");

#define IEEE80211_ACL_MODULE(name, alg, version) \
static int \
name##_module_init(void) \
{ \
	ieee80211_aclator_register(&alg); \
	return 0; \
} \
static void name##_module_exit(void) \
{ \
	ieee80211_aclator_unregister(&alg); \
} \
MODULE_LICENSE("Dual BSD/GPL");\
module_init(name##_module_init); \
module_exit(name##_module_exit); \
MODULE_VERSION("version##");

#define IEEE80211_AUTH_MODULE(name, version) \
static int \
name##_module_init(void) \
{ \
	ieee80211_authenticator_register(IEEE80211_AUTH_8021X, &xauth); \
	ieee80211_authenticator_register(IEEE80211_AUTH_WPA, &xauth); \
	return 0; \
} \
static void name##_module_exit(void) \
{ \
	ieee80211_authenticator_unregister(IEEE80211_AUTH_8021X); \
	ieee80211_authenticator_unregister(IEEE80211_AUTH_WPA); \
} \
MODULE_LICENSE("Dual BSD/GPL");\
module_init(name##_module_init); \
module_exit(name##_module_exit); \
MODULE_VERSION("version##");

#define IEEE80211_AUTH_ALG(a,b,c) 
/*
 * Convert Ethernet address to printable (loggable) representation.
 * This routine is for compatibility; it's better to just use
 *
 *      printf("%6D", <pointer to address>, ":");
 *
 * since there's no static buffer involved.
 */
static __inline char *
#ifdef CONFIG_11W
ether_sprintf(const uint8_t *ap) 
#else
ether_sprintf(const uint8_t *ap, char *etherbuf) 
#endif
{
#ifdef CONFIG_11W
	static char etherbuf[18];
#endif
	snprintf(etherbuf, ETHER_ADDR_LEN, "%d:%d:%d:%d:%d:%d", 
	ap[0], ap[1], ap[2], ap[3], ap[4], ap[5]);
	return (etherbuf);
}

#define M_WME_GETAC(m)  ((m)->m_pkthdr.ether_vtag)
#define M_WME_GETTID(m)  ((m)->tid)
#define M_MCAST         0x00000400 /* send/received as link-level multicast */
#define llc_control             llc_un.type_u.control
#define llc_control_ext         llc_un.type_raw.control_ext
#define llc_fid                 llc_un.type_u.format_id
#define llc_class               llc_un.type_u.class
#define llc_window              llc_un.type_u.window_x2
#define llc_frmrinfo            llc_un.type_frmr.frmr_rej_pdu0
#define llc_frmr_pdu0           llc_un.type_frmr.frmr_rej_pdu0
#define llc_frmr_pdu1           llc_un.type_frmr.frmr_rej_pdu1
#define llc_frmr_control        llc_un.type_frmr.frmr_control
#define llc_frmr_control_ext    llc_un.type_frmr.frmr_control_ext
#define llc_frmr_cause          llc_un.type_frmr.frmr_cause
#define llc_snap                llc_un.type_snap

struct llc 
{
	uint8_t llc_dsap;
	uint8_t llc_ssap;
	union 
	{
		struct 
		{
			uint8_t control;
			uint8_t format_id;
			uint8_t class;
			uint8_t window_x2;
		} __packed type_u;
		struct 
		{
			uint8_t num_snd_x2;
			uint8_t num_rcv_x2;
		} __packed type_i;
		struct 
		{
			uint8_t control;
			uint8_t num_rcv_x2;
		} __packed type_s;
		struct 
		{
			uint8_t control;
		/*
		 * We cannot put the following fields in a structure because
		 * the structure rounding might cause padding.
		 */
			uint8_t frmr_rej_pdu0;
			uint8_t frmr_rej_pdu1;
			uint8_t frmr_control;
			uint8_t frmr_control_ext;
			uint8_t frmr_cause;
		} __packed type_frmr;
		struct 
		{
			uint8_t  control;
			uint8_t  org_code[3];
			uint16_t ether_type;
		} __packed type_snap;
		struct 
		{
			uint8_t control;
			uint8_t control_ext;
		} __packed type_raw;
	} __packed llc_un;
} __packed;

struct route
{
	int reserved[0];
};

struct ieee80211_bpf_params 
{
	uint8_t         ibp_vers;       /* version */
#define IEEE80211_BPF_VERSION   0
	uint8_t         ibp_len;        /* header length in bytes */
	uint8_t         ibp_flags;
#define IEEE80211_BPF_SHORTPRE  0x01    /* tx with short preamble */
#define IEEE80211_BPF_NOACK     0x02    /* tx with no ack */
#define IEEE80211_BPF_CRYPTO    0x04    /* tx with h/w encryption */
#define IEEE80211_BPF_FCS       0x10    /* frame incldues FCS */
#define IEEE80211_BPF_DATAPAD   0x20    /* frame includes data padding */
#define IEEE80211_BPF_RTS       0x40    /* tx with RTS/CTS */
#define IEEE80211_BPF_CTS       0x80    /* tx with CTS only */
	uint8_t         ibp_pri;        /* WME/WMM AC+tx antenna */
	uint8_t         ibp_try0;       /* series 1 try count */
	uint8_t         ibp_rate0;      /* series 1 IEEE tx rate */
	uint8_t         ibp_power;      /* tx power (device units) */
	uint8_t         ibp_ctsrate;    /* IEEE tx rate for CTS */
	uint8_t         ibp_try1;       /* series 2 try count */
	uint8_t         ibp_rate1;      /* series 2 IEEE tx rate */
	uint8_t         ibp_try2;       /* series 3 try count */
	uint8_t         ibp_rate2;      /* series 3 IEEE tx rate */
	uint8_t         ibp_try3;       /* series 4 try count */
	uint8_t         ibp_rate3;      /* series 4 IEEE tx rate */
};
struct mbuf *
ieee80211_getmgtframe(uint8_t **frm, int headroom, int pktlen);
uint32_t uptime(void);
#define AF_IEEE80211    37              /* IEEE 802.11 protocol */

#define M_PROTO7        0x00100000 /* protocol-specific */
#define M_FRAG          0x00000800 /* packet is a fragment of a larger packet */
#define time_uptime     uptime()
#define M_PROTO3        0x00000040 /* protocol-specific */
#define M_EAPOL         M_PROTO3                /* PAE/EAPOL frame */
#define M_FIRSTFRAG     0x00001000 /* packet is first fragment */
#define M_LASTFRAG      0x00002000 /* packet is last fragment */
#define M_EAPOL         M_PROTO3                /* PAE/EAPOL frame */
#define M_TXCB          M_PROTO7                /* do tx complete callback */
#define M_80211_TX \
        (M_FRAG|M_FIRSTFRAG|M_LASTFRAG|M_ENCAP|M_EAPOL|M_PWR_SAV|\
         M_MORE_DATA|M_FF|M_TXCB|M_AMPDU_MPDU)
#define cv  __wait_queue_head 
void cv_init(wait_queue_head_t *a, char *b, int *flags);
void
ieee80211_flush_ifq(struct ifqueue *ifq, struct ieee80211vap *vap);

#define timevalcmp(tvp, uvp, cmp)                                       \
        (((tvp)->tv_sec == (uvp)->tv_sec) ?                             \
            ((tvp)->tv_usec cmp (uvp)->tv_usec) :                       \
            ((tvp)->tv_sec cmp (uvp)->tv_sec))

#define GETU32(pt) (((u32)(pt)[0] << 24) ^ ((u32)(pt)[1] << 16) ^ ((u32)(pt)[2] <<  8) ^ ((u32)(pt)[3]))
#define PUTU32(ct, st) { (ct)[0] = (u8)((st) >> 24); (ct)[1] = (u8)((st) >> 16); (ct)[2] = (u8)((st) >>  8); (ct)[3] = (u8)(st); }
#define STAILQ_INIT(head) do { \
	STAILQ_FIRST((head)) = NULL; \
	(head)->stqh_last = &STAILQ_FIRST((head)); \
} while (0)

#define STAILQ_FIRST(head) ((head)->stqh_first)
typedef __const char *  c_caddr_t;      /* core address, pointer to const */
int ppsratecheck(struct timeval *lasttime, int *curpps, int maxpps);
int ratecheck(struct timeval *lasttime, const struct timeval *mininterval);
void
ieee80211_load_module(const char *modname);
void
ieee80211_vap_destroy(struct ieee80211vap *vap);
void
ieee80211_sysctl_vdetach(struct ieee80211vap *vap);
#define IEEE80211_IOCTL_SET(a,b)
#define IEEE80211_IOCTL_GET(a,b)
#define TASK_INIT(a,b,c,d) INIT_WORK(a, c)

#define IF_ADDR_LOCK_INIT(if)   mtx_init(&(if)->if_addr_mtx,            \
                                    "if_addr_mtx", NULL, MTX_DEF)
#define IF_ADDR_LOCK_DESTROY(if)        mtx_destroy(&(if)->if_addr_mtx)
#define IF_ADDR_LOCK(if)        mtx_lock(&(if)->if_addr_mtx)
#define IF_ADDR_UNLOCK(if)      mtx_unlock(&(if)->if_addr_mtx)
#define IF_ADDR_LOCK_ASSERT(if) mtx_assert(&(if)->if_addr_mtx, MA_OWNED)
struct ieee80211req;
int ieee80211_ioctl_set80211(struct ieee80211vap *vap, u_long cmd, struct ieee80211req *ireq);
int ieee80211_ioctl_get80211(struct ieee80211vap *vap, u_long cmd, struct ieee80211req *ireq);
#define priv_check(a,b) 0
#define curthread
#undef min
#define min(a,b) (((a)<(b))?(a):(b))
typedef int ieee80211_ioctl_getfunc(struct ieee80211vap *, struct ieee80211req *);
typedef int ieee80211_ioctl_setfunc(struct ieee80211vap *, struct ieee80211req *);
void cv_init(wait_queue_head_t *a, char *b, int *flags);
void cv_signal(wait_queue_head_t *a, int *flags);
void cv_wait(wait_queue_head_t *a,struct ieee80211com *b, int *flags);
void cv_timedwait(wait_queue_head_t *a,struct ieee80211com *b,int c, int *flags);
#define AF_LINK AF_NETLINK
#define if_addmulti(p, a,b) null_function()
int ieee80211_node_dectestref(struct ieee80211_node *ni);
void set_fixedrate(struct ieee80211vap *vap, int ucastrate);
int ishtrate(struct ieee80211com *ic, int ucastrate);
void rt_ieee80211msg(struct ifnet *ifp, int what, void *data, uint32_t data_len);
int check_band(int mode);
int checkmcs(int mcs);
struct ieee80211_rateset;
int checkrate(const struct ieee80211_rateset *rs, int rate);
#endif /* _NET80211_IEEE80211_LINUX_H_ */
