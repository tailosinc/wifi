/*-
 * Copyright (c) 2009 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef __FREEBSD__
#include <sys/cdefs.h>
#ifdef __FreeBSD__
__FBSDID("$FreeBSD: src/sys/net80211/ieee80211_action.c,v 1.6 2012/03/04 05:49:39 adrian Exp $");
#endif

/*
 * IEEE 802.11 send/recv action frame support.
 */

#include "opt_inet.h"
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h> 
 
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/ethernet.h>
#endif

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_action.h>
#include <net80211/ieee80211_mesh.h>

static int
send_inval(struct ieee80211_node *ni, int cat, int act, void *sa)
{
	return EINVAL;
}

static ieee80211_send_action_func *ba_send_action[8] = {
	send_inval, send_inval, send_inval, send_inval,
	send_inval, send_inval, send_inval, send_inval,
};
static ieee80211_send_action_func *ht_send_action[8] = {
	send_inval, send_inval, send_inval, send_inval,
	send_inval, send_inval, send_inval, send_inval,
};
static ieee80211_send_action_func *meshpl_send_action[8] = {
	send_inval, send_inval, send_inval, send_inval,
	send_inval, send_inval, send_inval, send_inval,
};
static ieee80211_send_action_func *meshlm_send_action[4] = {
	send_inval, send_inval, send_inval, send_inval,
};
static ieee80211_send_action_func *hwmp_send_action[8] = {
	send_inval, send_inval, send_inval, send_inval,
	send_inval, send_inval, send_inval, send_inval,
};
static ieee80211_send_action_func *vendor_send_action[8] = {
	send_inval, send_inval, send_inval, send_inval,
	send_inval, send_inval, send_inval, send_inval,
};
#ifdef CONFIG_11W
static ieee80211_send_action_func *sa_query_send_action[2]={
	send_inval, send_inval
};
#endif
#ifdef IEEE80211K
static ieee80211_send_action_func *rm_send_action[8] = {
	send_inval, send_inval, send_inval, send_inval,
	send_inval, send_inval, send_inval, send_inval,
};
#endif

int
ieee80211_send_action_register(int cat, int act, ieee80211_send_action_func *f)
{
#define	N(a)	(sizeof(a) / sizeof(a[0]))
	switch (cat) {
	case IEEE80211_ACTION_CAT_BA:
		if (act >= N(ba_send_action))
			break;
		ba_send_action[act] = f;
		return 0;
	case IEEE80211_ACTION_CAT_HT:
		if (act >= N(ht_send_action))
			break;
		ht_send_action[act] = f;
		return 0;
	case IEEE80211_ACTION_CAT_MESHPEERING:
		if (act >= N(meshpl_send_action))
			break;
		meshpl_send_action[act] = f;
		return 0;
	case IEEE80211_ACTION_CAT_MESH:
		switch (act) {
		case IEEE80211_ACTION_MESH_LMETRIC:
			if (act >= N(meshlm_send_action))
				break;
			meshlm_send_action[act] = f;
			return 0;
		case IEEE80211_ACTION_MESH_HWMP:
			if (act >= N(hwmp_send_action))
				break;
			hwmp_send_action[act] = f;
			return 0;
		}
		break;
	case IEEE80211_ACTION_CAT_VENDOR:
		if (act >= N(vendor_send_action))
			break;
		vendor_send_action[act] = f;
		return 0;
#ifdef IEEE80211K
	case IEEE80211_ACTION_RADIO_MEAS:
	case IEEE80211_ACTION_CAT_SM:
		if (act >= N(rm_send_action))
			break;
		rm_send_action[act] = f;
		return 0;
#endif
#ifdef CONFIG_11W
	case IEEE80211_ACTION_CAT_SA_QUERY:
		if (act >= N(sa_query_send_action))
			break;
		sa_query_send_action[act] = f;
#endif
	}
	return EINVAL;
#undef N
}

void
ieee80211_send_action_unregister(int cat, int act)
{
	ieee80211_send_action_register(cat, act, send_inval);
}

int
ieee80211_send_action(struct ieee80211_node *ni, int cat, int act, void *sa)
{
#define	N(a)	(sizeof(a) / sizeof(a[0]))
	ieee80211_send_action_func *f = send_inval;

	switch (cat) {
	case IEEE80211_ACTION_CAT_BA:
		if (act < N(ba_send_action))
			f = ba_send_action[act];
		break;
	case IEEE80211_ACTION_CAT_HT:
		if (act < N(ht_send_action))
			f = ht_send_action[act];
		break;
#ifdef CONFIG_11W
	case IEEE80211_ACTION_CAT_SA_QUERY:
		if (act < N(sa_query_send_action))
			f = sa_query_send_action[act];
		break;
#endif
	case IEEE80211_ACTION_CAT_MESHPEERING:
		if (act < N(meshpl_send_action))
			f = meshpl_send_action[act];
		break;
	case IEEE80211_ACTION_CAT_MESH:
		switch (act) {
		case IEEE80211_ACTION_MESH_LMETRIC:
			if (act < N(meshlm_send_action))
				f = meshlm_send_action[act];
			break;
		case IEEE80211_ACTION_MESH_HWMP:
			if (act < N(hwmp_send_action))
				f = hwmp_send_action[act];
			break;
		}
		break;
	case IEEE80211_ACTION_CAT_VENDOR:
		if (act < N(vendor_send_action))
			f = vendor_send_action[act];
		break;
#ifdef IEEE80211K
	case IEEE80211_ACTION_RADIO_MEAS:
	case IEEE80211_ACTION_CAT_SM:
		if (act < N(rm_send_action))
		{
			f = rm_send_action[act];
		}
		break;
#endif
	}
	return f(ni, cat, act, sa);
#undef N
}

static int
recv_inval(struct ieee80211_node *ni, const struct ieee80211_frame *wh,
	const uint8_t *frm, const uint8_t *efrm)
{
	return EINVAL;
}

static ieee80211_recv_action_func *ba_recv_action[8] = {
	recv_inval, recv_inval, recv_inval, recv_inval,
	recv_inval, recv_inval, recv_inval, recv_inval,
};
static ieee80211_recv_action_func *ht_recv_action[8] = {
	recv_inval, recv_inval, recv_inval, recv_inval,
	recv_inval, recv_inval, recv_inval, recv_inval,
};
static ieee80211_recv_action_func *meshpl_recv_action[8] = {
	recv_inval, recv_inval, recv_inval, recv_inval,
	recv_inval, recv_inval, recv_inval, recv_inval,
};
static ieee80211_recv_action_func *meshlm_recv_action[4] = {
	recv_inval, recv_inval, recv_inval, recv_inval,
};
static ieee80211_recv_action_func *hwmp_recv_action[8] = {
	recv_inval, recv_inval, recv_inval, recv_inval,
	recv_inval, recv_inval, recv_inval, recv_inval,
};
static ieee80211_recv_action_func *vendor_recv_action[8] = {
	recv_inval, recv_inval, recv_inval, recv_inval,
	recv_inval, recv_inval, recv_inval, recv_inval,
};
#ifdef IEEE80211K
static ieee80211_recv_action_func *rm_recv_action[8] = {
	recv_inval, recv_inval, recv_inval, recv_inval,
	recv_inval, recv_inval, recv_inval, recv_inval,
};
static ieee80211_recv_action_func *sm_recv_action[8] = {
	recv_inval, recv_inval, recv_inval, recv_inval,
	recv_inval, recv_inval, recv_inval, recv_inval,
};
#endif
#ifdef CONFIG_11W
static ieee80211_recv_action_func *sa_query_recv_action[2] = {
	recv_inval, recv_inval
};
#endif
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
static ieee80211_recv_action_func *ft_bss_recv_action[8] = {
	recv_inval, recv_inval, recv_inval, recv_inval,
	recv_inval, recv_inval, recv_inval, recv_inval,
};
#endif

int
ieee80211_recv_action_register(int cat, int act, ieee80211_recv_action_func *f)
{
#define	N(a)	(sizeof(a) / sizeof(a[0]))
	switch (cat) {
	case IEEE80211_ACTION_CAT_BA:
		if (act >= N(ba_recv_action))
			break;
		ba_recv_action[act] = f;
		return 0;
	case IEEE80211_ACTION_CAT_HT:
		if (act >= N(ht_recv_action))
			break;
		ht_recv_action[act] = f;
		return 0;
#ifdef CONFIG_11W
	case IEEE80211_ACTION_CAT_SA_QUERY:
		if (act >= N(sa_query_recv_action))
			break;
		sa_query_recv_action[act] = f;
		return 0;
#endif
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
	case IEEE80211_ACTION_CAT_FT_BSS:
if (act >= N(ft_bss_recv_action)){
			break;
}
		ft_bss_recv_action[act] = f;
		return 0;
#endif
	case IEEE80211_ACTION_CAT_MESHPEERING:
		if (act >= N(meshpl_recv_action))
			break;
		meshpl_recv_action[act] = f;
		return 0;
	case IEEE80211_ACTION_CAT_MESH:
		switch (act) {
		case IEEE80211_ACTION_MESH_LMETRIC:
			if (act >= N(meshlm_recv_action))
				break;
			meshlm_recv_action[act] = f;
			return 0;
		case IEEE80211_ACTION_MESH_HWMP:
			if (act >= N(hwmp_recv_action))
				break;
			hwmp_recv_action[act] = f;
			return 0;
		}
		break;
	case IEEE80211_ACTION_CAT_VENDOR:
		if (act >= N(vendor_recv_action))
			break;
		vendor_recv_action[act] = f;
		return 0;
#ifdef IEEE80211K
	case IEEE80211_ACTION_RADIO_MEAS:
		if (act >= N(rm_recv_action))
			break;
		rm_recv_action[act] = f;
		return 0;
	case IEEE80211_ACTION_CAT_SM:
		if (act >= N(sm_recv_action))
			break;
		sm_recv_action[act] = f;
		return 0;
#endif
}
	return EINVAL;
#undef N
}

void
ieee80211_recv_action_unregister(int cat, int act)
{
	ieee80211_recv_action_register(cat, act, recv_inval);
}

int
ieee80211_recv_action(struct ieee80211_node *ni,
	const struct ieee80211_frame *wh,
	const uint8_t *frm, const uint8_t *efrm)
{
#define	N(a)	(sizeof(a) / sizeof(a[0]))
	ieee80211_recv_action_func *f = recv_inval;
	const struct ieee80211_action *ia =
	    (const struct ieee80211_action *) frm;

	switch (ia->ia_category) {
	case IEEE80211_ACTION_CAT_BA:
		if (ia->ia_action < N(ba_recv_action))
			f = ba_recv_action[ia->ia_action];
		break;
	case IEEE80211_ACTION_CAT_HT:
		if (ia->ia_action < N(ht_recv_action))
			f = ht_recv_action[ia->ia_action];
		break;
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
  case IEEE80211_ACTION_CAT_FT_BSS:
    if (ia->ia_action <= N(ft_bss_recv_action)){
     f = ft_bss_recv_action[ia->ia_action];
    }
    break;
#endif 
#ifdef CONFIG_11W
	case IEEE80211_ACTION_CAT_SA_QUERY:
		if (ia->ia_action < N(sa_query_recv_action))
			f = sa_query_recv_action[ia->ia_action];
		break;
#endif
	case IEEE80211_ACTION_CAT_MESHPEERING:
		if (ia->ia_action < N(meshpl_recv_action))
			f = meshpl_recv_action[ia->ia_action];
		break;
	case IEEE80211_ACTION_CAT_MESH:
		switch (ia->ia_action) {
		case IEEE80211_ACTION_MESH_LMETRIC:
			if (ia->ia_action < N(meshlm_recv_action))
				f = meshlm_recv_action[ia->ia_action];
			break;
		case IEEE80211_ACTION_MESH_HWMP:
			if (ia->ia_action < N(hwmp_recv_action))
				f = hwmp_recv_action[ia->ia_action];
			break;
		}
		break;
	case IEEE80211_ACTION_CAT_VENDOR:
		if (ia->ia_action < N(vendor_recv_action))
			f = vendor_recv_action[ia->ia_action];
		break;
#ifdef IEEE80211K
	case IEEE80211_ACTION_RADIO_MEAS:
		if (ia->ia_action < N(rm_recv_action))
			f = rm_recv_action[ia->ia_action];
		break;
	case IEEE80211_ACTION_CAT_SM:
		if (ia->ia_action < N(sm_recv_action))
			f = sm_recv_action[ia->ia_action];
		break;

#endif

	}
	return f(ni, wh, frm, efrm);
#undef N
}
