/*-
 * Copyright (c) 2007-2008 Sam Leffler, Errno Consulting
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

/*
 * IEEE 802.11w protocol support.
 */
#ifdef CONFIG_11W
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_action.h>
#include <net80211/ieee80211_input.h>

static	ieee80211_recv_action_func mfp_recv_action_sa_query_request;
static	ieee80211_recv_action_func mfp_recv_action_sa_query_response;

static	ieee80211_send_action_func mfp_send_action_sa_query_request;
static	ieee80211_send_action_func mfp_send_action_sa_query_response;

#define	ADDSHORT(frm, v) do {			\
	frm[0] = (v) & 0xff;			\
	frm[1] = (v) >> 8;			\
	frm += 2;				\
} while (0)

void
ieee80211_mfp_attach(struct ieee80211com *ic)
{
	/*
	 * Register action frame handlers.
	 */
	ieee80211_recv_action_register(IEEE80211_ACTION_CAT_SA_QUERY,
	    IEEE80211_ACTION_SA_QUERY_REQUEST, mfp_recv_action_sa_query_request);
	ieee80211_recv_action_register(IEEE80211_ACTION_CAT_SA_QUERY,
	    IEEE80211_ACTION_SA_QUERY_RESPONSE, mfp_recv_action_sa_query_response);

	ieee80211_send_action_register(IEEE80211_ACTION_CAT_SA_QUERY,
	    IEEE80211_ACTION_SA_QUERY_REQUEST, mfp_send_action_sa_query_request);
	ieee80211_send_action_register(IEEE80211_ACTION_CAT_SA_QUERY,
	    IEEE80211_ACTION_SA_QUERY_RESPONSE, mfp_send_action_sa_query_response);
}

void
ieee80211_mfp_detach(struct ieee80211com *ic)
{
	//callout_drain(&ic->ic_sa_procedure);
}

/*
 * Process a received action frame using the default aggregation
 * policy.  We intercept ADDBA-related frames and use them to
 * update our aggregation state.  All other frames are passed up
 * for processing by ieee80211_recv_action.
 */
static int
mfp_recv_action_sa_query_request(struct ieee80211_node *ni,
	const struct ieee80211_frame *wh,
	const uint8_t *frm, const uint8_t *efrm)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct ieee80211vap *vap = ni->ni_vap;
	uint16_t args;

	/* Not to own unicast address */
	if (!IEEE80211_ADDR_EQ(wh->i_addr1, vap->iv_myaddr))
		return 0;

	/* In STA mode, if connection is in progress or assoc retry is
	 * happening, ignore the request */
#if 1
	if ((vap->iv_opmode == IEEE80211_M_STA) &&
	    ((vap->hal_priv_vap->conn_in_prog) ||
	     (vap->hal_priv_vap->assoc_retry)))
		return 0;
#endif
	/* We do not handle SA query requests in AP Mode, for now */

	/* Extract the dialog token */
	args = *((uint16_t *)&frm[2]);

	ic->ic_send_action(ni, IEEE80211_ACTION_CAT_SA_QUERY,
			   IEEE80211_ACTION_SA_QUERY_RESPONSE, &args);
	return 0;
}

static int
mfp_recv_action_sa_query_response(struct ieee80211_node *ni,
	const struct ieee80211_frame *wh,
	const uint8_t *frm, const uint8_t *efrm)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct ieee80211vap *vap = ni->ni_vap;
	struct sa_query_info *sa_info = &ni->hal_priv_node.sa_info;
	uint8_t dialogtoken;

	dialogtoken = frm[2];

	/* Not to own unicast address */
	if (!IEEE80211_ADDR_EQ(wh->i_addr1, vap->iv_myaddr))
		return 0;

	if (sa_info->dialogtoken != dialogtoken) {
		return 0;
	}
	/* Dialog token matched */
	sa_info->recvd_response = 1;

	ni->ni_flags &= ~IEEE80211_NODE_AUTH;
	ni->ni_associd = 0;
	vap->iv_sta_assoc--;
	ic->ic_sta_assoc--;
	ieee80211_sta_leave(ni);
	memset(sa_info, 0, sizeof(sa_info));

	if (callout_pending(&ic->ic_sa_procedure)) {
		callout_stop(&ic->ic_sa_procedure);
	}

	return 0;
}

static int
mfp_action_output(struct ieee80211_node *ni, struct mbuf *m)
{
        struct ieee80211_bpf_params params;

        memset(&params, 0, sizeof(params));
        /* NB: we know all frames are unicast */
        params.ibp_try0 = ni->ni_txparms->maxretry;
        return ieee80211_mgmt_output(ni, m, IEEE80211_FC0_SUBTYPE_ACTION,
             &params);
}

static int
mfp_send_action_sa_query_request(struct ieee80211_node *ni,
        int category, int action, void *arg)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct sa_query_info *sa_info = &ni->hal_priv_node.sa_info;
	struct mbuf *m;
	uint8_t *frm;
	uint16_t *args = arg;
	m = ieee80211_getmgtframe(&frm,
            ic->ic_headroom + sizeof(struct ieee80211_frame),
            sizeof(uint16_t) * 2);   /* action+category+dialogtoken */
        if (m != NULL) {
                *frm++ = 8; //category;
                *frm++ = 0; //action;
		ADDSHORT(frm, *args);
		sa_info->dialogtoken = *args;
	}
	m->m_len = m->m_pkthdr.len = frm - mtod(m, uint8_t *);
	if (sa_info->sa_flags != IEEE80211_SA_INIT_DONE) {
		sa_info->sa_flags = IEEE80211_SA_INIT_DONE;
		callout_init(&ic->ic_sa_procedure, CALLOUT_MPSAFE);
	}
	return mfp_action_output(ni, m);
}

static int
mfp_send_action_sa_query_response(struct ieee80211_node *ni,
        int category, int action, void *arg)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct mbuf *m;
	uint8_t *frm;
	uint16_t *args = arg;
	m = ieee80211_getmgtframe(&frm,
            ic->ic_headroom + sizeof(struct ieee80211_frame),
            sizeof(uint16_t) * 2);    /* action+categor+dialogtoken */
        if (m != NULL) {
                *frm++ = category;
                *frm++ = action;
		ADDSHORT(frm, args[0]);
		printk("%s %d Dialogtoken is: %d\n",
		       __func__,__LINE__, *(uint16_t *)&args[0]);
	}
	m->m_len = m->m_pkthdr.len = frm - mtod(m, uint8_t *);
	return mfp_action_output(ni, m);
}

void
ieee80211_sa_query_request(unsigned long param)
{
	struct ieee80211_node *ni = (struct ieee80211_node *)param;
	struct ieee80211com *ic = ni->ni_ic;
	struct ieee80211vap *vap = ni->ni_vap;
	struct sa_query_info *sa_info = &ni->hal_priv_node.sa_info;
	uint16_t arg;
	int dialogtoken;
	static int tokens = 0;	/* XXX */
	uint32_t tu, ms;
	unsigned long timeout;

	dialogtoken = (tokens+1) % 63;		/* XXX */
	arg = dialogtoken;
	tokens++;

	if (sa_info->retries == 5) {
		printk("Max retries reached\n");
		ni->ni_flags &= ~IEEE80211_NODE_AUTH;
		ni->ni_associd = 0;
		vap->iv_sta_assoc--;
		ic->ic_sta_assoc--;
		/* Marking the station as disconnected */
		memset(sa_info, 0, sizeof(struct sa_query_info));
		ieee80211_sta_leave(ni);
		callout_stop(&ic->ic_sa_procedure);
		return;
	}

	if (sa_info->recvd_response) {
		memset(sa_info, 0, sizeof(sa_info));
		return;
	}

	sa_info->recvd_response = 0;
	sa_info->dialogtoken = dialogtoken;
	sa_info->initiate_sa = 1;
	++sa_info->retries;

	tu = le32_to_cpu(200);
	ms = tu * 1024/1000;
	timeout = msecs_to_jiffies(ms);

	mfp_send_action_sa_query_request(ni, IEEE80211_ACTION_CAT_SA_QUERY,
				IEEE80211_ACTION_SA_QUERY_REQUEST, &arg);
	callout_reset(&ic->ic_sa_procedure, timeout, ieee80211_sa_query_request, ni);
}
#endif
