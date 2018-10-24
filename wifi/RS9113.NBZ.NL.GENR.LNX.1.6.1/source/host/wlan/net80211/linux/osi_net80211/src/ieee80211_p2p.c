#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_input.h>

static void p2p_remain_on_channel_timeout(unsigned long arg)
{
	struct ieee80211vap *vap = (struct ieee80211vap *)arg;

	
	send_remain_on_channel_event(vap, vap->p2p->remain_on_channel_freq, vap->p2p->remain_on_channel_duration);
}

static void p2p_cancel_remain_on_channel_timeout(unsigned  long arg)
{
	struct ieee80211vap *vap = (struct ieee80211vap *)arg;
	struct ieee80211req ireq;
	

	/* Set the channel to any so that the scan requests followed can be serviced */
	ireq.i_val = IEEE80211_CHAN_ANY;
	ireq.i_data = 0;
	ieee80211_ioctl_setchannel(vap, &ireq);

	/* Send the cancellation event to supplicant */
	send_cancel_remain_on_channel_event(vap, vap->p2p->remain_on_channel_freq);
}

void p2p_noa_duration_timeout(unsigned long arg)
{
	struct ieee80211vap *vap = (struct ieee80211vap *)arg;
	/* Unblocking data queues after duration timeout */
	vap->iv_block(vap, 0, 0);
}

void p2p_noa_interval_timeout(unsigned long arg)
{
	struct ieee80211vap *vap = (struct ieee80211vap *)arg;
	struct ieee80211_p2p *p2p = vap->p2p; 

	//vap->iv_block(vap, 1, 0); /* Blocking data queues */
	if (p2p->noa->count == 0xff) {
		goto timer_reset;
	} else {
		p2p->noa->local_count--;
		if (p2p->noa->local_count > 0) {
			goto timer_reset;
		} else {
			/*stop both interval and duration timer and make p2p->noa->count = 0 */
			p2p->noa->count = 0;
			return;
		}
	}

timer_reset:
  /** Blocking data queues as required conditions
   * are met will unblock at NOA duration timeout
   */
	vap->iv_block(vap, 1, 0); /* Blocking data queues */
	callout_reset(&p2p->noa_duration_timer, msecs_to_jiffies(p2p->noa->noa_duration/1000),
	              p2p_noa_duration_timeout, vap);
	callout_reset(&p2p->noa_interval_timer, msecs_to_jiffies(p2p->noa->noa_interval/1000),
		      p2p_noa_interval_timeout, vap);
}

static int
is11bclient(const uint8_t *rates, const uint8_t *xrates)
{
	const uint32_t brates = (1<<2*1)|(1<<2*2)|(1<<11)|(1<<2*11);
	int i;

	/* NB: the 11b clients we care about will not have xrates */
	if (xrates != NULL || rates == NULL)
		return 0;
	for (i = 0; i < rates[1]; i++) {
		int r = rates[2+i] & IEEE80211_RATE_VAL;
		if (r > 2*11 || ((1<<r) & brates) == 0)
			return 0;
	}
	return 1;
}

static int
is11b_rate_present(const uint8_t *rates, const uint8_t *xrates)
{
	int i;

	/* NB: the 11b clients we care about will not have xrates */
	if(rates)
	{
		for (i = 0; i < rates[1]; i++) {
			int r = rates[2+i] & IEEE80211_RATE_VAL;

			if ((r == 2) || (r == 4) || (r == 11) || (r == 22))
				return 1;
		}
	}

	if(xrates)
	{
		for (i = 0; i < xrates[1]; i++) {
			int r = xrates[2+i] & IEEE80211_RATE_VAL;

			if ((r == 2) || (r == 4) || (r == 11) || (r == 22))
				return 1;
		}
	}
	return 0;
}

static int p2p_set_channel(struct ieee80211vap *vap, uint16_t freq)
{
	u_int flags = 0;
	struct ieee80211req ireq;
	int status;

	if((freq >= 2412) && (freq <= 2477))
	{
		flags = IEEE80211_CHAN_2GHZ;
		vap->iv_des_mode = IEEE80211_MODE_11NG;
	}
	else if ((freq >= 5180) && (freq <= 5320))
	{
		flags = IEEE80211_CHAN_5GHZ;
		vap->iv_des_mode = IEEE80211_MODE_11NA;
	}
	ireq.i_val = ieee80211_mhz2ieee(freq, flags);
	ireq.i_data = 0;
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
			("P2P: In %s and %d flags = %0x, freq = %d, channel = %d\n", __func__, __LINE__, flags, freq, ireq.i_val));

	/* Concurrent mode needs to be taken care 
	 * while changing channel here */
	status = ieee80211_ioctl_setchannel(vap, &ireq);
	//status = ieee80211_ioctl_setcurchan(vap, &ireq);
	if(status)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Unable to set channel: return value = %d\n", status));
		return status;
	}

	if (vap->iv_des_chan == NULL) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
				("P2P: In %s %d desc_chan", __func__, __LINE__));
		dump_stack();
		return -EINVAL;
	} else {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
				("P2P: In %s %d desc_chan =%p \n", __func__, __LINE__, vap->iv_des_chan));
	}
	ieee80211_setcurchan(vap->iv_ic, vap->iv_des_chan, vap);
	vap->iv_bss->ni_chan = vap->iv_ic->ic_curchan;
	return 0;
}

void p2p_set_probe_req_report(struct ieee80211vap *vap, uint16_t probe_req_report)
{
	if(probe_req_report)
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
				("Starting probe req reporting\n"));
	else
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
				("Stopping probe req reporting\n"));

	vap->p2p->probe_req_report = probe_req_report;
	return;
}

#ifndef ONEBOX_CONFIG_CFG80211
int p2p_send_action(struct ieee80211vap *vap, uint8_t *dst, uint8_t *src, uint8_t *bssid,
                     uint8_t *data, uint16_t data_len, unsigned int freq)
{ //TX_STATUS_EVENT has to be sent to supplicant
	struct mbuf *m;
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_bpf_params params;
	uint8_t *frm;
	struct ieee80211_frame *wh;
	int type = IEEE80211_FC0_SUBTYPE_ACTION;
	struct ieee80211_node *ni;
#ifdef IEEE80211_DEBUG
	uint8_t ether_buf[18];
#endif

	ni = ieee80211_find_txnode(vap, bssid);
	if(ni == NULL)
	{
		return -1;
	}

	m = ieee80211_getmgtframe(&frm,
	    ic->ic_headroom + sizeof(struct ieee80211_frame),
	    /* XXX may action payload */
	    + data_len
	);

	if(m == NULL)
	{
		return ENOMEM;
	}

	/* Copying the action packet body to mbuf */
	memcpy(frm, data, data_len);

	memset(&params, 0, sizeof(params));
	params.ibp_pri = WME_AC_VO;
	params.ibp_rate0 = ni->ni_txparms->mgmtrate;
	/* NB: we know all frames are unicast */
	params.ibp_try0 = ni->ni_txparms->maxretry;
	params.ibp_power = ni->ni_txpower;

	/* The following piece code is taken from ieee80211_mgmt_output function and 
	 * input params to ieee80211_send_setup have been changed */
	if (vap->iv_state == IEEE80211_S_CAC) {
		IEEE80211_NOTE(vap, IEEE80211_MSG_OUTPUT | IEEE80211_MSG_DOTH,
		    ni, "block %s frame in CAC state",
			ieee80211_mgt_subtype_name[
			    (type & IEEE80211_FC0_SUBTYPE_MASK) >>
				IEEE80211_FC0_SUBTYPE_SHIFT]);
		vap->iv_stats.is_tx_badstate++;
		ieee80211_free_node(ni);
		m_freem(m);
		return EIO;		/* XXX */
	}

	if (!(M_PREPEND(m, sizeof(struct ieee80211_frame), M_DONTWAIT))) {
		m_freem(m);
		ieee80211_free_node(ni);
		return ENOMEM;
	}

	wh = mtod(m, struct ieee80211_frame *);
	ieee80211_send_setup(ni, m,
	     IEEE80211_FC0_TYPE_MGT | type, IEEE80211_NONQOS_TID,
	     src, dst, bssid);
	m->m_flags |= M_ENCAP;		/* mark encapsulated */

	M_WME_SETAC(m, params.ibp_pri);

#ifdef IEEE80211_DEBUG
	/* avoid printing too many frames */
	if ((ieee80211_msg_debug(vap) && doprint(vap, type)) ||
	    ieee80211_msg_dumppkts(vap)) {
		printf("[%s] send %s on channel %u\n",
		    ether_sprintf(wh->i_addr1, ether_buf),
		    ieee80211_mgt_subtype_name[
			(type & IEEE80211_FC0_SUBTYPE_MASK) >>
				IEEE80211_FC0_SUBTYPE_SHIFT],
		    ieee80211_chan2ieee(ic, ic->ic_curchan));
	}
#endif
	IEEE80211_NODE_STAT(ni, tx_mgmt);
	if(freq != ic->ic_curchan->ic_freq)
	{
		p2p_set_channel(vap, freq);
	}
	return ic->ic_raw_xmit(ni, m, &params);
}
#else
int p2p_send_action(struct ieee80211vap *vap, unsigned int freq)
{
	p2p_set_channel(vap, freq);
	return 0;
}
#endif

static void p2p_remain_on_channel(struct ieee80211vap *vap, uint16_t duration, unsigned int freq)
{ //EVENT_REMAIN_ON_CHANNEL needs to be sent to supplicant
	struct ieee80211_p2p *p2p = vap->p2p;

	p2p->remain_on_channel_freq = freq;
	p2p->remain_on_channel_duration = duration;

	/* Program the Channel here */
	p2p_set_channel(vap, freq);
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("-p2p_set_channnel\n"));

	/* Ideally it should be sent when we receive the rf_prog confirm from lower layer
	 * For now assuming it will take 1 ms for changing the channel */
	callout_reset(&p2p->remain_on_channel_timer, msecs_to_jiffies(1),
		p2p_remain_on_channel_timeout, vap);

	/* Ideally this timer should be started upon receiving rf confirm */
	callout_reset(&p2p->cancel_remain_on_channel_timer, msecs_to_jiffies(duration),
		p2p_cancel_remain_on_channel_timeout, vap);
	return;
}

static void p2p_cancel_remain_on_channel(struct ieee80211vap *vap)
{ //CMD_CANCEL_REMAIN_ON_CHANNEL needs to be sent to supplicant
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
			("CMD_CANCEL_REMAIN_ON_CHANNEL came to driver from supplicant\n"));

	/* registering/modifying the timer to cancel channel immediately */
///	callout_reset(&vap->p2p->cancel_remain_on_channel_timer, 0,
	//	p2p_cancel_remain_on_channel_timeout, vap);
	callout_stop(&vap->p2p->cancel_remain_on_channel_timer);
	p2p_cancel_remain_on_channel_timeout( (unsigned long )vap);
}

static int change_vap_from_sta_to_hostap(struct ieee80211vap *vap)
{
	int new_opmode = IEEE80211_M_HOSTAP;
	//int old_opmode = IEEE80211_M_STA;
	int flags = 0;

	/* This is kind of patchy. Better if we figure out another intelligent/less-buggy
	 * way to do this */
	/* CHANGING ALL VAP PARAMETERS FROM STA TO HOSTAP */

	if (callout_pending(&vap->p2p->cancel_remain_on_channel_timer))
		callout_schedule(&vap->p2p->cancel_remain_on_channel_timer, IEEE80211_P2P_FOREVER);
	/* Uninitializing the STA mode parameters those need to be */
	ieee80211_vap_detach(vap, 1, 1);


	/* Initializing the params for HOSTAP mode */

	/* Setting up vap in new_opmode */
	flags = IEEE80211_CLONE_BSSID;
	ieee80211_vap_setup(vap->iv_ic, vap, NULL, 0, new_opmode, flags, NULL, NULL, 1);

	vap->iv_ifp->if_flags &= ~IFF_UP;
	vap->p2p_mode = IEEE80211_P2P_GO;
	vap->p2p->p2p_hal_mode_change_from_sta_to_hostap(vap);
	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
	//("In %s %d vap->iv_bss = %p\n", __func__, __LINE__, vap->iv_bss));
	/* complete setup */
	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s %d\n", __func__, __LINE__));
	ieee80211_vap_attach(vap, ieee80211_media_change, ieee80211_media_status, 1);
#ifdef BYPASS_TX_DATA_PATH
	vap->iv_caps |= IEEE80211_C_8023ENCAP;
#endif

	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s %d state = %d, opmode = %d\n", __func__, __LINE__, vap->iv_state, vap->iv_opmode));
	return 0;
}

static int p2p_change_vap_mode(struct ieee80211vap *vap, uint32_t mode)
{
	int status;
	enum ieee80211_opmode	new_opmode;	/* operation mode */

	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("+%s\n", __func__));

	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
			("CMD_CHANGE_VAP_MODE came to supplicant from driver mode = %d\n", mode));
	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("vap->iv_opmode = %d mode = %d\n", vap->iv_opmode, mode));
	switch (mode)
	{
		case 0: /* INFRA STA Mode */
		{
			if(vap->iv_opmode == IEEE80211_M_STA)
				return 0;
			new_opmode = IEEE80211_M_STA;
		}
		break;
		case IFM_IEEE80211_IBSS:
		{
			if(vap->iv_opmode == IEEE80211_M_IBSS)
				return 0;
			new_opmode = IEEE80211_M_IBSS;
		}
		break;
		case IFM_IEEE80211_HOSTAP:
		{
			if(vap->iv_opmode == IEEE80211_M_HOSTAP)
				return 0;
			new_opmode = IEEE80211_M_HOSTAP;
			vap->iv_state = IEEE80211_S_INIT;
		}
		break;
		default:
		{
			return 0;
		}
		break;
	}

	IEEE80211_LOCK(vap->iv_ic);
	/* Reading the state after aquiring lock because vap could be just changing
	 * the state when this function gets called */
	status = (vap->iv_state != IEEE80211_S_INIT);
	IEEE80211_UNLOCK(vap->iv_ic);

	if(status)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("vap->iv_state = %d\n", vap->iv_state));
		return -1;
	}

	/* For now handling only M_STA --> M_HOSTAP case */
	if((new_opmode != IEEE80211_M_HOSTAP) || (vap->iv_opmode != IEEE80211_M_STA))
	{
		return 0;
	}

	/* Changing the vap mode */
	change_vap_from_sta_to_hostap(vap);

	return 0;
}

static __inline int
isp2poui(const uint8_t *frm)
{
	return frm[1] > 3 && LE_READ_4(frm+2) == ((P2P_OUI_TYPE<<24)|WFA_OUI);
}

static void p2p_recv_mgmt(struct ieee80211_node *ni, struct mbuf *m)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_frame *wh;
	uint16_t recv_freq;
	uint8_t sub_type;
	struct ieee80211_p2p_action *action_header;

#define IEEE80211_GET_LE24(a) ((((u32) (a)[2]) << 16) | (((u32) (a)[1]) << 8) | \
			 ((u32) (a)[0]))

	wh = mtod(m, struct ieee80211_frame *);
	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
	//("P2P: In %s and %d mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\n", __func__, __LINE__, wh->i_addr2[0], wh->i_addr2[1], wh->i_addr2[2], wh->i_addr2[3], wh->i_addr2[4], wh->i_addr2[5]));
	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("pkt len = %d\n", m->m_len));
	action_header = (struct ieee80211_p2p_action *)(wh+1);

	/* Process only mgmt packets */
	if((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_MGT)
	{
		//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("P2P: In %s and %d Returning not of type mgmt\n", __func__, __LINE__));
		return;
	}

	sub_type = (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK);
	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("sub_type = %0x\n", sub_type));
	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
	//("In %s %d opmode = %d, p2p_mode = %d\n", __func__, __LINE__, vap->iv_opmode, vap->p2p_mode));

	switch (sub_type)
	{
		case IEEE80211_FC0_SUBTYPE_PROBE_REQ:
		{

			if(vap->p2p->probe_req_report == 0)
			{
				//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
				//					("P2P: In %s and %d probe request reporting is not enabled\n", __func__, __LINE__));
				return;
			}
#ifndef ONEBOX_CONFIG_CFG80211
			/* While using NL80211, Wpa supplicant will send probe responses, so disabling for NL80211 */
			/* net80211 will take care of this once we become GO/CLIENT */
			if((vap->iv_opmode == IEEE80211_M_STA) && (vap->p2p_mode == IEEE80211_P2P_DEVICE))
			{ /* Send the probe response from here */
				uint8_t *frm, *efrm, *sfrm;
				uint8_t *ssid, *rates, *xrates, *p2p;

				frm = (uint8_t *)&wh[1];
				efrm = mtod(m, uint8_t *) + m->m_len;

				/*
				 * prreq frame format
				 *	[tlv] ssid
				 *	[tlv] supported rates
				 *	[tlv] extended supported rates
				 *	[tlv] p2p element
				 */
				ssid = rates = xrates = p2p = NULL;
				sfrm = frm;
				while (efrm - frm > 1) {
					IEEE80211_VERIFY_LENGTH(efrm - frm, frm[1] + 2, return);
					switch (*frm) {
					case IEEE80211_ELEMID_SSID:
						ssid = frm;
						break;
					case IEEE80211_ELEMID_RATES:
						rates = frm;
						break;
					case IEEE80211_ELEMID_XRATES:
						xrates = frm;
						break;
					case IEEE80211_ELEMID_VENDOR:
					{
						if(isp2poui(frm))
						{
							/* If more than one p2p ie is present, the ultimate one will be assigned here
							 * Not a problem for now as we are not processing ie here */
							p2p = frm;
						}
					}
						break;
					}
					frm += frm[1] + 2;
				}

				/* Check whether p2p ie is present in received probe request */
				if(p2p == NULL)
				{
					//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("P2P: In %s and %d Returning : no P2P ie\n", __func__, __LINE__));
					return;
				}
				IEEE80211_VERIFY_ELEMENT(rates, IEEE80211_RATE_MAXSIZE, return);
				if (xrates != NULL)
					IEEE80211_VERIFY_ELEMENT(xrates,
						IEEE80211_RATE_MAXSIZE - rates[1], return);
				if(is11b_rate_present(rates, xrates))
				{
					//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("P2P: In %s and %d Returning : 11b rates present\n", __func__, __LINE__));
					return;
				}
				IEEE80211_VERIFY_ELEMENT(ssid, IEEE80211_NWID_LEN, return);

				/* Check for p2p wild card ssid */
				if((ssid[1] != vap->p2p->p2p_wildcard_essid_len) ||
				   (memcmp(&ssid[2], vap->p2p->p2p_wildcard_essid, vap->p2p->p2p_wildcard_essid_len)))
				{
					//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("P2P: In %s and %d Returning : No P2P_WILDCARD_SSID\n", __func__, __LINE__));
					return;
				}
				if ((vap->iv_flags & IEEE80211_F_HIDESSID) && ssid[1] == 0) {
					IEEE80211_DISCARD(vap, IEEE80211_MSG_INPUT,
					    wh, NULL,
					    "%s", "no ssid with ssid suppression enabled");
					vap->iv_stats.is_rx_ssidmismatch++; /*XXX*/
					//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("P2P: In %s and %d Returning : Because of Hide SSID\n", __func__, __LINE__));
					return;
				}
	
				/* XXX find a better class or define it's own */
				IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_INPUT, wh->i_addr2,
				    "%s", "recv probe req");
				/*
				 * Some legacy 11b clients cannot hack a complete
				 * probe response frame.  When the request includes
				 * only a bare-bones rate set, communicate this to
				 * the transmit side.
				 */
				//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s %d sending probe response\n", __func__, __LINE__));
				ieee80211_send_proberesp(vap, wh->i_addr2,
				    is11bclient(rates, xrates) ? IEEE80211_SEND_LEGACY_11B : 0, 1);
			}
#endif //ONEBOX_CONFIG_CFG80211
		}
		break;
		case IEEE80211_FC0_SUBTYPE_ACTION:
		case IEEE80211_FC0_SUBTYPE_ACTION_NOACK:
		{
			switch(action_header->category)
			{
				case IEEE80211_ACTION_CAT_PUBLIC:
				{
					switch(action_header->u.p2p_public_action_header.action_field)
					{
						/* For now allow only the following action categories */
						/* Eventually it will be better if we move p2p implementation to driver and hence 
						 * this switch case is wriiten and left empty for now */
						case IEEE80211_PA_VENDOR_SPECIFIC:
						case IEEE80211_PA_GAS_INITIAL_REQ:
						case IEEE80211_PA_GAS_INITIAL_RESP:
						case IEEE80211_PA_GAS_COMEBACK_REQ:
						case IEEE80211_PA_GAS_COMEBACK_RESP:
						break;
						default:
						{
							return;
						}
						break;
					}
					if(IEEE80211_GET_LE24(action_header->u.p2p_public_action_header.oui) != WFA_OUI)
					{ /* Don't give it to supplicant if not WFA_OUI */
						return;
					}
					if(action_header->u.p2p_public_action_header.oui_type != P2P_OUI_TYPE)
					{ /* For now allowing only P2P action packets later on this may have to change in order
						 to accomodate WiFi-Display etc. */
						return;
					}
				}
				break;
				case IEEE80211_ACTION_CAT_VENDOR:
				{
					if(IEEE80211_GET_LE24(action_header->u.p2p_action_header.oui) != WFA_OUI)
					{
						return;
					}
					if(action_header->u.p2p_action_header.oui_type != P2P_OUI_TYPE)
					{
						return;
					}
				}
				break;
				default:
				{
					return;
				}
				break;
			}
		}
		break;
		default:
		break;
	}

	/* Taking the recv freq from ic_curchan for now. Better if we get this value from the device*/
	recv_freq = vap->iv_ic->ic_curchan->ic_freq;
	notify_recv_mgmt_pkt(vap, mtod(m, uint8_t *), m->m_len, recv_freq);

	return;
#undef IEEE80211_GET_LE24
}

void p2p_deinit(struct ieee80211vap *vap)
{
	struct ieee80211_p2p *p2p = vap->p2p;

	if(p2p)
	{
		callout_stop(&p2p->remain_on_channel_timer);
		callout_stop(&p2p->cancel_remain_on_channel_timer);
		callout_stop(&p2p->noa_duration_timer);
		callout_stop(&p2p->noa_interval_timer);
		if (p2p->noa)
			kfree(p2p->noa);
		kfree(p2p);
	}
	vap->p2p = NULL;
	vap->p2p_enable = 0;
	return;
}

struct ieee80211_p2p * p2p_init(struct ieee80211vap *vap)
{
	struct ieee80211_p2p *p2p;

	p2p = (struct ieee80211_p2p *) ieee80211_malloc(sizeof(struct ieee80211_p2p), M_NOWAIT | M_ZERO);

	if(p2p == NULL)
	{
		return NULL;
	}

	vap->p2p = p2p;
	vap->p2p_enable = 1;
	p2p->p2p_vap = vap;
	vap->p2p_mode = IEEE80211_P2P_DEVICE;

	p2p->p2p_set_probe_req_report = p2p_set_probe_req_report;
	p2p->p2p_send_action = p2p_send_action;
	p2p->p2p_remain_on_channel = p2p_remain_on_channel;
	p2p->p2p_cancel_remain_on_channel = p2p_cancel_remain_on_channel;
	p2p->p2p_change_vap_mode = p2p_change_vap_mode;
	p2p->p2p_recv_probe_req = p2p_recv_mgmt;
	p2p->p2p_recv_action = p2p_recv_mgmt;
	p2p->p2p_deinit = p2p_deinit;
	strcpy(p2p->p2p_wildcard_essid, "DIRECT-");
	p2p->p2p_wildcard_essid_len = 7;

	/* Initializing remain_on_channel_timer 
	 * For now doing it in net80211 way */
	callout_init(&p2p->remain_on_channel_timer, 0);
	callout_init(&p2p->cancel_remain_on_channel_timer, 0);
	callout_init(&p2p->noa_duration_timer, 0);
	callout_init(&p2p->noa_interval_timer, 0);

	return p2p;
}

EXPORT_SYMBOL(p2p_init);

/**
 * ieee80211_process_noa() - This function processes the NoA attribute
 *			     for a P2P client that is connected.
 *
 * @vap: Pointer to the p2p structure.
 * @frm: Pointer to the data
 *
 * Return: None
 */
void ieee80211_process_noa(struct ieee80211_p2p *p2p, uint8_t *frm, uint8_t *tstamp)
{
	/* Allocating memory for p2p->noa of not done already */
	struct ieee80211vap *vap = p2p->p2p_vap;

	if (callout_pending(&p2p->noa_interval_timer) || 
	    callout_pending(&p2p->noa_duration_timer)) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Quiet already in progress\n"));
		return;
	}

	if (p2p->noa == NULL) {
		p2p->noa = (struct ieee80211_p2p_noa *) ieee80211_malloc(sizeof(struct ieee80211_p2p_noa), M_NOWAIT | M_ZERO);
		if (p2p->noa == NULL) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
					("In %s Line %d allocating memory for noa attr failed\n", __func__, __LINE__));
			return;
		}
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
				("In %s Line %d allocated memory for noa attr\n", __func__, __LINE__));
	}

	if (!p2p->noa->count) {
		memcpy(&p2p->noa->local_count, frm, 13);
		p2p->noa->count = p2p->noa->local_count;
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
				("Allocating values for first time\n"));
	}
	
	/* checking for start time by comparing with lower 4 bytes of beacon tsf */
	if (p2p->noa->count == 0xff) {
		if (memcmp(&tstamp[0], &p2p->noa->noa_start_time, 4) > 0) {
			goto timer_reset;
		}
    return;
	}  else {
		goto timer_reset;
	}

timer_reset:
	/* block the data queues */
	vap->iv_block(vap, 1, 0);

	/* start the interval and duration timers */
	callout_reset(&p2p->noa_interval_timer, msecs_to_jiffies(p2p->noa->noa_interval/1000),
			p2p_noa_interval_timeout, vap);
	callout_reset(&p2p->noa_duration_timer, msecs_to_jiffies(p2p->noa->noa_duration/1000),
			p2p_noa_duration_timeout, vap);
}
EXPORT_SYMBOL(ieee80211_process_noa);
