/*
 * IEEE 802.11k protocol support.
 */
#ifdef __FREEBSD__
#include <sys/cdefs.h>
#ifdef __FreeBSD__
__FBSDID("$FreeBSD: src/sys/net80211/ieee80211_action.c,v 1.6 2012/03/04 05:49:39 adrian Exp $");
#endif


#include "opt_inet.h"
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h> 
#include <sys/endian.h>
 
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/ethernet.h>
#endif

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_action.h>
#include <net80211/ieee80211_input.h>
#include <net80211/ieee80211_11k.h>
#ifdef IEEE80211K

static	ieee80211_send_action_func rm_send_action_neigh_rpt;
static	ieee80211_send_action_func rm_send_action_link_meas;
static	ieee80211_send_action_func rm_send_action_radio_meas_req;
static	ieee80211_send_action_func rm_send_action_radio_meas_rpt;

static  ieee80211_recv_action_func rm_recv_action_radio_meas;
static	ieee80211_recv_action_func rm_recv_action_neigh_rpt;
static	ieee80211_recv_action_func rm_recv_action_link_meas;
static	ieee80211_recv_action_func sm_recv_action_meas;

uint8_t *ieee80211_add_rm_capab(uint8_t *frm)
{
    frm[0] = IEEE80211_ELEMID_RM_CAPAB;
    frm[1] = 7;
    frm[2] = 0x00;
    frm[3] = (IEEE80211K_FRAME_MEAS | IEEE80211K_NOISE_HIST_MEAS | IEEE80211K_CHNL_LOAD_MEAS);
    frm[4] = 0x00;
    frm[5] = 0x00;
    frm[6] = 0x00;
    frm[7] = 0x00;
    frm[8] = 0x00;

	return frm + sizeof(struct ieee80211_ie_rm_capab);
}

static int rm_action_output( struct ieee80211_node *ni, struct mbuf *m)
{

	struct ieee80211_bpf_params params;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = vap->iv_ic;
	int type = IEEE80211_FC0_SUBTYPE_ACTION;
	struct ieee80211_frame *wh;

	memset(&params, 0, sizeof(params));
	params.ibp_pri = WME_AC_VO;
	params.ibp_rate0 = ni->ni_txparms->mgmtrate;
	/* NB: we know all frames are unicast */
	params.ibp_try0 = ni->ni_txparms->maxretry;
	params.ibp_power = ni->ni_txpower;
//	return ieee80211_mgmt_output(ni, m, IEEE80211_FC0_SUBTYPE_ACTION,
//	     &params);
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
//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s:%d\n"), __func__,__LINE__));	
	if(vap->hal_priv_vap->params_11k_set == 1)
	{
		ieee80211_send_setup(ni, m,
	     	IEEE80211_FC0_TYPE_MGT | type, IEEE80211_NONQOS_TID,
	     		vap->iv_myaddr, vap->hal_priv_vap->msrmnt_req.ucast_macaddr, ni->ni_bssid);
	}
	else
	{
			ieee80211_send_setup(ni, m,
	     			IEEE80211_FC0_TYPE_MGT | type, IEEE80211_NONQOS_TID,
	     				vap->iv_myaddr,ni->ni_macaddr, ni->ni_bssid);

	}

 	vap->hal_priv_vap->params_11k_set = 0;				
 
  
  m->m_flags |= M_ENCAP;		/* mark encapsulated */

	M_WME_SETAC(m, params.ibp_pri);

	IEEE80211_NODE_STAT(ni, tx_mgmt);
	
	return ic->ic_raw_xmit(ni, m, &params);
}

void ieee80211_rm_attach(struct ieee80211com *ic)
{
	ieee80211_send_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_RADIO_MEAS_REQ, rm_send_action_radio_meas_req);

	ieee80211_send_action_register(IEEE80211_ACTION_CAT_SM,IEEE80211_ACTION_SM_MEAS_REQ,rm_send_action_radio_meas_req);
	
	ieee80211_send_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_RADIO_MEAS_RPT, rm_send_action_radio_meas_rpt);
	
	ieee80211_send_action_register(IEEE80211_ACTION_CAT_SM,IEEE80211_ACTION_SM_MEAS_RPT, rm_send_action_radio_meas_rpt);

	ieee80211_send_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_LINK_MEAS_REQ, rm_send_action_link_meas);
	
	ieee80211_send_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_LINK_MEAS_REP, rm_send_action_link_meas);

	ieee80211_send_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_NG_RPT_REQ, rm_send_action_neigh_rpt);
	
	ieee80211_send_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_NG_RPT_RSP, rm_send_action_neigh_rpt);

	
	ieee80211_recv_action_register(IEEE80211_ACTION_CAT_SM,IEEE80211_ACTION_SM_MEAS_REQ,sm_recv_action_meas);
	

	ieee80211_recv_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_RADIO_MEAS_REQ, rm_recv_action_radio_meas);
	
	ieee80211_recv_action_register(IEEE80211_ACTION_CAT_SM,IEEE80211_ACTION_SM_MEAS_RPT,sm_recv_action_meas);

	ieee80211_recv_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_RADIO_MEAS_RPT, rm_recv_action_radio_meas);

	ieee80211_recv_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_LINK_MEAS_REQ, rm_recv_action_link_meas);
	
	ieee80211_recv_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_LINK_MEAS_REP, rm_recv_action_link_meas);

	ieee80211_recv_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_NG_RPT_REQ, rm_recv_action_neigh_rpt);
	
	ieee80211_recv_action_register(IEEE80211_ACTION_RADIO_MEAS, 
	    IEEE80211_ACTION_NG_RPT_RSP, rm_recv_action_neigh_rpt);
	
}
/*This Function is invoked through iwpriv ioctl call for radio measuremnt request, parse the arguments gievn through ioctl
 * @param1 : vap pointer 
 * @param2 : pointer to arguments provided in ioctl call
 * */
int ieee80211_11k_action(struct ieee80211vap *vap, void *data )
{
	struct ieee80211_node *ni = NULL;	
	struct ieee80211com *ic = NULL;
	uint8_t args[64];//i=0;
	static int tokens = 0;	/* XXX */
	uint8_t frame_type;
	struct radio_meas_req meas_req;
	uint32_t *data1;

	data1 = (uint32_t *)data;
 
	ni = ieee80211_find_txnode(vap, vap->iv_bss->ni_bssid);

	if( ni == NULL )
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s and %d  ni is NULL \n", __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}

	frame_type = *(uint32_t *)data;
	meas_req.channel_num = *(uint32_t *)(data + 4);
	meas_req.operating_class = *(uint32_t *)(data + 8);
	meas_req.meas_duration = *(uint32_t *)(data+ 12);
	meas_req.rand_int = 1000;
	
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s: Frame type    = %d \n", __func__,frame_type));
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s  channel NUm   = %d \n", __func__,meas_req.channel_num));
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s  oper_class    = %d \n", __func__,meas_req.operating_class));
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s  meas_duration = %d \n", __func__,meas_req.meas_duration));
	
	if(vap->hal_priv_vap->params_11k_set  == 1)
	{
		if(vap->hal_priv_vap->msrmnt_req.frame_type == frame_type)
		{
			if( frame_type == IEEE80211_CHANNEL_LOAD )
			{
				memcpy(&meas_req.unicast_mac_addr, &vap->hal_priv_vap->msrmnt_req.ucast_macaddr, ETHER_ADDR_LEN);
			}
			else if( frame_type == IEEE80211_FRAME )
			{
				memcpy(&meas_req.unicast_mac_addr, &vap->hal_priv_vap->msrmnt_req.ucast_macaddr, ETHER_ADDR_LEN);
				memcpy(&meas_req.bssid, &vap->hal_priv_vap->msrmnt_req.frame_req_macaddr, ETHER_ADDR_LEN);
			
			}
			else if( frame_type == IEEE80211_BEACON )
			{
				meas_req.meas_mode = *(uint32_t *)(data+ 16);
				memcpy(&meas_req.unicast_mac_addr, &vap->hal_priv_vap->msrmnt_req.ucast_macaddr, ETHER_ADDR_LEN);
				memcpy(&meas_req.mac_addr, &vap->hal_priv_vap->msrmnt_req.bssid_macaddr, ETHER_ADDR_LEN);
				meas_req.bse.ssid_ie_len = vap->hal_priv_vap->msrmnt_req.ssid_len;
				if(vap->hal_priv_vap->msrmnt_req.ssid_len)
				memcpy(&meas_req.bse.ssid_ie, &vap->hal_priv_vap->msrmnt_req.ssid_beacon_rpt, vap->hal_priv_vap->msrmnt_req.ssid_len);
			}
				
			else if(frame_type == IEEE80211_MULTICAST_DIG_REQ)
			{
				memcpy(&meas_req.unicast_mac_addr, &vap->hal_priv_vap->msrmnt_req.ucast_macaddr, ETHER_ADDR_LEN);
				memcpy(&meas_req.bssid, &vap->hal_priv_vap->msrmnt_req.frame_req_macaddr, ETHER_ADDR_LEN);
			}
						
			else
			{
	   IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Provide supported type of frame request\n", __func__));
				return ONEBOX_STATUS_FAILURE;
			}
					
		}
		else	
		{
	   IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:please provide same frame type input in onebox_util and iwpriv commands\n", __func__));
			return ONEBOX_STATUS_FAILURE;
		}
	}
	else
	{
	   IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Please provide required params through onebox_util ioctl\n", __func__));
			 return ONEBOX_STATUS_FAILURE;
	}

	args[0] = (tokens+1) % 63;
	args[1] = frame_type;
	memcpy(&args[2], &meas_req, sizeof(struct radio_meas_req));
	//memcpy(&args[2], &meas_req, 6);
	ic = ni->ni_ic;
	
	ic->ic_send_action(ni, IEEE80211_ACTION_RADIO_MEAS, IEEE80211_ACTION_RADIO_MEAS_REQ, args);
	return ONEBOX_STATUS_SUCCESS;
}

/*This Function is invoked through iwpriv ioctl call for spectral measuremnt request, parse the arguments gievn through ioctl
 * @param1 : vap pointer 
 * @param2 : pointer to arguments provided in ioctl call
 * */
int ieee80211_11k_sm_action(struct ieee80211vap *vap, void *data )
{
	struct ieee80211_node *ni = NULL;	
	struct ieee80211com *ic = NULL;
	uint8_t args[14];
	static int tokens = 0;	/* XXX */
	uint8_t frame_type;
	struct sm_meas_req meas_req;
	uint32_t meas_start_time_os=0;

	ni = ieee80211_find_txnode(vap, vap->iv_bss->ni_bssid);
	if( ni == NULL )
	{
	 IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:NI is NULL\n", __func__));
		return ONEBOX_STATUS_FAILURE;
	}

	/*Parameters from IOCTL */
	frame_type = *(uint32_t *)data;
	meas_req.chan_num = *(uint32_t *)(data + 4);
	meas_req.msrmnt_dur = *(uint32_t *)(data + 8);
	meas_start_time_os = *(uint32_t *)(data+ 12);
	args[0] = (tokens+1) % 63;
	args[1] = frame_type;
	
	 args[2] = 0;
 	meas_req.msrmnt_start_time = meas_start_time_os;	//Need to implement by maintain AP TSF.
	 IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s: frame type    = %d \n", __func__,frame_type));
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s  channel num   = %d \n", __func__,meas_req.chan_num));
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s  meas_duration    = %d \n", __func__,meas_req.msrmnt_dur));
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s  msrmnt_start_time = %d \n", __func__,meas_req.msrmnt_start_time));
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s  meas_mode = %d \n", __func__,args[2]));
	memcpy(&args[3], &meas_req,11);
	ic = ni->ni_ic;
	ic->ic_send_action(ni,IEEE80211_ACTION_CAT_SM,IEEE80211_ACTION_RADIO_MEAS_REQ,args);
	return ONEBOX_STATUS_SUCCESS;
}

static int rm_send_action_neigh_rpt(struct ieee80211_node *ni,
	int category, int action, void *arg0)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	uint16_t *args = arg0;
	struct mbuf *m;
	uint8_t *frm;
	
	ieee80211_ref_node(ni);
	
	m = ieee80211_getmgtframe(&frm,
	    ic->ic_headroom + sizeof(struct ieee80211_frame),
	    sizeof(uint16_t)	/* action+category */
	    + 1 /*dialog token */
		+ sizeof(uint16_t)	/* SSID+LENGTH */
		+ vap->iv_bss->ni_esslen
	);

	if (m != NULL) {
		*frm++ = category;
		*frm++ = action;
		*frm++ = args[0];		/* dialog token */
		*frm++ = IEEE80211_ELEMID_SSID;
		*frm++ = vap->iv_bss->ni_esslen;
		memcpy(frm, &vap->iv_bss->ni_essid[0], vap->iv_bss->ni_esslen);
		frm += vap->iv_bss->ni_esslen;
		m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
		return rm_action_output(ni, m);
	} else {
		vap->iv_stats.is_tx_nobuf++;
		ieee80211_free_node(ni);
		return ENOMEM;
	}
}

static int rm_send_action_link_meas(struct ieee80211_node *ni,
	int category, int action, void *arg0)
{
	return ONEBOX_STATUS_SUCCESS;
}

/*This Function is used to differentiate the received measurment request type and calls appropriate function accordingly
 * @param1 : node ni
 * @params2 : category,
 * @params3 : Action
 * @params4 arguments passed 
 * */
static int rm_send_action_radio_meas_req(struct ieee80211_node *ni,
	int category, int action, void *arg0)										
{
	uint8_t *args = arg0;
	uint8_t frame_type;
  struct ieee80211vap *vap = ni->ni_vap;
	
	frame_type = args[1];
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s and %d  frame_type = %d \n", __func__, __LINE__,frame_type));

	switch(frame_type)
	{
		case IEEE80211_BASIC_MEAS:
		case IEEE80211_CCA_MEAS:
		case IEEE80211_RPI_HIST_MEAS:
	  IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s: Sending SM measurement request\n", __func__));
			send_sm_meas_req(ni, category, action, args);
			break;
		case IEEE80211_CHANNEL_LOAD:
	  IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s: Sending channel load request\n", __func__));
			send_channel_load_req(ni, category, action, args);
			break;
		case IEEE80211_NOISE_HIST:
			send_noise_hist_req(ni, category, action, args); 
			break;
		case IEEE80211_FRAME:
	  IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s: Sending Frame Request\n", __func__));
			send_frame_req(ni, category, action, args); 
			break;
		case IEEE80211_BEACON:
	  IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s: Sending Beacon Request\n", __func__));
			send_beacon_req(ni, category, action, args); 
			break;	
		case IEEE80211_MULTICAST_DIG_REQ:
	  IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s: Sendind Multicast Diagnostics Request\n", __func__));
			send_multicat_dig_req(ni,category,action,args);
		break;

	}

	return ONEBOX_STATUS_SUCCESS;
}

static int rm_send_action_radio_meas_rpt(struct ieee80211_node *ni,
	int category, int action, void *arg0)
{
	struct rm_element meas_rpt;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct channel_load_rpt ch_rpt; 
	struct beacon_report beacon_rpt;
	struct frame_report fr_rpt;
	struct multicast_diagnostics_report mcast_report; 
	struct sm_cca_rept cca_rpt;
	struct sm_rpi_hist_rept rpi_rpt;
	uint8_t *args = arg0;
	struct mbuf *m;
	uint8_t *frm;
	uint8_t measurement_type;
	uint16_t no_of_frames;

	if(category)
	{
		measurement_type = ic->meas_info[0].meas_req.type;
	}
	else
	{
		measurement_type = ic->sm_meas_info[0].meas_req.type;
	}
	ieee80211_ref_node(ni);
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Measurement received \n", __func__));
	if( measurement_type == IEEE80211_NOISE_HIST ) {
			/* This code need to be rewritten based on inputs from F/W. */
			} 
			else if (measurement_type == IEEE80211_CCA_MEAS)
	{
			m = ieee80211_getmgtframe(&frm,
				ic->ic_headroom + sizeof(struct ieee80211_frame),
				sizeof(uint16_t)	/* action+category */
				+ 1 /*dialog token */
				+ sizeof(struct rm_element)
				+ sizeof(struct sm_cca_rept)
				);
			meas_rpt.element_id  = IEEE80211_ELEMID_MEASREP;
			meas_rpt.length	   = 3 + sizeof(struct sm_cca_rept);
			meas_rpt.token 	   = 1;
			meas_rpt.mode 	   = args[6];
			meas_rpt.type 	   = measurement_type; 
			if (m != NULL) {
			*frm++ = category;
			*frm++ = action;
			*frm++ = ic->sm_meas_info[0].dialogue_token;		/* dialog token */
			memcpy(frm, &meas_rpt, sizeof(struct rm_element));
			frm += sizeof(struct rm_element);
			cca_rpt.rpt.chan_num = ic->sm_meas_info[0].req.chan_num;
			cca_rpt.cca_busy_fraction = args[8];
			memcpy(&cca_rpt.rpt.msrmnt_dur, &args[10], 2);
			memcpy(&cca_rpt.rpt.msrmnt_start_time, &args[16], 8);

			if( (meas_rpt.mode >> 1 & 1) || (meas_rpt.mode >> 2 & 1)) {
	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Setting incable/refusal bit in CCA report\n", __func__));
					cca_rpt.rpt.msrmnt_start_time = 0;
			}
			memcpy(frm, &cca_rpt, sizeof(struct sm_cca_rept));
			frm += sizeof(struct sm_cca_rept);
			m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
			return rm_action_output(ni, m);
		} else {
			goto fail;
		}
	}
	else if (measurement_type == IEEE80211_RPI_HIST_MEAS)
	{
			m = ieee80211_getmgtframe(&frm,
				ic->ic_headroom + sizeof(struct ieee80211_frame),
				sizeof(uint16_t)	/* action+category */
				+ 1 /*dialog token */
				+ sizeof(struct rm_element)
				+ sizeof(struct sm_rpi_hist_rept)
				);
			meas_rpt.element_id  = IEEE80211_ELEMID_MEASREP;
			meas_rpt.length	   = 3 + sizeof(struct sm_cca_rept);
			meas_rpt.token 	   = 1;
			meas_rpt.mode 	   = args[6];
			meas_rpt.type 	   = measurement_type; 
			if (m != NULL) {
			*frm++ = category;
			*frm++ = action;
			*frm++ = ic->sm_meas_info[0].dialogue_token;		/* dialog token */
			memcpy(frm, &meas_rpt, sizeof(struct rm_element));
			frm += sizeof(struct rm_element);
			rpi_rpt.rpt.chan_num = ic->sm_meas_info[0].req.chan_num;
			memcpy(&rpi_rpt.rpt.msrmnt_dur, &args[10], 2);
			memcpy(&rpi_rpt.rpi_vals[0],&args[16],8);
			memcpy(&rpi_rpt.rpt.msrmnt_start_time, &args[24], 8);

			if( (meas_rpt.mode >> 1 & 1) || (meas_rpt.mode >> 2 & 1)) {
	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Setting incable/refusal bit in RPI report\n", __func__));
					rpi_rpt.rpt.msrmnt_start_time = 0;
			}
			memcpy(frm, &rpi_rpt, sizeof(struct sm_rpi_hist_rept));
			frm += sizeof(struct sm_rpi_hist_rept);
			m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
			return rm_action_output(ni, m);
		} else {
			goto fail;
		}	
	}

	else if (measurement_type == IEEE80211_CHANNEL_LOAD ) {
		m = ieee80211_getmgtframe(&frm,
				ic->ic_headroom + sizeof(struct ieee80211_frame),
				sizeof(uint16_t)	/* action+category */
				+ 1 /*dialog token */
				+ sizeof(struct rm_element)
				+ sizeof(struct channel_load_rpt)
				);

		meas_rpt.element_id  = IEEE80211_ELEMID_MEASREP;
		meas_rpt.length	   = 3 + sizeof(struct channel_load_rpt);
		meas_rpt.token 	   = 1;
		meas_rpt.mode 	   = args[6];
		meas_rpt.type 	   = IEEE80211_CHANNEL_LOAD;

		if (m != NULL) {
			*frm++ = category;
			*frm++ = action;
			*frm++ = ic->meas_info[0].dialogue_token;		/* dialog token */

			memcpy(frm, &meas_rpt, sizeof(struct rm_element));
			frm += sizeof(struct rm_element);
			ch_rpt.operating_class = ic->meas_info[0].req.operating_class;
			ch_rpt.channel_num 		 = ic->meas_info[0].req.channel_num;
			ch_rpt.channel_load		 = args[8];
			memcpy(&ch_rpt.tsf_time, &args[16], 8);
			memcpy(&ch_rpt.meas_duration, &args[10], 2);
			if( (meas_rpt.mode >> 1 & 1) || (meas_rpt.mode >> 2 & 1)) {
	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Setting incable/refusal bit in channel load report\n", __func__));
					ch_rpt.tsf_time[0] = 0;
					ch_rpt.tsf_time[1] = 0;
			}
							
			memcpy(frm, &ch_rpt, sizeof(struct channel_load_rpt));
			frm += sizeof(struct channel_load_rpt);

			m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
			return rm_action_output(ni, m);
		} else {
			goto fail;
		}

	}else if ( measurement_type == IEEE80211_FRAME ) {
			/*action frame allocation */
		m = ieee80211_getmgtframe(&frm,
				ic->ic_headroom + sizeof(struct ieee80211_frame),
				sizeof(uint16_t)	/* action+category */
				+ 1 /*dialog token */
				+ sizeof(struct rm_element)
				+ sizeof(struct frame_report)
				);

		meas_rpt.element_id  = IEEE80211_ELEMID_MEASREP;
		meas_rpt.length	   = 3 + sizeof(struct frame_report);
		meas_rpt.token 	   = 1;
		meas_rpt.mode 	   = args[6];
		meas_rpt.type 	   = IEEE80211_FRAME;

		if (m != NULL) {
			*frm++ = category;
			*frm++ = action;
			*frm++ = ic->meas_info[0].dialogue_token;		/* dialog token */

			memcpy(frm, &meas_rpt, sizeof(struct rm_element));
			frm += sizeof(struct rm_element);
			fr_rpt.operating_class = ic->meas_info[0].req.operating_class;
			fr_rpt.channel_num 		 = ic->meas_info[0].req.channel_num;
			memcpy(&no_of_frames, &args[8], 2); 

			/* if no of frames is more than 1, 
			 * then it should be taken care 
			 * in action frame allocation 
			 */

			memcpy(&fr_rpt.tsf_time, &args[36], 8);
			memcpy(&fr_rpt.meas_duration, &args[10], 2);

			fr_rpt.elem_id = 1;
			fr_rpt.length = (19 * no_of_frames);
			meas_rpt.length = 15 + 2 + fr_rpt.length ; 
			/*15 = 3 + intial frame report, 2 = sub_ele_id + sub_ele_len, then frame_report_entry */

			memcpy(&fr_rpt.tx_addr[0], &args[24], 2);
			memcpy(&fr_rpt.tx_addr[2], &args[26], 2);
			memcpy(&fr_rpt.tx_addr[4], &args[28], 2);
			memcpy(&fr_rpt.bssid[0], &args[30], 2);
			memcpy(&fr_rpt.bssid[2], &args[32], 2);
			memcpy(&fr_rpt.bssid[4], &args[34], 2);
			fr_rpt.phy_type = args[16];
			fr_rpt.avg_rcpi = args[18];
			fr_rpt.last_rcpi = args[19];
			fr_rpt.ant_id = args[17];
			fr_rpt.last_rsni = args[20];
			memcpy(&fr_rpt.frame_count, &args[22], 2);

			memcpy(frm, &fr_rpt, sizeof(struct frame_report));
			
			//frm += fr_rpt.length- 3;
			frm += sizeof(struct frame_report); 
		
			m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
			return rm_action_output(ni, m);
		}
			 else {
			goto fail;
		}
	}
	
	#if 1	
	else if ( measurement_type == IEEE80211_MULTICAST_DIG_REQ ) {
		
					/*action frame allocation */
		m = ieee80211_getmgtframe(&frm,
				ic->ic_headroom + sizeof(struct ieee80211_frame),
				sizeof(uint16_t)	/* action+category */
				+ 1 /*dialog token */
				+ sizeof(struct rm_element)
				+ sizeof(struct multicast_diagnostics_report)
				);
		meas_rpt.element_id  = IEEE80211_ELEMID_MEASREP;
		meas_rpt.length	   = 3 + sizeof(struct multicast_diagnostics_report );
		meas_rpt.token 	   = 1;
		meas_rpt.mode 	   = args[6];
		meas_rpt.type 	   = IEEE80211_MULTICAST_DIG_REQ;
		if (m != NULL) {
				*frm++ = category;
				*frm++ = action;
				*frm++ = ic->meas_info[0].dialogue_token;		/* dialog token */
				memcpy(frm, &meas_rpt, sizeof(struct rm_element));	
				frm += sizeof(struct rm_element);
				memcpy(mcast_report.measurement_time,&args[7],8);
         //for(ii = 0; ii <= 7 ; ii++)
         
				memcpy(mcast_report.meas_duration,&args[7],8);
				memcpy(mcast_report.group_mac,&args[15],ETHER_ADDR_LEN);	
			  mcast_report.mcast_report_reason = args[21] ; //mcast report reason
				
        mcast_report.mcast_pkt_count = (uint8_t) args[22];
        mcast_report.mcast_pkt_count = (uint8_t) args[23] >> 8;
				mcast_report.first_seq_num = args[24] ;
				mcast_report.last_seq_num = args[25] ;
        
        mcast_report.mcast_high_rate = (uint8_t) args[26];
        mcast_report.mcast_high_rate = (uint8_t) args[27] >> 8;
      
         memcpy(frm, &mcast_report, sizeof(struct multicast_diagnostics_report ));
					frm += sizeof(struct multicast_diagnostics_report );

					m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
					return rm_action_output(ni, m);


		}
	}
	#endif
	 else if(measurement_type == IEEE80211_BEACON ) {
			m = ieee80211_getmgtframe(&frm,
							ic->ic_headroom + sizeof(struct ieee80211_frame),
							sizeof(uint16_t)	/* action+category */
							+ 1 /*dialog token */
							+ sizeof(struct rm_element)
							+ sizeof(struct beacon_report)
							);
			meas_rpt.element_id  = IEEE80211_ELEMID_MEASREP;
			meas_rpt.length	   = 3 + sizeof(struct beacon_report);
			meas_rpt.token 	   = 1;
			meas_rpt.mode 	   = args[6];
			meas_rpt.type 	   = IEEE80211_BEACON;

			if (m != NULL) {
					*frm++ = category;
					*frm++ = action;
					*frm++ = ic->meas_info[0].dialogue_token;		/* dialog token */

					memcpy(frm, &meas_rpt, sizeof(struct rm_element));
					frm += sizeof(struct rm_element);
					beacon_rpt.operating_class = ic->meas_info[0].req.operating_class;
					beacon_rpt.channel_num 		 = ic->meas_info[0].req.channel_num;
					memcpy(&beacon_rpt.bssid, &ic->meas_info[0].req.bssid[0], 6); 
					memcpy(&beacon_rpt.tsf_time, &args[24], 8);
					memcpy(&beacon_rpt.meas_duration, &args[10], 2);

					beacon_rpt.rpt_frame_info = args[16];
					beacon_rpt.rcpi = args[18];
					beacon_rpt.rsni = args[19];
					beacon_rpt.ant_id = args[17];
					memcpy(&beacon_rpt.parent_tsf, &args[20], 4);

					memcpy(frm, &beacon_rpt, sizeof(struct beacon_report));
					frm += sizeof(struct beacon_report);

					m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
					return rm_action_output(ni, m);
			} else {
					goto fail;
			}
	}
fail:
		vap->iv_stats.is_tx_nobuf++;
		ieee80211_free_node(ni);
		return ENOMEM;
}


static int
sm_recv_action_meas(struct ieee80211_node *ni,
	const struct ieee80211_frame *wh,
	const uint8_t *frm, const uint8_t *efrm)
{
	struct ieee80211com *ic = ni->ni_ic;
  struct ieee80211vap *vap = ni->ni_vap;
	
 	if ( frm[1] == IEEE80211_ACTION_RADIO_MEAS_REQ ) {

		/* Storing the measurement details in sm_meas_info[0], 
		 * in future when parallel measurements are supported
		 * measurements can be stored in different indices
		 */

		ic->sm_meas_info[0].dialogue_token = frm[2];
		memcpy(&ic->sm_meas_info[0].meas_req, &frm[3], sizeof(struct rm_element));
		ic->sm_meas_info[0].ni = ni;

		if ( ic->sm_meas_info[0].meas_req.element_id == IEEE80211_ELEMID_MEASREQ ) {
			switch ( ic->sm_meas_info[0].meas_req.type ) {
				case 	IEEE80211_CCA_MEAS:
				case	IEEE80211_RPI_HIST_MEAS:
				case	IEEE80211_BASIC_MEAS:
	        IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received Spectrum management Measurement request of type %d\n", __func__,ic->sm_meas_info[0].meas_req.type));
					memcpy(&ic->sm_meas_info[0].req, &frm[8],11); /* opertaing class, ch_num, rand_int, meas_duration */
					break;
				default :
	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Invalid Measurement request\n", __func__));
					return ONEBOX_STATUS_FAILURE;
			}
			ic->ic_send_meas_info((uint8_t *)&ic->sm_meas_info[0],ic->sm_meas_info[0].meas_req.type );
		}
	} else if ( frm[1] == IEEE80211_ACTION_RADIO_MEAS_RPT ) {
		
		if(ni->meas_rpt_element.mode != 0)				//Checking if refused or incapable response is received
		{
	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Refused/Incapable Report received\n", __func__));
		}
		else if( ic->sm_meas_info[0].dialogue_token == frm[2] )
		{
			memcpy(&ni->meas_rpt_element, &frm[3], sizeof(struct rm_element));
			if( ic->sm_meas_info[0].measurement_type == ni->meas_rpt_element.type)
			{
				switch(ic->sm_meas_info[0].measurement_type)
				{
					case IEEE80211_CCA_MEAS:
	     IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received cca meas report\n", __func__));
						memcpy(&ni->cca_load_rpt, &frm[8], sizeof(struct sm_cca_rept));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:measurment start time in Hex:%x\n", __func__,ni->cca_load_rpt.rpt.msrmnt_start_time));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:measurment duration:%d\n", __func__,ni->cca_load_rpt.rpt.msrmnt_dur));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:CCA busy fraction:%d\n", __func__,ni->cca_load_rpt.cca_busy_fraction));
						break;
					case IEEE80211_RPI_HIST_MEAS:
						{
							uint8_t i=0;
	            IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received RPI Histogram meas report\n", __func__));
							memcpy(&ni->rpi_hist_rpt, &frm[8], sizeof(struct sm_rpi_hist_rept));
	            IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:measurment duration:%d\n\n", __func__,ni->rpi_hist_rpt.rpt.msrmnt_dur));
	            IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:measurment start time in Hex:%x\n", __func__,ni->rpi_hist_rpt.rpt.msrmnt_start_time));
							for(i=0;i<8;i++)
							{
	              IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:RPI Hostogram bin%d - %d\n\n", __func__,i,ni->rpi_hist_rpt.rpi_vals[i]));
							}
							break;
						}

				}
				ni->rm_rpt.type = ic->sm_meas_info[0].measurement_type;
				ni->rm_rpt.received_rpt = 1;
			}
			else
			{
	     IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Invalid Report received\n", __func__));
			}
		}
		else
		{
	     IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Invalid Dialogue token\n", __func__));
			return ONEBOX_STATUS_FAILURE;
		}
	}
	return ONEBOX_STATUS_SUCCESS;
}


static int
rm_recv_action_radio_meas(struct ieee80211_node *ni,
	const struct ieee80211_frame *wh,
	const uint8_t *frm, const uint8_t *efrm)
{

	struct ieee80211com *ic = ni->ni_ic;
	uint16_t remain_len = 0;
	uint16_t optional_elem_len = 0;
	uint16_t optional_elem_id = 0;
  struct ieee80211vap *vap = ni->ni_vap;

	if ( frm[1] == IEEE80211_ACTION_RADIO_MEAS_REQ ) {
		/* Storing the measurement details in meas_info[0], 
		 * in future when parallel measurements are supported
		 * measurements can be stored in different indices
		 */
		ic->meas_info[0].dialogue_token = frm[2];
		ic->meas_info[0].repetitions = LE_READ_2(frm+3);
		memcpy(&ic->meas_info[0].meas_req, &frm[5], sizeof(struct rm_element));
		ic->meas_info[0].ni = ni;

		if ( ic->meas_info[0].meas_req.element_id == IEEE80211_ELEMID_MEASREQ ) {
			switch ( ic->meas_info[0].meas_req.type ) {
				case IEEE80211_CHANNEL_LOAD:
					memcpy(&ic->meas_info[0].req, &frm[10], 6); /* opertaing class, ch_num, rand_int, meas_duration */
					if(ic->meas_info[0].meas_req.length > SINGLE_CHANNEL_LOAD_LEN){
							/* Add here for multiple requests 
							 * in single frame or optional sub elements  
							 */
					}
	     IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received channel load request\n", __func__));
					break;
			//	case IEEE80211_NOISE_HIST:
			//		break;
				case IEEE80211_FRAME:
					memcpy(&ic->meas_info[0].req, &frm[10], 6 + 7);/* opertaing class, ch_num, rand_int, meas_duration, mac_addr, mode */
					if(ic->meas_info[0].meas_req.length > SINGLE_FRAME_REQ_LEN){
							/* Add here for multiple requests 
							 * in single frame or optional sub elements  
							 */
					}
	     IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received Frame request\n", __func__));
					break;
				case IEEE80211_BEACON:
					memcpy(&ic->meas_info[0].req, &frm[10], 6 + 7); /* opertaing class, ch_num, rand_int, meas_duration, mac_addr, mode */   
					if(ic->meas_info[0].meas_req.length > SINGLE_BEACON_REQ_LEN){
							/* Checking for Optional
							 * subelements
							 */
							remain_len = ic->meas_info[0].meas_req.length - SINGLE_BEACON_REQ_LEN; 
							frm += 10 + 6 + 7; 
							while( remain_len ) {
									optional_elem_id = *frm++;
									optional_elem_len = *frm++;
									switch( optional_elem_id ) {
											case SSID_ELEM_ID:
	            IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Beacon request has SSID as an optional element\n", __func__));
													memcpy(&ic->meas_info[0].req.bse.ssid_ie[0], frm, optional_elem_len); 
													break;
											case BEACON_REPORT_INFO:
	            IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Beacon request has beacon report info as an optional element\n", __func__));
													memcpy(&ic->meas_info[0].req.bse.bcn_rpt_info[0], frm, optional_elem_len); 
													break;
											case REPORTING_DETAIL:
	            IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Beacon request has Reporting detail as an optional element\n", __func__));
													ic->meas_info[0].req.bse.rpt_detail = *frm;
													break;
											case AP_CHANNEL_RPT:
	            IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:AP channel report sub element is not supported\n", __func__));
													break;
											case VENDOR_SPECIFIC:
	            IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Vendor specific sub element is not supported\n", __func__));
													break;
											default:
	                    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Sub element ID %d is not supported\n\n", __func__,optional_elem_id));
													break;
									}
									frm += optional_elem_len;
									remain_len -= 2 + optional_elem_len;
							}
					}
	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received beacon request\n", __func__));
					break;
				case IEEE80211_MULTICAST_DIG_REQ:
					memcpy(&ic->meas_info[0].req.rand_int, &frm[10], 4); /* opertaing class, ch_num, rand_int, meas_duration, mac_addr, mode */   
					memcpy(&ic->meas_info[0].req.bssid, &frm[14], 6); /* opertaing class, ch_num, rand_int, meas_duration, mac_addr, mode */  
						
					if(ic->meas_info[0].meas_req.length > SINGLE_MULTICAST_DIAGNOSTICS_REQ_LEN){
							/* Add here for multiple requests 
							 * in single frame or optional sub elements  
							 */
							

					}
	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received Multicat request\n", __func__));
					break;
				default :
	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Invalid Measurement request\n", __func__));
					return ONEBOX_STATUS_FAILURE;
			}
			ic->ic_send_meas_info((uint8_t *)&ic->meas_info[0], ic->meas_info[0].meas_req.type  );
		}
	} else if ( frm[1] == IEEE80211_ACTION_RADIO_MEAS_RPT ) {
		
		if( ic->meas_info[0].dialogue_token == frm[2] )
		{
			
			memcpy(&ni->meas_rpt_element, &frm[3], sizeof(struct rm_element));
			
      if(ni->meas_rpt_element.mode != 0)				//Checking if refused or incapable response is received
			{
	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Refused/Incapable Report received\n", __func__));
			}
			else if( ic->meas_info[0].measurement_type == ni->meas_rpt_element.type)
			{
			  switch(ic->meas_info[0].measurement_type)
				{
					case IEEE80211_CHANNEL_LOAD:
	     IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received channel load report\n", __func__));
						memcpy(&ni->channel_rpt, &frm[8], sizeof(struct channel_load_rpt));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:OPERRATING_CLASS : %d\n", __func__,ni->channel_rpt.operating_class));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:CAHNNEL_NUM : %d\n", __func__,ni->channel_rpt.channel_num));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:OPERRATING_CLASS : %d\n", __func__,(ni->channel_rpt.tsf_time[0] | (ni->channel_rpt.tsf_time[1]) << 8)));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:TSF_TIME : %d", __func__,ni->channel_rpt.operating_class));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:CAHNNEL_LOAD : %d\n", __func__,ni->channel_rpt.channel_load));
						break;
					case IEEE80211_NOISE_HIST:
	     IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received Noise Histogram report\n", __func__));
						memcpy(&ni->noise_hist_rpt, &frm[8], sizeof(struct noise_hist_rpt));
						break;
					case IEEE80211_FRAME:
	     IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received frame report\n", __func__));
						memcpy(&ni->frame_rpt, &frm[8], sizeof(struct frame_report));
            IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:OPERRATING_CLASS : %d\n", __func__,ni->frame_rpt.operating_class));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:CAHNNEL_NUM : %d\n", __func__,ni->frame_rpt.channel_num));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:TSF_TIME : %d", __func__,ni->frame_rpt.tsf_time));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:MESUREMENT_DURATION : %ld\n", __func__,ni->frame_rpt.meas_duration));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:LENGTH : %d\n", __func__,ni->frame_rpt.length));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("PHY_TYPE : %d\n",ni->frame_rpt.phy_type));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("AVG_RCPI : %d\n",ni->frame_rpt.avg_rcpi));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("LAST_RSNI : %d\n",ni->frame_rpt.last_rsni));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("LAST_RCPI : %d\n",ni->frame_rpt.last_rcpi));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("ANT_ID : %d\n",ni->frame_rpt.ant_id));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("FRAME_COUNT : %d\n",ni->frame_rpt.frame_count));
						break;
					case IEEE80211_BEACON:
						memcpy(&ni->beacon_rpt, &frm[8], sizeof(struct frame_report));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("OPERRATING_CLASS : %d\n",ni->beacon_rpt.operating_class));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("CAHNNEL_NUM : %d\n",ni->beacon_rpt.channel_num));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("TSF_TIME : %ld\n",ni->beacon_rpt.tsf_time));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("MESUREMENT_DURATION : %ld\n",ni->beacon_rpt.meas_duration));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("RPT_FRAME_INFO : %d\n",ni->beacon_rpt.rpt_frame_info));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("RCPI : %d\n",ni->beacon_rpt.rcpi));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("RSNI : %d\n",ni->beacon_rpt.rsni));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("ANT_ID : %d\n",ni->beacon_rpt.ant_id));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("PARENT_TSF : %ld\n",ni->beacon_rpt.parent_tsf));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received valid beacon report\n", __func__));
						break;
          case IEEE80211_MULTICAST_DIG_REQ: 
						memcpy(&ni->mcast_report, &frm[8], sizeof(struct frame_report));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("Multicast Report Reason  : %d\n",ni->mcast_report.mcast_report_reason));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("Multicast Packet Count : %ld\n",ni->mcast_report.mcast_pkt_count));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("First Seq No : %d\n",ni->mcast_report.first_seq_num));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("First Last No : %d\n",ni->mcast_report.last_seq_num));
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("First Highest Multicast Rate : %d\n",ni->mcast_report.mcast_high_rate));
	          IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received valid multicast report\n", __func__));
						break;

				}
				ni->rm_rpt.type = ic->meas_info[0].measurement_type;
				ni->rm_rpt.received_rpt = 1;
			}
			else
			{
	     IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received report and sent request mismatch Invalid Report received\n", __func__));
			}
		}
		else
		{
	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Invalid Dialogue token\n", __func__));
			return ONEBOX_STATUS_FAILURE;
		}
	}

	return ONEBOX_STATUS_SUCCESS;
}
static int
rm_recv_action_neigh_rpt(struct ieee80211_node *ni,
	const struct ieee80211_frame *wh,
	const uint8_t *frm, const uint8_t *efrm)
{
	struct ieee80211vap *vap = ni->ni_vap;
	
  IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received Action Frame_1\n", __func__));
	return ONEBOX_STATUS_SUCCESS;
}
static int
rm_recv_action_link_meas(struct ieee80211_node *ni,
	const struct ieee80211_frame *wh,
	const uint8_t *frm, const uint8_t *efrm)
{
	struct ieee80211vap *vap = ni->ni_vap;

	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received Action Frame_2\n", __func__));
	    return ONEBOX_STATUS_SUCCESS;
}
/*This function is used to prepare frame request frame action frame onto air
 * @param1 : ni
 * @param2 : category
 * @param3: action
 * @param4: args passed to frame req ioctl
 * */
int send_frame_req(struct ieee80211_node *ni, int category, int action, uint8_t *args)
{
	struct rm_element meas_req;
	struct radio_meas_req frame_req;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct mbuf *m;
	uint8_t *frm;
	//uint8_t mac_addr[6] = {0xff,0xff,0xff,0xff,0xff,0xff};
	
	ni->rm_rpt.dialogue_token = args[0];
	ic->meas_info[0].dialogue_token = args[0];
	ic->meas_info[0].measurement_type = IEEE80211_FRAME;
	ic->meas_info[0].ni = ni;
	
	ieee80211_ref_node(ni);
	
	m = ieee80211_getmgtframe(&frm,
	    ic->ic_headroom + sizeof(struct ieee80211_frame),
	    sizeof(uint16_t)	/* action+category */
	    + 1 /*dialog token */
		+ 2 /*Number of repetitions. */
		+ sizeof(struct rm_element)
		+ sizeof(struct radio_meas_req)
		+ 1 /*frame request type */
		+ 6 /*MAC address*/
	);
	
	meas_req.element_id  = IEEE80211_ELEMID_MEASREQ;
	meas_req.length	   = 9 + 1 + 6 ;
	meas_req.token 	   = 1;
	meas_req.mode 	   = (BIT(1) | BIT(3));
//	meas_req.mode 	   = (BIT(1) | BIT(2) | BIT(3) | BIT(4) );
	meas_req.type 	   = IEEE80211_FRAME;

	//channel_load.operating_class = 4;
	//channel_load.channel_num = 140;
	//channel_load.rand_int = 100;
	//channel_load.meas_duration = 1000;
	memcpy(&frame_req, &args[2], sizeof(struct radio_meas_req));
	
	if (m != NULL) {
		*frm++ = category;
		*frm++ = action;
		*frm++ = args[0];		/* dialog token */
		*frm++ = 0;				/* Number of Repetitions */
		*frm++ = 0;				/* Number of Repetitions */

		memcpy(frm, &meas_req, sizeof(struct rm_element));
		frm += sizeof(struct rm_element);
		//memcpy(frm, &frame_req, sizeof(struct radio_meas_req));
		memcpy(frm, &args[2], 6);
		//frm += sizeof(struct radio_meas_req);
		frm += 6;

		*frm++ = 1;
		memcpy(frm, &frame_req.bssid, 6);
		frm += 6;

		m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
		
		//memcpy(&ni->ni_macaddr,&frame_req.unicast_mac_addr,6);	

		return rm_action_output(ni, m);
	} else {
		vap->iv_stats.is_tx_nobuf++;
		ieee80211_free_node(ni);
		return ENOMEM;
	}
}

/*This function is used to prepare channel load request action frame 
 * @param1 : ni
 * @param2 : category
 * @param3 : action
 * @param4 : args passed to ioctl
 * */
int send_channel_load_req(struct ieee80211_node *ni, int category, int action, uint8_t *args)
{
	struct rm_element meas_req;
	struct radio_meas_req channel_load;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct mbuf *m;
	uint8_t *frm;
	
	ieee80211_ref_node(ni);
	ni->rm_rpt.dialogue_token = args[0];
	ic->meas_info[0].dialogue_token = args[0];
	ic->meas_info[0].measurement_type = IEEE80211_CHANNEL_LOAD;
	ic->meas_info[0].ni = ni;
	
	m = ieee80211_getmgtframe(&frm,
	    ic->ic_headroom + sizeof(struct ieee80211_frame),
	    sizeof(uint16_t)	/* action+category */
	    + 1 /*dialog token */
		+ 2 /*Number of repetitions. */
		+ sizeof(struct rm_element)
		+ sizeof(struct radio_meas_req)
	);
	
	meas_req.element_id  = IEEE80211_ELEMID_MEASREQ;
	meas_req.length	   = 9;
	meas_req.token 	   = 1;
	meas_req.mode 	   = (0);
	meas_req.type 	   = IEEE80211_CHANNEL_LOAD;

	//channel_load.operating_class = 4;
	//channel_load.channel_num = 140;
	//channel_load.rand_int = 100;
	//channel_load.meas_duration = 1000;
	memcpy(&channel_load, &args[2], sizeof(struct radio_meas_req));
	//memcpy(&channel_load, &args[2], 6);
	
	if (m != NULL) {
		*frm++ = category;
		*frm++ = action;
		*frm++ = args[0];		/* dialog token */
		*frm++ = 0;				/* Number of Repetitions */
		*frm++ = 0;				/* Number of Repetitions */

		memcpy(frm, &meas_req, sizeof(struct rm_element));
		frm += sizeof(struct rm_element);
		//memcpy(frm, &channel_load, sizeof(struct radio_meas_req));
		memcpy(frm, &channel_load, 6);
		//frm += sizeof(struct radio_meas_req);
		frm += 6;

		m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
		//memcpy(&ni->ni_macaddr,&channel_load.unicast_mac_addr,6);	
		return rm_action_output(ni, m);
	} else {
		vap->iv_stats.is_tx_nobuf++;
		ieee80211_free_node(ni);
		return ENOMEM;
	}
}

/*This function is used to prepare multicast diagnostic request action frame 
 * @param1 : ni
 * @param2 : category
 * @param3 : action
 * @param4 : args passed to ioctl
 * */
int send_multicat_dig_req(struct ieee80211_node *ni, int category, int action, uint8_t *args)
{
	struct rm_element meas_req;
	struct radio_meas_req multicast_req;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct mbuf *m;
	uint8_t *frm;
	
	ieee80211_ref_node(ni);
	ni->rm_rpt.dialogue_token = args[0];
	ic->meas_info[0].dialogue_token = args[0];
	ic->meas_info[0].measurement_type = IEEE80211_MULTICAST_DIG_REQ;
	ic->meas_info[0].ni = ni;
	
	m = ieee80211_getmgtframe(&frm,
	    ic->ic_headroom + sizeof(struct ieee80211_frame),
	    sizeof(uint16_t)	/* action+category */
	    + 1 /*dialog token */
		+ 2 /*Number of repetitions. */
		+ sizeof(struct rm_element)
		+ sizeof(struct radio_meas_req)
	);
	
	meas_req.element_id  = IEEE80211_ELEMID_MEASREQ;
	meas_req.length	   = 9;
	meas_req.token 	   = 1;
	meas_req.mode 	   = (0);
	meas_req.type 	   = IEEE80211_MULTICAST_DIG_REQ;
	
   memcpy(&multicast_req, &args[2], sizeof(struct radio_meas_req));
	//memcpy(&channel_load, &args[2], 6);
	
	if (m != NULL) {
		*frm++ = category;
		*frm++ = action;
		*frm++ = args[0];		/* dialog token */
		*frm++ = 0;				/* Number of Repetitions */
		*frm++ = 0;				/* Number of Repetitions */

		memcpy(frm, &meas_req, sizeof(struct rm_element));
		frm += sizeof(struct rm_element);
		//memcpy(frm, &channel_load, sizeof(struct radio_meas_req));
		
		memcpy(frm, &multicast_req.rand_int, 4);         /*Here 4 bytes include rand_int -2 bytes measurement_duration -2 bytes*/
		frm += 4;
		memcpy(frm, multicast_req.bssid, 6);
		frm += 6;

		m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
		return rm_action_output(ni, m);
	} else {
		vap->iv_stats.is_tx_nobuf++;
		ieee80211_free_node(ni);
		return ENOMEM;
	}
}

/*This function is used to prepare spectral measurement request action frame 
 * @param1 : ni
 * @param2 : category
 * @param3 : action
 * @param4 : args passed to ioctl
 * */
int send_sm_meas_req(struct ieee80211_node *ni, int category, int action, uint8_t *args)
{
	struct rm_element meas_req;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct mbuf *m;
	uint8_t *frm;
	
	ieee80211_ref_node(ni);
	ni->rm_rpt.dialogue_token = args[0];
	ic->sm_meas_info[0].dialogue_token = args[0];
	ic->sm_meas_info[0].measurement_type = args[1];
	ic->sm_meas_info[0].ni = ni;
	
	m = ieee80211_getmgtframe(&frm,
	    ic->ic_headroom + sizeof(struct ieee80211_frame),
	    sizeof(uint16_t)	/* action+category */
	    + 1 /*dialog token */
	//	+ 2 /*Number of repetitions. */
		+ sizeof(struct rm_element)	
		+ sizeof(struct sm_meas_req)
	);
	
	meas_req.element_id  = IEEE80211_ELEMID_MEASREQ;
	meas_req.length	   = 3+11;
	meas_req.token 	   = 1;
	meas_req.mode 	   = args[2];
	meas_req.type 	   = args[1];
	
	if (m != NULL) {
		*frm++ = category;
		*frm++ = action;
		*frm++ = args[0];		/* dialog token */
		//*frm++ = 0;				/* Number of Repetitions */
		//*frm++ = 0;				/* Number of Repetitions */

		memcpy(frm, &meas_req, sizeof(struct rm_element));
		frm += sizeof(struct rm_element);
		memcpy(frm,&args[3],11);
		frm += 11;

		m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
		return rm_action_output(ni, m);
	} else {
		vap->iv_stats.is_tx_nobuf++;
		ieee80211_free_node(ni);
		return ENOMEM;
	}
}

/*This function is used to prepare beacon request action frame 
 * @param1 : ni
 * @param2 : category
 * @param3 : action
 * @param4 : args passed to ioctl
 * */
int send_beacon_req(struct ieee80211_node *ni, int category, int action, uint8_t *args)
{
	struct rm_element meas_req;
	struct radio_meas_req beacon_req;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct mbuf *m;
	uint8_t *frm;
	//uint8_t bssid[6] = {0xff,0xff,0xff,0xff,0xff,0xff};
	
	ni->rm_rpt.dialogue_token = args[0];
	ic->meas_info[0].dialogue_token = args[0];
	ic->meas_info[0].measurement_type = IEEE80211_BEACON;
	ic->meas_info[0].ni = ni;
	
	ieee80211_ref_node(ni);
	
	m = ieee80211_getmgtframe(&frm,
	    ic->ic_headroom + sizeof(struct ieee80211_frame),
	    sizeof(uint16_t)	/* action+category */
	    + 1 /*dialog token */
		+ 2 /*Number of repetitions. */
		+ sizeof(struct rm_element)
		+ sizeof(struct radio_meas_req)
		+ 1 /*frame request type */
		+ 6 /*MAC address*/
	);
	
	meas_req.element_id  = IEEE80211_ELEMID_MEASREQ;
	meas_req.length	   = 9 + 1 + 6 ;
	meas_req.token 	   = 1;
	meas_req.mode 	   = (BIT(1) | BIT(3));
//	meas_req.mode 	   = (BIT(1) | BIT(2) | BIT(3) | BIT(4) );
	meas_req.type 	   = IEEE80211_BEACON;

	//channel_load.operating_class = 4;
	//channel_load.channel_num = 140;
	//channel_load.rand_int = 100;
	//channel_load.meas_duration = 1000;
	memcpy(&beacon_req, &args[2], sizeof(struct radio_meas_req));
	
	if (m != NULL) {
		*frm++ = category;
		*frm++ = action;
		*frm++ = args[0];		/* dialog token */
		*frm++ = 0;				/* Number of Repetitions */
		*frm++ = 0;				/* Number of Repetitions */

		memcpy(frm, &meas_req, sizeof(struct rm_element));
		frm += sizeof(struct rm_element);
		//memcpy(frm, &beacon_req, sizeof(struct radio_meas_req));
		memcpy(frm, &args[2],6); //Operating class,Channel No,Rand_Int,Meas_Dur
		frm += 6; //
		//frm += sizeof(struct radio_meas_req);

		*frm++ = 0;//beacon_req.meas_mode;                       //Measurement mode
		memcpy(frm, &beacon_req.mac_addr, 6);				
		frm += 6;
	//	if(mease_req.bse.ssid_ie_len)
	//	{
	//		memcpy(frm, &meas_req.bse.ssid_ie, mease_req.bse.ssid_ie_len);
	//		frm += mease_req.bse.ssid_ie_len;				
	//	}		
		m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
		//memcpy(&ni->ni_macaddr,&beacon_req.unicast_mac_addr,6);	
		return rm_action_output(ni, m);
	} else {
		vap->iv_stats.is_tx_nobuf++;
		ieee80211_free_node(ni);
		return ENOMEM;
	}
}

/*This function is used to noise histogram request action frame 
 * @param1 : ni
 * @param2 : category
 * @param3 : action
 * @param4 : args passed to ioctl
 * */
int send_noise_hist_req(struct ieee80211_node *ni, int category, int action, uint8_t *args)
{
	struct rm_element meas_req;
	struct radio_meas_req noise_hist;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct mbuf *m;
	uint8_t *frm;
	
	ieee80211_ref_node(ni);
	ni->rm_rpt.dialogue_token = args[0];
	ic->meas_info[0].dialogue_token = args[0];
	ic->meas_info[0].measurement_type = IEEE80211_NOISE_HIST;
	ic->meas_info[0].ni = ni;

	m = ieee80211_getmgtframe(&frm,
	    ic->ic_headroom + sizeof(struct ieee80211_frame),
	    sizeof(uint16_t)	/* action+category */
	    + 1 /*dialog token */
		+ 2 /*Number of repetitions. */
		+ sizeof(struct rm_element)
		+ sizeof(struct radio_meas_req)
	);

	meas_req.element_id  = IEEE80211_ELEMID_MEASREQ;
	meas_req.length	   = 9;
	meas_req.token 	   = 1;
	meas_req.mode 	   = 0;
	meas_req.type 	   = IEEE80211_NOISE_HIST;
	
	memcpy(&noise_hist, &args[2], sizeof(struct radio_meas_req));

	if (m != NULL) {
		*frm++ = category;
		*frm++ = action;
		*frm++ = args[0];		/* dialog token */
		*frm++ = 0;				/* Number of Repetitions */
		*frm++ = 0;				/* Number of Repetitions */

		memcpy(frm, &meas_req, sizeof(struct rm_element));
		frm += sizeof(struct rm_element);
		memcpy(frm, &noise_hist, sizeof(struct radio_meas_req));
		frm += sizeof(struct radio_meas_req);

		m->m_pkthdr.len = m->m_len = frm - mtod(m, uint8_t *);
		return rm_action_output(ni, m);
	} else {
		vap->iv_stats.is_tx_nobuf++;
		ieee80211_free_node(ni);
		return ENOMEM;
	}
}

/*This function is invoked when response is received from firmware in respose to internal mgmt packet sent 
 * @param1 : vap pointer
 * @param2 : msg, response message from firmware
 * @param3 : length of meaasge
 * */
int ieee80211_send_meas_rpt(struct ieee80211vap *vap, uint8_t *msg, uint8_t len)
{
	struct ieee80211_node *ni = NULL;	
	struct ieee80211com *ic = NULL;
	uint8_t rpt_type;
	
	if( vap == NULL )
	{
	  IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:VAP is null\n", __func__));
		return ONEBOX_STATUS_FAILURE;
	}
	ic = vap->iv_ic;
	
		rpt_type = msg[5];
	if(rpt_type >2)
	{
		ni = ic->meas_info[0].ni;
		if( ni == NULL )
		{
	   IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:VAP is null\n", __func__));
		 return ONEBOX_STATUS_FAILURE;
		}
		if( rpt_type != ic->meas_info[0].meas_req.type) {
	    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Invalid report\n", __func__));
			return ONEBOX_STATUS_FAILURE;
		}
	}
	else
	{
		ni = ic->sm_meas_info[0].ni;
		if( ni == NULL )
		{
	   IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:VAP is null\n", __func__));
			return ONEBOX_STATUS_FAILURE;
		}
		if( rpt_type != ic->sm_meas_info[0].meas_req.type) {
	   IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Invalid report\n", __func__));
			return ONEBOX_STATUS_FAILURE;
		}
	}
	switch( rpt_type ) {
			case IEEE80211_CHANNEL_LOAD:
	        IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received channel load report from firmware\n", __func__));
					break;
			case IEEE80211_FRAME:
	        IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received frame report from firmware\n", __func__));
					break;
			case IEEE80211_BEACON:
	        IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received beacon report from firmware\n", __func__));
					break;
			case  IEEE80211_CCA_MEAS:
	        IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Received CCA report from firmware\n", __func__));
					break;
			case  IEEE80211_RPI_HIST_MEAS:
	        IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:received RPI Histogram report from firmware\n", __func__));
					break;
			case IEEE80211_MULTICAST_DIG_REQ:
	        IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Collected the multicast diagnostics stats for the required duration\n", __func__));
					break;

	}
	if( ic->ic_send_action ) {
		if(rpt_type>2)
		{
			ic->ic_send_action(ni, IEEE80211_ACTION_RADIO_MEAS, IEEE80211_ACTION_RADIO_MEAS_RPT, msg);
		}
		else
		{
			ic->ic_send_action(ni,IEEE80211_ACTION_CAT_SM, IEEE80211_ACTION_SM_MEAS_RPT, msg);
		}
	} else {
	        IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,("In %s:Action Not regsitered\n", __func__));
	}
	return ONEBOX_STATUS_SUCCESS;

}
EXPORT_SYMBOL(ieee80211_send_meas_rpt);
#endif

