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

#include "wlan_common.h"
#include "onebox_wlan_core.h"
#include "onebox_ps.h"

//static uint32 onebox_maxvaps = ONEBOX_VAPS_DEFAULT;   /* set default maximum vaps */
/** This function is used to initialize the Net80211 state machine
 * @param   Pointer to the driver private data structure
 * @ return  void
 */
#define MAX_SCAN_PER_ANTENNA 2

void core_net80211_attach(WLAN_ADAPTER w_adapter)
{
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct driver_assets *d_assets = w_adapter->d_assets;


	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\nCORE_MSG:Adding NET80211 functionality\n")));
	w_adapter->sc_nvaps = 0; //initially no vaps
	w_adapter->sc_default_ieee80211_debug = 0; //No debug option in VAP creation

	/*
	 * Setup rate tables for all potential media types.
	 */
	core_rate_setup(w_adapter, IEEE80211_MODE_11A);
	core_rate_setup(w_adapter, IEEE80211_MODE_11B);
	core_rate_setup(w_adapter, IEEE80211_MODE_11G);
	core_rate_setup(w_adapter, IEEE80211_MODE_11NA);
	core_rate_setup(w_adapter, IEEE80211_MODE_11NG);

	/*
	 * Init ic_caps prior to queue init, since WME cap setting
	 * depends on queue setup.
	 */
	ic->ic_caps = 0;
	ic->ic_caps |= IEEE80211_C_HOSTAP      |  /* hostap mode */
	               IEEE80211_C_SHPREAMBLE  |  /* short preamble supported */
	               IEEE80211_C_SHSLOT      |  /* short slot time supported */
	               IEEE80211_C_WME         |  /* Enable WME support */
	               IEEE80211_C_MONITOR     |  /* Enable Monitor Mode support */
	               IEEE80211_C_STA         |  /* Enable Station Support */
	               IEEE80211_C_TXFRAG      |  /* Enable fragmentation Support */
	               IEEE80211_C_TXPMGT;
	ic->ic_cryptocaps = ((1 << IEEE80211_CIPHER_WEP) | (1<< IEEE80211_CIPHER_TKIP) |
	                     (1 << IEEE80211_CIPHER_AES_CCM) | IEEE80211_CRYPTO_TKIPMIC);
	ic->ic_flags = 0;
	ic->ic_ifp = w_adapter->dev;
	ic->band_to_scan = 0x3;//By default we will enable dual band scan

	/* Enabling ht operation */
	/* Enabling transmit aggregation */
	ic->ic_htcaps = IEEE80211_HTC_HT;

	ic->ic_htcaps |= (IEEE80211_HTCAP_CHWIDTH40 | IEEE80211_HTCAP_SHORTGI20 |
	                  IEEE80211_HTCAP_SHORTGI40 | IEEE80211_HTC_AMPDU); 

	/* ieee80211com callback */
	ic->ic_vap_create = onebox_vap_create;
	ic->ic_vap_delete = onebox_vap_delete;
	ic->ic_newassoc = onebox_newassoc; 
	ic->ic_send_station_info = core_send_station_info;
	ic->ic_scan_end = onebox_scan_end; 
	ic->ic_scan_start = onebox_scan_start; 

	ic->ic_set_channel = core_set_channel;
	ic->ic_update_radio_params = core_radio_params_update;
	ic->ic_set_params  = core_set_params;
	ic->ic_wme.wme_update = core_wme_update;
	//ic->ic_wme_sta.wme_update = core_wme_update;
	ic->ic_updateslot     = core_updateslot;
	ic->ic_opmode = IEEE80211_M_HOSTAP;
	ic->ic_txstream = 1;
	ic->ic_rxstream = 1;
	ic->ic_stop_initial_timer = stop_initial_timer;
#ifdef IEEE80211K
	ic->ic_send_meas_info = onebox_send_meas_info;
#endif
	ic->ic_send_bgscan_params_default = send_bgscan_params_default;
	ic->ic_pwr_save = update_pwr_save_status;

	/* This ic_headroom will be used for both MGMT and DATA packets.
	 * But the HAL needs two different sizes of headrooms for mgmt(16+12) and data (16+64)
	 * So we need to take care of this either by using two variables or hardcoding the headroom 
	 * for data packets with the required value */
	ic->ic_headroom = (FRAME_DESC_SZ + TX_VECTOR_SZ + 128/*USB HEADROOM*/);
	
	/*set region domain  based on the country code selected by user */
	set_region(ic, d_assets->country_code);
	/* Fill the RF type as the channel list needs to be populated based on the
 	 * 02/03 Modules 
 	 * */
	ic->ic_regdomain.pad[1] = w_adapter->band_supported;
	/* call MI attach routine. */
	w_adapter->net80211_ops->onebox_ifattach(ic,w_adapter->mac_addr);

	ic->ic_raw_xmit = onebox_send_mgmt_pkt;
	ic->ic_bsschan = &ic->ic_channels[0];
	w_adapter->beacon_interval = ONEBOX_CPU_TO_LE16(200); // 
	ic->ic_bintval = w_adapter->beacon_interval;

	w_adapter->net80211_ops->onebox_radiotap_attach(ic, &w_adapter->txtap_hdr.wt_ihdr, 
	                                              sizeof(w_adapter->txtap_hdr),
	                                              0, &w_adapter->rxtap_hdr.wt_ihdr, 
	                                              sizeof(w_adapter->rxtap_hdr),0);
	ic->coex_mode = d_assets->oper_mode;
	ic->driver_mode = d_assets->asset_role;
	ic->mod_mode = d_assets->module_type;
	w_adapter->init_net80211_done = 1;
	ic->device_model = d_assets->device_model;

#ifdef ONEBOX_CONFIG_CFG80211
	if (!ic->cfg_priv) {
		ic->cfg_priv = w_adapter->net80211_ops->onebox_cfg80211_attach(w_adapter->device, w_adapter->mac_addr, ic);
		if(ic->cfg_priv) {
			ic->cfg_priv->ic = ic;
			w_adapter->net80211_ops->onebox_cfg80211_callbacks(ic);
		}
		w_adapter->net80211_ops->onebox_cfg80211_callbacks(ic);
	}
#endif

	return;
}

ONEBOX_STATUS is_vap_valid(struct ieee80211vap *vap)
{
	uint32 vap_id = 0;
	struct ieee80211com *ic;
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;

	if (!vap) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: VAP IS NULL In %s Line %d \n "),
					 	__func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	} else {
		if (!vap->hal_priv_vap) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: VAP Hal_priv_vap IS NULL In %s Line %d \n "),
							__func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		} else {
			ic = vap->iv_ic;
			os_intf_ops = onebox_get_os_intf_operations_from_origin();
			w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);
			vap_id = vap->hal_priv_vap->vap_id;

			if (vap_id >= ONEBOX_VAPS_DEFAULT) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s line %d Invalid vap id %d\n"),
								 __func__, __LINE__, vap_id));
				return ONEBOX_STATUS_FAILURE;
			}

			if(!(vap == w_adapter->hal_vap[vap_id].vap) || !(w_adapter->hal_vap[vap_id].vap_in_use)) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					     (TEXT("ERROR: In %s Line %d vap_id %d vap %p hal_vap %p vap_status %d\n "),
					     __func__, __LINE__, vap_id, vap, 
					     w_adapter->hal_vap[vap_id].vap, w_adapter->hal_vap[vap_id].vap_in_use));
				return ONEBOX_STATUS_FAILURE;

			}
		}
	}
	return ONEBOX_STATUS_SUCCESS;
}

static ONEBOX_STATUS is_node_valid(WLAN_ADAPTER w_adapter, struct ieee80211_node *ni)
{
		uint32_t sta_index;
		struct ieee80211vap *vap;

		if(!ni)
		{
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("ERROR: ni IS NULL In %s Line %d \n "),
										__func__, __LINE__));
				return ONEBOX_STATUS_FAILURE;

		}

		sta_index = ni->hal_priv_node.sta_id;
		vap = ni->ni_vap;
			/* Already connected.
			 * Hence checking whether sta_index is within range.
			 * */
		if(sta_index >= w_adapter->max_stations_supported)
		{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR: In %s Line %d Invalid Sta_id %d \n"), __func__, __LINE__, sta_index));
				return ONEBOX_STATUS_FAILURE;
		}
			/* Already connected.
			 * Hence checking whether mac address matches and whether sta_connected_bitmap is set.
			 * */
		else if(!((!w_adapter->os_intf_ops->onebox_memcmp(w_adapter->sta[sta_index].mac_addr, ni->ni_macaddr, ETH_ALEN)) 
					 && (w_adapter->sta_connected_bitmap[sta_index /32] & BIT(sta_index %32)))
					#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
					&& !(vap->ft_event_bit)
					#endif
		        ) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: In %s Line %d The address in Hal_station data structure and ni doesn't matches")
										,__func__, __LINE__));
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: The Mac adress in ni is \n ")));
				onebox_print_mac_address(w_adapter, ni->ni_macaddr);
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: The Mac adress in Hal sta data structure is \n ")));
				onebox_print_mac_address(w_adapter, w_adapter->sta[sta_index].mac_addr);
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
				vap->ft_event_bit = 0;
#endif
				return ONEBOX_STATUS_FAILURE;
		}
		return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function updates the beacon parameters.
 *
 * @param pointer to the VAP structure.
 * @param  value to set 
 * @return NONE.
 */
static void vap_beacon_update(struct ieee80211vap *vap, int item)
{
	struct core_vap *core_vp = (struct core_vap *)vap->hal_priv_vap->core_vp;
	struct ieee80211_beacon_offsets *bo = (struct ieee80211_beacon_offsets *)&core_vp->rv_boff;
	setbit(bo->bo_flags, item);
}

void onebox_scan_start(struct ieee80211com *ic)
{
	struct net_device *parent_dev = ic->ic_ifp;
	WLAN_ADAPTER w_adapter = netdev_priv(parent_dev);
	ONEBOX_STATUS status;
	struct ieee80211vap *vap =NULL;
	uint8_t send_vap_caps = 0 ;
	uint16_t rx_filter_word = 0;
	uint8_t antenna_value;

	if (w_adapter->antenna_diversity)
		w_adapter->scan_count++;

	if (w_adapter->scan_count > MAX_SCAN_PER_ANTENNA) {
		w_adapter->scan_count = 0;
		if (w_adapter->antenna_in_use == RSI_INTERNAL_ANTENNA)
			antenna_value = RSI_EXTERNAL_ANTENNA;
		else
			antenna_value = RSI_INTERNAL_ANTENNA;
		w_adapter->devdep_ops->onebox_program_ant_sel(w_adapter, antenna_value, CONFIG_ANT_SEL);
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO ,(TEXT("ANTENNA IN USE  %d\n"), w_adapter->antenna_in_use));
	}

	if(ic->band_flags & IEEE80211_CHAN_HT40) {
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s line %d band_flags %02x\n"), __func__, __LINE__, ic->band_flags));
	ic->band_flags &= ~IEEE80211_CHAN_HT40;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s line %d band_flags %02x\n"), __func__, __LINE__, ic->band_flags));
	send_vap_caps =1;
	}
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
	{ 
					if ((vap->iv_opmode == IEEE80211_M_HOSTAP) 
													&& (vap->iv_state == IEEE80211_S_RUN)) {
									if(!w_adapter->os_intf_ops->onebox_is_ifp_txq_stopped(vap->iv_ifp))
									{
													ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Stopping ifp queue \n")));
													vap->hal_priv_vap->stop_tx_q = 1;
													w_adapter->os_intf_ops->onebox_stop_ifp_txq(vap->iv_ifp);
													if(send_vap_caps) {
																	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Send AP vap_caps updated\n")));
																	vap->vap_caps(vap, 3); /*VAP UPDATE*/
													}
									}

									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Blocking AP data queues\n")));
									w_adapter->block_ap_queues = 1;
									onebox_send_block_unblock(vap, STA_DISCONNECTED, 0);
									//break;
					}
					else if((vap->iv_opmode == IEEE80211_M_STA) && send_vap_caps) {
									ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Send station vap_caps updated\n")));
									vap->vap_caps(vap, 3); /*VAP UPDATE*/
					}

	}
	if(send_vap_caps) {
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Sending bootup_params and radio_caps \n")));
		w_adapter->operating_chwidth = BW_20Mhz;	
		onebox_load_bootup_params(w_adapter);
	 	w_adapter->endpoint_params.per_ch_bw = w_adapter->operating_chwidth;
	  	if( ic->ic_curchan->ic_flags & IEEE80211_CHAN_11J ) 
				w_adapter->endpoint_params.enable_11j = 1;
		else 
				w_adapter->endpoint_params.enable_11j = 0;
		if(w_adapter->devdep_ops->onebox_band_check(w_adapter) == 1) {
			vap->vap_caps(vap, 3); /* Sending vap update frame */
		}
		if(onebox_load_radio_caps(w_adapter))
		{
			return ;
		}

	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("before Send Rx Filter frame to TA after scan start \n")));	
	//w_adapter->rx_filter_word = 0; //ALLOWING ALL MGMT AND DATA
	status = onebox_send_rx_filter_frame(w_adapter, rx_filter_word);
	if(status < 0)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Sending of RX filter frame on scan start failed\n")));
	}
	return;
}

void onebox_scan_end(struct ieee80211com *ic)
{
	struct net_device *parent_dev = ic->ic_ifp;
	WLAN_ADAPTER w_adapter = netdev_priv(parent_dev);
	ONEBOX_STATUS status;
	struct ieee80211vap *vap = NULL;
	uint16_t rx_filter_word = 0;

	if(w_adapter->hal_vap[w_adapter->sta_mode.vap_id].vap_in_use)
	{
		vap = w_adapter->hal_vap[w_adapter->sta_mode.vap_id].vap ;
		if(vap == NULL) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("In %s line %d station VAP is NULL\n",__func__, __LINE__));
			dump_stack();
			return;
		}
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT ("Send Rx Filter frame to TA after scan end \n")));

		if (w_adapter->rx_filter_word) {
			  rx_filter_word = w_adapter->rx_filter_word;
			 if ((vap->iv_flags_ext & IEEE80211_FEXT_SWBMISS) && (rx_filter_word & DISALLOW_BEACONS ))
				rx_filter_word &= ~DISALLOW_BEACONS;
 
		} else {
  			rx_filter_word = (ALLOW_DATA_ASSOC_PEER | ALLOW_CTRL_ASSOC_PEER |
		    			  ALLOW_MGMT_ASSOC_PEER );
		
			if (!(vap->iv_flags_ext & IEEE80211_FEXT_SWBMISS))
				rx_filter_word |= DISALLOW_BEACONS;
		}

		status = onebox_send_rx_filter_frame (w_adapter, rx_filter_word);

		if (status < 0)
		    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT ("Sending of RX filter frame on scan end failed\n")));

		ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (TEXT("In %s Line %d Sending enable deep sleep\n"), __func__, __LINE__));
		update_pwr_save_status(vap, PS_ENABLE, SCAN_END_PATH);
	}
	return;
}

/** 
 * This function changes the vap mode from station to hostap.
 *
 * @param       Pointer to vap structure.
 */
static void p2p_hal_mode_change_from_sta_to_hostap(struct ieee80211vap *vap)
{
	WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)vap->hal_priv_vap->hal_priv_ptr;

	vap->iv_rate = onebox_get_ieee80211_ratectl();

#ifdef SECURITY_SUPPORT 
	vap->iv_key_alloc = onebox_key_alloc;
	vap->iv_key_delete = onebox_key_delete;
	vap->iv_key_set = onebox_key_set;
	vap->iv_key_update_begin = onebox_key_update_begin;
	vap->iv_key_update_end = onebox_key_update_end;
	vap->iv_caps |= IEEE80211_C_WPA;
#endif

	vap->hal_priv_vap->core_vp->av_newstate = vap->iv_newstate;
	vap->iv_newstate = onebox_newstate;
	//! While we are switching VAP mode for P2P GO unblock beacon interrupts.
	if(vap->iv_opmode == IEEE80211_M_HOSTAP)
	{
	    if(!vap->beacon_ssid_notification)
	     vap->beacon_ssid_notification = set_beacon_ssid_notification;
	    if(!vap->channel_command)
	     vap->channel_command = set_channel_change_notification;
	}
	vap->hal_priv_vap->beacon_loc = 100; /* For beacon */
	//vap->iv_flags |= IEEE80211_F_NOBRIDGE; //REVIEWME LATER
	vap->iv_update_beacon = vap_beacon_update;
	vap->iv_rawbpf = vap;

	/* As the vap is changing to HOSTAP from STA one more STA can be accomodated */
	w_adapter->sc_nstavaps--;

	return;
}

/**
 * This function creates a network virtual interface.
 * 
 * @param Pointer to the ieee80211com structure 
 * @param Pointer to the interface name.
 * @parm  number used to define the interface for eg vap0, vap1
 * @pram  operating mode of the vap interface.
 * @param  flags
 * @param bssid the mac address of the device inorder to operate as AP.
 * @param Hardware address of the device.
 */
struct ieee80211vap *onebox_vap_create(struct ieee80211com *ic, 
                                       const char *name, int unit, enum ieee80211_opmode opmode, int flags, 
                                       const uint8 bssid[IEEE80211_ADDR_LEN],
                                       uint8_t macaddr[IEEE80211_ADDR_LEN])
{
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;
	struct ieee80211vap *vap = NULL, *vap_temp = NULL;
	enum ieee80211_opmode ic_opmode = IEEE80211_M_HOSTAP;
#ifdef ENABLE_P2P_SUPPORT
	enum ieee80211_opmode p2p_mode = IEEE80211_M_P2P;
#endif
	uint8 index = 0;
	uint8 vap_id;
	ONEBOX_STATUS status;
	struct driver_assets *d_assets;
#ifdef ONEBOX_CONFIG_CFG80211
	struct wireless_dev *wdev;
#endif
	uint8 std_rates[MAX_BG_SUPP_RATES] = {STD_RATE_01, STD_RATE_02, STD_RATE_5_5,
			STD_RATE_06, STD_RATE_09, STD_RATE_11, STD_RATE_12,
			STD_RATE_18, STD_RATE_24, STD_RATE_36, STD_RATE_48,
			STD_RATE_54}; 

	uint16 rps_rates[MAX_BG_SUPP_RATES] = {RSI_RATE_1,  RSI_RATE_2,  RSI_RATE_5_5,
			RSI_RATE_6,  RSI_RATE_9 , RSI_RATE_11, RSI_RATE_12,
			RSI_RATE_18, RSI_RATE_24, RSI_RATE_36, RSI_RATE_48,
			RSI_RATE_54}; /* Followed by power values */
#ifdef ENABLE_P2P_SUPPORT
	uint16 p2p_enable = 0;
#endif
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);
	d_assets = w_adapter->d_assets;
	switch (opmode) 
	{
		case IEEE80211_M_HOSTAP:
			if(!(d_assets->oper_mode & BIT(1))) {
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("AP not configured in COEX mode\n"));
					return NULL;
			}
			/* permit multiple APs and/or WDS links */
			if ((w_adapter->sc_nvaps != 0) && (ic->ic_opmode == IEEE80211_M_STA))
			{
				return NULL;
			}
			ic_opmode = IEEE80211_M_HOSTAP;
		break;
		case IEEE80211_M_STA:
			if(!(d_assets->oper_mode & BIT(0))) {
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("STA not configured in COEX mode\n"));
					return NULL;
			}
			if (w_adapter->sc_nstavaps != 0)  /* only one sta regardless */
			{
				return NULL;
			}
			if ((w_adapter->sc_nvaps != 0) && 
			    (!(flags & IEEE80211_CLONE_NOBEACONS))) 
			{
				if (ic->ic_opmode != IEEE80211_M_MONITOR)
				{
					return NULL; /* If using station beacons, must first up */
				}
			}
			if (flags & IEEE80211_CLONE_NOBEACONS) 
			{
				w_adapter->sc_nostabeacons = 1;
				ic_opmode = IEEE80211_M_HOSTAP; /* Run with chip in AP mode */
			} 
			else
			{
				ic_opmode = opmode;
			}
		break;
		case IEEE80211_M_IBSS:
			if ((w_adapter->sc_nvaps != 0) && (ic->ic_opmode == IEEE80211_M_STA))
			{
				return NULL;
			}
			if (ic->ic_opmode == IEEE80211_M_HOSTAP)
			{
				ic_opmode = ic->ic_opmode;
			}
			else
			{
				ic_opmode = opmode;
			}
		break;
		case IEEE80211_M_AHDEMO:
		case IEEE80211_M_MONITOR:
			if (w_adapter->sc_nvaps != 0 && (ic->ic_opmode != opmode)) 
			{
				/* preserve existing mode */
				ic_opmode = ic->ic_opmode;
			}
			else
			{
				ic_opmode = opmode;
			}
		break;
		case IEEE80211_M_WDS:
			return NULL;
#ifdef ENABLE_P2P_SUPPORT
		case IEEE80211_M_P2P:
		{
			p2p_enable = 1;
			opmode = IEEE80211_M_STA; // Start P2P Device in STA mode
			if (w_adapter->sc_nstavaps != 0)  /* only one sta regardless */
			{
				return NULL;
			}
			if ((w_adapter->sc_nvaps != 0) && 
			    (!(flags & IEEE80211_CLONE_NOBEACONS))) 
			{
				if (ic->ic_opmode != IEEE80211_M_MONITOR)
				{
					return NULL;   /* If using station beacons, must first up */
				}
			}
			if (flags & IEEE80211_CLONE_NOBEACONS) 
			{
				w_adapter->sc_nostabeacons = 1;
				ic_opmode = IEEE80211_M_HOSTAP; /* Run with chip in AP mode */
			} 
			else
			{
				ic_opmode = opmode;
			}
		}
		break;
#ifdef ONEBOX_CONFIG_CFG80211
		case IEEE80211_M_P2P_GO:
		{
			p2p_enable = 1;
			p2p_mode = IEEE80211_M_P2P_GO;
			opmode = IEEE80211_M_HOSTAP; // Start P2P Device in STA mode
			ic_opmode = opmode;
		}
		break;
#endif
#endif
		default:
			return NULL;
	}

	if (w_adapter->sc_nvaps >= ONEBOX_VAPS_DEFAULT) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("\nCORE_DBG: Too many virtual APs (%d already exist).\n",w_adapter->sc_nvaps));
		return NULL;
	}

#if 0
	for (index = 0; index < ONEBOX_VAPS_DEFAULT; index++)
	{
		struct ieee80211vap *vap;

		base_mac_address = ((w_adapter->mac_addr[5] & 0xfc) + ((w_adapter->mac_addr[5] + index) & 0x03));
		fill = 1;
		TAILQ_FOREACH(v, &ic->ic_vaps, iv_next)
		{
			if ((base_mac_address == v->iv_myaddr[5]) && (fill == 1))
			{
				fill = 0;
			}
		}
		if (fill)
		{ 
			free_id = index; 
			break;
		}
	}
	macaddr[5] = ((w_adapter->mac_addr[5] & 0xfc) + ((w_adapter->mac_addr[5] + free_id) & 0x03));
	vap_id = w_adapter->os_intf_ops->onebox_extract_vap_id(name);
#endif

	for(index = 0; index < ONEBOX_VAPS_DEFAULT; index++)
	{
		if(!w_adapter->hal_vap[index].vap_in_use){
				vap_id = index;
				break;
		}
	}

	if(index == ONEBOX_VAPS_DEFAULT)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to create VAP as MAX VAPS are already allocated\n")));
		return NULL;

	}
	macaddr[5] = ((w_adapter->mac_addr[5] & 0xfc) | vap_id);
		
	vap = w_adapter->net80211_ops->onebox_vap_setup(ic, NULL, name, unit, opmode, flags, bssid , macaddr, 0);
	w_adapter->os_intf_ops->onebox_memcpy((PVOID)vap->iv_ifp->dev_addr, (const void *)macaddr, (int)6);
	vap->iv_rate = onebox_get_ieee80211_ratectl();

#ifdef BYPASS_TX_DATA_PATH
	vap->iv_block = onebox_send_block_unblock;
	vap->iv_mod_bgscan_params = onebox_send_bgscan_params;
#endif
#ifdef PWR_SAVE_SUPPORT
	/*: Null for AP/other modes. Only station mode holds this */
	if (opmode == IEEE80211_M_STA)
	{
		vap->check_traffic = check_traffic_pwr_save; 
	}
#endif


	if (opmode == IEEE80211_M_MONITOR) 
	{
		ic->ic_montaps++;
		vap->iv_ifp->type = ARPHRD_IEEE80211_RADIOTAP;
		vap->iv_flags_ext  |= IEEE80211_FEXT_BPF;
	}

	//	disabled shortGI by default.
	//	vap->iv_flags_ht |= IEEE80211_FHT_SHORTGI20 | IEEE80211_FHT_SHORTGI40; 
	vap->iv_flags_ht |= IEEE80211_FHT_STBC_RX | IEEE80211_FHT_RIFS  | IEEE80211_FHT_GF;

#ifdef SECURITY_SUPPORT 
	vap->iv_key_alloc = onebox_key_alloc;
	vap->iv_key_delete = onebox_key_delete;
	vap->iv_key_set = onebox_key_set;
	vap->iv_key_update_begin = onebox_key_update_begin;
	vap->iv_key_update_end = onebox_key_update_end;
	vap->iv_caps |= IEEE80211_C_WPA;
#endif
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d rtnl %d\n"), __func__, __LINE__, rtnl_is_locked()));
#ifdef BYPASS_TX_DATA_PATH
	vap->iv_caps |= IEEE80211_C_8023ENCAP;
#endif
	if (vap->iv_opmode == IEEE80211_M_HOSTAP)
	{
	    vap->iv_des_chan = &ic->ic_channels[0];
			vap->beacon_ssid_notification = set_beacon_ssid_notification;
			vap->channel_command = set_channel_change_notification;
	}
	//These two structures should be merged later
	w_adapter->os_intf_ops->onebox_mem_alloc((PVOID *)&vap->hal_priv_vap, 
	                                       sizeof(struct hal_priv_ieee80211vap), GFP_ATOMIC);
	w_adapter->os_intf_ops->onebox_memset(vap->hal_priv_vap, 0, 
	                                    sizeof(struct hal_priv_ieee80211vap));
	w_adapter->os_intf_ops->onebox_mem_alloc((PVOID *)&vap->hal_priv_vap->core_vp, 
	                                        sizeof(struct core_vap), GFP_ATOMIC);
	w_adapter->os_intf_ops->onebox_memset(vap->hal_priv_vap->core_vp, 0, 
	                                    sizeof(struct core_vap));

	vap->hal_priv_vap->core_vp->av_newstate = vap->iv_newstate;
	vap->hal_priv_vap->hal_priv_ptr = (void *)w_adapter ;

#ifdef ONEBOX_CONFIG_CFG80211
	//w_adapter->net80211_ops->onebox_cfg80211_attach(vap->iv_ifp, (void *)w_adapter->device, sizeof(struct ieee80211vap));
#ifdef ENABLE_P2P_SUPPORT
	if (p2p_enable)
		wdev = w_adapter->net80211_ops->onebox_cfg80211_register_wireless_dev(vap->iv_ifp, ic->cfg_priv, p2p_mode);
	else
#endif
		wdev = w_adapter->net80211_ops->onebox_cfg80211_register_wireless_dev(vap->iv_ifp, ic->cfg_priv, vap->iv_opmode);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Line %d wdev %p opmode %d\n"), __func__, __LINE__, wdev, opmode));

#endif

	vap->iv_newstate = onebox_newstate;
	
	vap->hal_priv_vap->aggr_rx_limit = w_adapter->aggr_limit.rx_limit;

	vap->hal_priv_vap->beacon_loc = 100; /* For beacon */
	flags = IEEE80211_CLONE_BSSID;
	if (w_adapter->sc_default_ieee80211_debug) 
	{
		/* User specified defaults for new VAPs were provided, so
		 * use those (only). */
		vap->iv_debug = (w_adapter->sc_default_ieee80211_debug & ~IEEE80211_MSG_DEBUG);
	}
	else 
	{
		/* If no default VAP debug flags are passed, allow a few to
		 * transfer down from the driver to new VAPs so we can have load
		 * time debugging for VAPs too. */
		//: for debugging 
	}
//	vap->iv_debug = 0x0fffffff;
	vap->iv_debug = IEEE80211_MSG_ERROR;
	//vap->iv_debug = (IEEE80211_MSG_ERROR | IEEE80211_MSG_DEBUG | IEEE80211_MSG_ASSOC 
	//		| IEEE80211_MSG_SCAN | IEEE80211_MSG_AUTH);
	//vap->iv_flags |= IEEE80211_F_NOBRIDGE; //REVIEWME LATER
	vap->iv_update_beacon = vap_beacon_update;
	vap->iv_rawbpf = vap;
#ifdef ENABLE_P2P_SUPPORT
	if(p2p_enable)
	{
		/* This function should be made part of net80211 callbacks */ 
		w_adapter->net80211_ops->onebox_p2p_init(vap);

		if (p2p_mode == IEEE80211_M_P2P_GO)
			vap->p2p_mode = IEEE80211_P2P_GO;

		if(vap->p2p)
		{
			vap->p2p->p2p_hal_mode_change_from_sta_to_hostap = p2p_hal_mode_change_from_sta_to_hostap;
			/* This should be done in net80211 init itself along with others
			 * Eventually it should be moved there */
			ic->ic_caps |= IEEE80211_C_P2P;
		}
	}
#endif

	vap->vap_caps = set_vap_capabilities;
	vap->vap_dynamic_update = onebox_send_vap_dynamic_update_indication_frame;
#ifdef ONEBOX_CONFIG_WOWLAN
	vap->config_wowlan = onebox_vap_config_wowlan;
#endif
	/* complete setup */
	status = w_adapter->net80211_ops->onebox_vap_attach(vap, w_adapter->net80211_ops->onebox_media_change,
	                                         w_adapter->net80211_ops->onebox_media_status, 0);
	if(status < 0){
			//FIMXE: Free the memory which is allocated for ifp and dev ptr
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR: Failed to create vap_interface \n")));
		return NULL;
	}

	/** Storing Vap refernece for Hal*/
	w_adapter->hal_vap[vap_id].vap_in_use = 1;
	w_adapter->hal_vap[vap_id].vap = vap;
	
#ifdef ONEBOX_CONFIG_CFG80211
	vap->wdev = vap->iv_ifp->ieee80211_ptr;
	ic->cfg_priv->ndev[vap->hal_priv_vap->vap_id] = vap->iv_ifp;
	ic->cfg_priv->wdev[vap->hal_priv_vap->vap_id] = vap->wdev;
#endif
	vap->onebox_send_bgscan_host_triggered = send_bgscan_host_triggered_bgscan;

	ic->ic_opmode = ic_opmode;
	if (opmode != IEEE80211_M_WDS)
	{
		w_adapter->sc_nvaps++;
	}

	if (opmode == IEEE80211_M_STA)
	{
		w_adapter->sc_nstavaps++;
	}
	else if (opmode == IEEE80211_M_MONITOR)
	{
		w_adapter->sc_nmonvaps++;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("number of monitor vaps is %d\n"), w_adapter->sc_nmonvaps));

	/* Driving the HAL in IBSS sometimes adapts the TSF and other timing registers
	 * from received beacons/probes. If that happens, expected TX interrupts may
	 * not occur until next reset. Which triggers the "lost beacon" tasklet.
	 * Resulting effectively in not sending packets for minutes. Because that only
	 * happens in large mesh networks, this mode needs to be activated by a kernel
	 * module parameter: hostap_for_ibss=1. Note that using this mode has side
	 * effects. Such as not supressing beacons/probe answers randomly when
	 * receiving other node beacons. It's recommended to lower the beacon interval
	 * then. When using an IBSS-VAP together with an HOSTAP-VAP, you may also need
	 * to re-trigger IBSS beacon generation after creating the HOSTAP-VAP by
	 * issueing "iwpriv wl1650X bintval 1000".
	 */
	if ((flags & IEEE80211_CLONE_NOBEACONS) &&
	    (ic->ic_opmode == IEEE80211_M_IBSS))
	{
		w_adapter->sc_opmode = IEEE80211_M_HOSTAP;
	}
	else
	{
		/*
		 * Adhoc demo mode is a pseudo mode; to the HAL it's
		 * just IBSS mode and the driver doesn't use management
		 * frames.  Other modes carry over directly to the HAL.
		 */
		if (ic->ic_opmode == IEEE80211_M_AHDEMO)
		{
			w_adapter->sc_opmode = IEEE80211_M_HOSTAP;
		}
		else
		{
			w_adapter->sc_opmode = ic->ic_opmode;
		}
	}

	vap->hal_priv_vap->vap_id = vap_id;
 
	w_adapter->os_intf_ops->onebox_memcpy(w_adapter->std_rates, std_rates , sizeof(std_rates));
	w_adapter->os_intf_ops->onebox_memcpy(w_adapter->rps_rates, rps_rates , sizeof(rps_rates));
	if(opmode != IEEE80211_M_MONITOR) 	
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Sending vap_caps request vap_id %d\n"), vap_id));
		w_adapter->devdep_ops->onebox_set_vap_capabilities(vap, VAP_ADD);
		//	w_adapter->devdep_ops->onebox_set_vap_capabilities(w_adapter, unit, opmode, NULL);
#ifdef BYPASS_TX_DATA_PATH
		if(opmode == IEEE80211_M_STA)
		{
			w_adapter->devdep_ops->onebox_send_block_unblock_data(vap, STA_DISCONNECTED, 0);
			w_adapter->sta_mode.vap_id = vap->hal_priv_vap->vap_id;
		}
#endif
	}
	if((w_adapter->sc_nstavaps) && (w_adapter->sc_nvaps > 1)) {
		TAILQ_FOREACH(vap_temp, &ic->ic_vaps, iv_next)
		{
			if(vap_temp->iv_opmode == IEEE80211_M_STA) {
				if(vap_temp->hal_priv_vap->ps_params.ps_en){
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Disabling Power save as multi vap is enabled\n")));
					//update_pwr_save_status(vap_temp, PS_DISABLE, VAP_CREATE_PATH);
					send_ps_params_req(vap_temp, PS_DISABLE, 0);
				}
			}
		}
	}	

#ifdef ONEBOX_CONFIG_CFG80211
	if(vap->iv_opmode == IEEE80211_M_HOSTAP) {
		netif_carrier_off(vap->iv_ifp);
	}
#endif
	
#ifdef ONEBOX_CONFIG_GTK_OFFLOAD
  vap->send_gtk_rekey_data = onebox_send_gtk_rekey_data;
#endif
	return vap;
}

/**
 * This function deletes the created virtual interface of the driver.
 * @param Pointer to the vap structure.
 * @return none.
 */
void onebox_vap_delete(struct ieee80211vap *vap, uint8 wait_for_lock)
{
	WLAN_ADAPTER w_adapter;
	struct ieee80211com *ic;
	struct ieee80211vap *vap_temp = NULL;
	uint32 vap_id = vap->hal_priv_vap->vap_id;
	int stop_timer = 0 ;
	netbuf_ctrl_block_m_t *netbuf_m = NULL;
#ifdef ONEBOX_CONFIG_CFG80211
	struct wireless_dev *wdev = NULL;
#endif
	//uint32 sta_index;

	w_adapter = (WLAN_ADAPTER)vap->hal_priv_vap->hal_priv_ptr;
	ic = &w_adapter->vap_com;
	if (vap->iv_opmode != IEEE80211_M_WDS)
	{
		w_adapter->sc_nvaps--;
	}
	/* Stop loading beacons into ppe */
	if (vap->iv_opmode == IEEE80211_M_MONITOR)
	{
		w_adapter->sc_nmonvaps--;
		ic->ic_montaps--;
	}
	else if (vap->iv_opmode == IEEE80211_M_HOSTAP) 
	{
		netbuf_m = vap->hal_priv_vap->core_vp->rv_bcbuf;
		//w_adapter->sc_nvaps--;
	}
	else if (vap->iv_opmode == IEEE80211_M_STA)
	{
		w_adapter->sc_nstavaps--;
	}

#ifdef ENABLE_P2P_SUPPORT
	if(vap->p2p && vap->p2p->p2p_deinit)
	{
		vap->p2p->p2p_deinit(vap);
	}
#endif

#ifdef PWR_SAVE_SUPPORT
	if (vap->iv_opmode == IEEE80211_M_STA) {
		stop_timer = 1 ;
	}

#endif


#if 0
	if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
		for (sta_index=0; sta_index < w_adapter->max_stations_supported; sta_index++) {
			if ((w_adapter->sta_connected_bitmap[sta_index/32] & (1 << (sta_index%32)))) {//check for connected sta
				if (w_adapter->sta[sta_index].ni->ni_vap == vap) {//check whether sta maps to related vap 
					w_adapter->sta_connected_bitmap[sta_index /32] &= ~(1 << (sta_index % 32));
					w_adapter->os_intf_ops->onebox_memset(w_adapter->sta[sta_index].mac_addr, 0, ETH_ALEN);
					w_adapter->sta[sta_index].ni = NULL;
				}
			}
		}
	}
#endif
	w_adapter->os_intf_ops->onebox_acquire_sem(&w_adapter->ic_lock_vap, 0);
#ifdef ONEBOX_CONFIG_CFG80211
	//! Getting Pointer of wdev pointer to free since iv_ifp will not be valid after vap_detach function call.
	wdev = vap->iv_ifp->ieee80211_ptr;
#endif
	w_adapter->net80211_ops->onebox_vap_detach(vap, 0, wait_for_lock);
	if (netbuf_m) {
		dev_kfree_skb((struct sk_buff *)netbuf_m->pkt_addr); 
		kfree(netbuf_m);
	}
#ifdef ONEBOX_CONFIG_CFG80211
	//! Free the wdev which got allocated during VAP creation.
	if(wdev)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("[%s][%d]: Freeing wdev pointer!!! \n"), __func__, __LINE__));
		kfree(wdev);
		wdev = NULL;
	}
#endif
	if(stop_timer ){
			if(w_adapter->traffic_timer.function)
			{
					ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<<<< Removing traffic timer >>>>\n")));
					w_adapter->os_intf_ops->onebox_remove_timer(&w_adapter->traffic_timer);
			}
	}
	//w_adapter->net80211_ops->onebox_vap_detach(vap, 0, wait_for_lock);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d\n"), __func__, __LINE__));


	w_adapter->hal_vap[vap_id].vap_in_use = 0;
	w_adapter->hal_vap[vap_id].vap = NULL;
	w_adapter->os_intf_ops->onebox_release_sem(&w_adapter->ic_lock_vap);

	if((w_adapter->sc_nstavaps) && (w_adapter->sc_nvaps < 2)) {
		TAILQ_FOREACH(vap_temp, &ic->ic_vaps, iv_next)
		{
			if(vap_temp->iv_opmode == IEEE80211_M_STA) {
				if((vap_temp->iv_state == IEEE80211_S_INIT) ||
				   (vap_temp->iv_state == IEEE80211_S_RUN)){
					if(vap_temp->hal_priv_vap->ps_params.ps_en){
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Enabling Power Save mode as only STA Vap is running\n")));
						update_pwr_save_status(vap_temp, PS_ENABLE, IOCTL_PATH);
					}
				}
			}
		}
	}
}

void onebox_send_sta_supported_features(struct ieee80211vap *vap, WLAN_ADAPTER w_adapter)
{

	if((vap->hal_priv_vap->bgscan_params_ioctl.bg_ioctl))
	{
		if(onebox_send_bgscan_params(vap, (uint16 *)&vap->hal_priv_vap->bgscan_params_ioctl, 0))
    		{
      			return;
    		}
		send_bgscan_probe_req(w_adapter, vap->iv_bss, vap->hal_priv_vap->bgscan_params_ioctl.bg_cmd_flags);
	}
	/** By default TIMER will send Enable*/
	update_pwr_save_status(vap, PS_ENABLE, TIMER_PATH);
	return ;
}

/*Sending the adjacent report to the new AP*/
#if defined(RSI_CCX) || defined(CONFIG_11R)
int send_ap_adjacent_report(WLAN_ADAPTER w_adapter, struct ieee80211vap *vap)
{
	struct ieee80211_node *ni = NULL;
	struct ieee80211_iapp_header aironet_iapp_header;
	struct ieee80211_eth_header aironet_eth_header;
	netbuf_ctrl_block_t   *netbuf_cb = NULL;
	uint16 iapp_pkt_len = sizeof(struct ieee80211_eth_header) + sizeof(struct ieee80211_iapp_header) + sizeof(struct ieee80211_iapp_pkt);
	uint16 pkt_priority = BE_Q_STA;
	
	ni = vap->iv_bss;
	if(ni == NULL)
	{

		printk("%s %d: Ni is null\n", __func__, __LINE__);
		return ONEBOX_STATUS_FAILURE;
	}

	netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(iapp_pkt_len);
	if(netbuf_cb == NULL)
	{            
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}    
	w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, (iapp_pkt_len)); 
	printk("\nnetbuf_cb->len : %d",netbuf_cb->len);
        
	netbuf_cb->ni = ni;
	if ((netbuf_cb->flags & ONEBOX_BROADCAST) || (netbuf_cb->flags & ONEBOX_MULTICAST))
        {
		netbuf_cb->flags &= ~ONEBOX_BROADCAST;
		netbuf_cb->flags &= ~ONEBOX_MULTICAST;
        }
        
        /* Preparing Ethernet Header */
	w_adapter->os_intf_ops->onebox_memcpy(aironet_eth_header.da, vap->iv_bss->ni_macaddr, IEEE80211_ADDR_LEN);
	w_adapter->os_intf_ops->onebox_memcpy(aironet_eth_header.sa, w_adapter->mac_addr, IEEE80211_ADDR_LEN);
	aironet_eth_header.type = RSI_WLCCP_PROTOCOL ; /*Type as 0*/

	/* Preparing the IAPP header */
	aironet_iapp_header.type    = RSI_ADJACENT_AP_REPORT;
	aironet_iapp_header.length    = sizeof(struct ieee80211_iapp_pkt) + (6 * 2) ;
	aironet_iapp_header.version = 0x00;
	if (aironet_iapp_header.type == RSI_ADJACENT_AP_REPORT)
	{
		aironet_iapp_header.subtype = RSI_ADJ_AP_SUBTYPE_REPORT ;
		w_adapter->os_intf_ops->onebox_memcpy(aironet_iapp_header.da,
				vap->iv_bss->ni_macaddr,
				IEEE80211_ADDR_LEN);
	}
	w_adapter->os_intf_ops->onebox_memcpy(aironet_iapp_header.sa, w_adapter->mac_addr, IEEE80211_ADDR_LEN);


        /*Copying the ethernet header*/
	w_adapter->os_intf_ops->onebox_memcpy((void *)netbuf_cb->data, (const void *)&aironet_eth_header, sizeof(struct ieee80211_eth_header));
        
        /*Copying the iapp header*/
	w_adapter->os_intf_ops->onebox_memcpy((void *)netbuf_cb->data + sizeof(struct ieee80211_eth_header), (const void *)&aironet_iapp_header, sizeof(struct ieee80211_iapp_header)); 
        vap->aironet_iapp_pkt.aironet_disconect_at = jiffies_to_msecs((jiffies - vap->aironet_iapp_pkt.aironet_disconect_at)); 
        
        /*Copying the iapp packet information*/
	w_adapter->os_intf_ops->onebox_memcpy((void *)netbuf_cb->data + sizeof(struct ieee80211_eth_header) + sizeof(struct ieee80211_iapp_header),
                                                                              (const void *)&vap->aironet_iapp_pkt, sizeof(struct ieee80211_iapp_pkt));
        printk("\nPrinting the netbufcd->data before on air desc :\n");
        w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR,
                                            (uint8 *)netbuf_cb->data,
                                            netbuf_cb->len);
	w_adapter->IAPP_PKT = 1;	
	if(prepare_onair_data_pkt_desc(w_adapter, netbuf_cb, pkt_priority) == ONEBOX_STATUS_SUCCESS)
	{ 
		w_adapter->IAPP_PKT = 0;	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Queueing AP adjacent report pkt\n")));
		core_queue_pkt(w_adapter, netbuf_cb, (pkt_priority));
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR,
				(uint8 *)netbuf_cb->data,
				netbuf_cb->len);

		/* Schedule the data transmit task */
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: ===> Scheduling TX tasklet for IAPP Packet <===\n"),__func__));
		w_adapter->devdep_ops->onebox_schedule_pkt_for_tx(w_adapter);
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERR: Preparing the on air data packet desc for IAPP Pkt failed\n")));
		return ONEBOX_STATUS_FAILURE;

	}
	return ONEBOX_STATUS_SUCCESS;
}
#endif
/**
  * This function is used to setup driver-specific state for a newly associated node.
  * Note that we're called also on a re-associate, the isnew
  * param tells us if this is the first time or not.
  *
  * @param  Pointer to the associated node structure,
  * @param  currently unused 
  * @return void
  */
void onebox_newassoc(struct ieee80211_node *ni, int isnew)
{
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;
	struct ieee80211com *ic = ni->ni_ic;
	struct ieee80211vap *vap = ni->ni_vap;
        uint32 ret = 0;
	uint8 i=0;
	//: vap id needs to be revisited
	//vap_id = ONEBOX_GET_VAP_ID(ni->ni_bssid);
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);
	/* Its mandatory to send peer notify with a valid assoc id after conn.
	 * It will be used for the TIM checking in the beacon in LMAC for data retrieval
	 */
	core_send_station_info(ni, STA_CONNECTED);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("#######################################\n")));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("IN %s Line %d  no of rates = %d no of ht rates = %d supported rates are\n"), __func__, __LINE__,ni->ni_rates.rs_nrates,ni->ni_htrates.rs_nrates));
	for(i=0; i< ni->ni_rates.rs_nrates ; i++)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x  "),ni->ni_rates.rs_rates[i])); 
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n#######################################\n")));

	onebox_send_auto_rate_request(w_adapter, ni);

	if(vap->iv_opmode == IEEE80211_M_STA) {
		onebox_load_radio_caps(w_adapter);

#ifdef BGSCAN_SUPPORT
	if((!((vap->iv_flags & IEEE80211_F_WPA2) || (vap->iv_flags & IEEE80211_F_WPA1)) 
					&& (vap->iv_flags & IEEE80211_F_PRIVACY)) || !(vap->hal_priv_vap->conn_in_prog))
	{
		onebox_send_block_unblock(vap, STA_CONNECTED, 0);
#if defined(RSI_CCX) || defined(CONFIG_11R)
		if(vap->hal_priv_vap->IAPP_IND_FLAG)
		{
			printk("\n######################Sending AP Adjacent report####################\n");
			ret = send_ap_adjacent_report(w_adapter, vap);
			if(ret != ONEBOX_STATUS_SUCCESS)
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Err: Sending AP adjacent report failed in %s Line %d\n"), __func__, __LINE__));
			vap->hal_priv_vap->IAPP_IND_FLAG = 0;
		}
#endif
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("calling timeout initialziation In %s Line %d\n"), __func__, __LINE__));
		//initialize_sta_support_feature_timeout(vap, w_adapter);
		update_pwr_save_status(vap, PS_ENABLE, CONNECTED_PATH);
	}	
#endif
	}
}

struct ieee80211_node *core_node_alloc(struct ieee80211vap *vap, const uint8 *mac_addr)
{
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;
	struct ieee80211_node *node = NULL;

	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(vap->iv_ic->ic_ifp);
	node = w_adapter->sc_node_alloc(vap, mac_addr);

	if (node != NULL)
	{
#ifdef ENABLE_P2P_SUPPORT
		if(vap->p2p_enable)
#endif
		{
			node->ni_intval = vap->iv_ic->ic_bintval;
		}
		/*
		 * ath_rate_node_init needs a vap pointer in node
		 * to decide which mgt rate to use
		 */
		node->ni_vap = vap;
		return node;
	}
	else
	{
		return NULL;
	}
}

/** This function is used to free the node. It in turn calls node cleanup
 * for freeing the node
 * @param pointer to the ieee80211_node structure
 * @return void
 */
void core_node_free(struct ieee80211_node *ni)
{
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ni->ni_ic->ic_ifp);

	w_adapter->sc_node_free(ni);
}


/** This function is used to free the node. 
 * @param pointer to the ieee80211_node structure
 * @return void
 */
void onebox_node_cleanup(struct ieee80211_node *ni)
{
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;

	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ni->ni_ic->ic_ifp);
	//ATH_NODE_UAPSD_LOCK_IRQ(an);
	w_adapter->sc_node_cleanup(ni);
	//ATH_NODE_UAPSD_UNLOCK_IRQ(an);
}

void stop_initial_timer(struct ieee80211com *ic, struct ieee80211vap *vap)
{

	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);
	if(driver_ps.delay_pwr_sve_decision_flag && (vap->iv_state != IEEE80211_S_RUN)) {
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("In %s Line %d stoppping timer\n"), __func__, __LINE__));
			w_adapter->os_intf_ops->onebox_remove_timer(&w_adapter->traffic_timer);//NEED TO TEST
			driver_ps.delay_pwr_sve_decision_flag = 0;
	}
			return ;
}

void send_bgscan_params_default(struct ieee80211com *ic, uint8 band)
{

	struct net_device *parent_dev = ic->ic_ifp;
	WLAN_ADAPTER w_adapter = netdev_priv(parent_dev);
	onebox_mac_frame_t *mgmt_frame;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	uint8 status, ii; 
	struct ieee80211vap *vap = NULL;
	uint8 channels_5g[] = {36,40,44,48,
		52,56,60,64,
		100,104,108,112,
		116,120,124,128,
		132,136,140,
		149,153,157,161,165};
	
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) { 
		if (vap->iv_opmode == IEEE80211_M_STA)
			break;
	}

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((sizeof(mgmt_frame->u.bgscan_params) ) | 
			(ONEBOX_WIFI_MGMT_Q << 12));
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BG_SCAN_PARAMS);

	mgmt_frame->u.bgscan_params.bgscan_threshold = 10;
	mgmt_frame->u.bgscan_params.roam_threshold = 65;
	mgmt_frame->u.bgscan_params.bgscan_periodicity = 60;
	if( vap->iv_host_scan )
	{
		mgmt_frame->u.bgscan_params.bgscan_periodicity = 0;
		vap->iv_host_scan = 0;
	}

	if (band == BAND_5GHZ) {
		for (ii = 0; ii < (sizeof(channels_5g)/sizeof(uint8)) ; ii++) {
			printk("channel is %d\n", channels_5g[ii]);	
			if ((channels_5g[ii] >=52) && (channels_5g[ii] <= 140) ) {
				mgmt_frame->u.bgscan_params.channels2scan[ii] = (channels_5g[ii] | BIT(15));
			} else
				mgmt_frame->u.bgscan_params.channels2scan[ii] = channels_5g[ii];
		}
		mgmt_frame->u.bgscan_params.num_bg_channels = 4;
		//mgmt_frame->u.bgscan_params.channels2scan[0] = 36;
		//mgmt_frame->u.bgscan_params.channels2scan[1] = 48;
		//mgmt_frame->u.bgscan_params.channels2scan[2] = 56 | BIT(15) ;
		//mgmt_frame->u.bgscan_params.channels2scan[3] = 44;
	} else {
		for (ii = 0;ii < 11; ii++)
			mgmt_frame->u.bgscan_params.channels2scan[ii] = ii+1;
		mgmt_frame->u.bgscan_params.num_bg_channels = ii;
	}
	mgmt_frame->u.bgscan_params.active_scan_duration = 30;
	mgmt_frame->u.bgscan_params.passive_scan_duration = 50;


	status = w_adapter->devdep_ops->onebox_send_internal_mgmt_frame(w_adapter,
			(uint16 *)mgmt_frame,
			FRAME_DESC_SZ + sizeof(mgmt_frame->u.bgscan_params));

	if (band == BAND_5GHZ)
		ic->bgscan_5ghz_sent = 1;
	else
		ic->bgscan_2ghz_sent = 1;


	status = send_bgscan_probe_req(w_adapter, vap->iv_bss, BIT(4));

	return ;
}

void core_radio_params_update(struct ieee80211com *ic)
{
	onebox_mac_frame_t *mgmt_frame;
  WLAN_ADAPTER w_adapter;
	uint16 frame[256];
  uint16 status = 0;
	struct onebox_os_intf_operations *os_intf_ops;

	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);

	mgmt_frame = (onebox_mac_frame_t *)frame;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, 256);

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(RADIO_PARAMS_UPDATE);
	mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(BIT(0));

  if(ic->ic_txpowlimit > ic->ic_curchan->ic_maxregpower) {
    mgmt_frame->desc_word[3] |= ONEBOX_CPU_TO_LE16(ic->ic_curchan->ic_maxregpower << 8);
  }else{
    mgmt_frame->desc_word[3] |= ONEBOX_CPU_TO_LE16(ic->ic_txpowlimit << 8);
  }

	status = w_adapter->devdep_ops->onebox_send_internal_mgmt_frame(w_adapter,
			(uint16 *)mgmt_frame,
			FRAME_DESC_SZ );

}
/**
  * This function sets/change channels.  If the channel is really being changed,
  * it's done by resetting the chip.   
  * @param  Pointer to the ieee80211com  structure
  * @return void
  */
void core_set_channel(struct ieee80211com *ic, struct ieee80211vap *vap)
{
	WLAN_ADAPTER w_adapter;
	//struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct onebox_os_intf_operations *os_intf_ops;
	uint8 prev_chan_bw;
#ifndef PROGRAMMING_BBP_TA
	uint8 rf_band;
#endif
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Setting channel = %d\n"), ic->ic_curchan->ic_ieee));
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);
	if(w_adapter->fsm_state == FSM_MAC_INIT_DONE)
	{
#ifndef PROGRAMMING_BBP_TA
		if(ic->ic_curchan->ic_ieee <= 14)
		{
			rf_band = BAND_2_4GHZ;
		}
		else
		{
			rf_band = BAND_5GHZ;
		}

		if (w_adapter->operating_band != rf_band)
		{
			w_adapter->operating_band = rf_band;
			w_adapter->rf_reset = 1;
			w_adapter->devdep_ops->onebox_program_bb_rf(w_adapter);
		}
#else
		w_adapter->endpoint_params.channel = ic->ic_curchan->ic_ieee;
		prev_chan_bw = w_adapter->operating_chwidth;
		if(ic->band_flags & IEEE80211_CHAN_HT40) {
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ap40: in %s line %d 40mhz***\n"), __func__, __LINE__));
			w_adapter->operating_chwidth = BW_40Mhz;
		}else {
			/* else case added for setting operating chan width 20mhz */
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ap20: in %s line %d 20mhz***\n"), __func__, __LINE__));
			w_adapter->operating_chwidth = BW_20Mhz;	
		}
		if(prev_chan_bw != w_adapter->operating_chwidth)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("[%s][%d] : Change in operating channel BW, Need to load Boot-up Params again***\n"), __func__, __LINE__));
			onebox_load_bootup_params(w_adapter);
		}
		w_adapter->endpoint_params.per_ch_bw = w_adapter->operating_chwidth;
		if( ic->ic_curchan->ic_flags & IEEE80211_CHAN_11J ) 
			w_adapter->endpoint_params.enable_11j = 1;
		else 
			w_adapter->endpoint_params.enable_11j = 0;
		if(w_adapter->devdep_ops->onebox_band_check(w_adapter) == 1) {
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("********Sending VAP UPDATE %s Line %d\n"), __func__, __LINE__));
			vap->vap_caps(vap, 3);
		}
#endif		
		w_adapter->devdep_ops->onebox_set_channel(w_adapter, ic->ic_curchan->ic_ieee);
	}
}


/** This function switches the operating band depending on the mode settings
 * from user.
 * @param  pointer to the ieee80211com structure
 * @param  parameter to set 
 * @param  new band to set
 * @return interger on success else failure
 */
int core_set_params(struct ieee80211com *ic, int cmd, int data)
{
	WLAN_ADAPTER w_adapter ;
	struct onebox_os_intf_operations *os_intf_ops;
	int status = ONEBOX_STATUS_SUCCESS;
	struct ieee80211vap *vap = NULL;

	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Current operating band is :%d and new mode is %d\n"), w_adapter->operating_band, data));
	switch (cmd)
	{
		case IEEE80211_IOCTL_SETMODE:  
			if (w_adapter->operating_band != data)
			{
				w_adapter->operating_band = data;
				w_adapter->rf_reset = 1;
				status = w_adapter->devdep_ops->onebox_program_bb_rf(w_adapter);
				TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
				{ 
					if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
						if (vap->vap_caps) {
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("vap_caps not NULL. Hence updating VAP CAPS in %s:\n"), __func__));
							status = vap->vap_caps(vap, 3);
							if(status == ONEBOX_STATUS_FAILURE)
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to update VAP CAPS %s:\n"), __func__));
							else 
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("VAP CAPS updated %s:\n"), __func__));
						}
						else {
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("vap_caps is NULL. %s:\n"), __func__));
						}
						break;
					}
				} 
			}
			break;
		default:
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unknown cmd in %s:\n"), __func__));
			break;
	}
	return status;
}

/**This function is a callback from the 802.11 layer to update WME parameters.
  * @param  Pointer to the ieee80211com structure
  * @return  ONEBOX_STATUS_SUCCESS on success else failure
  */
int core_wme_update(struct ieee80211com *ic)
{
	int status = 0;
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;

	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);
	//: status = w_adapter->devdep_ops->onebox_send_qos_conf_frame(w_adapter, ic->ic_wme.wme_wmeChanParams); 
	
	/* To calculate the contention parameters for backoff procedure */
	/* This should be done with some locking mechanism 
	 * as this should not be done while dequeuing packets with already existing parameters
	 * or else care should be taken for hal_dequeue algo while setting these params*/ //

	/* Ideally while operating mltiple vaps, we should have dual params for both AP and STA 
	 * For now taking HOSTAP values only, even in case of STA */ //

	onebox_set_contention_vals(ic, w_adapter);
	status = onebox_load_radio_caps(w_adapter);
	return status;
}

/** This function is a callback from the 802.11 layer to update slot/preamble parameters.
 * @param  Pointer to the ieee80211com structure
 * @return  ONEBOX_STATUS_SUCCESS on success else failure
 */
void core_updateslot(struct ifnet *ifp)
{

	WLAN_ADAPTER w_adapter;
	struct ieee80211vap *vap = NULL;
	struct ieee80211com *ic = NULL;
	struct onebox_os_intf_operations *os_intf_ops;
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	
	
	w_adapter = os_intf_ops->onebox_get_priv(ifp);
	ic = &w_adapter->vap_com;
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
	{
		if( (vap != NULL) && (vap->iv_opmode == IEEE80211_M_STA) && ((vap->iv_state == IEEE80211_S_ASSOC) || (vap->iv_state == IEEE80211_S_RUN)))
		{
			if(ic->ic_flags & IEEE80211_F_SHSLOT)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("===> SHORT SLOT is enable <===\n")));
				w_adapter->slot_rx_11n = SHORT_SLOT_VALUE;
			}
			else
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("===> LONG SLOT is enable <===\n")));
				w_adapter->slot_rx_11n = LONG_SLOT_VALUE;
			}
			if(ic->ic_flags & IEEE80211_F_SHPREAMBLE)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("===> SHORT PREAMBLE is enable <===\n")));
				w_adapter->preamble_type = SHORT_PREAMBLE;
			}
			else
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("===> LONG PREAMBLE is enable <===\n")));
				w_adapter->preamble_type = LONG_PREAMBLE;
			}
			if(onebox_load_radio_caps(w_adapter) == ONEBOX_STATUS_SUCCESS)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
						(TEXT("%s: Radio Caps loaded successfully for change in capability info\n"),__FUNCTION__));
			}
		}
		else
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s vap=%p vap->state=%d opmode=%d\n"), __func__,  vap, vap->iv_state, vap->iv_opmode));
		}
	}
	return ;
}

/** This function is called whenever there is a state change in the virtual network interface
 * @param  pointer to the ieee80211vap structure
 * @param  new state change
 * @ param 
 */
int onebox_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;
	struct core_vap *core_vp;
	struct ieee80211com *vap_com = vap->iv_ic;
	enum ieee80211_state ostate;
	uint8 flag = 0;
	ostate = vap->iv_state;
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(vap_com->ic_ifp);
	core_vp = vap->hal_priv_vap->core_vp;

	if ((vap->iv_opmode == IEEE80211_M_HOSTAP) || (vap->iv_opmode == IEEE80211_M_STA))
	{
		/* Add vap to global vap array when it is RUN state only */
		/* Find the empty slot in vap array - For beacon to ppe*/
		if ((vap->hal_priv_vap->beacon_loc == 100) && (nstate == IEEE80211_S_RUN) && 
		     (vap->iv_opmode == IEEE80211_M_HOSTAP)) 
		{
			struct ieee80211_node *ni;
			struct ieee80211_beacon_offsets *bo;
			if ((nstate == IEEE80211_S_RUN) && (ostate == IEEE80211_S_INIT))
			{
				core_vp->av_newstate(vap, nstate, arg);
				flag = 1;
			}
			ni = vap->iv_bss;
			bo = &core_vp->rv_boff;
			core_vp->rv_bcbuf = w_adapter->net80211_ops->onebox_beacon_alloc(ni, bo);
			if (core_vp->rv_bcbuf == NULL) 
			{
				return ONEBOX_STATUS_FAILURE;
			}
			w_adapter->os_intf_ops->onebox_netbuf_queue_init(&core_vp->rv_mcastq);
#if 0
			/* Giving Beacon Slot to the VAP */
			for (ii = 0; ii < ONEBOX_VAPS_DEFAULT; ii++) 
			{
				if (w_adapter->hal_vap[ii].vap == NULL) 
				{
					w_adapter->hal_vap[ii].vap = vap;
					vap->hal_priv_vap->beacon_loc = ii;
					break;
				}
			}
				if (w_adapter->hal_vap[vap->hal_priv_vap->vap_id].vap == NULL) 
				{
					w_adapter->hal_vap[vap->hal_priv_vap->vap_id].vap = vap;
					vap->hal_priv_vap->beacon_loc = vap->hal_priv_vap->vap_id;
				}
#endif

			/* Set AMPDU Density */
			w_adapter->mpdu_density = vap->iv_ampdu_density;
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("CORE_MSG: MPDU Density = %d\n", w_adapter->mpdu_density));
			core_vp->curstate = IEEE80211_S_RUN;
			if (w_adapter->sc_nvaps == 0)
			{
				return ONEBOX_STATUS_FAILURE;
			}
		}

		if (nstate == IEEE80211_S_INIT)
		{
			if (ostate > IEEE80211_S_INIT) 
			{
				w_adapter->hal_vap[vap->hal_priv_vap->beacon_loc].vap = NULL;
				vap->hal_priv_vap->beacon_loc = 100;
			}
		}
	}

	if (!flag)
	{
		core_vp->av_newstate(vap, nstate, arg);
	}
	if (nstate == IEEE80211_S_RUN) 
	{
		core_vp->curstate = IEEE80211_S_RUN;
	}
	return 0;
}

int core_rate_setup(WLAN_ADAPTER w_adapter, uint8 mode)
{
	struct ieee80211com *ic = &w_adapter->vap_com;
	uint8 rs_rates[IEEE80211_RATE_MAXSIZE] = {0x02, 0x04, 0x0b,
	                                          0x16, 0x0c, 0x12,
	                                          0x18, 0x24, 0x30,
	                                          0x48, 0x60, 0x6c};
	switch (mode) 
	{
		case IEEE80211_MODE_11A:
		case IEEE80211_MODE_11NA:
			ic->ic_sup_rates[mode].rs_nrates = 8;
			w_adapter->os_intf_ops->onebox_memset(ic->ic_sup_rates[mode].rs_rates, 
			                                    0, ic->ic_sup_rates[mode].rs_nrates);
			w_adapter->os_intf_ops->onebox_memcpy(ic->ic_sup_rates[mode].rs_rates, 
			                                    &rs_rates[4], ic->ic_sup_rates[mode].rs_nrates);
			break;
		case IEEE80211_MODE_11B:
			ic->ic_sup_rates[mode].rs_nrates = 4;
			w_adapter->os_intf_ops->onebox_memset(ic->ic_sup_rates[mode].rs_rates, 0, 
			                                    ic->ic_sup_rates[mode].rs_nrates);
			w_adapter->os_intf_ops->onebox_memcpy(ic->ic_sup_rates[mode].rs_rates, 
			                                    rs_rates, ic->ic_sup_rates[mode].rs_nrates);
			break;
		case IEEE80211_MODE_11G:
			ic->ic_sup_rates[mode].rs_nrates = 12;
			w_adapter->os_intf_ops->onebox_memset(ic->ic_sup_rates[mode].rs_rates, 
			                                    0, ic->ic_sup_rates[mode].rs_nrates);
			w_adapter->os_intf_ops->onebox_memcpy(ic->ic_sup_rates[mode].rs_rates, 
			                                    rs_rates, ic->ic_sup_rates[mode].rs_nrates);
			break;
		case IEEE80211_MODE_11NG:
			ic->ic_sup_rates[mode].rs_nrates = 12;
			w_adapter->os_intf_ops->onebox_memset(ic->ic_sup_rates[mode].rs_rates, 
			                                    0, ic->ic_sup_rates[mode].rs_nrates);
			w_adapter->os_intf_ops->onebox_memcpy(ic->ic_sup_rates[mode].rs_rates, 
			                                    rs_rates, ic->ic_sup_rates[mode].rs_nrates);
			break;
		default:
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("Invalid mode %u\n", mode));
			return 0;
	}
	w_adapter->net80211_ops->onebox_setbasicrates(&ic->ic_sup_rates[mode], mode);
	return 1;
}

/** This function is used to send the beacon to HAL 
 * @param  Pointer to the driver private data structure
 * @param  Vap Id. Useful in case of Multiple vaps to determine which vap has to send the Beacon
 */ 
void vap_load_beacon(WLAN_ADAPTER w_adapter, int32 vap_id)
{
	struct ieee80211vap *vap = NULL;
	struct core_vap *core_vp;
	netbuf_ctrl_block_t *netbuf_cb;
	netbuf_ctrl_block_m_t *netbuf_cb_m;
	struct ieee80211_node *ni;
	struct ieee80211_beacon_offsets *bo;
	struct ieee80211com *ic= &w_adapter->vap_com;
	int mcast_len;
	unsigned long current_jiffies;
	vap = w_adapter->hal_vap[vap_id].vap;
	if (vap == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("\nCORE_DBG: Not loading beacon for the vap with vap_id = %d"), vap_id));
		return;
	}  

	if (((vap->iv_opmode != IEEE80211_M_HOSTAP) || (vap->iv_state != IEEE80211_S_RUN)) 
	    && !(vap->iv_state == IEEE80211_S_CSA))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\nCORE_DBG: Not loading beacon for the vap opmode = %d, state = %d"), 
		             vap->iv_opmode, vap->iv_state));
		return;
	}  

	core_vp = vap->hal_priv_vap->core_vp;
	ni = vap->iv_bss;
	if (ni == NULL) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("\nCORE_MSG: Node is null in load beacon"));
		return;
	} 
	if (core_vp->curstate != IEEE80211_S_RUN) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("\nCORE_MSG:Vap Load Beacon: Not in RUN State"));
		return;
	}
	bo = &core_vp->rv_boff;
	netbuf_cb_m = core_vp->rv_bcbuf;
	mcast_len=  w_adapter->os_intf_ops->onebox_netbuf_queue_len(&core_vp->rv_mcastq);

	if (ic->ic_apple_ie.add_apple_ie) {
		current_jiffies = jiffies;
		if ((jiffies_to_msecs(current_jiffies - ic->ic_apple_ie.jiffies)) > WAC_TIMEOUT) {
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("\n30 mins up, exiting WAC mode!"));
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("\nAPPLE IE is removed from beacons!"));
			dev_kfree_skb((struct sk_buff *)netbuf_cb_m->pkt_addr); 
			kfree(netbuf_cb_m);
			ic->ic_apple_ie.add_apple_ie = false;
			core_vp->rv_bcbuf = w_adapter->net80211_ops->onebox_beacon_alloc(ni, bo);
			if (core_vp->rv_bcbuf == NULL)
				return;

		}
	}

	if(ni->ni_chan != ic->ic_curchan) {
					
			/* Initialy AP mode has started in some channel, and later our Station 
			 * mode VAP is connected in other channel, then we need to update the 
			 * AP mode beacon according to the connected channel.
			 */
			dev_kfree_skb((struct sk_buff *)netbuf_cb_m->pkt_addr); 
			kfree(netbuf_cb_m);
			ni->ni_chan = ic->ic_curchan;
			core_vp->rv_bcbuf = w_adapter->net80211_ops->onebox_beacon_alloc(ni, bo);
			if (core_vp->rv_bcbuf == NULL)
				return;
	}

	if (w_adapter->net80211_ops->onebox_beacon_update(ni, bo, netbuf_cb_m, mcast_len)) 
	{
		//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("\nCORE_MSG: There is a change in beacon payload length"));
	}

	netbuf_cb = onebox_translate_mbuf_to_netbuf(netbuf_cb_m);
	netbuf_cb->ni    = ni;

	if (w_adapter->fsm_state == FSM_MAC_INIT_DONE) 
	{
		core_send_beacon(w_adapter, netbuf_cb, core_vp);
		kfree(netbuf_cb);
	}
	return;
}

/** This function sends the station information to the device. Events related to 
 *  station's connection, disconnection are sent to the PPE using this function.
 *  @param  Pointer to the ieee80211node structure
 *  @param  Type of the notification event to be sent to PPE
 *	@return  ONEBOX_STATUS_SUCCESS on success else ONEBOX_STATUS_FAILURE
 */
int core_send_station_info(struct ieee80211_node *ni, int notify_event)
{
	struct ieee80211vap *vap;
	struct ieee80211com *ic;
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;
	uint32 status;
	uint8_t sta_index;
	int tid, ac;
	sta_cbl_t *sta; 
	uint8 sec_chan = 0, center_freq = 0;

	if(ni == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: ni IS NULL In %s Line %d notify_event %d \n "),
							__func__, __LINE__, notify_event));
		dump_stack();
		return ONEBOX_STATUS_FAILURE;
	
	}

	vap = ni->ni_vap;
	

	if(is_vap_valid(vap) < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: VAP is Not a valid pointer In %s Line %d, So returning Recvd event is %d\n ")
						,	__func__, __LINE__, notify_event));
		dump_stack();
		return ONEBOX_STATUS_FAILURE;
	}

	ic = vap->iv_ic;
	

	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);
	if(notify_event == STA_CONNECTED)
	{
		/* Adding station update */
		/* check whether the station is already connected */
		for (sta_index = 0; sta_index < w_adapter->max_stations_supported; sta_index++)
		{
			if(!w_adapter->os_intf_ops->onebox_memcmp(w_adapter->sta[sta_index].mac_addr, ni->ni_macaddr, ETH_ALEN))
			{
					if(sta_index != ni->hal_priv_node.sta_id) {

							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
											(TEXT("ERROR: In %s Line %d The sta_id in Hal_station data structure and ni doesn't matches ni_sta_id %d hal_sta id %d\n"), 
											 __func__, __LINE__, ni->hal_priv_node.sta_id, sta_index));
							dump_stack();
							return ONEBOX_STATUS_FAILURE;

					}
				break;
			}
		}

		if(sta_index == w_adapter->max_stations_supported) /* In case if not already connected*/
		{
			/* Checking for the empty bit in station bitmap */
			for (sta_index = 0; sta_index < w_adapter->max_stations_supported; sta_index++)
			{
				if (!(w_adapter->sta_connected_bitmap[sta_index/32] & (1 << (sta_index%32))))
				{
					w_adapter->os_intf_ops->onebox_memcpy(w_adapter->sta[sta_index].mac_addr, ni->ni_macaddr, ETH_ALEN);
					w_adapter->sta_connected_bitmap[sta_index /32] |= (1 << (sta_index % 32));
					ni->hal_priv_node.sta_id = sta_index; /* Assigning station id */
					break;
				}
			}
		}

		if (sta_index == w_adapter->max_stations_supported) /* Reached max supported stations */
		{
			return ONEBOX_STATUS_FAILURE;
		}
	}
	else {

		sta_index = ni->hal_priv_node.sta_id;
		if(is_node_valid(w_adapter, ni) < 0) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("ERROR: In %s Line %d The sta_id in Hal_station data structure and ni doesn't matches ni_sta_id %d hal_sta id %d\n"), 
					 __func__, __LINE__, sta_index, w_adapter->sta[sta_index].sta_id));
			dump_stack();
			return ONEBOX_STATUS_FAILURE;
		}
		else if(notify_event == STA_DISCONNECTED)
		{
			w_adapter->sta_connected_bitmap[sta_index /32] &= ~(1 << (sta_index % 32));
			w_adapter->os_intf_ops->onebox_memset(w_adapter->sta[sta_index].mac_addr, 0, ETH_ALEN);
		}
		else if(notify_event == STA_ADDBA_DONE)
		{

			tid = ni->hal_priv_node.tidnum;
			ac = TID_TO_WME_AC(tid);
			if(ni->ni_tx_ampdu[tid].txa_flags & IEEE80211_AGGR_RUNNING)
			{
				ni->hal_priv_node.tid[tid].seq_start = (ni->ni_txseqs[tid] & 0xFFF);
				ni->hal_priv_node.tid[tid].baw_size = w_adapter->aggr_limit.tx_limit;
				//ni->hal_priv_node.tid[tid].baw_size = ni->ni_tx_ampdu[ac].txa_wnd;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s and tid =%d seq_start=%d baw_size = %d\n"),__func__, tid, (ni->ni_txseqs[tid] & 0xFFF), ni->hal_priv_node.tid[tid].baw_size));
			}
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s and %d tid %d seq_start %d \n"), __func__, __LINE__, tid, ni->hal_priv_node.tid[tid].seq_start));
		}
		else if((notify_event == STA_DELBA) || (notify_event == STA_RX_DELBA) || (notify_event == STA_RX_ADDBA_DONE))
		{
			tid = ni->hal_priv_node.tidnum;
		}
		else
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: Not handling this event %d\n "), notify_event));
			dump_stack();
			return ONEBOX_STATUS_FAILURE;
		}
	}
	sta = &(w_adapter->sta[ni->hal_priv_node.sta_id]); 

	if(notify_event == STA_CONNECTED)
	{
		sta->ni = ni;
		/* Storing Sta_mode Sta_id and VAP_ID for refernce in HAL */
		if(vap->iv_opmode == IEEE80211_M_STA) {
				w_adapter->sta_mode.sta_id = ni->hal_priv_node.sta_id;
				w_adapter->sta_mode.vap_id = vap->hal_priv_vap->vap_id;
		}
	}
	else if(notify_event == STA_DISCONNECTED)
	{
		sta->ni = NULL;
	}

#ifdef HT_SUPPORT
	if (ni->ni_htcap & ONEBOX_BIT(11))
	{
		sta->max_amsdu_size = 7935;
	}
	else
	{
		sta->max_amsdu_size = 3839;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\nCORE_MSG: Max AMSDU Size = %d"), sta->max_amsdu_size));
#endif

	status = w_adapter->devdep_ops->onebox_hal_send_sta_notify_frame(w_adapter, ni, notify_event);
	if (status) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\nCORE_MSG:Sending STA notification failed")));
		return ONEBOX_STATUS_FAILURE;
	}
/*: IN case if there is a band change then BB/RF programming  will
 * be initiated from driver, but in this case the fsm state gets moved to something
 * which is not mac_init_done. So we need to differentiate which sequece it is eg init/scan seq
 * By checking this state we will move to fsm_mac_init_done */

	/* If VAP is in STA mode, send new assoication to rate control module */
	if ((vap->iv_opmode == IEEE80211_M_STA) && (notify_event == STA_CONNECTED)) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("40Mhz: In %s Line %d ni->ni_chan.ic_flags = %02x ic->ic_curchan->ic_flags = %02x\n"), __func__, __LINE__, ni->ni_chan->ic_flags, ic->ic_curchan->ic_flags));
		if(ic->band_flags & IEEE80211_CHAN_HT40)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s %d calling chwidth change here \n"), __func__, __LINE__));
			/* After Assoc ch width changed to 40Mhz */
#ifndef PROGRAMMING_BBP_TA
			w_adapter->operating_chwidth = BW_40Mhz;
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d operating chwidth = %d def_chwidth = %d\n"), __func__, __LINE__,w_adapter->operating_chwidth,w_adapter->def_chwidth));
			//if(w_adapter->operating_chwidth != w_adapter->def_chwidth)
			{
				if(onebox_load_bootup_params(w_adapter) == ONEBOX_STATUS_SUCCESS)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
					             (TEXT("%s: BOOTUP Parameters loaded successfully for band switch\n"),__FUNCTION__));
				}
				if(onebox_load_radio_caps(w_adapter) == ONEBOX_STATUS_SUCCESS)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
							(TEXT("%s: Radio Caps loaded successfully for band switch\n"),__FUNCTION__));
				}	
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Enabling band change and program bbrf\n")));
				w_adapter->chw_flag = 1;
				w_adapter->devdep_ops->onebox_program_bb_rf(w_adapter);
				w_adapter->def_chwidth = w_adapter->operating_chwidth;
			}
#else
			w_adapter->endpoint_params.channel = ic->ic_curchan->ic_ieee;
			w_adapter->endpoint_params.per_ch_bw = BW_40Mhz;
			if( ic->ic_curchan->ic_flags & IEEE80211_CHAN_11J ) 
				w_adapter->endpoint_params.enable_11j = 1;
			else 
				w_adapter->endpoint_params.enable_11j = 0;
			if(w_adapter->devdep_ops->onebox_band_check(w_adapter) == 1) {
        			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("********Sending VAP UPDATE %s Line %d\n"), __func__, __LINE__));
        			vap->vap_caps(vap, 3);
			}
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d operating chwidth = %d def_chwidth = %d\n"), __func__, __LINE__,w_adapter->endpoint_params.per_ch_bw,w_adapter->def_chwidth));
			w_adapter->def_chwidth = w_adapter->operating_chwidth;
#endif			
			{
				/*40mhz connection */
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%dSetting channel = %d ic_flags =%x\n"), __LINE__, ic->ic_curchan->ic_ieee, ic->ic_curchan->ic_flags));
				if(ic->band_flags & IEEE80211_CHAN_HT40U)
				{
					/*secondary channel is above the primary channel */
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("40mhz: Secondary channel is above the primary channel\n")));
					sec_chan = (ic->ic_curchan->ic_ieee + 4);
					center_freq = ((sec_chan + ic->ic_curchan->ic_ieee)/2);
				}
				else if(ic->band_flags & IEEE80211_CHAN_HT40D)
				{
					/*secondary channel is below the primary channel */
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("40mhz: Secondary channel is below the primary channel\n")));
					sec_chan = (ic->ic_curchan->ic_ieee - 4);
					center_freq = ((sec_chan + ic->ic_curchan->ic_ieee)/2);
				}
			}
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%dSecondary channel = %d cent_freq=%d\n"), __LINE__, sec_chan, center_freq));
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d operating chwidth = %d cur chan=%d\n"), __func__, __LINE__,w_adapter->operating_chwidth, ic->ic_curchan->ic_ieee));
			w_adapter->devdep_ops->onebox_set_channel(w_adapter, center_freq);
		}
	}
	else if ((vap->iv_opmode == IEEE80211_M_STA) && (notify_event == STA_DISCONNECTED)) 
	{
		/*: Clearing this flag because HT40 flag is set in HTParsing code eventhough AP is not supporting 40mhz
			*HT parsing code in net80211  needs to be revisited.
			* */
		ic->ic_curchan->ic_flags &= ~IEEE80211_CHAN_HT40;
		ic->band_flags &= ~IEEE80211_CHAN_HT40;
		if(w_adapter->operating_chwidth == BW_40Mhz)
		{
#ifndef PROGRAMMING_BBP_TA
			w_adapter->operating_chwidth = w_adapter->def_chwidth = BW_20Mhz;
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d operating chwidth = %d def_chwidth = %d\n", __func__, __LINE__,w_adapter->operating_chwidth,w_adapter->def_chwidth)));
			if(onebox_load_bootup_params(w_adapter) == ONEBOX_STATUS_SUCCESS)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
				             (TEXT("%s: BOOTUP Parameters loaded successfully for band switch\n"),__FUNCTION__));
			}
			if(onebox_load_radio_caps(w_adapter) == ONEBOX_STATUS_SUCCESS)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
				             (TEXT("%s: Radio Caps loaded successfully for band switch\n"),__FUNCTION__));
			}
			w_adapter->chw_flag = 1;
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Enabling band change and program bbrf\n")));
			w_adapter->devdep_ops->onebox_program_bb_rf(w_adapter);
#else
			w_adapter->endpoint_params.channel = ic->ic_curchan->ic_ieee;
			w_adapter->endpoint_params.per_ch_bw = BW_20Mhz;
			if( ic->ic_curchan->ic_flags & IEEE80211_CHAN_11J ) 
				w_adapter->endpoint_params.enable_11j = 1;
			else 
				w_adapter->endpoint_params.enable_11j = 0;
			if(w_adapter->devdep_ops->onebox_band_check(w_adapter) == 1) {
        			vap->vap_caps(vap, 3); /*sending vap update frame */
			}
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d operating chwidth = %d def_chwidth = %d\n"), __func__, __LINE__,w_adapter->endpoint_params.per_ch_bw,w_adapter->def_chwidth));
			w_adapter->def_chwidth = w_adapter->operating_chwidth;
			w_adapter->devdep_ops->onebox_set_channel(w_adapter, ic->ic_curchan->ic_ieee);
#endif			
		}
	}
  
	return ONEBOX_STATUS_SUCCESS;
} 

int onebox_key_alloc(struct ieee80211vap *vap, 
                     struct ieee80211_key *k,ieee80211_keyix *keyix,
                     ieee80211_keyix *rxkeyix)
{
	struct onebox_os_intf_operations *os_intf_ops;
	WLAN_ADAPTER w_adapter;
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(vap->iv_ic->ic_ifp);

	/*
	 * Group key allocation must be handled specially for
	 * parts that do not support multicast key cache search
	 * functionality.  For those parts the key id must match
	 * the h/w key index so lookups find the right key.  On
	 * parts w/ the key search facility we install the sender's
	 * mac address (with the high bit set) and let the hardware
	 * find the key w/o using the key id.  This is preferred as
	 * it permits us to support multiple users for adhoc and/or
	 * multi-station operation.
	 */
	w_adapter->sc_mcastkey = 0;
#ifdef CONFIG_11W
	if (!strcmp(k->wk_cipher->ic_name, "AES-CMAC")) {
		int32 i;
		uint32 keyidx = IEEE80211_KEYIX_NONE;
		for (i = 0; i < 256; i++) 
		{
			if (k == &vap->iv_igtk_keys[i]) 
			{
				keyidx = i;
				break;
			}
		}
		if (keyidx == IEEE80211_KEYIX_NONE) 
		{
			/* should not happen */
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\nCORE_MSG: bogus group key")));
			return IEEE80211_KEYIX_NONE;
		}
		/*
		 * XXX we pre-allocate the global keys so
		 * have no way to check if they've already been allocated.
		 */
		*keyix = keyidx;
		++keyidx;
		return keyidx;
	}
#endif
	if ((k->wk_flags & IEEE80211_KEY_GROUP) && !w_adapter->sc_mcastkey) 
	{
		int32 i;
		uint32 keyidx = IEEE80211_KEYIX_NONE;
		for (i = 0; i < IEEE80211_WEP_NKID; i++) 
		{
			if (k == &vap->iv_nw_keys[i]) 
			{
				keyidx = i;
				break;
			}
		}
		if (keyidx == IEEE80211_KEYIX_NONE) 
		{
			/* should not happen */
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\nCORE_MSG: bogus group key")));
			return IEEE80211_KEYIX_NONE;
		}
		/*
		 * XXX we pre-allocate the global keys so
		 * have no way to check if they've already been allocated.
		 */
		*keyix = keyidx;
		++keyidx;
		return keyidx;
	}
	/*
	 * We allocate two pair for TKIP when using the h/w to do
	 * the MIC.  For everything else, including software crypto,
	 * we allocate a single entry.  Note that s/w crypto requires
	 * a pass-through slot on the 5211 and 5212.  The 5210 does
	 * not support pass-through cache entries and we map all
	 * those requests to slot 0.
	 *
	 * Allocate 1 pair of keys for WEP case. Make sure the key
	 * is not a shared-key.
	 */
	return 1;
}

/*
 * Delete an entry in the key cache allocated by wl1650_key_alloc.
 */
int onebox_key_delete(struct ieee80211vap *vap, 
                      const struct ieee80211_key *k)
{
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;
	//const struct ieee80211_cipher *cip = k->wk_cipher;
	uint32 keyix = k->wk_keyix;
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(vap->iv_ic->ic_ifp);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("\nCORE_MSG: delete key %u", keyix));
	if(vap->iv_opmode == IEEE80211_M_HOSTAP)
	{
		if(vap->iv_state != IEEE80211_S_RUN){
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,("\nCORE_MSG: delete key in AP mode with key index %u\n", keyix));
			w_adapter->devdep_ops->onebox_hal_load_key(w_adapter, NULL, 0, 0, ONEBOX_GROUP_KEY, 0, IEEE80211_CIPHER_NONE, vap);    //Sending Frame to Clear Group Keys to LMAC in AP mode.
		}
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,("\nCORE_MSG: delete key for STA with key index %u\n", keyix));
		w_adapter->devdep_ops->onebox_hal_load_key(w_adapter, NULL, 0, 0, ONEBOX_GROUP_KEY, 0, IEEE80211_CIPHER_NONE, vap);    //Sending Frame to Clear Group Keys to LMAC in STA mode.
	}

	return 1;
}

static int onebox_hal_keyset(WLAN_ADAPTER w_adapter, 
                      const struct ieee80211_key *k,
                      struct ieee80211vap *vap, struct ieee80211_node *ni_sta, 
                      uint32 cipher)
{
	u16 node_id;
	u32 ret =0; 
	uint8 key_buf[32];
	uint8 key_type;
	uint16 zero = 0;
	u32 t;
	uint16 key_len;
	uint32 vap_id = vap->hal_priv_vap->vap_id;

	struct ieee80211com *ic;

	ic = &w_adapter->vap_com;

	node_id = ni_sta?ni_sta->hal_priv_node.sta_id:0xff;
	vap->hal_priv_vap->cip_type = cipher;

	key_len = k->wk_keylen;

	if (((k->wk_flags & IEEE80211_KEY_XMIT)
							||(k->wk_flags & IEEE80211_KEY_RECV))
		&& (node_id != 0xff) 
		&& (cipher != IEEE80211_CIPHER_WEP))
	{
		w_adapter->os_intf_ops->onebox_memcpy(key_buf,k->wk_key,key_len);
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, key_buf, key_len);
		if(cipher == IEEE80211_CIPHER_TKIP)
		{
			key_len += IEEE80211_MICBUF_SIZE;
			w_adapter->os_intf_ops->onebox_memcpy(&key_buf[16],&k->wk_key[16],IEEE80211_MICBUF_SIZE);
		}
		key_type = ONEBOX_PAIRWISE_KEY;
	} 
	else 
	{  
		int ix = 0;
		if(cipher == IEEE80211_CIPHER_WEP)
		{
			key_buf[ix++] = 0;
		}
		w_adapter->os_intf_ops->onebox_memcpy(&key_buf[ix],k->wk_key,key_len);
		if(cipher == IEEE80211_CIPHER_TKIP)
		{
			key_len += IEEE80211_MICBUF_SIZE;
		}
		for (t=0; t<key_len; t++) 
		{
			if(k->wk_key[t] == 0)
			{
				zero++;
			}
		}
		if(zero == key_len)
		{
			return 0;
		}
		if(cipher == IEEE80211_CIPHER_TKIP)
		{
			w_adapter->os_intf_ops->onebox_memcpy(&key_buf[16],&k->wk_key[16],IEEE80211_MICBUF_SIZE);
		}
		key_type = ONEBOX_GROUP_KEY;
		if(cipher == IEEE80211_CIPHER_WEP)
		{
			key_len++;
		}
	}

	if(node_id == 0xff)
	{
		if (vap->iv_opmode == IEEE80211_M_STA) {
			node_id = w_adapter->sta_mode.sta_id;
		} else {
			node_id = vap_id;
		}
	}
	w_adapter->sec_mode[vap_id] = cipher;

	if(k->wk_flags & IEEE80211_KEY_XMIT)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s and %d \n"), __func__, __LINE__));
    if(cipher == IEEE80211_CIPHER_WEP) 
      w_adapter->wep_key_idx[vap_id]  = k->wk_keyix;
		vap->hal_priv_vap->key_idx = k->wk_keyix;
	}

	if(cipher == IEEE80211_CIPHER_WEP)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s and %d \n"), __func__, __LINE__));
		if(vap->iv_opmode == IEEE80211_M_HOSTAP)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s and %d \n"), __func__, __LINE__));
			w_adapter->devdep_ops->onebox_hal_load_key(w_adapter, key_buf, key_len, node_id, key_type, k->wk_keyix, cipher, vap);
		}
	  w_adapter->wep_keylen[vap_id][k->wk_keyix] = key_len;
		w_adapter->os_intf_ops->onebox_memcpy(&w_adapter->wep_key[vap_id][k->wk_keyix][0],key_buf,key_len); //selecting proper key_index for WEP
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)&w_adapter->wep_key, 128);
		return 0;
	}
	ret = w_adapter->devdep_ops->onebox_hal_load_key(w_adapter, key_buf, key_len, node_id, key_type,k->wk_keyix, cipher, vap);
	if(key_type == ONEBOX_PAIRWISE_KEY)
	{
		if(!ret)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Enable encryption as Pair wise key loading is successfull, ni_flags %x \n"), ni_sta->ni_flags));
			ni_sta->ni_flags |= IEEE80211_NODE_ENCRYPT_ENBL;
#ifdef CONFIG_11W
			ni_sta->hal_priv_node.conn_in_prog = 0;
#endif
		}
#if defined(RSI_CCX) || defined(CONFIG_11R)
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Sending block to Unblock\n")));
		vap->hal_priv_vap->conn_in_prog = 0;
		onebox_send_block_unblock(vap, STA_CONNECTED, 0);	
		if(vap->hal_priv_vap->IAPP_IND_FLAG)
		{
			ret = send_ap_adjacent_report(w_adapter, vap);
			if(ret != ONEBOX_STATUS_SUCCESS)
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Err: Sending AP adjacent report failed in %s Line %d\n"), __func__, __LINE__));
			vap->hal_priv_vap->IAPP_IND_FLAG = 0;
		}
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("calling timeout initialziation In %s Line %d\n"), __func__, __LINE__));
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Resting ptk ket variable in %s Line %d \n"), __func__, __LINE__));
		/***Reset ptk key as we dont have any key exchanges in ccx roaming*/
		w_adapter->sta_mode.ptk_key = 0; 
		update_pwr_save_status(vap, PS_ENABLE, CONNECTED_PATH);
#endif 			
	}
	return ret;
}

/*
 * Block/unblock tx+rx processing while a key change is done.
 * We assume the caller serializes key management operations
 * so we only need to worry about synchronization with other
 * uses that originate in the driver.
 */
void onebox_key_update_begin(struct ieee80211vap *vap)
{
	return;
}

void onebox_key_update_end(struct ieee80211vap *vap)
{
	return;
}

/*
 * Set a net80211 key into the hardware.  This handles the
 * potential distribution of key state to multiple key
 * cache slots for TKIP with hardware MIC support.
 */
int onebox_key_set(struct ieee80211vap *vap, 
                   const struct ieee80211_key *k,
                   const uint8 mac0[IEEE80211_ADDR_LEN])
{
#define N(a)    ((int)(sizeof(a)/sizeof(a[0])))
				const u_int8_t ciphermap[] = { HAL_CIPHER_WEP,      /* IEEE80211_CIPHER_WEP */
								HAL_CIPHER_TKIP,     /* IEEE80211_CIPHER_TKIP */
								HAL_CIPHER_AES_OCB,  /* IEEE80211_CIPHER_AES_OCB */
								HAL_CIPHER_AES_CCM,  /* IEEE80211_CIPHER_AES_CCM */
								(uint8) -1,          /* 4 is not allocated */
								HAL_CIPHER_CKIP,     /* IEEE80211_CIPHER_CKIP */
								HAL_CIPHER_CLR,      /* IEEE80211_CIPHER_NONE */
				};
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;
	struct ieee80211com *ic = NULL;
	struct ieee80211_node_table *nt;
	const struct ieee80211_cipher *cip = k->wk_cipher;
	struct ieee80211_node *ni;
	uint8 gmac[IEEE80211_ADDR_LEN];
	const uint8_t *mac;
	uint8_t vap_id = 0;
	HAL_KEYVAL hk;
	struct ieee80211_node *bss = NULL;
#ifdef CONFIG_11W
	const struct ieee80211_cipher *cipher = k->wk_cipher;
#endif

#ifdef CONFIG_11W
	if (!strcmp(cipher->ic_name, "AES-CMAC")) {
		return 0;
	}
#endif

	if(is_vap_valid(vap) < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: VAP is Not a valid pointer In %s Line %d vap %p\n "), 
					__func__, __LINE__, vap));
		dump_stack(); 
		return ONEBOX_STATUS_FAILURE;
	}

	ic = vap->iv_ic;
	bss = vap->iv_bss;
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);

	w_adapter->os_intf_ops->onebox_memset(&hk, 0, sizeof(hk));
	/*
	 * Software crypto uses a "clear key" so non-crypto
	 * state kept in the key cache are maintained and
	 * so that rx frames have an entry to match.
	 */
	if ((k->wk_flags & IEEE80211_KEY_SWCRYPT) == 0)
	{
		//KASSERT(cip->ic_cipher < N(ciphermap),
		//    ("invalid cipher type %u", cip->ic_cipher));
		hk.kv_type = ciphermap[cip->ic_cipher];
		hk.kv_len = k->wk_keylen;
		w_adapter->os_intf_ops->onebox_memcpy(hk.kv_val, k->wk_key, k->wk_keylen);
	}
	else
	{
		hk.kv_type = HAL_CIPHER_CLR;
	}

	if ((k->wk_flags & IEEE80211_KEY_GROUP) && w_adapter->sc_mcastkey)
	{
		/*
		 * Group keys on hardware that supports multicast frame
		 * key search use a mac that is the sender's address with
		 * the high bit set instead of the app-specified address.
		 */
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ERROR: Group key and sc_mcast key installed %02x %02x %02x %02x %02x %02x\n"),
						bss->ni_macaddr[0],
						bss->ni_macaddr[1],
						bss->ni_macaddr[2],
						bss->ni_macaddr[3],
						bss->ni_macaddr[4],
						bss->ni_macaddr[5]
						));
		IEEE80211_ADDR_COPY(gmac, bss->ni_macaddr);
		gmac[0] |= 0x80;
		mac = gmac;
	}
	else
	{
		mac = mac0;
	}

	nt = &ic->ic_sta;

	IEEE80211_NODE_LOCK(nt);
#ifdef IEEE80211_DEBUG_REFCNT
	ni = w_adapter->net80211_ops->onebox_find_node_locked_debug(nt, mac, func, line);
#else
	ni = w_adapter->net80211_ops->onebox_find_node_locked(nt, mac);
#endif
	IEEE80211_NODE_UNLOCK(nt);

	if (ni == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("\nCORE_MSG:ni is null"));
	}

	if((is_node_valid(w_adapter, ni) < 0) && !((mac0[0] == 0xff) && (mac0[1] == 0xff) && (mac0[5] == 0xff))) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: The received node ni is not valid In %s Line %d Unable to install key for specified node\n ")
					, __func__, __LINE__));
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: The Mac adress recvd for installing KEY is  ")));
		onebox_print_mac_address(w_adapter, (uint8_t *)mac0);
		dump_stack();
		return ONEBOX_STATUS_FAILURE;
	}


	vap_id = vap->hal_priv_vap->vap_id;

	return onebox_hal_keyset(w_adapter, k, vap, ni, 
	                         k->wk_cipher->ic_cipher);
#undef N
}

/** This function is used to detach the net80211 state machine
 * @param  Pointer to the driver private data structure
 * @return void
 */
void core_net80211_detach(WLAN_ADAPTER w_adapter)
{
	struct ieee80211com *ic = &w_adapter->vap_com;

	if(w_adapter->init_net80211_done == 1) {
		w_adapter->net80211_ops->onebox_ifdetach(ic);
		w_adapter->init_net80211_done = 0;
	}
#ifdef ONEBOX_CONFIG_CFG80211
	if (ic->cfg_priv && !rtnl_is_locked())
		cfg80211_detach(ic->cfg_priv);
#endif

}
EXPORT_SYMBOL(core_net80211_detach);

/* This function is used to set the region domain based on the
 * user selection. The channel list should be populated based on this
 * selection
 * @param : pointer to the ieee80211com structure
*/
uint32_t set_region(struct ieee80211com *ic, uint32_t country_value)

{

	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;
	struct driver_assets *d_assets;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;

	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);
	d_assets = w_adapter->d_assets;

	switch(country_value)
	{
		/* Countries in US Region */
		case CTRY_CANADA:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';		
			ic->ic_regdomain.isocc[0] = 'C';		
			ic->ic_regdomain.isocc[1] = 'A';		
			ic->ic_regdomain.pad[0] = FCC;			
			break;
		case CTRY_MEXICO:				
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'M';	
			ic->ic_regdomain.isocc[1] = 'X';	
			ic->ic_regdomain.pad[0] = FCC;			
			break;
DEFAULT_CTRY:
		case CTRY_UNITED_STATES:  
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'U';
			ic->ic_regdomain.isocc[1] = 'S';
			ic->ic_regdomain.pad[0] = FCC;			
			break;

			/* Countries in EU Region */
		case CTRY_BELGIUM:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'B';		
			ic->ic_regdomain.isocc[1] = 'E';		
			ic->ic_regdomain.pad[0] = ETSI;			//Just an indication of the country, to refer index in regdm_table[]
			break;
		case CTRY_FRANCE:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';	
			ic->ic_regdomain.isocc[0] = 'F';	
			ic->ic_regdomain.isocc[1] = 'R';	
			ic->ic_regdomain.pad[0] = ETSI;			
			break;
		case CTRY_GERMANY:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';	
			ic->ic_regdomain.isocc[0] = 'D';	
			ic->ic_regdomain.isocc[1] = 'E';	
			ic->ic_regdomain.pad[0] = ETSI;			
			break;
		case CTRY_ITALY:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'I';	
			ic->ic_regdomain.isocc[1] = 'T';	
			ic->ic_regdomain.pad[0] = ETSI;			
			break;

			/* Countries in Japan Region */
		case CTRY_JAPAN:
			if (ic->module_model_type != 0x201) {
				ic->ic_regdomain.country = country_value;
				ic->ic_regdomain.location = ' ';	
				ic->ic_regdomain.isocc[0] = 'J';	
				ic->ic_regdomain.isocc[1] = 'P';
				ic->ic_regdomain.pad[0] = TELEC;			
				break;
			} else {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Current region %d is not supported by the Module %02x Defaulting to US Region\n"),
							country_value, ic->module_model_type));
				country_value = CTRY_UNITED_STATES;
				status = ONEBOX_STATUS_FAILURE;
				goto DEFAULT_CTRY;
			}

		case CTRY_CHINA:
			if (ic->module_model_type == 0x03 || (ic->module_model_type == 0x301)) {
				ic->ic_regdomain.country = country_value;
				ic->ic_regdomain.location = ' ';
				ic->ic_regdomain.isocc[0] = 'C';
				ic->ic_regdomain.isocc[1] = 'N';
				ic->ic_regdomain.pad[0] = SRRC;
			} else {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Current region %d is not supported by the Module %02x Defaulting to US Region\n"),
							country_value, ic->module_model_type));
				country_value = CTRY_UNITED_STATES;
				status = ONEBOX_STATUS_FAILURE;
				goto DEFAULT_CTRY;
			}
			break;
			/* TAIWAN Region */
		case CTRY_TAIWAN:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'T';
			ic->ic_regdomain.isocc[1] = 'W';
			ic->ic_regdomain.pad[0] = TAIWAN;
			break;
			/* KOREA Regions*/

		case CTRY_KOREA_NORTH:
			if (ic->module_model_type == 0x201) {
				ic->ic_regdomain.country = country_value;
				ic->ic_regdomain.location = ' ';
				ic->ic_regdomain.isocc[0] = 'K';
				ic->ic_regdomain.isocc[1] = 'P';
				ic->ic_regdomain.pad[0] = KCC;
				break;
			} else {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Current region %d is not supported by the Module %02x Defaulting to US Region\n"),
							country_value, ic->module_model_type));
				country_value = CTRY_UNITED_STATES;
				status = ONEBOX_STATUS_FAILURE;
				goto DEFAULT_CTRY;
			}
			/* South Korea*/
		case CTRY_KOREA_ROC:
		case CTRY_KOREA_ROC2:
		case CTRY_KOREA_ROC3:
			if (ic->module_model_type == 0x201) {
				ic->ic_regdomain.country = country_value;
				ic->ic_regdomain.location = ' ';
				ic->ic_regdomain.isocc[0] = 'K';
				ic->ic_regdomain.isocc[1] = 'R';
				ic->ic_regdomain.pad[0] = KCC;
				break;
			} else {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Current region %d is not supported by the Module %02x Defaulting to US Region\n"),
							country_value, ic->module_model_type));
				country_value = CTRY_UNITED_STATES;
				status = ONEBOX_STATUS_FAILURE;
				goto DEFAULT_CTRY;
			}
			
			/* Countries in Rest of the World Region */

		case CTRY_AUSTRALIA:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'A';	
			ic->ic_regdomain.isocc[1] = 'U';	
			ic->ic_regdomain.pad[0] = WORLD;			
			break;
		case CTRY_INDIA:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.location = ' ';	
			ic->ic_regdomain.isocc[0] = 'I';	
			ic->ic_regdomain.isocc[1] = 'N';	
			ic->ic_regdomain.pad[0] = WORLD;			
			break;
		case CTRY_IRAN:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'I';	
			ic->ic_regdomain.isocc[1] = 'R';	
			ic->ic_regdomain.pad[0] = WORLD;			
			break;
		case CTRY_MALAYSIA:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'M';	
			ic->ic_regdomain.isocc[1] = 'Y';	
			ic->ic_regdomain.pad[0] = WORLD;			
			break;
		case CTRY_NEW_ZEALAND:			
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'N';	
			ic->ic_regdomain.isocc[1] = 'Z';	
			ic->ic_regdomain.pad[0] = WORLD;			
			break;
		case CTRY_RUSSIA:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'R';	
			ic->ic_regdomain.isocc[1] = 'S';	
			ic->ic_regdomain.pad[0] = WORLD;			
			break;
		case CTRY_SINGAPORE:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'S';	
			ic->ic_regdomain.isocc[1] = 'G';	
			ic->ic_regdomain.pad[0] = WORLD;			
			break;
		case CTRY_SOUTH_AFRICA:
			ic->ic_regdomain.country = country_value;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'S';	
			ic->ic_regdomain.isocc[1] = 'A';	
			ic->ic_regdomain.pad[0] = WORLD;			
			break;
		default:  
			ic->ic_regdomain.country = CTRY_DEFAULT;
			ic->ic_regdomain.location = ' ';
			ic->ic_regdomain.isocc[0] = 'W';
			ic->ic_regdomain.isocc[1] = 'R';
			ic->ic_regdomain.pad[0] = WORLD;			//Just an indication of the country, to refer index in regdm_table[]
			break;
	}
	return status;

}

ONEBOX_STATUS onebox_send_auto_rate_request (WLAN_ADAPTER w_adapter, struct ieee80211_node *ni )
{
	onebox_mac_frame_t *mgmt_frame;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	uint8	 status; 
	int32  ii;
	uint16 jj = 0;
	uint16 kk = 0;
	uint8 p = 0;
	uint16 min_rate = 0;
	uint16 min_mcs_rate=0;
	uint8 remaining;
	struct ieee80211_rateset rs = ni->ni_rates;
	struct ieee80211_htrateset htrs = ni->ni_htrates;
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211vap *vap = ni->ni_vap;

	uint8 ht_rate_bitmap = 0;
	uint8 mm, band_check = 0;

	short mcs_rates[] = {RSI_RATE_MCS0, RSI_RATE_MCS1, RSI_RATE_MCS2,
			RSI_RATE_MCS3, RSI_RATE_MCS4, RSI_RATE_MCS5, RSI_RATE_MCS6, 
			RSI_RATE_MCS7};

	uint16 mcs[] = {13, 26, 39, 52, 78, 104, 117, 130};
//	uint16 mcs_1[] = {13, 26, 39, 52, 78, 104, 117, 130};

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);

	w_adapter->auto_rate_params.failure_limit      = ONEBOX_CPU_TO_LE16(3); 
	w_adapter->auto_rate_params.Initial_boundary   = ONEBOX_CPU_TO_LE16(3);
	w_adapter->auto_rate_params.max_threshold_limt = ONEBOX_CPU_TO_LE16(27);

	mgmt_frame->u.auto_rate.aarf_rssi = ONEBOX_CPU_TO_LE16((((uint16)3 << 6) | (uint16)(18 & 0x3f)));
	mgmt_frame->u.auto_rate.moderate_rate_inx = (((rs.rs_nrates + htrs.rs_nrates)/2) - 1) ; 
	mgmt_frame->u.auto_rate.moderate_rate_inx  = ONEBOX_CPU_TO_LE16(mgmt_frame->u.auto_rate.moderate_rate_inx); 
	mgmt_frame->u.auto_rate.collision_tolerance = ONEBOX_CPU_TO_LE16(3);
	/* collision error rate allowable at stage 2 */
	w_adapter->os_intf_ops->onebox_memcpy((PVOID)&mgmt_frame->u.auto_rate.failure_limit,
						(const void *)&w_adapter->auto_rate_params,
						sizeof(AUTO_RATE_PARAMS));
	mgmt_frame->u.auto_rate.num_supported_rates = (rs.rs_nrates + htrs.rs_nrates) * 2;
	 mgmt_frame->u.auto_rate.num_supported_rates = ONEBOX_CPU_TO_LE16(mgmt_frame->u.auto_rate.num_supported_rates);  

	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(AUTO_RATE_IND);
	if(ic->band_flags & IEEE80211_CHAN_HT40) {
					if (ni->ni_htcap & IEEE80211_HTCAP_CHWIDTH40) 
					{
									ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("AUT0_RATE: Node is capable of 40Mhz\n")));
									mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16((uint16)1);
					}
	}
	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16((uint16)ni->hal_priv_node.sta_id << 8);

	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("In %s Line %d Supported rates = %d \n"), __func__, __LINE__, rs.rs_nrates));	

	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)rs.rs_rates, rs.rs_nrates);

	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("Supported ht rates ni %s:%d\n"), __func__, __LINE__));	
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_AUTORATE, (PUCHAR)htrs.rs_rates, htrs.rs_nrates);

  if (htrs.rs_nrates) {
    if (htrs.rs_rates[0] & 0x80)
      band_check = 0x80;
    else
      band_check = 0x00;
  }
	
	/*mapping the HT rates*/
	for (mm = 0; mm < htrs.rs_nrates; mm++) {
		if (htrs.rs_rates[mm] == (STD_RATE_MCS0 | band_check))
			ht_rate_bitmap |= BIT(0);
		else if (htrs.rs_rates[mm] == (STD_RATE_MCS1 | band_check))
			ht_rate_bitmap |= BIT(1);						
		else if (htrs.rs_rates[mm] == (STD_RATE_MCS2 | band_check))
			ht_rate_bitmap |= BIT(2);						
		else if (htrs.rs_rates[mm] == (STD_RATE_MCS3 | band_check))
			ht_rate_bitmap |= BIT(3);						
		else if (htrs.rs_rates[mm] == (STD_RATE_MCS4 | band_check))
			ht_rate_bitmap |= BIT(4);						
		else if (htrs.rs_rates[mm] == (STD_RATE_MCS5 | band_check))
			ht_rate_bitmap |= BIT(5);						
		else if (htrs.rs_rates[mm] == (STD_RATE_MCS6 | band_check))
			ht_rate_bitmap |= BIT(6);						
		else if (htrs.rs_rates[mm] == (STD_RATE_MCS7 | band_check))
			ht_rate_bitmap |= BIT(7);						
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("rate bitmap is %d\n"), ht_rate_bitmap));

	for(ii = 0; ii < rs.rs_nrates; ii++)
	{
		rs.rs_rates[ii] = (rs.rs_rates[ii] & 0x7F);/* In 5Ghz the basic rates will have the 8th bit to announce the basic rates */
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("<=== Supported rates after masking the 8th bit ===>\n")));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_AUTORATE, (PUCHAR)rs.rs_rates, rs.rs_nrates);

	if(rs.rs_nrates == 12)	/*station is 11bg supported*/
	{
		min_rate = RSI_RATE_1;
	}
	else if(rs.rs_nrates == 0 && htrs.rs_nrates == 8)	/*station is 11n only supported*/
	{
		min_rate = RSI_RATE_MCS0;
	}
	else if(w_adapter->operating_band == BAND_5GHZ && rs.rs_nrates == 8)			
	{
		min_rate = ONEBOX_CPU_TO_LE16(RSI_RATE_6);
	}
	else	/* sorting through the rates to get the min_rate*/
	{
		jj = rs.rs_rates[0];
		for (ii = 0; ii < rs.rs_nrates; ii++) {
			if (jj > rs.rs_rates[ii])
				jj = rs.rs_rates[ii];
		}
		for (ii = 0; ii < MAX_BG_SUPP_RATES; ii++) {
			if (jj == w_adapter->std_rates[ii]) {
				min_rate = ONEBOX_CPU_TO_LE16(w_adapter->rps_rates[ii]); 
				break;	
			}
		}
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("\nmin_rate: %d\n"), min_rate));
	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("\nmin_rate: %d index %d\n"), min_rate, ii));

	//remaining = rs.rs_nrates + htrs.rs_nrates;
	remaining = 20;
	if(htrs.rs_nrates && rs.rs_nrates)/** BG/N Mode **/
	{
		ii = rs.rs_nrates - 1;
		jj = htrs.rs_nrates - 1;
		ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("<===== Connected in A/BG/N Mode =====>\n")));
		for( kk = 0; kk < (rs.rs_nrates + htrs.rs_nrates); kk++)
		{
			//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ii = %d jj = %d kk = %d rs.rs_rates[%d] = %d mcs [%d] = %d\n", ii, jj, kk, ii, rs.rs_rates[ii], jj, mcs[jj])));
			if(rs.rs_rates[ii] < mcs[jj])
			{
				//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("copying final from map_rates ii = %d jj = %d kk = %d rs.rs_rates[%d] = %d \n", ii, jj, kk, ii, rs.rs_rates[ii])));
				mgmt_frame->u.auto_rate.supported_rates[kk] = ONEBOX_CPU_TO_LE16((uint16)mcs_rates[jj]);
				if(jj == 0)
				{
					kk++;
					for(; kk < (rs.rs_nrates + htrs.rs_nrates); kk++)
					{
						if((rs.rs_rates[ii] == (STD_RATE_54)) ||
													(rs.rs_rates[ii] == (STD_RATE_48)) ||
													(rs.rs_rates[ii] == (STD_RATE_36)) ||
													(rs.rs_rates[ii] == (STD_RATE_24)) ||
													(rs.rs_rates[ii] == (STD_RATE_18)) ||
													(rs.rs_rates[ii] == (STD_RATE_12)) ||
													(rs.rs_rates[ii] == (STD_RATE_11)) ||
													(rs.rs_rates[ii] == (STD_RATE_09)) ||
													(rs.rs_rates[ii] == (STD_RATE_06)) ||
													(rs.rs_rates[ii] == (STD_RATE_5_5)) ||
													(rs.rs_rates[ii] == (STD_RATE_02)) ||
													(rs.rs_rates[ii] == (STD_RATE_01)))
						{
							for(p = 0 ; p < MAX_BG_SUPP_RATES; p++)
							{
								if((w_adapter->std_rates[p]) == rs.rs_rates[ii])
								{
									mgmt_frame->u.auto_rate.supported_rates[kk] = ONEBOX_CPU_TO_LE16( w_adapter->rps_rates[p]);
									ii--;
									break;
								}

							}
						}
					}
					break;
				}
				jj--;
			}
			else
			{
				if( (rs.rs_rates[ii] == (STD_RATE_54)) ||
											(rs.rs_rates[ii] == (STD_RATE_48)) ||
											(rs.rs_rates[ii] == (STD_RATE_36)) ||
											(rs.rs_rates[ii] == (STD_RATE_24)) ||
											(rs.rs_rates[ii] == (STD_RATE_18)) ||
											(rs.rs_rates[ii] == (STD_RATE_12)) ||
											(rs.rs_rates[ii] == (STD_RATE_11)) ||
											(rs.rs_rates[ii] == (STD_RATE_09)) ||
											(rs.rs_rates[ii] == (STD_RATE_06)) ||
											(rs.rs_rates[ii] == (STD_RATE_5_5))||
											(rs.rs_rates[ii] == (STD_RATE_02)) ||
											(rs.rs_rates[ii] == (STD_RATE_01)))
				{
					for(p =0 ; p < MAX_BG_SUPP_RATES; p++)
					{
						//if((w_adapter->std_rates[p] & 0x7f) == rs.rs_rates[ii])
						if((w_adapter->std_rates[p]) == rs.rs_rates[ii])
						{
							mgmt_frame->u.auto_rate.supported_rates[kk] = ONEBOX_CPU_TO_LE16(w_adapter->rps_rates[p]);
						}
		
					}
				}
				ii--;
			}
		}//End of for loop
		/** Copying the aggregation rates **/

		for(ii = 7; ii >= 0; ii--)
		{
				if (ht_rate_bitmap & BIT(ii)) {
						if (((vap->iv_flags_ht & IEEE80211_FHT_SHORTGI20) ||
						     (vap->iv_flags_ht & IEEE80211_FHT_SHORTGI40)) &&
						    (((ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20) &&
						     (ic->ic_curchan->ic_flags & IEEE80211_CHAN_HT20))  || 
						    ((ni->ni_htcap & IEEE80211_HTCAP_SHORTGI40) &&
						     (ic->ic_curchan->ic_flags & IEEE80211_CHAN_HT40)))) {
							mgmt_frame->u.auto_rate.supported_rates[kk++] =
								mcs_rates[ii] | BIT(9);/* MCS SHORTGI SUPPORT RATES */
							mgmt_frame->u.auto_rate.supported_rates[kk-1] = ONEBOX_CPU_TO_LE16( mgmt_frame->u.auto_rate.supported_rates[kk-1]);
						} else {
							mgmt_frame->u.auto_rate.supported_rates[kk++] = mcs_rates[ii];
							mgmt_frame->u.auto_rate.supported_rates[kk-1] = ONEBOX_CPU_TO_LE16( mgmt_frame->u.auto_rate.supported_rates[kk-1]);
						}

						mgmt_frame->u.auto_rate.supported_rates[kk++] = (uint16)mcs_rates[ii];
						mgmt_frame->u.auto_rate.supported_rates[kk-1] = ONEBOX_CPU_TO_LE16( mgmt_frame->u.auto_rate.supported_rates[kk-1]);
						min_mcs_rate = mcs_rates[ii];	
						remaining -= 2;
				}
		}
		/*default rates for aggregation*/
		for (ii = remaining; ii > 2; ii--, kk++) { //check this, in the first loop RSI_RATE_36 mght not be a supported rate
			//mgmt_frame->u.auto_rate.supported_rates[kk] = min_rate;
			if (min_mcs_rate > RSI_RATE_MCS4) {
				if(rs.rs_nrates > 4) { /*11G support*/
					mgmt_frame->u.auto_rate.supported_rates[kk] = ONEBOX_CPU_TO_LE16(RSI_RATE_36);
				} else {
					mgmt_frame->u.auto_rate.supported_rates[kk] = ONEBOX_CPU_TO_LE16(RSI_RATE_1);
				}
			} else {
				mgmt_frame->u.auto_rate.supported_rates[kk] = ONEBOX_CPU_TO_LE16(min_mcs_rate);
			}
		}

		mgmt_frame->u.auto_rate.supported_rates[kk++] = ONEBOX_CPU_TO_LE16(min_rate);
		mgmt_frame->u.auto_rate.supported_rates[kk] = ONEBOX_CPU_TO_LE16(min_rate);

	}
	else if(htrs.rs_nrates)/** Only 11N **/
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("<======connected in 11n mode ======>\n")));
		for(ii = (htrs.rs_nrates - 1); ii >= 0; ii--, kk++)
		{
			mgmt_frame->u.auto_rate.supported_rates[kk] = mcs_rates[ii];
		}
		for(ii = htrs.rs_nrates - 1; ii >= 0; ii--, kk++)
		{
			if((ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20) || (ni->ni_htcap & IEEE80211_HTCAP_SHORTGI40) )
			{
				mgmt_frame->u.auto_rate.supported_rates[kk++] = mcs_rates[ii] | BIT(9);/* MCS SHORTGI SUPPORT RATES */
			}
			mgmt_frame->u.auto_rate.supported_rates[kk] = mcs_rates[ii];
			min_mcs_rate = mcs_rates[ii];	
			remaining--;
		}
		/*default rates for aggregation*/
		for(ii = remaining; ii > 0; ii--, kk++)
		{
			mgmt_frame->u.auto_rate.supported_rates[kk] = min_mcs_rate;
		}
			mgmt_frame->u.auto_rate.supported_rates[kk] = min_rate;

	}
	else/**BG Mode **/
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("<====== Connected in BG Mode  Band = %d =========>\n"), w_adapter->operating_band));
		//ii = (rs.rs_nrates - 1);
		//for(kk = 0; kk < rs.rs_nrates ; kk++)
		{
			for(ii = rs.rs_nrates; ii >= 0 ; ii--)
			{
						ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("rs.rs_rates[%d] = %02x \n"), ii, rs.rs_rates[ii] ));
						if( (rs.rs_rates[ii] == (STD_RATE_54)) ||
													(rs.rs_rates[ii] == (STD_RATE_48)) ||
													(rs.rs_rates[ii] == (STD_RATE_36)) ||
													(rs.rs_rates[ii] == (STD_RATE_24)) ||
													(rs.rs_rates[ii] == (STD_RATE_18)) ||
													(rs.rs_rates[ii] == (STD_RATE_12)) ||
													(rs.rs_rates[ii] == (STD_RATE_11)) ||
													(rs.rs_rates[ii] == (STD_RATE_09)) ||
													(rs.rs_rates[ii] == (STD_RATE_06)) ||
													(rs.rs_rates[ii] == (STD_RATE_5_5))||
													(rs.rs_rates[ii] == (STD_RATE_02)) ||
													(rs.rs_rates[ii] == (STD_RATE_01)))
						{
							for(p = 0 ; p < MAX_BG_SUPP_RATES ; p++)
							{
								if((w_adapter->std_rates[p]) == rs.rs_rates[ii])
								{
									//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("rs.rs_rates[%d] = %02x std_rates[%d] = %02x\n", ii,rs.rs_rates[ii], p, w_adapter->std_rates[p])));
									mgmt_frame->u.auto_rate.supported_rates[kk] = w_adapter->rps_rates[p];
									//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("mgmt_frame->u.auto_rate.supported_rates[%d] = %02x \n", kk, mgmt_frame->u.auto_rate.supported_rates[kk])));
									kk++;
								}
							}
						}
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("\n")));
		}
	}
	//jj += kk;
	mgmt_frame->u.auto_rate.num_supported_rates = ONEBOX_CPU_TO_LE16((rs.rs_nrates + htrs.rs_nrates)*2);
//	mgmt_frame->u.auto_rate.num_supported_rates = ONEBOX_CPU_TO_LE16(0x0028);//mgmt_frame->u.auto_rate.num_supported_rates );

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((sizeof(mgmt_frame->u.auto_rate) + ONEBOX_CPU_TO_LE16(mgmt_frame->u.auto_rate.num_supported_rates) * 2) | 
													                      (ONEBOX_WIFI_MGMT_Q << 12));
	
	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("Auto rate frame dump\n")));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_AUTORATE, (PUCHAR)mgmt_frame,(FRAME_DESC_SZ + sizeof(mgmt_frame->u.auto_rate) + (2*	ONEBOX_CPU_TO_LE16(mgmt_frame->u.auto_rate.num_supported_rates))));
	status = w_adapter->devdep_ops->onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (FRAME_DESC_SZ + (sizeof(mgmt_frame->u.auto_rate)+ 2 * ONEBOX_CPU_TO_LE16(mgmt_frame->u.auto_rate.num_supported_rates))));

	return status;
} /* End <onebox_send_auto_rate_request> */

ONEBOX_STATUS is_channel_valid(struct ieee80211vap *vap, uint16 chn_num)
{
  uint8 ii = 0;
  uint8 status = ONEBOX_STATUS_FAILURE;
  struct ieee80211com *ic = NULL;

  if(vap == NULL) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("In %s line %d station VAP is NULL\n",__func__, __LINE__));
			dump_stack();
			return status;
  }

  ic = vap->iv_ic;
  chn_num &= ~(BIT(15)); // clearing DFS indication in channel num 
  chn_num &= ~(BIT(14)); // clearing 11J indication in channel num.
  for (; ii < ic->ic_nchans; ii++)
  {
      if((ic->ic_channels[ii].ic_ieee == chn_num)) {
          status = ONEBOX_STATUS_SUCCESS;
          break;
      }
  }
  return status;
}

ONEBOX_STATUS onebox_send_bgscan_params(struct ieee80211vap *vap, uint16 *data, uint8 default_val)
{
	struct net_device *parent_dev = vap->iv_ic->ic_ifp;
	WLAN_ADAPTER w_adapter = netdev_priv(parent_dev);
	onebox_mac_frame_t *mgmt_frame;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	uint8	 status, ii, jj; 

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((sizeof(mgmt_frame->u.bgscan_params) ) | 
	                                              (ONEBOX_WIFI_MGMT_Q << 12));
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BG_SCAN_PARAMS);
	if(default_val == 1)
	{
		if(!(vap->hal_priv_vap->bgscan_params_ioctl.bg_ioctl))
		{
			//mgmt_frame->u.bgscan_params.bgscan_threshold = 10;
			mgmt_frame->u.bgscan_params.bgscan_threshold = 10;
			mgmt_frame->u.bgscan_params.roam_threshold = 65;
			mgmt_frame->u.bgscan_params.bgscan_periodicity = 5;
			if(w_adapter->operating_band == BAND_5GHZ)
			{
				mgmt_frame->u.bgscan_params.num_bg_channels = 4;
				mgmt_frame->u.bgscan_params.channels2scan[0] = 36;
				mgmt_frame->u.bgscan_params.channels2scan[1] = 48;
				mgmt_frame->u.bgscan_params.channels2scan[2] = 56 | BIT(15) ;
				mgmt_frame->u.bgscan_params.channels2scan[3] = 44;
			}
			else
			{
				mgmt_frame->u.bgscan_params.num_bg_channels = 3;
				mgmt_frame->u.bgscan_params.channels2scan[0] = 1;//need to change these 2 vals
				mgmt_frame->u.bgscan_params.channels2scan[1] = 6;//need to change these 2 vals
				mgmt_frame->u.bgscan_params.channels2scan[2] = 11;//need to change these 2 vals
			}
			mgmt_frame->u.bgscan_params.active_scan_duration = 20;
			mgmt_frame->u.bgscan_params.passive_scan_duration = 50;
			memcpy(&vap->hal_priv_vap->bgscan_params_ioctl, &mgmt_frame->u.bgscan_params, sizeof(mgmt_frame->u.bgscan_params));
		}
		else
		{
			memcpy(&mgmt_frame->u.bgscan_params, &vap->hal_priv_vap->bgscan_params_ioctl, sizeof(mgmt_frame->u.bgscan_params));
		}
	}
	else
	{
		memcpy(&mgmt_frame->u.bgscan_params, data, sizeof(mgmt_frame->u.bgscan_params));
		memset(&mgmt_frame->u.bgscan_params.channels2scan[0], 0, MAX_NUM_SCAN_BGCHANS);
		for(ii = 0, jj=0; ii < vap->hal_priv_vap->bgscan_params_ioctl.num_bg_channels; ii++ ) {
			if(is_channel_valid(vap, vap->hal_priv_vap->bgscan_params_ioctl.channels2scan[ii]) == ONEBOX_STATUS_SUCCESS) {
				mgmt_frame->u.bgscan_params.channels2scan[jj++] = vap->hal_priv_vap->bgscan_params_ioctl.channels2scan[ii];
			}
		}
		mgmt_frame->u.bgscan_params.num_bg_channels = jj;

		if(mgmt_frame->u.bgscan_params.bgscan_periodicity == 0)
		{
			vap->iv_host_scan = 0;
		}

		if (default_val == 2) /*DFS QUIET or CSA */
			mgmt_frame->u.bgscan_params.bgscan_periodicity = 0;
	}

	if(!mgmt_frame->u.bgscan_params.num_bg_channels)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\n\nERROR:Unable to perform BG Scan, No valid channel present in the given set\n ")));
		return ONEBOX_STATUS_FAILURE;
	}

	status = w_adapter->devdep_ops->onebox_send_internal_mgmt_frame(w_adapter,
			(uint16 *)mgmt_frame,
			FRAME_DESC_SZ + sizeof(mgmt_frame->u.bgscan_params));

	return status;
}


ONEBOX_STATUS send_bgscan_probe_req( WLAN_ADAPTER w_adapter, struct ieee80211_node *ni, uint16 flags)
{
	onebox_mac_frame_t *mgmt_frame;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	struct ieee80211com *ic = &w_adapter->vap_com;
	uint8	 status; 
	struct ieee80211vap *vap = ni->ni_vap;
	uint8 buf1[100] = {0};
	uint8 *buf;
	uint8 da[6] = {0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF};
	struct ieee80211_rateset *rs;
	//struct ieee80211_appie *ie;
	struct ieee80211_channel *c;
	uint8 len = 0;

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);
	//buf = kmalloc(512, GFP_ATOMIC);
	buf = buf1;


	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BG_SCAN_PROBE_REQ);

	if(w_adapter->operating_band == BAND_5GHZ)
	{
		mgmt_frame->u.bgscan_probe.mgmt_rate = RSI_RATE_6;
		mgmt_frame->u.bgscan_probe.channel_num = 40;
	}
	else
	{
		mgmt_frame->u.bgscan_probe.mgmt_rate = RSI_RATE_1;
		mgmt_frame->u.bgscan_probe.channel_num = 11;
	}
	//mgmt_frame->u.bgscan_probe.flags = BIT(5);
	mgmt_frame->u.bgscan_probe.flags = flags;
	mgmt_frame->u.bgscan_probe.channel_scan_time = 50;
	*buf++ = IEEE80211_FC0_SUBTYPE_PROBE_REQ;
	buf = buf+3;
	memcpy(buf, da, 6);
	buf = buf+6;
	memcpy(buf, vap->iv_myaddr, 6);
	buf = buf+6;
	memcpy(buf, da, 6);
	buf =buf+6;// more 2 for sequence number
	*buf++ = 0;
	*buf++ = 0;

	*buf++ = IEEE80211_ELEMID_SSID;
	if(vap->hal_priv_vap->bg_ssid.ssid_len)
	{
		*buf++ = vap->hal_priv_vap->bg_ssid.ssid_len;//len_ssid;
		memcpy(buf, vap->hal_priv_vap->bg_ssid.ssid,vap->hal_priv_vap->bg_ssid.ssid_len);
		buf = buf + vap->hal_priv_vap->bg_ssid.ssid_len;
	}
	else
	{
		*buf++ = 0;//len_ssid;
	}
	//rs = (struct ieee80211_rateset *)ieee80211_get_suprates(ic, ic->ic_curchan);
	c = ieee80211_find_channel_byieee(ic, 36,
			ic->ic_curchan->ic_flags);
	if(c == NULL) {
		rs = (struct ieee80211_rateset *)ieee80211_get_suprates(ic, ic->ic_curchan);
	}
	else {
		rs = (struct ieee80211_rateset *)ieee80211_get_suprates(ic, c);
	}
	buf = ieee80211_add_rates(buf, rs);
	buf = ieee80211_add_xrates(buf, rs);
#if 0
	if (vap->iv_flags & IEEE80211_F_WPA1) {
		if (vap->iv_wpa_ie != NULL)
		{
			ie = (struct ieee80211_appie *)vap->iv_wpa_ie;
			memcpy(buf, (uint8 *)ie->ie_data, (uint8 )ie->ie_len);
			buf = buf + (uint8 )ie->ie_len;
		}
		/* XXX else complain? */
	}
	if (vap->iv_appie_probereq != NULL)
	{
			ie = (struct ieee80211_appie *)vap->iv_appie_probereq;
			memcpy(buf, (uint8 *)ie->ie_data, (uint8 )ie->ie_len);
			buf = buf + (uint8 )ie->ie_len;
	}
#endif
	len = buf - buf1;
	mgmt_frame->u.bgscan_probe.probe_req_length = len;
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((sizeof(mgmt_frame->u.bgscan_probe)  + len) | 
	                                              (ONEBOX_WIFI_MGMT_Q << 12));

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("<= BGSCAN PROBE REQ FRAME len = %d==>\n", len));

	memcpy((pkt_buffer + (FRAME_DESC_SZ + sizeof(mgmt_frame->u.bgscan_probe))), buf1, len);
	status = w_adapter->devdep_ops->onebox_send_internal_mgmt_frame(w_adapter,
			(uint16 *)mgmt_frame,
			FRAME_DESC_SZ + sizeof(mgmt_frame->u.bgscan_probe) + len);
	return status;

}

void send_bgscan_host_triggered_bgscan(struct ieee80211vap *vap)
{
		WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)vap->hal_priv_vap->hal_priv_ptr;
		uint8 status;

		status = send_bgscan_probe_req(w_adapter, vap->iv_bss, (vap->hal_priv_vap->bgscan_params_ioctl.bg_cmd_flags|BIT(4)));
}

#ifdef ONEBOX_CONFIG_WOWLAN
ONEBOX_STATUS onebox_vap_config_wowlan(struct ieee80211vap *vap, void *priv)
{
	WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)priv;
	uint16_t rx_filter_word;

	if (onebox_send_wowlan_params(vap, vap->hal_priv_vap->hal_priv_ptr)) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG,
				("%s: Failed to send wowlan params\n",
				 __func__));
		return -1;
	}
	w_adapter->wowlan_enabled =
		vap->hal_priv_vap->wowlan_params.host_wakeup_state;
	onebox_send_vap_dynamic_update_indication_frame(vap);
	if (w_adapter->wowlan_enabled)
		rx_filter_word = (ALLOW_DATA_ASSOC_PEER | DISALLOW_BEACONS);
	else
		rx_filter_word = (ALLOW_DATA_ASSOC_PEER |
				ALLOW_CTRL_ASSOC_PEER |
				ALLOW_MGMT_ASSOC_PEER);
	onebox_send_rx_filter_frame(vap->hal_priv_vap->hal_priv_ptr,
				    rx_filter_word);

	return 0;
}
#endif

#ifdef ONEBOX_CONFIG_GTK_OFFLOAD
void onebox_send_gtk_rekey_data(struct ieee80211vap *vap, struct ieee80211_gtk_rekey_data *data)
{
  onebox_mac_frame_t *mgmt_frame;
  WLAN_ADAPTER w_adapter;
  struct onebox_os_intf_operations *os_intf_ops;
  uint8 pkt_buffer[MAX_MGMT_PKT_SIZE];
  uint16 status = 0;
  
  os_intf_ops = onebox_get_os_intf_operations_from_origin();
  w_adapter = os_intf_ops->onebox_get_priv(vap->iv_ic->ic_ifp);

  mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
  w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);

  mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((sizeof(mgmt_frame->u.gtk_rekey_data) ) | 
		  (ONEBOX_WIFI_MGMT_Q << 12));
  mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(GTK_REKEY_DATA);
  mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(w_adapter->gtk_en << 8);

  if (w_adapter->gtk_en) {
    memcpy(mgmt_frame->u.gtk_rekey_data.kek, data->kek, IEEE80211_KEK_LEN);
    memcpy(mgmt_frame->u.gtk_rekey_data.kck, data->kck, IEEE80211_KCK_LEN);
    memcpy(mgmt_frame->u.gtk_rekey_data.replay_ctr, data->replay_ctr, IEEE80211_REPLAY_CTR_LEN);
  }

  status = w_adapter->devdep_ops->onebox_send_internal_mgmt_frame(w_adapter,
            (uint16 *)mgmt_frame,
            FRAME_DESC_SZ + sizeof(mgmt_frame->u.gtk_rekey_data));
}
#endif

EXPORT_SYMBOL(send_bgscan_probe_req);
EXPORT_SYMBOL(onebox_send_bgscan_params);
EXPORT_SYMBOL (set_region);
