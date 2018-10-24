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

#ifdef ONEBOX_CONFIG_CFG80211
#include<linux/kernel.h>
#include<net80211/ieee80211_var.h>
#include<net80211/ieee80211_ioctl.h>
#include<net80211/ieee80211_crypto.h>
#include<net80211/ieee80211.h>
#include <linux/nl80211.h>
#include "cfg80211_ioctl.h"



int  onebox_prepare_ioctl_cmd(struct ieee80211vap *vap, uint8_t type, const void *data, int val, int len )
{
	struct ieee80211req ireq;

	ireq.i_type = type;
	ireq.i_data = (void *)data;
	ireq.i_val = val;
	ireq.i_len = len;
	return ieee80211_ioctl_set80211(vap, 0, &ireq);
	
}

uint8_t onebox_add_key(struct net_device *ndev, uint8_t index, 
                       const uint8_t  *mac_addr, struct ieee80211_key_params *params)
{
	struct ieee80211vap *vap = netdev_priv(ndev);
	struct ieee80211req_key wk;

	memset(&wk, 0, sizeof(wk));

	if (params->cipher == WPA_ALG_NONE) 
	{
		if (mac_addr == NULL ||
		    memcmp(mac_addr, "\xff\xff\xff\xff\xff\xff",
			      IEEE80211_ADDR_LEN) == 0)
		{
			return onebox_delete_key(ndev, NULL, index);
		}
		else
		{
			return onebox_delete_key(ndev, mac_addr, index);
		}
	}
	switch (params->cipher)
	{
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
		{
			if(index > 3)
			{
				break;
			}
			wk.ik_type = IEEE80211_CIPHER_WEP;
			break;
		}
		case WLAN_CIPHER_SUITE_TKIP:
		{
			wk.ik_type = IEEE80211_CIPHER_TKIP;
			break;
		}
		case WLAN_CIPHER_SUITE_CCMP:
		{
			wk.ik_type = IEEE80211_CIPHER_AES_CCM;
			break;
		}
		default:
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s: unknown alg=%d", __func__, params->cipher));
			return -1;

		break;
	}
	wk.ik_flags = IEEE80211_KEY_RECV ;
	wk.ik_flags |= IEEE80211_KEY_XMIT;

	/**need to set the flag based on transmission**/
	/*need to fix
	if(set_tx)
		wk.ik_flags |= IEEE80211_KEY_XMIT;*/

	if (mac_addr == NULL) 
	{
		memset(wk.ik_macaddr, 0xff, IEEE80211_ADDR_LEN);
		wk.ik_keyix = index;
	} 
	else 
	{
		memcpy(wk.ik_macaddr, mac_addr, IEEE80211_ADDR_LEN);
	}

	if (memcmp(wk.ik_macaddr, "\xff\xff\xff\xff\xff\xff",
		      IEEE80211_ADDR_LEN) == 0) 
	{
		wk.ik_flags |= IEEE80211_KEY_GROUP;
		wk.ik_keyix = index;
	} 
	else 
	{
		wk.ik_keyix = index == 0 ? IEEE80211_KEYIX_NONE : index;
	}

	//if (wk.ik_keyix != IEEE80211_KEYIX_NONE && set_tx)/* need to fix */
	if (wk.ik_keyix != IEEE80211_KEYIX_NONE )	{
		wk.ik_flags |= IEEE80211_KEY_DEFAULT;
	}

	wk.ik_keylen = params->key_len;
	memcpy(&wk.ik_keyrsc, params->seq, params->seq_len);
	memcpy(wk.ik_keydata, params->key, params->key_len);

	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_WPAKEY, (void *)&wk, 0, sizeof(wk)) < 0)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	else
	{
		return ONEBOX_STATUS_SUCCESS;
	}
}

uint8_t
onebox_delete_key(struct net_device *ndev, const uint8_t *mac_addr, uint8_t index)
{
	struct ieee80211vap *vap = netdev_priv(ndev);
	struct ieee80211req_del_key wk;

	if (mac_addr == NULL) 
	{
		wk.idk_keyix = index;
	} 
	else 
	{
		//IEEE80211_DBG_PRINT(NULL, IEEE80211_MSG_ERROR, ("%s: addr=" MACSTR, __func__, MAC2STR(addr)));
		memcpy(wk.idk_macaddr, mac_addr, IEEE80211_ADDR_LEN);
		wk.idk_keyix = (u_int8_t) IEEE80211_KEYIX_NONE;	/* XXX */
	}

	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_DELKEY, (void *)&wk, 0, sizeof(wk)) < 0)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	else
	{
		return ONEBOX_STATUS_SUCCESS;
	}
} 

int cfg80211_disconnect(struct net_device *ndev, int reason_code)
{
	struct ieee80211req_mlme mlme;
	struct ieee80211vap *vap = netdev_priv(ndev);

	memset(&mlme, 0, sizeof(mlme));
	mlme.im_op = IEEE80211_MLME_DISASSOC;
	mlme.im_reason = reason_code;

	if(reason_code == 99)
	{
		return 0;
	} 

	if (vap && (vap->iv_state != IEEE80211_S_RUN)) {
		return 0;
	}
	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_MLME, &mlme, 0, sizeof(mlme)) < 0)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	else
	{
		return ONEBOX_STATUS_SUCCESS;
	}
}

int32_t onebox_siwfrag(struct ieee80211com *ic, struct iw_param *frag)
{
#if 0
	struct ieee80211vap *vap;
	struct ieee80211req ireq;

	/* XXX: Need to get the vap pointer */
	//vap = TAILQ_FIRST(
	if (frag->disabled)
	{
		ireq.i_val = 2346;
	}
	else if ((frag->value < 256) || (frag->value > 2346))
	{
		return -EINVAL;
	}
	else
	{
		ireq.i_val = (frag->value & ~0x1);
	}

	ireq.i_type = IEEE80211_IOC_FRAGTHRESHOLD;
	return ieee80211_ioctl_set80211(vap, 0, &ireq);
#endif
	return 0;
}

int32_t onebox_siwrts(struct ieee80211com *ic, struct iw_param *rts)
{
#if 0
	struct ieee80211vap *vap = NULL;
	struct ieee80211req ireq;

	/* XXX: Need to get the vap pointer */
	//vap = TAILQ_FIRST(
	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("NL80211 : In %s and %d vap %p\n",__func__,__LINE__, vap));
	if (rts->disabled)
	{
		ireq.i_val = IEEE80211_RTS_MAX;
	}
	else if ((IEEE80211_RTS_MIN <= rts->value) &&
			(rts->value <= IEEE80211_RTS_MAX))
	{
		ireq.i_val = rts->value;
	}
	else
	{
		return -EINVAL;
	}

	ireq.i_type = IEEE80211_IOC_RTSTHRESHOLD;
	return ieee80211_ioctl_set80211(vap, 0, &ireq);
#endif
	return 0;
}

uint8_t tx_power(struct ieee80211com *ic, int dbm)
{
#if 0
	struct ieee80211vap *vap;
	struct ieee80211req ireq;

	/*XXX: Get vap ptr from ic */

	ireq.i_val = dbm;
	ireq.i_type = IEEE80211_IOC_TXPOWER;

	return ieee80211_ioctl_set80211(vap, 0, &ireq);
#endif
	return 0;
}

void cfg80211_init_callbacks(struct ieee80211com *ic)
{

	struct cfg80211_events *cfg_events = &ic->cfg80211_call_backs;

	cfg_events->cfg80211_scan_done = scan_done;
	cfg_events->cfg_sta_disconnect = notify_sta_disconnect_to_cfg80211;
	cfg_events->cfg_sta_connection_confirm = notify_sta_connect_to_cfg80211;
	cfg_events->inform_mic_failure_to_cfg80211 = notify_mic_failure_to_cfg80211;
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0))
	cfg_events->cfg_node_leave = inform_cfg80211_node_leave;
	cfg_events->cfg_node_join = inform_cfg80211_node_join;
#endif
	cfg_events->inform_recvd_mgmt_to_cfg80211 = obm_inform_mgmt_cfg80211;
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0))
	cfg_events->cfg80211_radar_status_event = obm_notify_cfg80211;
#endif
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
	cfg_events->cfg80211_notify_ft_event = obm_notify_ft_event;
#endif //CONFIG_11R
#ifdef ENABLE_P2P_SUPPORT
	cfg_events->cfg80211_ready_on_channel = obm_ready_on_channel;
	cfg_events->cfg80211_remain_on_channel_expired = obm_remain_on_channel_expired;
#endif
}

int
onebox_set_if_media(struct net_device *dev, int media)
{
	struct ieee80211vap *vap = netdev_priv(dev);
	struct ifreq ifr;

//	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("STAIOCTL: %s %d \n", __func__, __LINE__));
	memset(&ifr, 0, sizeof(ifr));
	ifr.ifr_ifru.ifru_ivalue = media;
	if(ifmedia_ioctl(vap->iv_ifp, (struct ifreq *)&ifr, &vap->iv_media, SIOCSIFMEDIA) < 0)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	else
	{
		return ONEBOX_STATUS_SUCCESS;
	}
}

int
onebox_get_if_media(struct net_device *dev)
{
	struct ifmediareq ifmr;
	struct ieee80211vap *vap = netdev_priv(dev);

	memset(&ifmr, 0, sizeof(ifmr));

	if(ifmedia_ioctl(vap->iv_ifp, (struct ifreq *)&ifmr, &vap->iv_media, SIOCGIFMEDIA) < 0)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	return ifmr.ifm_current;
}

int
onebox_set_mediaopt(struct net_device *dev, uint32_t mask, uint32_t mode)
{
	int media = onebox_get_if_media(dev);

	if (media < 0)
	{
		return -1;
	}
	media &= ~mask;
	media |= mode;

	if (onebox_set_if_media(dev, media) < 0)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	else
	{
		return ONEBOX_STATUS_SUCCESS;
	}
}

int
onebox_driver_nl80211_set_wpa(struct ieee80211vap *vap, int enabled)
{
	return onebox_driver_nl80211_set_wpa_internal(vap, enabled ? 3 : 0, enabled);
}

int
onebox_driver_nl80211_set_wpa_internal(struct ieee80211vap *vap, int wpa, int privacy)
{
	if (!wpa && onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_APPIE, NULL, IEEE80211_APPIE_WPA, 0) < 0)
	{
		return ONEBOX_STATUS_FAILURE;
	}

	if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_PRIVACY, NULL, privacy, 0) < 0)
	{
		return ONEBOX_STATUS_FAILURE;
	}

	if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_WPA, NULL, wpa, 0) < 0)
	{
		return ONEBOX_STATUS_FAILURE;
	}

	return ONEBOX_STATUS_SUCCESS;
}

int cfg80211_scan(void *temp_req, struct net_device *ndev, uint8_t num_ch, uint16_t *chans, 
                     const uint8_t *ie, size_t ie_len, int n_ssids, void *ssid)
{
	struct ieee80211_ssid *ssids = (struct ieee80211_ssid *)ssid;
	struct ieee80211vap *vap = netdev_priv(ndev);
	struct ieee80211_scan_req sr;
	uint8_t i;
	
	memset(&sr, 0, sizeof(sr));

	if(vap->iv_state == IEEE80211_S_RUN) //Fix made for bgscan
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_SCAN, ("NL80211 : In %s and %d \n" , __func__, __LINE__));
        vap->scan_request = temp_req;	
        if(vap->hal_priv_vap->bgscan_params_ioctl.bgscan_periodicity 
                && vap->hal_priv_vap->bgscan_params_ioctl.bg_ioctl ) {
			scan_done(vap->scan_request, 0);
			vap->scan_request = NULL;
		}
		return 0;
	}
	
	/*
	 * Uncomment the below code when AP mode
	 * supports Autochannel
	 */
#ifdef CONFIG_ACS
	if(vap->iv_opmode == IEEE80211_M_HOSTAP) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_SCAN, ("NL80211 : In %s and %d Done event to cfg80211\n" , __func__, __LINE__));
		if (onebox_set_mediaopt(ndev, IFM_OMASK, 512 ) < 0) 
		{
			printk("%s: failed to set operation mode", __func__);
			return -1;
		}
		vap->hal_priv_vap->acs_enable = 1;
		vap->iv_des_chan = IEEE80211_CHAN_ANYC;
		IEEE80211_LOCK(vap->iv_ic);
		vap->iv_ic->idx = 0; 
		IEEE80211_UNLOCK(vap->iv_ic);
	}
#endif
	if (vap->iv_opmode == IEEE80211_M_STA) { 
		if (onebox_set_mediaopt(ndev, IFM_OMASK, 0)) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Failed to set operation mode %s Line %d\n", __func__, __LINE__));
			return -1;
		}
	}
	
	if(onebox_set_mediaopt(ndev, IFM_MMASK, 0))
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Failed to set modulation mode\n"));
		return -1;
	}

	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_ROAMING, NULL, IEEE80211_ROAMING_AUTO, 0) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Roaming Ioctl Not set\n"));
		return -1;
	}

	if(vap->iv_opmode != IEEE80211_M_HOSTAP) {
		if(onebox_driver_nl80211_set_wpa(vap, 1) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Failed to set WPA\n"));
			return -1;
		}
	}

#if 0
	if(nl80211_ctrl_iface(ndev, 1) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Failed to set interafce \n"));
		return -1;
	}
#endif
	if(ie && ie_len)
	{
		if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_APPIE, ie, 
		                   (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_PROBE_REQ), ie_len) < 0)
		{
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("IE Ioctl not set\n"));
			return -1;
		}
	}
	sr.sr_flags = IEEE80211_IOC_SCAN_ACTIVE | IEEE80211_IOC_SCAN_ONCE | IEEE80211_IOC_SCAN_NOJOIN;
	sr.sr_duration = IEEE80211_IOC_SCAN_FOREVER;
	
	if(n_ssids > 0)
	{
		sr.sr_nssid = n_ssids;
		sr.sr_flags |= IEEE80211_IOC_SCAN_CHECK;
	}
	for (i = 0; i < sr.sr_nssid; i++) 
	{
		sr.sr_ssid[i].len = ssids->ssid_len;
		memcpy(sr.sr_ssid[i].ssid, ssids->ssid, sr.sr_ssid[i].len);
	}

	if(num_ch)
	{
		sr.num_freqs = num_ch;
		for(i = 0; i < num_ch; i++)
		{
			sr.freqs[i] = chans[i];
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_SCAN, ("Scanning Selective channels mentioned by user %d\n", sr.freqs[i]));
		}
	}
	else
	{
		sr.num_freqs = 0;
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_SCAN, ("Scanning All channels\n"));
		
	}
	
	vap->scan_request = temp_req;	
	
	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_SCAN_REQ, (void *)&sr, 0, sizeof(sr)) < 0)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	else
	{
		return ONEBOX_STATUS_SUCCESS;
	}
}

size_t os_nl_strlcpy(char *dest, const char *src, size_t siz)
{
	const char *s = src;
	size_t left = siz;

	if (left) {
		/* Copy string up to the maximum size of the dest buffer */
		while (--left != 0) {
			if ((*dest++ = *s++) == '\0')
				break;
		}
	}

	if (left == 0) {
		/* Not enough room for the string; force NUL-termination */
		if (siz != 0)
			*dest = '\0';
		while (*s++)
			; /* determine total src string length */
	}

	return s - src - 1;
}

int onebox_inform_bss_to_cfg80211(struct ieee80211_scan_entry *ise, struct ieee80211vap *vap)
{
	struct ieee80211_scan_h *hscan;
	int ret = ONEBOX_STATUS_SUCCESS;

	hscan = kmalloc(sizeof(struct ieee80211_scan_h), GFP_KERNEL);
	memcpy(hscan->se_bssid, ise->se_bssid, IEEE80211_ADDR_LEN);
	memcpy(hscan->se_ssid, ise->se_ssid, IEEE80211_NWID_LEN);
	hscan->se_intval = ise->se_intval;
	hscan->se_capinfo = ise->se_capinfo;
	hscan->se_chan = ise->se_chan;
	hscan->ic_ieee = ise->se_chan->ic_ieee;
	hscan->data = ise->se_ies.data;
	hscan->len = ise->se_ies.len;
	hscan->se_rssi = ise->se_rssi;
	hscan->se_noise = ise->se_noise;

	if (scan_results_sup(vap->wdev,hscan) < 0)
		ret = ONEBOX_STATUS_FAILURE;

	kfree(hscan);
	return ret;
}

void* nl80211_memcp(void *to, const void *from, int len)
{
	return memcpy(to, from, len);
}

uint8_t onebox_wep_key(struct net_device *ndev, int index, const uint8_t *mac_addr, uint8_t key_len, const uint8_t *key)
{
	struct ieee80211vap *vap = netdev_priv(ndev);
	struct ieee80211req_key wk;

	memset(&wk, 0, sizeof(wk));
	vap->in_wep = 1;

//	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ASSOC, ("NL80211 : In %s LINE %d\n",__func__,__LINE__));
	
	wk.ik_type = IEEE80211_CIPHER_WEP;

	wk.ik_flags = IEEE80211_KEY_RECV ;
	wk.ik_flags |= IEEE80211_KEY_XMIT;

	/**need to set the flag based on transmission**/
	/*need to fix
	if(set_tx)
		wk.ik_flags |= IEEE80211_KEY_XMIT;*/

	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ASSOC, ("ASSOC CMD: In %s and %d mac_addr = %02x:%02x:%02x:%02x:%02x:%02x \n", __func__, __LINE__, mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]));
	mac_addr = NULL;
	if (mac_addr == NULL) 
	{
		memset(wk.ik_macaddr, 0xff, IEEE80211_ADDR_LEN);
		wk.ik_keyix = index;
	} 
	else 
	{
		memcpy(wk.ik_macaddr, mac_addr, IEEE80211_ADDR_LEN);
	}
	/*
	 * Deduce whether group/global or unicast key by checking
	 * the address (yech).  Note also that we can only mark global
	 * keys default; doing this for a unicast key is an error.
	 */
	if (memcmp(wk.ik_macaddr, "\xff\xff\xff\xff\xff\xff",
		      IEEE80211_ADDR_LEN) == 0) {
		wk.ik_flags |= IEEE80211_KEY_GROUP;
		wk.ik_keyix = index;
	} else {
		wk.ik_keyix = index == 0 ? IEEE80211_KEYIX_NONE :
			index;
	}
	//if (wk.ik_keyix != IEEE80211_KEYIX_NONE && set_tx)/* need to fix */
	if (wk.ik_keyix != IEEE80211_KEYIX_NONE )/* need to fix */
		wk.ik_flags |= IEEE80211_KEY_DEFAULT;

	wk.ik_keylen = key_len;
//	memcpy(&wk.ik_keyrsc, params->seq, params->seq_len);
	memcpy(wk.ik_keydata, key, key_len);

//	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ASSOC, ("NL80211 : In %s LINE %d\n",__func__,__LINE__));
	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_WPAKEY, (void *)&wk, 0, sizeof(wk)) < 0)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	else
	{
		return ONEBOX_STATUS_SUCCESS;
	}
}

int cfg80211_connect_res(struct net_device *ndev, cfg80211_sta_connect_params *connect_params)
{
	struct ieee80211vap *vap;
	struct ieee80211req_mlme mlme;
	int authmode;
	vap = netdev_priv(ndev);
	
	if(onebox_set_mediaopt(ndev, IFM_OMASK, 0) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ASSOC, ("Failed to set Operation mode %s Line %d\n", __func__, __LINE__));
		return -1;
	}

	if (connect_params->auth_type == WPA_AUTH_ALG_SHARED) {
		authmode = IEEE80211_AUTH_SHARED;
	} else {
		authmode = IEEE80211_AUTH_OPEN;
	}

	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_AUTHMODE, NULL, authmode, 0) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Authmode Not set\n"));
		return -1;
	}

	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_APPIE, connect_params->ie, IEEE80211_APPIE_WPA, connect_params->ie_len) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Appie Ioctl Not set\n"));
		return -1;
	}
	
	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_PRIVACY, NULL, connect_params->privacy, 0) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Privacy Ioctl Not set\n"));
		return -1;
	}

	if((connect_params->ie_len && onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_WPA, NULL, connect_params->ie[0] == 48 ? 2 : 1, 0)) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("WPA Ioctl Not set\n"));
		return -1;
	}

	if(((connect_params->ssid != NULL) && onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_SSID, connect_params->ssid, 0, connect_params->ssid_len)) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("SSID Ioctl Not set\n"));
		return -1;
	}
	
	memset(&mlme, 0, sizeof(mlme));
	mlme.im_op = IEEE80211_MLME_ASSOC;
	if(connect_params->ssid != NULL)
	{
		memcpy(mlme.im_ssid, connect_params->ssid, connect_params->ssid_len);
	}
	if(connect_params->bssid != NULL)
	{
		memcpy(mlme.im_macaddr, connect_params->bssid, 6);
	}
	mlme.im_ssid_len = connect_params->ssid_len;

	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_MLME, &mlme, 0, sizeof(mlme)) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("MLME Ioctl Not set\n"));
		return -1;
	}
	return 0;
}

#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
int obm_update_ft_ies(struct net_device *ndev, cfg80211_ft_params *ftie)
{
	struct ieee80211vap *vap;
	vap = netdev_priv(ndev);
if (((vap->iv_state == IEEE80211_S_AUTH) && 
	    (vap->hal_priv_vap->roam_ind == 1)) || 
	    ((vap->iv_state == IEEE80211_S_FT) && 
	    (vap->hal_priv_vap->roam_ind == 1))) {
			IEEE80211_DBG_PRINT(vap,IEEE80211_MSG_DEBUG,("%s:%d Set Event for REASSOC REQ\n",__func__,__LINE__));
			atomic_set(&vap->ft_event.eventCondition, 0);
			wake_up_interruptible(&vap->ft_event.eventQueue);
	}
	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_APPIE, ftie->ies,
				(IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_AUTH), ftie->ie_len) < 0){
    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d IE Ioctl not set\n", __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}
	return ONEBOX_STATUS_SUCCESS;
}

int obm_send_auth(struct net_device *ndev, cfg80211_auth_req *req)
{
	struct ieee80211vap *vap;
	vap = netdev_priv(ndev);

	int auth_type = req->auth_type;
	if(auth_type == 2){
		IEEE80211_DBG_PRINT(vap,IEEE80211_MSG_DEBUG,("%s:%d Send REASSOC for roaming (11R) ++++\n",__func__,__LINE__));
		vap->iv_newstate(vap,IEEE80211_S_ASSOC,0);
	}
	else	
		return ONEBOX_STATUS_FAILURE;

	return ONEBOX_STATUS_SUCCESS;

}
#endif //CONFIG_11R

int obm_cfg80211_add_intf(struct wiphy *wiphy,
		char *name,
		u8 opmode,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,12,0))
		u32 *flags,
#endif
		u8 macaddr[6],
		struct cfg80211_priv *cfg_priv)
{
	struct ieee80211com *ic = cfg_priv->ic;
	struct ieee80211vap *vap = NULL;

/* XXX : Error number hardcoding should be changed */
	if (!cfg_priv)
	{
		return 20;
	}

	switch (opmode) {
		case IEEE80211_M_STA:
			ic->ic_flags = 0;
			vap = ic->ic_vap_create(ic, name, 0, IEEE80211_M_STA, 2, NULL, macaddr);
			break;
		case IEEE80211_M_HOSTAP:
		case IEEE80211_M_P2P:
		case IEEE80211_M_P2P_CLIENT:
		case IEEE80211_M_P2P_GO:
			ic->ic_flags = 0;
			vap = ic->ic_vap_create(ic, name, 0, opmode, 0, NULL, macaddr);
			break;
		default:
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s: Interface type %d not supported\n", __func__, opmode));
	}
	if (vap == NULL) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("ERR: while creating vap for opmode %d\n", opmode));
		return 20;
	}

	cfg_priv->ndev[vap->hal_priv_vap->vap_id] = vap->iv_ifp;
	cfg_priv->wdev[vap->hal_priv_vap->vap_id] = vap->wdev;
	return vap->hal_priv_vap->vap_id;
}

int obm_cfg80211_del_intf(struct cfg80211_priv *cfg_priv, struct net_device *ndev)
{
	struct ieee80211com *ic = cfg_priv->ic;
	struct ieee80211vap *vap = netdev_priv(ndev);
	int vap_id = vap->hal_priv_vap->vap_id;
	ic->ic_vap_delete(vap, 0);

	ic->cfg_priv->wdev[vap_id] = NULL; 
	ic->cfg_priv->ndev[vap_id] = NULL; 
	return vap_id;
}

int obm_get_sta_vap(struct wiphy *wiphy, struct cfg80211_priv *cfg_priv)
{
	struct net_device *ndev;
	struct ieee80211vap *vap;
	int i;

	for (i = 0; i < 4/*wiphy->iface_combinations->max_interfaces*/; i++) {
		ndev = cfg_priv->ndev[i];
		vap = netdev_priv(ndev);

		if (!vap)
			continue;

		if ((vap->iv_opmode == IEEE80211_M_STA) ||
				(vap->iv_opmode == IEEE80211_M_P2P))
			return vap->hal_priv_vap->vap_id;
	}
	return -1;
}

int obm_change_virt_intf(struct net_device *ndev, u8 curr_iftype, u8 type)
{
	struct ieee80211vap *vap;
	u32 mode;
	vap = netdev_priv(ndev);
	switch (type){
		case IEEE80211_M_P2P_GO:
			switch (curr_iftype){
				case IEEE80211_M_STA:
				case IEEE80211_M_P2P_CLIENT:
					mode = 0x00000200;
					if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_CHANGE_VAP_MODE, NULL, mode, 0) < 0)
					{
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d CHANGE VAP MODE IOCTL Error \n", __func__,__LINE__));
						return -1;
					}
					return ONEBOX_STATUS_SUCCESS;
					break;
				default:
					return ONEBOX_STATUS_FAILURE;
			}
			break;
		default:
			return ONEBOX_STATUS_FAILURE;
	}
}

int obm_delete_connected_sta(struct net_device *ndev, const uint8_t *mac, 
	                            uint8_t opcode, uint16_t reason_code)
{
	struct ieee80211vap *vap;
  struct ieee80211req_mlme mlme;

	vap = netdev_priv(ndev);

	if(!vap) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("ERR: %s Line %d Unable to get VAP pointer \n", __func__, __LINE__));
		return -1;
	}
  memset(&mlme, 0, sizeof(mlme));
  mlme.im_op = opcode;
  mlme.im_reason = reason_code;
	nl80211_memcpy(mlme.im_macaddr, mac, IEEE80211_ADDR_LEN);

  if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_MLME, &mlme, 0, sizeof(mlme)) < 0)
  {
    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d MLME Ioctl Not set\n", __func__, __LINE__));
    return -1;
  }
	return 0;
}

int obm_cfg_change_beacon(struct net_device *ndev, struct obm_cfg_ap_beacon_ie *ap_params)
{
  struct ieee80211vap *vap;

  vap = netdev_priv(ndev);


  if(ap_params->beacon_ies && ap_params->beacon_ies_len)
  {
    if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_APPIE, ap_params->beacon_ies, 
          (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_BEACON), ap_params->beacon_ies_len) < 0)
    {
      IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d IE Ioctl not set\n", __func__, __LINE__));
      return -1;
    }

  }

  if(ap_params->proberesp_ies && ap_params->proberesp_ies_len)
  {
    if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_APPIE, ap_params->proberesp_ies, 
          (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_PROBE_RESP), ap_params->proberesp_ies_len) < 0)
    {
      IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d IE Ioctl not set\n", __func__, __LINE__));
      return -1;
    }
  }

  if(ap_params->assocresp_ies && ap_params->assocresp_ies_len)
  {
    if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_APPIE, ap_params->assocresp_ies, 
          (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_ASSOC_RESP), ap_params->assocresp_ies_len) < 0)
    {
      IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d IE Ioctl not set\n", __func__, __LINE__));
      return -1;
    }
  }
  return 0;
}

int onebox_set_channel(struct net_device *ndev, int channel_num)
{
	struct ieee80211vap *vap;
	vap = netdev_priv(ndev);

	vap->cfg80211_ap_set_channel = channel_num;
	return ONEBOX_STATUS_SUCCESS;
}

int obm_cfg_stop_ap(struct net_device *ndev)
{
	struct ieee80211vap *vap = netdev_priv(ndev);
	struct ieee80211_node *ni;
	vap->block_beacon_interrupt = 1;

	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("[%s][%d] : Stopping AP !!!\n", __func__, __LINE__));	
	if(vap->hal_priv_vap == NULL)
	{
		netif_carrier_off(vap->iv_ifp);
		return 0;
	}
	/* Changing vap state from RUN to INIT*/	
	vap->iv_newstate(vap, IEEE80211_S_INIT, 0);
	netif_carrier_off(vap->iv_ifp);
	/* Clearing Keys in LMAC */
	ni = ieee80211_find_vap_node(&vap->iv_ic->ic_sta, vap, vap->iv_myaddr);
	if(ni)
	{
		if(!(vap->iv_key_delete(vap, &ni->ni_ucastkey)))
		{
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("[%s][%d] : Unable to delete Group key!!!\n", __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}
	}
	else
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("[%s][%d] : Node is not created!!!\n", __func__, __LINE__));
	}
	return 0;
}

int obm_cfg_start_ap(struct net_device *ndev, struct obm_cfg80211_ap_params *ap_params)
{

	struct ieee80211vap *vap;
	struct ieee80211req_mlme mlme;
	uint8_t data[512];
	uint8_t *wpa_rsn = NULL, *wpa_ie = NULL, *tail;
	uint32_t tail_len = 0;
	uint8_t total_rsn_wpa_ie_len = 0;
	uint32_t mode =0;
	uint16_t ht_cap_info = 0;


	vap = netdev_priv(ndev);
	netif_carrier_on(vap->iv_ifp);

	if(!vap) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s Line %d Unable to get VAP address\n", __func__, __LINE__));
	}
#ifdef CONFIG_ACS	
	if(vap->iv_opmode == IEEE80211_M_HOSTAP && vap->hal_priv_vap->acs_enable) {
		vap->iv_newstate(vap, IEEE80211_S_INIT, 0);
		vap->hal_priv_vap->acs_enable = 0;
	}
#endif
	if(onebox_set_mediaopt(ndev, IFM_OMASK, 0x200 )) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Failed to set Operation mode %s Line %d\n", __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}

	if (ap_params->channel > 0 && ap_params->channel < 14)
		mode = IFM_IEEE80211_11NG;
	else if (ap_params->channel == 0)
		mode = IFM_AUTO;
	else if (ap_params->channel == 14)
		mode = IFM_IEEE80211_11B;
	else
		mode = IFM_IEEE80211_11NA;

	if (onebox_set_mediaopt(ndev, IFM_MMASK, mode)) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Failed to set modulation mode\n"));
		return ONEBOX_STATUS_FAILURE;
	}

	if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_HIDESSID, 
				NULL, ap_params->hide_ssid, 0)) { 
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Unable to set HIDE SSID Ioctl\n"));
	}

	if (((ap_params->ssid != NULL) && 
				onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_SSID, 
					ap_params->ssid, 0, ap_params->ssid_len)) < 0) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Unable to set SSID Ioctl\n"));
		return ONEBOX_STATUS_FAILURE;
	}

	/*** Beacon period is not supported from supplicant as of Now**/
	/* Per vap Diff Beacon interval is not supported **/
	if (((ap_params->dtim_period != 0) && 
				onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_DTIM_PERIOD, 
					NULL, ap_params->dtim_period, 0)) < 0) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Unable to set DTIM Period IOCTL\n"));
		return ONEBOX_STATUS_FAILURE;
	}

	/*** Find The RSN IE and WPA IE in the tail buffer*/
	if(ap_params->tail) {
		tail_len = ap_params->tail_len;
		tail = (uint8_t *)ap_params->tail;
		while(tail_len && tail) {
			switch(*tail) {
				case IEEE80211_ELEMID_RSN:
					wpa_rsn = tail;
					break;
				case IEEE80211_ELEMID_HTCAP:
					ht_cap_info = *(uint16_t *)(tail+2);
					break;
				case IEEE80211_ELEMID_VENDOR:
					if((tail[5] == 0x1 ) && (tail[4] == 0xf2)) { //WPA Security IE OUI
						wpa_ie = tail;
					}
					break;
				default:
					break;
			}
			tail_len -= (tail[1] + 2);
			if(!tail_len)
				break;
			tail = (tail + tail[1] + 2);
		}
	}

	if (ap_params->privacy) {

		if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_PRIVACY, NULL, 1, 0) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set Privacy Ioctl\n", __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}

		if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_AUTHMODE, 
					NULL, IEEE80211_AUTH_AUTO, 0) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set Authmode IOCTL\n", __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}

		if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_WPA, NULL, 
					ap_params->security.wpa_versions, 0) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set WPA IOCTL\n", __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}

		if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_PRIVACY,
					NULL, 1, 0) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set PRIVACY IOCTL\n", __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}
	}
	else
	{
		if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_PRIVACY, NULL, 0, 0) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set Privacy Ioctl\n",
									__func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}

		if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_AUTHMODE, 
					NULL, IEEE80211_AUTH_OPEN, 0) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set Authmode IOCTL\n",
									__func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}
	}

	if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_SHORTGI, 
				NULL, ht_cap_info, 0) < 0) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set SHORTGI IOCTL\n", __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}

	if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_WPS, NULL, 1, 0) < 0) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set WPS IOCTL\n", __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}

	if (ap_params->beacon_ies && ap_params->beacon_ies_len) {
		if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_APPIE, ap_params->beacon_ies, 
					(IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_BEACON),
					ap_params->beacon_ies_len) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d IE Ioctl not set\n", __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}
	}

	if (ap_params->proberesp_ies && ap_params->proberesp_ies_len) {
		if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_APPIE, ap_params->proberesp_ies, 
					(IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_PROBE_RESP),
					ap_params->proberesp_ies_len) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d IE Ioctl not set\n", __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}
	}

	if (ap_params->assocresp_ies && ap_params->assocresp_ies_len) {
		if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_APPIE, ap_params->assocresp_ies, 
					(IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_ASSOC_RESP),
					ap_params->assocresp_ies_len) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d IE Ioctl not set\n", __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}
	}

	if (wpa_rsn && wpa_ie) {
		nl80211_memcpy(data , wpa_rsn, wpa_rsn[1]+2);
		nl80211_memcpy((data+wpa_rsn[1]+2) , wpa_ie, wpa_ie[1]+2);
		total_rsn_wpa_ie_len = (wpa_rsn[1]+2 + wpa_ie[1]+2);
	} else if (wpa_rsn) {
		nl80211_memcpy(data , wpa_rsn, wpa_rsn[1]+2);
		total_rsn_wpa_ie_len = wpa_rsn[1]+2;
	}else if (wpa_ie) {
		nl80211_memcpy(data , wpa_ie, wpa_ie[1]+2);
		total_rsn_wpa_ie_len = wpa_ie[1]+2;
	}

	if(ap_params->security.wpa_versions) {
		if((onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_APPIE, data, 
						(IEEE80211_APPIE_WPA & IEEE80211_FC0_SUBTYPE_MASK),
						total_rsn_wpa_ie_len) < 0)) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d IE Ioctl not set\n", __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}
	}
#if(LINUX_VERSION_CODE > KERNEL_VERSION(3,4,0))
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, ("Channel is %d\n", ap_params->channel));
	vap->cfg80211_ap_set_channel = ap_params->channel;
#endif

	if((ap_params->channel >= 2412) && (ap_params->channel <= 2477))
	{
		vap->iv_des_mode = IEEE80211_MODE_11NG;
	}
	else if ((ap_params->channel >= 5180) && (ap_params->channel <= 5825))
	{
		vap->iv_des_mode = IEEE80211_MODE_11NA;
	}

	if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_HTCONF, NULL,
				ap_params->channel_width, 0) < 0) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Unable to set Bandwidth IOCTL\n"));
		return ONEBOX_STATUS_FAILURE;
	}

	if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_CHANNEL, NULL,
				vap->cfg80211_ap_set_channel, 0) ) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set Channel Ioctl\n", __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}

	memset(&mlme, 0, sizeof(mlme));
	mlme.im_op = IEEE80211_MLME_DEAUTH;
	mlme.im_reason = 2;
	if (ap_params->ssid != NULL) {
		memcpy(mlme.im_ssid, ap_params->ssid, ap_params->ssid_len);
	}

	mlme.im_ssid_len = ap_params->ssid_len;

	if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_MLME, &mlme, 
				0, sizeof(mlme)) < 0) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set MLME Ioctl\n", __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}

	vap->block_beacon_interrupt = 0;
	return ONEBOX_STATUS_SUCCESS;
}

#if(LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0))
void obm_notify_cfg80211(struct net_device *dev, enum ieee80211_notify_cac_event cmd)
{
	uint8_t type = 0;
	struct wireless_dev	*wdev = dev->ieee80211_ptr;

	struct ieee80211vap *vap = netdev_priv(dev);

	if (!vap) {
			if_printf( vap->iv_ifp, "In %s Line %d unable to get VAP pointer\n", __func__, __LINE__);
	}

	if (!wdev){
		if_printf(vap->iv_ifp, " In %s line no %d, wdev pointer is NULL\n", __func__, __LINE__);
	}

	switch( cmd )
	{
		case IEEE80211_NOTIFY_CAC_START:
		case IEEE80211_NOTIFY_CAC_STOP:
			/* CFG80211 is not expecting a CAC start notification */
			/* we send CAC_STOP notification only when we receive CAC_RADAR 
			 * notification during CAC time.  */
			return;
		case IEEE80211_NOTIFY_CAC_EXPIRE:
			type = NL80211_RADAR_CAC_FINISHED;
			break;
		case IEEE80211_NOTIFY_CAC_RADAR:
			type = NL80211_RADAR_DETECTED;
			break;
	}

	obm_cfg80211_radar_notify(dev, type, &vap->ap_params);
	return;
}	

int obm_start_dfs_cac( struct net_device *ndev, 
		struct obm_cfg80211_ap_params *ap_params)
{
	struct ieee80211vap *vap = netdev_priv(ndev);

	if (vap == NULL) {
		if_printf( vap->iv_ifp, "In %s Line %d unable to get VAP pointer\n", __func__, __LINE__);
	}


	memcpy(&vap->ap_params, ap_params, sizeof(struct obm_cfg80211_ap_params));

#if(LINUX_VERSION_CODE > KERNEL_VERSION(3,4,0))
	if_printf( vap->iv_ifp, "Channel is %d\n", ap_params->channel);
	vap->cfg80211_ap_set_channel = ap_params->channel;
#endif
	if_printf( vap->iv_ifp, "ap_params.channel_width = %d\n", ap_params->channel_width);
	if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_HTCONF, NULL,
				ap_params->channel_width, 0) < 0) {
		if_printf( vap->iv_ifp, "Unable to set Bandwidth IOCTL\n");
		return ONEBOX_STATUS_FAILURE;
	}

	if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_CHANNEL, NULL,
				vap->cfg80211_ap_set_channel, 0) ) {
		if_printf( vap->iv_ifp, "%s Line %d Unable to set Channel Ioctl\n", __func__, __LINE__);
		return ONEBOX_STATUS_FAILURE;
	}

	return ONEBOX_STATUS_SUCCESS;
}

int obm_set_mac_acl(struct net_device *ndev, uint8_t acl_policy, 
				uint8_t no_of_entries, 
				acl_mac_address *mac_addr )
{

		uint8_t ii = 0, jj = 0;
		struct ieee80211vap *vap = netdev_priv(ndev);

		if (vap == NULL) {
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s Line %d unable to get VAP pointer\n", __func__, __LINE__));
		}

		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ACL, ("%s Received ACL Request %d\n", __func__,acl_policy));
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ACL, ("%s params->n_acl_entries %d\n", __func__,no_of_entries));

		if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_MACCMD, NULL, (acl_policy) ? IEEE80211_MACCMD_POLICY_ALLOW
								: IEEE80211_MACCMD_POLICY_DENY, 0)) {
				return ONEBOX_STATUS_FAILURE;
		}

		for ( jj = 0; jj < no_of_entries; jj++ ) {
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ACL, ("MAC ADDR is : "));
				for ( ii = 0; ii < ETH_ALEN; ii++) {
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ACL, ("%02x", mac_addr[jj].addr[ii]));
						if (ii != 5)
								IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ACL, (":"));
				}
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ACL, ("\n"));

				if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_ADDMAC, mac_addr[jj].addr, 0, IEEE80211_ADDR_LEN)) {
						return ONEBOX_STATUS_FAILURE;
				}
		}
		return ONEBOX_STATUS_SUCCESS;
}
#endif

#ifdef ENABLE_P2P_SUPPORT
int obm_remain_on_chan_cfg(struct net_device *ndev, int channel_num, uint32_t duration, u64 *cookie)
{

	struct ieee80211vap *vap;

	vap = netdev_priv(ndev);
	vap->p2p->cookie = *cookie;

	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_REMAIN_ON_CHANNEL, &duration, channel_num, sizeof(uint32_t)) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set REMAIN ON CHANNEL Ioctl\n", __func__, __LINE__));
		return -1;
	}
	return ONEBOX_STATUS_SUCCESS;
}

int obm_cancel_remain_on_chan_cfg(struct net_device *ndev)
{

	struct ieee80211vap *vap;

	vap = netdev_priv(ndev);

	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_CANCEL_REMAIN_ON_CHANNEL, NULL, 0, sizeof(uint32_t)) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set CANCEL REMAIN ON CHANNEL Ioctl\n", __func__, __LINE__));
		return -1;
	}
	return ONEBOX_STATUS_SUCCESS;
}

int obm_probe_req_report (struct net_device *ndev, int report)
{
	struct ieee80211vap *vap;
	vap = netdev_priv(ndev);

	if(onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_PROBE_REQ_REPORT, NULL, report, sizeof(uint32_t)) < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set CANCEL REMAIN ON CHANNEL Ioctl\n", __func__, __LINE__));
		return -1;
	}
	return ONEBOX_STATUS_SUCCESS;
}
#endif //ENABLE_P2P_SUPPORT

int obm_send_probe_resp(struct net_device *ndev,const u8 *data,unsigned int freq,size_t pktlen)
{
	struct ieee80211vap *vap;
	struct mbuf *m;
	struct sk_buff *skb = NULL;
	int total_len = 0;

	vap = netdev_priv(ndev);
	struct ieee80211_node *bss = vap->iv_bss;
	struct ieee80211com *ic = vap->iv_ic;

	total_len = pktlen  + ic->ic_headroom;
	skb = dev_alloc_skb(total_len);
	if(skb == NULL)
	{
		printk("%s %d: Couldn't allocate Skb \n", __func__, __LINE__);
		return ONEBOX_STATUS_FAILURE;   
	}

	skb_reserve(skb, ic->ic_headroom);      
	skb_put(skb, pktlen);
	memcpy(skb->data, data, skb->len);
	m = onebox_translate_skb_to_netcb_m(skb);

	if(freq == ic->ic_curchan->ic_freq){
		ic->ic_raw_xmit(bss,m, NULL);
	}else {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to Send Mangement Frames\n",__func__,__LINE__));
		return ONEBOX_STATUS_FAILURE;
	}
	return ONEBOX_STATUS_SUCCESS;
}

void apply_chan_flags (struct ieee80211com *ic, uint32_t center_freq, uint32_t flags, int32_t dbm) 
{
	struct ieee80211_channel *ic_chan;

	if(!ic) 
		return;

	ic_chan =  ieee80211_find_channel(ic, center_freq, flags);

	if (!ic_chan) 
			return;
	else {
		if (flags & CFG80211_CHAN_DISABLED) {
			ic_chan->scan_on = 0;
			return;
		}
		ic_chan->scan_on = 1;
		if (flags & CFG80211_CHAN_PASSIVE_SCAN )
			ic_chan->ic_flags |= IEEE80211_CHAN_PASSIVE;
		else
			ic_chan->ic_flags &= ~IEEE80211_CHAN_PASSIVE;

		if (flags & CFG80211_CHAN_RADAR) 
			ic_chan->ic_flags |= IEEE80211_CHAN_DFS;
	}
	ic_chan->ic_maxregpower = dbm;
	return;
}

void onebox_update_country(uint8_t *country, struct cfg80211_priv *cfg_priv) 
{
	struct ieee80211com *ic;

	ic = cfg_priv->ic;

	if (!ic)
		return ;

	if ((country[0] != 0) && (country[1] !=0)) {
		ic->ic_regdomain.isocc[0] = country[0];
		ic->ic_regdomain.isocc[1] = country[1];
		ic->ic_regdomain.pad[0] = 0;			
	}

}

/* Short GI defines as expected by Net80211 layer */
#define SHORTGI_20_EN   0x0020
#define SHORTGI_40_EN   0x0040

int obm_siwrate(struct wiphy *wiphy,
		struct net_device *dev,
		const u8 *peer,
		int legacy_value, int mcs_value
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
		, int gi
#endif
		)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211com *ic = vap->iv_ic;
	int user_rate, mode = vap->iv_des_mode, is11n,check;
	struct ieee80211_rate *rs_supp = NULL; 
	struct ieee80211_node *ni = vap->iv_bss; 
	struct ieee80211_rateset *rs = NULL;
	struct ieee80211_htrateset *htrs = NULL;
	int mcs[] = {13, 26, 39, 52, 78, 104, 117, 130};
	uint8_t ii, found = 0 ;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
	if(gi == 1) {
		if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_SHORTGI, 
					NULL, (SHORTGI_20_EN | SHORTGI_40_EN), 0) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set SHORTGI IOCTL\n", __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}
		return ONEBOX_STATUS_SUCCESS;
	}else if(gi == 2){
		if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_SHORTGI, 
					NULL, 0, 0) < 0) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s Line %d Unable to set SHORTGI IOCTL\n", __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}
		return ONEBOX_STATUS_SUCCESS;
	}
#endif
	if (legacy_value != 0xfff)
		check = legacy_value;
	else
		check = mcs_value;

    switch (check) {
        case LEGACY_INDEX_1:
            if (legacy_value != 0xfff)
                user_rate = IEEE80211_RATE_1M;
            else
                user_rate = IEEE80211_RATE_6_5M;
            break;
        case LEGACY_INDEX_2:
            if (legacy_value != 0xfff)
                user_rate = IEEE80211_RATE_2M;
            else
                user_rate = IEEE80211_RATE_13M;
            break;
        case LEGACY_INDEX_4:
            if (legacy_value != 0xfff)
                user_rate = IEEE80211_RATE_5_5M;
            else
                user_rate = IEEE80211_RATE_19_5M;
            break;
        case LEGACY_INDEX_8:
            if (legacy_value != 0xfff)
                user_rate = IEEE80211_RATE_11M; 
            else
                user_rate = IEEE80211_RATE_26M;
            break;
        case LEGACY_INDEX_16:
            if (legacy_value != 0xfff)
                user_rate = IEEE80211_RATE_6M;
            else
                user_rate = IEEE80211_RATE_39M;
            break;
        case LEGACY_INDEX_32:
            if (legacy_value != 0xfff)
                user_rate = IEEE80211_RATE_9M;
            else
                user_rate = IEEE80211_RATE_52M;
            break;
        case LEGACY_INDEX_64:
            if (legacy_value != 0xfff)
                user_rate = IEEE80211_RATE_12M;
            else
                user_rate = IEEE80211_RATE_58_5M;
            break;
        case LEGACY_INDEX_128:
            if (legacy_value != 0xfff)
                user_rate = IEEE80211_RATE_18M;
            else
                user_rate = IEEE80211_RATE_65M;
            break;
        case LEGACY_INDEX_256:
            user_rate = IEEE80211_RATE_24M;
            break;
        case LEGACY_INDEX_512:
            user_rate =IEEE80211_RATE_36M;
            break;
        case LEGACY_INDEX_1024:
            user_rate = IEEE80211_RATE_48M;
            break;
        case LEGACY_INDEX_2048:
            user_rate = IEEE80211_RATE_54M;
            break;
        default:
            user_rate = IEEE80211_RATE_AUTO;
            break;
    }

	if (user_rate == IEEE80211_RATE_AUTO) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, (" Setting to Auto rate : Invalid rate, Please give valid rate\n"));
		user_rate = IEEE80211_FIXED_RATE_NONE;
		vap->hal_priv_vap->fixed_rate_enable = 0;
		vap->vap_dynamic_update(vap);
		return 0;
	}

	rs_supp = (struct ieee80211_rate *)&ic->ic_sup_rates[mode]; /* NB: 11n maps to legacy */
	is11n = (mode == IEEE80211_MODE_11NA || mode == IEEE80211_MODE_11NG);
	if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
		if ((!ishtrate(ic, user_rate)) && (!checkrate((const struct ieee80211_rateset *)rs_supp, user_rate) && (!is11n || !checkmcs(user_rate)))) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Fixed rate : Invalid rate, Please give valid rate\n"));
			return EINVAL;
		}
		found = 1;
	}	else if (vap->iv_opmode == IEEE80211_M_STA) {
		if(vap->iv_state == IEEE80211_S_RUN) {
			/*set values if vap in connected state*/
			rs = (struct ieee80211_rateset *)&ni->ni_rates;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("In %s line %d supported rates by ap\n",__func__,__LINE__));
			htrs = (struct ieee80211_htrateset *)&ni->ni_htrates;
			if (checkrate(rs, user_rate)) {
				for (ii = 0; ii < rs->rs_nrates; ii++) {
					if ((rs->rs_rates[ii] & IEEE80211_RATE_VAL) == user_rate) {	
						found = 1;
					}
				}
			} else if ((htrs->rs_nrates > 0) && ishtrate(ic, user_rate)) {
				for (ii = 0; ii < htrs->rs_nrates; ii++) {
					if (mcs[htrs->rs_rates[ii] & IEEE80211_RATE_VAL] == user_rate) {
						found = 1;
					}
				}
			} else {
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("\n<<<given rate is  not supported in current mode>>>\n"));
				return EINVAL;
			}						
		}
	}

	if (found) {
		set_fixedrate(vap, user_rate);
		vap->iv_txparms[vap->iv_des_mode].ucastrate = user_rate;
		vap->vap_dynamic_update(vap);
		return 0;
	}
	return EINVAL;
}

int
obm_dump_station_info(struct net_device *dev,struct local_info * data, u8 *mac,struct station_info *sinfo, int idx )
{
	struct ieee80211vap *vap = netdev_priv(dev);
	struct ieee80211com *ic = NULL;
	struct ieee80211_node_table *nt = NULL;
	struct ieee80211_node *ni = NULL;
	uint32_t index = 0;

	if (vap->iv_state != IEEE80211_S_RUN)	
		data->connected = false;
	else
		data->connected = true;

	if ((vap->iv_opmode != IEEE80211_M_STA)) {
		ic = vap->iv_ic;
		nt = &ic->ic_sta;
		//data->legacy= (vap->iv_txparms[vap->iv_des_mode].ucastrate*10)/2;
		data->mode = NL80211_IFTYPE_AP; 

		TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
			/* First ni will be our AP ni, so incrementing the received index with 1*/
			if (index == (idx+1)) {
				memcpy(mac, ni->ni_macaddr, IEEE80211_ADDR_LEN);
				break;
			}
#if 0
			if ((ni->ni_associd & 0xFFF )== (idx+1)) { /** Index should be with 1 because associd starts from 1*/
				memcpy(mac, ni->ni_macaddr, IEEE80211_ADDR_LEN);
				break;
			}
#endif
			index++;
		}
		if (ni == NULL)
			return ONEBOX_STATUS_FAILURE;

	} else {
		if (idx)
			return ONEBOX_STATUS_FAILURE;
		memcpy(mac, vap->iv_bss->ni_macaddr, IEEE80211_ADDR_LEN);
		data->mode = NL80211_IFTYPE_STATION;
		ni = vap->iv_bss;
	}

	data->rx_bytes = ni->ni_stats.ns_rx_bytes;
	data->rx_data = ni->ni_stats.ns_rx_data;
	data->tx_bytes = ni->ni_stats.ns_tx_bytes;
	data->tx_data = ni->ni_stats.ns_tx_data;
	#if 0
	data->signal = ni->ni_avgrssi;
	if (data->signal < 0)
		data->signal = -data->signal;
	if (data->signal > 90)
		data->signal = 90;
	#endif
	//! In AP mode rssi value is taken from the last received packet from that particular station.
	//! In STA mode rssi value is taken from the last received beacon.
	if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
		data->signal = -abs(ni->ni_avgrssi);
	} else if (vap->iv_opmode == IEEE80211_M_STA) { 
		data->signal = -abs(vap->hal_priv_vap->rssi);
	}
	data->rx_legacy_rate = ni->ni_rxrate;
	if (ni->ni_ies.wme_ie)
		data->wme_ie = true;

	if (vap->hal_priv_vap->fixed_rate_enable) {
		data->tx_legacy_rate = vap->hal_priv_vap->rate_hix;
		if((vap->iv_flags_ht & IEEE80211_FHT_SHORTGI20) || (vap->iv_flags_ht & IEEE80211_FHT_SHORTGI40))
		{
			/*checking whether the connected node supports SHORT GI*/
			if ((ni->ni_htcap & IEEE80211_HTCAP_SHORTGI40) || (ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20))
				if(((ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20))  && (vap->iv_flags_ht & IEEE80211_FHT_SHORTGI20))
				{
					if(vap->hal_priv_vap->rate_hix & 0x100) /*check this for MCS rates only */
					{
						data->tx_legacy_rate |= BIT(9);/* Indicates shortGi HT Rate */
					}			
				}			
		}
	} else
		data->tx_legacy_rate = ni->ni_txrate;

	return ONEBOX_STATUS_SUCCESS;

}

void obm_set_pwr_state(struct net_device *dev, bool enabled)
{
#define IOCTL_PATH 2
#define SLEEP_TYPE 1 /* 1 for LP, 2 for ULP */
	struct ieee80211vap *vap = netdev_priv(dev);
	struct ieee80211com *ic = vap->iv_ic;
	struct pwr_save_params pwr_save;
	bool ps_status;	
	if(enabled)
	{
		pwr_save.ps_en = enabled;
		ps_status = 1;
	}
	else
	{
		pwr_save.ps_en = 0;
		ps_status = 0;
	}
	pwr_save.sleep_type = SLEEP_TYPE; 
	pwr_save.tx_threshold = 0;
	pwr_save.rx_threshold = 0;
	pwr_save.tx_hysterisis = 0;
	pwr_save.rx_hysterisis = 0;
	pwr_save.monitor_interval = 0;
	pwr_save.listen_interval = 0;
	pwr_save.num_beacons_per_listen_interval = 0;
	pwr_save.dtim_interval_duration = 0;
	pwr_save.num_dtims_per_sleep = 0;
	pwr_save.deep_sleep_wakeup_period = 1;

	driver_ps.update_ta = 1;
	strncpy((char *)&vap->hal_priv_vap->ps_params_ioctl, (char *)&pwr_save, sizeof(struct pwr_save_params));
	ic->ic_pwr_save(vap, ps_status, IOCTL_PATH);
#undef IOCTL_PATH
#undef SLEEP_TYPE
	return;
}

#if defined ONEBOX_CONFIG_CFG80211 && defined CONFIG_ACS
int 
obm_dump_survey( struct net_device *dev, int idx, struct survey_dump *survey)
{
	struct ieee80211vap *vap = netdev_priv(dev);
	IEEE80211_LOCK (vap->iv_ic);	
	memcpy(survey, &vap->iv_ic->obm_survey[idx], sizeof(struct survey_dump));
	//memcpy(survey->channel, &vap->iv_ic->ic_curchan, sizeof(vap->iv_ic->ic_curchan));
	memset(&vap->iv_ic->obm_survey[idx], 0, sizeof(struct survey_dump));
	IEEE80211_UNLOCK(vap->iv_ic);
	return 0;
}
#endif

int 
obm_get_station_info(struct net_device *dev,struct local_info * data, u8 *mac,struct station_info *sinfo)
{
	struct ieee80211vap *vap = netdev_priv(dev);
	struct ieee80211_node *ni = NULL;
	uint32_t index = 0;

	if(vap->iv_opmode == IEEE80211_M_HOSTAP)
		return 2;

	if (vap->iv_state != IEEE80211_S_RUN)	
		data->connected = false;
	else
		data->connected = true;
	if ((vap->iv_opmode != IEEE80211_M_STA)) {
		return ONEBOX_STATUS_FAILURE;
	}
	else
	{
		memcpy(mac, vap->iv_bss->ni_macaddr, IEEE80211_ADDR_LEN);
		data->mode = NL80211_IFTYPE_STATION;
		ni = vap->iv_bss;
	}
	data->rx_bytes = ni->ni_stats.ns_rx_bytes;
	data->rx_data = ni->ni_stats.ns_rx_data;
	data->tx_bytes = ni->ni_stats.ns_tx_bytes;
	data->tx_data = ni->ni_stats.ns_tx_data;
#if 0
	data->signal = ni->ni_avgrssi;
	
	printk("tx bytes and tx packets are %d and %d in func %s:%d\n",data->tx_data,data->tx_bytes,__func__,__LINE__);
	printk("rx bytes and rx packets are %d and %d in func %s:%d\n",data->rx_data,data->rx_bytes,__func__,__LINE__);

	if (data->signal < 0)
		data->signal = -data->signal;
	if (data->signal > 90)
		data->signal = 90;
#endif
	//! In AP mode rssi value is taken from the last received packet from that particular station.
	//! In STA mode rssi value is taken from the last received beacon. 
	if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
		data->signal = -abs(ni->ni_avgrssi);
	} else if (vap->iv_opmode == IEEE80211_M_STA) { 
		data->signal = -abs(vap->hal_priv_vap->rssi);
	}
	data->rx_legacy_rate = ni->ni_rxrate;
	if (ni->ni_ies.wme_ie)
		data->wme_ie = true;

	if (vap->hal_priv_vap->fixed_rate_enable) {
		data->tx_legacy_rate = vap->hal_priv_vap->rate_hix;
		if((vap->iv_flags_ht & IEEE80211_FHT_SHORTGI20) || (vap->iv_flags_ht & IEEE80211_FHT_SHORTGI40))
		{
			/*checking whether the connected node supports SHORT GI*/
			if ((ni->ni_htcap & IEEE80211_HTCAP_SHORTGI40) || (ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20))
				if(((ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20))  && (vap->iv_flags_ht & IEEE80211_FHT_SHORTGI20))
				{
					if(vap->hal_priv_vap->rate_hix & 0x100) /*check this for MCS rates only */
					{
						data->tx_legacy_rate |= BIT(9);/* Indicates shortGi HT Rate */
					}			
				}			
		}
	} else
		data->tx_legacy_rate = ni->ni_txrate;

	return ONEBOX_STATUS_SUCCESS;

}

#ifdef ONEBOX_CONFIG_WOWLAN
int onebox_cfg80211_config_wowlan(struct net_device *ndev, u16 wow_triggers,
		bool wow_enable)
{
	struct ieee80211vap *vap = netdev_priv(ndev);
	struct ieee80211_wowlan_config wowcfg;

	wowcfg.wow_triggers = wow_triggers;
	wowcfg.wow_enable = wow_enable;

	if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_WOWLAN, &wowcfg,
				0, sizeof(wowcfg))) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR,
				("%s: Failed to configure WoWLAN\n",
				 __func__));
		return -1;
	}
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR,
			("%s: Successfully configured WoWLAN\n",
			 __func__));
	return 0;
}
#endif

#ifdef ONEBOX_CONFIG_GTK_OFFLOAD
int obm_cfg80211_set_rekey_data(struct net_device *dev,
                                struct obm_cfg80211_gtk_rekey_data *data)
{
  struct ieee80211vap *vap = netdev_priv(dev);
  if (onebox_prepare_ioctl_cmd(vap, IEEE80211_IOC_GTK_REKEY_INFO, data, 0, 
        sizeof(struct obm_cfg80211_gtk_rekey_data))) {
    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR,
        ("%s: Failed to send gtk rekey data\n",
         __func__));
    return -1;
  }
  IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR,
      ("%s: Successfully sent gtk rekey data\n",
       __func__));
  return 0;
}
#endif
int	onebox_set_cqm_rssi_config(struct wiphy *wiphy,
        struct net_device *dev,
        s32 rssi_thold, u32 rssi_hyst)
{
    struct ieee80211vap *vap = netdev_priv(dev);
    vap->rssi_hyst = rssi_hyst;
    vap->rssi_thold = abs(rssi_thold);
    return 0;
}

EXPORT_SYMBOL(onebox_inform_bss_to_cfg80211);
#endif


