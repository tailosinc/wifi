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
static struct onebox_net80211_operations net80211_ops =
{
	ieee80211_find_rxnode,      /* .onebox_find_rxnode      = ieee80211_find_rxnode */
	ieee80211_input_all,        /* .onebox_input_all        = ieee80211_input_all  */
	ieee80211_ifattach,         /* .onebox_ifattach         = ieee80211_ifattach  */
	ieee80211_ifdetach,         /* .onebox_ifdetach         = ieee80211_ifdetach   */
	ieee80211_vap_setup,        /* .onebox_vap_setup        = ieee80211_vap_setup  */
	ieee80211_vap_attach,       /* .onebox_vap_attach       = ieee80211_vap_attach */
	ieee80211_vap_detach,       /* .onebox_vap_detach       = ieee80211_vap_detach */
#ifndef BYPASS_RX_DATA_PATH
	ieee80211_encap,            /* .onebox_encap            = ieee80211_encap   */
#endif
	ieee80211_iterate_nodes,    /* .onebox_iterate_nodes    = ieee80211_iterate_node */
	ieee80211_beacon_alloc,     /* .onebox_beacon_alloc     = ieee80211_beacon_alloc */
	ieee80211_free_node,        /* .onebox_free_node        = ieee80211_free_node */
	ieee80211_find_node_locked, /* .onebox_find_node_locked = ieee80211_find_node_locked*/
	ieee80211_beacon_update,    /* .onebox_beacon_update    = ieee80211_beacon_update*/
	ieee80211_media_change,     /* .onebox_media_change     = ieee80211_media_change*/
	ieee80211_media_status,     /* .onebox_media_status     = ieee80211_media_status    */
	ieee80211_radiotap_attach,  /* .onebox_radiotap_attach  = ieee80211_radiotap_attach*/
	ieee80211_radiotap_tx,      /* .onebox_radiotap_tx      = ieee80211_radiotap_tx*/
#ifdef ENABLE_P2P_SUPPORT
	p2p_init,                   /* .onebox_p2p_init         = p2p_init */
#endif
	ieee80211_find_node,        /* .onebox_find_vap_node    = ieee80211_find_vap_node */
	ieee80211_notify_michael_failure, /* .onebox_rxmic_failure = ieee80211_notify_michael_failure */
	ieee80211_notify_scan_done,
	ieee80211_probereq_confirm,
#ifdef PWR_SAVE_SUPPORT
	.onebox_ieee80211_sta_leave = ieee80211_reset_adapter,
#endif
#ifdef ONEBOX_CONFIG_CFG80211
	.onebox_cfg80211_attach = cfg80211_attach,
	.onebox_cfg80211_register_wireless_dev = cfg80211_wdev_register,
#endif
	ieee80211_beacon_miss,      /* .onebox_beacon_miss  = ieee80211_beacon_miss */
	ieee80211_node_pwrsave,
	ieee80211_setbasicrates,	/* .onebox_setbasicrates = ieee80211_setbasicrates*/
        .onebox_media_init = ieee80211_media_init,
	.onebox_scanreq_signal = scanreq_signal,
	.onebox_radar_notify = ieee80211_dfs_notify_radar,
#ifdef ONEBOX_CONFIG_CFG80211
	.onebox_cfg80211_callbacks = cfg80211_callbacks,
#endif
	.onebox_find_channel = findchannel,
#ifdef IEEE80211K
	.onebox_meas_rpt = ieee80211_send_meas_rpt,
#endif
};  

struct onebox_net80211_operations *onebox_get_net80211_operations (void)
{
	return (&net80211_ops);
}
EXPORT_SYMBOL(onebox_get_net80211_operations);
