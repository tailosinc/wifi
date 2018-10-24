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

#include <net/cfg80211.h>
#include "cfg80211_ioctl.h"

#define ONEBOX_STATUS int
#define ONEBOX_STATUS_FAILURE -1
#define ONEBOX_STATUS_SUCCESS 0


#define DFL_FRAG_THRSH 2346
#define DFL_RTS_THRSH 2346

#define ETH_ADDR_LEN  6

#define MAX_CHANNELS_24GHZ 14
#define MAX_CHANNELS_5GHZ  25

#define IEEE80211_IOC_APPIE  95/* application IE's */
#define	IEEE80211_FC0_TYPE_MGT			0x00
#define	IEEE80211_FC0_SUBTYPE_PROBE_REQ		0x40

#define RSI_RATE_00   0x00
#define RSI_RATE_1    0x0
#define RSI_RATE_2    0x2
#define RSI_RATE_5_5  0x4
#define RSI_RATE_11   0x6
#define RSI_RATE_6    0x8b
#define RSI_RATE_9    0x8f
#define RSI_RATE_12   0x8a
#define RSI_RATE_18   0x8e
#define RSI_RATE_24   0x89
#define RSI_RATE_36   0x8d
#define RSI_RATE_48   0x88
#define RSI_RATE_54   0x8c

#define RSI_RATE_1M     1 
#define RSI_RATE_2M     2 
#define RSI_RATE_5_5M   5.5 
#define RSI_RATE_11M    11 
#define RSI_RATE_6M     6 
#define RSI_RATE_9M     9 
#define RSI_RATE_12M    12 
#define RSI_RATE_18M    18 
#define RSI_RATE_24M    24 
#define RSI_RATE_36M    36 
#define RSI_RATE_48M    48 
#define RSI_RATE_54M    54 

#define STD_RATE_MCS7 0x82
#define STD_RATE_MCS6 0x75
#define STD_RATE_MCS5 0x68
#define STD_RATE_MCS4 0x4E
#define STD_RATE_MCS3 0x34            
#define STD_RATE_MCS2 0x27
#define STD_RATE_MCS1 0x1A
#define STD_RATE_MCS0 0x0D 
#define STD_RATE_54   0x6c
#define STD_RATE_48   0x60
#define STD_RATE_36   0x48
#define STD_RATE_24   0x30
#define STD_RATE_18   0x24
#define STD_RATE_12   0x18
#define STD_RATE_11   0x16
#define STD_RATE_09   0x12
#define STD_RATE_06   0x0C
#define STD_RATE_5_5  0x0B
#define STD_RATE_02   0x04
#define STD_RATE_01   0x02

/* cipher suite selectors */
#define WLAN_CIPHER_SUITE_USE_GROUP	0x000FAC00
#define WLAN_CIPHER_SUITE_WEP40		0x000FAC01
#define WLAN_CIPHER_SUITE_TKIP		0x000FAC02
/* reserved: 				0x000FAC03 */
#define WLAN_CIPHER_SUITE_CCMP		0x000FAC04
#define WLAN_CIPHER_SUITE_WEP104	0x000FAC05
#define WLAN_CIPHER_SUITE_AES_CMAC	0x000FAC06
#define OBM_MAX_TXPWR 20


static const u32 cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_AES_CMAC,
};

enum ieee80211_roamingmode {
	IEEE80211_ROAMING_DEVICE= 0,	/* driver/hardware control */
	IEEE80211_ROAMING_AUTO	= 1,	/* 802.11 layer control */
	IEEE80211_ROAMING_MANUAL= 2,	/* application control */
};
struct ieee80211_rate ;

static struct ieee80211_rate obm_rates[] = {
	{ .bitrate = STD_RATE_01 * 5},
	{ .bitrate = STD_RATE_02 * 5},
	{ .bitrate = STD_RATE_5_5 * 5},
	{ .bitrate = STD_RATE_11 * 5},
	{ .bitrate = STD_RATE_06 * 5},
	{ .bitrate = STD_RATE_09 * 5},
	{ .bitrate = STD_RATE_12 * 5},
	{ .bitrate = STD_RATE_18 * 5},
	{ .bitrate = STD_RATE_24 * 5},
	{ .bitrate = STD_RATE_36 * 5},
	{ .bitrate = STD_RATE_48 * 5},
	{ .bitrate = STD_RATE_54 * 5},
};

#define AMPDU_DEF_MPDU_DENSITY  6

static struct ieee80211_channel obm_channels_2_4_GHz[] = {
	{ .hw_value = 1, .center_freq = 2412, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 2, .center_freq = 2417, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 3, .center_freq = 2422, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 4, .center_freq = 2427, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 5, .center_freq = 2432, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 6, .center_freq = 2437, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 7, .center_freq = 2442, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 8, .center_freq = 2447, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 9, .center_freq = 2452, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 10, .center_freq = 2457, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 11, .center_freq = 2462, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 12, .center_freq = 2467, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 13, .center_freq = 2472, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 14, .center_freq = 2484, .max_power = OBM_MAX_TXPWR },
};

static struct ieee80211_channel obm_channels_5GHz[] = {
	//Enable when supporting 11J
#if 0
	{ .hw_value = 7, .center_freq = 5035, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 8, .center_freq = 5040, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 9, .center_freq = 5045, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 11, .center_freq = 5055, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 12, .center_freq = 5060, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 16, .center_freq = 5080, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 34, .center_freq = 5170, .max_power = OBM_MAX_TXPWR },
#endif
	{ .hw_value = 36,  .center_freq = 5180, .max_power = OBM_MAX_TXPWR },
	//{ .hw_value = 38,  .center_freq = 5190, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 40,  .center_freq = 5200, .max_power = OBM_MAX_TXPWR },
	//{ .hw_value = 42,  .center_freq = 5210, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 44,  .center_freq = 5220, .max_power = OBM_MAX_TXPWR },
	//{ .hw_value = 46,  .center_freq = 5230, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 48,  .center_freq = 5240, .max_power = OBM_MAX_TXPWR },
#ifndef DISABLE_DFS_CHANNELS
	{ .hw_value = 52,  .center_freq = 5260, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 56,  .center_freq = 5280, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 60,  .center_freq = 5300, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 64,  .center_freq = 5320, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 100, .center_freq = 5500, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 104, .center_freq = 5520, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 108, .center_freq = 5540, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 112, .center_freq = 5560, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 116, .center_freq = 5580, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 120, .center_freq = 5600, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 124, .center_freq = 5620, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 128, .center_freq = 5640, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 132, .center_freq = 5660, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 136, .center_freq = 5680, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
	{ .hw_value = 140, .center_freq = 5700, .max_power = OBM_MAX_TXPWR , .flags = CFG80211_CHAN_RADAR | CFG80211_CHAN_PASSIVE_SCAN},
#endif
	{ .hw_value = 149, .center_freq = 5745, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 153, .center_freq = 5765, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 157, .center_freq = 5785, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 161, .center_freq = 5805, .max_power = OBM_MAX_TXPWR },
	{ .hw_value = 165, .center_freq = 5825, .max_power = OBM_MAX_TXPWR },
};
static const struct ieee80211_regdomain onebox_regdom = {
	.n_reg_rules = 4,
	.alpha2 =  "99",
	.reg_rules = {
		/* IEEE 802.11b/g, channels 1..11 */
		REG_RULE(2412-10, 2472+10, 40, 5, 20, 0),
		/* If any */
		/* IEEE 802.11 channel 14 - Only JP enables
		 * this and for 802.11b only
		 */
		REG_RULE(2484-10, 2484+10, 20, 5, 20, 0),
		/* IEEE 802.11a, channel 36..64 */
		REG_RULE(5150-10, 5350+10, 40, 5, 20, 0),
		/* IEEE 802.11a, channel 100..165 */
		REG_RULE(5470-10, 5850+10, 40, 5, 20, 0), }
};

struct ieee80211_supported_band obm_band_24ghz =  {
#if(LINUX_VERSION_CODE > KERNEL_VERSION(4, 6,0))
	.band = NL80211_BAND_2GHZ,
#else
	.band = IEEE80211_BAND_2GHZ,
#endif
	.bitrates   = obm_rates,
	.n_bitrates = ARRAY_SIZE(obm_rates),
	.n_channels = (MAX_CHANNELS_24GHZ),
	.ht_cap.ht_supported = true,
	.ht_cap.cap = (IEEE80211_HT_CAP_SUP_WIDTH_20_40 | 
			IEEE80211_HT_CAP_SGI_20		 | 
			(1 << IEEE80211_HT_CAP_RX_STBC_SHIFT)  | // 1--->Indicates Single RX STBC Stream
			IEEE80211_HT_CAP_SGI_40),

	.ht_cap.mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED,
	.ht_cap.mcs.rx_mask[0] = 0xff,
	.ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_16K,
	.ht_cap.ampdu_density = AMPDU_DEF_MPDU_DENSITY,
	.channels = obm_channels_2_4_GHz
};

struct ieee80211_supported_band obm_band_5ghz = {
#if(LINUX_VERSION_CODE > KERNEL_VERSION(4, 6,0))
	.band = NL80211_BAND_2GHZ,
#else
	.band = IEEE80211_BAND_5GHZ,
#endif
	.bitrates   = (obm_rates + 4),
	.n_bitrates = (ARRAY_SIZE(obm_rates) - 4),
	.n_channels = ARRAY_SIZE(obm_channels_5GHz),
	.ht_cap.ht_supported = true,
	.ht_cap.cap = (IEEE80211_HT_CAP_SUP_WIDTH_20_40 | 
			IEEE80211_HT_CAP_SGI_20	 | 
			(1 << IEEE80211_HT_CAP_RX_STBC_SHIFT)  | // 1--->Indicates Single RX STBC Stream
			IEEE80211_HT_CAP_SGI_40),

	.ht_cap.mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED,
	.ht_cap.mcs.rx_mask[0] = 0xff,
	.ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_16K,
	.ht_cap.ampdu_density = AMPDU_DEF_MPDU_DENSITY,
	.channels = obm_channels_5GHz

};

struct ieee80211_scan_req {
	int      sr_flags;
	u_int    sr_duration;   /* duration (ms) */
	u_int    sr_mindwell;   /* min channel dwelltime (ms) */
	u_int    sr_maxdwell;   /* max channel dwelltime (ms) */
	int sr_nssid;
#define IEEE80211_NWID_LEN 32
#define IEEE80211_IOC_SCAN_MAX_SSID 3
	struct 
	{
		uint8_t len;             /* length in bytes */
		uint8_t ssid[IEEE80211_NWID_LEN]; /* ssid contents */
	} sr_ssid[IEEE80211_IOC_SCAN_MAX_SSID];

//#define IEEE80211_MAX_FREQS_ALLOWED 25
#define IEEE80211_MAX_FREQS_ALLOWED 32

//#ifdef ENABLE_P2P_SUPPORT
	uint16_t num_freqs;
	uint16_t freqs[IEEE80211_MAX_FREQS_ALLOWED];
//#endif
};

/* Function prototypes */
//uint8_t cfg80211_wrapper_attach(struct net_device *dev, void *dev_ptr, int size);
struct cfg80211_priv* cfg80211_wrapper_attach(struct device *dev, u8 *mac_addr , struct ieee80211com *ic, u8 dual_band);
//void cfg80211_wrapper_free_wdev(struct wireless_dev *wdev);
void cfg80211_wrapper_detach(struct wiphy *wiphy);
struct wireless_dev* onebox_register_wireless_dev(struct net_device *dev, struct cfg80211_priv *cfg_priv, int opmode);

#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38))
ONEBOX_STATUS 
onebox_cfg80211_set_txpower(struct wiphy *wiphy, enum nl80211_tx_power_setting   type, int dbm);
#endif

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38))
ONEBOX_STATUS onebox_cfg80211_del_key(struct wiphy *wiphy, struct net_device *ndev, uint8_t index,
                                      const uint8_t *mac_addr );

ONEBOX_STATUS onebox_cfg80211_add_key(struct wiphy *wiphy,struct net_device *ndev,
                                      uint8_t index, const uint8_t  *mac_addr, struct key_params *params);
#else
ONEBOX_STATUS onebox_cfg80211_del_key(struct wiphy *wiphy, struct net_device *ndev, uint8_t index,
                                      bool pairwise, const uint8_t *mac_addr );

ONEBOX_STATUS onebox_cfg80211_add_key(struct wiphy *wiphy,struct net_device *ndev,
                                      uint8_t index, bool pairwise, const uint8_t  *mac_addr, struct key_params *params);
#endif

ONEBOX_STATUS 
onebox_cfg80211_set_default_key(struct wiphy *wiphy, struct net_device *ndev, uint8_t index
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38))
		, bool unicast, bool multicast
#endif
		);

struct cfg80211_priv {
	struct wireless_dev *wdev[4]; //PRAVEEN: change to MAX_VAP macro
	struct net_device *ndev[4]; 
	struct wiphy *wiphy;
	struct ieee80211com *ic;
};


ONEBOX_STATUS 
onebox_cfg80211_scan(struct wiphy *wiphy,

#if(LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0))
                      struct net_device *ndev,
#endif
                     struct cfg80211_scan_request *request);

struct wireless_dev* onebox_register_wiphy_dev(struct device *dev, int size);

int cfg80211_scan(void *temp_req, struct net_device *ndev, uint8_t num_chn,  uint16_t *chan,
                     const uint8_t *ie, size_t ie_len, int n_ssids, void *ssids);

int obm_cfg80211_add_intf(struct wiphy *wiphy,
		const char *name,
		u8 opmode,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,12,0))
		u32 *flags,
#endif
		u8 macaddr[6],
		struct cfg80211_priv *cfg_priv);

//int obm_cfg80211_del_intf(struct cfg80211_priv *cfg_priv, struct wireless_dev *wdev);
int obm_cfg80211_del_intf(struct cfg80211_priv *cfg_priv, struct net_device *ndev);
int obm_cfg_stop_ap(struct net_device *ndev);
int obm_cfg_start_ap(struct net_device *ndev, struct obm_cfg80211_ap_params *ap_params); 
int onebox_set_channel(struct net_device *ndev, int channel_num);
int obm_cfg_change_beacon(struct net_device *ndev, struct obm_cfg_ap_beacon_ie *ap_params); 
int obm_delete_connected_sta(struct net_device *ndev, const uint8_t *mac,
                         uint8_t opcode, uint16_t reason_code);
int obm_set_mac_acl(struct net_device *ndev, uint8_t acl_policy, uint8_t no_of_entries, acl_mac_address *mac_addr);
int obm_start_dfs_cac( struct net_device *ndev, struct obm_cfg80211_ap_params *ap_params);
#ifdef ENABLE_P2P_SUPPORT
int obm_remain_on_chan_cfg(struct net_device *ndev, int channel_num, uint32_t duration, u64 *cookie);
int obm_cancel_remain_on_chan_cfg(struct net_device *ndev);
int obm_probe_req_report (struct net_device *ndev, int report);
#endif //ENABLE_P2P_SUPPORT
void apply_chan_flags (struct ieee80211com *ic, uint32_t center_freq, uint32_t flags, int32_t dbm);
int onebox_set_bitrate_mask(struct wiphy *wiphy,struct net_device *dev,const u8 *peer,const struct cfg80211_bitrate_mask *mask);
int obm_siwrate(struct wiphy *wiphy, struct net_device *dev, const u8 *peer, int legacy_value
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0))
	, int mcs_value
#endif
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
      , int gi
#endif 
);

int obm_send_probe_resp(struct net_device *ndev, const u8 *data, unsigned int freq, size_t len);
int obm_change_virt_intf(struct net_device *ndev, u8 curr_iftype, u8 type);
int	onebox_set_cqm_rssi_config(struct wiphy *wiphy, struct net_device *dev,
                                             s32 rssi_thold, u32 rssi_hyst);
