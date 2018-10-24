
#include <net80211/ieee80211_var.h>

#define IEEE80211_P2P_FOREVER 0x7fffffff
struct ieee80211vap;

struct ieee80211_p2p_noa {
	uint8_t count;
  uint16_t reserved;
	uint8_t local_count;
	uint32_t noa_duration;
	uint32_t noa_interval;
	uint32_t noa_start_time;
};

struct ieee80211_p2p
{
	struct ieee80211vap *p2p_vap;
	struct ieee80211_p2p_noa *noa;
	uint16_t probe_req_report;
	uint8_t  p2p_wildcard_essid[IEEE80211_NWID_LEN];
	uint8_t  p2p_wildcard_essid_len;
	uint16_t remain_on_channel_freq;
	uint16_t remain_on_channel_duration;
	struct timer_list remain_on_channel_timer;
	struct timer_list cancel_remain_on_channel_timer;
	struct timer_list noa_duration_timer;
	struct timer_list noa_interval_timer;

	struct scan_channel_set_s
	{
		uint16_t num_freqs;
		uint16_t freqs[IEEE80211_MAX_FREQS_ALLOWED];
	}scan_channels;

	void (*p2p_set_probe_req_report) (struct ieee80211vap *vap, uint16_t probe_req_report);
#ifndef ONEBOX_CONFIG_CFG80211
	int  (*p2p_send_action) (struct ieee80211vap *vap, uint8_t *dst, uint8_t *src, uint8_t *bssid, uint8_t *data, uint16_t data_len, unsigned int freq);
#else
	int  (*p2p_send_action) (struct ieee80211vap *vap, unsigned int freq);
#endif
	void (*p2p_remain_on_channel) (struct ieee80211vap *vap, uint16_t duration, unsigned int freq);
	void (*p2p_cancel_remain_on_channel) (struct ieee80211vap *vap);
	int (*p2p_change_vap_mode) (struct ieee80211vap *vap, uint32_t mode);
	void (*p2p_recv_probe_req)(struct ieee80211_node *ni, struct mbuf *m);
	void (*p2p_recv_action)(struct ieee80211_node *ni, struct mbuf *m);
	void (*p2p_hal_mode_change_from_sta_to_hostap)(struct ieee80211vap *vap);
	void (*p2p_deinit)(struct ieee80211vap *vap);
#ifdef ONEBOX_CONFIG_CFG80211
	u64 cookie;
#endif //ONEBOX_CONFIG_CFG80211
};

struct ieee80211_p2p_action
{
	uint8_t category;
	union
	{
		struct
		{
			uint8_t action_field; //0x09
			uint8_t oui[3]; //0x506F9A
			uint8_t oui_type;
		} p2p_public_action_header;
		struct
		{
			uint8_t oui[3]; // 0x506F9A
			uint8_t oui_type; //0x09
		} p2p_action_header;
	}u;
}__packed;

#define IEEE80211_ACTION_CAT_PUBLIC 0x04

/* Public action codes */
#define IEEE80211_PA_VENDOR_SPECIFIC   0x09
#define IEEE80211_PA_GAS_INITIAL_REQ   0x0A
#define IEEE80211_PA_GAS_INITIAL_RESP  0x0B
#define IEEE80211_PA_GAS_COMEBACK_REQ  0x0C
#define IEEE80211_PA_GAS_COMEBACK_RESP 0x0D

#define WFA_OUI 0x9A6F50
#define P2P_OUI_TYPE 0x09

struct ieee80211_p2p * p2p_init(struct ieee80211vap *vap);
void send_remain_on_channel_event(struct ieee80211vap *vap, uint16_t freq, uint16_t duration);
void send_cancel_remain_on_channel_event(struct ieee80211vap *vap, uint16_t freq);
void notify_recv_mgmt_pkt(struct ieee80211vap *vap, uint8_t *buf, uint16_t len, uint16_t recv_freq);
#ifdef ENABLE_P2P_SUPPORT
/* This declaration should not be here */
__noinline int ieee80211_ioctl_setchannel(struct ieee80211vap *vap,
	const struct ieee80211req *ireq);
#endif
