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

#define IEEE80211_AGGR_BAWMAX  16
#define WME_BA_BMP_SIZE   IEEE80211_AGGR_BAWMAX /* max block ack window size */
#define WME_MAX_BA        WME_BA_BMP_SIZE
#define ONEBOX_TID_MAX_BUFS  (2 * WME_MAX_BA)
#define SKB_CB(_skb)     ((struct ieee80211_cb *)(_skb)->cb)
#define ONEBOX_MULTICAST 0x00000400 /* net80211 has already used some skb->flag bits */
#define ONEBOX_BROADCAST 0x00000200 /* net80211 has already used some skb->flag bits */
#define MAX_MGMT_PKT_SIZE 512
#define MAX_HW_QUEUES	 12
#define NUM_SOFT_QUEUES  10 /* Beacon, Broadcast softqueues are maintained separately.They should be defined per vap
				Rest of the queues(4-EDCA STA mode + 4 EDCA AP mode + 1MGMT + 1Reserved) are defined here */

#define MGMT_SOFT_Q        8 /* Queue No 10 is used for MGMT_QUEUE in Device FW */
#define MGMT_HW_Q   10 /* Queue No 10 is used for MGMT_QUEUE in Device FW, Hence this is Reserved */
#define BROADCAST_HW_Q   9 
#define BEACON_HW_Q     11

#define CWMIN            7
#define CWMAX            15
#define AIFSN            3
#define TXOP             0

#define IFF_DRV_RUNNING 0x40        /* (d) resources allocated */
#define IFF_DRV_OACTIVE 0x400       /* (d) tx hardware queue is full */
#define IEEE80211_ADDR_LEN 6
typedef enum 
{
	STA_CONNECTED = 0,
	STA_DISCONNECTED,
	STA_ADDBA_DONE,
	STA_DELBA,
	STA_RX_ADDBA_DONE,
	STA_RX_DELBA
}sta_notify_events_t;

#if 0
typedef struct netbuf_ctrl_block_s 
{
	struct netbuf_ctrl_block_s *next; // next ptr
	uint8_t                    *data;          // Data address
	void                       *head;          // head address
	void                       *dev;
	void                       *pkt_addr;
	void                       *ni;                   //store the node reference in ni 
	uint16_t                    len;            // length of the packet
	uint16_t                    aggr_len;       // Total aggregation length
	uint8_t                     mac_hdr_len;    // MAC header length(80211)
	uint8_t                     hdr_pad_len;    // Header padding length
	uint8_t                     sta_id;         // Station id
	uint8_t                     tid;            // Traffic identifier
	uint16_t                     flags;          // Flags
	uint8_t                     aggr_flag;      // Aggregation flags
	uint8_t                     aggr_pcnt;      // No. of AMPDU aggregated packets
	uint8_t                     agg_retry;      // Is it a retry
	uint8_t                     retry_count;    // No. of times this packet retried
	uint8_t                     priority;      
	uint8_t                     null_delim;
	uint8_t                     bt_pkt_type;
	uint8_t                     vap[6];        // Pointer to vap structure reserved
	/* Removed frag_flags, need to use flags in case required */
}netbuf_ctrl_block_t;
#endif

struct wlan_tid 
{
	netbuf_ctrl_block_t *buf_q_head;
	netbuf_ctrl_block_t *buf_q_tail;
	struct wlan_ac *ac;
	struct wlbuf *tx_buf[ONEBOX_TID_MAX_BUFS]; 
	u16 seq_start;
	u16 seq_next;
	u16 baw_size;
	int tidno;
	int baw_head;   /* first un-acked tx buffer */
	int baw_tail;   /* next unused tx buffer slot */
	int sched;
	int paused;
	u8 state;
	u16 pkt_count;
	int addba_exchangeattempts;
	struct wlan_tid *next;
};

struct wlan_ac 
{
	int sched;
	int qnum;
	struct wlan_tid *tid_q_head;
	struct wlan_tid *tid_q_tail;
	struct wlan_ac *next;
};

struct bgscan_ssid{
	u_int16_t ssid_len;
	u_int8_t ssid[32];
};


#define ps_params_def vap->hal_priv_vap->ps_params
#define ps_params_def_ioctl vap->hal_priv_vap->ps_params_ioctl
#define traffic_ps_params_def vap->hal_priv_vap->traffic_ps_params
#define uapsd_params_def vap->hal_priv_vap->uapsd_params_updated 
//#define BYTES_CONVRTR (125 * 1000)
#define BYTES_CONVRTR (125 * ps_params_def.monitor_interval )
#define driver_ps vap->hal_priv_vap->drv_params
#define PS_STATE vap->hal_priv_vap->drv_params.ps_state
//#define TRAFFIC_PS_EN (ps_params_def.tx_threshold || ps_params_def.rx_threshold || ps_params_def.monitor_interval)
#define TRAFFIC_PS_EN (ps_params_def.monitor_interval)
#define IOCTL_TRAFFIC_PS_EN (ps_params_def_ioctl.monitor_interval)
/* POWER SAVE STATES */
#define PS_NONE 0
#define PS_EN_REQ_SENT 1
#define DEV_IN_PWR_SVE 2
#define PS_DIS_REQ_SENT 3
#define PS_EN_REQ_QUEUED 4

#define SLEEP_REQUEST 1
#define WAKEUP_REQUEST 2

struct uapsd_params
{
	u_int8_t mimic_support;
	u_int8_t uapsd_acs;
	u_int8_t uapsd_wakeup_period;
};

struct pwr_save_params {
	u_int8_t ps_en;
	u_int8_t sleep_type; //LP or ULP type
	u_int8_t tx_threshold;
	u_int8_t rx_threshold;
	u_int8_t tx_hysterisis;
	u_int8_t rx_hysterisis;
	u_int16_t monitor_interval;
	u_int32_t listen_interval;
	u_int16_t num_beacons_per_listen_interval;
	u_int32_t dtim_interval_duration;
	u_int16_t num_dtims_per_sleep;
	u_int32_t deep_sleep_wakeup_period;
#ifdef DUTY_CYCLE_IOCTL
        u_int8_t duty_cycle;
#endif
};


struct wowlan_config
{
  u_int8_t macaddr[6];
  u_int16_t flags;
  u_int16_t host_wakeup_state;
};

#define MAX_NUM_SCAN_BGCHANS 24
struct hal_priv_ieee80211vap
{
	u_int8_t short_gi;
	u_int8_t rifs;
	u_int8_t green_field;
	u_int8_t ap_isol;
	u_int8_t onebox_tx_pow;
	u_int8_t fixed_rate_enable;
	u_int8_t update_wmmparams;
	u_int8_t stop_udp_pkts[NUM_SOFT_QUEUES];
#ifdef CONFIG_11W
	u_int8_t rsn_caps;
	u_int8_t mfp_enabled;
	u_int8_t assoc_retry;
#endif
	u_int8_t stop_per_q[NUM_SOFT_QUEUES];
	int rate_hix;
	u_int16_t beacon_loc;
	//pwr_save_params_nl	ps_params;
	struct core_vap *core_vp;
	void  *hal_priv_ptr;
	u_int8_t preamble;
	struct bgscan_ssid bg_ssid;
	struct ieee80211_node	*iv_bss_roam;	/* information for this node */
	u_int8_t conn_in_prog;
	u_int8_t roam_ind;
	u_int8_t roam_ioctl;
	u_int8_t eapol_4;
	u_int8_t probe_req_bcast;
	u_int8_t passive;
	u_int8_t assoc_req;
	u_int8_t reassoc_ind;
	u_int8_t reauth_state;
	u_int8_t sta_data_block;
	u_int8_t vap_id;
	u_int8_t rssi;
	u_int8_t extended_desc_size;
	u_int8_t cip_type;
	struct
	{
		u_int16_t bgscan_threshold;
		u_int16_t roam_threshold;
		u_int16_t bgscan_periodicity;
		u_int8_t num_bg_channels;
		u_int8_t two_probe;
		u_int16_t active_scan_duration;
		u_int16_t passive_scan_duration;
		u_int16_t channels2scan[MAX_NUM_SCAN_BGCHANS];
		u_int8_t bg_ioctl;
		u_int16_t bg_cmd_flags;
	}bgscan_params_ioctl;
 	struct pwr_save_params ps_params_ioctl;//Parameters to store IOCTL parameters from USER
 	struct pwr_save_params ps_params;// Driver current running PWR_SVE_Parameters
#ifdef IEEE80211K 	
	struct msrmnt_req_params 					//This structure is used to copy the given params through onebox_util ioctl(user space) to kernel space 
	{
		uint8_t frame_type,ssid_len;
		unsigned char ucast_macaddr[IEEE80211_ADDR_LEN];

		union
		{
		  unsigned char frame_req_macaddr[IEEE80211_ADDR_LEN];
		  unsigned char bssid_macaddr[IEEE80211_ADDR_LEN];
		};
		
		uint8_t ssid_beacon_rpt[32];	
	}msrmnt_req; 
  u_int8_t params_11k_set;
#endif

	struct {
		u_int8_t ps_en;
		u_int32_t tx_dataload;
		u_int32_t rx_dataload;
		unsigned long jiffies_prev;

	}traffic_ps_params;
	struct uapsd_params uapsd_params_ioctl;
	struct uapsd_params uapsd_params_updated;
	uint8_t deep_sleep_en;
	uint8_t join;
	uint8_t key_idx;
	struct
	{
		u_int8_t update_ta;
		u_int16_t ps_state;
		u_int8_t delay_pwr_sve_decision_flag;
		u_int8_t disable_ps_for_change_in_params;
	}drv_params;
	uint8_t aggr_rx_limit;
	uint8_t rtp_prints;
	uint8_t stop_tx_q;
	uint8_t dfs_passive_to_active;
  struct wowlan_config wowlan_params;
#if defined(RSI_CCX) || defined(CONFIG_11R)
	uint8_t IAPP_IND_FLAG;
#endif
	uint8_t mgmt_rate_enable;
	uint16_t default_mgmt_rate;
#ifdef CONFIG_ACS
	u_int8_t acs_enable;
#endif
};

#define WME_NUM_AC 4    /* 4 AC categories */
#define WME_NUM_TID 8  /* 8 tids */
#ifdef CONFIG_11W
#define IEEE80211_SA_INIT_DONE 0x01

struct sa_query_info {
        int     dialogtoken;
        u_int8_t initiate_sa;
        u_int8_t retries;
        u_int8_t recvd_response;
        u_int8_t sa_flags;
};
#endif
struct ieee80211_uaspd {
	uint8_t eosp_triggered;
	uint8_t uapsd_flags;
	uint8_t trigger_frame_recvd;
	uint8_t max_sp_length;
	uint32_t eosp_tsf;
	
};
struct hal_priv_ieee80211_node
{
	u_int8_t sta_id;
	struct wlan_tid tid[WME_NUM_TID];
	u_int16_t tidnum;
	struct wlan_ac ac[WME_NUM_AC];
	u_int16_t sta_capability;
	struct minstrel_node *ni_mn;    /* pointer to minstrel node */
	u_int8_t chno;
	uint8_t delba_ind;
	uint8_t *probereq;
	uint8_t node_cnt;
#ifdef CONFIG_11W
	uint8_t conn_in_prog;
        uint8_t mfp_enabled;
        struct sa_query_info sa_info;
#endif
};
