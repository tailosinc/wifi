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
 *
 * $FreeBSD: src/sys/net80211/ieee80211_11k.h,v 1.18 2011/03/10 18:17:24 bschmidt Exp $
 */
#ifndef _NET80211_IEEE80211_11K
#define _NET80211_IEEE80211_11K

/*
 * 802.11k protocol implementation definitions.
 */

#define MAX_MEASUREMENT_REQ		1
#define ONEBOX_STATUS_SUCCESS 0
#define ONEBOX_STATUS_FAILURE -1

void	ieee80211_rm_attach(struct ieee80211com *);
void	ieee80211_rm_detach(struct ieee80211com *);
int ieee80211_11k_action(struct ieee80211vap *, void *data);
int ieee80211_11k_sm_action(struct ieee80211vap *, void *data);

struct beacon_sub_ele {
	uint8_t ssid_ie[32];
	uint8_t bcn_rpt_info[2];
	uint8_t rpt_detail;
	uint8_t ssid_ie_len;
	uint8_t beacon_meas_mode;
	
};
struct rm_element {
	uint8_t element_id;
	uint8_t length;
	uint8_t token;
	uint8_t mode;
	uint8_t type;
}__packed;
struct sm_meas_req{
	uint8_t chan_num;
	uint64_t msrmnt_start_time;
	uint16_t msrmnt_dur;

}__packed;
struct sm_cca_rept{
 struct sm_meas_req rpt;
	uint8_t cca_busy_fraction;
}__packed;
struct sm_rpi_hist_rept{
	struct sm_meas_req rpt;
	uint8_t rpi_vals[8];
}__packed;


struct radio_meas_req {
	uint8_t operating_class;
	uint8_t channel_num;
	uint16_t rand_int;
	uint16_t meas_duration;
	
	union
	{
			uint8_t frame_req_type;
			uint8_t meas_mode;
	};
	union
	{
			uint8_t bssid[6];
			uint8_t mac_addr[6];
	};
	union 
	{
			struct beacon_sub_ele bse;
	};

	uint8_t unicast_mac_addr[ETHER_ADDR_LEN];
}__packed;

struct se_ch_load {
};
struct noise_hist_rpt {
	uint8_t operating_class;
	uint8_t channel_num;
	uint32_t tsf_time[2];
	uint16_t meas_duration;
	uint8_t antenna_id;
	uint8_t anpi;
	uint8_t ipi_density[11];
}__packed;

struct channel_load_rpt {
	uint8_t operating_class;
	uint8_t channel_num;
	uint32_t tsf_time[2];
	uint16_t meas_duration;
	uint8_t channel_load;
}__packed;

struct multicast_diagnostics_report {
	uint32_t measurement_time[2];
	uint16_t meas_duration;
	uint8_t group_mac[6];
	uint8_t mcast_report_reason;
	uint16_t mcast_pkt_count;
	uint8_t first_seq_num;
	uint8_t last_seq_num;
	uint8_t mcast_high_rate;
}__packed;
struct frame_report {
	uint8_t operating_class;
	uint8_t channel_num;
	uint32_t tsf_time[2];
	uint16_t meas_duration;
	/* optional subelements, n*19*/
	uint8_t elem_id;
	uint8_t length;
	uint8_t tx_addr[6];
	uint8_t bssid[6];
	uint8_t phy_type;
	uint8_t avg_rcpi;
	uint8_t last_rsni;
	uint8_t last_rcpi;
	uint8_t ant_id;
	uint16_t frame_count;
}__packed;

struct beacon_report {
	uint8_t operating_class;
	uint8_t channel_num;
	uint32_t tsf_time[2];
	uint16_t meas_duration;
	uint8_t rpt_frame_info;
	uint8_t rcpi;
	uint8_t rsni;
	uint8_t bssid[6];
	uint8_t ant_id;
	uint32_t parent_tsf;
};

struct radio_meas_rpt {
	uint8_t dialogue_token;
	uint8_t type;
	uint8_t received_rpt;
}__packed;

struct radio_meas_info {
	uint8_t dialogue_token;
	uint16_t repetitions;
	struct rm_element meas_req;
	struct radio_meas_req req;
	uint8_t measurement_type;
	struct ieee80211_node *ni;
}__packed;

struct sm_meas_info {
	uint8_t dialogue_token;
	struct rm_element meas_req;
	struct sm_meas_req req;
	uint8_t measurement_type;
	struct ieee80211_node *ni;
}__packed;

/* 11k Actions */
#define IEEE80211_ACTION_RADIO_MEAS_REQ		0
#define IEEE80211_ACTION_RADIO_MEAS_RPT		1
#define IEEE80211_ACTION_LINK_MEAS_REQ		2
#define IEEE80211_ACTION_LINK_MEAS_REP		3
#define	IEEE80211_ACTION_NG_RPT_REQ 		4
#define	IEEE80211_ACTION_NG_RPT_RSP 		5
/* Radio Measurement types */
#define IEEE80211_CHANNEL_LOAD	3
#define IEEE80211_NOISE_HIST	4
#define IEEE80211_BEACON		5
#define IEEE80211_FRAME			6
#define IEEE80211_STA_STAS		7
#define IEEE80211_LCI			8
#define IEEE80211_TRANS_STR_CAT 9
#define IEEE80211_MULTICAST_DIG_REQ  10
#define IEEE80211_MEAS_PAUSE	255

///*11K SM Actions */
#define IEEE80211_ACTION_SM_MEAS_REQ		0
#define IEEE80211_ACTION_SM_MEAS_RPT		1
/* SM Measurement Types */
#define IEEE80211_BASIC_MEAS			0
#define IEEE80211_CCA_MEAS				1
#define IEEE80211_RPI_HIST_MEAS		2


/* IEEE80211K Capabilities */
#define IEEE80211K_LINK_MEAS					BIT(0)
#define IEEE80211K_NEIGH_REPORT					BIT(1)
#define IEEE80211K_BEAC_PAS_MEAS				BIT(4)
#define IEEE80211K_BEAC_ACT_MEAS				BIT(5)
#define IEEE80211K_BEAC_TBL_MEAS				BIT(6)


#define IEEE80211K_FRAME_MEAS					BIT(0)
#define IEEE80211K_CHNL_LOAD_MEAS				BIT(1)
#define IEEE80211K_NOISE_HIST_MEAS				BIT(2)
#define IEEE80211K_STA_MEAS						BIT(3)

#define SINGLE_CHANNEL_LOAD_LEN				9
#define SINGLE_FRAME_REQ_LEN					16
#define SINGLE_BEACON_REQ_LEN					16
#define SINGLE_MULTICAST_DIAGNOSTICS_REQ_LEN		9	
/* Optional Elements for Beacon request */
#define SSID_ELEM_ID				0
#define SSID_LEN				32
#define BEACON_REPORT_INFO          1
#define REPORTING_DETAIL			2
#define AP_CHANNEL_RPT				51
#define VENDOR_SPECIFIC				221

void rm_req( struct ieee80211_node *ni, uint16_t repetitions, const uint8_t *frm);
int ieee80211_send_meas_rpt( struct ieee80211vap *vap, uint8_t *msg, uint8_t msg_len);
int send_noise_hist_req(struct ieee80211_node *ni, int category, int action, uint8_t* args); 
int send_channel_load_req(struct ieee80211_node *ni, int category, int action, uint8_t* args); 
int send_frame_req(struct ieee80211_node *ni, int category, int action, uint8_t* args);
int send_multicat_dig_req(struct ieee80211_node *ni, int category, int action, uint8_t *args);

int send_beacon_req(struct ieee80211_node *ni, int category, int action, uint8_t* args);
int send_sm_meas_req(struct ieee80211_node *ni, int category, int action, uint8_t *args);
 
uint8_t *ieee80211_add_rm_capab(uint8_t *);


#endif /* _NET80211_IEEE80211_11K */



