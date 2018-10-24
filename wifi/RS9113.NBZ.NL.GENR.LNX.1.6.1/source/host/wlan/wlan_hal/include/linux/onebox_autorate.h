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

#ifndef __ONEBOX_AUTORATE_H__
#define __ONEBOX_AUTORATE_H__
#include "onebox_vap.h"

#define MAX_RATES (12+8)

/**
 * struct ieee80211_tx_rate - rate selection/status
 *
 * @idx: rate index to attempt to send with
 * @count: number of tries in this rate before going to the next rate
 * @flags: rate control flags (&enum mac80211_rate_control_flags)
 *
 * A value of -1 for @idx indicates an invalid rate and, if used
 * in an array of retry rates, that no more rates should be tried.
 *
 * When used for transmit status reporting, the driver should
 * always report the rate along with the flags it used.
 */

struct ieee80211_tx_rate 
{
	int8 idx;
	uint8 count;
	uint8 flags;
} __attribute__((packed));

struct ieee80211_tx_info 
{
	/* common information */
	uint32 flags;
	/* rate control */
	struct 
	{
		//struct ieee80211_tx_rate rates[IEEE80211_TX_MAX_RATES];
		struct ieee80211_tx_rate rates[5];
		int8  rts_cts_rate_idx;
	}control;
	/* Fill remaining */
};

struct ieee80211_rate_ops 
{
	int ratectl_id;
	struct rsi_ratectrl *(*attach)(WLAN_ADAPTER *sc);
	void (*detach)(WLAN_ADAPTER *arc);
	/* Register proc entries with a VAP */
	void (*dynamic_proc_register)(struct ieee80211vap *vap);

	void (* update_ratestats)(WLAN_ADAPTER w_adapter);
	void (*findrate)(WLAN_ADAPTER w_adapter, 
	                 struct ieee80211_node *ni, 
	                 struct ieee80211_tx_info *txinfo);

	/* Update rate control state on station associate/reassociate 
	 * (when operating as an ap or for nodes discovered when operating
	 * in ibss mode) */
	void (* newassoc)(WLAN_ADAPTER sc, struct ieee80211_node *rn, int isnew);

	/* Update/reset rate control state for 802.11 state transitions.
	 * Important mostly as the analog to newassoc when operating
	 * in station mode */
	void (*newstate)(struct ieee80211vap *vap,
	                 enum ieee80211_state state);
};

struct wl_ratectrl 
{
	struct ieee80211_rate_ops *ops;
	uint32 arc_space;       /* space required for per-node state */
	uint32 arc_vap_space;   /* space required for per-vap state */
};

/* per-device state */
struct minstrel_softc 
{
	struct wl_ratectrl arc;  /* base state */
#ifdef CONFIG_SYSCTL
	struct ctl_table_header *sysctl_header;
	struct ctl_table *sysctls;
#endif
	WLAN_ADAPTER w_adapter;
};

struct minstrel_priv 
{
	//struct ieee80211_hw *hw;
	uint8 has_mrr;
	uint32 cw_min;
	uint32 cw_max;
	uint32 max_retry;
	uint32 ewma_level;
	uint32 segment_size;
	uint32 update_interval;
	uint32 lookaround_rate;
	uint32 lookaround_rate_mrr;
};

struct rate_info 
{
	int32 rate;
	int32 rix;
	int32 rateCode;
};

struct ieee80211_rate 
{ 
	/* Free BSD */
	uint32 flags;
	uint16 bitrate;
	uint16 hw_value, hw_value_short;
};

struct minstrel_rate 
{ 
	/* Free BSD */
	int32 bitrate;
	int32 rix;
	int32 hix; // Here we will store the index expected by hardware(ppe)
	uint32 perfect_tx_time;
	uint32 ack_time;
	int32 sample_limit;
	uint32 retry_count;
	uint32 retry_count_cts;
	uint32 retry_count_rtscts;
	uint32 adjusted_retry_count;
	uint32 success;
	uint32 attempts;
	uint32 last_attempts;
	uint32 last_success;
	/* parts per thousand */
	uint32 cur_prob;
	uint32 probability;
	/* per-rate throughput */
	uint32 cur_tp;
	uint32 throughput;
	uint32 succ_hist;
	uint32 att_hist;
};

/* per-node state */
struct minstrel_node 
{
	uint32 static_rate_ndx; /*User has bypassed dynamic selection. Fix on one rate */
	uint32 current_rate;
	/* Free BSD */
	unsigned long stats_update; /* It will be holding jiffies so shud be long*/
	uint32 sp_ack_dur;
	uint32 rate_avg;
	uint32 lowest_rix;
	uint32 max_tp_rate;
	uint32 max_tp_rate2;
	uint32 max_prob_rate;
	uint32 packet_count;
	uint32 sample_count;
	int32 sample_deferred;
	uint32 sample_idx;
	uint32 sample_column;
	int32 n_rates;
	struct minstrel_rate *r;
	uint8 prev_sample;
	/* sampling table */
	uint8 *sample_table;
};

/* Auto rate stats */
struct autorate_stats_s 
{
	uint16 total_attempts;
	uint16 total_success;
};

#ifndef MIN
#define MIN(a,b)        ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b)        ((a) > (b) ? (a) : (b))
#endif

#define RATES_ALLOCATED                 1
#define DO_NOT_AGGREGATE                1

struct ieee80211_rate_ops *core_rate_attach(WLAN_ADAPTER w_adapter);
struct minstrel_softc * ar_attach(WLAN_ADAPTER w_adapter,struct minstrel_softc *osc);
void core_rate_detach(WLAN_ADAPTER w_adapter);
struct ieee80211_ratectl *onebox_get_ieee80211_ratectl(void);

#endif
