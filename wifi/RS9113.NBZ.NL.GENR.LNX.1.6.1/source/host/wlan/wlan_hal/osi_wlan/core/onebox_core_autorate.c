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
#include "ieee80211_ratectl.h"
//#include "net80211/ieee80211_rate.h"

#define ONE_SECOND (1000 * 1000)  /* 1 second, or 1000 milliseconds, eternity in other words */


static int minstrel_get_next_sample(struct minstrel_node *mi);
static void ar_rate_ctl_reset(WLAN_ADAPTER w_adapter, struct ieee80211_node *ni);

static struct minstrel_priv *ar_mp = NULL;

#define SAMPLE_COLUMNS  10
#define SAMPLE_TBL(_mi, _idx, _col) \
                   _mi->sample_table[(_idx * SAMPLE_COLUMNS) + _col]

/**
 * This function allocates the memory for the minstrel node and the
 * sample table to hold the rates & indexes corresponding to the rates.
 *
 * @param       Pointer to the driver private structure.
 * @param       Pointer to the node structure.
 * @return      none.
 */
static void *minstrel_alloc_sta(WLAN_ADAPTER w_adapter,struct ieee80211_node *ni)
{
	struct minstrel_node *mi;
	int max_rates = MAX_RATES; /* : Take from ni */

	/* Create minstrel node */
	mi = w_adapter->os_intf_ops->onebox_mem_zalloc(sizeof(struct minstrel_node), 0);
	if (!mi)
	{
		return NULL;
	}
	w_adapter->os_intf_ops->onebox_memset(mi, 0, sizeof(struct minstrel_node));

	/* Create Sample Table */
	mi->r = w_adapter->os_intf_ops->onebox_mem_zalloc(sizeof(struct minstrel_rate) * max_rates,0);
	if (!mi->r)
	{
		goto error;
	}
	w_adapter->os_intf_ops->onebox_memset(mi->r, 0, sizeof(struct minstrel_rate));

	mi->sample_table = w_adapter->os_intf_ops->onebox_mem_zalloc(SAMPLE_COLUMNS * max_rates, 0);
	if (!mi->sample_table)
	{
		goto error1;
	}

	mi->stats_update = w_adapter->os_intf_ops->onebox_get_jiffies();
	return mi;

error1:
	w_adapter->os_intf_ops->onebox_mem_free(mi->r);
error:
	w_adapter->os_intf_ops->onebox_mem_free(mi);
	return NULL;
}

/**
 * This function calls a wrapper function to initialize the autorate rate node.
 *
 * @param       Pointer to the driver private structure.
 * @param       Pointer to the core node structure.
 * @return      none.
 */
static void minstrel_node_init(struct ieee80211_node *node)
{
	WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)node->ni_vap->hal_priv_vap->hal_priv_ptr;
	struct minstrel_node *mn = NULL;

	mn = minstrel_alloc_sta(w_adapter, node);
	node->hal_priv_node.ni_mn = mn;
}

/**
 * This function frees the autorate node & associated sample table and rates.
 *
 * @param       Pointer to the driver private structure.
 * @param       Pointer to the core node structure.
 * @return      none.
 */
static void minstrel_free_sta(WLAN_ADAPTER w_adapter,struct ieee80211_node *sta)
{
	struct minstrel_node *mi = sta->hal_priv_node.ni_mn;
	if(!mi)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("NULL FIX ME ni_mn\n")));
		return ;
	}

	if (mi->sample_table != NULL)
	{
		w_adapter->os_intf_ops->onebox_mem_free(mi->sample_table);
	}
	if (mi->r != NULL)
	{
		w_adapter->os_intf_ops->onebox_mem_free(mi->r);
	}
	if (mi != NULL)
	{
		w_adapter->os_intf_ops->onebox_mem_free(mi);
	}
}

/**
 * This function calls a wrapper function to free the autorate node.
 *
 * @param       Pointer to the driver private structure.
 * @param       Pointer to the node structure.
 * @return      none.
 */
static void minstrel_node_deinit(struct ieee80211_node *ni)
{
	WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)ni->ni_vap->hal_priv_vap->hal_priv_ptr;
	minstrel_free_sta(w_adapter, ni);
}

/**
 * This function updates the statistics for calculating the best rate
 * next best rate and probabilities.Using the above statistics retry chain
 * information is formed.
 *
 * @param       Pointer to the data.
 * @param       Pointer to the node structure.
 * @return      none.
 */
static void minstrel_update_stats(void *arg, struct ieee80211_node *ni)
{
	struct minstrel_priv *mp = ar_mp;
	WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)arg;
	struct autorate_stats_s *sta_ratestats = NULL;
	struct minstrel_node *sn = NULL;
	uint32 max_tp = 0, index_max_tp = 0, index_max_tp2 = 0;
	uint32 max_prob = 0;
	uint32 index_max_prob = 0;
	uint32 usecs;
	uint32 p;
	unsigned int i;

	if (((ni->ni_vap->iv_opmode == IEEE80211_M_HOSTAP) || (ni->ni_vap->iv_opmode == IEEE80211_M_STA)) && (!ni->ni_associd))
	{
		return;
	}

	if (!w_adapter->os_intf_ops->onebox_memcmp(ni->ni_vap->iv_myaddr,
	                                         ni->ni_macaddr,
	                                         ETHER_ADDR_LEN))
	{
		return;
	}

	sn = ni->hal_priv_node.ni_mn;
	sta_ratestats = w_adapter->autorate_stats[ni->hal_priv_node.sta_id];
	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("CORE_MSG: ni = %p, mac_addr = "MAC_FMT" sta_id = %d\n"),
	                                    ni, ni->ni_macaddr[0], ni->ni_macaddr[1], ni->ni_macaddr[2], ni->ni_macaddr[3],
	                                    ni->ni_macaddr[4], ni->ni_macaddr[5], ni->hal_priv_node.sta_id));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_AUTORATE, (uint8 *)sta_ratestats, 80);

	sn->stats_update = w_adapter->os_intf_ops->onebox_get_jiffies();

	for (i = 0; i < sn->n_rates; i++) 
	{
		struct minstrel_rate *mr = &sn->r[i];

		usecs = mr->perfect_tx_time;
		if (!usecs)
		usecs = ONEBOX_CPU_TO_LE32(1000000);
		mr->attempts = ONEBOX_CPU_TO_LE16(sta_ratestats[mr->hix].total_attempts);
		mr->success  = ONEBOX_CPU_TO_LE16(sta_ratestats[mr->hix].total_success);
		sta_ratestats[mr->hix].total_attempts = 0;
		sta_ratestats[mr->hix].total_success = 0;
		ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("CORE_MSG: %d Mbps: mr->attempts[%d] = %d,mr->success[%d] = %d\n"),
		                                  mr->bitrate, i, mr->attempts, i, mr->success));

		/* To avoid rounding issues, probabilities scale from 0 (0%)
		 * to 18000 (100%) */
		if (mr->attempts) 
		{
			p = (mr->success * 18000) / mr->attempts;
			mr->succ_hist += mr->success;
			mr->att_hist += mr->attempts;
			mr->cur_prob = p;
			p = ((p * (100 - mp->ewma_level)) + 
			     (mr->probability * mp->ewma_level)) / 100;
			mr->probability = p;
			mr->cur_tp = p * (1000000 / usecs);
			ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("CORE_MSG:mr->cur_tp= %d, mr->probability = %d\n"),mr->cur_tp, mr->probability));
		}
		mr->last_success = mr->success;
		mr->last_attempts = mr->attempts;
		mr->success = 0;
		mr->attempts = 0;

		/* Sample less often below the 10% chance of success.
		 * Sample less often above the 95% chance of success. */
		if ((mr->probability > 17100) || (mr->probability < 1800)) 
		{
			mr->adjusted_retry_count = mr->retry_count >> 1;
			if (mr->adjusted_retry_count > 2)
			{
				mr->adjusted_retry_count = 2;
			}
			mr->sample_limit = 4;
		} 
		else 
		{
			mr->sample_limit = -1;
			mr->adjusted_retry_count = mr->retry_count;
		}
		if (!mr->adjusted_retry_count)
		mr->adjusted_retry_count = 2;
	}

	for (i = 0; i < sn->n_rates; i++) 
	{
		struct minstrel_rate *mr = &sn->r[i];

		if (max_tp < mr->cur_tp) 
		{
			index_max_tp = i;
			max_tp = mr->cur_tp;
		}
		if (max_prob < mr->probability) 
		{
			index_max_prob = i;
			max_prob = mr->probability;
		}
	}
	max_tp = 0;
	for (i = 0; i < sn->n_rates; i++) 
	{
		struct minstrel_rate *mr = &sn->r[i];

		if (i == index_max_tp)
		continue;
		if (max_tp < mr->cur_tp) 
		{
			index_max_tp2 = i;
			max_tp = mr->cur_tp;
		}
	}
	sn->max_tp_rate = index_max_tp;
	sn->max_tp_rate2 = index_max_tp2;
	sn->max_prob_rate = index_max_prob;
	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("CORE_MSG: max_tp_rate= %d,max_tp_rate2= %d, max_prob_rate=%d\n\n"),
	                                   sn->max_tp_rate, sn->max_tp_rate2,sn->max_prob_rate));
}

/**
 * This function gives the retry count.
 *
 * @param       Pointer to the ministrel rate structure.
 * @param       Pointer to transmit information structure.
 * @return      retry count.
 */
static inline unsigned int
minstrel_get_retry_count(struct minstrel_rate *mr,
                         struct ieee80211_tx_info *info)
{
	uint32 retry = mr->adjusted_retry_count;
	//retry = max(2U, min(mr->retry_count_rtscts, retry)); 
	return retry;
}

/**
 * This function picks the data rate.
 *
 * @param       Pointer to the driver private data.
 * @param       Pointer to the node structure.
 * @param       Pointer to the transmit information structure.
 * @return      none.
 */
static void minstrel_get_rate(WLAN_ADAPTER w_adapter, struct ieee80211_node *ni, 
                              struct ieee80211_tx_info *txinfo)
{
	struct minstrel_node *sn = ni->hal_priv_node.ni_mn;
	struct minstrel_priv *mp = ar_mp;
	struct ieee80211_tx_rate *ar = txinfo->control.rates;
	uint32 ndx, sample_ndx = 0;
	uint8 mrr;
	uint8 sample_slower = FALSE;
	uint8 sample = FALSE;
	int32 i, delta;
	int32 mrr_ndx[3];
	int32 sample_rate;
	uint32 sample_index;

	txinfo->flags = RATES_ALLOCATED;
	mrr = 1;
	ndx = sn->max_tp_rate;

	if (mrr)
	{
		sample_rate = mp->lookaround_rate_mrr;
	}
	else
	{
		sample_rate = mp->lookaround_rate;
	}
	sn->packet_count++;
	delta = (sn->packet_count * sample_rate / 100) - (sn->sample_count + sn->sample_deferred / 2);
	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("delta = %d, sn->packet_count = %d, sample_deferred = %d ,sn->sample_count= %d\n"),
	                                    delta, sn->packet_count, sn->sample_deferred,sn->sample_count));

	/* delta > 0: sampling required */
	if ((delta > 0) && (mrr || !sn->prev_sample)) 
	{
		struct minstrel_rate *msr;

		if (sn->packet_count >= 10000) 
		{
			sn->sample_deferred = 0;
			sn->sample_count = 0;
			sn->packet_count = 0;
		} 
		else if ((uint32)delta > (uint32)(sn->n_rates * 2)) 
		{
			/* With multi-rate retry, not every planned sample
			 * attempt actually gets used, due to the way the retry
			 * chain is set up - [max_tp,sample,prob,lowest] for
			 * sample_rate < max_tp.
			 *
			 * If there's too much sampling backlog and the link
			 * starts getting worse, minstrel would start bursting
			 * out lots of sampling frames, which would result
			 * in a large throughput loss. */
			sn->sample_count += (delta - sn->n_rates * 2);
		}
		sample_ndx = minstrel_get_next_sample(sn);
		msr = &sn->r[sample_ndx];
		sample = ONEBOX_TRUE;
		sample_slower = mrr && (msr->perfect_tx_time >= sn->r[ndx].perfect_tx_time);

		if (!sample_slower) 
		{
			if (msr->sample_limit != 0) 
			{
				ndx = sample_ndx;
				sn->sample_count++;
				if (msr->sample_limit > 0)
				{
					msr->sample_limit--;
				}
			} 
			else 
			{
				sample = FALSE;
			}
		} 
		else 
		{
			/* Only use IEEE80211_TX_CTL_RATE_CTRL_PROBE to mark
			 * packets that have the sampling rate deferred to the
			 * second MRR stage. Increase the sample counter only
			 * if the deferred sample rate was actually used.
			 * Use the sample_deferred counter to make sure that
			 * the sampling is not done in large bursts */
			txinfo->flags |= 0;
			sn->sample_deferred++;
		}
	}
	sn->prev_sample = sample;

	/* If we're not using MRR and the sampling rate already
	 * has a probability of >95%, we shouldn't be attempting
	 * to use it, as this only wastes precious airtime */
	if (!mrr && sample && (sn->r[ndx].probability > 17100))
	{
		ndx = sn->max_tp_rate;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,
	             (TEXT("sn->max_tp_rate = %d, sn->r[ndx].probability=%d\n"),sn->max_tp_rate, sn->r[ndx].probability));

	ar[0].idx = sn->r[ndx].hix;
	ar[0].count = minstrel_get_retry_count(&sn->r[ndx], txinfo);
	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("ndx = %d, ar[0].idx = %d, ar[0].count = %d\n"),
                                             ndx, ar[0].idx, ar[0].count));

	if (!mrr) 
	{
		if (!sample)
		ar[0].count = mp->max_retry;
		ar[1].idx = sn->lowest_rix;
		ar[1].count = mp->max_retry;
		return;
	}

	/* MRR setup */
	sample_index = 0;
	if (sample) 
	{
		if (sample_slower) 
		{
			mrr_ndx[0] = sn->r[sample_ndx].hix;
			sample_index = 1;
		} 
		else
		{
			mrr_ndx[0] = sn->r[sn->max_tp_rate].hix;
		}
	} 
	else 
	{
		mrr_ndx[0] = sn->r[sn->max_tp_rate2].hix;
	}
	mrr_ndx[1] = sn->r[sn->max_prob_rate].hix;

	mrr_ndx[2] = sn->r[0].hix;

	if (ar[0].idx < 12) 
	{
		ar[0].flags = DO_NOT_AGGREGATE;
	}
	for (i = 1; i < 4; i++) 
	{
		ar[i].idx = mrr_ndx[i - 1];
		ar[i].count = sn->r[mrr_ndx[i - 1]].adjusted_retry_count;
		ar[i].flags = ((ar[0].flags & DO_NOT_AGGREGATE) || (ar[i].idx < 12)) ? DO_NOT_AGGREGATE : 0;
	}
	if (sample) 
	{
		ar[sample_index].count = 1;
	}
}

static void ar_newassoc(WLAN_ADAPTER sc, struct ieee80211_node *rn, int isnew)
{
	if (isnew)
	{
		ar_rate_ctl_reset(sc, rn);
	}
}

/**
 * This function initializes the sample table for a particular node.
 *
 * @param       Pointer to the driver private structure.
 * @param       Pointer to the ministrel node structe.
 * @param       Pointer to the node structure.
 * return       none.
 */
static void init_sample_table(WLAN_ADAPTER w_adapter,struct minstrel_node *mi,
                              struct ieee80211_node *ni)
{
	uint32 i, col, new_idx;
	uint32 n_srates = mi->n_rates - 1;
	uint8 rnd[8];

	mi->sample_column = 0;
	mi->sample_idx = 0;
	w_adapter->os_intf_ops->onebox_memset(mi->sample_table, 0, 
	                                    SAMPLE_COLUMNS * mi->n_rates);

	for (col = 0; col < SAMPLE_COLUMNS; col++) 
	{
		for (i = 0; i < n_srates; i++) 
		{
			w_adapter->os_intf_ops->onebox_get_random_bytes(rnd, sizeof(rnd));
			new_idx = (i + rnd[i & 7]) % n_srates;

			while (SAMPLE_TBL(mi, new_idx, col) != 0)
				new_idx = (new_idx + 1) % n_srates;

			/* Don't sample the slowest rate (i.e. slowest base
			 * rate). We must presume that the slowest rate works
			 * fine, or else other management frames will also be
			 * failing and the link will break */
			SAMPLE_TBL(mi, new_idx, col) = i + 1;
		}
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("\nCORE_MSG:random table")));
	for (col = 0; col < SAMPLE_COLUMNS; col++) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("\n")));
		for (i = 0; i < n_srates; i++) 
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT(" %d"),SAMPLE_TBL(mi, i, col)));
		}
	}
}

/**
 * This function returns the next sample rate
 *
 * @param
 *    ni        pointer to ministrel node structure
 * return
 *   next sample rate
 */
static int minstrel_get_next_sample(struct minstrel_node *mi)
{
	unsigned int sample_ndx;

	sample_ndx = SAMPLE_TBL(mi, mi->sample_idx, mi->sample_column);
	if(sample_ndx > 20)
	{
		//uint32 i, col;
		//uint32 n_srates = mi->n_rates - 1;
		//	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("\nCORE_MSG:random table")));
		//	for (col = 0; col < SAMPLE_COLUMNS; col++) {
		//	  ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("\n")));
		//	  for (i = 0; i < n_srates; i++) {
		//	    ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT(" %d"),SAMPLE_TBL(mi, i, col)));
		//	  }
		//	}
	}
	mi->sample_idx++;
	if (mi->sample_idx > (mi->n_rates - 2)) 
	{
		mi->sample_idx = 0;
		mi->sample_column++;
		if (mi->sample_column >= SAMPLE_COLUMNS)
		mi->sample_column = 0;
	}
	return sample_ndx;
}

static int ieee80211_frame_duration(uint32 len, int32 rate, int32 erp, int32 short_preamble)
{
	int32 dur;

	/* calculate duration (in microseconds, rounded up to next higher
	 * integer if it includes a fractional microsecond) to send frame of
	 * len bytes (does not include FCS) at the given rate. Duration will
	 * also include SIFS.
	 *
	 * rate is in 100 kbps, so divident is multiplied by 10 in the
	 * DIV_ROUND_UP() operations.
	 */
	if(!rate)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE, (TEXT("%s %d BLUNDER rate = %d\n"), __func__, __LINE__, rate));
		return 10;
	}
	if (0 || erp) 
	{
		/*
		 * OFDM:
		 *
		 * N_DBPS = DATARATE x 4
		 * N_SYM = Ceiling((16+8xLENGTH+6) / N_DBPS)
		 *      (16 = SIGNAL time, 6 = tail bits)
		 * TXTIME = T_PREAMBLE + T_SIGNAL + T_SYM x N_SYM + Signal Ext
		 *
		 * T_SYM = 4 usec
		 * 802.11a - 17.5.2: aSIFSTime = 16 usec
		 * 802.11g - 19.8.4: aSIFSTime = 10 usec +
		 *      signal ext = 6 usec
		 */
		dur = 16; /* SIFS + signal ext */
		dur += 16; /* 17.3.2.3: T_PREAMBLE = 16 usec */
		dur += 4; /* 17.3.2.3: T_SIGNAL = 4 usec */
		dur += 4 * DIV_ROUND_UP((16 + 8 * (len + 4) + 6) * 10, 4 * rate); /* T_SYM x N_SYM */
	} 
	else 
	{
		/*
		 * 802.11b or 802.11g with 802.11b compatibility:
		 * 18.3.4: TXTIME = PreambleLength + PLCPHeaderTime +
		 * Ceiling(((LENGTH+PBCC)x8)/DATARATE). PBCC=0.
		 *
		 * 802.11 (DS): 15.3.3, 802.11b: 18.3.4
		 * aSIFSTime = 10 usec
		 * aPreambleLength = 144 usec or 72 usec with short preamble
		 * aPLCPHeaderLength = 48 usec or 24 usec with short preamble
		 */
		dur = 10; /* aSIFSTime = 10 usec */ /* : */
		dur += short_preamble ? (72 + 24) : (144 + 48);

		dur += DIV_ROUND_UP(8 * (len + 4) * 10, rate);
	}
	return dur;
}

static void calc_rate_durations(struct minstrel_node *mi, struct minstrel_rate *d, 
                                struct ieee80211_rate *rate)
{
	int erp = (d->rix > 3)? 1 : 0;

	/* : multiply rate by 5, rate*10 is required input 
	 * d->perfect_tx_time = ieee80211_frame_duration(1200,
	 * rate->bitrate, erp, 1);
	 */
	d->perfect_tx_time = ieee80211_frame_duration(1200, rate->bitrate*5, erp, 1);
	d->ack_time = ieee80211_frame_duration(10, rate->bitrate*5, erp, 1);
}
#if 0
void maxifi_eliminate_rates(struct ieee80211_node *ni)
{
	uint8 nss1_rates[] = {0,1,2,3,4,5,6,7};
	uint8 nss2_rates[] = {0,2,6,7,8,9,10,11,12,13,14,15};
	uint8 nss3_rates[] = {0,2,6,7,8,9,10,11,12,13,14,15};
	//u8 nss3_rates[] = {0,7,8,9,11,13,14,15,16,17,18,19,20,21,22,23};
	//u8 nss3_rates[] = {0,7,8,9,11,13,14,15,16,17,18,19,20};
	uint8 i;

	if (ni->ni_htrates.rs_nrates > 31) 
	{
		ni->ni_htrates.rs_nrates = 32;
	}
	if (ni->ni_htrates.rs_rates[ni->ni_htrates.rs_nrates-1] == 32) 
	{
		ni->ni_htrates.rs_nrates--;
	}
	if (ni->ni_flags & IEEE80211_NODE_HT) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("HT station %d %d \n"), 
		             ni->ni_htrates.rs_nrates, 
		             ni->ni_htrates.rs_rates[ni->ni_htrates.rs_nrates-1]));
		ni->ni_rates.rs_nrates = 4;
		if (ni->ni_htrates.rs_rates[ni->ni_htrates.rs_nrates-1] < 8) 
		{
			ni->ni_htrates.rs_nrates = sizeof(nss1_rates)/sizeof(nss1_rates[0]);
			for(i=0;i<ni->ni_htrates.rs_nrates;i++) 
			{
				ni->ni_htrates.rs_rates[i] = nss1_rates[i];
			}
			ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("single stream \n")));
		} 
		else if (ni->ni_htrates.rs_rates[ni->ni_htrates.rs_nrates-1] < 16) 
		{
			ni->ni_htrates.rs_nrates = sizeof(nss2_rates)/sizeof(nss2_rates[0]);
			for (i=0;i<ni->ni_htrates.rs_nrates;i++) 
			{
				ni->ni_htrates.rs_rates[i] = nss2_rates[i];
			}
			ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("two stream \n")));
		} 
		else 
		{
			ni->ni_htrates.rs_nrates = sizeof(nss3_rates)/sizeof(nss3_rates[0]);
			for (i=0;i<ni->ni_htrates.rs_nrates;i++) 
			{
				ni->ni_htrates.rs_rates[i] = nss3_rates[i];
			}
			ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("three stream\n")));
		}
	} 
	else 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("Legacy station\n")));
		ni->ni_htrates.rs_nrates = 0;
	}
}
#endif
/** This function is used to initialize the sample table with the rate indices
 * These indices are picked based on the node's supported rates.
 * @param  Pointer to driver w_adapter structure
 * @param  Pointer to the ieee80211_node structure
 * @return void
 */
static void ar_rate_ctl_reset(WLAN_ADAPTER w_adapter, struct ieee80211_node *ni)
{
	uint32 i, n = 0;
	uint32 t_slot = 9; /* : get real slot time */
	struct ieee80211_rate ctrl_rate;
	struct minstrel_node *sn = ni->hal_priv_node.ni_mn;
	struct ieee80211_rate ctl_rate;
	struct minstrel_priv *mp = ar_mp;

	//maxifi_eliminate_rates(ni);

	sn->static_rate_ndx = -1;
	ni->ni_txrate = 0; /* txmit rate index */ /* : */

	sn->n_rates = ni->ni_rates.rs_nrates + ni->ni_htrates.rs_nrates;

	if (sn->n_rates <= 0) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("CORE_MSG: rates not suppored by station\n")));
		return;
	}

	// Assign hix instead of rix here for hardware usage
	// if 11b mode is there lowest_rix = 0
	// if no 11b and 11g mode is there lowest_rix = 4
	// if no 11b and no 11g and 11n mode is there lowest_rix = 12
	sn->lowest_rix = 0;
	ctl_rate.bitrate = ni->ni_rates.rs_rates[sn->lowest_rix];
	sn->sp_ack_dur = ieee80211_frame_duration(10,ctl_rate.bitrate*5,0,1); // 

	for (i = 0; i < sn->n_rates; i++) 
	{
		struct minstrel_rate *mr = &sn->r[n];
		uint32 tx_time = 0, tx_time_cts = 0, tx_time_rtscts = 0;
		uint32 tx_time_single;
		uint32 cw = mp->cw_min;
		/* : Check for supported rates 
		 * if (!rate_supported(sta, sband->band, i))
		 * continue;
		 */

		n++;
		w_adapter->os_intf_ops->onebox_memset(mr, 0, sizeof(*mr));

		mr->rix = i;
		//mr->hix = i < ni->ni_rates.rs_nrates ? i : 12+ni->ni_htrates.rs_rates[i-ni->ni_rates.rs_nrates];
		ctrl_rate.flags = 0; /* :*/
		ctrl_rate.bitrate = i < ni->ni_rates.rs_nrates ? (ni->ni_rates.rs_rates[i] & IEEE80211_RATE_VAL) : 
		ieee80211_htrates[ni->ni_htrates.rs_rates[i - ni->ni_rates.rs_nrates]].ht20_rate_800ns;
#if 0
		ctrl_rate.bitrate = (i < ni->ni_rates.rs_nrates) ? ni->ni_rates.rs_rates[i] : ieee80211_htrates[ni->ni_htrates.rs_rates[i-(ni->ni_rates.rs_nrates)]];
		ctrl_rate.bitrate = (i < ni->ni_rates.rs_nrates) ?
		(ni->ni_rates.rs_rates[i] : 
		 ieee80211_htrates[ni->ni_htrates.rs_rates[i-(ni->ni_rates.rs_nrates)]]);//.ht20_rate_800ns;
#endif
		mr->bitrate = ctrl_rate.bitrate;
		//: Add a function in devdep layer
		//w_adapter->devdep_ops->onebox_get_hix(mr->bitrate);

		switch(mr->bitrate)
		{
			case IEEE80211_RATE_1M:
				mr->hix  = 0;
				break;
			case IEEE80211_RATE_2M:
				mr->hix  = 1;
				break;
			case IEEE80211_RATE_5_5M:
				mr->hix  = 2;
				break;
			case IEEE80211_RATE_11M:
				mr->hix  = 3;
				break;
			case IEEE80211_RATE_6M:
				mr->hix  = 4;
				break;
			case IEEE80211_RATE_9M:
				mr->hix  = 5;
				break;
			case IEEE80211_RATE_12M:
				mr->hix  = 6;
				break;
			case IEEE80211_RATE_18M:
				mr->hix  = 7;
				break;
			case IEEE80211_RATE_24M:
				mr->hix  = 8;
				break;
			case IEEE80211_RATE_36M:
				mr->hix = 9;
				break;
			case IEEE80211_RATE_48M:
				mr->hix = 10;
				break;
			case IEEE80211_RATE_54M:
				mr->hix = 11;
				break;
			case IEEE80211_RATE_6_5M:
				mr->hix = 12;
				break;
			case IEEE80211_RATE_13M:
				mr->hix = 13;
				break;
			case IEEE80211_RATE_19_5M:
				mr->hix = 14;
				break;
			case IEEE80211_RATE_26M:
				mr->hix = 15;
				break;
			case IEEE80211_RATE_39M:
				mr->hix = 16;
				break;
			case IEEE80211_RATE_52M:
				mr->hix = 17;
				break;
			case IEEE80211_RATE_58_5M:
				mr->hix = 18;
				break;
			case IEEE80211_RATE_65M:
				mr->hix = 19;
				break;
			default:
				if(w_adapter->operating_band == BAND_2_4_GHZ)
				{
					mr->hix = 0;
				}
				else // 5Ghz band default rate is 6Mbps
				{
					mr->hix = 4;
				}
				break;
		}
		calc_rate_durations(sn, mr, &ctrl_rate);

		/* calculate maximum number of retransmissions before
		 * fallback (based on maximum segment size) */
		mr->sample_limit = -1;
		mr->retry_count = 1;
		mr->retry_count_cts = 1;
		mr->retry_count_rtscts = 1;
		tx_time = mr->perfect_tx_time + sn->sp_ack_dur;

		do 
		{
			/* add one retransmission */
			tx_time_single = mr->ack_time + mr->perfect_tx_time;

			/* contention window */
			tx_time_single += t_slot + min(cw, mp->cw_max);
			cw = (cw + 1) << 1;

			tx_time += tx_time_single;
			tx_time_cts += tx_time_single + sn->sp_ack_dur;
			tx_time_rtscts += tx_time_single + 2 * sn->sp_ack_dur;
			if ((tx_time_cts < mp->segment_size) && (mr->retry_count_cts < mp->max_retry))
			{
				mr->retry_count_cts++;
			}
			if ((tx_time_rtscts < mp->segment_size) && (mr->retry_count_rtscts < mp->max_retry))
			{
				mr->retry_count_rtscts++;
			}
		} 
		while ((tx_time < mp->segment_size) && (++mr->retry_count < mp->max_retry));
		mr->adjusted_retry_count = mr->retry_count;
	}

	for (i = 0; i < sn->n_rates; i++) 
	{
		struct minstrel_rate *mr = &sn->r[i];
		ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,(TEXT("rix %d,hix %d,bitrate %d\n"),mr->rix,mr->hix,mr->bitrate));
	} 

	init_sample_table(w_adapter,sn, ni);
}

static void maxifi_rate_cb(void *arg, struct ieee80211_node *ni)
{
	struct onebox_os_intf_operations *os_intf_ops;
	WLAN_ADAPTER w_adapter;
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv((void *)ni->ni_ic->ic_ifp);
	ar_rate_ctl_reset(w_adapter, ni);
}

/**
 * This function  Resets the rate control state for each 802.11 state transition.
 *
 * @param       Pointer to the vap structure.
 * @param       variable.
 */
static void ar_newstate(struct ieee80211vap *vap, enum ieee80211_state newstate)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct onebox_os_intf_operations *os_intf_ops;
	WLAN_ADAPTER w_adapter;
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv((void *)ic->ic_ifp);

	if (newstate == IEEE80211_S_RUN) 
	{
		if (ic->ic_opmode != IEEE80211_M_STA) 
		{
			/* Sync rates for associated stations and neighbors. */
			w_adapter->net80211_ops->onebox_iterate_nodes(&ic->ic_sta, maxifi_rate_cb, NULL);
		}

		ar_newassoc(w_adapter, vap->iv_bss, 0);
	}
}

/**
 * This function allocates the per device minstrel information the driver.
 *
 * @param       Pointer to the driver private structure
 * @return      none 
 */
static void *minstrel_alloc(WLAN_ADAPTER w_adapter)
{
	struct minstrel_priv *mp;

	mp = w_adapter->os_intf_ops->onebox_mem_zalloc(sizeof(struct minstrel_priv), 0);
	if (!mp)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:ERROR:mp is NULL\n"),__func__));
		return NULL;
	}

	/* contention window settings
	 * Just an approximation. Using the per-queue values would complicate
	 * the calculations and is probably unnecessary */
	mp->cw_min = 15;
	mp->cw_max = 1023;

	/* number of packets (in %) to use for sampling other rates
	 * sample less often for non-mrr packets, because the overhead
	 * is much higher than with mrr */
	mp->lookaround_rate = 5;
#ifdef AMPDU_AGGR_SUPPORT
	mp->lookaround_rate_mrr = 10;
#else
	mp->lookaround_rate_mrr = 10;
#endif

	/* moving average weight for EWMA */
	mp->ewma_level = 75;

	/* maximum time that the hw is allowed to stay in one MRR segment */
	mp->segment_size = 6000;

	mp->max_retry = 3; //7

	mp->has_mrr = 0;

	mp->update_interval = 100;

	return mp;
}

/**
 * This function frees the per device minstrel information in the driver.
 *
 * @param       Pointer to the driver private structure.
 * @return      none.
 */
static void minstrel_free(WLAN_ADAPTER w_adapter,void *priv)
{
	w_adapter->os_intf_ops->onebox_mem_free(priv);
}

/**
 * This function detaches the autorate algorithm registered with the driver.
 *
 * @param       Pointer to the driver private structure.
 * @param       Pointer to the ratecontrol structure.
 * @return      none.
 */
static void ar_detach(WLAN_ADAPTER w_adapter,struct wl_ratectrl *arc)
{
	struct minstrel_softc *osc = (struct minstrel_softc *) arc;

	if (ar_mp != NULL) 
	{
		minstrel_free(w_adapter,ar_mp);
	}
	if(osc == NULL)
	{
		return;
	}

	//: crashing while freeing this address FarSow
	// dynamic allocation was not not done for arc so can't use kfree here.
	//  w_adapter->os_intf_ops->onebox_mem_free(osc);
}

/**
 * This function updates the autorate statistics periodically.
 * @param       Pointer to the driver private structure.
 * @return      none.
 */
static void ar_update_ratestats(WLAN_ADAPTER w_adapter)
{
	struct ieee80211com *ic = &w_adapter->vap_com;

	/* Sync rates for associated stations and neighbors. */
	w_adapter->net80211_ops->onebox_iterate_nodes(&ic->ic_sta, minstrel_update_stats, (void *)w_adapter);
}


static struct wl_ratectrl rate_module;


static struct ieee80211_rate_ops core_rate_ops = 
{
	IEEE80211_RATECTL_AMRR,
	NULL,
	NULL,
	NULL,
	ar_update_ratestats,
	minstrel_get_rate,
	ar_newassoc,
	ar_newstate,
};

static struct ieee80211_ratectl minstrel_ratectl = 
{
	.ir_name        = "minstrel",
	.ir_attach      = NULL,
	.ir_detach      = NULL,
	.ir_init        = NULL,
	.ir_deinit      = NULL,
	.ir_node_init    = minstrel_node_init,
	.ir_node_deinit = minstrel_node_deinit,
	.ir_rate        = NULL,
	.ir_tx_complete = NULL,
	.ir_tx_update   = NULL,
	.ir_setinterval = NULL,
};

struct ieee80211_ratectl *onebox_get_ieee80211_ratectl(void)
{
	return (&minstrel_ratectl);
}

struct ieee80211_rate_ops *onebox_get_ieee80211_rate_ops(void)
{
	return (&core_rate_ops);
}
EXPORT_SYMBOL(onebox_get_ieee80211_rate_ops);

struct minstrel_softc * ar_attach(WLAN_ADAPTER w_adapter,struct minstrel_softc *osc)
{

	struct minstrel_priv *mp = NULL; /* FreeBSD */

	osc = w_adapter->os_intf_ops->onebox_mem_zalloc(sizeof(struct minstrel_softc),0);
	if (osc == NULL) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:ERROR:osc is NULL\n"),__func__));
		return osc;
	}

	osc->arc.arc_space = sizeof(struct minstrel_node);
	osc->arc.arc_vap_space = 0;
	osc->arc.ops = &core_rate_ops;

	osc->w_adapter = w_adapter;

	mp = (struct minstrel_priv *) minstrel_alloc(w_adapter);
	ar_mp = mp;

	return osc;
}

/**
 * This function attaches the autorate algorithm to be used in the driver.
 *
 * @param       pointer to the driver private structure.
 * @return      none.
 */
struct ieee80211_rate_ops *core_rate_attach(WLAN_ADAPTER w_adapter)
{
	struct minstrel_softc *min_sc=NULL;

	min_sc = ar_attach(w_adapter,min_sc);

	if (min_sc == NULL) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:ERROR:min_sec is NULL\n"),__func__));
		return NULL;
	}

	return min_sc->arc.ops;
}

/**
 * This function detaches the autorate algorithm registered with the driver.
 * @param       pointer to the driver private structure.
 * @return      none.
 */
void core_rate_detach(WLAN_ADAPTER w_adapter)
{
	ar_detach(w_adapter,&rate_module);
	return;
}

