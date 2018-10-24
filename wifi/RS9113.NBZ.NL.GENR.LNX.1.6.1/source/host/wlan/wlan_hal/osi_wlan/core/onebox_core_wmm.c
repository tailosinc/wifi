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

/**
 * This function gets the contention values for the backoff procedure.
 *
 * @param Pointer to the channel acces params sturcture
 * @param Pointer to the driver's private structure.
 * @return none.
 */ 
void onebox_set_contention_vals(struct ieee80211com *ic, WLAN_ADAPTER w_adapter)
{
	struct chanAccParams *wme_params = &ic->ic_wme.wme_wmeChanParams;

	w_adapter->wme_org_q[VO_Q_STA] = (((wme_params->cap_wmeParams[WME_AC_VO].wmep_logcwmin / 2 ) + 
	                            (wme_params->cap_wmeParams[WME_AC_VO].wmep_aifsn)) * WMM_SHORT_SLOT_TIME + SIFS_DURATION);
	w_adapter->wme_org_q[VI_Q_STA] = (((wme_params->cap_wmeParams[WME_AC_VI].wmep_logcwmin / 2 ) + 
	                            (wme_params->cap_wmeParams[WME_AC_VI].wmep_aifsn)) * WMM_SHORT_SLOT_TIME + SIFS_DURATION);
	w_adapter->wme_org_q[BE_Q_STA] = (((wme_params->cap_wmeParams[WME_AC_BE].wmep_logcwmin / 2 ) + 
	                            (wme_params->cap_wmeParams[WME_AC_BE].wmep_aifsn-1)) * WMM_SHORT_SLOT_TIME + SIFS_DURATION);
	w_adapter->wme_org_q[BK_Q_STA] = (((wme_params->cap_wmeParams[WME_AC_BK].wmep_logcwmin / 2 ) + 
	                            (wme_params->cap_wmeParams[WME_AC_BK].wmep_aifsn)) * WMM_SHORT_SLOT_TIME + SIFS_DURATION);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d QUEUE WT are \n"), __func__, __LINE__));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("BE_Q %d \n"),w_adapter->wme_org_q[BE_Q_STA] ));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("BK_Q %d \n"),w_adapter->wme_org_q[BK_Q_STA] ));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("VI_Q %d \n"),w_adapter->wme_org_q[VI_Q_STA] ));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("VO_Q %d \n"),w_adapter->wme_org_q[VO_Q_STA] ));

	/* For AP 4 more Queues */
	w_adapter->wme_org_q[VO_Q_AP] = (((wme_params->cap_wmeParams[WME_AC_VO].wmep_logcwmin / 2 ) + 
	                            (wme_params->cap_wmeParams[WME_AC_VO].wmep_aifsn)) * WMM_SHORT_SLOT_TIME + SIFS_DURATION);
	w_adapter->wme_org_q[VI_Q_AP] = (((wme_params->cap_wmeParams[WME_AC_VI].wmep_logcwmin / 2 ) + 
	                            (wme_params->cap_wmeParams[WME_AC_VI].wmep_aifsn)) * WMM_SHORT_SLOT_TIME + SIFS_DURATION);
	w_adapter->wme_org_q[BE_Q_AP] = (((wme_params->cap_wmeParams[WME_AC_BE].wmep_logcwmin / 2 ) + 
	                            (wme_params->cap_wmeParams[WME_AC_BE].wmep_aifsn)) * WMM_SHORT_SLOT_TIME + SIFS_DURATION);
	w_adapter->wme_org_q[BK_Q_AP] = (((wme_params->cap_wmeParams[WME_AC_BK].wmep_logcwmin / 2 ) + 
	                            (wme_params->cap_wmeParams[WME_AC_BK].wmep_aifsn)) * WMM_SHORT_SLOT_TIME + SIFS_DURATION);
	w_adapter->os_intf_ops->onebox_memcpy(w_adapter->per_q_wt, w_adapter->wme_org_q, sizeof(w_adapter->per_q_wt));
	w_adapter->os_intf_ops->onebox_memset(w_adapter->pkt_contended, 0, sizeof(w_adapter->pkt_contended));
}

/**
 * This function determines the HAL queue from which packets has to be dequeued while transmission.
 *
 * @param Pointer to the driver's private structure .
 * @return ONEBOX_STATUS_SUCCESS on success else ONEBOX_STATUS_FAILURE.
 */
uint8 core_determine_hal_queue(WLAN_ADAPTER w_adapter)
{
	uint8 q_num = INVALID_QUEUE;
	uint8 ii,min = 0;
	uint8 fresh_contention;
	struct ieee80211com *ic;
	struct chanAccParams *wme_params_sta;
	
	ic= &w_adapter->vap_com;
	wme_params_sta = &ic->ic_wme.wme_wmeChanParams;
	
	if (w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[MGMT_SOFT_Q]))
	{
		q_num = MGMT_SOFT_Q;
		return q_num;
	}   
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("per q wt values in %d:  %d %d %d %d \n"), 
		                                __LINE__, w_adapter->per_q_wt[0], w_adapter->per_q_wt[1],
		                                w_adapter->per_q_wt[2], w_adapter->per_q_wt[3]));
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("selected queue num and pkt cnt are : %d %d %d \n"),
		                                __LINE__, w_adapter->selected_qnum, w_adapter->pkt_cnt));
		if (w_adapter->pkt_cnt != 0)
		{
			w_adapter->pkt_cnt -= 1;
			return (w_adapter->selected_qnum);
		}

GET_QUEUE_NUM:
		q_num = 0;
		fresh_contention = 0;

		/* Selecting first valid contention value */
		for(ii = 0; ii < NUM_EDCA_QUEUES ; ii++)
		{
			if(w_adapter->pkt_contended[ii] && 
			   ((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[ii])) != 0)) /* Check for contended value*/
			{
				min = w_adapter->per_q_wt[ii];
				q_num = ii;
				break;
			}
		}

		/* Selecting the queue with least back off */
		for(; ii < NUM_EDCA_QUEUES ; ii++) /* Start finding the least value from first valid value itself
		                                    * Leave the value of ii as is  from previous loop */
		{
			if(w_adapter->pkt_contended[ii] && (w_adapter->per_q_wt[ii] < min) 
			   && ((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[ii]) !=0))) /* Check only if contended */
			{
				min = w_adapter->per_q_wt[ii];
				q_num = ii;
			}
		}
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("min =%d and qnum=%d\n"), min, q_num));

		/* Adjust the back off values for all queues again */
		w_adapter->pkt_contended[q_num] = 0; /* Reset the contention for the current queue so that it gets org value again if it has more packets */

		for(ii = 0; ii< NUM_EDCA_QUEUES; ii++)
		{
			if(w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[ii]))/* Check for the need of contention */
			{ 
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("queue %d len %d\n"), 
				                             ii, w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[ii])));
				if(w_adapter->pkt_contended[ii])
				{
					if(w_adapter->per_q_wt[ii] > min)
					{ /* Subtracting the backoff value if already contended */
						w_adapter->per_q_wt[ii] -= min;
					}
					else /* This case occurs only two queues end up in having same back off value and is least */
					{
						w_adapter->per_q_wt[ii] = 0;
					}
				}
				else /* Fresh contention */
				{
					w_adapter->pkt_contended[ii] = 1;
					w_adapter->per_q_wt[ii] = w_adapter->wme_org_q[ii];
					fresh_contention = 1;
				}
			}
			else
			{ /* No packets so no contention */
				w_adapter->per_q_wt[ii] = 0;
				w_adapter->pkt_contended[ii] = 0;
			}
		}
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("per q values in %d:  %d %d %d %d \n"),
		                       __LINE__, w_adapter->per_q_wt[0], w_adapter->per_q_wt[1], w_adapter->per_q_wt[2], w_adapter->per_q_wt[3]));
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("pkt contended val in %d:  %d %d %d %d \n\n"),
		                            __LINE__, w_adapter->pkt_contended[0], w_adapter->pkt_contended[1], 
		                            w_adapter->pkt_contended[2], w_adapter->pkt_contended[3]));
		if((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[q_num])) == 0)
		{
		/* If any queues are freshly contended and the selected queue doesn't have any packets
		 * then get the queue number again with fresh values */
			if(fresh_contention)
			{
				goto GET_QUEUE_NUM;
			}
			q_num = INVALID_QUEUE;
			return q_num;
		}

		w_adapter->selected_qnum = q_num ;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("WMM::queue num after algo= %d \n"), q_num));
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("queue %d len %d \n"), q_num, 
				                        w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[q_num])));

		if ((w_adapter->selected_qnum == VO_Q_STA) || (w_adapter->selected_qnum == VI_Q_STA))
		{
			if(w_adapter->selected_qnum == VO_Q_STA) {
				if(wme_params_sta->cap_wmeParams[w_adapter->selected_qnum].wmep_acm)
					w_adapter->pkt_cnt = 1;
				else
					w_adapter->pkt_cnt = 6;
				//w_adapter->pkt_cnt = ((wme_params_sta->cap_wmeParams[w_adapter->selected_qnum].wmep_txopLimit << 5) / 150);
			}	else { 
				if(wme_params_sta->cap_wmeParams[w_adapter->selected_qnum].wmep_acm) {
					w_adapter->pkt_cnt = 1;
				} else {
					w_adapter->pkt_cnt = ((wme_params_sta->cap_wmeParams[w_adapter->selected_qnum].wmep_txopLimit << 5) / 800);
					//w_adapter->pkt_cnt = 6;
				}
			}

			if((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[q_num])) <= w_adapter->pkt_cnt)
			{
				w_adapter->pkt_cnt = ((w_adapter->os_intf_ops->onebox_netbuf_queue_len(&w_adapter->host_tx_queue[q_num])));
			}
			if(w_adapter->pkt_cnt != 0)
			{
			w_adapter->pkt_cnt -= 1;
			}
			else
			{
			w_adapter->pkt_cnt = 0;
			}
		}
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("pkt_cnt and q_num are: %d %d \n"), w_adapter->pkt_cnt, q_num));
		return (q_num);
	}
	return ONEBOX_STATUS_FAILURE;
}
