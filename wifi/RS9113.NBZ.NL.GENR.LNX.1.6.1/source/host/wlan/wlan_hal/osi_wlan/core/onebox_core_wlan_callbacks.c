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

#include "onebox_wlan_core.h"

/* Core callbacks */
struct onebox_core_operations core_ops =
{
	core_init,               /*.onebox_core_init                = core_init,*/
	core_deinit,             /*.onebox_core_deinit              = core_deinit,*/
	core_xmit,               /*.onebox_core_xmit                = core_xmit, */
	core_update_tx_status,   /*.onebox_update_tx_status         = core_update_tx_status,*/
	core_tx_data_done,       /*.onebox_tx_data_done             = core_tx_data_done, */
	wlan_core_pkt_recv,           /*.onebox_indicate_pkt_to_net80211 = wlan_core_pkt_recv,*/
	NULL,//core_pkt_recv,           /*.onebox_indicate_pkt_to_core = bt_core_pkt_recv,*/
#ifdef BYPASS_RX_DATA_PATH
	bypass_data_pkt,           /*.onebox_bypass_rx_data_pkt = bypass_data_pkt,*/
#endif
	core_qos_processor,      /*.onebox_core_qos_processor       = core_qos_processor,*/
	vap_load_beacon,         /*.onebox_vap_load_beacon          = vap_load_beacon,*/
	onebox_vap_create,       /*.onebox_create_vap               = onebox_vap_create,*/
	core_radiotap_tx,        /*.onebox_core_radiotap_tx         = core_radiotap_tx*/
	onebox_print_dump,       /*.onebox_dump                     = onebox_print_dump,*/
	core_michael_failure, /*.onebox_mic_failure=core_mic_failure,*/
	stats_frame,              /*.onebox_stats_frame              = stats_frame*/
	NULL,                    /*.onebo_load_deep_sleep        = NULL,*/
	start_per_tx,            /*.onebox_start_per_tx             = start_per_tx,*/
	start_per_burst,         /*.onebox_start_per_burst          = start_per_burst*/
	NULL,//core_bt_xmit,			/*.onebox_core_bt_xmit		   = core_bt_xmit */
  core_net80211_attach,
};

struct onebox_core_operations *onebox_get_core_wlan_operations (void)
{
	return (&core_ops);
}
EXPORT_SYMBOL(onebox_get_core_wlan_operations);
