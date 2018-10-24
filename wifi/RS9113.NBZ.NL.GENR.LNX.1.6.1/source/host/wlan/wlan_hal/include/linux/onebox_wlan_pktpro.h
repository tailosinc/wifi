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

#ifndef __ONEBOX_PKTPRO_H__
#define __ONEBOX_PKTPRO_H__

#include "onebox_datatypes.h"
#include "wlan_common.h"

#define IEEE80211_FCTL_STYPE  0xF0
#define ONEBOX_80211_FC_PROBE_RESP 0x50

/* Function Prototypes */
void interrupt_handler(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS send_beacon(WLAN_ADAPTER w_adapter, 
                          netbuf_ctrl_block_t *netbuf_cb,
                          struct core_vap *core_vp,
                          int8 dtim_beacon);
ONEBOX_STATUS send_onair_data_pkt(WLAN_ADAPTER w_adapter,
                                  netbuf_ctrl_block_t *netbuf_cb,
                                  int8 q_num);
ONEBOX_STATUS send_bt_pkt(WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb);
ONEBOX_STATUS send_zigb_pkt(WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb);
ONEBOX_STATUS onebox_load_config_vals(WLAN_ADAPTER  w_adapter);
int32 load_ta_instructions(WLAN_ADAPTER w_adapter);
void sdio_scheduler_thread(void *data);
ONEBOX_STATUS device_init(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS device_deinit(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS device_queues_status(WLAN_ADAPTER w_adapter, uint8 q_num);
int hal_set_sec_wpa_key(WLAN_ADAPTER w_adapter,const struct ieee80211_node *ni_sta, uint8 key_type);
uint32 hal_send_sta_notify_frame(WLAN_ADAPTER w_adapter, struct ieee80211_node *sta_addr, int32 sta_id);

#endif

