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

#ifndef __ONEBOX_PS_H__
#define __ONEBOX_PS_H__
#include "wlan_common.h"

#define DEEP_SLEEP		1
#define CONNECTED_SLEEP		2

#define PS_ENABLE 1
#define PS_DISABLE 0
#if 0
#define CONFIRM_PATH 1
#define MGMT_PENDING_PATH 2
#define IOCTL_PATH 3
#define CONNECTED_PATH 4
#define DISCONNECTED_PATH 5
#endif

enum ps_paths {
MGMT_PENDING_PATH = 1,//will handle both scan_start and any mgmt pending in DEEP_SLEEP pwr_save state 
IOCTL_PATH,
CONNECTED_PATH,
DISCONNECTED_PATH,
SCAN_END_PATH,
TIMER_PATH,
VAP_CREATE_PATH,
VAP_DELETE_PATH,
TX_RX_PATH,
PS_EN_DEQUEUED_PATH,
PS_EN_REQ_CNFM_PATH,
PS_DIS_REQ_CNFM_PATH
};



void update_pwr_save_status(struct ieee80211vap *vap, uint32 ps_status, uint32 path);
void support_mimic_uapsd(WLAN_ADAPTER w_adapter, unsigned char *buffer);

#endif
