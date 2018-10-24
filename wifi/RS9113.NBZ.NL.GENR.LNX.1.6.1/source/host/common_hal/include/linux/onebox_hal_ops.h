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

#include "onebox_common.h"
#include "onebox_thread.h"
#include "onebox_linux.h"
#include "onebox_hal.h"

#include <linux/usb.h>

/* COEX OPERATIONS */
struct onebox_coex_osi_operations
{
	void (*onebox_handle_interrupt)(PONEBOX_ADAPTER adapter);
	void (*onebox_coex_pkt_processor)(PONEBOX_ADAPTER adapter);
	void (*onebox_dump)(int32 dbg_zone_l, PUCHAR msg_to_print_p,int32 len);
	ONEBOX_STATUS (*onebox_device_init)(PONEBOX_ADAPTER adapter, uint8 fw_load);
	ONEBOX_STATUS (*onebox_device_deinit)(PONEBOX_ADAPTER adapter);
	void (*onebox_coex_sched_pkt_xmit)(PONEBOX_ADAPTER adapter);
	void (*onebox_coex_transmit_thread)(void *data);
	ONEBOX_STATUS (*onebox_coex_mgmt_frame)(PONEBOX_ADAPTER adapter, uint16 *addr, uint16 len);
	ONEBOX_STATUS (*onebox_manual_flash_write)(PONEBOX_ADAPTER adapter);
	ONEBOX_STATUS (*onebox_auto_flash_write)(PONEBOX_ADAPTER adapter, uint8 *, uint32);
	ONEBOX_STATUS (*onebox_flash_read)(PONEBOX_ADAPTER adapter);
	ONEBOX_STATUS (*onebox_auto_fw_upgrade)(PONEBOX_ADAPTER adapter, uint8 *flash_content, uint32 content_size);
 	int (*onebox_bl_write_cmd)(PONEBOX_ADAPTER adapter, uint8 cmd, uint8 exp_resp, uint16 *cmd_resp);
	ONEBOX_STATUS (*onebox_read_reg_params) (PONEBOX_ADAPTER adapter);
	ONEBOX_STATUS (*onebox_common_hal_init)(struct driver_assets *d_assets, PONEBOX_ADAPTER adapter);
    void (*onebox_common_hal_deinit)(struct driver_assets *d_assets, PONEBOX_ADAPTER adapter);
    void (*send_coex_configuration)(PONEBOX_ADAPTER adapter,coex_cmd_t *coex_cmd_p );
    ONEBOX_STATUS (*onebox_wait_for_rf_prog_frame_rsp)(struct driver_assets *d_assets);
};

/* EOF */
