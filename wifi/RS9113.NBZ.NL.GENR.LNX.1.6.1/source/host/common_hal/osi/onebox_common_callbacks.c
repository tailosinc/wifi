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
#include "onebox_core.h"
#include "onebox_linux.h"
#include "onebox_pktpro.h"

static struct onebox_coex_osi_operations coex_osi_ops =
{
	/* In case if u want to change the order of these member assignments, you should change
	 * the order in the declarations also */
	.onebox_handle_interrupt          = sdio_interrupt_handler,
	.onebox_coex_pkt_processor        = coex_pkt_processor,
	.onebox_dump                      = onebox_print_dump,
	.onebox_device_init               = device_init,
	.onebox_device_deinit             = device_deinit,
	.onebox_coex_sched_pkt_xmit       = coex_sched_pkt_xmit,
	.onebox_coex_transmit_thread      = coex_transmit_thread,
	.onebox_coex_mgmt_frame           = onebox_coex_mgmt_frame,
	.onebox_auto_flash_write          = auto_flash_write,
	.onebox_manual_flash_write        = manual_flash_write,
	.onebox_flash_read                = flash_read,
	.onebox_auto_fw_upgrade           = auto_fw_upgrade,
 	.onebox_bl_write_cmd              = bl_write_cmd,
	.onebox_read_reg_params           = read_reg_parameters,
	.onebox_common_hal_init           = common_hal_init,
    .onebox_common_hal_deinit         = common_hal_deinit,
    .send_coex_configuration         = send_coex_configuration, 
    .onebox_wait_for_rf_prog_frame_rsp = onebox_wait_for_rf_prog_frame_rsp,
};

struct onebox_coex_osi_operations *onebox_get_coex_osi_operations(void)
{
	return (&coex_osi_ops);
}
EXPORT_SYMBOL(onebox_get_coex_osi_operations);

