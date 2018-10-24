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

#include "bt_common.h"
#include "onebox_bt_core.h"
#include "onebox_bt_pktpro.h"

static struct onebox_osi_bt_ops osi_bt_ops = {
	.onebox_core_init            = core_bt_init,
	.onebox_core_deinit          = core_bt_deinit,
	.onebox_indicate_pkt_to_core = bt_core_pkt_recv,
	.onebox_dump                 = onebox_print_dump,
	.onebox_bt_xmit              = bt_xmit,
	.onebox_snd_cmd_pkt          = onebox_bt_mgmt_pkt_recv,
	.onebox_send_pkt             = send_bt_pkt,
	.onebox_bt_cw                = bt_cw,
 	.onebox_bt_ber               = bt_ber,
	.onebox_bt_read_reg_params   = bt_read_reg_params,
};

struct onebox_osi_bt_ops *onebox_get_osi_bt_ops(void)
{
	return &osi_bt_ops;
}
EXPORT_SYMBOL(onebox_get_osi_bt_ops);
