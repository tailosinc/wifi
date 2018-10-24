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
#include "onebox_linux.h"

struct onebox_osi_bt_ops {
	int32	(*onebox_core_init)(BT_ADAPTER bt_adapter);
	int32	(*onebox_core_deinit)(BT_ADAPTER bt_adapter);
	int32	(*onebox_indicate_pkt_to_core)(BT_ADAPTER bt_adapter,
					       netbuf_ctrl_block_t *netbuf_cb);
	void	(*onebox_dump)(int32 dbg_zone_l, PUCHAR msg_to_print_p,int32 len);
	int32 	(*onebox_bt_xmit) (BT_ADAPTER bt_adapter, 
	       		   netbuf_ctrl_block_t *netbuf_cb);
	int32	(*onebox_snd_cmd_pkt)(BT_ADAPTER  bt_adapter,
	       		      netbuf_ctrl_block_t *netbuf_cb);
	int32	(*onebox_send_pkt)(BT_ADAPTER bt_adapter,
                                   netbuf_ctrl_block_t *netbuf_cb);
	int32	(*onebox_bt_cw)(BT_ADAPTER bt_adapter,
                                   unsigned char *,int );
	int32	(*onebox_bt_ber)(BT_ADAPTER bt_adapter,
                                   unsigned char *,int );
	int32 (*onebox_bt_read_reg_params)(BT_ADAPTER bt_adapter);
};

struct onebox_bt_osd_operations 
{
#ifdef USE_BLUEZ_BT_STACK
  int32	(*onebox_btstack_init)(BT_ADAPTER bt_adapter);
  int32	(*onebox_btstack_deinit)(BT_ADAPTER bt_adapter);
  int32	(*onebox_send_pkt_to_btstack)(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb);
#endif
  int32	(*onebox_btgenl_init)(BT_ADAPTER bt_adapter);
  int32	(*onebox_btgenl_deinit)(BT_ADAPTER bt_adapter);
  int32	(*onebox_send_pkt_to_genl)(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb);
#ifdef RSI_CONFIG_ANDROID
  int32 (*onebox_bdroid_init)(BT_ADAPTER bt_adapter);
  void  (*onebox_bdroid_deinit)(BT_ADAPTER bt_adapter);
#endif
};

/* EOF */
