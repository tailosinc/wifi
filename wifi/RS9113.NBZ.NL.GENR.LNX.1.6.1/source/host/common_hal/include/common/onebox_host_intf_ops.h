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
//#include "onebox_thread.h"
#include "onebox_linux.h"

#include <linux/usb.h>

/* HOST_INTERFACE OPERATIONS */
struct onebox_osd_host_intf_operations
{
	ONEBOX_STATUS (*onebox_read_register)(PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data);
	int (*onebox_write_register)(PONEBOX_ADAPTER adapter,uint8 reg_dmn,uint32 Addr,uint8 *data);
	ONEBOX_STATUS (*onebox_read_multiple)(PONEBOX_ADAPTER adapter, uint32 Addr, uint32 Count, uint8 *data );
	ONEBOX_STATUS (*onebox_write_multiple)(PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data, uint32 Count, netbuf_ctrl_block_t *netbuf_cb);
	int (*onebox_register_drv)(void);
	void (*onebox_unregister_drv)(void);
	ONEBOX_STATUS (*onebox_remove)(void);
	void (*onebox_rcv_urb_submit) (PONEBOX_ADAPTER adapter, struct urb *urb, uint8 ep_num);
	ONEBOX_STATUS (*onebox_ta_write_multiple)(PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data, uint32 Count);
	ONEBOX_STATUS (*onebox_ta_read_multiple)(PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data, uint32 Count);
	int32 (*onebox_deregister_irq)(PONEBOX_ADAPTER adapter);
	ONEBOX_STATUS (*onebox_master_reg_read)(PONEBOX_ADAPTER adapter,uint32 addr,uint32 *data, uint16 size);
	ONEBOX_STATUS (*onebox_master_reg_write)(PONEBOX_ADAPTER adapter, unsigned long addr, unsigned long data, uint16 size);
  ONEBOX_STATUS (*onebox_trace_firmware)(struct driver_assets *d_assets);
  ONEBOX_STATUS (*sdio_fnx_cmd52)(PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data, uint8 function_no,uint8 write);
  ONEBOX_STATUS (*sdio_fnx_cmd53) (PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data,uint8 write, uint8 Count);
};

/* HOST_INTERFACE OPERATIONS */
struct onebox_osi_host_intf_operations
{
	ONEBOX_STATUS (*onebox_host_intf_read_pkt)(PONEBOX_ADAPTER adapter,uint8 *pkt,uint32 Len);
	//ONEBOX_STATUS (*onebox_host_intf_write_pkt)(PONEBOX_ADAPTER adapter,uint8 *pkt,uint32 Len, uint8 queueno);
	ONEBOX_STATUS (*onebox_host_intf_write_pkt)(PONEBOX_ADAPTER adapter,uint8 *pkt,uint32 Len, uint8 queueno, netbuf_ctrl_block_t *netbuf_cb);
	ONEBOX_STATUS (*onebox_ack_interrupt)(PONEBOX_ADAPTER adapter,uint8 int_BIT);
	ONEBOX_STATUS (*onebox_disable_sdio_interrupt)(PONEBOX_ADAPTER adapter);
	ONEBOX_STATUS (*onebox_init_host_interface)(PONEBOX_ADAPTER adapter);
	ONEBOX_STATUS (*onebox_master_access_msword)(PONEBOX_ADAPTER adapter, uint16 ms_word);
};
/* EOF */
