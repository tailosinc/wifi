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
#include "onebox_common.h"

#define IEEE80211_FCTL_STYPE  0xF0
#define ONEBOX_80211_FC_PROBE_RESP 0x50
/* Function Prototypes */
void sdio_interrupt_handler(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS onebox_read_pkt(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS load_ta_instructions(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS load_fw_thru_sbl(PONEBOX_ADAPTER adapter);
void coex_transmit_thread(void *data);
ONEBOX_STATUS device_init(PONEBOX_ADAPTER adapter, uint8 fw_load);
ONEBOX_STATUS device_deinit(PONEBOX_ADAPTER adapter);
int  bl_cmd(PONEBOX_ADAPTER adapter, uint8 cmd, uint8 exp_resp, char *str);
int bl_write_cmd(PONEBOX_ADAPTER adapter, uint8 cmd, uint8 exp_resp, uint16 *cmd_resp);
ONEBOX_STATUS ping_pong_write(PONEBOX_ADAPTER adapter, uint8 cmd, uint8 *addr, uint32 size);
int32 sdio_ta_reset_ops(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS sdio_load_data_master_write(PONEBOX_ADAPTER adapter, uint32 base_address, uint32 instructions_sz,
	uint32 block_size, uint8 *ta_firmware);
ONEBOX_STATUS usb_load_data_master_write(PONEBOX_ADAPTER adapter, uint32 base_address, uint32 instructions_sz,
	uint32 block_size, uint8 *ta_firmware);
void bl_cmd_start_timer(PONEBOX_ADAPTER adapter, uint32 timeout);
void bl_cmd_stop_timer(PONEBOX_ADAPTER adapter);
void bl_cmd_timer_expired(PONEBOX_ADAPTER adapter);
uint32 read_flash_capacity(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS verify_flash_content( PONEBOX_ADAPTER adapter, uint8 *flash_content, 
		uint32 instructions_sz, uint32 eeprom_offset, uint8  read_mode);
ONEBOX_STATUS read_flash_content(PONEBOX_ADAPTER adapter,uint8 *temp_buf,uint32 address,uint32 len);
uint32 checksum_32bit(uint8 **scatter_addr, uint32 *scatter_len, uint32 no_of_scatters);
#endif

