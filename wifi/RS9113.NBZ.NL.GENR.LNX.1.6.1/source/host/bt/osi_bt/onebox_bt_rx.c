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

/* include files */
#include "bt_common.h"
#include "onebox_linux.h"
#include "onebox_bt_pktpro.h"
#include "onebox_bt_lmp.h"
#include "onebox_bt_core.h"

#define HOST_WLCSP 1 

extern short watch_bufferfull_count;
extern char hci_events[78][47];

static void print_lmp_pkt_details(uint8 *buffer, uint16 pkt_len)
{
	uint8 slave_id, i;
	uint8 lmp_opcode;
	uint8 buffer_index = 0;
	uint8 tx_or_rx;
	char  prefix_string[6] = "LMPTX";

	slave_id = buffer[buffer_index];
	buffer_index++;

	tx_or_rx = buffer[buffer_index];
	buffer_index++;
	if(tx_or_rx == RX_PKT_DEBUG_MSG)
	{
		prefix_string[3] = 'R';
	}

	lmp_opcode = buffer[buffer_index] >> 1;
	buffer_index++;

	if(lmp_opcode != LMP_ESCAPE_OPCODE)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s %s\n"), prefix_string, lmp_opcodes[lmp_opcode]));
		switch(lmp_opcode)
		{
			case LMP_NAME_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s NAME_OFFSET: %02x \n"), prefix_string, buffer[buffer_index]));
			}
			break;
			case LMP_NAME_RES:
			{
				uint8 len;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s NAME_OFFSET: %02x \n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s NAME_FRAG_LEN: %02x \n"), prefix_string, buffer[buffer_index]));
				len = buffer[buffer_index];
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s NAME_FRAGMENT:"), prefix_string));
				for(i = 0; i < len; i++)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%c"), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_ACCEPTED:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s REQ_OPCODE: %s \n"), prefix_string, lmp_opcodes[buffer[buffer_index]]));
			}
			break;
			case LMP_NOT_ACCEPTED:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s REQ_OPCODE: %s \n"), prefix_string, lmp_opcodes[buffer[buffer_index]]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s REJECTION_REASON: %02x \n"), prefix_string, buffer[buffer_index]));//create an array for error codes?
			}
			break;
			case LMP_CLKOFFSET_REQ:
			{
			}
			break;
			case LMP_CLKOFFSET_RES:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s CLOCK_OFFSET: %04x \n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
			}
			break;
			case LMP_DETACH:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s REASON_CODE: %02x \n"), prefix_string, buffer[buffer_index]));// create an array for error codes?
			}
			break;
			case LMP_IN_RAND:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s IN_RAND_RANDOM_NUM:"), prefix_string));
				for(i = 0; i < 16; i++) //replace with onebox_dump
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_COMB_KEY:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s COMB_KEY_RANDOM_NUM:"), prefix_string));
				for(i = 0; i < 16; i++) //replace with onebox_dump
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_UNIT_KEY:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s UNIT_KEY_RANDOM_NUM:"), prefix_string));
				for(i = 0; i < 16; i++)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_AU_RAND:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s AU_RANDOM_NUM:"), prefix_string));
				for(i = 0; i < 16; i++)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_SRES:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s SRES: %08x\n"), prefix_string, *(uint32 *)&buffer[buffer_index]));
			}
			break;
			case LMP_TEMP_RAND:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LMP TEMP RAND: "), prefix_string));
				for(i = 0; i < 16; i++)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_TEMP_KEY:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LMP TEMP KEY : "), prefix_string));
				for(i = 0; i < 16; i++)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_ENCRYPTION_MODE_REQ:
			{
				if(buffer[buffer_index])
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s ENC_ENABLE\n"), prefix_string));
				}
				else
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s ENC_DISABLE\n"), prefix_string));
				}

			}
			break;
			case LMP_ENCRYPTION_KEY_SIZE_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s ENC_KEY_SIZE: %02x\n"), prefix_string, buffer[buffer_index]));
			}
			break;
			case LMP_START_ENCRYPTION_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s ENC_RANDOM_NUM:"), prefix_string));
				for(i = 0; i < 16; i++)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_STOP_ENCRYPTION_REQ:
			{
			}
			break;
			case LMP_SWITCH_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s SWITCH INSTANT: %08x\n"), prefix_string, *(uint32 *)&buffer[buffer_index]));
			}
			break;
			case LMP_HOLD_PKT:
			{
			}
			case LMP_HOLD_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s HOLD TIME %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s HOLD TIME %08x\n"), prefix_string, *(uint32 *)&buffer[buffer_index]));
			}
			break;
			case LMP_SNIFF_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s TIMING CONTROL FLAGS %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Dsniff %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Tsniff %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s SNIFF ATTEMPT %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s SNIFF TIMEOUT %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
			}
			break;
			case LMP_UNSNIFF_REQ:
			{
			}
			break;
			case LMP_PARK_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s TIMING CONTROL FLAGS %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Db %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Tb %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Nb %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s DELTAb %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s PM ADDR %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s AR ADDR %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Nbsleep %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Dbsleep %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Daccess %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Taccess %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Nacc slots %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Npoll %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Maccess %01x\n"), prefix_string, (buffer[buffer_index] & 0xF)));
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s ACCESS SCHEME %01x\n"), prefix_string, (buffer[buffer_index] >> 4)));
			}
			break;
			case LMP_SET_BROADCAST_SCAN_WINDOW:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s TIMING CONTROL FLAGS %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Db %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Broadcast scan window %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
			}
			break;
			case LMP_MODIFY_BEACON:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s TIMING CONTROL FLAGS %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Db %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Tb %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Nb %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s DELTAb %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Daccess %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Taccess %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Nacc slots %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Npoll %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Maccess %01x\n"), prefix_string, (buffer[buffer_index] & 0xF)));
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s ACCESS SCHEME %01x\n"), prefix_string, (buffer[buffer_index] >> 4)));
			}
			break;
			case LMP_UNPARK_BD_ADDR_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s TIMING CONTROL FLAGS %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Db %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LT ADDR FIRST UNPARK %01x\n"), prefix_string, (buffer[buffer_index] & 0x7)));
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LT ADDR SECOND UNPARK %01x\n"), prefix_string, ((buffer[buffer_index] & 0x70) >> 4)));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s BD ADDR FIRST UNPARK %01x\n"), prefix_string, (buffer[buffer_index] & 0x7)));
				for(i = 0; i < 6; i++)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s BD ADDR SECOND UNPARK %01x\n"), prefix_string, ((buffer[buffer_index] & 0x70) >> 4)));
				for(i = 0; i < 6; i++)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_UNPARK_PM_ADDR_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s TIMING CONTROL FLAGS %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s Db %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LT ADDR FIRST UNPARK %01x\n"), prefix_string, (buffer[buffer_index] & 0x7)));
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LT ADDR SECOND UNPARK %01x\n"), prefix_string, ((buffer[buffer_index] & 0x70) >> 4)));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s PM ADDR FIRST UNPARK %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s PM ADDR SECOND UNPARK %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LT ADDR THIRD UNPARK %01x\n"), prefix_string, (buffer[buffer_index] & 0x7)));
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LT ADDR FOURTH UNPARK %01x\n"), prefix_string, ((buffer[buffer_index] & 0x70) >> 4)));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s PM ADDR THIRD UNPARK %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s PM ADDR FOURTH UNPARK %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LT ADDR FIFTH UNPARK %01x\n"), prefix_string, (buffer[buffer_index] & 0x7)));
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LT ADDR SIXTH UNPARK %01x\n"), prefix_string, ((buffer[buffer_index] & 0x70) >> 4)));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s PM ADDR FIFTH UNPARK %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s PM ADDR SIXTH UNPARK %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LT ADDR SEVENTH UNPARK %01x\n"), prefix_string, (buffer[buffer_index] & 0x7)));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s PM ADDR SEVENTH UNPARK %02x\n"), prefix_string, buffer[buffer_index]));
			}
			break;
			case LMP_INCR_POWER_REQ:
			{
			}
			case LMP_DECR_POWER_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s FOR FUTURE USE %02x\n"), prefix_string, buffer[buffer_index]));
			}
			break;
			case LMP_MAX_POWER:
			{
			}
			case LMP_MIN_POWER:
			{
			}
			break;
			case LMP_AUTO_RATE:
			{
			}
			break;
			case LMP_PREFERRED_RATE:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s PREFERRED_RATE: %02x\n"), prefix_string, buffer[buffer_index]));
			}
			break;
			case LMP_VERSION_REQ:
			{
				/* Same as LMP_VERSION_RES */
			}
			case LMP_VERSION_RES:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s VERSION_NUM: %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s COMP_ID: %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s SUB_VERSION_NUM: %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
			}
			break;
			case LMP_FEATURES_REQ:
			{
				/* Same as LMP_FEATURES_RES */
			}
			case LMP_FEATURES_RES:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s FEATURES_BITMAP:"), prefix_string));
				for(i = 0; i < 8; i++)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_QUALITY_OF_SERVICE:
			{
				/* Same as LMP_QUALITY_OF_SERVICE_REQ */
			}
			case LMP_QUALITY_OF_SERVICE_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s POLL_INTERVAL: %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s NBR: %02x\n"), prefix_string, buffer[buffer_index]));
			}
			break;
			case LMP_SCO_LINK_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_REMOVE_SCO_LINK_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_MAX_SLOT:
			{
				/* Same as LMP_MAX_SLOT_REQ */
			}
			case LMP_MAX_SLOT_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s MAX_NUM_SLOTS: %02x\n"), prefix_string, buffer[buffer_index]));
			}
			break;
			case LMP_TIMING_ACCURACY_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_TIMING_ACCURACY_RES:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s DRIFT: %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s JITTER: %02x\n"), prefix_string, buffer[buffer_index]));
			}
			break;
			case LMP_SETUP_COMPLETE:
			{
			}
			break;
			case LMP_USE_SEMI_PERMANENT_KEY:
			{
			}
			break;
			case LMP_HOST_CONNECTION_REQ:
			{
			}
			break;
			case LMP_SLOT_OFFSET:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s SLOT_OFFSET: 0x%04x = %d\n"), prefix_string, *(uint16 *)&buffer[buffer_index], *(uint16 *)&buffer[buffer_index]));
				buffer_index += 2;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s BD_ADDR: %02x:%02x:%02x:%02x:%02x:%02x\n"), prefix_string, buffer[buffer_index], buffer[buffer_index + 1], buffer[buffer_index + 2], buffer[buffer_index + 3], buffer[buffer_index + 4], buffer[buffer_index + 5]));
			}
			break;
			case LMP_PAGE_MODE_REQ:
			{
				/* Same as LMP_PAGE_SCAN_MODE_REQ */
			}
			case LMP_PAGE_SCAN_MODE_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s PAGING_SCHEME: %02x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s PAGING_SCHEME_SETTINGS: %02x\n"), prefix_string, buffer[buffer_index]));
			}
			break;
			case LMP_SUPERVISION_TIMEOUT:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s LINK_SUPERVISION_TOUT: %04x\n"), prefix_string, *(uint16 *)&buffer[buffer_index]));
			}
			break;
			case LMP_TEST_ACTIVATE:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_TEST_CONTROL:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_ENCRYPTION_KEYSIZE_MASK_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_ENCRYPTION_KEYSIZE_MASK_RES:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_SET_AFH:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s AFH_INSTANT: %08x\n"), prefix_string, *(uint32 *)&buffer[buffer_index]));
				buffer_index += 4;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s AFH_MODE: %x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s AFH_CHANNEL_MAP:"), prefix_string));
				for(i = 0; i < 10; i++)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_ENCAPSULATED_HEADER:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_ENCAPSULATED_PAYLOAD:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_SIMPLE_PAIRING_CONFIRM:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_SIMPLE_PAIRING_NUMBER:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_DHKEY_CHECK:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			default:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s Unknown LMP packet, opcode = %d\n"), prefix_string, lmp_opcode));
			}
			break;
		}
	}
	else
	{
		lmp_opcode = buffer[buffer_index];
		buffer_index++;
		
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s %s\n"), prefix_string, lmp_ext_opcodes[lmp_opcode]));
		switch(lmp_opcode)
		{
			case LMP_ACCEPTED_EXT:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s REQ_OPCODE: %s \n"), prefix_string, lmp_opcodes[buffer[buffer_index]]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s REQ_EXT_OPCODE: %s \n"), prefix_string, lmp_ext_opcodes[buffer[buffer_index]]));
			}
			break;
			case LMP_NOT_ACCEPTED_EXT:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s REQ_OPCODE: %s \n"), prefix_string, lmp_opcodes[buffer[buffer_index]]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s REQ_EXT_OPCODE: %s \n"), prefix_string, lmp_ext_opcodes[buffer[buffer_index]]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s REJECTION_REASON: %02x \n"), prefix_string, buffer[buffer_index]));//create an array for error codes?
			}
			break;
			case LMP_FEATURES_REQ_EXT:
			{
				/* Same as LMP_FEATURES_RES_EXT */
			}
			case LMP_FEATURES_RES_EXT:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s FEATURES_PAGE: %x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s MAX_SUPPORTED_FEATURES_PAGE: %x\n"), prefix_string, buffer[buffer_index]));
				buffer_index++;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s EXT_FEATURES_BITMAP: "), prefix_string));
				for(i = 0; i < 8; i++)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), buffer[buffer_index]));
					buffer_index++;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
			case LMP_PACKET_TYPE_TABLE_REQ:
			{
				if(buffer[buffer_index])
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s DATA_RATE: EDR RATE\n"), prefix_string));
				}
				else
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s DATA_RATE: BR RATE\n"), prefix_string));
				}
			}
			break;
			case LMP_ESCO_LINK_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_REMOVE_eSCO_LINK_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_CHANNEL_CLASSIFICATION_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_CHANNEL_CLASSIFICATION:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_SNIFF_SUBRATING_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_SNIFF_SUBRATING_RES:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_PAUSE_ENCRYPTION_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_RESUME_ENCRYPTION_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_IO_CAPABILITIES_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_IO_CAPABILITIES_RES:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_NUMERIC_COMPARISON_FAILED:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_PASSKEY_FAILED:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_OOB_FAILED:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_KEYPRESS_NOTIFICATION:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_POWER_CONTROL_REQ:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			case LMP_POWER_CONTROL_RES:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s \n"), prefix_string));
			}
			break;
			default:
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s Unknown EXT LMP packet, opcode = %d\n"), prefix_string, lmp_opcode));
			}
			break;
		}
	}
}


static ONEBOX_STATUS print_hci_event_type(BT_ADAPTER bt_adapter, uint8 *buffer)
{
	uint8 pkt_type = buffer[14];

	switch (pkt_type)
	{
		case HCI_ACLDATA_PKT:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ACL DATA PACKET\n")));
			break;
		}
		case HCI_SCODATA_PKT:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("SCO DATA PACKET\n")));
			break;
		}
		case HCI_EVENT_PKT:
		{
			uint8 event_type = buffer[16];

			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s"), hci_events[event_type]));
			if(event_type == HCI_EV_CMD_COMPLETE)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" - ")));
				print_rx_hci_cmd_type(bt_adapter, *(uint16 *)&buffer[19], HCI_COMMAND_PKT);
			}
			else
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			}
			break;
		}
		case HCI_VENDOR_PKT:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("VENDOR PACKET\n")));
			{
				uint16 pkt_buff_index = 16;
				uint16 vendor_pkt_type;
				uint16 pkt_debug_msg_len;

				vendor_pkt_type = *(uint16 *)&buffer[pkt_buff_index];
				pkt_buff_index += 2;
				switch(vendor_pkt_type)
				{
					case LMP_PKT_DEBUG_MSG:
					{
						pkt_debug_msg_len = *(uint16 *)&buffer[pkt_buff_index];
						pkt_buff_index += 2;

						print_lmp_pkt_details(&buffer[pkt_buff_index], pkt_debug_msg_len);

						return ONEBOX_DEBUG_MESG_RECVD;
					}
					break;
					case LLP_PKT_DEBUG_MSG:
					{
						pkt_debug_msg_len = *(uint16 *)&buffer[pkt_buff_index];
						pkt_buff_index += 2;

						//print_llp_pkt_details(&buffer[pkt_buff_index], pkt_debug_msg_len);

						return ONEBOX_DEBUG_MESG_RECVD;
					}
					break;
					default:
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unknown VENDOR PACKET, type = 0x%04x\n"), vendor_pkt_type));
					}
					break;
				}
			}
			break;
		}
		default:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("UNKNOWN PACKET TYPE %x\n"),pkt_type));
			break;
		}
	}
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function prepares the netbuf control block
 *
 * @param 
 * bt_adapter pointer to the driver private structure
 * @param 
 * buffer pointer to the packet data
 * @param 
 * len length of the packet 
 * @return .This function returns ONEBOX_STATUS_SUCCESS.
 */
int prepare_netbuf_cb(BT_ADAPTER bt_adapter, uint8 *buffer,
		                    uint32 pkt_len)
{
	netbuf_ctrl_block_t *netbuf_cb;

	bt_adapter->stats.rx_packets++;

	{
		/* Receive packet dump */
		//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("\nRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRX\n")));
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("RX PACKET LENGTH = %d\n"), pkt_len+16));
		if(print_hci_event_type(bt_adapter, buffer))
		{
			bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO, buffer, pkt_len+16);
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\nRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRX\n")));
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n***************************************************\n")));
			/* this packet need not to be sent to upper layers and is debug message packet */
			return ONEBOX_STATUS_SUCCESS;
		}
		bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO, buffer, pkt_len+16);
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\nRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRX\n")));
	}

	netbuf_cb = bt_adapter->os_intf_ops->onebox_alloc_skb(pkt_len);

	if(netbuf_cb != NULL)
	{
		/* Assigning packet type */
		netbuf_cb->bt_pkt_type = buffer[14];
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to allocate skb\n")));
		return ONEBOX_STATUS_FAILURE;
	}
	/* Preparing The netbuf_cb To Indicate To core */
	bt_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);

	/* Copy the packet to into netbuf_cb */
	/*
	 * netbuf_cb->data contains extend_desc + payload
	 */ 
	bt_adapter->os_intf_ops->onebox_memcpy((VOID *)netbuf_cb->data, (VOID *)(buffer + FRAME_DESC_SZ), pkt_len);

	bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO, netbuf_cb->data, netbuf_cb->len);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("@@@ Indicating packet to core:\n")));
	bt_adapter->osi_bt_ops->onebox_indicate_pkt_to_core(bt_adapter, netbuf_cb);
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function receives the management packets from the hardware and process them
 *
 * @param 
 *  bt_adapter  Pointer to driver private structure
 * @param 
 *  msg      Received packet
 * @param 
 *  len      Length of the received packet
 *
 * @returns 
 *  ONEBOX_STATUS_SUCCESS on success, or corresponding negative
 *  error code on failure
 */
int32 onebox_bt_mgmt_pkt_recv(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb)
{
	int32 msg_type;
	int32 msg_len;
	int32 total_len;
	uint8 *msg;
	int8 ret;
  	uint16 bt_ber_pkt_len;
	//EEPROM_READ read_buf;

	FUNCTION_ENTRY(ONEBOX_ZONE_MGMT_RCV);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("In %s function inside file mgmt.c\n"),__func__));

	msg =  netbuf_cb->data;
	msg_len = ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[0]) & 0xFFF;
  	bt_ber_pkt_len = msg_len;
	msg_type = msg[14] & 0xFF;
	total_len = msg_len + 16;
	ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_RCV,(TEXT("Rcvd Mgmt Pkt Len = %d\n"), total_len));
	bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_MGMT_DUMP, msg, total_len);

	ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_RCV,
	             (TEXT("Msg Len: %d, Msg Type: %#x\n"), msg_len, msg_type));

	switch (bt_adapter->fsm_state)
	{
#if 0
		case FSM_CARD_NOT_READY: 
		{
      			if (msg_type == MGMT_DESC_TYP_CARD_RDY_IND)
      			{
        			ONEBOX_DEBUG(ONEBOX_ZONE_FSM,(TEXT("onebox_mgmt_fsm: CARD_READY\n")));
				bt_adapter->fsm_state = FSM_DEVICE_READY;
				bt_adapter->osi_bt_ops->onebox_core_init(bt_adapter);
		  	}
		}
		break;
#endif
		case FSM_DEVICE_READY:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Inside case FSM_DEVICE_READY\n")));
			switch (msg_type)
			{
				case RESULT_CONFIRM: 
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_RCV,
					             (TEXT("Rcvd Result Confirm  Msg_type: %x\n"),msg[3]));
				}
				break;
				case STATION_STATISTICS_RESP:
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_RCV,
					             (TEXT("Rcvd Station Statics Response\n")));
				}
        		break;
        		case BT_PER:
		        {
		          ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		              (TEXT("Rcvd BT_PER CNFM msg_len = %d\n"),msg_len));

		          bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO,msg, msg_len);
		          if(bt_adapter->read_cmd || bt_adapter->wait_cmd)
		          {
		            bt_adapter->os_intf_ops->onebox_set_event(&(bt_adapter->bt_per_event));
		            if( bt_adapter->read_cmd )
		            {
		              bt_core_pkt_recv(bt_adapter, netbuf_cb);
		              ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" Sending PKT to HOST\n")));
#if 0
		              bt_adapter->os_intf_ops->onebox_memset(&bt_adapter->bb_rf_params, 0,sizeof(bt_adapter->bb_rf_params));
		              bt_adapter->os_intf_ops->onebox_memcpy(&bt_adapter->bb_rf_params.Data[0], &msg[16], msg_len * 2);
		              bt_adapter->os_intf_ops->onebox_set_event(&(bt_adapter->bt_per_event));
#endif
		            }
		          }
		        }
				break;
        		case BT_BER:
		        {
		          if(bt_adapter->bt_ber_pkts)
		          {	
		            bt_ber_params_t *param = &(bt_adapter->ber_info);
		            uint16 *framebody_bt;
		            framebody_bt = (uint16 *)&msg[16];

		            ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("==>>> RECEIVED ADAPTER_COUNT: %d   ==> BER_MODE \n"),bt_adapter->bt_ber_pkts));

		            if(param->ber_pkts[param->push_loc].data != NULL) 
		            {
		              bt_adapter->pkt_drop_cnt++;
		              ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("====>>PKT DROPPED COUNT DUE TO QUEUE FULL<<====")));
		              ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("pkt dropped count due to queue full= %d\n"), bt_adapter->pkt_drop_cnt));
		              //bt_adapter->os_intf_ops->onebox_release_spinlock(&bt_adapter->lock_rx, 0);
									bt_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
		              return 0;
		            }
		            //bt_adapter->os_intf_ops->onebox_mem_alloc((PVOID *)param->ber_pkts[param->push_loc].data,1035, GFP_ATOMIC);
		            bt_adapter->os_intf_ops->onebox_mem_alloc((PVOID *)&bt_adapter->ber_packet,1035, GFP_ATOMIC);
		            bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)framebody_bt, bt_ber_pkt_len);

		            //bt_adapter->os_intf_ops->onebox_memcpy(param->ber_pkts[param->push_loc].data, framebody_bt,bt_ber_pkt_len);
		            bt_adapter->os_intf_ops->onebox_memcpy(&bt_adapter->ber_packet[0], framebody_bt, bt_ber_pkt_len);
		            param->ber_pkts[param->push_loc].data = bt_adapter->ber_packet;

		            //param->ber_pkts[param->push_loc].len = bt_ber_pkt_len - 16;
		            param->ber_pkts[param->push_loc].len = bt_ber_pkt_len;
		            param->push_loc = ((param->push_loc + 1) % 256);

		            param->num_pkts_avail++;
		            ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("====>>>>>>> 0x%02x\n"),bt_ber_pkt_len ));

		            bt_adapter->bt_ber_pkts--;
		            //bt_adapter->os_intf_ops->onebox_release_spinlock(&bt_adapter->lock_rx, 0);
		          }
		          else
		          {
		          }
		        }
        		break;
				default:
				{
					/* Forward the frames to mgmt sw module */
					ret = onebox_mgmt_pkt_to_core(bt_adapter, netbuf_cb, total_len, 1);
					/* skb is freed in the bluez stack, here we are freeing netbuf_cb. */
					kfree(netbuf_cb);
					return ret;
				}
				break;
			} /* End switch (msg_type) */
			break; 
		}
		default:
		{
			bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INIT, msg, total_len);
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			             (TEXT("Invalid FSM State %d\n"), bt_adapter->fsm_state));
		} 
		break;
	} /* End switch (bt_adapter->fsm_state) */

	bt_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
	FUNCTION_EXIT(ONEBOX_ZONE_MGMT_RCV);
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * Entry point of Management module
 *
 * @param
 *  bt_adapter    Pointer to driver private data
 * @param
 *  msg    Buffer recieved from the device
 * @param
 *  msg_len    Length of the buffer
 * @return
 *  Status
 */
int onebox_mgmt_pkt_to_core(BT_ADAPTER bt_adapter,
                          netbuf_ctrl_block_t *netbuf_cb,
                          int32 msg_len,
                          uint8 type)
{
	uint8 *msg;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("+%s"),__func__));

	FUNCTION_ENTRY(ONEBOX_ZONE_MGMT_RCV);

	msg = netbuf_cb->data;
	bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_MGMT_DUMP, msg, msg_len);
	if (type)
	{
		msg_len -= FRAME_DESC_SZ;
		if ((msg_len <= 0) || (!msg))
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND, 
			             (TEXT("Invalid incoming message of message length = %d\n"), msg_len));
			bt_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
			return ONEBOX_STATUS_FAILURE;
		}
		
#ifdef USE_BLUEZ_BT_STACK
		if( bt_adapter->driver_mode == BT_E2E_MODE_ON )
		{
			bt_adapter->os_intf_ops->onebox_netbuf_adj(netbuf_cb, FRAME_DESC_SZ);
		}
#endif

		bt_adapter->osi_bt_ops->onebox_indicate_pkt_to_core(bt_adapter, netbuf_cb);
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND,(TEXT ("Internal Packet\n")));
	}

	FUNCTION_EXIT(ONEBOX_ZONE_MGMT_RCV);
	return 0;
}

/**
 * This function read frames from the SD card.
 *
 * @param  Pointer to driver bt_adapter structure.  
 * @param  Pointer to received packet.  
 * @param  Pointer to length of the received packet.  
 * @return 0 if success else -1. 
 */
ONEBOX_STATUS bt_read_pkt(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb)
{
  uint32 queueno;
  uint8 extended_desc;
  uint8 *frame_desc_addr = netbuf_cb->data;
  uint32 length = 0;
  uint16 offset =0;

  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("bt_adapter = %p\n"),
        bt_adapter));

  queueno = (uint32)((*(uint16 *)&frame_desc_addr[offset] & 0x7000) >> 12);
  length   = (*(uint16 *)&frame_desc_addr[offset] & 0x0fff);
  extended_desc   = (*(uint8 *)&frame_desc_addr[offset + 4] & 0x00ff);
  ONEBOX_DEBUG(ONEBOX_ZONE_DATA_RCV, (TEXT("###Received in QNumber:%d Len=%d###!!!\n"), queueno, length));

  switch(queueno)
  {
    case BT_DATA_Q:      
      {
        if (length > (ONEBOX_RCV_BUFFER_LEN * 4 ))
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
              (TEXT("%s: Toooo big packet %d\n"), __func__, length));	    
          bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_DATA_RCV, frame_desc_addr, FRAME_DESC_SZ);
          length = ONEBOX_RCV_BUFFER_LEN * 4 ;
        }
        if (length < 3)
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
              (TEXT("%s: Too small packet %d\n"), __func__, length));
          bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_DATA_RCV, frame_desc_addr, FRAME_DESC_SZ);
        }

        if (!length)
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Pkt size is zero\n"), __func__));
          bt_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb,0);
          return ONEBOX_STATUS_FAILURE;
        }

        bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_DATA_RCV, frame_desc_addr, length + FRAME_DESC_SZ);
        netbuf_cb->bt_pkt_type = frame_desc_addr[14];
        bt_adapter->stats.rx_packets++;

        {
          /* Receive packet dump */
          //ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("\nRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRX\n")));
          ONEBOX_DEBUG(ONEBOX_ZONE_DATA_RCV, (TEXT("RX PACKET LENGTH = %d\n"), length+16));
          if(print_hci_event_type(bt_adapter, netbuf_cb->data))
          {
            bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_DATA_RCV, netbuf_cb->data, length+16);
            ONEBOX_DEBUG(ONEBOX_ZONE_DATA_RCV, (TEXT("\nRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRX\n")));
            ONEBOX_DEBUG(ONEBOX_ZONE_DATA_RCV, (TEXT("\n***************************************************\n")));
            /* this packet need not to be sent to upper layers and is debug message packet */
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s: Freeing Internal HCI event from device packet\n"), __func__));
						bt_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
            return ONEBOX_STATUS_SUCCESS;
          }
          bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_DATA_RCV, netbuf_cb->data, length+16);
          ONEBOX_DEBUG(ONEBOX_ZONE_DATA_RCV, (TEXT("\nRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRXRX\n")));
        }
        /*
         * frame_desc_addr contains FRAME DESC + extended DESC + payload
         * lenght = extended descriptor size + payload size 
         */
#ifdef USE_BLUEZ_BT_STACK
        if( bt_adapter->driver_mode == BT_E2E_MODE_ON )
        {
        	bt_adapter->os_intf_ops->onebox_netbuf_adj(netbuf_cb, FRAME_DESC_SZ);
        }
#endif
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("@@@ Indicating packet to core:\n")));
        bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO, netbuf_cb->data, netbuf_cb->len);
        bt_adapter->osi_bt_ops->onebox_indicate_pkt_to_core(bt_adapter, netbuf_cb);
      }
      break;
      /*
         case RCV_LMAC_MGMT_Q:
         case RCV_TA_MGMT_Q1:
         case RCV_TA_MGMT_Q:
         */
    case BT_INT_MGMT_Q:
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("*****received qno is %d********\n"),queueno));
        onebox_bt_mgmt_pkt_recv(bt_adapter, netbuf_cb);
      } 
      break;

    default:
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: pkt from invalid queue\n"), __func__));
        bt_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
      }
      break;
  } /* End switch */      


  return ONEBOX_STATUS_SUCCESS;
}
EXPORT_SYMBOL(bt_read_pkt);
