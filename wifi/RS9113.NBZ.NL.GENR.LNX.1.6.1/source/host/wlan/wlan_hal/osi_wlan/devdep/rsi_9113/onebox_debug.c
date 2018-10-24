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

#include "wlan_common.h"

#include "onebox_hal.h"
#include "onebox_linux.h"
#include "onebox_wlan_pktpro.h"

//void onebox_print_mac_address(WLAN_ADAPTER w_adapter, uint8 mac_addr[ETHER_ADDR_LEN])
void onebox_print_mac_address(WLAN_ADAPTER w_adapter, uint8 *mac_addr)
{

		uint32 ii = 0;

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\nThe MAc addess is ")));
		for(; ii < ETHER_ADDR_LEN; ii++) {
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%02x "), mac_addr[ii]));
		}
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("\n")));

}

ONEBOX_STATUS onebox_send_debug_frame(WLAN_ADAPTER w_adapter, struct test_mode *test)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[FRAME_DESC_SZ];

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

	/* Bit{0:11} indicates length of the Packet
 	 * Bit{12:16} indicates host queue number
 	 */ 

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12);
	/* Fill frame type for debug frame */
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(DEBUG_FRAME);
	/* Fill data in form of flags*/
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(test->subtype);
	mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(test->args);

	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, FRAME_DESC_SZ);
	return status;
	
}

ONEBOX_STATUS onebox_do_master_ops(WLAN_ADAPTER w_adapter,struct master_params_s *master ,uint16 type)
{
	int status = 0;
	int i;
	uint8 *data;
	uint8  temp_buf[1000];
	uint16 msb_address = (master->address >> 16);
	uint16 lsb_address = (uint16)master->address;
	struct master_params_s master_get; 

	struct driver_assets *d_assets = w_adapter->d_assets;

	if(copy_from_user(&master_get, master, sizeof(struct master_params_s)))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
		return -EINVAL;
	}

	msb_address = (master_get.address >> 16);
	lsb_address = (uint16)master_get.address;

	//: Make use of spin locks before doing master reads/write's.

	//unsigned long flags = 0;
	//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s :Aquireing lock\n", __func__)));
	//w_adapter->os_intf_ops->onebox_acquire_spinlock(&w_adapter->master_ops_lock, flags);
	if (d_assets->host_intf_type == HOST_INTF_SDIO) {

		if (onebox_wlan_sdio_master_access_msword(w_adapter, msb_address) != ONEBOX_STATUS_SUCCESS)      
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,	(TEXT("%s: Unable to set ms word reg\n"), __func__));

			goto end_master_ops;
		}
	}

	if(type == ONEBOX_MASTER_READ)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s line %d master_read \n"), __func__, __LINE__));
		data = kmalloc(master_get.no_of_bytes, GFP_KERNEL);
		if (d_assets->host_intf_type == HOST_INTF_SDIO) {
			status = d_assets->onebox_common_read_multiple(d_assets,
					lsb_address | SD_REQUEST_MASTER,
					master_get.no_of_bytes,
					(uint8 *)data);
		} else {
			status = d_assets->onebox_common_ta_read_multiple(d_assets,
					master_get.address,
					(uint8 *)data,
					master_get.no_of_bytes);
		}
		/*: mimic copy to user  functionality here*/
#if 1
		if(copy_to_user(master_get.data, data, master_get.no_of_bytes))
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
			status = ONEBOX_STATUS_FAILURE;
			goto end_master_ops;
		}
#endif
		kfree(data);
	}
	else if(type == ONEBOX_MASTER_WRITE)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s line %d master_write\n"), __func__, __LINE__));
		master_get.data = kmalloc(master_get.no_of_bytes, GFP_KERNEL);
		if(master_get.data == NULL) {
			status = ONEBOX_STATUS_FAILURE;
			goto end_master_ops;
		}
		if(copy_from_user(master_get.data, master->data, master_get.no_of_bytes))
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
			kfree(master_get.data);
			status = ONEBOX_STATUS_FAILURE;
			goto end_master_ops;
		}
		for(i = 0; i < master_get.no_of_bytes; i++)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" Data to write = 0x%x \n"), master_get.data[i]));
		}

		if (d_assets->host_intf_type == HOST_INTF_SDIO) {
			w_adapter->os_intf_ops->onebox_memset(temp_buf, 0, master_get.no_of_bytes);
			w_adapter->os_intf_ops->onebox_memcpy(temp_buf, (uint8 *)master_get.data, master_get.no_of_bytes);
			status = d_assets->onebox_common_write_multiple(d_assets,
					lsb_address | SD_REQUEST_MASTER,
					temp_buf, 
					master_get.no_of_bytes, NULL);
		} else {
			status = d_assets->onebox_common_ta_write_multiple(d_assets,
					master_get.address,
					(uint8 *)master_get.data,
					master_get.no_of_bytes);
		}
		kfree(master_get.data);
	}

end_master_ops:
	if (d_assets->host_intf_type == HOST_INTF_SDIO) {
		if (onebox_wlan_sdio_master_access_msword(w_adapter, 0x4105) != ONEBOX_STATUS_SUCCESS)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,	(TEXT("%s: Unable to set ms word to common reg\n"), __func__));
			status = ONEBOX_STATUS_FAILURE;

		}
	}
	return status;
}

void dump_debug_frame(uint8 *msg, int32 msg_len)
{

		uint32 *debug;
		msg = msg +16;

		debug = (uint32 *)msg;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, 
					 (TEXT("In %s Line %d Printing the debug frame contents\n"), __func__, __LINE__));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		             (TEXT("bg_state : %02x \n"), debug[0]));
		
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		             (TEXT("bgscan_flags : %02x \n"), (debug[1] >> 16)));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		             (TEXT("bgscan_nextchannel : %02x \n"), (*(uint16 *)&debug[1])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		             (TEXT("TX buffers : %02x \n"), (debug[2] >> 16)));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		             (TEXT("RX buffers : %02x \n"), (*(uint16 *)&debug[2])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		             (TEXT("Glbl buffers : %02x \n"), (debug[3])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		             (TEXT("Tx Hostq_pkt_cnt : %02x \n"), (debug[4])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("rx Hostq_pkt_cnt : %02x \n"), (debug[5])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("BBP_CTRL_REG : %02x \n"), (debug[6])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("ED REG : %02x \n"), (debug[7])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("PPE PC : %02x \n"), (debug[8])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("PPE PC : %02x \n"), (debug[9])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("PPE PC : %02x \n"), (debug[10])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("PPE PC : %02x \n"), (debug[11])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_0 lmac_state  : %02x QUEUE_1 lmac_state  : %02x \n"), (debug[12] >> 16), ONEBOX_CPU_TO_LE16(*(uint16 *)&debug[12])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_2 lmac_state  : %02x QUEUE_3 lmac_state  : %02x \n"), (debug[13] >> 16), ONEBOX_CPU_TO_LE16(*(uint16 *)&debug[13])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_4 lmac_state  : %02x QUEUE_5 lmac_state  : %02x \n"), (debug[14] >> 16), ONEBOX_CPU_TO_LE16(*(uint16 *)&debug[14])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_6 lmac_state  : %02x QUEUE_7 lmac_state  : %02x \n"), (debug[15] >> 16), ONEBOX_CPU_TO_LE16(*(uint16 *)&debug[15])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_8 lmac_state  : %02x QUEUE_9 lmac_state  : %02x \n"), (debug[16] >> 16), ONEBOX_CPU_TO_LE16(*(uint16 *)&debug[16])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_10 lmac_state  : %02x QUEUE_11 lmac_state  : %02x \n"), (debug[17] >> 16), ONEBOX_CPU_TO_LE16(*(uint16 *)&debug[17])));


		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_0 SW_q.pkt_cnt  : %02x QUEUE_0 dot11_q.pkt_cnt  : %02x \n"), debug[18] , debug[19]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_1 SW_q.pkt_cnt  : %02x QUEUE_1 dot11_q.pkt_cnt  : %02x \n"), debug[20] , debug[21]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_2 SW_q.pkt_cnt  : %02x QUEUE_2 dot11_q.pkt_cnt  : %02x \n"), debug[22] , debug[23]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_3 SW_q.pkt_cnt  : %02x QUEUE_3 dot11_q.pkt_cnt  : %02x \n"), debug[24] , debug[25]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_4 SW_q.pkt_cnt  : %02x QUEUE_4 dot11_q.pkt_cnt  : %02x \n"), debug[26] , debug[27]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_5 SW_q.pkt_cnt  : %02x QUEUE_5 dot11_q.pkt_cnt  : %02x \n"), debug[28] , debug[29]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_6 SW_q.pkt_cnt  : %02x QUEUE_6 dot11_q.pkt_cnt  : %02x \n"), debug[30] , debug[31]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_7 SW_q.pkt_cnt  : %02x QUEUE_7 dot11_q.pkt_cnt  : %02x \n"), debug[32] , debug[33]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_8 SW_q.pkt_cnt  : %02x QUEUE_8 dot11_q.pkt_cnt  : %02x \n"), debug[34] , debug[35]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_9 SW_q.pkt_cnt  : %02x QUEUE_9 dot11_q.pkt_cnt  : %02x \n"), debug[36] , debug[37]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_10 SW_q.pkt_cnt  : %02x QUEUE_10 dot11_q.pkt_cnt  : %02x \n"), debug[38] , debug[39]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("QUEUE_11 SW_q.pkt_cnt  : %02x QUEUE_11 dot11_q.pkt_cnt  : %02x \n"), debug[38] , debug[41]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("Lmac_abort stats delay  : %02x \n"), debug[42]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("Block_q_bitmap : %02x \n"), (debug[43] >> 16)));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("bgscan Block_q_bitmap : %02x \n"), ONEBOX_CPU_TO_LE16(*(uint16 *)&debug[43])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("host Block_q_bitmap : %02x  coex block_q_bitmap %02x \n"), (debug[44] >> 16), ONEBOX_CPU_TO_LE16(*(uint16 *)&debug[44])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("PS state: %02x  tx_pkt_ts %02x rx_pkt_ts %02x expected_dtim_tsf %02lx\n"), 
												(debug[45]),(debug[46]), debug[47], ONEBOX_CPU_TO_LE64(*(uint64 *)&debug[48])));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("scaheduler_info_g[0] th_int_event_map : %02x  th_int_mask_map1 : %02x th_int_map2 : %02x th_int_map1_poll : %02x \n")
										 , (debug[50]),(debug[51]), debug[52], debug[53]));
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("scaheduler_info_g[1] th_int_event_map : %02x  th_int_mask_map1 %02x th_int_map2 %02x th_int_map1_poll %02x \n")
										 , (debug[54]),(debug[55]), debug[56], debug[57]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("scaheduler_info_g[2] th_int_event_map : %02x  th_int_mask_map1 %02x th_int_map2 %02x th_int_map1_poll %02x \n")
										 , (debug[58]),(debug[59]), debug[60], debug[61]));

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
										(TEXT("scaheduler_info_g[3] th_int_event_map : %02x  th_int_mask_map1 %02x th_int_map2 %02x th_int_map1_poll %02x \n")
										 , (debug[62]),(debug[63]), debug[64], debug[65]));
		return;
}
