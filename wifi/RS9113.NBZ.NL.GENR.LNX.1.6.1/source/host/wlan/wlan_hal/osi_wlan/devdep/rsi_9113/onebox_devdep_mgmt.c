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

#include "wlan_common.h"
#include "onebox_per.h"
#include "onebox_qspi.h"
#include "onebox_bootup_params.h"
#include "onebox_mgmt.h"

/**
 * This function prepares the Baseband Programming request
 * frame and sends to the PPE
 *
 * @param
 *  w_adapter       Pointer to Driver Private Structure
 * @param
 *  bb_prog_vals  Pointer to bb programming values
 * @param
 *  num_of_vals   Number of BB Programming values
 *
 * @return
 *  Returns ONEBOX_STATUS_SUCCESS on success, or ONEBOX_STATUS_FAILURE
 *  on failure
 */


uint32_t uptime()
 {   
   struct timeval tv;
   do_gettimeofday(&tv);
   return tv.tv_usec;
 } 

static ONEBOX_STATUS onebox_mgmt_send_bb_prog_frame(WLAN_ADAPTER w_adapter,
                                             uint16 *bb_prog_vals,
                                             uint16 num_of_vals)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status_l;
	uint16 frame_len;
	uint16 count;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	uint8 ii = 0;

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
	             (TEXT("===> Sending Baseband Programming Packet <===\n")));

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);

	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)bb_prog_vals, num_of_vals);
	/* Preparing BB Request Frame Body */
	for (count=1; ((count < num_of_vals) && (ii< num_of_vals)); ii++, count+=2) 
	{
		mgmt_frame->u.bb_prog_req[ii].reg_addr = ONEBOX_CPU_TO_LE16(bb_prog_vals[count]);
		mgmt_frame->u.bb_prog_req[ii].bb_prog_vals = ONEBOX_CPU_TO_LE16(bb_prog_vals[count+1]);
	}

	if (num_of_vals % 2)
	{
		mgmt_frame->u.bb_prog_req[ii].reg_addr = ONEBOX_CPU_TO_LE16(bb_prog_vals[count]);
	}
	/* Preparing BB Request Frame Header */
	frame_len = ((num_of_vals) * 2);	//each 2 bytes
	
	/*prepare the frame descriptor */
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((frame_len) | (ONEBOX_WIFI_MGMT_Q << 12));
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BB_PROG_VALUES_REQUEST);

	if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_BEFORE_PROG)
	{
  		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_BEFORE_PROG);
	}	
	if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_AFTER_PROG)
	{
  		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_AFTER_PROG);	
	}	
	
	if (w_adapter->bb_rf_rw)
	{		
  		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REG_READ);
		w_adapter->bb_rf_rw = 0;
	}	
  	w_adapter->soft_reset = 0;
	
	//Flags are not handled :
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(num_of_vals);
#ifdef RF_8230
	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (NONRSI_RF_TYPE << 4));
#else
	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (RSI_RF_TYPE << 4));
#endif  
	//: What is the radio id to fill here
//	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (RADIO_ID << 8 );


	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_MGMT_DUMP, (PUCHAR)mgmt_frame, (frame_len + FRAME_DESC_SZ ));
	status_l = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (frame_len + FRAME_DESC_SZ));

	return status_l;
}

/**
 * This function prepares the RF Programming Request frame and sends to the PPE
 *
 * @param 
 *  w_adapter  Pointer to Driver Private Structure
 *  w_adapter  Pointer to RF programming values Structure
 *  w_adapter  number of programming values to be loaded
 *  w_adapter  number of values in a row
 *
 * @returns 
 *  ONEBOX_STATUS_SUCCESS on success, or corresponding negative
 *  error code on failure
 */
static ONEBOX_STATUS onebox_mgmt_send_rf_prog_frame(WLAN_ADAPTER w_adapter,
                                             uint16 *rf_prog_vals,
                                             uint16 num_of_sets,
                                             uint16 vals_per_set,
                                             uint8  type) 
{
	//: How are we taking care of band here ??
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status_l;
	uint16 frame_len;
	uint16 count;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

	FUNCTION_ENTRY(ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("===> Sending RF Programming Packet <===\n")));

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, 256);

	
	/* Preparing RF Request Frame Header */
	frame_len = (vals_per_set * num_of_sets * 2);
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(frame_len | (ONEBOX_WIFI_MGMT_Q << 12));
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(RF_PROG_VALUES_REQUEST);
  
	if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_BEFORE_PROG)
	{
  		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_BEFORE_PROG);
	}	
	if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_AFTER_PROG)
	{
  		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_AFTER_PROG);	
	}	
	
	if (w_adapter->bb_rf_rw)
	{		
  		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REG_READ);
		w_adapter->bb_rf_rw = 0;
	}	
  	w_adapter->soft_reset = 0;

	if (w_adapter->Driver_Mode == RF_EVAL_MODE_ON)
	{
		if (w_adapter->bb_rf_params.value == 4 || w_adapter->bb_rf_params.value == 5)
		{
			mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16(ULP_MODE);  //indicating ULP 
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ULP \n")));
		}  
	}
	//Flags are not handled :
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(vals_per_set | (num_of_sets << 8));
#ifdef RF_8230
	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (NONRSI_RF_TYPE << 4));
#else
	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (RSI_RF_TYPE << 4));
#endif  
	if(w_adapter->rf_reset)
	{
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (RF_RESET_ENABLE);
		ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("===> RF RESET REQUEST SENT <===\n")));
		w_adapter->rf_reset = 0;
	}
  //: What is the radio id to fill here
//	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (RADIO_ID << 8 );

	/* Preparing RF Request Frame Body */

	for (count = 0; count < (vals_per_set * num_of_sets); count++)
	{
		mgmt_frame->u.rf_prog_req.rf_prog_vals[count] = ONEBOX_CPU_TO_LE16(rf_prog_vals[count]);
	}


	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_MGMT_DUMP,(PUCHAR)mgmt_frame, (frame_len + FRAME_DESC_SZ ));
	status_l = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (frame_len + FRAME_DESC_SZ));
	return status_l;
} /* onebox_mgmt_send_rf_prog_frame */


/**
 * This function prepares the Baseband buffer Programming request
 * frame 
 *
 * @param
 *  w_adapter       Pointer to Driver Private Structure
 * @param
 *  bb_buf_vals  Pointer to bb_buf programming values
 * @param
 *  num_of_vals   Number of BB Programming values
 *
 * @return
 *  Returns ONEBOX_STATUS_SUCCESS on success, or ONEBOX_STATUS_FAILURE
 *  on failure
 */
static ONEBOX_STATUS onebox_bb_buffer_request_direct(WLAN_ADAPTER w_adapter,
                                             uint16 *bb_buf_vals,
                                             uint16 num_of_vals)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status_l;
	uint16 frame_len;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("===> Sending Buffer Programming Packet DIRECT<===\n")));

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);
	if(!w_adapter->bb_rf_rw)
	{
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)bb_buf_vals, ((num_of_vals*2) + 2));
		/* Preparing BB BUFFER Request Frame Body */

		frame_len = ((num_of_vals * 2));	
		w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bb_buf_req.bb_buf_vals[0], &bb_buf_vals[1], (frame_len));
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)&mgmt_frame->u.bb_buf_req.bb_buf_vals[0], (frame_len));

		/* Preparing BB Request Frame Header */

		/*prepare the frame descriptor */
		mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((frame_len) | (ONEBOX_WIFI_MGMT_Q << 12));
		mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BB_BUF_PROG_VALUES_REQ);
		mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(1);

		if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_BEFORE_PROG)
		{
			mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_BEFORE_PROG);
		}	
		if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_AFTER_PROG)
		{
			mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_AFTER_PROG);	
		}	

		if (w_adapter->bb_rf_rw)
		{		
			mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REG_READ);
			w_adapter->bb_rf_rw = 0;
		}	
		w_adapter->soft_reset = 0;

		mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(num_of_vals);
		mgmt_frame->desc_word[6] = ONEBOX_CPU_TO_LE16(w_adapter->bb_rf_params.no_of_fields);
#ifdef RF_8230
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (NONRSI_RF_TYPE << 4));
#else
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (RSI_RF_TYPE << 4));
#endif  
		//Flags are not handled :
		//: What is the radio id to fill here
		//	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (RADIO_ID << 8 );
	}
	else 
	{
		frame_len = 0;
		w_adapter->buffer_read_len = num_of_vals;
		mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((frame_len) | (ONEBOX_WIFI_MGMT_Q << 12));
		mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BB_BUF_PROG_VALUES_REQ);
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REG_READ);
		mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(1);
		w_adapter->bb_rf_rw = 0;
		w_adapter->soft_reset = 0;
		mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(num_of_vals);
		mgmt_frame->desc_word[6] = ONEBOX_CPU_TO_LE16(w_adapter->bb_rf_params.no_of_fields);
#ifdef RF_8230
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (NONRSI_RF_TYPE << 4));
#else
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (RSI_RF_TYPE << 4));
#endif  
	}
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, (frame_len + FRAME_DESC_SZ ));
	status_l = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (frame_len + FRAME_DESC_SZ));
	return status_l;
}

static ONEBOX_STATUS onebox_bb_buffer_request_indirect(WLAN_ADAPTER w_adapter,
                                             uint16 *bb_buf_vals,
                                             uint16 num_of_vals)
{
		onebox_mac_frame_t *mgmt_frame;
		ONEBOX_STATUS status_l;
		uint16 frame_len;
		uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

		FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
						(TEXT("===> Sending Buffer Programming Packet INDIRECT<===\n")));

		/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
		mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

		w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)bb_buf_vals, ((num_of_vals*2 *3) + 6));
		/* Preparing BB BUFFER Request Frame Body */

		frame_len = ((num_of_vals * 2 *3) + 6);	//for 1 value there are 3 regs and each 2 bytes
		w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bb_buf_req.bb_buf_vals[0], &bb_buf_vals[1], (frame_len));
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)&mgmt_frame->u.bb_buf_req.bb_buf_vals[0], (frame_len));

		/* Preparing BB Request Frame Header */

		/*prepare the frame descriptor */
		mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((frame_len) | (ONEBOX_WIFI_MGMT_Q << 12));
		mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BB_BUF_PROG_VALUES_REQ);
		mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(2);

		if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_BEFORE_PROG)
		{
				mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_BEFORE_PROG);
		}	
		if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_AFTER_PROG)
		{
				mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_AFTER_PROG);	
		}	

		if (w_adapter->bb_rf_rw)
		{		
				mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REG_READ);
				w_adapter->bb_rf_rw = 0;
		}	
		w_adapter->soft_reset = 0;

		mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(num_of_vals);
#ifdef RF_8230
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (NONRSI_RF_TYPE << 4));
#else
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (RSI_RF_TYPE << 4));
#endif  
		//Flags are not handled :
		//: What is the radio id to fill here
		//	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (RADIO_ID << 8 );

		//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, (frame_len + FRAME_DESC_SZ ));
		status_l = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (frame_len + FRAME_DESC_SZ));
		return status_l;
}

static ONEBOX_STATUS onebox_mgmt_send_rf_reset_req(WLAN_ADAPTER w_adapter,
                                             uint16 *bb_prog_vals)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status_l;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("ONEBOX_IOCTL: RF_RESET REQUEST\n"));

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
			(TEXT("===> Frame request to reset RF<===\n")));

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);
	if (bb_prog_vals == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("ONEBOX_IOCTL: RF_RESET REQUEST NULL PTR\n"));
		return ONEBOX_STATUS_FAILURE;

	}
	/* FrameType*/
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(RF_RESET_FRAME);
	mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(bb_prog_vals[1] & 0x00ff);
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16((bb_prog_vals[2]) >> 8);
	mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT( " RF_RESET: value is 0x%x, RF_DELAY: %d\n"),mgmt_frame->desc_word[3],mgmt_frame->desc_word[4]));
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, FRAME_DESC_SZ);
	status_l = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, ( FRAME_DESC_SZ));
	return status_l;
}

ONEBOX_STATUS bb_reset_req(WLAN_ADAPTER w_adapter)
{
	uint16 bb_prog_vals[3];

	bb_prog_vals[1] = 0x3;
	bb_prog_vals[2] = 0xABAB;

	if(onebox_mgmt_send_rf_reset_req(w_adapter, bb_prog_vals) != ONEBOX_STATUS_SUCCESS) {
		return ONEBOX_STATUS_FAILURE;
	}
	return ONEBOX_STATUS_SUCCESS;
}

ONEBOX_STATUS check_scan( WLAN_ADAPTER w_adapter)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status_l;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
  FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("ONEBOX_IOCTL: CHECK_SCAN\n"));

  mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

  w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);
  mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(SCAN_PROGRESS_CHECK);
  mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12);
  status_l = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, ( FRAME_DESC_SZ));
	return status_l;
}
ONEBOX_STATUS onebox_ant_sel(WLAN_ADAPTER w_adapter, uint8 value, uint8 frame_type)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status_l;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	struct driver_assets *d_assets = w_adapter->d_assets;

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("Frame req: ANT_SEL REQUEST\n"));


	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(ANT_SEL_FRAME);
	mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12);
	if( frame_type == CONFIG_ANT_SEL) 
	{
		/* FrameType*/
		mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(frame_type << 8);
		mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(value & 0x00ff);
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT( " ANT_SEL: value is 0x%x, \n"),mgmt_frame->desc_word[3]));
	}
	else if( frame_type == CONFIG_ANT_TYPE)
	{
		mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(frame_type << 8);
		mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(w_adapter->ant_path & 0x00ff);
		mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(w_adapter->ant_type & 0x00ff);
		mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(d_assets->onboard_antenna);

	}
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, FRAME_DESC_SZ);
	status_l = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, ( FRAME_DESC_SZ));
	w_adapter->antenna_in_use = value;
	return status_l;
}


static ONEBOX_STATUS onebox_mgmt_lmac_reg_ops_req(WLAN_ADAPTER w_adapter,
                                             uint16 *prog_vals,
                                             uint8  type)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status_l;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

  FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("ONEBOX_IOCTL: LMAC_REG_OPS REQUEST\n"));

  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
      (TEXT("===> Frame request  FOR LMAC_REG_OPS<===\n")));

  mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

  w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);
	if (prog_vals == NULL)
	{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("ONEBOX_IOCTL: LMAC_REG_OPS REQUEST NULL PTR\n"));
					return ONEBOX_STATUS_FAILURE;

	}
  /* FrameType*/
  mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(LMAC_REG_OPS);
  mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(prog_vals[1]);
  mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(prog_vals[2]);
  if (type == LMAC_REG_WRITE)
  {  
    mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(prog_vals[3]);
    mgmt_frame->desc_word[6] = ONEBOX_CPU_TO_LE16(prog_vals[4]);
    mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(0);// write Indication
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT( " LMAC_REG_OPS: type %d : addr  is 0x%x,  addr1  is 0x%x,\n \t \t Data: 0x%x\nData1: 0x%x\n"), 
          mgmt_frame->desc_word[7], mgmt_frame->desc_word[3],mgmt_frame->desc_word[4], mgmt_frame->desc_word[5],mgmt_frame->desc_word[6]));
  }
  else
  {
    mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(1); //Read Indication
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT( " LMAC_REG_OPS: type %d : addr  is 0x%x,  addr1  is 0x%x"), 
                                    mgmt_frame->desc_word[7], mgmt_frame->desc_word[3],mgmt_frame->desc_word[4]));
  }  
  mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12);
  //w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, FRAME_DESC_SZ);
  status_l = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, ( FRAME_DESC_SZ));
	return status_l;
}

#if 0
void load_calc_timer_values(WLAN_ADAPTER w_adapter)
{
	uint16 short_slot_time=0;
	uint16 long_slot_time=0 ;
	uint16 ack_time=0;
	uint16 in_sta_dist = 0;        

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO ,(TEXT("Loading Calculated values\n")));
	ack_time = ( (2 * in_sta_dist) / 300) + 192;/*In micro sec*/
	short_slot_time = ( in_sta_dist / 300) + 9;/*In micro sec*/
	long_slot_time = ( in_sta_dist / 300) + 20;
	w_adapter->short_slot_time = short_slot_time * 40;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO ,
	             (TEXT("short_slot_time:%d\n"),w_adapter->short_slot_time)); 
	w_adapter->long_slot_time = long_slot_time * 40;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("long_slot_time:%d\n"),w_adapter->long_slot_time)); 
	//w_adapter->short_difs_rx = 0x2F8;
	w_adapter->short_difs_rx = (SIFS_DURATION +(1*short_slot_time)-6-11)*40;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("short_difs_rx:%d\n"), w_adapter->short_difs_rx));
	/* DIFS = (2 * Slot_time + SIFS ) 1 slot will be added in the PPE */ 
	//w_adapter->short_difs_tx = 0x3E8;
	w_adapter->short_difs_tx = ((SIFS_DURATION +(1*short_slot_time)-6-2)*40);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("short_difs_tx:%d\n"),w_adapter->short_difs_tx)); 
	w_adapter->long_difs_rx = (SIFS_DURATION +(2*long_slot_time)-6-11)*40;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("long_difs_rx:%d\n"),w_adapter->long_difs_rx)); 
	w_adapter->long_difs_tx = (SIFS_DURATION +(2*long_slot_time)-6-2)*40;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("long_difs_tx:%d\n"),w_adapter->long_difs_tx));
	w_adapter->difs_g_delay = 240;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("difs_g_delay:%d\n"),w_adapter->difs_g_delay)); 
	w_adapter->ack_timeout = (ack_time * 40 );
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("ACK timeout:%d\n"),w_adapter->ack_timeout)); 
	return;
}

#endif

/**
 * This function sends the start_autorate frame to the TA firmware.
 * @param 
 *  w_adapter  Pointer to the driver private structure
 *
 * @returns 
 *  ONEBOX_STATUS_SUCCESS on success, or corresponding negative
 *  error code on failure
 */
ONEBOX_STATUS start_autorate_stats(WLAN_ADAPTER w_adapter)
{
	onebox_mac_frame_t *mgmt_frame;
	uint8  pkt_buffer[FRAME_DESC_SZ];

	ONEBOX_DEBUG(ONEBOX_ZONE_AUTORATE,
	             (TEXT("%s: Starting autorate algo\n"), __func__));


	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(FRAME_DESC_SZ);  
	//mgmt.desc_word[0] |= ONEBOX_TA_MGMT_Q  << 12; 
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(MGMT_DESC_START_AUTORATE);
	return onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, FRAME_DESC_SZ);
}


/**
* This function sends the radio capabilities to firmware
* @param 
*   w_adapter  Pointer to the driver private structure
*
* @returns 
*   ONEBOX_STATUS_SUCCESS on success, or corresponding negative
* error code on failure
*/
ONEBOX_STATUS onebox_load_radio_caps(WLAN_ADAPTER w_adapter)
{
	struct ieee80211com *ic = NULL;
	struct chanAccParams wme;
	struct chanAccParams wme_sta;
	onebox_mac_frame_t *mgmt_frame;
	uint16 inx = 0;
	int32 status = 0;
	uint16 pkt[256];
	uint8 ii;
	uint8 radio_id;
	uint16 gc[20] = {0xf0, 0xf0, 0xf0, 0xf0,
					 0xf0, 0xf0, 0xf0, 0xf0,
					 0xf0, 0xf0, 0xf0, 0xf0,
					 0xf0, 0xf0, 0xf0, 0xf0,
					 0xf0, 0xf0, 0xf0, 0xf0};	

	FUNCTION_ENTRY(ONEBOX_ZONE_INIT);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("%s: in %d Sending rate symbol req frame\n"), __func__,__LINE__));
	ic = &w_adapter->vap_com;
	wme = ic->ic_wme.wme_wmeChanParams;
	wme_sta = ic->ic_wme.wme_wmeChanParams;
	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, 256);

	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(RADIO_CAPABILITIES);
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(ic->ic_curchan->ic_ieee);
	mgmt_frame->desc_word[4] |= ONEBOX_CPU_TO_LE16(RSI_RF_TYPE<<8);

	if( ic->ic_curchan->ic_flags & IEEE80211_CHAN_11J)
		mgmt_frame->desc_word[6] = (ONEBOX_CPU_TO_LE16(1 << 8 ));
	else
		mgmt_frame->desc_word[6] = 0; 

	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(ONEBOX_LMAC_CLOCK_FREQ_80MHZ);	// Radio config info	
	if(w_adapter->operating_chwidth == BW_40Mhz)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("===> chwidth is 40Mhz <===\n")));
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(ONEBOX_ENABLE_40MHZ);			
		if (w_adapter->endpoint_params.per_ch_bw)
		{
			mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16(w_adapter->endpoint_params.per_ch_bw << 12);	// PPE ACK rate info 2-upper is primary
			mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16(FULL_40M_ENABLE);	// PPE ACK rate info 2-upper is primary
		}
		if(ic->band_flags & IEEE80211_CHAN_HT40)
		{
      // PPE ACK rate info 2-upper is primary
			mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(0);	
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("40Mhz: Programming the radio caps for 40Mhz mode\n")));
		if(ic->band_flags & IEEE80211_CHAN_HT40U)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Lower 20 Enable\n")));
				mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16(LOWER_20_ENABLE);	//PPE ACK MASK FOR 11B
				mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16(LOWER_20_ENABLE >> 12);	// PPE ACK rate info 2-upper is primary
			}
			else
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Upper 20 Enable\n")));
				mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16(UPPER_20_ENABLE);	//Indicates The primary channel is above the secondary channel
				mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16(UPPER_20_ENABLE >> 12);	// PPE ACK rate info 2-upper is primary
			}
   	 }

#ifdef	MODE_11AH
 	/*FIX ME-PPE ACK Rate;		*/
		

#endif
	}
#ifdef MODE_11AH

	else if(w_adapter->operating_chwidth == BW_2Mhz)
	{

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("===> chwidth is 11AH 2Mhz <===\n")));
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(ONEBOX_ENABLE_11AH);			
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(ONEBOX_ENABLE_2MHZ);
				


	}
	else if(w_adapter->operating_chwidth == BW_4Mhz)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("===> chwidth is 11AH 4Mhz <===\n")));
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(ONEBOX_ENABLE_11AH);			
		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(ONEBOX_ENABLE_4MHZ);


	}

#endif
    if (w_adapter->device_model == RSI_DEV_9116) {
       if(w_adapter->operating_chwidth == BW_20Mhz) {
        //LMAC runs at 40Mhz clk in 9116
        uint16 value;

        value = ONEBOX_LE16_TO_CPU(mgmt_frame->desc_word[7]);
        value = value & ~0x3;
        mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(value);
      }
    }
	radio_id = 0;
	mgmt_frame->u.radio_caps.sifs_tx_11n = ONEBOX_CPU_TO_LE16(w_adapter->sifs_tx_11n);
	mgmt_frame->u.radio_caps.sifs_tx_11b = ONEBOX_CPU_TO_LE16(w_adapter->sifs_tx_11b);
	mgmt_frame->u.radio_caps.slot_rx_11n = ONEBOX_CPU_TO_LE16(w_adapter->slot_rx_11n);
 	mgmt_frame->u.radio_caps.ofdm_ack_tout = ONEBOX_CPU_TO_LE16(w_adapter->ofdm_ack_tout);
  	mgmt_frame->u.radio_caps.cck_ack_tout = ONEBOX_CPU_TO_LE16(w_adapter->cck_ack_tout);
 	mgmt_frame->u.radio_caps.preamble_type = ONEBOX_CPU_TO_LE16(w_adapter->preamble_type);

	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(radio_id << 8);			//radio_id	

	mgmt_frame->u.radio_caps.qos_params[0].cont_win_min_q = ONEBOX_CPU_TO_LE16((1 << (wme_sta.cap_wmeParams[1].wmep_logcwmin)) - 1); 
	mgmt_frame->u.radio_caps.qos_params[0].cont_win_max_q = ONEBOX_CPU_TO_LE16((1 << wme_sta.cap_wmeParams[1].wmep_logcwmax) - 1); 
	mgmt_frame->u.radio_caps.qos_params[0].aifsn_val_q = ONEBOX_CPU_TO_LE16((wme_sta.cap_wmeParams[1].wmep_aifsn));
	mgmt_frame->u.radio_caps.qos_params[0].txop_q = ONEBOX_CPU_TO_LE16((wme_sta.cap_wmeParams[1].wmep_txopLimit << 5));
	
	mgmt_frame->u.radio_caps.qos_params[1].cont_win_min_q = ONEBOX_CPU_TO_LE16((1 << wme_sta.cap_wmeParams[0].wmep_logcwmin) - 1); 
	mgmt_frame->u.radio_caps.qos_params[1].cont_win_max_q = ONEBOX_CPU_TO_LE16((1 << wme_sta.cap_wmeParams[0].wmep_logcwmax) - 1); 
	mgmt_frame->u.radio_caps.qos_params[1].aifsn_val_q = ONEBOX_CPU_TO_LE16((wme_sta.cap_wmeParams[0].wmep_aifsn));
	mgmt_frame->u.radio_caps.qos_params[1].txop_q = ONEBOX_CPU_TO_LE16((wme_sta.cap_wmeParams[0].wmep_txopLimit << 5));

	mgmt_frame->u.radio_caps.qos_params[4].cont_win_min_q = ONEBOX_CPU_TO_LE16((1 << wme.cap_wmeParams[1].wmep_logcwmin) - 1); 
	mgmt_frame->u.radio_caps.qos_params[4].cont_win_max_q = ONEBOX_CPU_TO_LE16((1 << wme.cap_wmeParams[1].wmep_logcwmax) - 1); 
	mgmt_frame->u.radio_caps.qos_params[4].aifsn_val_q = ONEBOX_CPU_TO_LE16((wme.cap_wmeParams[1].wmep_aifsn ));
	mgmt_frame->u.radio_caps.qos_params[4].txop_q = ONEBOX_CPU_TO_LE16((wme.cap_wmeParams[1].wmep_txopLimit << 5));
	
	mgmt_frame->u.radio_caps.qos_params[5].cont_win_min_q = ONEBOX_CPU_TO_LE16((1 << wme.cap_wmeParams[0].wmep_logcwmin) - 1); 
	mgmt_frame->u.radio_caps.qos_params[5].cont_win_max_q = ONEBOX_CPU_TO_LE16((1 << wme.cap_wmeParams[0].wmep_logcwmax) - 1); 
	mgmt_frame->u.radio_caps.qos_params[5].aifsn_val_q = ONEBOX_CPU_TO_LE16((wme.cap_wmeParams[0].wmep_aifsn));
	mgmt_frame->u.radio_caps.qos_params[5].txop_q = ONEBOX_CPU_TO_LE16((wme.cap_wmeParams[0].wmep_txopLimit << 5));


	for(ii = 2; ii < (MAX_HW_QUEUES - 8); ii++)
	{
		/* : Num of AC's are mapped to 4 How to fill in for Q4-Q7 */
		mgmt_frame->u.radio_caps.qos_params[ii].cont_win_min_q = ONEBOX_CPU_TO_LE16((1  << wme_sta.cap_wmeParams[ii].wmep_logcwmin) - 1); 
		mgmt_frame->u.radio_caps.qos_params[ii].cont_win_max_q = ONEBOX_CPU_TO_LE16((1 << wme_sta.cap_wmeParams[ii].wmep_logcwmax) - 1); 
		//if(ii != 5)
			//mgmt_frame->u.radio_caps.qos_params[ii].aifsn_val_q = (wme_sta.cap_wmeParams[ii].wmep_aifsn -1);
		//else
			mgmt_frame->u.radio_caps.qos_params[ii].aifsn_val_q = ONEBOX_CPU_TO_LE16((wme_sta.cap_wmeParams[ii].wmep_aifsn ));

		mgmt_frame->u.radio_caps.qos_params[ii].txop_q = ONEBOX_CPU_TO_LE16((wme_sta.cap_wmeParams[ii].wmep_txopLimit << 5));

		mgmt_frame->u.radio_caps.qos_params[ii + 4].cont_win_min_q = ONEBOX_CPU_TO_LE16((1  << wme.cap_wmeParams[ii].wmep_logcwmin)); 
		mgmt_frame->u.radio_caps.qos_params[ii + 4].cont_win_max_q = ONEBOX_CPU_TO_LE16((1 << wme.cap_wmeParams[ii].wmep_logcwmax)); 
		mgmt_frame->u.radio_caps.qos_params[ii + 4].aifsn_val_q = ONEBOX_CPU_TO_LE16((wme.cap_wmeParams[ii].wmep_aifsn));
		mgmt_frame->u.radio_caps.qos_params[ii + 4].txop_q = ONEBOX_CPU_TO_LE16((wme.cap_wmeParams[ii].wmep_txopLimit << 5));
	}


	for(ii = 0; ii < MAX_HW_QUEUES ; ii++)
	{
		/* : Num of AC's are mapped to 4 How to fill in for Q4-Q7 */
		if(!(mgmt_frame->u.radio_caps.qos_params[ii].cont_win_min_q)) 
		{
			/* Contention window Min is zero indicates that parameters for this queue are not assigned 
			 * Hence Assigning them to defaults */
			mgmt_frame->u.radio_caps.qos_params[ii].cont_win_min_q = ONEBOX_CPU_TO_LE16(7); 
			mgmt_frame->u.radio_caps.qos_params[ii].cont_win_max_q = ONEBOX_CPU_TO_LE16(0x3f); 
			mgmt_frame->u.radio_caps.qos_params[ii].aifsn_val_q = 	ONEBOX_CPU_TO_LE16(2);
			mgmt_frame->u.radio_caps.qos_params[ii].txop_q = 0;
		}
	}
	/*Need to write MACROS for Queue_Nos*/
	mgmt_frame->u.radio_caps.qos_params[BROADCAST_HW_Q].txop_q = 0xffff;
	mgmt_frame->u.radio_caps.qos_params[MGMT_HW_Q].txop_q = 0;
	mgmt_frame->u.radio_caps.qos_params[BEACON_HW_Q].txop_q = 0xffff;

	w_adapter->os_intf_ops->onebox_memcpy(&w_adapter->rps_rate_mcs7_pwr, &gc[0], 40);
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_01_pwr  & 0x00FF));    /*1*/ 
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_02_pwr  & 0x00FF));    /*2*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_5_5_pwr & 0x00FF));   /*5.5*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_11_pwr  & 0x00FF));   /*11*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_48_pwr & 0x00FF));   /*48*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_24_pwr & 0x00FF));    /*24*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_12_pwr & 0x00FF));    /*12*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_06_pwr & 0x00FF));    /*6*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_54_pwr & 0x00FF));   /*54*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_36_pwr & 0x00FF));   /*36*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_18_pwr & 0x00FF));    /*18*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_09_pwr & 0x00FF));    /*9*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_mcs0_pwr & 0x00FF));  /*MCS0*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_mcs1_pwr & 0x00FF));  /*MCS1*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_mcs2_pwr & 0x00FF));  /*MCS2*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_mcs3_pwr & 0x00FF)); /*MCS3*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_mcs4_pwr & 0x00FF)); /*MCS4*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_mcs5_pwr & 0x00FF)); /*MCS5*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx++] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_mcs6_pwr & 0x00FF)); /*MCS6*/
	mgmt_frame->u.radio_caps.gcpd_per_rate[inx] = ONEBOX_CPU_TO_LE16((w_adapter->rps_rate_mcs7_pwr & 0x00FF)); /*MCS7*/
	
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(sizeof(mgmt_frame->u.radio_caps)  | (ONEBOX_WIFI_MGMT_Q << 12));

	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)mgmt_frame, sizeof(mgmt_frame->u.radio_caps) + FRAME_DESC_SZ);
	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("===> RADIO CAPS FRAME SENT <===\n")));

	
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (sizeof(mgmt_frame->u.radio_caps)+ FRAME_DESC_SZ));

	if (status == ONEBOX_STATUS_FAILURE)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to send radio caps frame\n")));
		return ONEBOX_STATUS_FAILURE;
	}

	FUNCTION_EXIT(ONEBOX_ZONE_INIT);
	return ONEBOX_STATUS_SUCCESS;
}


#if 0
static void onebox_eeprom_read_band(WLAN_ADAPTER w_adapter)
{
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Reading EEPROM RF TYPE\n" )));
        w_adapter->eeprom.length = ((WLAN_MAC_MAGIC_WORD_LEN + 3) & (~3)); /* 4 bytes are read to extract RF TYPE , always read dword aligned */
        w_adapter->eeprom.offset = WLAN_EEPROM_RFTYPE_ADDR; /* Offset val to read RF type is zero */
        if(eeprom_read(w_adapter) == ONEBOX_STATUS_SUCCESS)
        {
                w_adapter->fsm_state = FSM_EEPROM_READ_RF_TYPE;
        }
        else
        {
                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                                (TEXT("%s: Unable to Read RF type  %d, len %d\n"), __func__,  w_adapter->eeprom.offset, w_adapter->eeprom.length));
              w_adapter->operating_band = BAND_2_4GHZ;
        }
        return;
}
#endif
/**
 * This function receives the management packets from the hardware and process them
 *
 * @param 
 *  w_adapter  Pointer to driver private structure
 * @param 
 *  msg      Received packet
 * @param 
 *  len      Length of the received packet
 *
 * @returns 
 *  ONEBOX_STATUS_SUCCESS on success, or corresponding negative
 *  error code on failure
 */
int32 onebox_mgmt_pkt_recv(WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb)
{
	uint16 msg_type;
	int32 msg_len;
	uint8 sub_type;
	uint8 status;
	uint8 associd;
	uint8 *msg;
	uint8 sta_id ;
	uint32 path = 0;
#ifdef CONFIG_ACS              
	int id;
	acs_stats_t *acs_data = NULL;
#endif
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211vap *vap =NULL;
	struct ieee80211_node *ni =NULL;
	struct driver_assets *d_assets = w_adapter->d_assets;

#ifdef PWR_SAVE_SUPPORT
	uint16 confirm_type;
#ifdef ENABLE_DEEP_SLEEP
	//pwr_save_params	ps_params;
#endif
#endif
	//EEPROM_READ read_buf;

	FUNCTION_ENTRY(ONEBOX_ZONE_MGMT_RCV);

	msg = netbuf_cb->data;
	msg_len   = (ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[0]) & 0x0fff);
	
	/*Type is upper 5bits of descriptor */
	msg_type = (msg[2] );

	sub_type = (msg[15] & 0xff);
	ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_RCV,(TEXT("Rcvd Mgmt Pkt Len = %d type =%04x subtype= %02x w_adapter->fsm_state : %x\n"), msg_len, msg_type, sub_type,w_adapter->fsm_state));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_MGMT_RCV, msg, msg_len);

	ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_RCV,
	             (TEXT("WLAN Msg Len: %d, Msg Type: %4x\n"), msg_len, msg_type));
	w_adapter->total_mgmt_rx_done_intr++;
	switch (w_adapter->fsm_state)
	{
		case FSM_LOAD_BOOTUP_PARAMS:
			{
#ifndef FPGA_VALIDATION    
				if (msg_type == TA_CONFIRM_TYPE && sub_type == BOOTUP_PARAMS_REQUEST)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_FSM,
							(TEXT("%s: Received bootup parameters confirm sucessfully\n"),__FUNCTION__));
#endif
					if (w_adapter->Driver_Mode == QSPI_FLASHING)
					{
						w_adapter->fsm_state = FSM_MAC_INIT_DONE; 
						break;
					} 
					if (d_assets->band_version == 0x03) {
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("!!!! Dual band supported!!!!!\n")));
						w_adapter->operating_band = BAND_2_4GHZ;
						w_adapter->band_supported = 1;
						if (d_assets->onboard_antenna)
							ic->module_model_type = 0x301;
						else 
							ic->module_model_type = 0x03;
					} else {
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("!!!! 2.4Ghz band supported!!!!!\n")));
						w_adapter->operating_band = BAND_2_4GHZ;
						w_adapter->band_supported = 0;
						if (d_assets->onboard_antenna)
							ic->module_model_type = 0x201;
						else 
							ic->module_model_type = 0x02;
					}

					w_adapter->os_intf_ops->onebox_memcpy((PVOID)(w_adapter->mac_addr),
							d_assets->wlan_mac_address, IEEE80211_ADDR_LEN);
					w_adapter->os_intf_ops->onebox_memcpy(w_adapter->dev->dev_addr, 
							w_adapter->mac_addr, ETH_ALEN);
					w_adapter->core_ops->onebox_net80211_attach(w_adapter);
					if (onebox_send_reset_mac(w_adapter) == 0) {
						ONEBOX_DEBUG(ONEBOX_ZONE_FSM,
								(TEXT("%s: Reset MAC frame sent sucessfully\n"),__FUNCTION__));
						w_adapter->fsm_state = FSM_RESET_MAC_CFM;
						break;
					} else {
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: Failed to load reset mac frame \n"), __FUNCTION__));
					}
#ifndef FPGA_VALIDATION    
				}
#endif
			}
		break;
#if 0

		case FSM_EEPROM_READ_MAC_ADDR:
		{
			if (msg_type == TA_CONFIRM_TYPE && sub_type == EEPROM_READ_TYPE)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_FSM,
						(TEXT("%s: Received MAC Addr read confirmed \n"),__FUNCTION__));
				w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_FSM, msg, (16 +msg_len));
				if (msg_len > 0)
				{ 
					if  (msg[16] == MAGIC_WORD)
					{	
						w_adapter->os_intf_ops->onebox_memcpy((PVOID)(w_adapter->mac_addr),
								&msg[FRAME_DESC_SZ + WLAN_HOST_MODE_LEN + WLAN_MAC_MAGIC_WORD_LEN], IEEE80211_ADDR_LEN);
						//w_adapter->os_intf_ops->onebox_fill_mac_address(w_adapter); 
						w_adapter->os_intf_ops->onebox_memcpy(w_adapter->dev->dev_addr, 
										    w_adapter->mac_addr, ETH_ALEN);
					}
					else
          {
                  if (!w_adapter->calib_mode)
                  {
                          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                                          (TEXT("%s: UNABLE TO READ MAC_ADDR \n"),__FUNCTION__));
#ifndef CHIP_9116
                          w_adapter->fsm_state = FSM_CARD_NOT_READY;
                          break;
#else
						  random = (uptime() & 0x000ff);
						  w_adapter->mac_addr[0] = 0x00;
						  w_adapter->mac_addr[1] = 0x23;
						  w_adapter->mac_addr[2] = 0xA7;
						  w_adapter->mac_addr[3] = 0x00;
						  w_adapter->mac_addr[4] = 0x00;
						  w_adapter->mac_addr[5] = random;
#endif		
                  }						
          }	
				}  
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, w_adapter->mac_addr, (IEEE80211_ADDR_LEN));
        onebox_eeprom_read_band(w_adapter);
        break;
        case FSM_EEPROM_READ_RF_TYPE:

        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("CONFIRM FOR RF TYPE READ CAME\n")));
        if (msg_type == TA_CONFIRM_TYPE && sub_type == EEPROM_READ_TYPE)
        {
                ONEBOX_DEBUG(ONEBOX_ZONE_FSM,
                                (TEXT("%s: Received EEPROM RF type read confirmed \n"),__func__));
  		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, msg, (16 +msg_len));
#ifdef CHIP_9116
              ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("!!!! Dual band supported!!!!!\n")));
              w_adapter->operating_band = BAND_5GHZ;
       	      w_adapter->band_supported = 1;
#else
                if  (msg[16] == MAGIC_WORD)
                {
                        if((msg[17] & 0x3) == 0x3)
                        {
                                ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("!!!! Dual band supported!!!!!\n")));
                                w_adapter->operating_band = BAND_2_4GHZ;
																w_adapter->band_supported = 1;
                        }
                        else if((msg[17] & 0x3) == 0x1)
                        {
                                ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("!!!! 2.4Ghz band supported!!!!!\n")));
                                w_adapter->operating_band = BAND_2_4GHZ;
																w_adapter->band_supported = 0;
                        }
                        else
                        {
                                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Magic word present, But valid band is not present\n")));
                                w_adapter->fsm_state=FSM_CARD_NOT_READY;
                                break;
                        }
                }	
                else
                {
                        if (!w_adapter->calib_mode)
                        {
                                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("No Magic word present, Defaulting to 2.4Ghz\n")));
                                w_adapter->fsm_state=FSM_CARD_NOT_READY;
                                break;
                        }
                }
#endif
                w_adapter->core_ops->onebox_net80211_attach(w_adapter);
        }

        if(onebox_send_reset_mac(w_adapter) == 0)
        {

					ONEBOX_DEBUG(ONEBOX_ZONE_FSM,
							(TEXT("%s: Reset MAC frame sent sucessfully\n"),__FUNCTION__));
					w_adapter->fsm_state = FSM_RESET_MAC_CFM;
				}
				else
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: Failed to load reset mac frame \n"), __FUNCTION__));
					//: return from here ???
				}
			}
		}
		break;
#endif

		case FSM_RESET_MAC_CFM:
		{
			if(msg_type == TA_CONFIRM_TYPE && sub_type == RESET_MAC_REQ)	
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_FSM, (TEXT("Reset mac confirm is received\n")));
				ONEBOX_DEBUG(ONEBOX_ZONE_FSM, (TEXT("sending radio caps \n")));
				w_adapter->rx_filter_word = 0x0000;
				w_adapter->sifs_tx_11n = SIFS_TX_11N_VALUE;
				w_adapter->sifs_tx_11b = SIFS_TX_11B_VALUE;
				w_adapter->slot_rx_11n = SHORT_SLOT_VALUE;
				w_adapter->ofdm_ack_tout = OFDM_ACK_TOUT_VALUE;
				w_adapter->cck_ack_tout = CCK_ACK_TOUT_VALUE;
				w_adapter->preamble_type = LONG_PREAMBLE;
				if(!onebox_load_radio_caps(w_adapter))
				{
					return ONEBOX_STATUS_FAILURE;
				}	
			 	ONEBOX_DEBUG(ONEBOX_ZONE_FSM, (TEXT("Program BB/RF frames \n")));	
			}

			if(msg_type == TA_CONFIRM_TYPE && sub_type == RADIO_CAPABILITIES)
			{
         
				w_adapter->rf_reset = 1;
				ONEBOX_DEBUG(ONEBOX_ZONE_FSM, (TEXT(" RF_RESET AFTER at RESET REQ   = :\n")));

        if (w_adapter->device_model == RSI_DEV_9116) {
          if(onebox_send_w_9116_features(w_adapter)) {
            break;
          }
        }

				if(!program_bb_rf(w_adapter))
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_FSM,
				             (TEXT("%s: BB_RF values loaded successfully\n"),__FUNCTION__));
					w_adapter->fsm_state = FSM_BB_RF_START;
					w_adapter->fsm_state = FSM_MAC_INIT_DONE;
				}
			}
		}
		break;
		
		case FSM_BB_RF_START:	
		ONEBOX_DEBUG(ONEBOX_ZONE_FSM, (TEXT("In %s Line %d bb_rf_count  %d\n"), __func__, __LINE__,w_adapter->bb_rf_prog_count));
		if(((msg_type == TA_CONFIRM_TYPE) && ((sub_type == BB_PROG_VALUES_REQUEST) 
						|| (sub_type == RF_PROG_VALUES_REQUEST) || (sub_type == BBP_PROG_IN_TA) )))
		{
			w_adapter->bb_rf_prog_count--;
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
			         (TEXT(" FSM_STATE: BB RF count after receiving confirms for previous bb/rf frames %d\n"), w_adapter->bb_rf_prog_count));
			if(!w_adapter->bb_rf_prog_count)
				w_adapter->fsm_state = FSM_MAC_INIT_DONE;  
		}
		break;	
#if 0
		case FSM_DEEP_SLEEP_ENABLE:
		{
			if(msg_type == TA_CONFIRM_TYPE && sub_type == DEEP_SLEEP_ENABLE)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_FSM,
				        (TEXT("%s: Received confirm for Deep sleep enable frame\n"), __func__));
			}
				
		}
		break;
#endif
		case FSM_AMPDU_IND_SENT:
		{
			if(msg_type == TA_CONFIRM_TYPE && sub_type == AMPDU_IND)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_FSM,
						(TEXT("%s: Received confirm for ampdu indication frame\n"), __func__));
				ONEBOX_DEBUG(ONEBOX_ZONE_OID, (TEXT("onebox_set_per_tx_mode: Enabling the PER burst mode\n")));
				if (w_adapter->wlan_osd_ops->onebox_init_wlan_thread(&(w_adapter->sdio_scheduler_thread_handle_per), 
							"PER THREAD", 
							0, 
							w_adapter->wlan_osd_ops->onebox_per_thread, 
							w_adapter) != 0)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("onebox_set_per_tx_mode: Unable to initialize thread\n")));
					return -EFAULT;
				}
				w_adapter->os_intf_ops->onebox_start_thread(&(w_adapter->sdio_scheduler_thread_handle_per));
				w_adapter->tx_running = BURST_RUNNING;//indicating PER_BURST_MODE
			}
			w_adapter->fsm_state = FSM_MAC_INIT_DONE;
				
		}
		break;	
		
		case FSM_SCAN_CFM:
		{
			if((msg_type == TA_CONFIRM_TYPE) && (sub_type == SCAN_REQUEST))
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_FSM,(TEXT( "FSM_SCAN_CFM Received: \n")));
				w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
			}
		}
		break;

		case FSM_MAC_INIT_DONE:
		{
			//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Msg type %d\n"), msg_type));
			if(msg_type == TA_CONFIRM_TYPE)
			{
				switch (sub_type)
				{
					case STATS_REQUEST_FRAME:
					{
							//uint16 *framebody;
							ONEBOX_DEBUG(ONEBOX_ZONE_FSM,(TEXT("\nPER STATS PACKET  :\n")));

							//framebody = (uint16 *)&msg[16];

							//w_adapter->os_intf_ops->onebox_memset(&w_adapter->sta_info, 0, sizeof(per_stats));
							//w_adapter->os_intf_ops->onebox_memcpy((&w_adapter->sta_info), &msg[16], sizeof(per_stats));
							w_adapter->os_intf_ops->onebox_memset(&w_adapter->sta_info, 0, msg_len);
							w_adapter->os_intf_ops->onebox_memcpy((&w_adapter->sta_info), &msg[16], msg_len);
#if 0
                            printk("%s: STATS RECVD ack=%x msg_len = %x sizeof =%x \n",__func__,w_adapter->sta_info.ack_sent,msg_len,sizeof(per_stats));
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, &msg[16], (msg_len));
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("tx_pkts = 0x%04x\n)",w_adapter->sta_info.tx_pkts ));
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("tx_retries = 0x%04x\n)", w_adapter->sta_info.tx_retries));
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("xretries = 0x%04x\n)", w_adapter->sta_info.xretries));
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("rssi = 0x%x\n"), w_adapter->sta_info.rssi));
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("max_cons_pkts_dropped = 0x%04x\n"), w_adapter->sta_info.max_cons_pkts_dropped));
#endif						
							ONEBOX_DEBUG(ONEBOX_ZONE_FSM,
									(TEXT("%s: Sending Stats response frame\n"), __func__));
							w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->stats_event));
					}
					break;   
					case BB_PROG_VALUES_REQUEST:
					{
							ONEBOX_DEBUG(ONEBOX_ZONE_FSM,(TEXT(
											"BBP PROG STATS: Utils BB confirm Received: msg_len: %d \n"),msg_len));

							if ((msg_len) > 0)
							{	
							ONEBOX_DEBUG(ONEBOX_ZONE_FSM,(TEXT(
											"BBP PROG STATS: Utils BB confirm Received: msg_len -16 : %d \n"),msg_len));
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, &msg[16], (msg_len));
							w_adapter->os_intf_ops->onebox_memcpy((PVOID)(&w_adapter->bb_rf_read.Data[0]), &msg[16], (msg_len));
							w_adapter->bb_rf_read.no_of_values = (msg_len)/2 ;
							ONEBOX_DEBUG(ONEBOX_ZONE_FSM,(TEXT(
											"BBP PROG STATS: msg_len is : %d no of vls is %d  \n"),msg_len,w_adapter->bb_rf_read.no_of_values));
							w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
							}

					}
					break;
					case RF_PROG_VALUES_REQUEST:
					{
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
											"RF_PROG_STATS: Utils RF confirm Received: msg_len : %d \n"),msg_len));

							if ((msg_len) > 0)
							{	
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
											"RF_PROG_STATS: Utils RF confirm Received: msg_len -16 : %d \n"),msg_len));
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, msg, msg_len);
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, &msg[16], (msg_len));
							w_adapter->os_intf_ops->onebox_memcpy((PVOID)(&w_adapter->bb_rf_read.Data[0]), &msg[16], (msg_len));
							w_adapter->bb_rf_read.no_of_values = (msg_len)/2 ;
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
											"RF_PROG_STATS: msg_len is : %d no of vls is %d  \n"),msg_len,w_adapter->bb_rf_read.no_of_values));
							w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
							}
					}
					break;
					case BB_BUF_PROG_VALUES_REQ:
					{
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
											"BBP_USING_BUFFER: Utils BUF confirm Received: msg_len : %d \n"),msg_len));

							if ((msg_len) > 0)
							{	
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
											"BBP_USING_BUFFER: Utils BUF confirm Received: msg_len -16 : %d \n"),msg_len));
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, msg, msg_len);
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, &msg[16], (msg_len));
							w_adapter->os_intf_ops->onebox_memcpy((PVOID)(&w_adapter->bb_rf_read.Data[0]), &msg[16], (msg_len));
							w_adapter->bb_rf_read.no_of_values = (msg_len)/2 ;
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
											"BBP_USING_BUFFER: msg_len is : %d no of vls is %d  \n"),msg_len,w_adapter->bb_rf_read.no_of_values));
							w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
							}
              						else if(w_adapter->bb_rf_params.value == 7) //BUFFER_WRITE
              						{
                						w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
             						}
              						break;
          			 		}
					 case LMAC_REG_OPS:
					 {
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
											"LMAC_REG_OPS : Utils LMAC_REG_READ confirm Received: msg_len: %d \n"),msg_len));
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, &msg[0], (16));

							if ((msg_len) > 0)
							{	
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
											"LMAC_REG_OPS : Utils LMAC_REG_OPS confirm Received: msg_len -16 : %d \n"),msg_len));
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, &msg[16], (msg_len));
							w_adapter->os_intf_ops->onebox_memcpy((PVOID)(&w_adapter->bb_rf_read.Data[0]), &msg[16], (msg_len));
							w_adapter->bb_rf_read.no_of_values = (msg_len)/2 ;
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
											"LMAC_REG_OPS : msg_len is : %d no of vls is %d  \n"),msg_len,w_adapter->bb_rf_read.no_of_values));
							w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
							}

					}
					break;
          			case CW_MODE_REQ:
          			{
            			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
                    		"CW_MODE_REQ:: Utils CW_MODE_REQ: confirm Received: msg_len : %d \n"),msg_len));

            			w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
            			break;
					}
					case RF_LOOPBACK_REQ:
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
										"BBP_USING_BUFFER: Utils RF_LOOPBACK_REQ confirm Received: msg_len : %d \n"),msg_len));

						if ((msg_len) > 0)
						{	
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
											"BBP_USING_BUFFER: Utils RF_LOOPBACK_REQ confirm Received: msg_len -16 : %d \n"),msg_len));
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, msg, msg_len);
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(
											"BBP_USING_BUFFER: msg_len is : %d no of vls is %d rf_lpbk: %d \n"),msg_len,w_adapter->bb_rf_read.no_of_values,w_adapter->rf_lpbk_len));
							{ 
								w_adapter->os_intf_ops->onebox_memcpy((PVOID)(&w_adapter->rf_lpbk_data[w_adapter->rf_lpbk_len]), &msg[16], (msg_len));
								w_adapter->rf_lpbk_len += msg_len;
								ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("rf_lpbk_len : %d,\n"),w_adapter->rf_lpbk_len));
								if (w_adapter->rf_lpbk_len >= (4096))
								{  
									ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("SET EVENT rf_lpbk_len : %d,\n"),w_adapter->rf_lpbk_len));
									w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
								}
								else
								{  
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("BREAK EVENT rf_lpbk_len : %d,\n"),w_adapter->rf_lpbk_len));
									break;
								}
							}
						}
					}
					break;

					case RF_LPBK_M3:
					{
								
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
										"BBP_USING_BUFFER: Utils RF_LPBK_REQ confirm Received: msg_len : %d \n"),msg_len));
						if ((msg_len) > 0)
						{	
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(
											"BBP_USING_BUFFER: Utils RF_LPBK_M3_REQ confirm Received: msg_len -16 : %d \n"),msg_len));
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, msg, msg_len);
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(
											"BBP_USING_BUFFER: msg_len is : %d no of vls is %d rf_lpbk_m3: %d \n"),msg_len,w_adapter->bb_rf_read.no_of_values,w_adapter->rf_lpbk_len));
								w_adapter->os_intf_ops->onebox_memcpy((PVOID)(&w_adapter->rf_lpbk_data[w_adapter->rf_lpbk_len]), &msg[16], (msg_len));
								w_adapter->rf_lpbk_len += msg_len;
								ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("rf_lpbk_len : %d,\n"),w_adapter->rf_lpbk_len));
								if (w_adapter->rf_lpbk_len >= (512))
								{  
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("SET EVENT rf_lpbk_len : %d,\n"),w_adapter->rf_lpbk_len));
									w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
								}
								else
								{  
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("BREAK EVENT rf_lpbk_len : %d,\n"),w_adapter->rf_lpbk_len));
									break;
								}
						}
					}
					break;

					case BG_SCAN_PROBE_REQ:
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("<==== Recv Bg scan complete event ======>\n")));
							TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
							{
								if(vap == NULL)
								{							
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("In %s in %d vap is Null \n"),__func__,__LINE__));
									break;
								}
								else if (vap->iv_opmode == IEEE80211_M_STA)
								{
									if ( (ic->bgscan_host_triggered && 
												((ic->bgscan_2ghz_sent && !w_adapter->band_supported) ||
												 (w_adapter->band_supported && ic->bgscan_2ghz_sent && ic->bgscan_5ghz_sent))
											 ) 
											|| !ic->bgscan_host_triggered) {
											ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("<==== Indicating Scan complete event ======>\n")));
										//Limiting SCAN block for the STA connected state only. If already disconnected
										//and bgscan complete frame received from firmware scan should not be stopped. RSC-1328.
										if ((vap->iv_state == IEEE80211_S_RUN) || (vap->iv_host_scan)) {
											ic->ic_flags &= ~IEEE80211_F_SCAN;
											w_adapter->net80211_ops->onebox_send_scan_done(vap, ONEBOX_STATUS_SUCCESS);
										}
										vap->iv_block_scan = 0;
										break;
									}
									else if (w_adapter->band_supported && ic->bgscan_host_triggered){ 
										ic->ic_send_bgscan_params_default(ic, BAND_5GHZ);
									}
								}
							}

						}
						break;
#ifdef PWR_SAVE_SUPPORT
					case WAKEUP_SLEEP_REQUEST:
						{
							if(w_adapter->Driver_Mode != RF_EVAL_MODE_ON)
							{
									TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
									{
											if(vap == NULL)
													break;
											else if (vap->iv_opmode == IEEE80211_M_STA)
											{
															ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (TEXT("pwr save state PS_STATE %d \n"), PS_STATE));
															ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (TEXT("PWR save token is %d \n"), *(uint16 *)&msg[12]));
															confirm_type = ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[12]);
															if((confirm_type == SLEEP_REQUEST)) {
																			path = PS_EN_REQ_CNFM_PATH;
															}else if(confirm_type == WAKEUP_REQUEST){
																			path = PS_DIS_REQ_CNFM_PATH;
															}
															else {
																			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d Invalid confirm type %d\n"), __func__, __LINE__, confirm_type));
																			return ONEBOX_STATUS_FAILURE;
															}
															update_pwr_save_status(vap, PS_DISABLE, path);
															break;
															//w_adapter->net80211_ops->onebox_send_scan_done(vap);
											}
									}
							}
														
						}
						break;

					case DEV_SLEEP_REQUEST:
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Recv device sleep request======>\n")));
						}
						break;
					case DEV_WAKEUP_CNF:
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Recv device wakeup confirm======>\n")));
#if 0
							w_adapter->sleep_request_received = 0;
							if(w_adapter->queues_have_pkts == 1){
								schedule_pkt_for_tx(w_adapter);
							}
#endif
						}
						break;
#endif
					case SCAN_REQUEST:
					{	

						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("SCAN confirm received ifname is %s\n"), ic->ic_ifp->name));
						TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
						{
							if(vap == NULL)
								break;
							else if ((vap->iv_opmode == IEEE80211_M_STA) && (ic->ic_flags & IEEE80211_F_SCAN) && (!vap->iv_host_scan))
							{
								w_adapter->net80211_ops->onebox_scanreq_signal(vap->iv_ic);
							}

							else if((vap->iv_opmode == IEEE80211_M_HOSTAP)) {
								if(w_adapter->channel_change)
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("[%s][%d] : Received Channel programming Confirmation \n"), __func__, __LINE__));
									w_adapter->start_beacon = 1;
									w_adapter->channel_change = 0;
								}
								else
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("[%s][%d] : No Channel came from supplicant, Confirmation for default Channel programming\n"), __func__, __LINE__));
								}
							}
#if defined ONEBOX_CONFIG_CFG80211 && defined CONFIG_ACS              
							if((vap->iv_opmode == IEEE80211_M_HOSTAP) && vap->hal_priv_vap->acs_enable) {
								w_adapter->net80211_ops->onebox_scanreq_signal(vap->iv_ic);
								IEEE80211_LOCK(ic);
								acs_data = (struct acs_stats_s *)(netbuf_cb->data + FRAME_DESC_SZ);
								if(ic->idx < MAX_NUM_CHANNELS){
									id = ic->idx;
									ic->obm_survey[id].time_busy = acs_data->chan_busy_time;
									ic->obm_survey[id].noise = -abs(acs_data->noise_floor_rssi);
									ic->obm_survey[id].time = ACS_TIMEOUT_TIME;
									ic->obm_survey[id].channel = ic->ic_curchan;
									ic->idx++;
								}
								IEEE80211_UNLOCK(ic);
							}
#endif              
						}
					}
					break;
					case EEPROM_READ_TYPE:
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
										"EEPROM_READ: confirm Received:  \n")));

						w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, msg, 16);
						if (msg_len > 0)
						{  
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, &msg[16], msg_len);
							w_adapter->os_intf_ops->onebox_memcpy((PVOID)(&w_adapter->eeprom.data[0]), &msg[16], (msg_len));
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("  eeprom length: %d, eeprom msg_len %d\n"),w_adapter->eeprom.length, msg_len));
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, &w_adapter->eeprom.data[0], w_adapter->eeprom.length);
						}  
						w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
					}
					break;
					case EEPROM_WRITE:
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
										"EEPROM_WRITE: confirm Received:  \n")));

						w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, msg, 16);
						if (msg_len > 0)
							w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, &msg[16], msg_len);
						w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
					}
						break;
					case TX_POWER_REQUEST:
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(
										"TX_POWER_REQUEST: confirm Received:  \n")));
						w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, msg, netbuf_cb->len);
						TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
						{
							if(vap == NULL)
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("In %s in %d vap is Null \n"),__func__,__LINE__));
								break;
							}
							else
							{
							        vap->iv_bss->ni_txpower = ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[16]); 
							}
						}
						w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->bb_rf_event));
					}
					case SCAN_PROGRESS_CHECK:
						w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, msg, netbuf_cb->len);
						w_adapter->scan_state = ONEBOX_CPU_TO_LE16(*(uint16 *)&msg[16]);
						w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->scan_check_event));
					break;
					default:
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\nInvalid type in FSM_MAC_INIT_DONE  :\n")));
						break;
				}
			}
			else if (msg_type == TX_STATUS_IND)
			{
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\n STATUS IND   subtype %02x:\n"), sub_type));
						switch(sub_type)
						{
							case NULLDATA_CONFIRM:
								{
									status = msg[12];
									associd = msg[13];
									TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
									{
										if (vap->iv_opmode == IEEE80211_M_HOSTAP)
										{ 
											ni = w_adapter->sta[associd].ni;
											if (status){
												ni->ni_inact = ni->ni_inact_reload;
												ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s line %d:inact timer reloaded %d ni is %p associd is %d\n"), __func__, __LINE__, ni->ni_inact, ni, associd));
											} else {
												ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s line %d:keepalive confirm failed\n"), __func__, __LINE__));
												break;
											}
										}
									}
								}
								break;
							case EAPOL4_CONFIRM: 
								{
									if(msg[12])
									{
										ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Received EAPOL4 CONFIRM ======>\n")));
										TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
										{
												if (vap->iv_opmode == IEEE80211_M_STA)
												{ 
														vap->hal_priv_vap->conn_in_prog = 0;
														w_adapter->sta_mode.eapol4_cnfrm = 1;
														ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d  eapol %d ptk %d sta_block %d ap_block %d\n"), __func__, __LINE__,
																		w_adapter->sta_mode.eapol4_cnfrm, w_adapter->sta_mode.ptk_key, w_adapter->sta_data_block, w_adapter->block_ap_queues)); 
														if((w_adapter->sta_mode.ptk_key) && 
																						(w_adapter->sta_mode.eapol4_cnfrm) && 
																						(w_adapter->sta_data_block)) {
																onebox_send_block_unblock(vap, STA_CONNECTED, 0);
														}
														if(w_adapter->sta_mode.ptk_key && w_adapter->sta_mode.gtk_key && w_adapter->sta_mode.eapol4_cnfrm) {
																ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Reseting ptk key variable in %s Line %d \n"), __func__, __LINE__));
																w_adapter->sta_mode.ptk_key = 0; 
																ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (TEXT("calling update_pwr_save_status In %s Line %d\n"), __func__, __LINE__));
																update_pwr_save_status(vap, PS_ENABLE, CONNECTED_PATH);

														}
														break;
												}
										}

								}
								else
								{
										ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("EAPOL4 failed Doesn't had sta_id vap_id\n")));
										TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
										{
										  if (vap->iv_opmode == IEEE80211_M_STA)
										  { 
										    ni = vap->iv_bss; 
										    if(ni)
										      w_adapter->net80211_ops->onebox_ieee80211_sta_leave(ni);
										  }
										}
										// :vap_id should be handled here 
										//w_adapter->sta_mode.ptk_key = 0;
										//hal_load_key(w_adapter, NULL, 0, w_adapter->sta_mode.sta_id, ONEBOX_PAIRWISE_KEY, 0,
										//				w_adapter->sec_mode[0], w_adapter->hal_vap[w_adapter->sta_mode.vap_id].vap);
										//hal_load_key(w_adapter, NULL, 0, 0, ONEBOX_PAIRWISE_KEY, 0, w_adapter->sec_mode[vap_id]);
								}
						}
						break;
					case PROBEREQ_CONFIRM :
						{
							if((uint16)msg[12])
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("<==== Recv PROBEREQ CONFIRM ======>\n")));
							}
							else
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Probe Request Failed \n")));
							}

							TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
							{
									if (vap->iv_opmode == IEEE80211_M_STA)
									{
										w_adapter->net80211_ops->onebox_send_probereq_confirm(vap);
									}
							}
						}
						break;
					default:
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\nInvalid type in FSM_MAC_INIT_DONE %02x :\n"), sub_type));
						break;
				}
				w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, msg, msg_len);
			}
			else if(msg_type == DEBUG_IND)
			{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Received debug frame of len %d\n"), msg_len));
					dump_debug_frame(msg, msg_len);
			}
			else if(msg_type == RADAR_DETECTED)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n\nRADAR PULSE NOTIFICTAION RECEIVED\n")));
		//		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, msg, msg_len);
				if (onebox_radar_detect_algo( w_adapter, msg, msg_len) == ONEBOX_STATUS_FAILURE )
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR while detecting radar\n")));
					break;
				}
			}
#ifdef IEEE80211K
			else if( msg_type == MEASUREMENT_REPORT)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("\n\nMeasurement Report Received\n")));
				TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
				{
					if(vap == NULL)
					{							
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("In %s in %d vap is Null \n"),__func__,__LINE__));
						break;
					}
					else if (vap->iv_opmode == IEEE80211_M_STA) /* assuming Measurements are made by STA vaps */
					{
						w_adapter->core_ops->onebox_dump(1, msg, msg_len);
						w_adapter->net80211_ops->onebox_meas_rpt(vap, msg, msg_len); 
					}
				}
			}
#endif

			else if (msg_type == HW_BMISS_EVENT)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Beacon miss occured..! Indicate the same to Net80211 state machine\n")));
				w_adapter->net80211_ops->ieee80211_beacon_miss(&w_adapter->vap_com);
			}
			else if (msg_type == TSF_SYNC_CONFIRM)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Recvd TSF SYNC CONFIRM\n")));
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO ,(TEXT("\n STA_ID %02x TSF_LSB is %02x \n"), msg[13], ONEBOX_CPU_TO_LE32(*(uint32 *)&msg[16])));
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO ,(TEXT("\n STA_ID %02x TSF_MSB is %02x \n"), msg[13], ONEBOX_CPU_TO_LE32(*(uint32 *)&msg[20])));
			        sta_id = msg[13];
				if(sta_id > MAX_STATIONS_SUPPORTED) {
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid Station ID %s line %d\n"), __func__, __LINE__));
					w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, msg, msg_len);
					return -1;
				}
				ni = w_adapter->sta[sta_id].ni;
				if(ni != NULL) {
					if(ni->ni_vap->iv_opmode != IEEE80211_M_HOSTAP) {
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid ni \n")));
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid Station ID %s line %d sta_id %d vap_opmode %d\n"), __func__, __LINE__, sta_id, vap->iv_opmode));
					w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, msg, msg_len);
						return ONEBOX_STATUS_FAILURE;
					}
				}else{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(" %s %d: Ni is NULL \n"), __func__, __LINE__));
					w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, msg, msg_len);
					return ONEBOX_STATUS_FAILURE;
				}
				ni->uapsd.eosp_tsf = ONEBOX_CPU_TO_LE32(*(uint32 *)&msg[16]);
				ni->uapsd.eosp_triggered = 0;
				w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, msg, msg_len);
				//w_adapter->net80211_ops->ieee80211_beacon_miss(&w_adapter->vap_com);
			}
			else if (msg_type == RATE_GC_TABLE_UPDATE) {
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG ,(TEXT(" <===== RATE GC TABLE UPDATE RECVD =====>\n")) );
				w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, msg, msg_len);
      			}
			else if (msg_type == ANTENNA_SELECT) {
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO ,(TEXT("ANTENNA SELECT UPDATE FRAME RECEIVED\n")) );
				w_adapter->antenna_in_use = msg[12];
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO ,(TEXT("ANTENNA IN USE UPDATED TO %d\n"), w_adapter->antenna_in_use));
			}
			else if (msg_type == WLAN_IQ_STATS) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR ,(TEXT("WLAN_IQ_STATS RECEIVED \n")) );
        if(w_adapter->wlan_iqs_stats.pkt == NULL ){
          w_adapter->wlan_iqs_stats.pkt =(unsigned char * )w_adapter->os_intf_ops->onebox_mem_zalloc(w_adapter->wlan_iqs_stats.no_of_samples, GFP_KERNEL);
          if(w_adapter->wlan_iqs_stats.pkt == NULL){
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR ,(TEXT("Mem alloc Failed dropping pkt \n")));
          w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->wlan_iqs_event));
          break;
          }
          w_adapter->os_intf_ops->onebox_memset(w_adapter->wlan_iqs_stats.pkt, 0,w_adapter->wlan_iqs_stats.no_of_samples);
        }
        if(w_adapter->wlan_iqs_stats.no_of_samples >= msg_len){
        w_adapter->os_intf_ops->onebox_memcpy(((w_adapter->wlan_iqs_stats.pkt)+w_adapter->wlan_iqs_stats.offset), &msg[16], msg_len);
        w_adapter->wlan_iqs_stats.no_of_samples -= msg_len;
        w_adapter->wlan_iqs_stats.offset += msg_len; 
        } else {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR ,(TEXT("Wrong wlan IQ no_of_samples recvd\n")) );
        w_adapter->os_intf_ops->onebox_memcpy(((w_adapter->wlan_iqs_stats.pkt)+w_adapter->wlan_iqs_stats.offset), &msg[16],w_adapter->wlan_iqs_stats.no_of_samples);
        w_adapter->wlan_iqs_stats.no_of_samples = 0;
        }
        if((w_adapter->wlan_iqs_stats.no_of_samples <= 0) || (msg[5] == 1 )) {
          w_adapter->wlan_iqs_stats.offset  = 0;
          w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->wlan_iqs_event));
        }
				w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, msg, (msg_len+FRAME_DESC_SZ));
			}
			else
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
						(TEXT("Reached FSM state %d :Not an internal management frame\n"), w_adapter->fsm_state));
				//	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, msg, msg_len);
				if (!onebox_mgmt_pkt_to_core(w_adapter, netbuf_cb, msg_len, msg_type))
					return ONEBOX_STATUS_SUCCESS;
			}

		}
		break;
		default:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			         		(TEXT("Reached FSM state %d :Not an internal management frame\n"), w_adapter->fsm_state));
			break;
		} 
	} /* End switch (w_adapter->fsm_state) */

	w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
	FUNCTION_EXIT(ONEBOX_ZONE_MGMT_RCV);
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * Entry point of Management module
 *
 * @param
 *  w_adapter    Pointer to driver private data
 * @param
 *  msg    Buffer recieved from the device
 * @param
 *  msg_len    Length of the buffer
 * @return
 *  Status
 */
int onebox_mgmt_pkt_to_core(WLAN_ADAPTER w_adapter,
			   netbuf_ctrl_block_t *netbuf_cb,
                            int32 msg_len,
                            uint8 type)
{
	int8 rssi;
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211vap *vap =NULL;
	uint16  sta_flags;
	uint8 conn_ni_mac[IEEE80211_ADDR_LEN];
	struct ieee80211_node *ni;
	uint32 vap_id;
 	uint8 *msg;
	uint8 pad_bytes;
	int8 chno;
	uint32 sta_index=0;

	FUNCTION_ENTRY(ONEBOX_ZONE_MGMT_RCV);
	vap	= TAILQ_FIRST(&ic->ic_vaps);
	msg = netbuf_cb->data;
	pad_bytes = msg[4];
	rssi = ONEBOX_CPU_TO_LE16(*(uint16 *)(msg + 16));
	chno = msg[15];

	if (type == RX_DOT11_MGMT)
	{
		msg_len -= pad_bytes;
		if ((msg_len <= 0) || (!msg))
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
			             (TEXT("Invalid incoming message of message length = %d\n"), msg_len));
			return ONEBOX_STATUS_FAILURE;
		}
		
		if(w_adapter->Driver_Mode == SNIFFER_MODE)
		{
      if(vap) 
			  vap->hal_priv_vap->extended_desc_size = pad_bytes;
      else {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
			             (TEXT("Unable to Fetch the VAP pointer \n")));
			return ONEBOX_STATUS_FAILURE;
      }

			w_adapter->os_intf_ops->onebox_netbuf_adj(netbuf_cb, (FRAME_DESC_SZ));
		}
		else
		{
			w_adapter->os_intf_ops->onebox_netbuf_adj(netbuf_cb, (FRAME_DESC_SZ + pad_bytes));
		}

#if 0

		if( msg[FRAME_DESC_SZ + pad_bytes] == 0x80)
		//if(buffer[0] == 0x50 || buffer[0] == 0x80)
		{
				if(vap->iv_state == IEEE80211_S_RUN)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("===> Beacon rcvd <===\n")));
					w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, buffer, 30);
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("===> vap->iv_myaddr  = %02x:%02x:%02x <===\n"), vap->iv_myaddr[0], vap->iv_myaddr[1], vap->iv_myaddr[2]));

				}
		}
#endif

    if(!(w_adapter->Driver_Mode == SNIFFER_MODE))
    {
		  recv_onair_dump(w_adapter, netbuf_cb->data, netbuf_cb->len);
    }

#ifdef PWR_SAVE_SUPPORT
		//support_mimic_uapsd(w_adapter, buffer);
#endif
		w_adapter->core_ops->onebox_indicate_pkt_to_net80211(w_adapter, netbuf_cb, rssi, chno);
		return 0;
	}
	else if(type == PS_NOTIFY_IND)
	{
		//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, msg,  16);

		w_adapter->os_intf_ops->onebox_memcpy(conn_ni_mac, (msg + 6), IEEE80211_ADDR_LEN);

	//	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(" PS NOTIFY: connected station %02x:%02x:%02x:%02x:%02x:%02x\n"), conn_ni_mac[0], conn_ni_mac[1],
		//		conn_ni_mac[2], conn_ni_mac[3], conn_ni_mac[4], conn_ni_mac[5]));

		sta_flags = ONEBOX_CPU_TO_LE16(*(uint16 *) &msg[12]); /* station flags are defined in descriptor word6 */	
		vap_id = ((sta_flags & 0xC) >> 2);
		
		for (sta_index = 0; sta_index < w_adapter->max_stations_supported; sta_index++)
		{
			if(!w_adapter->os_intf_ops->onebox_memcmp(w_adapter->sta[sta_index].mac_addr, &conn_ni_mac, ETH_ALEN))
			{
					ni = w_adapter->sta[sta_index].ni;
					if(sta_index != ni->hal_priv_node.sta_id) {

							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
											(TEXT("ERROR: In %s Line %d The sta_id in Hal_station data structure and ni doesn't matches ni_sta_id %d hal_sta id %d\n"), 
											 __func__, __LINE__, ni->hal_priv_node.sta_id, sta_index));
							//dump_stack();
							return ONEBOX_STATUS_FAILURE;
					}
				break;
			}
		}
		
		if(sta_index == w_adapter->max_stations_supported) /* In case if not already connected*/
		{

			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("ERROR: In %s Line %d The sta_id in Hal_station data structure and ni doesn't matches hal_sta id %d\n"), 
					 __func__, __LINE__, sta_index));
			//dump_stack();
			return ONEBOX_STATUS_FAILURE;
		}
		if(vap_id > 3)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Vap_id is wrong in %s Line %d vap_id %d\n"), __func__, __LINE__, vap_id ));
			/*Freeing the netbuf coming as input to this function as the upper layers don't take care of the freeing*/
			return ONEBOX_STATUS_FAILURE;
		}

		IEEE80211_LOCK(ic);
		if(!w_adapter->hal_vap[vap_id].vap_in_use) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR: In %s Line %d vap_id %d is not installed\n"), __func__, __LINE__, vap_id ));
			/*Freeing the netbuf coming as input to this function as the upper layers don't take care of the freeing*/
			IEEE80211_UNLOCK(ic);
			return ONEBOX_STATUS_FAILURE;
			
		}

		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
		{
			if(vap->hal_priv_vap->vap_id == vap_id)
			{
				break;
			}
		}

		/* Find the node of the station */
		if(!vap){
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to find the vap##########vap_id %d  \n"), vap_id));
			IEEE80211_UNLOCK(ic);
				//dump_stack();
			return ONEBOX_STATUS_FAILURE;
		}

		if((&vap->iv_ic->ic_sta) == NULL)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR: In %s Line %d\n"), __func__, __LINE__));
			return ONEBOX_STATUS_FAILURE;
		}

		ni = w_adapter->net80211_ops->onebox_find_node(&vap->iv_ic->ic_sta, conn_ni_mac);

		if(ni && (ni->ni_vap == vap))
		{
			//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Found the exact node\n")));
		}
		else
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to find the node so discarding\n")));
			/*Freeing the netbuf coming as input to this function as the upper layers don't take care of the freeing*/
			IEEE80211_UNLOCK(ic);
			return ONEBOX_STATUS_FAILURE;
		}

		if(sta_flags & STA_ENTERED_PS)
		{
			w_adapter->net80211_ops->onebox_node_pwrsave(ni, 1);
			//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("powersave: STA Entered into pwrsave\n")));
			/* Set the power management bit to indicate station entered into pwr save */
			//ni->ni_flags |= IEEE80211_NODE_PWR_MGT;
		}
		else if(sta_flags & STA_EXITED_PS)
		{
			/* Reset the power management bit to indicate station came out of pwr save */
		//	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("powersave: STA Exited from pwrsave\n")));
			//ni->ni_flags &= ~IEEE80211_NODE_PWR_MGT;
			w_adapter->net80211_ops->onebox_node_pwrsave(ni, 0);
		}
		IEEE80211_UNLOCK(ic);
	}
	else if(type == BEACON_EVENT_IND) {
		vap_id = msg[15];
		IEEE80211_LOCK(ic);
		if(!w_adapter->hal_vap[vap_id].vap_in_use) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR: In %s Line %d vap_id %d is not installed\n"), __func__, __LINE__, vap_id ));
			/*Freeing the netbuf coming as input to this function as the upper layers don't take care of the freeing*/
			IEEE80211_UNLOCK(ic);
			return ONEBOX_STATUS_FAILURE;
			
		}
		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
		{
			if(vap->hal_priv_vap->vap_id == vap_id)
			{
				/* We set beacon interrupt only when VAP reaches RUN state, as firmware
				 * gives interrupt after sending vap_caps frame, so due to this there is
				 * a probabilty of sending beacon before programming channel as we give highest
				 * priority to beacon event in core_qos_processor.
				 */
				if((((vap->iv_state == IEEE80211_S_RUN) && (!w_adapter->block_ap_queues))
				   || (vap->iv_state == IEEE80211_S_CSA)) && (!vap->block_beacon_interrupt))
				{
					if(w_adapter->beacon_ssid_loaded && w_adapter->start_beacon)
					{
						w_adapter->beacon_event = 1;
						w_adapter->beacon_event_vap_id = vap_id;
						w_adapter->beacon_interrupt++;
						ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("Beacon Interrupt Received for vap_id %d \n"), vap_id));
						//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)msg, FRAME_DESC_SZ);
					}
					w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->sdio_scheduler_event));
				}
			}
		}
			IEEE80211_UNLOCK(ic);
	}
	else if(type == DECRYPT_ERROR_IND)
	{
		w_adapter->core_ops->onebox_mic_failure(w_adapter, msg);

	}
#ifdef ONEBOX_CONFIG_GTK_OFFLOAD
  else if(type == UPDATE_MCAST_PN)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("Updating iv_mcast_pn\n")));
    w_adapter->os_intf_ops->onebox_memcpy(&vap->iv_mcast_pn, (msg + 6), 8);
  }
#endif
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND,(TEXT ("Internal Packet\n")));
	}

	/*Freeing the netbuf coming as input only in success case*/
	w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
	FUNCTION_EXIT(ONEBOX_ZONE_MGMT_RCV);
	return 0;
}

/**
* Assigns the MAC address to the w_adapter
*
*/
ONEBOX_STATUS onebox_read_cardinfo(WLAN_ADAPTER w_adapter)
{
	ONEBOX_STATUS status = 0;
	//EEPROM_READ mac_info_read;

	//mac_info_read.off_set = EEPROM_READ_MAC_ADDR;
	//mac_info_read.length  = 6;     /* Length in words i.e 3*2 = 6 bytes */

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s: Reading MAC address\n"), __func__));
//	status = onebox_eeprom_read(w_adapter,(PEEPROM_READ)&mac_info_read);
	if (status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: EEPROM read Failed\n"), __func__));
	}
	return status;
}


int hal_set_sec_wpa_key(WLAN_ADAPTER w_adapter, const struct ieee80211_node *ni_sta, uint8 key_type)
{
	return 0;
}


uint32 hal_send_sta_notify_frame(WLAN_ADAPTER w_adapter, struct ieee80211_node *ni, uint8 notify_event)
{
#define P2P_GRP_GROUP_OWNER  0x0
	struct ieee80211vap *vap = NULL;
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	uint16 vap_id = 0;  
  uint8 mpdu_density_map[] = {0,1,1,1,2,4,8,16};

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
	             (TEXT("===> Sending Peer Notify Packet <===\n")));
	
	vap = ni->ni_vap;
	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);
	
	/* Fill frame body */
	switch(vap->iv_opmode)
	{
		case IEEE80211_M_STA: 
		{	
			/* connected peer is AP */
			mgmt_frame->u.peer_notify.command =  ONEBOX_CPU_TO_LE16((IEEE80211_OP_AP) << 1); //ap
			break;
		}
		case IEEE80211_M_HOSTAP: 
		{	
			mgmt_frame->u.peer_notify.command =  ONEBOX_CPU_TO_LE16((IEEE80211_OP_STA) << 1); //sta
			break;
		}
		case IEEE80211_M_MBSS: 
		{	
			mgmt_frame->u.peer_notify.command =  ONEBOX_CPU_TO_LE16((IEEE80211_OP_IBSS) << 1); //IBSS
			break;
		}
		default:
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid Mode\n")));
			return ONEBOX_STATUS_FAILURE;
	}

	vap_id = vap->hal_priv_vap->vap_id;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" In %s and %d notify_event = %d vap_id %d sta_id %d \n"), __func__, __LINE__, notify_event, vap_id, ni->hal_priv_node.sta_id));


	switch(notify_event)
	{
		case STA_CONNECTED:
				if(vap->iv_opmode == IEEE80211_M_STA) {
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Starting ifp queue \n")));
						vap->hal_priv_vap->stop_tx_q = 0;
						w_adapter->os_intf_ops->onebox_start_ifp_txq(vap->iv_ifp);
				}
		      mgmt_frame->u.peer_notify.command |= ONEBOX_CPU_TO_LE16(ONEBOX_ADD_PEER);
     		      mgmt_frame->u.peer_notify.mpdu_density = ONEBOX_CPU_TO_LE16(mpdu_density_map[((ni->ni_htparam >> 2) & 0x7)]);
		      ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("mpdu Density %d\n"), ONEBOX_CPU_TO_LE16(mgmt_frame->u.peer_notify.mpdu_density)));
			break;
		case STA_DISCONNECTED:
			mgmt_frame->u.peer_notify.command |= ONEBOX_CPU_TO_LE16(ONEBOX_DELETE_PEER);
			if(!(vap->hal_priv_vap->roam_ind) && (vap->iv_opmode == IEEE80211_M_STA))
			{
				//w_adapter->sec_mode[vap_id] = IEEE80211_CIPHER_NONE;
			}
			ni->ni_flags &= ~IEEE80211_NODE_ENCRYPT_ENBL;
#ifdef BYPASS_RX_DATA_PATH
			/** This should be changed based on STA ID in AP mode **/
			if(vap->iv_opmode == IEEE80211_M_STA && (!w_adapter->sta_data_block))
			{
				onebox_send_block_unblock(vap, STA_DISCONNECTED, 0);
			}

			if(vap->iv_opmode == IEEE80211_M_STA) {	
							w_adapter->os_intf_ops->onebox_memset(&w_adapter->sta_mode.ptk_key, 0, sizeof(struct sta_conn_flags));

							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d  eapol %d ptk %d sta_block %d ap_block %d\n"), __func__, __LINE__,
															w_adapter->sta_mode.eapol4_cnfrm, w_adapter->sta_mode.ptk_key, w_adapter->sta_data_block, w_adapter->block_ap_queues)); 

							if(w_adapter->traffic_timer.function)
											w_adapter->os_intf_ops->onebox_remove_timer(&w_adapter->traffic_timer);

							update_pwr_save_status(vap, PS_ENABLE, DISCONNECTED_PATH);
			}

#endif
		break;
		case STA_ADDBA_DONE:
		case STA_DELBA:
		case STA_RX_ADDBA_DONE:
		case STA_RX_DELBA:
		/*	: handle here */
			status = onebox_send_ampdu_indication_frame(w_adapter, ni, notify_event);
			return status;
		break;
		default:
		break;
	}
	/* Fill the association id */
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" association id =%x and command =%02x\n"), ni->ni_associd, mgmt_frame->u.peer_notify.command));
	mgmt_frame->u.peer_notify.command |= ONEBOX_CPU_TO_LE16((ni->ni_associd & 0xfff) << 4);
	w_adapter->os_intf_ops->onebox_memcpy(mgmt_frame->u.peer_notify.mac_addr, ni->ni_macaddr, ETH_ALEN);
	/*  : Fill ampdu/amsdu size, short gi, GF if supported here */
	mgmt_frame->u.peer_notify.sta_flags |= ONEBOX_CPU_TO_LE32((ni->ni_flags & IEEE80211_NODE_QOS) ? 1 : 0); //connected ap is qos supported or not

	/* Bit{0:11} indicates length of the Packet
 	 * Bit{12:16} indicates host queue number
 	 */ 
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((sizeof(mgmt_frame->u.peer_notify)) | ONEBOX_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(PEER_NOTIFY); 
	// : IN AP Mode fill like this 
	mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(ni->hal_priv_node.sta_id | (vap_id << 8));
	//mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16((vap_id << 8)); /* Peer ID is zero in sta mode */

	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, sizeof(mgmt_frame->u.peer_notify) + FRAME_DESC_SZ);


	//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Sending peer notify frame\n")));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("IN %s Line %d Vap_id %d Sta_id %d pkt_queued to head\n"), __func__, __LINE__, vap_id, ni->hal_priv_node.sta_id));
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (sizeof(mgmt_frame->u.peer_notify)+ FRAME_DESC_SZ));

	if(status)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to send peer notify frame\n")));
		return ONEBOX_STATUS_FAILURE;
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Opmode %d secmode %d \n"), vap->iv_opmode, w_adapter->sec_mode[vap->hal_priv_vap->vap_id]));
	if(!((vap->iv_flags & IEEE80211_F_WPA2) || (vap->iv_flags & IEEE80211_F_WPA1)) 
			&& (vap->iv_flags & IEEE80211_F_PRIVACY))
  {
    /** WEP Mode */
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Key idx %d key_len %d \n "), w_adapter->wep_key_idx[vap_id], w_adapter->wep_keylen[vap_id][w_adapter->wep_key_idx[vap_id]]));
    status = hal_load_key( w_adapter, w_adapter->wep_key[vap_id][w_adapter->wep_key_idx[vap_id]],  //selecting proper key_index for WEP 
                           w_adapter->wep_keylen[vap_id][w_adapter->wep_key_idx[vap_id]], 
                           ni->hal_priv_node.sta_id,
                           ONEBOX_PAIRWISE_KEY, 
                           w_adapter->wep_key_idx[vap_id], 
                           IEEE80211_CIPHER_WEP, vap);

    if(vap->iv_opmode == IEEE80211_M_STA)
    {
      status = hal_load_key(w_adapter, w_adapter->wep_key[vap_id][w_adapter->wep_key_idx[vap_id]],  //selecting proper key_index for WEP  
                             w_adapter->wep_keylen[vap_id][w_adapter->wep_key_idx[vap_id]], 
                             ni->hal_priv_node.sta_id, 
                             ONEBOX_GROUP_KEY, 
                             w_adapter->wep_key_idx[vap_id], 
                             IEEE80211_CIPHER_WEP, vap);
      ni->ni_vap->hal_priv_vap->conn_in_prog = 0;
#ifdef CONFIG_11W
    } else {
	ni->hal_priv_node.conn_in_prog = 0;
#endif
    }
    ni->ni_flags |= IEEE80211_NODE_ENCRYPT_ENBL;
  }
	else if((vap->iv_opmode == IEEE80211_M_STA) 
					&& (vap->iv_state == IEEE80211_S_RUN)
					&& !(vap->iv_flags & IEEE80211_F_PRIVACY)) {
		/** OPEN Mode **/
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d resetting conn_in_prog iv_state %d \n"), __func__, __LINE__, vap->iv_state));
			ni->ni_vap->hal_priv_vap->conn_in_prog = 0;
	}

	return ONEBOX_STATUS_SUCCESS;
}

#ifdef BYPASS_TX_DATA_PATH
ONEBOX_STATUS onebox_send_block_unblock(struct ieee80211vap *vap, uint8 notify_event, uint8 quiet_enable)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	struct net_device *parent_dev = vap->iv_ic->ic_ifp;
	WLAN_ADAPTER w_adapter = netdev_priv(parent_dev);
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211vap *vap_temp;
	struct ifnet *ifp = vap->iv_ifp;

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BLOCK_UNBLOCK);
	mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(0x1);

	if(notify_event == STA_DISCONNECTED)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d \n"), __func__, __LINE__));
		vap->hal_priv_vap->sta_data_block = 1;
		w_adapter->sta_data_block = 1;
		if (quiet_enable)
			mgmt_frame->desc_word[3] |= ONEBOX_CPU_TO_LE16(0x2); /* Enable QUIET */
		mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(0xf);
		/* We may receive disconnect event when we have programmed the timer 
 		 * so stop timer first
 		 */ 	
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Stopping the timer in %s Line %d\n"), __func__, __LINE__));
		ic->ic_stop_initial_timer(ic, vap);
#ifndef ONEBOX_CONFIG_CFG80211
               ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
               ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Carrier OFF is set\n")));
               netif_carrier_off(vap->iv_ifp);
#endif
		
		if(!w_adapter->os_intf_ops->onebox_is_ifp_txq_stopped(vap->iv_ifp))
		{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Stopping ifp queue \n")));
				vap->hal_priv_vap->stop_tx_q = 1;
#ifndef WIFI_ALLIANCE
				w_adapter->os_intf_ops->onebox_stop_ifp_txq(vap->iv_ifp);
#endif
		}
	}
	else if(notify_event == STA_CONNECTED)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d \n"), __func__, __LINE__));
		vap->hal_priv_vap->sta_data_block = 0;
		w_adapter->sta_data_block = 0;
		w_adapter->block_ap_queues = 0;
		mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(0xf);

#ifndef ONEBOX_CONFIG_CFG80211 //JHELUM-6692
		ifp->if_drv_flags |= IFF_DRV_RUNNING;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Carrier ON is set\n")));
		netif_carrier_on(vap->iv_ifp);
#endif

		if (w_adapter->antenna_diversity)
			w_adapter->scan_count = 0;	
	}

	if(w_adapter->block_ap_queues) {
		mgmt_frame->desc_word[4] |= ONEBOX_CPU_TO_LE16(0xf << 4);
		
	} else {
		TAILQ_FOREACH(vap_temp, &ic->ic_vaps, iv_next) 
		{ 
				if(vap_temp->iv_opmode == IEEE80211_M_HOSTAP) {
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Starting ifp queue at %s Line %d\n"), __func__, __LINE__));
					vap_temp->hal_priv_vap->stop_tx_q = 0;
					w_adapter->os_intf_ops->onebox_start_ifp_txq(vap_temp->iv_ifp);
				}
		}
		mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16(0xf << 4);
	
	}

	//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("<<<<<< BLOCK/UNBLOCK %d >>>>>>\n", notify_event)));
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, (FRAME_DESC_SZ));

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d sta_block %d ap_data %d\n"), __func__, __LINE__, w_adapter->sta_data_block, w_adapter->block_ap_queues));
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame,  FRAME_DESC_SZ);

	if(status)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Unable to send BLOCK/UNBLOCK indication frame\n")));
		return ONEBOX_STATUS_FAILURE;
	}

	return ONEBOX_STATUS_SUCCESS;
}
#endif

ONEBOX_STATUS onebox_send_ampdu_indication_frame(WLAN_ADAPTER w_adapter, struct ieee80211_node *ni, uint8 event)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	uint16 tidno;
	/*: For ap mode get the peerid */
	/* Fill peer_id in case of AP mode. For sta mode it is zero */
	//uint8 peer_id = 0;
	uint8 peer_id = ni->hal_priv_node.sta_id;

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
	             (TEXT("===> Sending AMPDU Indication Packet <===\n")));

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, 256);

	tidno = ni->hal_priv_node.tidnum;
	/* Bit{0:11} indicates length of the Packet
	 * Bit{12:16} indicates host queue number
 	*/ 
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((sizeof(mgmt_frame->u.ampdu_ind)) | ONEBOX_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(AMPDU_IND); 

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s and %d event %d \n"), __func__, __LINE__, event));
	if(event == STA_ADDBA_DONE)
	{
		mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(ni->hal_priv_node.tid[tidno].seq_start); 
		mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(ni->hal_priv_node.tid[tidno].baw_size); 
		mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(tidno | (START_AMPDU_AGGR << 4)| ( peer_id << 8)); 
	}
	else if(event == STA_RX_ADDBA_DONE)
	{
		mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(ni->hal_priv_node.tid[tidno].seq_start); 
		mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(tidno | (START_AMPDU_AGGR << 4)| (RX_BA_INDICATION << 5) | ( peer_id << 8)); 
	}
	else if(event == STA_DELBA)
	{
		mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(tidno | (STOP_AMPDU_AGGR << 4)| (peer_id << 8)); 
	}
	else if(event == STA_RX_DELBA)
	{
		if(ni->hal_priv_node.delba_ind)
		{
			mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(tidno | (STOP_AMPDU_AGGR << 4)| (RX_BA_INDICATION << 5) | (peer_id << 8)); 
		}
		else
		{
			mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(tidno | (STOP_AMPDU_AGGR << 4)| (peer_id << 8)); 
		}
	}
		
	//ONEBOX_DEBUG(ONEBOX_ZONE_MGMT_SEND, (TEXT("Sending ampdu indication frame\n")));
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_MGMT_SEND, (PUCHAR)mgmt_frame, sizeof(mgmt_frame->u.ampdu_ind) + FRAME_DESC_SZ);

	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (sizeof(mgmt_frame->u.ampdu_ind)+ FRAME_DESC_SZ));
	if(status)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to send ampdu indication frame\n")));
		return ONEBOX_STATUS_FAILURE;
	}
	return ONEBOX_STATUS_SUCCESS;
}

ONEBOX_STATUS onebox_send_internal_mgmt_frame(WLAN_ADAPTER w_adapter, uint16 *addr, uint16 len)
{
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	onebox_mac_frame_t *mgmt_frame;
  struct driver_assets *d_assets = w_adapter->d_assets;

	FUNCTION_ENTRY(ONEBOX_ZONE_MGMT_SEND);

  //! Checking for device remove status, and if it is removed should not send any mgmt packets
  if (d_assets->card_state != GS_CARD_DETACH)
  {
	netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(len);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;

	}

	w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);
	w_adapter->os_intf_ops->onebox_memset(netbuf_cb->data, 0, FRAME_DESC_SZ);
	/*copy the internal mgmt frame to netbuf and queue the pkt */
	w_adapter->os_intf_ops->onebox_memcpy((uint8 *)netbuf_cb->data, (uint8 *)addr, len);
	mgmt_frame = (onebox_mac_frame_t *)netbuf_cb->data;
	netbuf_cb->data[1] |= BIT(7);/* Immediate Wakeup bit*/
	netbuf_cb->flags |= INTERNAL_MGMT_PKT;	
	netbuf_cb->tx_pkt_type = WLAN_TX_M_Q;
	w_adapter->os_intf_ops->onebox_netbuf_queue_tail(&w_adapter->host_tx_queue[MGMT_SOFT_Q], netbuf_cb->pkt_addr);
	w_adapter->devdep_ops->onebox_schedule_pkt_for_tx(w_adapter);
  }
	return status;
}


ONEBOX_STATUS update_device_op_params(WLAN_ADAPTER w_adapter)
{

	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function is called after initial configuration is done. 
 * It starts the base band and RF programming
 *
 * @param 
 *      w_adapter     Pointer to hal private info structure
 *
 * @return 
 *      0 on success, -1 on failure
 */
ONEBOX_STATUS program_bb_rf(WLAN_ADAPTER w_adapter)
{

	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[FRAME_DESC_SZ];

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("===> Send BBP_RF_INIT frame in TA<===\n")));

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16( ONEBOX_WIFI_MGMT_Q << 12);
	
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BBP_PROG_IN_TA);
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(w_adapter->endpoint);

	mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(w_adapter->rf_pwr_mode);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("#### in %s rf pwr mode is %d\n"), __func__, w_adapter->rf_pwr_mode));

	if(w_adapter->rf_reset)
	{
		mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16 (RF_RESET_ENABLE);
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("===> RF RESET REQUEST SENT <===\n")));
		w_adapter->rf_reset = 0;
	}
	w_adapter->bb_rf_prog_count = 1;
#if 0
	if(w_adapter->chw_flag)
	{
		if(w_adapter->operating_chwidth == BW_40Mhz)
		{
			mgmt_frame->desc_word[4] |= ONEBOX_CPU_TO_LE16(0x1 << 8); /*20 to 40 Bandwidth swicth*/
		}
		else
		{
			mgmt_frame->desc_word[4] |= ONEBOX_CPU_TO_LE16(0x2 << 8);/*40 to 20 Bandwidth switch */
		}
		w_adapter->chw_flag = 0;
	}
#endif
	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (RSI_RF_TYPE << 4));
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, FRAME_DESC_SZ );
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame,  FRAME_DESC_SZ);
	return status;
}

void onebox_init_chan_pwr_table(WLAN_ADAPTER w_adapter,
                                uint16 *bg_power_set,
                                uint16 *an_power_set)
{
	uint8  ii = 0, cnt = 0, band = 0;
	uint16 *low_indx, *med_indx, *high_indx;
	uint16 *alow_indx, *amed_indx, *ahigh_indx;

	if (w_adapter->eeprom_type >= EEPROM_VER2)
	{
		if (an_power_set == NULL)
		{
			/* In case of 2.4 GHz band, GC/PD values are given for only 3 channels */
			cnt = 3;
			band = BAND_2_4GHZ;
		}
		else
		{
			/* In case of 5 GHz band, GC/PD values are given for 8 channels */
			cnt = 8;
			band = BAND_5GHZ;
		} /* End if <condition> */  
		
		for (ii = 0; ii < cnt; ii++)
		{
			if (band == BAND_2_4GHZ)
			{
				w_adapter->os_intf_ops->onebox_memset(&w_adapter->TxPower11BG[ii], 0, sizeof(CHAN_PWR_TABLE));
				low_indx  = (uint16 *) ((&bg_power_set[0] + (BG_PWR_VAL_LEN * ii)));
				med_indx  = (uint16 *) ((&bg_power_set[0] + (BG_PWR_VAL_LEN * ii) + 1));
				high_indx = (uint16 *) ((&bg_power_set[0] + (BG_PWR_VAL_LEN * ii) + 2));

				w_adapter->TxPower11BG[ii].mid_pwr = *med_indx;
				w_adapter->TxPower11BG[ii].low_pwr = *low_indx;
				w_adapter->os_intf_ops->onebox_memcpy(&w_adapter->TxPower11BG[ii].high_pwr[0], 
				                                      high_indx, 
				                                      (BG_PWR_VAL_LEN - 2) * 2);
			}
			else
			{
				w_adapter->os_intf_ops->onebox_memset(&w_adapter->TxPower11A[ii],  0, sizeof(CHAN_PWR_TABLE));
				if (ii < 5)
				{
					/* Mapping between indices and channels is as follows
					 * 0 - 36 channel, 1 - 60 channel, 2 - 157 channel
					 * 3 - 120 channel, 4 - 11j channel
					 */
					alow_indx  = ((&an_power_set[0] + (AN_PWR_VAL_LEN * ii)));
					amed_indx  = ((&an_power_set[0] + (AN_PWR_VAL_LEN * ii) + 1));
					ahigh_indx = ((&an_power_set[0] + (AN_PWR_VAL_LEN * ii) + 2));
					w_adapter->TxPower11A[ii].mid_pwr  = *amed_indx;
					w_adapter->TxPower11A[ii].low_pwr  = *alow_indx;
					w_adapter->os_intf_ops->onebox_memcpy(&w_adapter->TxPower11A[ii].high_pwr[0],
					                                    ahigh_indx, 
					                                    (AN_PWR_VAL_LEN - 2) * 2);
				}
				else
				{
					/* For index 5 onwards, a single word of info is given, which needs
					 * to be applied to all rates and all pwr profiles. Mapping between
					 * channels and indices is as follows:
					 * 5 - 64 channel; 6 - 100 channel; 7 - 140 channel 
					 */
					ahigh_indx = (&an_power_set[0] + (AN_PWR_VAL_LEN * 5) + (ii - 5));
					/*  - Anyways PD values are ignored as of now */
					w_adapter->os_intf_ops->onebox_memset(&w_adapter->TxPower11A[ii], 
					                                    *ahigh_indx, 
					                                    (AN_PWR_VAL_LEN * 2));
				} /* End if <condition> */
			} /* End if <condition> */
		} /* End for loop */
	}
	else 
	{
		for (ii = 0; ii < 3; ii++)
		{
			w_adapter->os_intf_ops->onebox_memset(&w_adapter->TxPower11BG[ii], 0, sizeof(CHAN_PWR_TABLE));
			w_adapter->os_intf_ops->onebox_memset(&w_adapter->TxPower11A[ii],  0, sizeof(CHAN_PWR_TABLE));
			if (w_adapter->eeprom_type == EEPROM_VER1)
			{
				/* New EEPROM Map */
				low_indx  = (uint16 *) ((&bg_power_set[0] + (BG_PWR_VAL_LEN * ii)));
				med_indx  = (uint16 *) ((&bg_power_set[0] + (BG_PWR_VAL_LEN * ii) + 1));
				high_indx = (uint16 *) ((&bg_power_set[0] + (BG_PWR_VAL_LEN * ii) + 2));
				    
				w_adapter->TxPower11BG[ii].mid_pwr = *med_indx;
				w_adapter->TxPower11BG[ii].low_pwr = *low_indx;
				w_adapter->os_intf_ops->onebox_memcpy(&w_adapter->TxPower11BG[ii].high_pwr[0], 
				                                    high_indx, 
				                                    (BG_PWR_VAL_LEN - 2) * 2);
				
				if (w_adapter->RFType == ONEBOX_RF_8230)
				{
					alow_indx  = ((&an_power_set[0] + (AN_PWR_VAL_LEN * ii)));
					amed_indx  = ((&an_power_set[0] + (AN_PWR_VAL_LEN * ii) + 1));
					ahigh_indx = ((&an_power_set[0] + (AN_PWR_VAL_LEN * ii) + 2));
					w_adapter->TxPower11A[ii].mid_pwr  = *amed_indx;
					w_adapter->TxPower11A[ii].low_pwr  = *alow_indx;
					w_adapter->os_intf_ops->onebox_memcpy(&w_adapter->TxPower11A[ii].high_pwr[0],
					                                    ahigh_indx, 
					                                    (AN_PWR_VAL_LEN - 2) * 2);
				}
			}
			else
			{
				if (w_adapter->RFType == ONEBOX_RF_8230)
				{
					if (w_adapter->operating_band == BAND_5GHZ)
					{
						w_adapter->power_mode = ONEBOX_PWR_HIGH;
					}
					ahigh_indx   = ((&an_power_set[0] + (LEGACY_AN_PWR_VAL_LEN * ii)));
					w_adapter->os_intf_ops->onebox_memcpy(&w_adapter->TxPower11A[ii].high_pwr[0], 
					                                    ahigh_indx, 
					                                    LEGACY_AN_PWR_VAL_LEN * 2);
				}
				
				low_indx  = (uint16 *) ((&bg_power_set[0] + (LEGACY_BG_PWR_VAL_LEN * ii)));
				med_indx  = (uint16 *) ((&bg_power_set[0] + (LEGACY_BG_PWR_VAL_LEN * ii) + 1));
				high_indx = (uint16 *) ((&bg_power_set[0] + (LEGACY_BG_PWR_VAL_LEN * ii) + 2));
				
				w_adapter->TxPower11BG[ii].low_pwr = *low_indx;
				w_adapter->TxPower11BG[ii].mid_pwr = *med_indx;
				w_adapter->os_intf_ops->onebox_memcpy(&w_adapter->TxPower11BG[ii].high_pwr[0], 
				                                    high_indx, 
				                                    (LEGACY_BG_PWR_VAL_LEN - 2) * 2);
			} /* End if <condition> */
		} /* End for loop */
	} /* End if <condition> */
	return;
} 

ONEBOX_STATUS set_beacon_ssid_notification(struct ieee80211vap *vap, uint8 loaded)
{
	WLAN_ADAPTER w_adapter = NULL;

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("[%s][%d] : <======= Entry to %s =======>\n "),	__func__, __LINE__, __func__));
	w_adapter = (WLAN_ADAPTER)vap->hal_priv_vap->hal_priv_ptr;
	if(w_adapter)	
	{
		w_adapter->beacon_ssid_loaded = loaded;
		return ONEBOX_STATUS_SUCCESS;
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("[%s][%d] : <======= No w_adapter found =======>\n "),	__func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}
}
ONEBOX_STATUS set_channel_change_notification(struct ieee80211vap *vap, uint8 change)
{
	WLAN_ADAPTER w_adapter = NULL;

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("[%s][%d] : <======= Entry to %s =======>\n "),	__func__, __LINE__, __func__));
	w_adapter = (WLAN_ADAPTER)vap->hal_priv_vap->hal_priv_ptr;
	if(w_adapter)	
	{
		w_adapter->channel_change = change;
		return ONEBOX_STATUS_SUCCESS;
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("[%s][%d] : <======= No w_adapter found =======>\n "),	__func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}
}
ONEBOX_STATUS set_vap_capabilities(struct ieee80211vap *vap, uint8 vap_status)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	WLAN_ADAPTER w_adapter = NULL;
	uint8 opmode ;
	struct ieee80211com *ic = NULL;

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);


	if(is_vap_valid(vap) < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: VAP is Not a valid pointer In %s Line %d, So returning Recvd vap_status %d\n "), 
								__func__, __LINE__, vap_status));
		dump_stack();
		return ONEBOX_STATUS_FAILURE;

	}

	w_adapter = (WLAN_ADAPTER)vap->hal_priv_vap->hal_priv_ptr;
	ic = &w_adapter->vap_com;
	opmode = vap->iv_opmode;

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, 256);
	switch(opmode)
	{
		case IEEE80211_M_STA:
			opmode = IEEE80211_OP_STA;
		break;
		case IEEE80211_M_HOSTAP:
			opmode = IEEE80211_OP_AP;
			//w_adapter->sta_data_block = 0; //: Handle during Multiple Vaps; Kept this for P2P mode
		break;
		case IEEE80211_M_IBSS:
			opmode = IEEE80211_OP_IBSS;
		break;
		default:
	/* : In case of P2P after connection if the device becomes GO indicate opmode in some way to firmware */
			opmode = IEEE80211_OP_P2P_GO;
		break;
	}

	/* Fill the length & host queue num */	
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(sizeof(mgmt_frame->u.vap_caps) | (ONEBOX_WIFI_MGMT_Q << 12));
	/* Fill the frame type */
	mgmt_frame->desc_word[1] = (ONEBOX_CPU_TO_LE16(VAP_CAPABILITIES));
	mgmt_frame->desc_word[2] |= (ONEBOX_CPU_TO_LE16 (vap_status) << 8);
	mgmt_frame->desc_word[2] |= (ONEBOX_CPU_TO_LE16(vap_status) >> 8);
#ifdef ENABLE_P2P_SUPPORT
	if (vap->p2p_enable) {
		if (vap->p2p_mode == IEEE80211_P2P_GO) {
			mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(IEEE80211_OP_P2P_GO | w_adapter->ch_bandwidth << 8);
		} else {
			mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(IEEE80211_OP_P2P_CLIENT | w_adapter->ch_bandwidth << 8);
		}
	} else 
#endif
		mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(opmode | w_adapter->ch_bandwidth << 8);
	/* : Fill antenna Info here */
	mgmt_frame->desc_word[5] = 0;  
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In func %s Line %d Vap_id %d \n"), __func__, __LINE__, vap->hal_priv_vap->vap_id));
	mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16((vap->hal_priv_vap->vap_id << 8) | (w_adapter->mac_id << 4) | w_adapter->radio_id);
	//mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16((vap_id << 8) | (w_adapter->mac_id << 4) | w_adapter->radio_id);

	/*Frame Body*/	
	//w_adapter->os_intf_ops->onebox_memcpy(mgmt_frame->u.vap_caps.mac_addr, w_adapter->mac_addr, IEEE80211_ADDR_LEN);
	w_adapter->os_intf_ops->onebox_memcpy(mgmt_frame->u.vap_caps.mac_addr, vap->iv_myaddr, IEEE80211_ADDR_LEN);
	//Default value for keep alive period is 90secs.
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In func %s Line %d keep_alive_period %d \n"), __func__, __LINE__, vap->iv_keep_alive_period));
	mgmt_frame->u.vap_caps.keep_alive_period = ONEBOX_CPU_TO_LE16(vap->iv_keep_alive_period);; 
	//w_adapter->os_intf_ops->onebox_memcpy(mgmt_frame->u.vap_caps.bssid, bssid, IEEE80211_ADDR_LEN);
	if(ic->ic_flags & IEEE80211_F_USEPROT)
	{
		/* Enable this bit if a non erp station is present or if the sizeof the pkt is greater than RTS threshold*/
		if(ic->ic_nonerpsta)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Enabling self cts bit\n")));
			mgmt_frame->u.vap_caps.flags |= ONEBOX_CPU_TO_LE32(ONEBOX_SELF_CTS_ENABLE);
		}
	}
	mgmt_frame->u.vap_caps.frag_threshold = ONEBOX_CPU_TO_LE16(2346); 
	mgmt_frame->u.vap_caps.rts_threshold = ONEBOX_CPU_TO_LE16(vap->iv_rtsthreshold);
	mgmt_frame->u.vap_caps.default_mgmt_rate_bbpinfo = ONEBOX_CPU_TO_LE32(RSI_RATE_6);
	mgmt_frame->u.vap_caps.beacon_miss_threshold = ONEBOX_CPU_TO_LE16(vap->iv_bmissthreshold);
	if(w_adapter->operating_band == BAND_5GHZ)
	{
			mgmt_frame->u.vap_caps.default_ctrl_rate_bbpinfo = ONEBOX_CPU_TO_LE32(RSI_RATE_6);
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("vap->iv_flags_ht = %x IEEE80211_FHT_USEHT40 = %x\n"),vap->iv_flags_ht, IEEE80211_FHT_USEHT40));
	}
	else
	{
			/* 2.4 Ghz band */
			mgmt_frame->u.vap_caps.default_ctrl_rate_bbpinfo = ONEBOX_CPU_TO_LE32(RSI_RATE_1);
	}

	if(ic->band_flags & IEEE80211_CHAN_HT40) {
		if(ic->band_flags & IEEE80211_CHAN_HT40U) {
			/* primary channel is below secondary channel */
			mgmt_frame->u.vap_caps.default_ctrl_rate_bbpinfo |= ONEBOX_CPU_TO_LE32((LOWER_20_ENABLE >> 12) << 16);
		} else {
			/* primary channel is above secondary channel */
			mgmt_frame->u.vap_caps.default_ctrl_rate_bbpinfo |= ONEBOX_CPU_TO_LE32(((UPPER_20_ENABLE >> 12) << 16));
		}
	}

	mgmt_frame->u.vap_caps.default_data_rate_bbpinfo = ONEBOX_CPU_TO_LE32(0);
	mgmt_frame->u.vap_caps.beacon_interval = ONEBOX_CPU_TO_LE16(ic->ic_bintval);
	mgmt_frame->u.vap_caps.dtim_period = ONEBOX_CPU_TO_LE16(vap->iv_dtim_period);
	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
	             (TEXT("===> Sending Vap Capabilities Packet <===\n")));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)mgmt_frame, sizeof(mgmt_frame->u.vap_caps) + FRAME_DESC_SZ);
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (sizeof(mgmt_frame->u.vap_caps) + FRAME_DESC_SZ));
	if(status)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unable to send vap capabilities  frame\n")));
		return ONEBOX_STATUS_FAILURE;
	}
	return ONEBOX_STATUS_SUCCESS;
}



ONEBOX_STATUS onebox_send_vap_dynamic_update_indication_frame(struct ieee80211vap *vap)
{

 	struct dynamic_s *dynamic_frame=NULL;
	struct ieee80211com * ic=NULL;
	WLAN_ADAPTER w_adapter =NULL;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);
	
	w_adapter = (WLAN_ADAPTER)vap->hal_priv_vap->hal_priv_ptr;
	ic = &w_adapter->vap_com;

	dynamic_frame = (struct dynamic_s *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(dynamic_frame, 0, 256);

	dynamic_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((sizeof(dynamic_frame->frame_body)) | (ONEBOX_WIFI_MGMT_Q << 12));
	dynamic_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(VAP_DYNAMIC_UPDATE); 
	dynamic_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(vap->iv_rtsthreshold);
	//dynamic_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(vap->iv_fragthreshold );
#ifdef ONEBOX_CONFIG_WOWLAN
#define ONEBOX_WOWLAN_BMISS_THRESHOLD		24
#define ONEBOX_WOWLAN_KEEP_ALIVE_PERIOD		5
	if (w_adapter->wowlan_enabled) {
		dynamic_frame->desc_word[6] =
			ONEBOX_CPU_TO_LE16(ONEBOX_WOWLAN_BMISS_THRESHOLD);
		dynamic_frame->frame_body.keep_alive_period =
			ONEBOX_CPU_TO_LE16(ONEBOX_WOWLAN_KEEP_ALIVE_PERIOD);
	} else {
#endif
		dynamic_frame->desc_word[6] =
			ONEBOX_CPU_TO_LE16(vap->iv_bmissthreshold);
		dynamic_frame->frame_body.keep_alive_period =
			ONEBOX_CPU_TO_LE16(vap->iv_keep_alive_period);
#ifdef ONEBOX_CONFIG_WOWLAN
	}
#endif

	if(ic->ic_flags & IEEE80211_F_USEPROT) {

			if((ic->ic_nonerpsta && (vap->iv_opmode == IEEE80211_M_HOSTAP)) 
			   || (vap->iv_opmode == IEEE80211_M_STA)) {
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Enabling self cts bit\n")));
				dynamic_frame->desc_word[2] |= ONEBOX_CPU_TO_LE32(ONEBOX_SELF_CTS_ENABLE);
			}
	}

	if(vap->hal_priv_vap->fixed_rate_enable) {
		dynamic_frame->desc_word[3] |= ONEBOX_CPU_TO_LE16(ONEBOX_FIXED_RATE_EN); //Fixed rate is enabled
		dynamic_frame->frame_body.data_rate = ONEBOX_CPU_TO_LE16(vap->hal_priv_vap->rate_hix);
	}

	dynamic_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16((vap->hal_priv_vap->vap_id << 8));
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("IN %s Line %d VAP_ID %d\n"), __func__, __LINE__, vap->hal_priv_vap->vap_id));

	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)dynamic_frame, (sizeof(struct dynamic_s)));

	if(status) {
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Unable to send vap dynamic update indication frame\n")));
		return ONEBOX_STATUS_FAILURE;
	}
	
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function load the station update frame to PPE
 *
 * @param
 *  hal_info Pointer to the hal information structure
 * @param
 *  sta_offset sta_id of the station
 * @param
 *  data Station update information buffer
 * @param
 *  len Length of the station update frame
 *
 * @return
 *  This function returns ONEBOX_STATUS_SUCCESS if template loading
 *  is successful otherwise ONEBOX_STATUS_FAILURE.
 */
ONEBOX_STATUS hal_load_key(WLAN_ADAPTER w_adapter,
                           uint8 *data, 
                           uint16 key_len, 
                           uint16 sta_id,
                           uint8 key_type,
                           uint8 key_id,
                           uint32 cipher,
                           struct ieee80211vap *vap)
{
	onebox_mac_frame_t *mgmt_frame;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE]; 
	ONEBOX_STATUS status;
	uint8 key_t1 = 0;
	uint16 key_descriptor = 0;
	//struct ieee80211com *ic = NULL;
	uint32 vap_id = vap->hal_priv_vap->vap_id;
	FUNCTION_ENTRY(ONEBOX_ZONE_INFO);
	
	//ic = &w_adapter->vap_com;
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, 256);
	switch (key_type) 
	{
		case ONEBOX_GROUP_KEY:
			/* Load the key into PPE*/
			key_t1 = 1 << 1;
			if(vap->iv_opmode == IEEE80211_M_HOSTAP)
			{
				key_descriptor = ONEBOX_BIT(7);
			}
			else {
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<==== Recvd Group Key ====>\n")));
				if((sta_id >= w_adapter->max_stations_supported) || !(w_adapter->sta_connected_bitmap[sta_id/32] & (BIT(sta_id%32)))) {
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT( " Invalid Sta_id %d in %s Line %d \n"),sta_id, __func__, __LINE__));
						return -1;
				}
			}
			break;
		case ONEBOX_PAIRWISE_KEY:
			/* Load the key into PPE */
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<==== Recvd Pairwise Key ====>\n")));
				if((sta_id >= w_adapter->max_stations_supported) || !(w_adapter->sta_connected_bitmap[sta_id/32] & (BIT(sta_id%32)))) {
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT( "ERROR: Invalid Sta_id %d in %s Line %d \n"),sta_id, __func__, __LINE__));
						return -1;
				
				}
			key_t1 = 0 << 1;
			if(cipher != IEEE80211_CIPHER_WEP)
			{
				key_id = 0;
			}
			break;
	}
	key_descriptor |= key_t1 | ONEBOX_BIT(13) | (key_id << 14);
	if(cipher == IEEE80211_CIPHER_WEP)
	{
		key_descriptor |= ONEBOX_BIT(2);
		if(key_len >= 13)
		{
			key_descriptor |= ONEBOX_BIT(3);
		}
	}
	else if(cipher != IEEE80211_CIPHER_NONE)
	{
		key_descriptor |= ONEBOX_BIT(4);
		if(cipher == IEEE80211_CIPHER_TKIP)
		{
			key_descriptor |= ONEBOX_BIT(5);
		}
	}
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(sizeof(mgmt_frame->u.set_key) | (ONEBOX_WIFI_MGMT_Q << 12));
	mgmt_frame->desc_word[1] = (ONEBOX_CPU_TO_LE16(SET_KEY));
//	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16((key_t1) | (1<<13)| (key_id <<14) | (1<<4) | (1<<5));
//	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16((key_descriptor) | (1<<5) | (1<<4));
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16((key_descriptor));

	mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16((sta_id));
	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(vap_id << 8);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d sta_id %d vap_id %d key_type %d\n"), __func__, __LINE__, sta_id, vap_id, key_type ));
	
	//ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("In %s %d \n"), __func__, __LINE__));
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)data, FRAME_DESC_SZ);

	//w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.set_key.key, data, 16);
	//w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.set_key.tx_mic_key, &data[16], 16);
	//w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.set_key.rx_mic_key, &data[24], 8);
	//w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.set_key.key, data, sizeof(mgmt_frame->u.set_key));
	if(data) {
    if(cipher != IEEE80211_CIPHER_WEP) //selecting key_id = 0 for WPA and WPA2 modes but for WEP, key_id is selected from supplicant
        key_id = 0;
	w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.set_key.key[key_id], data, 4*32);
	w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.set_key.tx_mic_key, &data[16], 8);
	w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.set_key.rx_mic_key, &data[24], 8);
	} else {
		w_adapter->os_intf_ops->onebox_memset(&mgmt_frame->u.set_key, 0, sizeof(mgmt_frame->u.set_key));
	}

	//mgmt_frame->u.set_key.key = data;
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, sizeof(mgmt_frame->u.set_key) + FRAME_DESC_SZ);

	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (sizeof(mgmt_frame->u.set_key)+ FRAME_DESC_SZ));
	if(status)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Unable to load the keys frame\n")));
		return ONEBOX_STATUS_FAILURE;
	}


	if(vap->iv_opmode == IEEE80211_M_STA) {
		if((key_type == ONEBOX_PAIRWISE_KEY)) 
		{
			w_adapter->sta_mode.ptk_key = 1;
		}	
		else if((key_type == ONEBOX_GROUP_KEY)) 
		{
			w_adapter->sta_mode.gtk_key = 1;

		}

		if((w_adapter->sta_mode.ptk_key) && 
		   (w_adapter->sta_mode.eapol4_cnfrm) && 
		   (w_adapter->sta_data_block)) {
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d  eapol %d ptk %d sta_block %d ap_block %d\n"), __func__, __LINE__,
								w_adapter->sta_mode.eapol4_cnfrm, w_adapter->sta_mode.ptk_key, w_adapter->sta_data_block, w_adapter->block_ap_queues)); 
			onebox_send_block_unblock(vap, STA_CONNECTED, 0);
		}
		if(w_adapter->sta_mode.ptk_key && w_adapter->sta_mode.gtk_key && w_adapter->sta_mode.eapol4_cnfrm) {
			//onebox_send_sta_supported_features(vap, w_adapter);
			ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("calling timeout initialziation In %s Line %d\n"), __func__, __LINE__));
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Reseting ptk key variable in %s Line %d \n"), __func__, __LINE__));
			w_adapter->sta_mode.ptk_key = 0; 
			//initialize_sta_support_feature_timeout(vap, w_adapter);
			update_pwr_save_status(vap, PS_ENABLE, CONNECTED_PATH);

		}
	}

	return ONEBOX_STATUS_SUCCESS;
}


/* This function sends bootup parameters frame to TA.
 * @param pointer to driver private structure
 * @return 0 if success else -1. 
 */   
uint8 onebox_load_bootup_params(WLAN_ADAPTER w_adapter)
{         
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
	             (TEXT("===> Sending Bootup parameters Packet <===\n")));

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, 256);
	if (w_adapter->operating_chwidth == BW_40Mhz)
	{ 
		if (w_adapter->device_model == RSI_DEV_9116)
	     		w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bootup_params_9116,
				                            &boot_params_9116_40,
				                            sizeof(BOOTUP_PARAMETERS_9116));
		else
    			w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bootup_params,
				                            &boot_params_40,
				                            sizeof(BOOTUP_PARAMETERS));
		ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
				(TEXT("===> Sending Bootup parameters Packet 40MHZ <=== %d\n"),UMAC_CLK_40BW));
		mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(UMAC_CLK_40BW);
	}
#ifdef MODE_11AH
	else if(w_adapter->operating_chwidth == BW_4Mhz)
	{
	w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bootup_params,
				                            &boot_params_4,
				                            sizeof(BOOTUP_PARAMETERS));
		ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
				(TEXT("===> Sending Bootup parameters Packet 4MHZ <=== %d\n"),UMAC_CLK_20BW));
		mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(UMAC_CLK_20BW);

	}
	else if(w_adapter->operating_chwidth == BW_2Mhz)
	{

	w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bootup_params,
				                            &boot_params_2,
				                            sizeof(BOOTUP_PARAMETERS));
		ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
				(TEXT("===> Sending Bootup parameters Packet 2MHZ <=== %d\n"),UMAC_CLK_20BW));
		mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(UMAC_CLK_20BW);

	}
#endif

	else if( w_adapter->operating_chwidth == BW_20Mhz )
	{
		if (w_adapter->device_model == RSI_DEV_9116)
			w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bootup_params_9116,
				                            &boot_params_9116_20,
				                            sizeof(BOOTUP_PARAMETERS_9116));
		else
	  		w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bootup_params,
				                            &boot_params_20,
				                            sizeof(BOOTUP_PARAMETERS));

		if (w_adapter->device_model == RSI_DEV_9116) {
			if (boot_params_9116_20.valid == VALID_20)
			{
				mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(UMAC_CLK_20BW);
				ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
					(TEXT("===> Sending Bootup parameters Packet 20MHZ <=== %d \n"),UMAC_CLK_20BW));
			}	
    		}
		else if(boot_params_20.valid == VALID_20) {

			mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(UMAC_CLK_20BW);
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
					(TEXT("===> Sending Bootup parameters Packet 20MHZ <=== %d \n"),UMAC_CLK_20BW));

		}
		else
		{
			//: This should not occur need to remove
			mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(UMAC_CLK_40MHZ);
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
					(TEXT("===>ERROR: Sending Bootup parameters Packet for  40MHZ  In %s Line %d <=== %d \n"),__func__, __LINE__, UMAC_CLK_40MHZ));
		}	
	}
	else if(w_adapter->operating_chwidth == BW_10Mhz)
	{
		if (!(w_adapter->device_model == RSI_DEV_9116)) {    
			w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bootup_params,
				                            &boot_params_10,
				                            sizeof(BOOTUP_PARAMETERS));
			ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
				(TEXT("===> Sending Bootup parameters Packet 10MHZ <=== %d\n"),UMAC_CLK_40BW));
			mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(UMAC_CLK_40BW);
		}
	}
	/* Bit{0:11} indicates length of the Packet
 	 * Bit{12:15} indicates host queue number
 	 */

	if (w_adapter->device_model == RSI_DEV_9116) {
		mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(sizeof(BOOTUP_PARAMETERS_9116) | (ONEBOX_WIFI_MGMT_Q << 12));
		mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BOOTUP_PARAMS_REQUEST);
		status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (sizeof(BOOTUP_PARAMETERS_9116)+ FRAME_DESC_SZ));
	}
	else {
		mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(sizeof(BOOTUP_PARAMETERS) | (ONEBOX_WIFI_MGMT_Q << 12));
		mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BOOTUP_PARAMS_REQUEST);

		//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)mgmt_frame, (FRAME_DESC_SZ + sizeof(BOOTUP_PARAMETERS)));
		status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (sizeof(BOOTUP_PARAMETERS)+ FRAME_DESC_SZ));
	}

	return status;
} /*end of onebox_load_bootup_params */
EXPORT_SYMBOL(onebox_load_bootup_params);

/**
 * This function prepares reset MAC request frame and send it to LMAC.
 *
 * @param  Pointer to Adapter structure.  
 * @return 0 if success else -1. 
 */
ONEBOX_STATUS onebox_send_reset_mac(WLAN_ADAPTER w_adapter)
{
	struct driver_assets *d_assets = w_adapter->d_assets;
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[FRAME_DESC_SZ];

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
	             (TEXT("===> Send Reset Mac frame <===\n")));

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

	/* Bit{0:11} indicates length of the Packet
 	 * Bit{12:16} indicates host queue number
 	 */ 

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12);
	/* Fill frame type for reset mac request */
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(RESET_MAC_REQ);
	if (w_adapter->Driver_Mode == RF_EVAL_MODE_ON)
	{
		mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(1); //Value is (2 - 1)
	}
	else if (w_adapter->Driver_Mode == SNIFFER_MODE)
	{
		mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(1);
	}
	if (w_adapter->calib_mode)
	{
		mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16((1) << 8); 
	}
	if (w_adapter->per_lpbk_mode)
	{
		mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16((2) << 8);
	}
#ifdef BYPASS_RX_DATA_PATH
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(0x0001);
#endif
	mgmt_frame->desc_word[4] |= ONEBOX_CPU_TO_LE16(d_assets->retry_count << 8);
		
	/*TA level aggregation of pkts to host */
	if(w_adapter->Driver_Mode == SNIFFER_MODE)
		mgmt_frame->desc_word[3] |=  ONEBOX_CPU_TO_LE16(1 << 8);
	else
		mgmt_frame->desc_word[3] |=  ONEBOX_CPU_TO_LE16(d_assets->ta_aggr << 8);

	if (d_assets->antenna_diversity)
		mgmt_frame->desc_word[6] = ONEBOX_CPU_TO_LE16(d_assets->antenna_diversity);
		
	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
	             (TEXT("===> Sending TXPkt Lifetime as %d <===\n"), d_assets->txpkt_lifetime));
	
	mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(d_assets->txpkt_lifetime);
	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_MGMT_SEND, (PUCHAR)mgmt_frame, (FRAME_DESC_SZ));
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, FRAME_DESC_SZ);

	return status;
} /*end of onebox_send_reset_mac */

ONEBOX_STATUS set_channel(WLAN_ADAPTER w_adapter, uint16 chno)
{
	/* Prepare the scan request using the chno information */
	struct ieee80211com *ic = NULL;
	onebox_mac_frame_t *mgmt_frame;
	struct ieee80211vap *vap = NULL;
	int32 status = 0;
	uint16 frame[256];
	uint16 ch_num;
	ch_num = chno;
#ifndef PROGRAMMING_SCAN_TA
	uint16 *rf_prog_vals;
	uint16 vals_per_set;
	uint8 count;
#endif
	ic = &w_adapter->vap_com;
	
	FUNCTION_ENTRY(ONEBOX_ZONE_INIT);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("%s: Sending scan req frame\n"), __func__));
	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)frame;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, 256);

	
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(SCAN_REQUEST);
#ifdef CONFIG_ACS	
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
	{
		if(vap->iv_opmode == IEEE80211_M_HOSTAP && vap->hal_priv_vap->acs_enable) {
			mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(1 << 8);    //ACS_DEBUG :Enable timer
			mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(15);      //ACS_DEBUG: Set ACS_TIMEOUT
			mgmt_frame->desc_word[3] |= ONEBOX_CPU_TO_LE16(ACS_TIMEOUT_TIME << 8);    //ACS_TIMEOUT: 100ms
			break;
		}
	}
#endif
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(chno);/*channel num is required */
	
#ifdef RF_8111
	mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (RSI_RF_TYPE << 4));
#else
	/* RF type here is 8230 */
	mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (NONRSI_RF_TYPE << 4));
#endif
	//mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(radio_id << 8);				 
	

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("scan values in device ch_bandwidth = %d ch_num %d \n"), w_adapter->operating_chwidth, ch_num));

	if (w_adapter->Driver_Mode == RF_EVAL_MODE_ON) 
	{
		w_adapter->ch_power = w_adapter->endpoint_params.power;
		mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(ch_num);
	} 
	else if (w_adapter->Driver_Mode == SNIFFER_MODE)
	{
		w_adapter->ch_power = w_adapter->endpoint_params.power;
		mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(ch_num);
   		ic->ic_curchan->ic_ieee = ch_num;
		if(ch_num == 14)
			ic->ic_curchan->ic_freq = 2484;
		else if (ch_num < 14)
			ic->ic_curchan->ic_freq = 2407 + ch_num * 5;
		else if (ch_num >= 36)
			ic->ic_curchan->ic_freq = 5000 + ch_num * 5;
		if( w_adapter->endpoint_params.enable_11j )
		{
			if( ch_num <=16 )
				ic->ic_curchan->ic_freq = 5000 + ch_num * 5;
			else
				ic->ic_curchan->ic_freq = 4000 + ch_num * 5;
		}
	}
	else
	{  
		//w_adapter->ch_power = ic->ic_curchan->ic_maxregpower;
		if(ic->ic_txpowlimit > ic->ic_curchan->ic_maxregpower)
		{
			w_adapter->ch_power = ic->ic_curchan->ic_maxregpower;
		}else{
			w_adapter->ch_power = ic->ic_txpowlimit;
		}
		mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(ic->ic_curchan->ic_ieee);
	}
	mgmt_frame->desc_word[4] |= ONEBOX_CPU_TO_LE16((int8)w_adapter->ant_gain[0] << 8);
  	mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16((int8)w_adapter->ant_gain[1]);
  	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ANT GAIN is 2G %02x 5G %02x \n"), mgmt_frame->desc_word[4], mgmt_frame->desc_word[5]));

	mgmt_frame->desc_word[6] = ONEBOX_CPU_TO_LE16(w_adapter->ch_power & 0xff);/*POWER VALUE*/
	
	if (w_adapter->Driver_Mode == RF_EVAL_MODE_ON) 
	{
			if( w_adapter->endpoint_params.enable_11j )
					mgmt_frame->desc_word[6] |= (ONEBOX_CPU_TO_LE16(1 << 8 ));
	}
	else if( ic->ic_curchan->ic_flags & IEEE80211_CHAN_11J ) 
	{
			mgmt_frame->desc_word[6] |= (ONEBOX_CPU_TO_LE16(1 << 8 ));
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("IN %s Line %d TX_PWER is %d ifname is %s\n"), __func__, __LINE__, w_adapter->ch_power, ic->ic_ifp->name));
	if(w_adapter->operating_chwidth == BW_40Mhz)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("40Mhz is enabled\n")));
		mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16(0x1 << 8);/*scan values in TA */
	}
	else if(w_adapter->operating_chwidth == BW_10Mhz)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("10Mhz is enabled\n")));
		mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16(0x2 << 8);/*scan values in TA */
	}

	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
	{
		if(vap->iv_state != IEEE80211_S_RUN ) {
			if(vap->iv_opmode == IEEE80211_M_HOSTAP) {
				vap->cur_rate = 0x8b;
			} else if (vap->iv_opmode == IEEE80211_M_STA) {
				vap->iv_bss->ni_txrate = 0x8b;
			}
		}
	}
	mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(ic->ic_regdomain.pad[0]);

	if( w_adapter->max_pwr_enable ) {
			mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BIT(3) << 8);				 
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("[%s][%d] : w_adapter->spec_mask_set is %d\n", __func__, __LINE__, w_adapter->spec_mask_set));
	if(w_adapter->spec_mask_set) {
			mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16((BIT(1) << 8));   //Sending Spectral MAsk Enable Notification to LMAC.
	}
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12);
	status= onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame,  FRAME_DESC_SZ);

	FUNCTION_EXIT(ONEBOX_ZONE_INIT);
	return status;
}

/**
* This function sends the configuration values  to firmware
* @param 
*   w_adapter  Pointer to the driver private structure
*
* @returns 
*   ONEBOX_STATUS_SUCCESS on success, or corresponding negative
* error code on failure
*/
static ONEBOX_STATUS onebox_load_config_vals(WLAN_ADAPTER  w_adapter)
{

	uint16 onebox_config_vals[] = 
	{ 19,           /* num params */ 
	                      ONEBOX_CPU_TO_LE16(0x1),          /* internalPmuEnabled */
	                       ONEBOX_CPU_TO_LE16(0x0),          /* ldoControlRequired */
	                      ONEBOX_CPU_TO_LE16(0x9),          /* crystalFrequency values */
	                       /* 9 --> 40MHz, 8 --> 20MHz 7 --> 44MHz   6 --> 52MHz */
	                       /* 5 --> 26MHz, 4 --> 13MHz 2 --> 38.4MHz 1 --> 19.2MHz */
	                       /* 0 --> 9.6MHz */
	                      ONEBOX_CPU_TO_LE16(0x2D),         /* crystalGoodTime */
	                       0x0,          /* TAPLLEnabled  */
#ifndef USE_KTV_INTF
	                       0,            /* TAPLLFrequency */
#else
	                     ONEBOX_CPU_TO_LE16(40),           /* TAPLLFrequency */
#endif
	                    ONEBOX_CPU_TO_LE16(0x2),          /* sleepClockSource */
	                       ONEBOX_CPU_TO_LE16(0x0),          /* voltageControlEnabled */
#ifndef USE_KTV_INTF
	                   ONEBOX_CPU_TO_LE16(5200),         /* wakeupThresholdTime */
#else
	                       ONEBOX_CPU_TO_LE16(6500),         /* wakeupThresholdTime */
#endif
	                       0x0,          /* reserved */        
	                       0x0,          /* reserved */
	                       0x0,          /* reserved */
	                       0x0,          /* host based wkup enable */          
	                       0x0,          /* FR4 enable */
	                       0x0,          /* BT Coexistence */
	                       0x0,          /* host based interrupt enable */ 
	                       0x0,          /* host wakeup_active_high */
	                       0x0,          /* xtal_ip_enable */
	                       0x0           /* external PA*/
	};
	/* Driver Expecting Only 19 paramaeters */
	if (onebox_config_vals[0] == 19)
	{
		w_adapter->os_intf_ops->onebox_memcpy(&w_adapter->config_params, &onebox_config_vals[1], 38);
		w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)&w_adapter->config_params, 38);
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		    (TEXT("%s: Wrong Number of Config params found\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	} /* End if <condition> */

	return ONEBOX_STATUS_SUCCESS;
}

ONEBOX_STATUS onebox_umac_init_done (WLAN_ADAPTER w_adapter)
{

	struct driver_assets *d_assets = w_adapter->d_assets;
	if (d_assets->host_intf_type == HOST_INTF_USB) {
		// 
		//it is here as usb will disconnect and connect again. so w_adapter will be reset
		w_adapter->mac_addr[0] = 0;
		w_adapter->mac_addr[1] = 0x23;
		w_adapter->mac_addr[2] = 0xa7;
		w_adapter->mac_addr[3] = 0x04;
		w_adapter->mac_addr[4] = 0x02;
		w_adapter->mac_addr[5] = 0x48;
		w_adapter->operating_band = BAND_2_4GHZ;
		w_adapter->def_chwidth = BW_20Mhz;
		w_adapter->operating_chwidth = BW_20Mhz;
#ifdef RF_8111
		w_adapter->RFType = ONEBOX_RF_8111;
#else
		w_adapter->RFType = ONEBOX_RF_8230;
#endif    
		if (onebox_load_config_vals(w_adapter) != 0)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR ,
					(TEXT("%s: Initializing the Configuration vals Failed\n"), __func__));
			return ONEBOX_STATUS_FAILURE;
		}
	}

	w_adapter->core_ops->onebox_core_init(w_adapter);
	return ONEBOX_STATUS_SUCCESS;
}
EXPORT_SYMBOL(onebox_umac_init_done);

/**
 * This function is to set channel, Band and Channel Bandwidth
 * specified by user in PER mode.
 *
 * @param  Pointer to w_adapter structure.  
 * @param  Power Save Cmd.  
 * @return 0 if success else -1. 
 */
ONEBOX_STATUS band_check (WLAN_ADAPTER w_adapter)
{
	uint8 set_band = 0;
	uint8 previous_chwidth = 0;
	uint8 change_in_endpoint = 0;
#ifdef PROGRAMMING_BBP_TA	
	uint8 previous_endpoint = 0;
	previous_endpoint = w_adapter->endpoint;
#endif

	if(w_adapter->endpoint_params.channel <= 14 && (!w_adapter->endpoint_params.enable_11j))
	{
		set_band = BAND_2_4GHZ;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Band is 2GHz\n")));	
	}
	else if((w_adapter->endpoint_params.channel >= 36) && (w_adapter->endpoint_params.channel <= 165))
	{
		set_band = BAND_5GHZ;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Band is 5GHz\n")));	
	}
	else if((w_adapter->endpoint_params.channel >= 183) && (w_adapter->endpoint_params.channel <= 196))
	{
		set_band = BAND_4_9GHZ;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Band is 4.9GHz\n")));	
	}
	else if((w_adapter->endpoint_params.channel >= 7 && w_adapter->endpoint_params.channel <= 16) 
					&& (w_adapter->endpoint_params.enable_11j))
	{
		set_band = BAND_4_9GHZ;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Band is 4.9GHz\n")));	
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid channel issued by user\n")));	
		return ONEBOX_STATUS_FAILURE;
	}
	// per_ch_bw:  0 - 20MHz,  4 - Lower 40MHz, 2 - Upper 40MHz, 6 - Full 40MHz
	previous_chwidth = w_adapter->operating_chwidth;
	switch( w_adapter->endpoint_params.per_ch_bw )
	{
			case 0:
					w_adapter->operating_chwidth = BW_20Mhz;
					break;
			case 1:
			case 2:
			case 4:
			case 6:
					w_adapter->operating_chwidth = BW_40Mhz;
					break;
			case 5:
					w_adapter->operating_chwidth = BW_5Mhz;
					break;
			case 7:
					w_adapter->operating_chwidth = BW_10Mhz;
					break;
			default:
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s line %d Invalid BW settings\n"), __func__, __LINE__));					
			return ONEBOX_STATUS_FAILURE;
	}
#ifdef MODE_11AH

else if (w_adapter->endpoint_params.per_ch_bw == 2)
	{

		printk("\n\n\t  2MHZ");
		w_adapter->operating_chwidth = BW_2Mhz;
	}
else if (w_adapter->endpoint_params.per_ch_bw == 3)
	{
		printk("\n\n\t 4MHZ");
		w_adapter->operating_chwidth = BW_4Mhz;
	}

#endif



	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s line %d band %d band %d\n"), __func__, __LINE__, w_adapter->operating_band, set_band));
	if (w_adapter->operating_band != set_band)
	{
		change_in_endpoint = 1;
		w_adapter->operating_band = set_band;
	}
	if (set_band == BAND_2_4GHZ)
	{
		if (w_adapter->operating_chwidth == BW_40Mhz)
		{  
			w_adapter->endpoint = 1;
		}
		else
		{
			w_adapter->endpoint = 0;
		}
	}
	else if( set_band ==  BAND_5GHZ )
	{
		if (w_adapter->operating_chwidth == BW_40Mhz)
		{  
			w_adapter->endpoint = 3;
		}
		else if( w_adapter->operating_chwidth == BW_20Mhz)
		{
			w_adapter->endpoint = 2;
		}
	}

/* For 11AH Testing we are using channels 1 to 12*/

#ifdef MODE_11AH
		else if (w_adapter->operating_chwidth == BW_2Mhz)
		{
			w_adapter->endpoint = 9;
		} 
		else if (w_adapter->operating_chwidth == BW_4Mhz)
		{
			w_adapter->endpoint = 10;
		}
#endif
	if( set_band == BAND_4_9GHZ)
	{
		if (w_adapter->operating_chwidth == BW_40Mhz)
		{  
			w_adapter->endpoint = 3;
		}
		else if( w_adapter->operating_chwidth == BW_20Mhz)
		{
			w_adapter->endpoint = 2;
		}
		else if( w_adapter->operating_chwidth == BW_10Mhz)
		{
			w_adapter->endpoint = 4;
		}
		else if( w_adapter->operating_chwidth == BW_5Mhz)
		{
			w_adapter->endpoint = 5;
		}
	}

    if (w_adapter->device_model == RSI_DEV_9116) {
        if ((w_adapter->Driver_Mode >= RF_EVAL_MODE_ON) && (w_adapter->Driver_Mode <= RF_EVAL_LPBK)) {
            ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("%s: skipping this for 9116 \n"),__func__));
            return ONEBOX_STATUS_SUCCESS;
        }
    }

	if (w_adapter->operating_chwidth != previous_chwidth)
	{
		w_adapter->chw_flag = 1;
		change_in_endpoint = 1;
		if(onebox_load_bootup_params(w_adapter) == ONEBOX_STATUS_SUCCESS)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s: BOOTUP Parameters loaded successfully\n"),
						__FUNCTION__));
		}
		else
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: Failed to load bootup parameters\n"), 
						__FUNCTION__));
		}
		if(onebox_load_radio_caps(w_adapter))
		{
			return ONEBOX_STATUS_FAILURE;
		}
	}
  	else if ((w_adapter->operating_chwidth == BW_40Mhz) && 
          (w_adapter->primary_channel != w_adapter->endpoint_params.per_ch_bw))
  	{
	  	w_adapter->primary_channel = w_adapter->endpoint_params.per_ch_bw;
	  	if(onebox_load_radio_caps(w_adapter))
	  	{
			return ONEBOX_STATUS_FAILURE;
	  	}
  	}
	if ((w_adapter->endpoint != previous_endpoint) || (w_adapter->rf_power_mode_change))
	{
		w_adapter->rf_power_mode_change = 0;
		w_adapter->devdep_ops->onebox_program_bb_rf(w_adapter);
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" ENDPOINT : %d \n"),w_adapter->endpoint));
	}
	return change_in_endpoint;
}
ONEBOX_STATUS set_per_configurations (WLAN_ADAPTER w_adapter)
{
  uint8 status = 0 ;
    if(onebox_load_bootup_params(w_adapter) == ONEBOX_STATUS_SUCCESS)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s: BOOTUP Parameters loaded successfully\n"),
            __FUNCTION__));
    }
    else
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: Failed to load bootup parameters\n"), 
            __FUNCTION__));
    }
    status = onebox_load_radio_caps(w_adapter);
    if(status)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: Failed to Send Radio parameters\n"), 
            __FUNCTION__));
      return status;
    }
    if (w_adapter->device_model == RSI_DEV_9116) {
    status = onebox_send_w_9116_features(w_adapter);
    if(status)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: Failed to Send wlan_9116 features parameters\n"), 
            __FUNCTION__));
      return status;
    }
    }
    status = w_adapter->devdep_ops->onebox_program_bb_rf(w_adapter);
    if(status){ 
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: Failed to SEND BB_RF_INIT \n"), 
            __FUNCTION__));
      return status;
    }
  return status;
}
EXPORT_SYMBOL(set_per_configurations);

/**
 * This function programs BB and RF values provided 
 * using MATLAB.
 *
 * @param  Pointer to w_adapter structure.  
 * @param  Power Save Cmd.  
 * @return 0 if success else -1. 
 */
ONEBOX_STATUS set_bb_rf_values (WLAN_ADAPTER w_adapter, struct iwreq *wrq )
{
	uint8 i = 0;
	uint8 type = 0; 
	uint16 len = 0;
	uint8 rf_len = 0;
	int32 rt = 0;
	uint8 rw_type = 0;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("ONEBOX_read_bb_rf_values: Address  = %x  value = %d "),
				w_adapter->bb_rf_params.Data[0],
				w_adapter->bb_rf_params.value));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" No of values = %d No of fields = %d:\n"),w_adapter->bb_rf_params.no_of_values,w_adapter->bb_rf_params.no_of_fields));
	w_adapter->bb_rf_params.Data[0] =  w_adapter->bb_rf_params.no_of_values;
	type =  w_adapter->bb_rf_params.value;
	if (!(w_adapter->device_model == RSI_DEV_9116)) {
		for(i = 0; i <= w_adapter->bb_rf_params.no_of_values; i++)
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" bb_rf_params.Data[] = %x :\n"),w_adapter->bb_rf_params.Data[i]));
	}

	if(type % 2 == 0)
	{
		w_adapter->bb_rf_rw = 1; //set_read
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,  (TEXT(" *** read  \n")));
	}
	w_adapter->soft_reset = w_adapter->bb_rf_params.soft_reset;


	if(type == BB_WRITE_REQ || type == BB_READ_REQ )
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,  (TEXT("ONEBOX_read_buf_values : BB_REQ\n")));
		if(onebox_mgmt_send_bb_prog_frame(w_adapter, w_adapter->bb_rf_params.Data, w_adapter->bb_rf_params.no_of_values) != ONEBOX_STATUS_SUCCESS)
		{
			return ONEBOX_STATUS_FAILURE;
		}
	}
	else if((type == RF_WRITE_REQ) || (type == RF_READ_REQ)
			|| (type == ULP_READ_REQ) || (type == ULP_WRITE_REQ))
    {
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,  (TEXT("ONEBOX_read_buf_values : RF_REQ\n")));
        if (type == RF_WRITE_REQ)
        {  
            w_adapter->bb_rf_params.no_of_fields = 3;

            if(!(w_adapter->device_model == RSI_DEV_9116)) {
                if(onebox_mgmt_send_rf_prog_frame(w_adapter,&w_adapter->bb_rf_params.Data[2], w_adapter->bb_rf_params.no_of_values, w_adapter->bb_rf_params.no_of_fields, RSI_RF_TYPE ) != ONEBOX_STATUS_SUCCESS) {
                    return ONEBOX_STATUS_FAILURE;
                }
            } else {
                if(w_adapter->bb_rf_params.protocol_id == 0xff) {
                    if(onebox_mgmt_send_rf_prog_frame(w_adapter,&w_adapter->bb_rf_params.Data[2], w_adapter->bb_rf_params.no_of_values, w_adapter->bb_rf_params.no_of_fields, RSI_RF_TYPE ) != ONEBOX_STATUS_SUCCESS) {
                        return ONEBOX_STATUS_FAILURE;
                    }
                } else {
                    if(onebox_mgmt_send_9116_rf_prog_frame(w_adapter,&w_adapter->bb_rf_params.Data[2], w_adapter->bb_rf_params.no_of_values, w_adapter->bb_rf_params.no_of_fields, RSI_RF_TYPE ,w_adapter->bb_rf_params.protocol_id) != ONEBOX_STATUS_SUCCESS) {
                        return ONEBOX_STATUS_FAILURE;
                    }
                }
            }
        }
        else 
        {
            if(!(w_adapter->device_model == RSI_DEV_9116)) {
                if(onebox_mgmt_send_rf_prog_frame(w_adapter,&w_adapter->bb_rf_params.Data[1], w_adapter->bb_rf_params.no_of_values, w_adapter->bb_rf_params.no_of_fields, RSI_RF_TYPE ) != ONEBOX_STATUS_SUCCESS) {
                    return ONEBOX_STATUS_FAILURE;
                }
            } else {
                if(w_adapter->bb_rf_params.protocol_id == 0xff) {
                    if(onebox_mgmt_send_rf_prog_frame(w_adapter,&w_adapter->bb_rf_params.Data[1], w_adapter->bb_rf_params.no_of_values, w_adapter->bb_rf_params.no_of_fields, RSI_RF_TYPE ) != ONEBOX_STATUS_SUCCESS) {
                        return ONEBOX_STATUS_FAILURE;
                    }
                } else {
                    if(onebox_mgmt_send_9116_rf_prog_frame(w_adapter,&w_adapter->bb_rf_params.Data[1], w_adapter->bb_rf_params.no_of_values, w_adapter->bb_rf_params.no_of_fields, RSI_RF_TYPE ,w_adapter->bb_rf_params.protocol_id) != ONEBOX_STATUS_SUCCESS) {
                        return ONEBOX_STATUS_FAILURE;
                    } else {
                        if( w_adapter->d_assets->onebox_wait_for_rf_prog_frame_rsp(w_adapter->d_assets) < 0 ) {
                            return ONEBOX_STATUS_FAILURE;
                        }
                        wrq->u.data.length = sizeof(bb_rf_params_t);
                        if(copy_to_user(wrq->u.data.pointer, &(w_adapter->d_assets->common_bb_rf_params), sizeof(bb_rf_params_t))) {
                            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,  (TEXT( "onebox_ioctl: Failed to perform operation\n")));
                            return -EFAULT;
                        }
                        return ONEBOX_STATUS_SUCCESS;
                    }
                }
            }
        }
    } else if(type == BUF_READ_REQ || type == BUF_WRITE_REQ )
	{
                    
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,  (TEXT("ONEBOX_read_buf_values : BB_BUFFER\n")));
		rw_type = w_adapter->bb_rf_params.soft_reset >> 4;
		if( rw_type == 1 )
		{
			if(onebox_bb_buffer_request_direct(w_adapter, w_adapter->bb_rf_params.Data, w_adapter->bb_rf_params.no_of_values) != ONEBOX_STATUS_SUCCESS)
			{
				return ONEBOX_STATUS_FAILURE;
			}
		}else if (rw_type == 2 ) {
			if(onebox_bb_buffer_request_indirect(w_adapter, w_adapter->bb_rf_params.Data, w_adapter->bb_rf_params.no_of_values) != ONEBOX_STATUS_SUCCESS)
			{
				return ONEBOX_STATUS_FAILURE;
			}
		} else {
			printk("Invalid R/W type\n");
		}
	}
	else if(type == RF_LOOPBACK_M2 || type == RF_LOOPBACK_M3 )
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,  (TEXT("ONEBOX_read_buf_values : RF_LOOPBACK_REQ\n")));
		//    len =  wrq->u.data.length;
		if ( type == RF_LOOPBACK_M3 )
			len =  256*2;
		else
			len = 2048 * 2;
		w_adapter->rf_lpbk_len = 0;
		w_adapter->os_intf_ops->onebox_memset(&w_adapter->rf_lpbk_data[0], 0, len );
		if(onebox_rf_loopback(w_adapter, w_adapter->bb_rf_params.Data, w_adapter->bb_rf_params.no_of_values, type) != ONEBOX_STATUS_SUCCESS)
		{
			return ONEBOX_STATUS_FAILURE;
		}
	}
	else if(type == LMAC_REG_WRITE || type == LMAC_REG_READ )
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,  (TEXT("ONEBOX_read_buf_values : LMAC_REG_READ/WRITE\n")));
		if(onebox_mgmt_lmac_reg_ops_req(w_adapter, w_adapter->bb_rf_params.Data, type) != ONEBOX_STATUS_SUCCESS)
		{
			return ONEBOX_STATUS_FAILURE;
		}
	}
	else if(type == RF_RESET_REQ)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,  (TEXT("ONEBOX_read_buf_values : RF_RESET_REQ\n")));
		if(onebox_mgmt_send_rf_reset_req(w_adapter, w_adapter->bb_rf_params.Data) != ONEBOX_STATUS_SUCCESS)
		{
			return ONEBOX_STATUS_FAILURE;
		}
	}
	else if(type == EEPROM_RF_PROG_WRITE || type == EEPROM_RF_PROG_READ )
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,  (TEXT("ONEBOX_read_buf_values : EEPROM_RF_PROG_READ\n")));
//		if(eeprom_read(w_adapter, w_adapter->bb_rf_params.Data) != ONEBOX_STATUS_SUCCESS)
//		if(eeprom_read(w_adapter, cw_mode_buf_write_array) != ONEBOX_STATUS_SUCCESS)
		{
			return ONEBOX_STATUS_FAILURE;
		}
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,  (TEXT(
						"%s: Failed to perform operation type =%d\n"), __func__, type));
		return -EFAULT;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,  (TEXT(" Written success and trying to read  \n")));
	if(type % 2 == 0)
	{
		rt = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->bb_rf_event), 10000); 
		if (rt < 0) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d: bb_rf_event wait event failed %d\n", __func__, __LINE__, rt));
		}

		w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->bb_rf_event));
		//w_adapter->os_intf_ops->onebox_acquire_spinlock(&w_adapter->lock_bb_rf, 0);
		if (type == RF_LOOPBACK_M3 || type == RF_LOOPBACK_M2)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Initial rf_lpbk_len : %d"),w_adapter->rf_lpbk_len));
			//   while (rf_len < w_adapter->rf_lpbk_len)
			{ 
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Initial else rf_lpbk_len : %d"),w_adapter->rf_lpbk_len));

				w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)&w_adapter->rf_lpbk_data[0], w_adapter->rf_lpbk_len );
				if(copy_to_user((wrq->u.data.pointer), &w_adapter->rf_lpbk_data[0], w_adapter->rf_lpbk_len))
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,  (TEXT(
									"onebox_ioctl: Failed to perform operation\n")));
					//         w_adapter->os_intf_ops->onebox_release_spinlock(&w_adapter->lock_bb_rf, 0);
					return -EFAULT;
				}
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("rf_lpbk_len : %d, rf_len: %d, len: %d"),w_adapter->rf_lpbk_len,rf_len,len));
				rf_len += w_adapter->rf_lpbk_len/2;
				//w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->bb_rf_event), 10000); 
			}
			w_adapter->os_intf_ops->onebox_memset(&w_adapter->rf_lpbk_data[0], 0, w_adapter->rf_lpbk_len );
			w_adapter->rf_lpbk_len = 0;
		} 
		else 
		{  
			wrq->u.data.length = sizeof(bb_rf_params_t);

			if(copy_to_user(wrq->u.data.pointer, &w_adapter->bb_rf_read, sizeof(bb_rf_params_t)))
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,  (TEXT(
								"onebox_ioctl: Failed to perform operation\n")));
				w_adapter->os_intf_ops->onebox_release_spinlock(&w_adapter->lock_bb_rf, 0);
				return -EFAULT;
			}
		}
		//		w_adapter->os_intf_ops->onebox_release_spinlock(&w_adapter->lock_bb_rf, 0);
		for(i=0;i<w_adapter->bb_rf_read.no_of_values;i++)
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("After reading bb_rf_read.Data[] = 0x%x \n"),w_adapter->bb_rf_read.Data[i]));
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(
					"%s: Success in performing operation\n"), __func__));
	return status;
}  

/**
 * This function programs BBP registers for CW transmissions 
 *
 * @param  Pointer to w_adapter structure.  
 * @param  Power Save Cmd.  
 * @return 0 if success else -1. 
 */

ONEBOX_STATUS set_cw_mode (WLAN_ADAPTER w_adapter, uint8 mode)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status_l;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	int i = 0;
	int j = 0;
	uint16 *cw_mode_buf_write_array;
	int32 rt = 0;

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
	             (TEXT("===> Sending CW transmission Programming Packet <===\n")));

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);

	/*prepare the frame descriptor */
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(CW_MODE_REQ);

	w_adapter->soft_reset = 0;

	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16((w_adapter->cw_sub_type << 8) | w_adapter->cw_type);

	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, FRAME_DESC_SZ );
	status_l = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (FRAME_DESC_SZ));
  if (status_l != ONEBOX_STATUS_SUCCESS)
	{
		return status_l;
	}
	else
	{
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("CW MODE write waiting \n")));
    rt = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->bb_rf_event), 10000); 
   	if (rt < 0) {
   	    ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d: bb_rf_event wait event failed %d\n", __func__, __LINE__, rt));
   	}
    w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->bb_rf_event));
#if 1
    //mode = 2;
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("CW_mode %d\n"),mode));
    switch (mode)
    {
      case 0:
      case 1:
        cw_mode_buf_write_array = cw_mode_buf_write_array_1;
        break;
      case 2:
        cw_mode_buf_write_array = cw_mode_buf_write_array_2;
        break;
      case 3:
        cw_mode_buf_write_array = cw_mode_buf_write_array_3;
        break;
      case 4:
        cw_mode_buf_write_array = cw_mode_buf_write_array_4;
        break;
      case 5:
        cw_mode_buf_write_array = cw_mode_buf_write_dc_tone;
        break;
      case 6:
        cw_mode_buf_write_array = cw_mode_buf_write_array_6;
        break;
      case 7:
        //cw_mode_buf_write_array = cw_mode_buf_write_array_7;
        cw_mode_buf_write_array = cw_mode_buf_write_array_1;
        break;
      default:
        {
          cw_mode_buf_write_array = cw_mode_buf_write_array_1;
          break;
        }
    } /* End switch */
		i = 6;
		while (i< (258*3))
		{
			memset (&w_adapter->bb_rf_params, 0, sizeof (bb_rf_params_t));
			w_adapter->bb_rf_params.Data[1] = 0x315;
			w_adapter->bb_rf_params.Data[2] = 0x316;
			w_adapter->bb_rf_params.Data[3] = 0x317;
			for (j =4; j< (35*3);j+=3 )
			{
				if (i >= 258*3 )
					break;	
				w_adapter->bb_rf_params.Data[j] = cw_mode_buf_write_array[i];
				w_adapter->bb_rf_params.Data[j+1] = cw_mode_buf_write_array[i+1];
				w_adapter->bb_rf_params.Data[j+2] = cw_mode_buf_write_array[i+2];
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("***** Data[%d] = 0x%x ,cw_mode_buf_write_array[%d] = 0x%x\n"),j,w_adapter->bb_rf_params.Data[j],i,cw_mode_buf_write_array[i]));
				i+=3;
			}	
			w_adapter->bb_rf_params.value = 7; //BUFFER_WRITE
			w_adapter->bb_rf_params.no_of_values = 34;
      			w_adapter->soft_reset = BBP_REMOVE_SOFT_RST_AFTER_PROG;
			if(onebox_bb_buffer_request_indirect(w_adapter, w_adapter->bb_rf_params.Data, w_adapter->bb_rf_params.no_of_values) != ONEBOX_STATUS_SUCCESS)
			{
				return ONEBOX_STATUS_FAILURE;
			}
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("CW MODE BUFFER write waiting \n")));
      	rt = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->bb_rf_event), 10000); 
   		if (rt < 0) {
   		    ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : bb_rf_event wait event failed %d\n", __func__, __LINE__, rt));
   		}
      w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->bb_rf_event));
		}	
			memset (&w_adapter->bb_rf_params, 0, sizeof (bb_rf_params_t));
			w_adapter->bb_rf_params.Data[1] = 0x318;
			w_adapter->bb_rf_params.Data[2] = 0x80;
      w_adapter->bb_rf_params.value = 1; //BB_WRITE
			w_adapter->bb_rf_params.no_of_values = 2;
			w_adapter->soft_reset = 0;
		if(onebox_mgmt_send_bb_prog_frame(w_adapter, w_adapter->bb_rf_params.Data, w_adapter->bb_rf_params.no_of_values) != ONEBOX_STATUS_SUCCESS)
		{
			return ONEBOX_STATUS_FAILURE;
		}

#endif
	}	
	return status_l;
}


ONEBOX_STATUS onebox_rf_loopback(WLAN_ADAPTER w_adapter,
                                             uint16 *bb_prog_vals,
                                             uint16 num_of_vals, uint8 type)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status_l;
	uint16 frame_len;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE]; //1024* 4(4 bytes data) = 512*4*2(bytes)

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
	             (TEXT("===> Sending RF_LOOPBACK Programming Packet <===\n")));

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);
//  this commented code may be used for LOOPBACK write for bebugging
#if 0
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_MGMT_DUMP, (PUCHAR)bb_prog_vals, num_of_vals);
	if (w_adapter->bb_rf_rw)
	{		
  	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REG_READ);
		w_adapter->bb_rf_rw = 0;
    frame_len = 0;
	}
  else
  {
    /* Preparing BB Request Frame Body */
    for (count=1; ((count < num_of_vals) && (ii< num_of_vals)); ii++, count+=2) 
    {
      mgmt_frame->u.bb_prog_req[ii].reg_addr = ONEBOX_CPU_TO_LE16(bb_prog_vals[count]);
      mgmt_frame->u.bb_prog_req[ii].bb_prog_vals = ONEBOX_CPU_TO_LE16(bb_prog_vals[count+1]);
    }

    if (num_of_vals % 2)
    {
      mgmt_frame->u.bb_prog_req[ii].reg_addr = ONEBOX_CPU_TO_LE16(bb_prog_vals[count]);
    }
    /* Preparing BB Request Frame Header */
    frame_len = ((num_of_vals) * 2);	//each 2 bytes
  }  
#endif
    frame_len = 0;
	/*prepare the frame descriptor */
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((frame_len) | (ONEBOX_WIFI_MGMT_Q << 12));
	if( type == RF_LOOPBACK_M2 )
	{		
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(RF_LOOPBACK_REQ);
	}
	else
	{
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(RF_LPBK_M3);
	}	

#if 0  
	if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_BEFORE_PROG)
	{
  		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_BEFORE_PROG);
	}	
	if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_AFTER_PROG)
	{
  		mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_AFTER_PROG);	
	}	
  	w_adapter->soft_reset = 0;
	
	//Flags are not handled :
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(num_of_vals/2);
	//: What is the radio id to fill here
//	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (RADIO_ID << 8 );

	mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(bb_prog_vals[0]);
#endif	

	//w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, (frame_len + FRAME_DESC_SZ ));
	status_l = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, (frame_len + FRAME_DESC_SZ));

	return status_l;
}

ONEBOX_STATUS onebox_conf_beacon_recv(WLAN_ADAPTER w_adapter, uint8 value )
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status_l;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

  FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("ONEBOX_IOCTL: CONF MGMT PKT RECV\n"));

  mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

  w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);
  /* FrameType*/
  mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BEACON_RECV_DIS);
  mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(value & 0x00ff);
  mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12);
  status_l = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, ( FRAME_DESC_SZ));
  w_adapter->beacon_recv_disable = value;
	return status_l;
}

ONEBOX_STATUS eeprom_read(WLAN_ADAPTER w_adapter)
{
	uint16 pkt_len = 0;
	uint8  *pkt_buffer;

	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;

	pkt_len = FRAME_DESC_SZ;
	pkt_buffer = w_adapter->os_intf_ops->onebox_mem_zalloc(pkt_len, GFP_ATOMIC);
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
			(TEXT("===> Frame to PERFORM EEPROM READ <===\n")));

	/* FrameType*/
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(EEPROM_READ_TYPE);


	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12 | (pkt_len - FRAME_DESC_SZ));

	/* Number of bytes to read*/
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" offset = 0x%x, length = %d\n"),w_adapter->eeprom.offset,w_adapter->eeprom.length));
	mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(w_adapter->eeprom.length << 4);
	mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(3 << 8); //hsize = 3 as 32 bit transfer

	/* Address to read*/
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(w_adapter->eeprom.offset);
	mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(w_adapter->eeprom.offset >> 16);
	mgmt_frame->desc_word[6] = ONEBOX_CPU_TO_LE16(0); //delay = 0

	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, pkt_len);
	w_adapter->os_intf_ops->onebox_mem_free(pkt_buffer);
	return status;
}

ONEBOX_STATUS get_tx_power(WLAN_ADAPTER w_adapter)
{

	uint8  *pkt_buffer;

	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;

	pkt_buffer = w_adapter->os_intf_ops->onebox_mem_zalloc(FRAME_DESC_SZ, GFP_ATOMIC);
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
			(TEXT("===> Frame to GET TX_POWER from Firmware <===\n")));

	/* FrameType*/
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(TX_POWER_REQUEST);
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12);


	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, FRAME_DESC_SZ);
	w_adapter->os_intf_ops->onebox_mem_free(pkt_buffer);
	return status;
 
}

ONEBOX_STATUS process_eeprom_write(WLAN_ADAPTER w_adapter)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	uint8  extended_desc = 0;
	uint8  *temp;
	spi_config_t spi_config;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("===> Frame to PERFORM EEPROM WRITE <===\n")));
  ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("\neeprom_erase bit in flash_write function: %d, \n"), w_adapter->eeprom_erase));
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ + extended_desc + sizeof(spi_config_t));
	/* FrameType*/
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(EEPROM_WRITE);
	mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12);
	/* flags*/
	extended_desc = 4;
	mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(extended_desc); //extended_desc

	printk(" eeprom length  %d\n",w_adapter->eeprom.length);
	printk(" eeprom offset   %d \n",w_adapter->eeprom.offset);

	mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(w_adapter->eeprom.length);
	/* Address to read*/
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(w_adapter->eeprom.offset);
	mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(w_adapter->eeprom.offset >> 16);

	w_adapter->os_intf_ops->onebox_memcpy(&spi_config, &spi_default_configs, sizeof(spi_config_t));

	if (w_adapter->eeprom_erase)
	{
			mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(BIT(10));
	} 
	if(w_adapter->eeprom.write)
	{
			mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(BIT(11));
	} 

	spi_config.spi_config_2.protection = REM_WR_PROT;

	temp = ((uint8 *)mgmt_frame + FRAME_DESC_SZ + extended_desc);
	w_adapter->os_intf_ops->onebox_memcpy(temp, &spi_config, sizeof(spi_config_t));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)&spi_default_configs, sizeof(spi_config_t));
#if 1                                                                                       //KK

	temp = ((uint8 *)mgmt_frame + FRAME_DESC_SZ + extended_desc + sizeof(spi_config_t));
	w_adapter->os_intf_ops->onebox_memset(temp, 0, w_adapter->eeprom.length);  
  if(w_adapter->eeprom.write)
  {
    w_adapter->os_intf_ops->onebox_memcpy(temp, &w_adapter->eeprom.data, w_adapter->eeprom.length);
  }

#endif
	mgmt_frame->desc_word[0] |= ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12 | (w_adapter->eeprom.length + extended_desc*3));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)mgmt_frame, FRAME_DESC_SZ + extended_desc + sizeof(spi_config_t));
  if(w_adapter->eeprom.write)
  {
	  //printk("[%s][%d] : Data going to write before sending to FW is : \n", __func__, __LINE__);
    w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)&w_adapter->eeprom.data, w_adapter->eeprom.length);
    w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)temp, w_adapter->eeprom.length);
  }
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, 
					(extended_desc + FRAME_DESC_SZ + sizeof(spi_config_t) + w_adapter->eeprom.length));
	return status;
}

/**
 * This function prepares reset RX filter request frame and send it to LMAC.
 *
 * @param  Pointer to Adapter structure.  
 * @return 0 if success else -1. 
 */
ONEBOX_STATUS onebox_send_rx_filter_frame(WLAN_ADAPTER w_adapter, uint16_t rx_filter_word)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[FRAME_DESC_SZ];

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
	             (TEXT("===> Send Rx Filter frame <===\n")));
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Rx filter word is %x\n"), rx_filter_word));

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

	/* Bit{0:11} indicates length of the Packet
 	 * Bit{12:16} indicates host queue number
 	 */ 

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12);
	/* Fill frame type set_rx_filter */
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(SET_RX_FILTER);
	/* Fill data in form of flags*/
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(rx_filter_word);

	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)mgmt_frame, (FRAME_DESC_SZ));
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, FRAME_DESC_SZ);
	return status;
} /*end of onebox_send_rx_filter_frame */
#define MAC_ADDR_LEN 6
#ifdef IEEE80211K
/*This Function is called when rm_mcast_diagnostic_timeout happen, 
 *the data collected before timeout regarding multicast data is sent as a report 
 * in response to multicast diagnostic request received from this function
 * @params: w_adapter
 * */
void rm_mcast_var_reset( WLAN_ADAPTER w_adapter)                         
{
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211vap *vap =NULL;
	uint8_t msg[100],msg_len,ii;
	msg_len = 28;

		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
		{ 
  				if(vap->iv_opmode != IEEE80211_M_STA)
        continue;
       
							vap->rm_mcast_data.mcast_var_set = 0;
							msg[5] = IEEE80211_MULTICAST_DIG_REQ;
							msg[6] = 0;
              for(ii=0 ; ii<=7 ;ii++)
                msg[7+ii] = 0;
							memcpy(&msg[15],ic->meas_info[0].req.bssid,ETHER_ADDR_LEN);
							msg[21] = BIT(1); //mcast report reason
              msg[22] = (uint8_t) vap->rm_mcast_data.multicast_frame_count;  
							msg[23] = (uint8_t) vap->rm_mcast_data.multicast_frame_count >> 8;
              msg[24] = vap->rm_mcast_data.first_seq_num;
							msg[25] = vap->rm_mcast_data.last_seq_num;
              msg[26] = (uint8_t)vap->rm_mcast_data.highest_mcast_rate;
              msg[27] = (uint8_t)vap->rm_mcast_data.highest_mcast_rate >> 8;
							w_adapter->core_ops->onebox_dump(1, msg, msg_len);
							w_adapter->net80211_ops->onebox_meas_rpt(vap, msg, msg_len); 	
  }	
		if (!w_adapter->hal_vap[w_adapter->sta_mode.vap_id].vap_in_use) {
		  	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d timer is expired sta_mode vap is not in use\n"),
										__func__, __LINE__));
			  return;
		 }
		  w_adapter->os_intf_ops->onebox_remove_timer(&w_adapter->rm_mcast_diagnostics_timeout);
    return ;
}

/*This function is used to send radio measurement request to firmware
 * as an internal mgmt packet.
 * @param1 : structure containg measurment request patameters
 * @param2 : measurement type , based on which we can diff between radio or spectral measurement 
 */
ONEBOX_STATUS onebox_send_meas_info(uint8_t *meas_struct , uint8_t msrmnt_type)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[FRAME_DESC_SZ + 200]; //Better to define dynamically
	struct ieee80211vap *vap;
	struct ieee80211com *ic;
	struct radio_meas_info *meas_info;	
	struct sm_meas_info *sm_meas_info;
	WLAN_ADAPTER w_adapter;
	struct onebox_os_intf_operations *os_intf_ops;
	uint8 mac_addr_len = 6;
	uint16 frame_body_len = 0;
	uint8 dfs = 0;
	uint8 channel_width = 0; /* default 20Mhz */

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);
	if(msrmnt_type<3)
	{
		sm_meas_info = (struct sm_meas_info *) meas_struct;
		vap = sm_meas_info->ni->ni_vap;
	}
	else
	{
		meas_info = (struct radio_meas_info *) meas_struct;
		vap = meas_info->ni->ni_vap;
	}

	if(is_vap_valid(vap) < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("ERROR: VAP is Not a valid pointer In %s Line %d\n ")
						,	__func__, __LINE__));
		dump_stack();
		return ONEBOX_STATUS_FAILURE;
	}

	ic = vap->iv_ic;
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
	w_adapter = os_intf_ops->onebox_get_priv(ic->ic_ifp);
	if(msrmnt_type == IEEE80211_MULTICAST_DIG_REQ  && !vap->rm_mcast_data.mcast_var_set )
	{	

		w_adapter->os_intf_ops->onebox_init_sw_timer(&w_adapter->rm_mcast_diagnostics_timeout,(unsigned long)w_adapter,
									(void *)&rm_mcast_var_reset, msecs_to_jiffies(meas_info->req.meas_duration));	
		vap->rm_mcast_data.mcast_var_set = 1;	
		return 0;	
		
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
	             (TEXT("===> Sending Measurement Request <===\n")));
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(SEND_MEAS_INFO);
	if(msrmnt_type <3)
	{
		mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(sm_meas_info->meas_req.type << 8);
		mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(sm_meas_info->meas_req.mode);
		mgmt_frame->desc_word[3] |=ONEBOX_CPU_TO_LE16(sm_meas_info->req.chan_num << 8);
		mgmt_frame->desc_word[4] =ONEBOX_CPU_TO_LE16(sm_meas_info->req.msrmnt_dur);

		w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.byte.buf[frame_body_len], &sm_meas_info->req.msrmnt_start_time, 8); //TSF -start time length.
		frame_body_len +=8;	
	}
	else
	{
		mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(meas_info->meas_req.type << 8);
		mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(meas_info->meas_req.mode);
		mgmt_frame->desc_word[3] |=ONEBOX_CPU_TO_LE16(meas_info->req.channel_num << 8);
		mgmt_frame->desc_word[4] =ONEBOX_CPU_TO_LE16(meas_info->req.meas_duration);
		mgmt_frame->desc_word[5] =ONEBOX_CPU_TO_LE16(meas_info->req.rand_int);
		mgmt_frame->desc_word[6] =ONEBOX_CPU_TO_LE16(dfs << 8); /* This could be updated in future */
		mgmt_frame->desc_word[6] |=ONEBOX_CPU_TO_LE16(channel_width);/* This could be updated in future */

		if( meas_info->meas_req.type == IEEE80211_FRAME ) 
		{
			w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.byte.buf[frame_body_len], &meas_info->req.mac_addr[0], mac_addr_len);
			frame_body_len +=mac_addr_len; 
			mgmt_frame->u.byte.buf[frame_body_len] = meas_info->req.frame_req_type;
			frame_body_len +=1; 
		} 
		else if ( meas_info->meas_req.type == IEEE80211_BEACON ) 
		{
			w_adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.byte.buf[frame_body_len], &meas_info->req.bssid[0], mac_addr_len);
			frame_body_len +=mac_addr_len; 
			w_adapter->os_intf_ops->onebox_memset(&mgmt_frame->u.byte.buf[frame_body_len], 0, SSID_LEN); //WILD CARD SSID 32 bytes
			frame_body_len +=SSID_LEN; 
			mgmt_frame->u.byte.buf[frame_body_len] = 0; //FIX ME: As required by Firmware when there is no rpt_detail. Need to modify.
			frame_body_len +=1; 
			mgmt_frame->u.byte.buf[frame_body_len] = meas_info->req.meas_mode;
			frame_body_len +=1; 
			if( meas_info->req.meas_mode == 1) 
			{
				uint8 probe_len;
				probe_len = rm_prepare_probe_req_body(&mgmt_frame->u.byte.buf[frame_body_len+1],meas_info,w_adapter);
				mgmt_frame->u.byte.buf[frame_body_len] = probe_len;
				frame_body_len += (probe_len+1); 
				/*Add probe request frame */
			}
		} 
}
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(frame_body_len | (ONEBOX_WIFI_MGMT_Q << 12));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)mgmt_frame, (FRAME_DESC_SZ) + frame_body_len);
	status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, FRAME_DESC_SZ + frame_body_len);
	return status;
}

uint8 rm_prepare_probe_req_body(uint8 *buf,struct radio_meas_info *meas_info,WLAN_ADAPTER w_adapter )
{
	struct ieee80211vap *vap = meas_info->ni->ni_vap;
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211_channel *c;	
	struct ieee80211_rateset *rs;
	uint8_t *temp_addr;	
	uint8 len = 0;

	temp_addr = buf;
	*buf++ = IEEE80211_FC0_SUBTYPE_PROBE_REQ;
	buf = buf+3;
	memcpy(buf,&meas_info->req.bssid[0], MAC_ADDR_LEN);
	buf = buf+MAC_ADDR_LEN;
	memcpy(buf, vap->iv_myaddr, MAC_ADDR_LEN);
	buf = buf+MAC_ADDR_LEN;
	memcpy(buf,&meas_info->req.bssid[0], MAC_ADDR_LEN);
	buf =buf+MAC_ADDR_LEN;// more 2 for sequence number
	*buf++ = 0;
	*buf++ = 0;

	*buf++ = IEEE80211_ELEMID_SSID;

	if(meas_info->req.bse.ssid_ie_len)
	{
		memcpy(buf, &meas_info->req.bse.ssid_ie[0],meas_info->req.bse.ssid_ie_len);
		buf = buf + meas_info->req.bse.ssid_ie_len;
	}
	else
	{
		*buf++ = 0;//len_ssid;
	}

	c = ieee80211_find_channel_byieee(ic, 36, ic->ic_curchan->ic_flags);
	if(c == NULL) 
	{
		rs = (struct ieee80211_rateset *)ieee80211_get_suprates(ic, ic->ic_curchan);
	}
	else 
	{
		rs = (struct ieee80211_rateset *)ieee80211_get_suprates(ic, c);
	}
	buf = ieee80211_add_rates(buf, rs);
	buf = ieee80211_add_xrates(buf, rs);

	len = temp_addr - &buf[0];
	return len;
}
#endif	

EXPORT_SYMBOL(onebox_send_rx_filter_frame);

#ifdef ONEBOX_CONFIG_WOWLAN
ONEBOX_STATUS onebox_send_wowlan_params(struct ieee80211vap *vap,
		WLAN_ADAPTER w_adapter)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status = 0;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16
		(sizeof(mgmt_frame->u.wowlan_params) |
		 (ONEBOX_WIFI_MGMT_Q << 12));
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(WOWLAN_CONFIG_PARAMS);
	memcpy(&mgmt_frame->u.wowlan_params, &vap->hal_priv_vap->wowlan_params,
			sizeof(struct wowlan_config));

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			("%s: <==== Sending WOWLAN PARAMS =====>\n", __func__));
	status = w_adapter->devdep_ops->onebox_send_internal_mgmt_frame(w_adapter,
			(uint16 *)mgmt_frame,
			FRAME_DESC_SZ + sizeof(mgmt_frame->u.wowlan_params));
	if (!status) {
		if (mgmt_frame->u.wowlan_params.host_wakeup_state)
			w_adapter->wowlan_enabled = true;
		else
			w_adapter->wowlan_enabled = false;
	}

	return status;
}
#endif
/**
 * This function prepares the RF Programming Request frame and sends to the PPE
 *
 * @param 
 *  w_adapter  Pointer to Driver Private Structure
 *  w_adapter  Pointer to RF programming values Structure
 *  w_adapter  number of programming values to be loaded
 *  w_adapter  number of values in a row
 *
 * @returns 
 *  ONEBOX_STATUS_SUCCESS on success, or corresponding negative
 *  error code on failure
 */
ONEBOX_STATUS onebox_mgmt_send_9116_rf_prog_frame(WLAN_ADAPTER w_adapter,
        uint16 *rf_prog_vals,
        uint16 num_of_sets,
        uint16 vals_per_set,
        uint8  type, uint8 protocol_id) 
{

    netbuf_ctrl_block_t *netbuf_cb = NULL;
    onebox_mac_frame_t *mgmt_frame;
    ONEBOX_STATUS status_l;
    uint16 frame_len;
    uint16 count;
    uint32 len;

    frame_len = (vals_per_set * num_of_sets * 2);
    len = frame_len + FRAME_DESC_SZ;
    netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(len);
    if(netbuf_cb == NULL)
    {	
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
        status_l = ONEBOX_STATUS_FAILURE;
        return status_l;

    }

   
    w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);

    mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
    w_adapter->os_intf_ops->onebox_memset(mgmt_frame,0x0,len);


    FUNCTION_ENTRY(ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,(TEXT("===> Sending PROTOCOL RF Programming Packet <===\n")));



    mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(frame_len | (COEX_TX_Q << 12));
    mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(RF_PROG_VALUES_REQUEST);
    mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(protocol_id << 8);
    netbuf_cb->tx_pkt_type = COEX_Q;

    if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_BEFORE_PROG)
    {
        mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_BEFORE_PROG);
    }	
    if (w_adapter->soft_reset & BBP_REMOVE_SOFT_RST_AFTER_PROG)
    {
        mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REMOVE_SOFT_RST_AFTER_PROG);	
    }	

    if (w_adapter->bb_rf_rw)
    {		
        mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(BBP_REG_READ);
        w_adapter->bb_rf_rw = 0;
    }	
    w_adapter->soft_reset = 0;

    if (w_adapter->Driver_Mode == RF_EVAL_MODE_ON)
    {
        if (w_adapter->bb_rf_params.value == 4 || w_adapter->bb_rf_params.value == 5)
        {
            mgmt_frame->desc_word[5] |= ONEBOX_CPU_TO_LE16(ULP_MODE);  //indicating ULP 
            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ULP \n")));
        }  
    }
    //Flags are not handled :
    mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(vals_per_set | (num_of_sets << 8));
#ifdef RF_8230
    mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (NONRSI_RF_TYPE << 4));
#else
    mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (PUT_BBP_RESET | BBP_REG_WRITE | (RSI_RF_TYPE << 4));
#endif  
    if(w_adapter->rf_reset)
    {
        mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16 (RF_RESET_ENABLE);
        ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("===> RF RESET REQUEST SENT <===\n")));
        w_adapter->rf_reset = 0;
    }

    for (count = 0; count < (vals_per_set * num_of_sets); count++)
    {
        mgmt_frame->u.rf_prog_req.rf_prog_vals[count] = ONEBOX_CPU_TO_LE16(rf_prog_vals[count]);
    }


    w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR,(PUCHAR)mgmt_frame,len);
    status_l = w_adapter->d_assets->common_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, COEX_Q);

    return status_l;
} /* onebox_mgmt_send_9116_rf_prog_frame*/


