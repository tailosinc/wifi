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
#include "onebox_sdio_intf.h"
#include "onebox_host_intf_ops.h"
#include "onebox_intf_ops.h"
#include "onebox_zone.h"

/**
 * This function writes the packet to the device.
 * @param  Pointer to the driver's private data structure
 * @param  Pointer to the data to be written on to the device
 * @param  Length of the data to be written on to the device.  
 * @return 0 if success else a negative number. 
 */
ONEBOX_STATUS usb_host_intf_write_pkt(PONEBOX_ADAPTER adapter, uint8 *pkt, uint32 Len, uint8 q_no, netbuf_ctrl_block_t *netbuf_cb)
{
	uint32 Address;
	uint32 queueno = (q_no & 0x7);
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;

	if( !Len  && (queueno == ONEBOX_WIFI_DATA_Q ))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(" Wrong length \n")));
		return ONEBOX_STATUS_FAILURE;
	} /* End if <condition> */  

	/* Fill endpoint numbers based on queueno */
	if ((queueno == COEX_TX_Q) || (queueno == WLAN_TX_M_Q) || (queueno == WLAN_TX_D_Q)) {
		Address = 1;
		//} else if ((queueno == BT_TX_Q) || (queueno == ZIGB_TX_Q)) {
	}
	else if ((adapter->device_model == RSI_DEV_9116) && (queueno == ZIGB_TX_Q)) { 
		Address = 3;	
	}
	else {
		Address = 2;	
	}

	if (adapter == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("%s: adapter null: %d\n"),__func__, status));
		goto fail_case;
	}
	if (adapter->osd_host_intf_ops == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("%s: adapter osd_host_intf_ops null: %d\n"),__func__, status));
		goto fail_case;
	}	
	if (adapter->osd_host_intf_ops->onebox_write_multiple == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("%s: adapter onebox_write_multiple null: %d\n"),__func__, status));
		goto fail_case;
	}

	status = adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
					Address, 
					(uint8 *)pkt,
					Len,
					netbuf_cb);
	if (status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("%s: Unable to write onto the card: %d\n"),__func__,
			 status));
		ONEBOX_DEBUG( ONEBOX_ZONE_DEBUG, (" <======= Desc details written previously ========== >\n"));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)adapter->prev_desc, FRAME_DESC_SZ);
		ONEBOX_DEBUG( ONEBOX_ZONE_DEBUG, (" Current Pkt: Card write PKT details\n"));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)pkt, Len);
	} /* End if <condition> */

	ONEBOX_DEBUG(ONEBOX_ZONE_DATA_SEND,
		(TEXT("%s:Successfully written onto card\n"),__func__));
fail_case:
	return status;
}/*End <usb_host_intf_write_pkt>*/

/**
 * This function reads the packet from the SD card.
 * @param  Pointer to the driver's private data structure
 * @param  Pointer to the packet data  read from the the device
 * @param  Length of the data to be read from the device.  
 * @return 0 if success else a negative number. 
 */
ONEBOX_STATUS usb_host_intf_read_pkt(PONEBOX_ADAPTER adapter,uint8 *pkt,uint32 length)
{
	//uint32 Blocksize = adapter->ReceiveBlockSize;
	ONEBOX_STATUS Status  = ONEBOX_STATUS_SUCCESS;
	//uint32 num_blocks;

	ONEBOX_DEBUG(ONEBOX_ZONE_DATA_RCV,(TEXT( "%s: Reading %d bytes from the card\n"),__func__, length));
	if (!length)
	{
		//ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
		 //            (TEXT( "%s: Pkt size is zero\n"),__func__));
		return Status;
	}

	//num_blocks = (length / Blocksize);
	
	/*Reading the actual data*/  
	Status = adapter->osd_host_intf_ops->onebox_read_multiple(adapter,
	                                                          length,
	                                                          length, /*num of bytes*/
	                                                          (uint8 *)pkt); 

	if (Status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(
		             "%s: Failed to read frame from the card: %d\n"),__func__,
		             Status));
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Card Read PKT details len =%d :\n", length));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, (PUCHAR)pkt, length);

		return Status;
	}
	return Status;
}



