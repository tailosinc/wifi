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
#include "onebox_sdio_intf.h"
#include "onebox_zone.h"

/**
 * This function disables the sdio interrupt
 * @param  pointer to the driver private data structure
 * @return ONEBOX_STATUS_SUCCESS on success else failure status is returned
 */

ONEBOX_STATUS disable_sdio_interrupt(PONEBOX_ADAPTER adapter)
{
//FIXME update the reason for this requirement.
#if 0
	uint8 byte_buf;
	ONEBOX_STATUS status;

	byte_buf = 0x0;
	status = adapter->osd_host_intf_ops->onebox_write_register(adapter, 0,
	                                                           ONEBOX_SDIO_INTERRUPT_ENABLE_REGISTER, 
	                                                           &byte_buf);
	if (status != ONEBOX_STATUS_SUCCESS) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Failed to write ONEBOX_SDIO_INTERRUPT_ENABLE_REGISTER status = %d\n"), __func__, status));
		return status;
	}
#endif
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function writes the packet to the device.
 * @param  Pointer to the driver's private data structure
 * @param  Pointer to the data to be written on to the device
 * @param  Length of the data to be written on to the device.  
 * @return 0 if success else a negative number. 
 */
ONEBOX_STATUS sdio_host_intf_write_pkt(PONEBOX_ADAPTER adapter, uint8 *pkt, uint32 Len, uint8 q_no, netbuf_ctrl_block_t *netbuf_cb)
{
	uint32 block_size    = adapter->TransmitBlockSize;
	uint32 num_blocks,Address,Length;
	uint32 queueno = q_no;
#ifndef RSI_SDIO_MULTI_BLOCK_SUPPORT
	uint32 ii;
#endif  
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	
	if( !Len  && (queueno == WLAN_TX_D_Q ))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(" Wrong length \n")));
		return ONEBOX_STATUS_FAILURE;
	} /* End if <condition> */  
	num_blocks = Len/block_size;
	if (Len % block_size)
	{
		num_blocks++;
	}
#ifdef ENABLE_SDIO_CHANGE
	if (num_blocks < 2)
	{
		num_blocks = 2;
	}
#endif
	Address = num_blocks * block_size | (queueno << 12);
	Length  = num_blocks * block_size;

#ifdef RSI_SDIO_MULTI_BLOCK_SUPPORT
	status = adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
	                                                           Address, 
	                                                           (uint8 *)pkt,
	                                                           Length, netbuf_cb);
	if (status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Unable to write onto the card: %d\n"),__func__,
		             status));
		ONEBOX_DEBUG( ONEBOX_ZONE_DEBUG, (" <======= Desc details written previously ========== >\n"));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)adapter->prev_desc, FRAME_DESC_SZ);
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, (" Current Pkt: Card write PKT details\n"));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)pkt, Length);
		onebox_sdio_abort_handler(adapter);
	} /* End if <condition> */
#else
	/* Non multi block read */
	for(ii = 0; ii < num_blocks; ii++)
	{
		if(ii==0)
		{
			status = adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
			                                                           (num_blocks*block_size |
			                                                           (queueno<<12)),
			                                                           pkt + (ii*block_size),
			                                                           block_size, netbuf_cb);
		}
		else
		{
			status = adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
			                                                           (num_blocks*block_size),
			                                                           pkt + (ii*block_size),
			                                                           block_size, netbuf_cb);
		} /* End if <condition> */
		if(status != ONEBOX_STATUS_SUCCESS) 
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			             (TEXT("Multi Block Support: Writing to CARD Failed\n")));
			onebox_sdio_abort_handler(adapter);
			status = ONEBOX_STATUS_FAILURE;
		} /* End if <condition> */
	} /* End for loop */
#endif
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("%s:Successfully written onto card\n"),__func__));
	return status;
}/*End <sdio_host_intf_write_pkt>*/

/**
 * This function reads the packet from the SD card.
 * @param  Pointer to the driver's private data structure
 * @param  Pointer to the packet data  read from the the device
 * @param  Length of the data to be read from the device.  
 * @return 0 if success else a negative number. 
 */
ONEBOX_STATUS sdio_host_intf_read_pkt(PONEBOX_ADAPTER adapter,uint8 *pkt,uint32 length)
{
	//uint32 Blocksize = adapter->ReceiveBlockSize;
	ONEBOX_STATUS Status  = ONEBOX_STATUS_SUCCESS;
	//uint32 num_blocks;

	ONEBOX_DEBUG(ONEBOX_ZONE_DATA_RCV,(TEXT( "%s: Reading %d bytes from the card\n"),__func__, length));
	if (!length)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT( "%s: Pkt size is zero\n"),__func__));
		return Status;
	}

	//num_blocks = (length / Blocksize);
#ifdef GPIO_HANDSHAKE
	if(!adapter->os_intf_ops->onebox_get_device_status())
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s: Device GPIO Status is LOW %d\n",__func__,__LINE__));
	}	
#endif
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

		ONEBOX_DEBUG( ONEBOX_ZONE_DEBUG, ("Card Read PKT details len =%d :\n", length));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)pkt, length);

		return Status;
	}
	return Status;
}

/**
 * This function intern calls the SDIO slave registers initialization function 
 * where SDIO slave registers are being initialized.
 *
 * @param  Pointer to our adapter structure.  
 * @return 0 on success and -1 on failure. 
 */
ONEBOX_STATUS init_sdio_host_interface(PONEBOX_ADAPTER adapter)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Initializing interface\n"),__func__)); 
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s: Sending init cmd\n"),__func__)); 

	/* Initialise the SDIO slave registers */
	return onebox_init_sdio_slave_regs(adapter);
}/* End <onebox_init_interface> */


/**
 * This function does the actual initialization of SDBUS slave registers. 
 *
 * @param  Pointer to the driver adapter structure. 
 * @return ONEBOX_STATUS_SUCCESS on success and ONEBOX_STATUS_FAILURE on failure. 
 */

ONEBOX_STATUS onebox_init_sdio_slave_regs(PONEBOX_ADAPTER adapter)
{
	uint8 byte;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	uint8 reg_dmn;
	FUNCTION_ENTRY(ONEBOX_ZONE_INIT);

	reg_dmn = 0; //TA domain
	/* initialize Next read delay */
	if(adapter->next_read_delay)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		             (TEXT("%s: Initialzing SDIO_NXT_RD_DELAY2\n"), __func__));
		byte = adapter->next_read_delay;

		status = adapter->osd_host_intf_ops->onebox_write_register(adapter,
		                                                           reg_dmn,
		                                                           SDIO_NXT_RD_DELAY2,&byte);
		if(status)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			             (TEXT("%s: fail to write SDIO_NXT_RD_DELAY2\n"), __func__));
			return ONEBOX_STATUS_FAILURE;
		}
	}

	if(adapter->sdio_high_speed_enable)
	{
#define SDIO_REG_HIGH_SPEED      0x13
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
		             (TEXT("%s: Enabling SDIO High speed\n"), __func__));
		byte = 0x3;

		status = adapter->osd_host_intf_ops->onebox_write_register(adapter,reg_dmn,SDIO_REG_HIGH_SPEED,&byte);              
		if(status)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			             (TEXT("%s: fail to enable SDIO high speed\n"), __func__));
			return ONEBOX_STATUS_FAILURE;
		}
	}

	/* This tells SDIO FIFO when to start read to host */
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
	             (TEXT("%s: Initialzing SDIO read start level\n"), __func__));
	byte = 0x24;

	status = adapter->osd_host_intf_ops->onebox_write_register(adapter,reg_dmn,SDIO_READ_START_LVL,&byte);              
	if(status)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: fail to write SDIO_READ_START_LVL\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	} 

	/* Change these parameters to load firmware */
	/* This tells SDIO FIFO when to start read to host */
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
	             (TEXT("%s: Initialzing FIFO ctrl registers\n"), __func__));
	byte = (128-32);

	status = adapter->osd_host_intf_ops->onebox_write_register(adapter,reg_dmn,SDIO_READ_FIFO_CTL,&byte);              
	if(status)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: fail to write SDIO_READ_FIFO_CTL\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}

	/* This tells SDIO FIFO when to start read to host */
	byte = 32;
	status = adapter->osd_host_intf_ops->onebox_write_register(adapter,reg_dmn,SDIO_WRITE_FIFO_CTL,&byte);              
	if(status)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: fail to write SDIO_WRITE_FIFO_CTL\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}

	FUNCTION_EXIT(ONEBOX_ZONE_INIT);
	return ONEBOX_STATUS_SUCCESS;
}/* End <onebox_init_sdio_slave_regs> */

/**
 * This function acks the interrupt received.
 *
 * @param  Pointer to the driver adapter structure. 
 * @param  Interrupt bit to write into register. 
 * @return VOID. 
 */
ONEBOX_STATUS sdio_ack_interrupt(PONEBOX_ADAPTER adapter, uint8 INT_BIT)
{
	ONEBOX_STATUS Status;
#if 1
	uint8 reg_dmn;

	reg_dmn = 1; //HOST domain
	Status = adapter->osd_host_intf_ops->onebox_write_register(adapter,
	                                                           reg_dmn,
	                                                           SDIO_FUN1_INTR_CLR_REG | SD_REQUEST_MASTER,
	                                                           &INT_BIT);

#else
	Status = adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
							SDIO_FUN1_INTR_CLR_REG | SD_REQUEST_MASTER,
							&INT_BIT,
							2
							);
#endif
	if(Status!=ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: unable to send ack\n"), __func__));
		return Status;
	}

	return Status;
}

/**
 * This function reads the abort register until it is cleared.
 *
 * This function is invoked when ever CMD53 read or write gets
 * failed. 
 *
 * @param  Pointer to the driver adapter structure. 
 * @return 0 on success and -1 on failure. 
 */

ONEBOX_STATUS onebox_sdio_abort_handler(PONEBOX_ADAPTER adapter )
{
	return ONEBOX_STATUS_SUCCESS;
#if 0
	uint8 InterruptStatus;
	uint8 retry_count    = 0;
	uint8 abort_data     = 1;        
	uint8 FirmwareStatus = 0;
	uint8 reg_dmn = 0;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	
	if(adapter->ShutDown == ONEBOX_TRUE)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s:Shutdown is TRUE..Returning..\n"),__func__));
		return ONEBOX_STATUS_FAILURE;
	} /* End if <condition> */   
	
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
	             (TEXT("%s: Reading f/w status:\n"),__func__));  
	status = adapter->osd_host_intf_ops->onebox_read_register(adapter, 
	                                                          SDIO_FW_STATUS_REG,&FirmwareStatus);
	if (!status) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Unable to read f/w status: %x\n"),__func__,
		             status));
	}  
	else
	{   
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Firmware status is: %08x\n"),__func__,   
		              FirmwareStatus));
	} /* End if <condition> */  
	
	status = adapter->osd_host_intf_ops->onebox_write_register(adapter,reg_dmn,0x06, &abort_data);           
	do
	{
		status = adapter->osd_host_intf_ops->onebox_read_register(adapter,SDIO_REG_ABORT_FEEDBACK, &InterruptStatus);
		adapter->os_intf_ops->onebox_msec_delay(500);
		retry_count++;   
	} 
	while((InterruptStatus != SDIO_ABORT_FEEDBACK_VALUE) &&
	       (retry_count < MAX_ABORT_FEEDBACK_RETRY));
	
	if (retry_count >= MAX_ABORT_FEEDBACK_RETRY )
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
		             (TEXT("%s: Feed back check exceeded retry limit \n"),__func__));
		status = ONEBOX_STATUS_FAILURE;
	} /* End if <condition> */
	retry_count = 0x33;
	status = adapter->osd_host_intf_ops->onebox_write_register(adapter,reg_dmn,SDIO_REG_ABORT_FEEDBACK, &retry_count); 
	
	return status;
#endif
} /* End <onebox_sdio_abort_handler> */
