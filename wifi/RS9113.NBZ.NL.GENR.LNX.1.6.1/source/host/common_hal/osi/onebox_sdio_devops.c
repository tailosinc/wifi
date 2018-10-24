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
#include "onebox_common.h"
#include "onebox_hal.h"
#include "onebox_linux.h"
#include "onebox_pktpro.h"
#include "onebox_host_intf_ops.h"
#include "onebox_intf_ops.h"
#include "onebox_sdio_intf.h"
#include "onebox_zone.h"

DEFINE_MUTEX(sdio_interrupt_lock);

ONEBOX_STATUS sdio_load_data_master_write(PONEBOX_ADAPTER adapter, uint32 base_address, uint32 instructions_sz,
	uint32 block_size, uint8 *ta_firmware)
{
	uint32 num_blocks;
	uint16 msb_address;
	uint32 cur_indx , ii;
	uint8  temp_buf[256];
	uint16 lsb_address;
	num_blocks = instructions_sz / block_size;
	msb_address = base_address >> 16;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("num_blocks: %d\n"),num_blocks));
	/* Loading DM ms word in the sdio slave */ 
	if (onebox_sdio_master_access_msword(adapter, (msb_address)) != ONEBOX_STATUS_SUCCESS)      
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to set ms word reg\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}

	for (cur_indx = 0,ii = 0; ii < num_blocks; ii++,cur_indx += block_size)
	{
		adapter->os_intf_ops->onebox_memset(temp_buf, 0, block_size);
		adapter->os_intf_ops->onebox_memcpy(temp_buf, ta_firmware + cur_indx, block_size);
		lsb_address = (uint16)base_address;
		if (adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
					lsb_address | SD_REQUEST_MASTER,
					(uint8 *)(temp_buf),
					block_size, NULL)
				!=ONEBOX_STATUS_SUCCESS)
			{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: failed to write \n"), __func__));
				return ONEBOX_STATUS_FAILURE;
			}      
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
				(TEXT("%s: loading block: %d\n"), __func__,ii));
		base_address += block_size;
		if((base_address >> 16) != msb_address)
		{
			msb_address += 1;
			/* Loading DM ms word in the sdio slave */
			if (onebox_sdio_master_access_msword(adapter,
						(msb_address)) !=
					ONEBOX_STATUS_SUCCESS) {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(
								"onebox_load_TA_inst: Unable to set ms word reg\n")));
				return ONEBOX_STATUS_FAILURE;
			}
		}

	}

	if (instructions_sz % block_size)
	{
		adapter->os_intf_ops->onebox_memset(temp_buf, 0, block_size);
		adapter->os_intf_ops->onebox_memcpy(temp_buf,
				ta_firmware + cur_indx,
				instructions_sz % block_size);
		lsb_address = (uint16)base_address;
		if (adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
					lsb_address | SD_REQUEST_MASTER,
					(uint8 *)(temp_buf),
					instructions_sz % block_size,
					NULL)!=ONEBOX_STATUS_SUCCESS)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("In %s Line %d: failed to write instructions \n"), __func__, __LINE__));

				return ONEBOX_STATUS_FAILURE;
			}
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
				(TEXT("*Written Last Block in Address 0x%x Successfully**\n"),
				 cur_indx | SD_REQUEST_MASTER));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (uint8 *)temp_buf, instructions_sz % block_size);
	}
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function enables the sdio interrupts
 *
 * @param
 *  adapter Pointer to driver's private area
 *
 * @return
 *  If the interrupts are disabled successfully then it will
 *  return ONEBOX_STATUS_SUCCESS otherwise error value
 */
ONEBOX_STATUS onebox_enable_host_interrupt(PONEBOX_ADAPTER adapter)
{
	//adapter->os_intf_ops->onebox_release_mutex(&adapter->sdio_interrupt_lock);
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This function read and process the SDIO interrupts
 *
 * @param
 *  adapter   Pointer to drivers private data area
 * @return
 *  None
 */
void sdio_interrupt_handler(PONEBOX_ADAPTER adapter)
{
	ONEBOX_STATUS status;
	SDIO_INTERRUPT_TYPE interrupt_type;
	uint8 sdio_interrupt_status = 0;
	uint8 fw_status = 0;
	uint8 device_buf_status = 0;
	struct driver_assets *d_assets = adapter->d_assets;

	adapter->sdio_int_counter++;
#ifdef SDIO_DEBUG
	adapter->timer_sdio_int_counter++;
#endif

	do
	{
		status = adapter->osd_host_intf_ops->onebox_read_register(adapter,
		                                                          ONEBOX_FUNCTION_1_INTERRUPT_REGISTER,
		                                                          &sdio_interrupt_status); /* Buffer */
		if (status != ONEBOX_STATUS_SUCCESS)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			             (TEXT("%s: Failed to Read Intr Status Register\n"),__func__));
#ifdef USE_SDIO_INTF
			onebox_sdio_abort_handler(adapter); 
#endif
			return;
		}

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT
		             ("%s: Intr_status = %x\n"),__func__,sdio_interrupt_status));
		adapter->interrupt_status = sdio_interrupt_status;
        	sdio_interrupt_status &= 0xE;

		if (sdio_interrupt_status == 0)
		{
			/* Not Our Interrupt */
			onebox_enable_host_interrupt(adapter);
			adapter->os_intf_ops->onebox_set_event(&(adapter->sdio_scheduler_thread_handle.thread_event));
			adapter->sdio_intr_status_zero++;
#ifdef SDIO_DEBUG
			adapter->timer_sdio_intr_status_zero++;
#endif
			return;
		}

		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT
		             ("%s: Intr_status = %x\n"),__func__,sdio_interrupt_status));

		do
		{
			/* Extracting Higher Priority Interrupt Pending */
			interrupt_type = ONEBOX_GET_SDIO_INTERRUPT_TYPE(sdio_interrupt_status);

			switch (interrupt_type)
			{
				case BUFF_STATUS_UPDATE:
					/* Acknowledging The BUFFER_FREE Interrupt */
					adapter->buf_status_interrupts++;
#ifdef SDIO_DEBUG
					adapter->timer_buf_status_interrupts++;
#endif
					adapter->osi_host_intf_ops->onebox_ack_interrupt(adapter, (1 << SD_BUFF_STATUS_UPDATE));
					status = adapter->osd_host_intf_ops->onebox_read_register(adapter,
					                                  ONEBOX_DEVICE_BUFFER_STATUS_REGISTER,
					                                  &device_buf_status); /* Buffer */
					if (status != ONEBOX_STATUS_SUCCESS)
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Failed to Buffer Status Register\n"),__func__));
						break;
					}
					if (d_assets->techs[WLAN_ID].drv_state == MODULE_ACTIVE) {
						if (d_assets->techs[WLAN_ID].onebox_get_buf_status) {
							d_assets->techs[WLAN_ID].onebox_get_buf_status(device_buf_status, d_assets);
						}
					}
				break;

				case FIRMWARE_ASSERT_IND:
          			{

					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				             (TEXT("%s: ==> FIRMWARE Assert <==\n"), __func__));
            				/* Whenever firmware hits an assert condition, it shall raise this
                 			 * interrupt to the host */
					status = adapter->osd_host_intf_ops->onebox_read_register(adapter,
		                                                          SDIO_FW_STATUS_REG,
		                                                          &fw_status); /* Buffer */
					if (status != ONEBOX_STATUS_SUCCESS)
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			        		     (TEXT("%s: Failed to Read  Firmware Status Register\n"),__func__));
					}
            				else
					{  
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
								(TEXT("%s: Firmware Status is 0x%08x\n"), __func__ , fw_status));
						if (d_assets->techs[WLAN_ID].drv_state == MODULE_ACTIVE) 
								d_assets->techs[WLAN_ID].wlan_dump_mgmt_pending(d_assets);
						//dump_wlan_pending_pkts_in_mgmt_queue();
						//adapter->osi_host_intf_ops->onebox_ack_interrupt(adapter, (1 << SD_FW_ASSERT_IND));
						status = panic_stop_all_activites(adapter, SD_FW_ASSERT_IND);
						return ;
					} /* End if <condition> */  
            	
					/* The driver should be stalled now */
            				adapter->fsm_state = FSM_CARD_NOT_READY;
										d_assets->card_state = GS_CARD_DETACH;
          			} 
				break;

				case MSDU_PACKET_PENDING:
				{
					onebox_enable_host_interrupt(adapter);
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
				             (TEXT("MSDU_PACKET_PENDING Interrupt Received\n")));
					adapter->total_sdio_msdu_pending_intr++;
#ifdef SDIO_DEBUG
					adapter->timer_total_sdio_msdu_pending_intr++;
#endif
					status = onebox_read_pkt(adapter);
					if (status != ONEBOX_STATUS_SUCCESS)
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Failed to read the received pkt\n"), __func__));
					}
				}
				break;
				default:
				{
					adapter->osi_host_intf_ops->onebox_ack_interrupt(adapter, sdio_interrupt_status);
					adapter->total_sdio_unknown_intr++;
#ifdef SDIO_DEBUG
					adapter->timer_total_sdio_unknown_intr++;
#endif
					sdio_interrupt_status = 0;
					/* Unknown Interrupt */
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				             (TEXT("Unknown Interrupt %x\n"), sdio_interrupt_status));
					onebox_enable_host_interrupt(adapter);
				}
				break;
			} /* End of Switch  */
  			/* Mask the corresponding interrupt serviced */		
			sdio_interrupt_status ^= BIT(interrupt_type - 1);
		}while(sdio_interrupt_status);
	}while (1);
	return;
}

/**
 * This function set the AHB master access MS word in the SDIO slave registers.
 *
 * @param  Pointer to the driver adapter structure. 
 * @param  ms word need to be initialized.
 * @return ONEBOX_STATUS_SUCCESS on success and ONEBOX_STATUS_FAILURE on failure. 
 */
ONEBOX_STATUS onebox_sdio_master_access_msword(PONEBOX_ADAPTER adapter,
                                               uint16 ms_word)
{
	UCHAR byte;
	uint8 reg_dmn;
	ONEBOX_STATUS  status=ONEBOX_STATUS_SUCCESS;

	reg_dmn = 0; //TA domain
	/* Initialize master address MS word register with given value*/
	byte=(UCHAR)(ms_word&0x00FF);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("%s: MASTER_ACCESS_MSBYTE:0x%x\n"), __func__,byte));
	status = adapter->osd_host_intf_ops->onebox_write_register(adapter,reg_dmn,
	                                                           SDIO_MASTER_ACCESS_MSBYTE,
	                                                           &byte);             
	if(status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: fail to access MASTER_ACCESS_MSBYTE\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}
	byte=(UCHAR)(ms_word >>8);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("%s:MASTER_ACCESS_LSBYTE:0x%x\n"), __func__,byte));
	status = adapter->osd_host_intf_ops->onebox_write_register(adapter,reg_dmn,
	                                                           SDIO_MASTER_ACCESS_LSBYTE,
	                                                           &byte);
	if(status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: fail to access MASTER_ACCESS_LSBYTE\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}
	return ONEBOX_STATUS_SUCCESS;
} /* End <onebox_sdio_master_access_msword */

int32 sdio_ta_reset_ops(PONEBOX_ADAPTER adapter)
{
	uint32 data; 
	if (onebox_sdio_master_access_msword(adapter, 0x2200)
			!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to set ms word to common reg\n"), __func__));
		return ONEBOX_STATUS_FAILURE;              
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Bringing TA Out of Reset\n"), __func__));
	data = TA_HOLD_THREAD_VALUE;
	/* Bringing TA out of reset */
	if(adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
				TA_HOLD_THREAD_REG |
				SD_REQUEST_MASTER,
				(uint8 *) &data,
				4, NULL
				)!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to hold TA threads\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}
	data = TA_SOFT_RST_CLR;
	/* Bringing TA out of reset */
	if(adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
				TA_SOFT_RESET_REG |
				SD_REQUEST_MASTER,
				(uint8 *) &data,
				4,
				NULL)!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to get TA out of reset state\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}
	/* Assuming TA will go to hold by this time
	 * If you find that TA is not in hold by this time
	 * in any chip or any project, then wait till TA goes to 
	 * hold by polling poll_status register. 
	 */
	data = TA_PC_ZERO;
	/* Bringing TA out of reset */
	if(adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
				TA_TH0_PC_REG |
				SD_REQUEST_MASTER,
				(uint8 *) &data,
				4,
				NULL)!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to Reset TA PC value\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}
	data = TA_RELEASE_THREAD_VALUE;
	/* Bringing TA out of reset */
	if(adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
				TA_RELEASE_THREAD_REG |
				SD_REQUEST_MASTER,
				(uint8 *) &data,
				4,
				NULL)!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to release TA threads\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: loading firmware\n"), __func__));
	if (onebox_sdio_master_access_msword(adapter, 0x4105)
			!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to set ms word to common reg\n"), __func__));
		return ONEBOX_STATUS_FAILURE;              
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Setting ms word to common reg 0x41050000\n"), __func__));
	return ONEBOX_STATUS_SUCCESS;
}
