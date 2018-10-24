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
#include "onebox_qspi.h"
#include "onebox_qspi_bootup_params.h"
#include "onebox_host_intf_ops.h"
#include "onebox_intf_ops.h"
#include "onebox_sdio_intf.h"
#include <linux/usb.h>
#include "onebox_zone.h"

static METADATA calib_data = {"calib_data", 0x00000000};

//#define ERASE_MBR_MAGICWORD /*Enable this macro to disable bootloader*/

#define FLASH_WRITE_CHUNK_SIZE (4 * 1024)
#define USB_FLASH_READ_CHUNK_SIZE  ((2 * 1024) - 4)
#define SDIO_FLASH_READ_CHUNK_SIZE  (2 * 1024)

uint32 read_flash_capacity(PONEBOX_ADAPTER adapter)
{
	uint32 flash_sz = 0;
	if(adapter->osd_host_intf_ops->onebox_master_reg_read(adapter, 
				FLASH_SIZE_ADDR, &flash_sz, 2) < 0) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d Flash size reading failed..\n",__func__,
				__LINE__));
		return 0;
	}
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Flash capacity is %d KiloBytes\n",flash_sz));
	return (flash_sz * 1024); /* Return size in bytes */
}

//extern ONEBOX_STATUS load_data_master_write(PONEBOX_ADAPTER adapter, uint32 base_address, uint32 instructions_sz,
//	uint32 block_size, uint8 *ta_firmware);

// This function is used to manually flash the contents on to the eeprom
ONEBOX_STATUS manual_flash_write(PONEBOX_ADAPTER adapter)
{
	
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  extended_desc = 0;
	uint8  *temp;
	netbuf_ctrl_block_t *netbuf_cb = NULL;
  	spi_config_t spi_config;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("===> Frame to PERFORM EEPROM WRITE <===\n")));

	if (adapter->qspi_flashing)
		extended_desc = 4;

	netbuf_cb = adapter->os_intf_ops->onebox_alloc_skb(FRAME_DESC_SZ + extended_desc + sizeof(spi_config_t));
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;

	}
	adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, (FRAME_DESC_SZ + extended_desc + sizeof(spi_config_t)));

	//mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	mgmt_frame = (onebox_mac_frame_t *)netbuf_cb->data;

	adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ + extended_desc + sizeof(spi_config_t));

	/* FrameType*/
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(EEPROM_WRITE);
	mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12);
	/* flags*/
	mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(3 << 8); //hsize = 3
	if (adapter->qspi_flashing)
		extended_desc = 4;
	mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(extended_desc); //extended_desc
	mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(ONEBOX_BIT(10)); //erase_flash 
	mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(ONEBOX_BIT(11)); //write_flash 
	mgmt_frame->desc_word[2] &= ~ONEBOX_CPU_TO_LE16(ONEBOX_BIT(12)); //dis_hw_ctrl
	mgmt_frame->desc_word[2] &= ~ONEBOX_CPU_TO_LE16(ONEBOX_BIT(13)); //reserved
	/* Number of bytes to read*/
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, (" eeprom length  0x%x, %d\n",adapter->eeprom.length,adapter->eeprom.length));
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, (" eeprom offset  0x%x, %d \n",adapter->eeprom.offset,adapter->eeprom.offset));
	mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16((adapter->eeprom.length & 0x03) << 14);
	mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(adapter->eeprom.length >> 2);
	/* Address to read*/
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(adapter->eeprom.offset);
	mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(adapter->eeprom.offset >> 16);
	mgmt_frame->desc_word[6] = ONEBOX_CPU_TO_LE16(256); 

	if (!(adapter->device_model == RSI_DEV_9116))  /* SECTOR MODE */
		mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(SECTOR_ERASE); //for sector erase
	else
	mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(BLOCK_ERASE); //for Block erase
	
	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(0x02 << 8 ); //for sector erase
	
	adapter->os_intf_ops->onebox_memcpy(&spi_config, &spi_default_configs, sizeof(spi_config_t));
	{
		if (!adapter->eeprom_erase || adapter->eeprom.eeprom_erase)
		{
			mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(BIT(10));
			adapter->eeprom_erase = 1;
			adapter->eeprom.eeprom_erase = 0;
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("ERASE FLASH \n"));
		}
		if (!(adapter->device_model == RSI_DEV_9116)) { /* SECTOR MODE */
			mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(BIT(10));
		}
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("B4 Final Writing %p\n",mgmt_frame));
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Flash offset 0x%x, %d\n",adapter->flash_offset, adapter->flash_offset));
		*(uint32 *)((uint16 *)&mgmt_frame->desc_word[7] + 1)  = ONEBOX_CPU_TO_LE32(adapter->flash_offset);	
		if (adapter->qspi_flashing || adapter->eeprom.write)
		{
			spi_config.spi_config_2.protection = REM_WR_PROT;
		}  
	}  
	temp = ((uint8 *)mgmt_frame + FRAME_DESC_SZ + extended_desc);
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, (" temp Writing %p\n",temp));
	adapter->os_intf_ops->onebox_memcpy(temp, &spi_config, sizeof(spi_config_t));
	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)&spi_default_configs, sizeof(spi_config_t));
	
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, (" Final Writing %p\n",mgmt_frame));
	mgmt_frame->desc_word[0] |= ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12 | (extended_desc*3));
	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)mgmt_frame, 
				       FRAME_DESC_SZ + extended_desc + sizeof(spi_config_t));
#if 0
	status = adapter->osi_host_intf_ops->onebox_host_intf_write_pkt(adapter, 
			 (uint8 *)mgmt_frame, (extended_desc + FRAME_DESC_SZ + sizeof(spi_config_t)),
			 WLAN_TX_M_Q);
#endif

	status = adapter->osi_host_intf_ops->onebox_host_intf_write_pkt(adapter, 
			 netbuf_cb->data, netbuf_cb->len,
			 WLAN_TX_M_Q, netbuf_cb);

	if(adapter->host_intf_type == HOST_INTF_SDIO)
		adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
  	return status;

}

// This function is used to automatically flash the contents on to the eeprom
ONEBOX_STATUS auto_flash_write(PONEBOX_ADAPTER adapter, uint8 *flash_content, uint32 block_size)
{
	
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  *pkt_buffer;
	uint16 pkt_len = 0;
	spi_config_t spi_config;
	uint8  *temp;
	uint8 extended_desc = 0;
	netbuf_ctrl_block_t *netbuf_cb = NULL;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("===> Frame to PERFORM EEPROM WRITE <===\n")));
#define DELAYS_LEN 12
	if (adapter->device_model == RSI_DEV_9116) {
		extended_desc = 4;
		pkt_len = (FRAME_DESC_SZ + block_size + extended_desc + sizeof(spi_config_t));
	}
	else { 
		pkt_len = (FRAME_DESC_SZ + block_size + DELAYS_LEN);
	}
	pkt_buffer = adapter->os_intf_ops->onebox_mem_zalloc(pkt_len, GFP_KERNEL);
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);
	

	/* FrameType*/
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(EEPROM_WRITE);
	//mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12);
	/* flags*/
	mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(3 << 8); //hsize = 3
	if (adapter->eeprom_erase)
	{
	  	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Flash erase sent"));
		mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(BIT(10));
	}  
	if(adapter->eeprom_init)
	{
	  	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("spi init sent"));
		mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(BIT(13));
	}
	if (adapter->device_model == RSI_DEV_9116) {
		mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(3 << 8); //hsize = 3
		mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(extended_desc); //extended_desc
	}
	else {
		//mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(ONEBOX_BIT(10)); //erase_flash 
		mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(ONEBOX_BIT(11)); //write_flash 
		mgmt_frame->desc_word[2] &= ~ONEBOX_CPU_TO_LE16(ONEBOX_BIT(12)); //dis_hw_ctrl
	}
	/* Number of bytes to read*/
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, (" eeprom length  0x%x, %d\n",adapter->eeprom.length,adapter->eeprom.length));
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, (" eeprom offset  0x%x, %d \n",adapter->eeprom.offset,adapter->eeprom.offset));
	if (adapter->device_model == RSI_DEV_9116) {
		mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16((adapter->eeprom.length & 0x03) << 14);
		mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(adapter->eeprom.length >> 2);
	} 
	else{
		mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(adapter->eeprom.length << 4);
	}
	/* Address to read*/
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(adapter->eeprom.offset);
	mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(adapter->eeprom.offset >> 16);
	if (!(adapter->device_model == RSI_DEV_9116)) {
		mgmt_frame->desc_word[6] = ONEBOX_CPU_TO_LE16(256); 

		mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(SECTOR_ERASE); //for sector erase

	mgmt_frame->desc_word[7] |= ONEBOX_CPU_TO_LE16(0x02 << 8 ); //Write Command
	
	temp = ((uint8 *)mgmt_frame + FRAME_DESC_SZ);

	//! delay
	*(uint32 *)temp = 0; 
	temp += 4; 
	//! delay
	*(uint32 *)temp = 0; 
	temp += 4;
	//! delay
	*(uint32 *)temp = 0; 
	temp += 4;

	adapter->os_intf_ops->onebox_memcpy(temp, flash_content, block_size);
	}
	else {	
		temp = ((uint8 *)mgmt_frame + FRAME_DESC_SZ + extended_desc);
		adapter->os_intf_ops->onebox_memcpy(&spi_config, &spi_default_configs, sizeof(spi_config_t));
		spi_config.spi_config_2.protection = REM_WR_PROT;
		adapter->os_intf_ops->onebox_memcpy(temp, &spi_config, sizeof(spi_config_t));
		temp = ((uint8 *)mgmt_frame + FRAME_DESC_SZ + extended_desc + sizeof(spi_config_t));
		adapter->os_intf_ops->onebox_memcpy(temp, flash_content, block_size);
	}
	
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, (" Final Writing %p\n",mgmt_frame));
	if (!(adapter->device_model == RSI_DEV_9116))
		mgmt_frame->desc_word[0] |= ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12 | (pkt_len - FRAME_DESC_SZ));
	else
		mgmt_frame->desc_word[0] |= ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12 | (block_size + extended_desc*3));

#if 1	
	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)mgmt_frame, 
			//FRAME_DESC_SZ + pkt_len );
		FRAME_DESC_SZ + 50 );
#endif	

#if 1	
	netbuf_cb = adapter->os_intf_ops->onebox_alloc_skb(pkt_len);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;

	}
	adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);
	adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, (uint8 *)mgmt_frame, pkt_len);
  
	status = adapter->osi_host_intf_ops->onebox_host_intf_write_pkt(adapter, 
			 						netbuf_cb->data, netbuf_cb->len ,
			 						WLAN_TX_M_Q, netbuf_cb);

	if (adapter->host_intf_type != HOST_INTF_USB) 
		adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);

	adapter->os_intf_ops->onebox_mem_free(pkt_buffer);
#endif	
  return status;
}

// This function is used to read the contents of the flash
ONEBOX_STATUS flash_read(PONEBOX_ADAPTER adapter)
{
	uint16 pkt_len = 0;
	uint8  *pkt_buffer;

	onebox_mac_frame_t *mgmt_frame;
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	ONEBOX_STATUS status;
	//uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	pkt_len = FRAME_DESC_SZ;
	pkt_buffer = adapter->os_intf_ops->onebox_mem_zalloc(pkt_len, GFP_KERNEL);
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("===> Frame to PERFORM FLASH READ <===\n")));

	/* FrameType*/
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(EEPROM_READ_TYPE);
	//	mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12);

	/* Format of length and offset differs for autoflashing and swbl flashing */
	if(adapter->Driver_Mode == QSPI_FLASHING) {
		mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12 | (pkt_len - FRAME_DESC_SZ));

		/* Number of bytes to read*/
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, (" eeprom length  0x%x, %d\n",adapter->eeprom.length,adapter->eeprom.length));
		mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(adapter->eeprom.length << 4);
	} else {
		mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12);

		/* Number of bytes to read*/
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, (" eeprom length  0x%x, %d\n",adapter->eeprom.length,adapter->eeprom.length));
		mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(adapter->eeprom.length);
	}  

	mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(3 << 8); //hsize = 3 as 32 bit transfer
	if(adapter->eeprom_init)
	{
	  	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("spi init sent"));
		mgmt_frame->desc_word[2] |= ONEBOX_CPU_TO_LE16(BIT(13));
	}

	/* Address to read*/
	mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(adapter->eeprom.offset);
	mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(adapter->eeprom.offset >> 16);
  	mgmt_frame->desc_word[6] = ONEBOX_CPU_TO_LE16(0); //delay = 0

	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)mgmt_frame, pkt_len);

#if 1	
	netbuf_cb = adapter->os_intf_ops->onebox_alloc_skb(pkt_len);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;

	}
	adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);
	adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, (uint8 *)mgmt_frame, pkt_len);
  
	status = adapter->osi_host_intf_ops->onebox_host_intf_write_pkt(adapter, 
			 						netbuf_cb->data, netbuf_cb->len ,
			 						WLAN_TX_M_Q, netbuf_cb);

	if (adapter->host_intf_type != HOST_INTF_USB)
		adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
	adapter->os_intf_ops->onebox_mem_free(pkt_buffer);
#endif	
	return status;
}

// This function is used to send the final flashing status
ONEBOX_STATUS send_flashing_status(PONEBOX_ADAPTER adapter, uint8 status)
{
	uint16 pkt_len = 0;
	uint8  *pkt_buffer;
	onebox_mac_frame_t *mgmt_frame;
	netbuf_ctrl_block_t *netbuf_cb = NULL;

	pkt_len = FRAME_DESC_SZ;
	pkt_buffer = adapter->os_intf_ops->onebox_mem_zalloc(pkt_len, GFP_KERNEL);
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("===> Frame to send flashing status <===\n")));
	/* FrameType*/
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(TX_MISC_IND);

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12 | (pkt_len - FRAME_DESC_SZ));

	mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(status);

	mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(FW_UPGRADE_DONE << 8);
	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)mgmt_frame, pkt_len);
#if 0
	status = adapter->osi_host_intf_ops->onebox_host_intf_write_pkt(adapter, 
			(uint8 *)mgmt_frame, pkt_len,
			WLAN_TX_M_Q);
	adapter->os_intf_ops->onebox_mem_free(pkt_buffer);
#endif

#if 1	
	netbuf_cb = adapter->os_intf_ops->onebox_alloc_skb(pkt_len);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;

	}
	adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);
	adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, (uint8 *)mgmt_frame, pkt_len);
  
	status = adapter->osi_host_intf_ops->onebox_host_intf_write_pkt(adapter, 
			 						netbuf_cb->data, netbuf_cb->len ,
			 						WLAN_TX_M_Q, netbuf_cb);

	if (adapter->host_intf_type != HOST_INTF_USB)
		adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
	adapter->os_intf_ops->onebox_mem_free(pkt_buffer);
#endif	
	return status;
}

static ONEBOX_STATUS trigger_manual_flash_burn( PONEBOX_ADAPTER adapter, 
				  	 METADATA *meta_rcv, 
				  	 uint32 instructions_sz, 
				  	 uint32 iteration, 
				  	 uint32 eeprom_offset
				       )
{

	uint32 status = 0;	
	uint8 num_loops = 0, idx;
	uint32 chunk_size = 0;
	uint32 flash_chunk_size=0;

	if (!(adapter->device_model == RSI_DEV_9116)) { /* --- SECTOR MODE --- */
		flash_chunk_size = FLASH_SECTOR_SIZE;
	}
	else {
		flash_chunk_size = FLASH_BLOCK_SIZE;
	}

	num_loops = instructions_sz / flash_chunk_size;

	if(instructions_sz % flash_chunk_size) {
			num_loops++;
	}
	adapter->eeprom.offset = eeprom_offset;

  	adapter->flash_offset = meta_rcv->address;
	if (iteration % 2)
	{
		meta_rcv->address = 0x00010000;
		adapter->eeprom.eeprom_erase = 0;
	}
	else
	{
		//meta_rcv->address = 0x0001C000;
		meta_rcv->address = 0x00010000;
		adapter->eeprom.eeprom_erase = 1;
	}	
	
  	for (idx = 0; idx < num_loops; idx++)
	{
		if(instructions_sz < flash_chunk_size) {
			chunk_size = instructions_sz;
		} else {
			chunk_size = flash_chunk_size;
		}
		adapter->eeprom.length = chunk_size;
	status = adapter->coex_osi_ops->onebox_manual_flash_write(adapter);
	if (status == ONEBOX_STATUS_SUCCESS)
	{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT ("%s: BLOCK/SECTOR WRITING SUCCESSFULL \n"), __FUNCTION__));
		adapter->os_intf_ops->onebox_reset_event(&adapter->flash_event);
		adapter->os_intf_ops->onebox_wait_event(&(adapter->flash_event),  12000); 
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: BLOCK/SECTOR WRITING TO FLASH FAILED\n"), __FUNCTION__));
		return ONEBOX_STATUS_FAILURE; 
	}	
		adapter->eeprom.offset += chunk_size;
		adapter->flash_offset += chunk_size;
		instructions_sz -= chunk_size;
	}
	return ONEBOX_STATUS_SUCCESS; 
}

static ONEBOX_STATUS trigger_auto_flash_burn( PONEBOX_ADAPTER adapter, 
				       uint8 *flash_content, 
				       uint32 instructions_sz, 
				       uint32 eeprom_offset
				     )
{

	uint32 status = 0;	
	uint32 num_loops = 0, idx;
	uint32 chunk_size = 0;
 	uint32 flash_chunk_size=0; 


	if (!(adapter->device_model == RSI_DEV_9116)) /* *** SECTOR MODE *** */
		flash_chunk_size = FLASH_WRITE_CHUNK_SIZE;
	else
		flash_chunk_size = 256;

	num_loops = instructions_sz / flash_chunk_size;

	adapter->eeprom_init = 1;
	if(instructions_sz % flash_chunk_size) {
			num_loops++;
	}
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Number of loops required is %d\n",num_loops));
	for (idx = 0; idx < num_loops; idx++)
	{
		if(instructions_sz < flash_chunk_size) {
			chunk_size = instructions_sz;
		} else {
			chunk_size = flash_chunk_size;
		}
		if (!(adapter->device_model == RSI_DEV_9116)) {  
			if ((idx % (FLASH_SECTOR_SIZE/FLASH_WRITE_CHUNK_SIZE)) == 0) {
				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("idx is %d and hence calling erase\n", idx));
				adapter->eeprom_erase = 1;
			}
		} 
		else if( idx == 0 ) {
			adapter->eeprom_erase = 1;
		}
		else { 
			adapter->eeprom_erase = 0;
		}

		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("idx is %d and chunk size is %d\n", idx, chunk_size));
		adapter->eeprom.offset = eeprom_offset;
		adapter->eeprom.length = chunk_size;
		status = adapter->coex_osi_ops->onebox_auto_flash_write(adapter,&flash_content[idx * flash_chunk_size], chunk_size);
		if (status == ONEBOX_STATUS_SUCCESS)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT ("%s: BLOCK/SECTOR WRITING SUCCESSFULL \n"), __FUNCTION__));
			adapter->os_intf_ops->onebox_reset_event(&adapter->flash_event);
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("idx + 1 is %d and eeprom_cfm_count is %d\n",idx + 1, adapter->eeprom_cfm_count));
			//! No of flash write frames - flash write confirms should less than 2(.i.e. Writing one frame in pipeline)
			if(((idx + 1) - adapter->eeprom_cfm_count) >= 2) {
				if (!(adapter->device_model == RSI_DEV_9116)) {
					status = adapter->os_intf_ops->onebox_wait_event(&(adapter->flash_event),  12000); 
					if(!status) {
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
								(TEXT ("%s: TIMEOUT ELAPSED WHILE FLASH WRITING\n"), __FUNCTION__));
						return ONEBOX_STATUS_FAILURE;	
					}
				} else {
					status = adapter->os_intf_ops->onebox_wait_event(&(adapter->flash_event),EVENT_WAIT_FOREVER ); 
				}
			}
		}
		else
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: BLOCK/SECTOR WRITING TO FLASH FAILED\n"), __FUNCTION__));
			return ONEBOX_STATUS_FAILURE; 
		}	
		eeprom_offset += chunk_size;
		instructions_sz -= chunk_size;
		
		adapter->eeprom_init = 0;
	}
	while(idx != adapter->eeprom_cfm_count) {
		adapter->os_intf_ops->onebox_reset_event(&adapter->flash_event);
		status = adapter->os_intf_ops->onebox_wait_event(&(adapter->flash_event),  12000); 
		if(!status) {
		  ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		      (TEXT ("%s: TIMEOUT IN PIPELINING\n"), __FUNCTION__));
		  return ONEBOX_STATUS_FAILURE;	
		}
	}
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("idx is %d and eeprom_cfm_count is %d\n",idx, adapter->eeprom_cfm_count));
	return ONEBOX_STATUS_SUCCESS; 
}

ONEBOX_STATUS verify_flash_content( PONEBOX_ADAPTER adapter, 
		uint8 *flash_content, 
		uint32 instructions_sz, 
		uint32 eeprom_offset,
		uint8  read_mode
		)
{
	uint32 status = 0;	
	uint32 num_loops = 0, idx;
	uint32 chunk_size = 0;
	uint8 *dest_addr = NULL;
	uint32 addr = 0;
	
	uint32 flash_chunk_size ;
	
	if(adapter->host_intf_type == HOST_INTF_USB)
		flash_chunk_size = USB_FLASH_READ_CHUNK_SIZE;
	else
		flash_chunk_size = SDIO_FLASH_READ_CHUNK_SIZE;

	num_loops = instructions_sz / flash_chunk_size;

	if(instructions_sz % flash_chunk_size) {
			num_loops++;
	}

	if(read_mode == EEPROM_READ_MODE) {
		dest_addr = &adapter->DataRcvPacket[0];
	}else {
		dest_addr  = adapter->os_intf_ops->onebox_mem_zalloc(instructions_sz, GFP_KERNEL);
		if(dest_addr ==NULL)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Memory allocation for dest_addr failed\n"), __func__));   
			return ONEBOX_STATUS_FAILURE;
		}
	}

	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Number of loops required is %d\n",num_loops));
	for (idx = 0; idx < num_loops; idx++)
	{
		if(instructions_sz < flash_chunk_size) {
			chunk_size = instructions_sz;
		} else {
			chunk_size = flash_chunk_size;
		}
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("idx is %d and chunk size is %d\n", idx, chunk_size));
		if(read_mode == EEPROM_READ_MODE) {
			adapter->eeprom.offset = eeprom_offset;
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("eeprom offset is %x\n", eeprom_offset));
			adapter->eeprom.length = chunk_size;
			status = adapter->coex_osi_ops->onebox_flash_read(adapter);
			if (status == ONEBOX_STATUS_SUCCESS) {
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT ("%s: BLOCK/SECTOR READING SUCCESSFULL \n"), __FUNCTION__));
				adapter->os_intf_ops->onebox_reset_event(&adapter->flash_event);
				status = adapter->os_intf_ops->onebox_wait_event(&(adapter->flash_event),  12000); 
				if(!status) {
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
							(TEXT ("%s: TIMEOUT ELAPSED WHILE FLASH READING\n"), __FUNCTION__));
					return ONEBOX_STATUS_FAILURE;	
				}
			} else {
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: READING FROM FLASH FAILED\n"), __FUNCTION__));
				return ONEBOX_STATUS_FAILURE; 
			}
		}else {
			adapter->os_intf_ops->onebox_memset(dest_addr, 0, chunk_size);
			addr = SOC_FLASH_ADDR + eeprom_offset;
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Reading flash addr 0x%0x\n",addr));
			if(read_flash_content(adapter, dest_addr, 
						addr, 
						flash_chunk_size) < 0) { 
				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:Failed to read calib data using master read\n",__func__));
				adapter->os_intf_ops->onebox_mem_free(dest_addr);
				return ONEBOX_STATUS_FAILURE; 
			}
		}
#if 0
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("EXPECTED DUMP\n"));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, &flash_content[idx * flash_chunk_size], chunk_size);

		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("RECEIVED DUMP\n"));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, dest_addr, chunk_size);
#endif
		if(adapter->os_intf_ops->onebox_memcmp(&flash_content[idx * flash_chunk_size], 
						       dest_addr, 
						       chunk_size)) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: VERIFICATION OF FLASH CHUNK FAILED\n"), __FUNCTION__));
			return ONEBOX_STATUS_FAILURE;	
		} else {
			eeprom_offset += chunk_size;
			instructions_sz -= chunk_size;
		}
	}
	return ONEBOX_STATUS_SUCCESS; 
}

ONEBOX_STATUS read_flash_content(PONEBOX_ADAPTER adapter, uint8 *temp_buf, uint32 address, uint32 len)
{

#ifdef USE_SDIO_INTF
	if (adapter->host_intf_type == HOST_INTF_SDIO) {
		if (onebox_sdio_master_access_msword(adapter, address >> 16)
				!= ONEBOX_STATUS_SUCCESS)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: Unable to set ms word to common reg\n"), __func__));
			return ONEBOX_STATUS_FAILURE;              
		}
		address &= 0xFFFF; 
		//FOR SDIO MODE Please look into this.
		return adapter->osd_host_intf_ops->onebox_read_multiple(adapter,
				address | SD_REQUEST_MASTER,
				len,
				(uint8 *)(temp_buf));
	} 
#endif

#ifdef USE_USB_INTF
	if (adapter->host_intf_type == HOST_INTF_USB) {
		return adapter->osd_host_intf_ops->onebox_ta_read_multiple(adapter,
				address,
				(uint8 *)(temp_buf),
				len);
	}
#endif
	return ONEBOX_STATUS_FAILURE;
}


#define MAX_SCATTERS_CHECKSUM  1
static ONEBOX_STATUS read_calib_data(PONEBOX_ADAPTER adapter, uint8 *flash_content, uint8 read_mode)
{

	uint32 status = 0;	
	uint32 num_loops = 0, idx;
	uint32 chunk_size = 0;
	uint32 instructions_sz = EEPROM_DATA_SIZE;
	uint32 eeprom_offset = 0;
	uint8  *temp_buffer = NULL;
	uint32 addr = 0;
	uint32 calc_crc = 0, read_crc = 0;
	uint8 *checksum_scatter_addr[MAX_SCATTERS_CHECKSUM];
	uint32 checksum_scatter_len[MAX_SCATTERS_CHECKSUM];
	uint32 flash_chunk_size ;
	
	if(adapter->host_intf_type == HOST_INTF_USB)
		flash_chunk_size = USB_FLASH_READ_CHUNK_SIZE;
	else
		flash_chunk_size = SDIO_FLASH_READ_CHUNK_SIZE;
	
	if(adapter->flashing_mode == SWBL_FLASHING_NOSBL) {
		num_loops = instructions_sz / flash_chunk_size;

		if(instructions_sz % flash_chunk_size) {
			num_loops++;
		}
		temp_buffer = adapter->os_intf_ops->onebox_mem_zalloc(instructions_sz, GFP_KERNEL);
		if(temp_buffer == NULL)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Memory allocation for temp buffer failed\n"), __func__));   
			return ONEBOX_STATUS_FAILURE;
		}

		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Number of loops required is %d\n",num_loops));
		for (idx = 0; idx < num_loops; idx++)
		{
			if(instructions_sz < flash_chunk_size) {
				chunk_size = instructions_sz;
			} else {
				chunk_size = flash_chunk_size;
			}
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("idx is %d and chunk size is %d\n", idx, chunk_size));
			if(read_mode == EEPROM_READ_MODE) {
				adapter->eeprom.offset = eeprom_offset;
				adapter->eeprom.length = chunk_size;
				status = adapter->coex_osi_ops->onebox_flash_read(adapter);
				if (status == ONEBOX_STATUS_SUCCESS)
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT ("%s: BLOCK/SECTOR READING SUCCESSFULL \n"), __FUNCTION__));
					adapter->os_intf_ops->onebox_reset_event(&adapter->flash_event);
					status = adapter->os_intf_ops->onebox_wait_event(&(adapter->flash_event),  12000); 
					if(!status) {
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
								(TEXT ("%s: TIMEOUT ELAPSED WHILE FLASH READING\n"), __FUNCTION__));
						goto fail;
					}
				}
				else
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: READING FROM FLASH FAILED\n"), __FUNCTION__));
					goto fail; 
				}	
			} else {
				ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Reading flash addr 0x%0x\n",addr));
				addr = SOC_FLASH_ADDR + eeprom_offset;
				ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Reading flash addr 0x%0x\n",addr));
				if(read_flash_content(adapter, &adapter->DataRcvPacket[0], 
							addr, 
							flash_chunk_size) < 0) { 
					ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:Failed to read calib data using master read\n",__func__));
					goto fail;
				}
			}	
			adapter->os_intf_ops->onebox_memcpy(&temp_buffer[idx * flash_chunk_size], 
					&adapter->DataRcvPacket[0], 
					chunk_size);
			eeprom_offset += chunk_size;
			instructions_sz -= chunk_size;
		}
	} else {
		instructions_sz = 0;
		temp_buffer = (uint8 *)adapter->os_intf_ops->onebox_get_firmware(calib_data.name,
				(uint32 *)&instructions_sz,
				BIN_FILE,(uint8 *)adapter->firmware_path);
		if (temp_buffer == NULL) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
						("%s: Failed to open file %s\n"), __func__,
						calib_data.name));
			goto fail;
		}
		if(instructions_sz < EEPROM_DATA_SIZE) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
						("%s: Size of the file less than 4K %d\n"), __func__,
						instructions_sz));
			adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, temp_buffer, EEPROM_DATA_SIZE);
			goto fail;
		}
	}

	ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Calibration Data Dump\n"));
	/* XXX:Don't change the zone.. This dump is really necessary*/
	adapter->coex_osi_ops->onebox_dump(1, temp_buffer, EEPROM_DATA_SIZE);
	
	if(temp_buffer[MAGIC_WORD_OFFSET_1] != MAGIC_WORD ||
		temp_buffer[MAGIC_WORD_OFFSET_2] != MAGIC_WORD) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Invalid Calibration data present.. Skipping Flashing\n"));
		goto fail;
	}

	if(temp_buffer[EEPROM_VERSION_OFFSET] >= EEPROM_VER6) {
					
		checksum_scatter_addr[0] = (uint8 *) &temp_buffer[CALIB_VALUES_START_ADDR];
		checksum_scatter_len[0]  = (CALIB_DATA_SIZE - 4);

		calc_crc = checksum_32bit(&checksum_scatter_addr[0], &checksum_scatter_len[0], MAX_SCATTERS_CHECKSUM);
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("checksum = %02x\n", calc_crc));

		read_crc = ONEBOX_CPU_TO_LE32(*(uint32 *)&temp_buffer[CALIB_CRC_OFFSET]);
		if(calc_crc != read_crc) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("CRC verification Failed.. calc_crc:%x and read_crc:%x"
													"Skipping Flashing\n",calc_crc,read_crc ));
									goto fail;
					}
	}

	if(adapter->flashing_mode == SWBL_FLASHING_NOSBL) {
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Started writing calibration data to a file\n"));
		status = adapter->os_intf_ops->onebox_write_to_file("backup_calib",
				(uint16 *)temp_buffer,
				4096,
				0,
				(uint8 *)adapter->firmware_path); 
		if(status == ONEBOX_STATUS_FAILURE) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Failed to write calib data to a file\n"));
		}
	}

	adapter->os_intf_ops->onebox_memcpy(&flash_content[CALIB_VALUES_START_ADDR], 
			&temp_buffer[CALIB_VALUES_START_ADDR], 
			CALIB_DATA_SIZE);
	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, flash_content, 64);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT ("%s: Successfully overwritten Calib data\n"), __FUNCTION__));

	if(adapter->flashing_mode == SWBL_FLASHING_NOSBL) {
		adapter->os_intf_ops->onebox_mem_free(temp_buffer);
	} else {
		adapter->os_intf_ops->onebox_vmem_free(temp_buffer);
	} 
	return ONEBOX_STATUS_SUCCESS; 
fail:
	if(adapter->flashing_mode == SWBL_FLASHING_NOSBL) {
		adapter->os_intf_ops->onebox_mem_free(temp_buffer);
	} else {
		adapter->os_intf_ops->onebox_vmem_free(temp_buffer);
	} 
	return ONEBOX_STATUS_FAILURE;
}

ONEBOX_STATUS manual_load_flash_content(PONEBOX_ADAPTER adapter, METADATA metadata_rcv)
{
	uint8 *ta_firmware = NULL;
	uint8 *ta_firmware_temp = NULL;
	uint8 *flash_start_ptr = NULL;
	uint32 instructions_sz;
	uint32 temp_instructions_sz;
	uint32 num_flash;
	uint32 block_size = adapter->TransmitBlockSize;
	uint32 base_address;
	uint32 kk;
	uint32 flash_indx =0;
	uint32 flash_capacity = 0;
	ONEBOX_STATUS status = ONEBOX_STATUS_FAILURE;
  	uint32 flash_start_address;

	/* Loading in non-multi block mode */
	ta_firmware = (uint8 *)adapter->os_intf_ops->onebox_get_firmware(metadata_rcv.name,
			(uint32 *)&instructions_sz,
			BIN_FILE,(uint8 *)adapter->firmware_path);
	if (ta_firmware == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
					("%s: Failed to open file %s\n"), __func__,
					metadata_rcv.name));
		return ONEBOX_STATUS_FAILURE;
	}
	ta_firmware_temp = ta_firmware;

	if(instructions_sz > MAX_FLASH_FILE_SIZE) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Content size is more than 400K i.e %u and so, not flashing\n",MAX_FLASH_FILE_SIZE));
		adapter->os_intf_ops->onebox_vmem_free(ta_firmware_temp);
		return ONEBOX_STATUS_FAILURE;
	}

	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, ta_firmware, 64);
	if (adapter->flashing_mode == QSPI_UPDATE) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("First 4K in new Flash File \n "));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, ta_firmware, EEPROM_DATA_SIZE);
		read_flash_content(adapter,
				(ta_firmware + CALIB_VALUES_START_ADDR),
				(SOC_FLASH_ADDR + CALIB_VALUES_START_ADDR),
				CALIB_DATA_SIZE);
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("First 4K from existing Flash on card \n"));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, ta_firmware, EEPROM_DATA_SIZE);
		adapter->flashing_mode = QSPI_FLASHING;
	}
	if (adapter->flashing_mode == QSPI_FLASHING) {
		if (!(adapter->device_model == RSI_DEV_9116)) { /* SECTOR MODE */   
			adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, &ta_firmware[FLASHING_START_ADDRESS], 10);

			flash_start_address = ONEBOX_CPU_TO_LE32(*(uint32 *)&ta_firmware[FLASHING_START_ADDRESS]);
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("flash start address is %08x\n",flash_start_address));

			if(flash_start_address <= FLASH_SECTOR_SIZE) {
				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Start Address is less than 4K.. So not flashing\n"));
				adapter->os_intf_ops->onebox_vmem_free(ta_firmware_temp);
				return ONEBOX_STATUS_FAILURE;
			}

			if(flash_start_address % FLASH_SECTOR_SIZE) {
				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Start Address is not multiple of 4K.. So not flashing\n"));
				adapter->os_intf_ops->onebox_vmem_free(ta_firmware_temp);
				return ONEBOX_STATUS_FAILURE;
			}

			flash_capacity = read_flash_capacity(adapter);
			if(flash_capacity <= 0) {
				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Unable to read flash size from EEPROM\n"));
				adapter->os_intf_ops->onebox_vmem_free(ta_firmware_temp);
				return ONEBOX_STATUS_FAILURE;
			}

			if(flash_start_address + instructions_sz > flash_capacity) {
				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Content will cross max flash size.. So not flashing\n"));
				adapter->os_intf_ops->onebox_vmem_free(ta_firmware_temp);
				return ONEBOX_STATUS_FAILURE;
			}
			flash_indx = flash_start_address;
		}
		else {
			flash_indx = STARTING_BLOCK_INDEX * FLASH_BLOCK_SIZE;
		}
	}
	if (adapter->flashing_mode == SWBL_FLASHING_NOSBL ||
			adapter->flashing_mode == SWBL_FLASHING_NOSBL_FILE) {
		flash_indx = 0;

		/* In sw bootloader, skip first 32 bytes header from flashing */
		ta_firmware += BL_HEADER;
		instructions_sz -= BL_HEADER;

#ifdef ERASE_MBR_MAGICWORD
		*(uint16 *)&ta_firware[0] = 0;
#endif		
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, ta_firmware, 64);
		status = read_calib_data(adapter, ta_firmware, EEPROM_READ_MODE); 
		if(status == ONEBOX_STATUS_FAILURE) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Failed to read calibration data\n"), __func__));   
			return ONEBOX_STATUS_FAILURE;
		}
	}

#if 0
	if (instructions_sz % 4)
	{   
		instructions_sz += (4 - (instructions_sz % 4));
	}
#endif
	temp_instructions_sz = instructions_sz ;
	flash_start_ptr = ta_firmware;

	num_flash = instructions_sz / FLASH_BLOCK_SIZE;
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("instructions_sz:%d\n"),
				instructions_sz));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("num_flash: %d\n"),num_flash));

	for (kk = 0; kk <= num_flash; kk++,flash_indx += FLASH_BLOCK_SIZE)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("flash_indx: %d\n"),flash_indx));
		if (adapter->qspi_flashing)
		{
			if (kk != num_flash)
			{	
				instructions_sz = FLASH_BLOCK_SIZE;
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("QSPI instructions_sz:%d\n"),
							instructions_sz));
			}
			else
			{
				instructions_sz = temp_instructions_sz % FLASH_BLOCK_SIZE;
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("QSPI LAST BLOCK BEING WRITTEN instructions_sz:%d\n"),
							instructions_sz));
				if (!instructions_sz)
				{ 
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("INSTRUCTION SIZE ZEROOOOOOOOO: \n")));
					break;
				}  
			}	
		}
		if (!instructions_sz)
			continue;
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("QSPI metadata_rcv.address :0x%x\n"),
					metadata_rcv.address));
		base_address = metadata_rcv.address;

#ifdef USE_USB_INTF
		if(adapter->host_intf_type == HOST_INTF_USB) {
			status = usb_load_data_master_write(adapter, 
					base_address, 
					instructions_sz, 
					block_size, 
					ta_firmware);
		} 
#endif
		
#ifdef USE_SDIO_INTF
		if(adapter->host_intf_type == HOST_INTF_SDIO) {
			status = sdio_load_data_master_write(adapter, 
					base_address, 
					instructions_sz, 
					block_size, 
					ta_firmware);

		}
#endif


		if(ONEBOX_STATUS_FAILURE == status) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: Unable to load %s blk\n"), __func__,
					 metadata_rcv.name));
			adapter->os_intf_ops->onebox_vmem_free(ta_firmware_temp); 
			return ONEBOX_STATUS_FAILURE;
		}

		ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
				(TEXT("%s: Succesfully loaded %s instructions\n"), __func__,
				 metadata_rcv.name));
		ta_firmware += instructions_sz; 
		trigger_manual_flash_burn(adapter, &metadata_rcv, instructions_sz, kk,flash_indx);
	}

	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("%s:Starting Flash Verification Process\n",__func__));
	status = verify_flash_content(adapter, flash_start_ptr, 
			//temp_instructions_sz, STARTING_BLOCK_INDEX, MASTER_READ_MODE);
			temp_instructions_sz, STARTING_BLOCK_INDEX, EEPROM_READ_MODE);
	if(status == ONEBOX_STATUS_FAILURE) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("flash index = %d\n",flash_indx));
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:FLASHING Firmware failed in FLASH VERIFICATION phase\n",__func__));
		adapter->os_intf_ops->onebox_vmem_free(ta_firmware_temp); 
		return ONEBOX_STATUS_FAILURE;
	}
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("%s:Flash Verification Process Completed Successfully\n",__func__));
	adapter->os_intf_ops->onebox_vmem_free(ta_firmware_temp);
	return ONEBOX_STATUS_SUCCESS;	
}

ONEBOX_STATUS auto_fw_upgrade(PONEBOX_ADAPTER adapter, uint8 *flash_content, uint32 content_size)
{
	uint8  cmd;
	uint8  *temp_flash_content;
	uint32  temp_content_size;
	uint32 num_flash;
	uint32 kk;
	uint32 flash_start_address;

	temp_flash_content = flash_content;

	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, &flash_content[FLASHING_START_ADDRESS], 10);

	if(content_size > MAX_FLASH_FILE_SIZE) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Content size is more than 400K i.e %u and so, not flashing\n",MAX_FLASH_FILE_SIZE));
		goto fail;
	}

	flash_start_address = ONEBOX_CPU_TO_LE32(*(uint32 *)&flash_content[FLASHING_START_ADDRESS]);
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("flash start address is %08x\n",flash_start_address));

	if(adapter->Driver_Mode == QSPI_FLASHING &&
			adapter->flashing_mode == SWBL_FLASHING_SBL) {
		if(flash_start_address < FLASH_SECTOR_SIZE) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Start Address is less than 4K.. So not flashing\n"));
			goto fail;
		}
	} else {
		if(flash_start_address < FW_IMAGE_MIN_ADDRESS) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Fw image Flash Start Address is less than 68K.. So not flashing\n"));
			goto fail;
		}
	}

	if(flash_start_address % FLASH_SECTOR_SIZE) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Start Address is not multiple of 4K.. So not flashing\n"));
		goto fail;
	}
	if (!(adapter->device_model == RSI_DEV_9116)) {
		if(flash_start_address + content_size > adapter->flash_capacity) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Content will cross max flash size.. So not flashing\n"));
			goto fail;
		}
	}
#if 0
	if (content_size % 4)
	{   
		content_size += (4 - (content_size % 4));
	}
#endif
	temp_content_size  = content_size;
	num_flash = content_size / FLASH_WRITE_CHUNK_SIZE;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("content_size:%d\n"),
				content_size));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("num_flash: %d\n"),num_flash));

	for (kk = 0; kk <= num_flash; kk++)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("flash_indx: %d\n"),kk));
		if (kk != num_flash)
		{	
			content_size = FLASH_WRITE_CHUNK_SIZE;
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("QSPI content_size:%d\n"),
						content_size));
		}
		else
		{
			content_size = temp_content_size % FLASH_WRITE_CHUNK_SIZE;
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("QSPI LAST SECTOR BEING WRITTEN content_size:%d\n"),
						content_size));
			if (!content_size)
			{ 
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("INSTRUCTION SIZE ZEROOOOOOOOO: \n")));
				break;
			}  
		}	

		if(kk % 2)
			cmd = PING_WRITE;
		else
			cmd = PONG_WRITE;

		if(ONEBOX_STATUS_FAILURE == ping_pong_write(adapter, 
							    cmd,
							    flash_content, 
							    content_size)) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: Unable to load %d blk\n"), __func__,
					 kk));
			goto fail;
		}

		ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
				(TEXT("%s: Succesfully loaded %d instructions\n"), __func__,
				 kk));
		flash_content += content_size; 
	}
	if(bl_cmd(adapter, EOF_REACHED, FW_LOADING_SUCCESSFUL, "EOF_REACHED") < 0) {
		bl_cmd_stop_timer(adapter);
		goto fail;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("FW loading is done and FW is running..\n"));
	return ONEBOX_STATUS_SUCCESS;	
fail:
	return ONEBOX_STATUS_FAILURE;
}

ONEBOX_STATUS auto_load_flash_content(PONEBOX_ADAPTER adapter, METADATA metadata_rcv )
{
	uint8 *flash_content;
	uint32 instructions_sz;
	uint32 temp_instructions_sz;
	uint8 *flash_content_temp;
	uint32 flash_indx =0;

	uint32 flash_start_address;

	uint32 status = ONEBOX_STATUS_SUCCESS;

	/* Loading in non-multi block mode */
	flash_content = (uint8 *)adapter->os_intf_ops->onebox_get_firmware(metadata_rcv.name,
			(uint32 *)&instructions_sz,
			BIN_FILE,(uint8 *)adapter->firmware_path);
	if (flash_content == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
					("%s: Failed to open file %s\n"), __func__,
					metadata_rcv.name));
		return ONEBOX_STATUS_FAILURE;
	}
	flash_content_temp = flash_content;

	if (!(adapter->device_model == RSI_DEV_9116)) {   /* SECTOR MODE */
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_ERROR, &flash_content[FLASHING_START_ADDRESS], 10);

		if(instructions_sz > MAX_FLASH_FILE_SIZE) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Content size is more than 400K i.e %u and so, not flashing\n",MAX_FLASH_FILE_SIZE));
			goto fail;
		}

		flash_start_address = ONEBOX_CPU_TO_LE32(*(uint32 *)&flash_content[FLASHING_START_ADDRESS]);
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("flash start address is %08x\n",flash_start_address));

		if(flash_start_address <= FLASH_SECTOR_SIZE) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Start Address is less than 4K.. So not flashing\n"));
			goto fail;
		}

		if(flash_start_address % FLASH_SECTOR_SIZE) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Start Address is not multiple of 4K.. So not flashing\n"));
			goto fail;
		}

		if(flash_start_address + instructions_sz > FLASH_MAX_ADDRESS) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Flash Content will cross max flash size of 4MB.. So not flashing\n"));
			goto fail;
		}

		flash_indx = flash_start_address;
	}
	else {
		flash_indx = STARTING_BLOCK_INDEX * FLASH_BLOCK_SIZE;
	}

	temp_instructions_sz = instructions_sz ;
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("instructions_sz:%d\n"),
				instructions_sz));

#ifdef FLASHING_TEST_MODE
	adapter->os_intf_ops->onebox_memset(flash_content, 0xFF, instructions_sz);
#endif		
	status = trigger_auto_flash_burn(adapter, flash_content, instructions_sz, flash_indx);
	if(status == ONEBOX_STATUS_FAILURE) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("FLASHING Firmware failed in FLASHING phase\n"));
		goto fail;
	}
	if (!(adapter->device_model == RSI_DEV_9116)) {
	status = verify_flash_content(adapter, flash_content, 
			instructions_sz, flash_indx, EEPROM_READ_MODE);
	if(status == ONEBOX_STATUS_FAILURE) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("FLASHING Firmware failed in FLASH VERIFICATION phase\n"));
		goto fail;
	}
	}
	adapter->os_intf_ops->onebox_vmem_free(flash_content_temp);
	return ONEBOX_STATUS_SUCCESS;	
fail:	
	adapter->os_intf_ops->onebox_vmem_free(flash_content_temp);
	return ONEBOX_STATUS_FAILURE;
}

/* This function sends bootup parameters frame to TA.
 * @param pointer to driver private structure
 * @return 0 if success else -1. 
 */   
uint8 onebox_qspi_bootup_params(PONEBOX_ADAPTER adapter)
{         
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

	FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
	             (TEXT("===> Sending qspi Bootup parameters Packet <===\n")));

	/* Allocating Memory For Mgmt Pkt From Mgmt Free Pool */
	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

	adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, 256);
	if (adapter->operating_chwidth == BW_40Mhz)
	{  
		if (adapter->device_model == RSI_DEV_9116) {
      		adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bootup_params_9116,
				                            &boot_params_9116_40,
				                            sizeof(BOOTUP_PARAMETERS_9116));
    		} else {
			adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bootup_params,
				                            &boot_params_40,
				                            sizeof(BOOTUP_PARAMETERS));
		}
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
				(TEXT("===> Sending qspi Bootup parameters Packet 40MHZ <=== %d\n"),UMAC_CLK_40BW));
		mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(UMAC_CLK_40BW);
	}
	else
	{
    		if (adapter->device_model == RSI_DEV_9116)
		  adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bootup_params_9116,
				                            &boot_params_9116_20,
				                            sizeof(BOOTUP_PARAMETERS_9116));
    		else
		  adapter->os_intf_ops->onebox_memcpy(&mgmt_frame->u.bootup_params,
				                            &boot_params_20,
				                            sizeof(BOOTUP_PARAMETERS));
    		if (adapter->device_model == RSI_DEV_9116) {
			if (boot_params_9116_20.valid != VALID_20)
			{
				mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(UMAC_CLK_20BW);
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("===> Sending qspi Bootup parameters Packet 20MHZ <=== %d \n"),UMAC_CLK_20BW));
			}
		}
		else if(boot_params_9116_20.valid != VALID_20)
		{
			mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(UMAC_CLK_20BW);
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("===> Sending qspi Bootup parameters Packet 20MHZ <=== %d \n"),UMAC_CLK_20BW));
    		}
		else
		{
			mgmt_frame->desc_word[7] = ONEBOX_CPU_TO_LE16(UMAC_CLK_40MHZ);
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
					(TEXT("===> Sending qspi Bootup parameters Packet 20MHZ <=== %d \n"),UMAC_CLK_40MHZ));
		}	
	}  
	/* Bit{0:11} indicates length of the Packet
	 * Bit{12:15} indicates host queue number
	 */
	 
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(BOOTUP_PARAMS_REQUEST);

	if (adapter->device_model == RSI_DEV_9116) { 
		mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(sizeof(BOOTUP_PARAMETERS_9116) | (ONEBOX_WIFI_MGMT_Q << 12));
		status = onebox_coex_mgmt_frame(adapter, (uint16 *)mgmt_frame, (sizeof(BOOTUP_PARAMETERS_9116)+ FRAME_DESC_SZ));
	}
	else {
		mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(sizeof(BOOTUP_PARAMETERS) | (ONEBOX_WIFI_MGMT_Q << 12));
		status = onebox_coex_mgmt_frame(adapter, (uint16 *)mgmt_frame, (sizeof(BOOTUP_PARAMETERS)+ FRAME_DESC_SZ));
	}

	return status;
} /*end of onebox_qspi_bootup_params */
