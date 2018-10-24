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

/*** Do not Change the order ****/
static METADATA metadata[] = {{"pmemdata_dummy", 0x00000000},
                       {"pmemdata", 0x00000000}, 
                       {"pmemdata_wlan_bt_classic", 0x00000000}, 
                       {"pmemdata_wlan_zigb", 0x00000000},
                       {"pmemdata_wlan_bt_classic", 0x00000000} 
};

static METADATA metadata_lpbk_9116[]    = {{"pmemdata_wlan_bt_classic", 0x00000000}}; 
static METADATA metadata_lpbk[]    = {{"pmemdata", 0x00000000}}; 

static METADATA metadata_burn[]    = {{"pmemdata_burn_9116", 0x00000000}};

static METADATA metadata_burn_data[]    = {{"pmemdata_burn", 0x00000000}};


static METADATA metadata_sniffer[] = {{"pmemdata", 0x00000000}};
/* XXX: metadata_swbl_with_mbr is not being used right now */
METADATA metadata_swbl_with_mbr[]    = {{"RS9113_WC_BL_with_mbr.rps", 0x00010000}};
static METADATA metadata_swbl_without_mbr[]    = {{"RS9113_WC_BL.rps", 0x00010000}};
/*** FLASH Firmware****/
METADATA metadata_flash_content[] = {
                                 {"flash_content_dummy", 0x00010000}, 
                                 {"RS9113_WLAN_QSPI.rps", 0x00010000}, 
                                 {"RS9113_WLAN_BT_DUAL_MODE.rps", 0x00010000}, 
                                 {"RS9113_WLAN_ZIGBEE.rps", 0x00010000},
                                 {"RS9113_AP_BT_DUAL_MODE.rps", 0x00010000}, 
                                 {"RS9113_WLAN_QSPI.rps", 0x00010000}, 
                                 {"RS9113_ZIGBEE_COORDINATOR.rps", 0x00010000},
                                 {"RS9113_ZIGBEE_ROUTER.rps", 0x00010000} 
};

/** The below array is used for RF EVALUATION MODE binaries.
 */
METADATA metadata_per_flash_content[] = {
                                 {"flash_content_dummy", 0x00010000}, 
                                 {"RS9113_WLAN_QSPI.rps", 0x00010000}, 
                                 {"RS9113_WLAN_BT_DUAL_MODE_PER.rps", 0x00010000}, 
                                 {"RS9113_WLAN_ZIGBEE.rps", 0x00010000},
                                 {"RS9113_WLAN_BT_DUAL_MODE_PER.rps", 0x00010000} 
};

static ONEBOX_STATUS load_fw(PONEBOX_ADAPTER adapter, METADATA metadata_rcv )
{
	uint8 *ta_firmware;
	uint8 *firmware_ptr;
	uint32 instructions_sz;
	uint32 block_size = adapter->TransmitBlockSize;
	uint32 base_address;
	ONEBOX_STATUS status =0;
	uint16 ii;
	/*take structure with boot descriptor*/
	bootload_ds_t bootload_ds;


	/* Loading in non-multi block mode */
	ta_firmware = (uint8 *)adapter->os_intf_ops->onebox_get_firmware(metadata_rcv.name,
			(uint32 *)&instructions_sz,
			BIN_FILE,(uint8 *)adapter->firmware_path);
	firmware_ptr = ta_firmware;
	if (ta_firmware == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
					("%s: Failed to open file %s\n"), __func__,
					metadata_rcv.name));
		return ONEBOX_STATUS_FAILURE;
	}

	if(!strncmp(metadata_rcv.name, "pmemdata", strlen("pmemdata")))
	{
#if 0
		adapter->lmac_ver.major	= ((ta_firmware[LMAC_VER_OFFSET]) & 0xFF);
		adapter->lmac_ver.minor = ((ta_firmware[LMAC_VER_OFFSET+1] ) & 0xFF);
		adapter->lmac_ver.release_num = ((ta_firmware[LMAC_VER_OFFSET+2] ) & 0xFF);
		adapter->lmac_ver.patch_num = ((ta_firmware[LMAC_VER_OFFSET+3] ) & 0xFF);
		adapter->lmac_ver.ver.info.fw_ver[0] = ((ta_firmware[LMAC_VER_OFFSET+4] ) & 0xFF);
#else
		adapter->lmac_ver.major = ((ta_firmware[LMAC_VER_OFFSET_RS9116]) & 0xFF);
		adapter->lmac_ver.minor = ((ta_firmware[LMAC_VER_OFFSET_RS9116+1] ) & 0xFF);
		adapter->lmac_ver.release_num = ((ta_firmware[LMAC_VER_OFFSET_RS9116+2] ) & 0xFF);
		adapter->lmac_ver.patch_num = ((ta_firmware[LMAC_VER_OFFSET_RS9116+3] ) & 0xFF);
		adapter->lmac_ver.ver.info.fw_ver[0] = ((ta_firmware[LMAC_VER_OFFSET_RS9116+4] ) & 0xFF);
#endif
	}

	if (instructions_sz % 4)
	{   
		instructions_sz += (4 - (instructions_sz % 4));
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("instructions_sz:%d\n"),
				instructions_sz));
  
	base_address = metadata_rcv.address;

	if ((adapter->fw_load_mode == FW_LOAD_WITH_DESC) &&
		ONEBOX_CPU_TO_LE16((*(uint16 *)firmware_ptr) == 0x5aa5)) 
	{
		/*memcpy to boot descriptor from TA firmware len should be assign*/
		adapter->os_intf_ops->onebox_memcpy(&bootload_ds, (uint8_t *)firmware_ptr,sizeof(bootload_ds_t));
		/* move ta firmware poiner to ta_fw+ len*/
		firmware_ptr = firmware_ptr + bootload_ds.offset;

		ii=0; 

		do {

			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: Loading chunk %d\n"), __func__, ii));

			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("length %d: destination %x\n"), bootload_ds.bl_entry[ii].control.len, bootload_ds.bl_entry[ii].dst_addr));


			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("FW start %x\n"), *(uint32 *)firmware_ptr));

			if (adapter->host_intf_type == HOST_INTF_USB) { 
				status = usb_load_data_master_write(adapter, bootload_ds.bl_entry[ii].dst_addr, bootload_ds.bl_entry[ii].control.len, block_size, firmware_ptr);
			} else {
				status = sdio_load_data_master_write(adapter, bootload_ds.bl_entry[ii].dst_addr, bootload_ds.bl_entry[ii].control.len, block_size, firmware_ptr);
			}

			firmware_ptr += bootload_ds.bl_entry[ii].control.len;
			if (bootload_ds.bl_entry[ii].control.last_entry) {
				break;
			}
			ii++;
		} while(1);
	}  
	else 
	{
		if (adapter->host_intf_type == HOST_INTF_USB) 
			status = usb_load_data_master_write(adapter, base_address, instructions_sz, block_size, ta_firmware);
		else
			status = sdio_load_data_master_write(adapter, base_address, instructions_sz, block_size, ta_firmware);
	}
	if(status) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to load %s blk\n"), __func__,
				 metadata_rcv.name));
		adapter->os_intf_ops->onebox_vmem_free(ta_firmware);
		return ONEBOX_STATUS_FAILURE;
	}


	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
			(TEXT("%s: Succesfully loaded %s instructions\n"), __func__,
			 metadata_rcv.name));
	adapter->os_intf_ops->onebox_vmem_free(ta_firmware);
	return ONEBOX_STATUS_SUCCESS;	
}
int  bl_cmd(PONEBOX_ADAPTER adapter, uint8 cmd, uint8 exp_resp, char *str)
{
	uint16 regout_val = 0;
	uint32 timeout = 0;

	if((cmd == EOF_REACHED) || (cmd == PING_VALID) || (cmd == PONG_VALID)) {
		timeout = BL_BURN_TIMEOUT;
	} else {
		timeout = BL_CMD_TIMEOUT;
	}
	bl_cmd_start_timer(adapter, timeout);
	if(bl_write_cmd(adapter, cmd, exp_resp, &regout_val) < 0) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d %s Command %0x writing failed..response : %d \n",__func__,
				__LINE__, str, cmd,regout_val));
		goto fail;
	}
	bl_cmd_stop_timer(adapter);
	return ONEBOX_STATUS_SUCCESS;
fail:
	return ONEBOX_STATUS_FAILURE;
}

void bl_cmd_start_timer(PONEBOX_ADAPTER adapter, uint32 timeout)
{
	adapter->bl_timer_expired = 0;
	adapter->os_intf_ops->onebox_init_sw_timer(&adapter->bl_timer, 
			(unsigned long)adapter,
			(void *)&bl_cmd_timer_expired,
			msecs_to_jiffies(timeout));
	return;
}

void bl_cmd_stop_timer(PONEBOX_ADAPTER adapter)
{
	adapter->bl_timer_expired = 0;
	if(adapter->os_intf_ops->onebox_sw_timer_pending(&adapter->bl_timer)) {
		adapter->os_intf_ops->onebox_remove_timer(&adapter->bl_timer);
	}
	return;
}

void bl_cmd_timer_expired(PONEBOX_ADAPTER adapter) 
{
	adapter->bl_timer_expired = 1;
}

int bl_write_cmd(PONEBOX_ADAPTER adapter, uint8 cmd, uint8 exp_resp, uint16 *cmd_resp)
{
	uint32 regin_val = 0, regout_val = 0;
	uint8 output = 0;
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("waiting for Regin to be invalid\n"));
	while(!adapter->bl_timer_expired) {
		regin_val = 0;
		if(adapter->osd_host_intf_ops->onebox_master_reg_read(adapter, 
					SWBL_REGIN, &regin_val, 2) < 0) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d Command %0x REGIN reading failed..\n",__func__,
					__LINE__,cmd));
			goto fail;
		}
		adapter->os_intf_ops->onebox_msec_delay(1);	
		if((regin_val >> 12) != REGIN_VALID) {
			break;
		}
	}
	if(adapter->bl_timer_expired) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d Command %0x REGIN reading timed out..\n",__func__,
				__LINE__,cmd));
		goto fail;
	}

	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Issuing write into Regin regin_val:%0x sending cmd :%0x\n",regin_val, (cmd | REGIN_INPUT << 8)));
	if((adapter->osd_host_intf_ops->onebox_master_reg_write(adapter, 
					SWBL_REGIN, (cmd | REGIN_INPUT << 8), 2)) < 0) {
		goto fail;
	}
	adapter->os_intf_ops->onebox_msec_delay(1);	

	if(cmd == LOAD_HOSTED_FW || cmd == JUMP_TO_ZERO_PC) {
		/* JUMP_TO_ZERO_PC doesn't expect 
		 * any response. So return from here*/
		return ONEBOX_STATUS_SUCCESS;
	}

	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("waiting for regout to become valid\n"));
	while(!adapter->bl_timer_expired) {
		regout_val = 0;
		if(adapter->osd_host_intf_ops->onebox_master_reg_read(adapter, 
					SWBL_REGOUT, &regout_val, 2) < 0) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d Command %0x REGOUT reading failed..\n",__func__,
					__LINE__,cmd));
			goto fail;
		}
		adapter->os_intf_ops->onebox_msec_delay(1);	
		if((regout_val >> 8) == REGOUT_VALID) {
			break;
		}
	}
	if(adapter->bl_timer_expired) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d Command %0x REGOUT reading timed out..\n",__func__,
				__LINE__,cmd));
		goto fail;
	}

	*cmd_resp = regout_val;

	output = regout_val & 0xff;

	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Invalidating regout\n"));
	if((adapter->osd_host_intf_ops->onebox_master_reg_write(adapter, 
					SWBL_REGOUT, (cmd | REGOUT_INVALID << 8), 2)) < 0) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d Command %0x REGOUT writing failed..\n",__func__,
				__LINE__,cmd));
		goto fail;
	}
	adapter->os_intf_ops->onebox_msec_delay(1);	
	if(output == exp_resp) {
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, 
			("%s:received expected response 0x%0X for cmd 0x%0X\n",
			 __func__, output, cmd));
	} else {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR,
			("%s:received response 0x%X for cmd 0x%0X\n",
			 __func__ ,output , cmd));
		goto fail;
	}
	return ONEBOX_STATUS_SUCCESS;

fail:
	return ONEBOX_STATUS_FAILURE;
}/* End <bl_write_cmd> */

ONEBOX_STATUS ping_pong_write(PONEBOX_ADAPTER adapter, uint8 cmd, uint8 *addr, uint32 size)
{
	uint32 block_size = adapter->TransmitBlockSize;
	uint32 cmd_addr;
	uint16 cmd_resp = 0, cmd_req = 0;
	uint8 *str;
	ONEBOX_STATUS status = 0;

	if(cmd == PING_WRITE) {
		cmd_addr = PING_BUFFER_ADDRESS;
		cmd_resp = PONG_AVAIL;
		cmd_req = PING_VALID;
		str = "PING_VALID";
	} else {
		cmd_addr = PONG_BUFFER_ADDRESS;
		cmd_resp = PING_AVAIL;
		cmd_req = PONG_VALID;
		str = "PONG_VALID";
	}
	
	if (adapter->host_intf_type == HOST_INTF_USB) 
		status = usb_load_data_master_write(adapter, cmd_addr, size, block_size, addr);
	else
		status = sdio_load_data_master_write(adapter, cmd_addr, size, block_size, addr);

	if(status) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to write blk at addr %0x\n"), __func__,
				*addr));
		goto fail;
	}
	if(bl_cmd(adapter, cmd_req, cmd_resp, str) < 0) {
		bl_cmd_stop_timer(adapter);
		goto fail;
	}
	return ONEBOX_STATUS_SUCCESS;
fail:
	return ONEBOX_STATUS_FAILURE;
}

/**
 * This function writes the bootloader header required for fw loading. 
 *
 * @param  Pointer to the driver adapter structure. 
 * @return ONEBOX_STATUS_SUCCESS on success and ONEBOX_STATUS_FAILURE on failure. 
 */
static ONEBOX_STATUS write_bl_header(PONEBOX_ADAPTER adapter, uint8 *flash_content, uint32 content_size)
{
	struct bl_header bl_hdr;

#define CHECK_SUM_OFFSET 20
#define LEN_OFFSET 8
#define ADDR_OFFSET 16

	bl_hdr.flags = 0; /*Loading Complete header */
	bl_hdr.image_no = (*(uint32 *)&adapter->coex_mode); /*Image number */ 		
	bl_hdr.check_sum = (*(uint32 *)&flash_content[CHECK_SUM_OFFSET]);
	bl_hdr.flash_start_address = (*(uint32 *)&flash_content[ADDR_OFFSET]);
	bl_hdr.flash_len = (*(uint32 *)&flash_content[LEN_OFFSET]);

	if(adapter->host_intf_type == HOST_INTF_USB) {
		if (adapter->osd_host_intf_ops->onebox_ta_write_multiple(adapter,
					PING_BUFFER_ADDRESS,
					(uint8*)&bl_hdr,
					sizeof(struct bl_header)	
					)!=ONEBOX_STATUS_SUCCESS) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d Failed to load Version/CRC structure\n",__func__,__LINE__));
			goto fail;
		}
	} else { 
		if (adapter->osi_host_intf_ops->onebox_master_access_msword(adapter, (PING_BUFFER_ADDRESS >> 16))
				!= ONEBOX_STATUS_SUCCESS)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: Unable to set ms word to common reg\n"), __func__));
			return ONEBOX_STATUS_FAILURE;              
		}
		if (adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
					SD_REQUEST_MASTER | 
					(PING_BUFFER_ADDRESS & 0xFFFF),
					(uint8*)&bl_hdr,
					sizeof(struct bl_header), NULL	
					)!=ONEBOX_STATUS_SUCCESS)
		{
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d Failed to load Version/CRC structure\n",__func__,__LINE__));
			goto fail;
		}

	}
	return ONEBOX_STATUS_SUCCESS;

fail:
	return ONEBOX_STATUS_FAILURE;
}

/**
 * This function includes the actual funtionality of loading the TA firmware. 
 *
 * Loading the TA firmware includes opening,reading the TA file and writing
 * in a blocks of data.
 *
 * @param  Pointer to the driver adapter structure. 
 * @return ONEBOX_STATUS_SUCCESS on success and ONEBOX_STATUS_FAILURE on failure. 
 */
ONEBOX_STATUS load_ta_instructions(PONEBOX_ADAPTER adapter)
{
	ONEBOX_STATUS status;
	METADATA *metadata_p; 
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
	             (TEXT("%s: LOAD TA Instructions\n"), __func__));

#if 1
  if (adapter->device_model == RSI_DEV_9116) {
    if((adapter->osd_host_intf_ops->onebox_master_reg_write(adapter, MEM_ACCESS_CTRL_FROM_HOST, RAM_384K_ACCESS_FROM_TA , 4)) < 0) {
      goto fail;
    }
  }
#endif    

  if(adapter->Driver_Mode == QSPI_FLASHING) {
		if (adapter->device_model == RSI_DEV_9116) {
			metadata_p = metadata_burn;
		}
		else {
			metadata_p = metadata_burn_data;
		}
	}
	else if(adapter->Driver_Mode == RF_EVAL_MODE_ON) {
		metadata_p = metadata_lpbk;
    if ((adapter->device_model == RSI_DEV_9116) && (adapter->coex_mode > WIFI_ALONE)) {
			metadata_p = metadata_lpbk_9116;
		}
		//metadata_p = &metadata[WIFI_ALONE]; /*WLAN PER mode*/
	}
	else if(adapter->Driver_Mode == RF_EVAL_LPBK_CALIB ||
			adapter->Driver_Mode == RF_EVAL_LPBK) {
		metadata_p = metadata_lpbk;
	}
	else if(adapter->Driver_Mode == SNIFFER_MODE) {
		metadata_p = metadata_sniffer;
	}
	else {
		metadata_p = &metadata[adapter->coex_mode];
	}

	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("**DRIVER MODE is %d COEX MODE is %d and loading file %s**\n", adapter->Driver_Mode, 
			adapter->coex_mode,
			metadata_p->name));
	status = load_fw(adapter, *metadata_p);
	if(status == ONEBOX_STATUS_FAILURE) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d Unable to load pmemdata...\n",__func__,__LINE__));
		goto fail;
	}

	if((adapter->fw_load_mode == FLASH_RAM_NO_SBL) ||
      (adapter->fw_load_mode == FW_LOAD_WITH_DESC)) {
		if (adapter->host_intf_type == HOST_INTF_SDIO)
			status = sdio_ta_reset_ops(adapter);

		if(status == ONEBOX_STATUS_FAILURE) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d Unable to put ta in reset\n",__func__,__LINE__));
			goto fail;
		}
	}
	return ONEBOX_STATUS_SUCCESS;
fail:
	return ONEBOX_STATUS_FAILURE;
}/* End <onebox_load_ta_instructions> */

/**
 * This function includes the firmware loading through secondary bootloader. 
 *
 * Loading the TA firmware includes verifying CRC, upgrade fw if CRC fails and issue
 * bootloader commands to load fw
 *
 * @param  Pointer to the driver adapter structure. 
 * @return ONEBOX_STATUS_SUCCESS on success and ONEBOX_STATUS_FAILURE on failure. 
 */
ONEBOX_STATUS load_fw_thru_sbl(PONEBOX_ADAPTER adapter)
{
	uint16 regout_val = 0;
	uint8 *flash_content = NULL;
	uint32 content_size = 0, status;
	METADATA *metadata_p; 
	uint8 zigb_mode = 0;
	if (!(adapter->device_model == RSI_DEV_9116)) {
		if(bl_cmd(adapter, CONFIG_AUTO_READ_MODE, CMD_PASS, "AUTO_READ_CMD") < 0) {
			goto fail;
		}
		adapter->flash_capacity = read_flash_capacity(adapter);
		if(adapter->flash_capacity <= 0) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Unable to read flash size from EEPROM\n"));
			goto fail;
		}
	}

	if(adapter->Driver_Mode == QSPI_FLASHING &&
			adapter->flashing_mode == SWBL_FLASHING_SBL) {
		goto BURN_BOOTLOADER;
	} else if(adapter->fw_load_mode == FULL_FLASH_SBL ||
			adapter->fw_load_mode == FLASH_RAM_SBL) {
		goto RPS_IMAGE_HEADER;
	} else if(adapter->fw_load_mode == FULL_RAM_SBL){
		goto POLLING_MODE_CMD;
	} else {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Invalid fw load mode configured %d\n",adapter->fw_load_mode));
		goto fail;
	}

RPS_IMAGE_HEADER:
	/*RPS Header(32 bytes) contains the following Information 
	 * The image no, checksum and flash start address and flash len is present in the RPS header 
	 * This information is used for verifying the CRC checks.
	 */

	if((adapter->Driver_Mode == RF_EVAL_MODE_ON) || (adapter->Driver_Mode == RF_EVAL_LPBK_CALIB)) {
		metadata_p = &metadata_per_flash_content[adapter->coex_mode];
	} else if(adapter->Driver_Mode == 7){
		metadata_p = &metadata_flash_content[5];
	} else if(adapter->d_assets->coex_mode == WIFI_ZIGBEE) {
		zigb_mode = (adapter->d_assets->oper_mode >> 4) & ZIGBEE_OPERMODE_MASK;
		switch (zigb_mode)
		{
			case ZIGBEE_END_DEVICE:
				metadata_p = &metadata_flash_content[adapter->coex_mode];
				break;
			case ZIGBEE_COORDINATOR:
				metadata_p = &metadata_flash_content[6];
				/* Operating mode changed to Zigbee alone */
				adapter->d_assets->oper_mode = 16;
				break;
			case ZIGBEE_ROUTER:
				metadata_p = &metadata_flash_content[7];
				/* Operating mode changed to Zigbee alone */
				adapter->d_assets->oper_mode = 16;
				break;
			default:
				goto fail;
		}
 	} else {
		metadata_p = &metadata_flash_content[adapter->coex_mode];
	}

	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("**DRIVER MODE is %d COEX MODE is %d and loading file %s**\n", adapter->Driver_Mode, 
			adapter->coex_mode,
			metadata_p->name));
	flash_content = (uint8 *)adapter->os_intf_ops->onebox_get_firmware(metadata_p->name,
			(uint32 *)&content_size,
			BIN_FILE, (uint8 *)adapter->firmware_path);
	if (flash_content == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
					("%s: Failed to open file %s\n"), __func__,
					metadata_p->name));
		goto fail;
	}

	if(!strncmp(metadata_p->name, "RS9113", strlen("RS9113")))
	{
		adapter->lmac_ver.ver.info.fw_ver[0] = ((flash_content[LMAC_VER_OFFSET] ) & 0xFF);
		adapter->lmac_ver.ver.info.fw_ver[1] = ((flash_content[LMAC_VER_OFFSET+1] ) & 0xFF);
		adapter->lmac_ver.major	= ((flash_content[LMAC_VER_OFFSET + 2]) & 0xFF);
		adapter->lmac_ver.release_num = ((flash_content[LMAC_VER_OFFSET + 3]) & 0xFF);
		adapter->lmac_ver.minor = ((flash_content[LMAC_VER_OFFSET + 4] ) & 0xFF);
		adapter->lmac_ver.patch_num = 0;

	}

	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, &flash_content[0], 16);
	if (!(adapter->device_model == RSI_DEV_9116)) {
		status = write_bl_header(adapter, flash_content, content_size);
		if(status == ONEBOX_STATUS_SUCCESS) {
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("RPS Image Header Written Successfully\n"));;
			goto CHECK_CRC_CMD;
		} else {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("RPS Image header loading failed\n"));
			goto fail;
		}
	}

CHECK_CRC_CMD:
	bl_cmd_start_timer(adapter, BL_CMD_TIMEOUT);
	if(bl_write_cmd(adapter, CHECK_CRC, CMD_PASS, &regout_val) < 0) {
		bl_cmd_stop_timer(adapter);
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d CHECK_CRC Command writing failed..\n",__func__,__LINE__));
		if((regout_val & 0xff) != CMD_PASS) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Onebox:Proceeding to Upgrade Firmware\n"));
			goto FW_UPGRADE;
		}
	}
	bl_cmd_stop_timer(adapter);
	if(adapter->fw_load_mode == FLASH_RAM_SBL ||
			adapter->fw_load_mode == FULL_FLASH_SBL) {
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("CRC Pass..Issuing Polling Command\n"));
		goto POLLING_MODE_CMD;
	} else {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Entered wrong path in fw_load_mode :%d\n",adapter->fw_load_mode));
		goto fail;
	}

LOAD_IMAGE_CMD:
	if(bl_cmd(adapter, LOAD_HOSTED_FW, LOADING_INITIATED, "LOAD_HOSTED_FW") < 0) {
		goto fail;
	}
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Load Image command passed..\n"));
	goto success;

POLLING_MODE_CMD:
	if(bl_cmd(adapter, POLLING_MODE, CMD_PASS, "POLLING_MODE") < 0) {
		goto fail;
	}
	if(adapter->fw_load_mode == FULL_FLASH_SBL) {
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Polling mode command passed.. Issuing Load hosted fw cmd\n"));
		goto LOAD_IMAGE_CMD;
	} else {    
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Polling mode command passed.. Loading pmemdata\n"));
		goto LOAD_PMEMDATA;
	}

LOAD_PMEMDATA:
	status = load_ta_instructions(adapter);
	if(status == ONEBOX_STATUS_FAILURE) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d PMEMDATA Loading Failed\n",__func__,__LINE__));
		goto fail;
	} else {
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("pmemdata loading successful.. Issuing Jump to Zero command\n"));
		goto JUMP_TO_ZERO;
	}

JUMP_TO_ZERO:
	if(bl_cmd(adapter, JUMP_TO_ZERO_PC, CMD_PASS, "JUMP_TO_ZERO") < 0) {
		goto fail;
	}
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Jump to zero command successful\n"));
	goto success;

FW_UPGRADE:
	/* After burning the RPS header, firmware has to be burned using the below steps */
	if(bl_cmd(adapter, BURN_HOSTED_FW, SEND_RPS_FILE, "FW_UPGRADE") < 0) {
		goto fail;
	}
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Burn Command Pass.. Upgrading the firmware\n"));
	status = adapter->coex_osi_ops->onebox_auto_fw_upgrade(adapter, flash_content, content_size);
	if(status == ONEBOX_STATUS_SUCCESS) {
		goto LOAD_IMAGE_CMD;
	} else {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("****FIRMWARE UPGRADATION FAILED****\n"));
		goto AUTO_READ_MODE_CMD;
	}

BURN_BOOTLOADER:
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Reading calib data using Master Read\n"));
	adapter->calib_data = &adapter->DataRcvPacket[0];
	if(read_flash_content(adapter, adapter->calib_data, 
				SOC_FLASH_ADDR, EEPROM_DATA_SIZE) < 0) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Failed to read calib data using master read\n"));
		goto fail;
	}

	if(bl_cmd(adapter, BURN_BL, SEND_RPS_FILE, "BURN_BOOTLOADER") < 0) {
		goto fail;
	}

	metadata_p = metadata_swbl_without_mbr;
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("**DRIVER MODE is %d COEX MODE is %d and loading file %s**\n", adapter->Driver_Mode, 
			adapter->coex_mode,
			metadata_p->name));
	flash_content = (uint8 *)adapter->os_intf_ops->onebox_get_firmware(metadata_p->name,
			(uint32 *)&content_size,
			BIN_FILE,(uint8 *)adapter->firmware_path);
	if (flash_content == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
					("%s: Failed to open file %s\n"), __func__,
					metadata_p->name));
		goto fail;
	}
	adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, &flash_content[0], 16);
	status = adapter->coex_osi_ops->onebox_auto_fw_upgrade(adapter, flash_content, content_size);
	if(status == ONEBOX_STATUS_SUCCESS) {
		goto AUTO_READ_MODE_CMD;
	} else {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("****FIRMWARE UPGRADATION FAILED****\n"));
		goto fail;
	}

AUTO_READ_MODE_CMD:
	if(bl_cmd(adapter, CONFIG_AUTO_READ_MODE, CMD_PASS, "AUTO_READ_MODE") < 0) {
		goto fail;
	}
	if(adapter->Driver_Mode == QSPI_FLASHING &&
			adapter->flashing_mode == SWBL_FLASHING_SBL) {
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("%s:Starting Flash Verification Process\n",__func__));
		status = verify_flash_content(adapter, adapter->calib_data, 
				EEPROM_DATA_SIZE, 0, MASTER_READ_MODE);
		if(status == ONEBOX_STATUS_FAILURE) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:FLASHING SBL failed in Calib VERIFICATION phase\n",__func__));
			goto fail;
		}
		status = verify_flash_content(adapter, flash_content + BL_HEADER, 
				(content_size - BL_HEADER), EEPROM_DATA_SIZE, MASTER_READ_MODE);
		if(status == ONEBOX_STATUS_FAILURE) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:FLASHING SBL failed in SBL VERIFICATION phase\n",__func__));
			goto fail;
		}
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("%s:Flash Verification Process Completed Successfully\n",__func__));
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("SWBL FLASHING THROUGH SWBL PASSED...\n"));
		goto success;
	} else {
		goto fail;
	}

success:
	if (flash_content != NULL)
		adapter->os_intf_ops->onebox_vmem_free(flash_content);
	return ONEBOX_STATUS_SUCCESS;
fail:
	bl_cmd_stop_timer(adapter);
	if (flash_content != NULL)
		adapter->os_intf_ops->onebox_vmem_free(flash_content);
	return ONEBOX_STATUS_FAILURE;
}
