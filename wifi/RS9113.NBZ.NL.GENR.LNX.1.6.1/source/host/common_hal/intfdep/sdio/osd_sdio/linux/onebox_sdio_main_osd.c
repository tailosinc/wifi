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

ONEBOX_EXTERN int onebox_register_os_intf_operations(struct onebox_os_intf_operations *os_intf_ops);

#if KERNEL_VERSION_BTWN_2_6_(18, 22)
ONEBOX_EXTERN BOOLEAN __devinit
onebox_probe (PSDFUNCTION pfunction, PSDDEVICE pDevice);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
int onebox_probe (struct sdio_func *pfunction, const struct sdio_device_id *id);
#endif

#if KERNEL_VERSION_BTWN_2_6_(18, 22)
ONEBOX_EXTERN VOID __devexit onebox_disconnect (PSDFUNCTION pFunction, 
						PSDDEVICE  pDevice);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
VOID onebox_disconnect (struct sdio_func *pfunction);
#endif

ONEBOX_STATIC int onebox_suspend(struct device *dev);
ONEBOX_STATIC int onebox_resume(struct device *dev);

#if KERNEL_VERSION_BTWN_2_6_(18, 22)
ONEBOX_STATIC SD_PNP_INFO onebox_IdTable[] __devinitdata = 
{
	{
		.SDIO_ManufacturerID   = 0x101,
		.SDIO_ManufacturerCode = 0x041b,
		.SDIO_FunctionNo       = 1,
		.SDIO_FunctionClass    = 0
	},
	{
		.SDIO_ManufacturerID   = 0x201,
		.SDIO_ManufacturerCode = 0x041b,
		.SDIO_FunctionNo       = 1,
		.SDIO_FunctionClass    = 0
	},
	{
		.SDIO_ManufacturerID   = 0x301,
		.SDIO_ManufacturerCode = 0x041b,
		.SDIO_FunctionNo       = 1,
		.SDIO_FunctionClass    = 0
	},
	{
		.SDIO_ManufacturerID   = 0x100,
		.SDIO_ManufacturerCode = 0x0303,
		.SDIO_FunctionNo       = 1,
		.SDIO_FunctionClass    = 0
	},
	{}
};

ONEBOX_STATIC SDFUNCTION onebox_driver = 
{
	.pName       = "Onebox-SDIO WLAN",
	.Version     = CT_SDIO_STACK_VERSION_CODE,
	.MaxDevices  = 1,
	.NumDevices  = 0,
	.pIds        = onebox_IdTable,
	.pProbe      = (PVOID)onebox_probe,
	.pRemove     = onebox_disconnect,
	.pSuspend    = NULL,
	.pResume     = NULL,
	.pWake       = NULL,
	/* .Function.pContext    = &onebox_SDIO,  */
};

#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
ONEBOX_STATIC const struct sdio_device_id onebox_IdTable[] = 
{
	{ SDIO_DEVICE(0x303, 0x100) },
	{ SDIO_DEVICE(0x041B, 0x0301) },
	{ SDIO_DEVICE(0x041B, 0x0201) },
	{ SDIO_DEVICE(0x041B, 0x9330) },
	{ SDIO_DEVICE(0x041B, 0x9116) },
	{ /* Blank */},
};

ONEBOX_STATIC const struct dev_pm_ops onebox_pm_ops = {
	.suspend = onebox_suspend,
	.resume = onebox_resume,
};

ONEBOX_STATIC struct sdio_driver onebox_driver = 
{
	.name       = "Onebox-SDIO",
	.id_table   = onebox_IdTable,
	.probe      = onebox_probe,
	.remove     = onebox_disconnect,
#ifdef CONFIG_PM
	.drv = {
		.pm = &onebox_pm_ops,
	}
#endif
};
#endif

#define SDIO_SET_CMD52_ARG(arg,rw,func,raw,address,writedata) \
	(arg) = (((rw) & 1) << 31)           | \
(((func) & 0x7) << 28)       | \
(((raw) & 1) << 27)          | \
(1 << 26)                    | \
(((address) & 0x1FFFF) << 9) | \
(1 << 8)                     | \
((writedata) & 0xFF)

#define SDIO_SET_CMD52_READ_ARG(arg,func,address) \
	SDIO_SET_CMD52_ARG(arg,0,(func),0,address,0x00)
#define SDIO_SET_CMD52_WRITE_ARG(arg,func,address,value) \
	SDIO_SET_CMD52_ARG(arg,1,(func),0,address,value)

/**
 * This function issues cmd52 byte write onto the card.
 *
 * @param  Pointer to the mmc_card.  
 * @param  address to write.  
 * @param  data to write.  
 * @return the write status 
 */
ONEBOX_STATIC  int onebox_cmd52writebyte(struct mmc_card *card, 
					 unsigned int address,
					 unsigned char byte)
{
	struct mmc_command ioCmd;
	unsigned long      arg;

	memset(&ioCmd,0,sizeof(ioCmd));
	SDIO_SET_CMD52_WRITE_ARG(arg,0,address,byte);
	ioCmd.opcode = SD_IO_RW_DIRECT;
	ioCmd.arg = arg;
	ioCmd.flags = MMC_RSP_R5 | MMC_CMD_AC;

	return mmc_wait_for_cmd(card->host, &ioCmd, 0);
}

/**
 * This function issues cmd52 byte read onto the card.
 *
 * @param  Pointer to the mmc_card.  
 * @param  address to read from.  
 * @param  variable to store read value.  
 * @return the read status 
 */
ONEBOX_STATIC int onebox_cmd52readbyte(struct mmc_card *card,
				       unsigned int address,
				       unsigned char *byte)
{
	struct mmc_command ioCmd;
	unsigned long   arg;
	int err;

	memset(&ioCmd,0,sizeof(ioCmd));
	SDIO_SET_CMD52_READ_ARG(arg,0,address);
	ioCmd.opcode = SD_IO_RW_DIRECT;
	ioCmd.arg = arg;
	ioCmd.flags = MMC_RSP_R5 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(card->host, &ioCmd, 0);

	if ((!err) && (byte)) 
	{
		*byte =  ioCmd.resp[0] & 0xFF;
	}

	return err;
}

/**
 * This function issues sdio commands.
 *
 * @param  Pointer to the sdio_func.  
 * @param  opcode value.  
 * @param  arguments to pass.  
 * @param  flags.  
 * @param  pointer to store response.  
 * @return the command status
 */
ONEBOX_STATIC int onebox_issue_sdiocommand(struct sdio_func *func, uint32 opcode, uint32 arg, uint32 flags, uint32 *resp)
{
	struct mmc_command cmd;
	int err;
	struct mmc_host *host;

	host = func->card->host;

	memset(&cmd, 0, sizeof(struct mmc_command)); 
	cmd.opcode = opcode;
	cmd.arg = arg;
	cmd.flags = flags;
	err = mmc_wait_for_cmd(host, &cmd, 3);

	if ((!err) && (resp)) 
	{
		*resp = cmd.resp[0];
	}

	return err;
}

/**
 * This function is called upon the occurence of an interrupt from a hardware.
 *
 * @param  Pointer to sdio_func structure.  
 * @return VOID. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22) 
static VOID onebox_linux_handle_interrupt(PVOID pContext)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26) 
static VOID onebox_linux_handle_interrupt(struct sdio_func *function)
#endif
{
	struct onebox_os_intf_operations *os_intf_ops;
	PONEBOX_ADAPTER adapter;
#if KERNEL_VERSION_BTWN_2_6_(18, 22) 
	adapter = (PONEBOX_ADAPTER)pContext;
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26) 
	adapter = onebox_getcontext(function);
#endif
	os_intf_ops = onebox_get_os_intf_operations_from_origin();
#if KERNEL_VERSION_GREATER_THAN_2_6_(26)  
	adapter->in_sdio_litefi_irq = current;
#endif
	adapter->coex_osi_ops->onebox_handle_interrupt(adapter); 
#if KERNEL_VERSION_GREATER_THAN_2_6_(26)  
	adapter->in_sdio_litefi_irq = NULL;
#endif
	return;
} /* End <onebox_linux_handle_interrupt> */

/**
 *This function enables the SDcard interface.
 *
 * @param  Pointer to SDcard interface.  
 * @return On success SD_SUCCESS is returned or SD_ERROR_NODEV on error. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
ONEBOX_STATUS  __devinit onebox_enable_interface(PVOID ptr)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
ONEBOX_STATUS  onebox_enable_interface(PVOID ptr)
#endif
{

#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	PSDDEVICE pDevice = (PSDDEVICE)ptr;
	SDCONFIG_FUNC_ENABLE_DISABLE_DATA fData;
	fData.EnableFlags = SDCONFIG_ENABLE_FUNC;
	fData.TimeOut     = 500;
	return SDLIB_IssueConfig(pDevice,
			SDCONFIG_FUNC_ENABLE_DISABLE,
			&fData,
			sizeof(fData));
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	int32 ret;
	struct sdio_func *pfunction = (struct sdio_func *)ptr; 

	/* Wait for 100 ms timeout to enable the function */
	pfunction->enable_timeout = 100;
	ret = sdio_enable_func(pfunction);
	return ret;
#endif
}/* End <onebox_enable_interface> */ 

/*
 * Data Alignment -
 * Address has to be 4bit aligned. In some older kernels
 * un aligned address may lead to read failures or undefined behaviour
 */
#define align_address(a) ((unsigned long)(a) & ~0x7)

ONEBOX_STATUS sdio_master_reg_write(PONEBOX_ADAPTER adapter, unsigned long addr, unsigned long data, uint16 size)
{
	unsigned long data1[2];
	unsigned long *data_alligned;

	data_alligned = (unsigned long *)align_address(&data1[1]);

	if(size == 2) {
		*data_alligned  = ((data << 16) |(data & 0xFFFF));
	}
	else if(size == 1) {
		uint32 temp_data;
		temp_data = (data & 0xFF);
		*data_alligned = ((temp_data << 24) | (temp_data << 16) | (temp_data << 8) | (temp_data));
	}
	else {
		*data_alligned = data;
	}

	size = 4;
	if(1) {
		if (adapter->osi_host_intf_ops->onebox_master_access_msword(adapter, (addr >> 16))
				!= ONEBOX_STATUS_SUCCESS)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: Unable to set ms word to common reg\n"), __func__));
			return ONEBOX_STATUS_FAILURE;              
		}
		addr = addr & 0xFFFF;
	}
	/* Bringing TA out of reset */
	if(adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
				addr |
				SD_REQUEST_MASTER,
				(uint8 *)data_alligned,
				//(uint8 *)&data,
				size,NULL
				)!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to do AHB reg write\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}
	return ONEBOX_STATUS_SUCCESS;
}

ONEBOX_STATUS sdio_master_reg_read(PONEBOX_ADAPTER adapter, uint32 addr, uint32 *rd_buf, uint16 size)
{
	uint32 *data = NULL;
	uint16 ms_addr = 0;
	uint32 align[2] = {};
	uint32 addr_on_bus;

	data = (uint32 *)align_address(&align[1]);
	//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, (" In %s %d  data =%p &align[0]= %p &align[1]=%p\n", __func__, __LINE__, data, &align[0], &align[1]));
	if (1) {
		ms_addr = (addr >> 16);
		if (adapter->osi_host_intf_ops->onebox_master_access_msword(adapter, ms_addr)
				!= ONEBOX_STATUS_SUCCESS)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: Unable to set ms word to common reg\n"), __func__));
			return ONEBOX_STATUS_FAILURE;              
		}
		addr = addr & 0xFFFF;
		//ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, (" In %s %d  setting msword =%x\n", __func__, __LINE__, ms_addr));

	}
	//	ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, (" In %s %d  addr =%x\n", __func__, __LINE__,addr));

	addr_on_bus = (addr & 0xFF000000);
	if ((addr_on_bus == (FLASH_SIZE_ADDR & 0xFF000000)) || /* This is for flash access*/
			(addr_on_bus == 0x0)) {/* This check is for Internal RAM access */
		addr_on_bus = (addr & ~(0x3));
	}
	else {
		/* This is for accessing peripherals on APB(AMBA- Peripheral Bus) bus of the device */
		addr_on_bus = addr;	
	}
	/* Bringing TA out of reset */
	if(adapter->osd_host_intf_ops->onebox_read_multiple(adapter,
				addr_on_bus |
				SD_REQUEST_MASTER,
				4,
				(uint8 *)data
				)!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: AHB register read failed\n"), __func__));
		return ONEBOX_STATUS_FAILURE;              

	}

	if(size == 2) {
		if((addr & 0x3) == 0) {
			*rd_buf = *data;
		}
		else {
			/*((addr & 0x3) == 2) */
			*rd_buf  = ((*data >> 16));
		}
		*rd_buf = (*rd_buf & 0xFFFF);
	}
	else if(size == 1) {
		if((addr & 0x3) == 0) {
			*rd_buf = *data;
		} else if((addr & 0x3) == 1) {
			*rd_buf = (*data >> 8);
		} else if((addr & 0x3) == 2) {
			*rd_buf = (*data >> 16);
		} else {
			*rd_buf = (*data >> 24);
		}
		*rd_buf = (*rd_buf & 0xFF);
	}
	else { /*size is 4 */
		*rd_buf = *data;
	}

	//	ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("In %s %d after doing read_multiple  *rd_data= %x, *data=%x\n", __func__, __LINE__, *rd_buf, *data));
	return ONEBOX_STATUS_SUCCESS;
}

void onebox_gspi_init(PONEBOX_ADAPTER adapter)
{
	uint32 gspi_ctrl_reg0_val;
	//! RF control reg 
	//! clk_ratio [3:0] 
	/* Programming gspi frequency = soc_frequency / 2 */
	/* Warning : ULP seemed to be not working
	 * well at high frequencies. Modify accordingly */
	gspi_ctrl_reg0_val = 0x4;
	//! csb_setup_time [5:4] 
	gspi_ctrl_reg0_val |= 0x10; 
	//! csb_hold_time [7:6] 
	gspi_ctrl_reg0_val |= 0x40; 
	//! csb_high_time [9:8] 
	gspi_ctrl_reg0_val |= 0x100; 
	//! spi_mode [10] 
	gspi_ctrl_reg0_val |= 0x000; 
	//! clock_phase [11] 
	gspi_ctrl_reg0_val |= 0x000; 
	/* Initializing GSPI for ULP read/writes */
	sdio_master_reg_write(adapter, GSPI_CTRL_REG0, gspi_ctrl_reg0_val, 2);
}

void ulp_read_write(PONEBOX_ADAPTER adapter, uint16 addr, uint16 *data, uint16 len_in_bits)
{
	sdio_master_reg_write(adapter, GSPI_DATA_REG1, ((addr << 6) | (data[1] & 0x3f)), 2);
	sdio_master_reg_write(adapter, GSPI_DATA_REG0, ONEBOX_CPU_TO_LE16(*(uint16 *)&data[0]), 2);
	onebox_gspi_init(adapter);
	sdio_master_reg_write(adapter, GSPI_CTRL_REG1, ((len_in_bits - 1) | GSPI_TRIG), 2);
	msleep(10);
}

void reset_chip(PONEBOX_ADAPTER adapter)
{
	struct onebox_os_intf_operations *os_intf_ops;
	uint16 temp[4] = {0};
	uint32 data; 

	os_intf_ops = onebox_get_os_intf_operations_from_origin();


	/* Put TA on hold */
	/* Set MS ACCESS WORD */
	if (adapter->osi_host_intf_ops->onebox_master_access_msword(adapter, 0x2200) != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to set ms word to common reg\n"), __func__));
		return ;              
	}

	data = TA_HOLD_THREAD_VALUE;
	if(adapter->osd_host_intf_ops->onebox_write_multiple(adapter,
				TA_HOLD_THREAD_REG |
				SD_REQUEST_MASTER,
				(uint8 *) &data,
				4, NULL
				)!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to hold TA threads\n"), __func__));
		return ;
	}

#if 0
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("%s %d\n",__func__,__LINE__));
	if (adapter->osi_host_intf_ops->onebox_master_access_msword(adapter,
				0x4008 )!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to set ms word to rf spi reg\n"), __func__));
	}
#endif

	/* This msleep will ensure TA processor to go to hold and any pending dma transfers to rf spi in device to finish */
	msleep(100);
#ifndef SDIO_CMD52_RESET
	if (adapter->device_model == RSI_DEV_9116) {
		if((sdio_master_reg_write(adapter, NWP_WWD_INTERRUPT_TIMER, 5, 4)) < 0) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: One_Box Reset Card Failed\n"), __func__));
		}
		if((sdio_master_reg_write(adapter, NWP_WWD_SYSTEM_RESET_TIMER, 4, 4)) < 0) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: One_Box Reset Card Failed\n"), __func__));
		}
		if((sdio_master_reg_write(adapter, NWP_WWD_MODE_AND_RSTART, 0xAA0001, 4)) < 0) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: One_Box Reset Card Failed\n"), __func__));
		}
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Watch Dog Reset Successful\n"), __func__));
	}
	else {
		*(uint32 *)temp = 0;
		ulp_read_write(adapter, ULP_RESET_REG, temp, 32);
		*(uint32 *)temp = 2;
		ulp_read_write(adapter, WATCH_DOG_TIMER_1, temp, 32);
		*(uint32 *)temp = 0;
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("%s %d\n",__func__,__LINE__));
		ulp_read_write(adapter, WATCH_DOG_TIMER_2, temp, 32);

		*(uint32 *)temp = 50;
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("%s %d\n",__func__,__LINE__));
		ulp_read_write(adapter, WATCH_DOG_DELAY_TIMER_1, temp, 32);
		*(uint32 *)temp = 0;
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("%s %d\n",__func__,__LINE__));
		ulp_read_write(adapter, WATCH_DOG_DELAY_TIMER_2, temp, 32);

		*(uint32 *)temp = ((0xaa000) | RESTART_WDT | BYPASS_ULP_ON_WDT);
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("%s %d value= %x\n",__func__,__LINE__, *temp));
		ulp_read_write(adapter, WATCH_DOG_TIMER_ENABLE, temp, 32);
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("%s %d\n",__func__,__LINE__));
	}
#endif
	msleep(1000);
}


/**
 * This function resets and re-initializes the card.
 *
 * @param  Pointer to sdio_func.  
 * @VOID 
 */
VOID onebox_reset_card(struct sdio_func *pfunction)
{
#ifdef SDIO_CMD52_RESET
	int ret = 0;

	/* Reset 9110 chip */
	ret = onebox_cmd52writebyte(pfunction->card, SDIO_CCCR_ABORT, (1 << 3)); 
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("SDIO CMD52 RESET : %d \n"), ret));
	/* Card will not send any response as it is getting reset immediately
	 * Hence expect a timeout status from host controller */
	if (ret != -ETIMEDOUT)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("RS-9110 reset failed : %d \n"), ret));
	}
	msleep(1000);
#endif
	/* Wait for few milli seconds to get rid of residue charges if any */
	msleep(2);

	/* Initialize the SDIO card */
	do
	{
		int err = ONEBOX_STATUS_FAILURE;
		struct mmc_card *card = pfunction->card;
		struct mmc_host *host = card->host;
		uint8 cmd52_resp = 0;
		uint32 clock;
		uint32 resp, i;
		uint16 rca;
		int bit = fls(host->ocr_avail) - 1;

		/* emulate the mmc_power_up(...) */
		host->ios.vdd = bit;
		host->ios.chip_select = MMC_CS_DONTCARE;
		host->ios.bus_mode = MMC_BUSMODE_OPENDRAIN;
		host->ios.power_mode = MMC_POWER_UP;
		host->ios.bus_width = MMC_BUS_WIDTH_1;
		host->ios.timing = MMC_TIMING_LEGACY;
		host->ops->set_ios(host, &host->ios);
		/*
		 * This delay should be sufficient to allow the power supply
		 * to reach the minimum voltage.
		 */
		msleep(2);

		host->ios.clock = host->f_min;
		host->ios.power_mode = MMC_POWER_ON;
		host->ops->set_ios(host, &host->ios);

		/*
		 * This delay must be at least 74 clock sizes, or 1 ms, or the
		 * time required to reach a stable voltage.
		 */
		msleep(2);

		/* Issue CMD0. Goto idle state */
		host->ios.chip_select = MMC_CS_HIGH;
		host->ops->set_ios(host, &host->ios);
		msleep(1);
		err = onebox_issue_sdiocommand(pfunction, MMC_GO_IDLE_STATE, 0, (MMC_RSP_NONE | MMC_CMD_BC), NULL);
		host->ios.chip_select = MMC_CS_DONTCARE;
		host->ops->set_ios(host, &host->ios);
		msleep(1);
		host->use_spi_crc = 0;

		if (err) 
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: CMD0 failed : %d \n"),__func__,err));    
			break;
		}        

		//if (!host->ocr_avail) 
		{
			/* Issue CMD5, arg = 0 */
			err = onebox_issue_sdiocommand(pfunction, SD_IO_SEND_OP_COND, 0, (MMC_RSP_R4 | MMC_CMD_BCR), &resp);
			if (err) 
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: CMD5 failed : %d \n"),__func__,err));    
				break;
			}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0))
			host->ocr = resp;
#else
			card->ocr = resp;
#endif
		}

		/* Issue CMD5, arg = ocr. Wait till card is ready  */
		for (i = 0; i < 100; i++) 
		{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0))
			err = onebox_issue_sdiocommand(pfunction, SD_IO_SEND_OP_COND, host->ocr, (MMC_RSP_R4 | MMC_CMD_BCR), &resp);
#else
			err = onebox_issue_sdiocommand(pfunction, SD_IO_SEND_OP_COND, card->ocr, (MMC_RSP_R4 | MMC_CMD_BCR), &resp);
#endif
			if (err) 
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: CMD5 failed : %d \n"),__func__,err));    
				break;
			}
			if (resp & MMC_CARD_BUSY) 
			{
				break;
			}
			msleep(10);
		}

		if ((i == 100) || (err)) 
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: card in not ready : %d %d \n"),__func__, i, err));    
			break;
		}

		/* Issue CMD3, get RCA */
		err = onebox_issue_sdiocommand(pfunction, SD_SEND_RELATIVE_ADDR, 0, MMC_RSP_R6 | MMC_CMD_BCR, &resp);
		if (err) 
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: CMD3 failed : %d \n"),__func__,err));    
			break;
		}
		rca = resp >> 16;
		host->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
		host->ops->set_ios(host, &host->ios);

		/* Issue CMD7, select card  */
		err = onebox_issue_sdiocommand(pfunction, MMC_SELECT_CARD, (rca << 16), MMC_RSP_R1 | MMC_CMD_AC, NULL);
		if (err) 
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: CMD7 failed : %d \n"),__func__,err));    
			break;
		}

		/* Enable high speed */
		if (card->host->caps & MMC_CAP_SD_HIGHSPEED) 
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: Set high speed mode\n"),__func__));    
			err = onebox_cmd52readbyte(card, SDIO_CCCR_SPEED, &cmd52_resp);
			if (err) 
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: CMD52 read to CCCR speed register failed  : %d \n"),__func__,err));    
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0))
				card->state &= ~MMC_STATE_HIGHSPEED;
#endif
				/* no need to break */
			} 
			else 
			{
				err = onebox_cmd52writebyte(card, SDIO_CCCR_SPEED, (cmd52_resp | SDIO_SPEED_EHS));
				if (err) 
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: CMD52 write to CCCR speed register failed  : %d \n"),__func__,err));    
					break;
				}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0))
				mmc_card_set_highspeed(card);
#endif
				host->ios.timing = MMC_TIMING_SD_HS;
				host->ops->set_ios(host, &host->ios);
			}
		}

		/* Set clock */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0))
		if (mmc_card_highspeed(card)) 
#else
			if (mmc_card_hs(card))
#endif
			{
				clock = 5000000;
			} 
			else 
			{
				clock = card->cis.max_dtr;
			}

		if (clock > host->f_max) 
		{
			clock = host->f_max;
		}
		host->ios.clock = clock;
		host->ops->set_ios(host, &host->ios);

		if (card->host->caps & MMC_CAP_4_BIT_DATA) 
		{
			/* CMD52: Set bus width & disable card detect resistor */
			err = onebox_cmd52writebyte(card, SDIO_CCCR_IF, SDIO_BUS_CD_DISABLE | SDIO_BUS_WIDTH_4BIT);
			if (err) 
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: CMD52 to set bus mode failed : %d \n"),__func__,err));    
				break;
			}
			host->ios.bus_width = MMC_BUS_WIDTH_4;
			host->ops->set_ios(host, &host->ios);
		}
	} while (0);

	return;
}

/**
 * This function disables the card interface.
 *
 * @param  Pointer to SDcard interface.  
 * @return On success SD_SUCCESS is returned or SD_ERROR_NODEV on error. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
ONEBOX_STATUS  __devexit onebox_disable_interface(void *ptr)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
ONEBOX_STATUS            onebox_disable_interface(void *ptr)
#endif
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	PSDDEVICE pDevice = (PSDDEVICE)ptr; 
	SDCONFIG_FUNC_ENABLE_DISABLE_DATA fData;

	fData.EnableFlags = SDCONFIG_DISABLE_FUNC;
	fData.TimeOut     = 500;
	return SDLIB_IssueConfig(pDevice,
			SDCONFIG_FUNC_ENABLE_DISABLE,
			&fData,
			sizeof(fData));
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	struct sdio_func *pfunction = (struct sdio_func*)ptr;
	int ret = 0;

	ret = sdio_disable_func(pfunction);
	if (ret)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Failed to diable sdio func : %d \n"),ret));
	}
	return ret;
#endif
}/* End <onebox_disable_interface> */


/**
 * This function registers the client driver's interrupt handler
 * with the bus driver.
 *
 * @param  Pointer to SD card interface.  
 * @param  Pointer to interrupt handler.  
 * @return VOID. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
VOID __devinit onebox_request_interrupt_handler(void *ptr,
		SD_INTERRUPT  interrupt_handler,
		PVOID pContext
		)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)  
VOID onebox_request_interrupt_handler(void *ptr,
		SD_INTERRUPT  interrupt_handler,
		PVOID pContext
		)
#endif
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	PSDDEVICE pDevice = (PSDDEVICE)ptr;
	int32 status;

	SDDEVICE_SET_IRQ_HANDLER(pDevice, interrupt_handler, pContext);
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
			(TEXT("%s: Unmasking IRQ \n"), __func__));
	status = SDLIB_IssueConfig(pDevice, SDCONFIG_FUNC_UNMASK_IRQ, NULL, 0);
	if (!SDIO_SUCCESS((status))) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT( "%s: failed to unmask IRQ %d\n"), __func__, status));
	}
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	struct sdio_func *pfunction = (struct sdio_func*)ptr;
	int32         status;
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
			(TEXT( "%s: Unmasking IRQ\n"), __func__));

	status = sdio_claim_irq(pfunction,(sdio_irq_handler_t *)interrupt_handler);
	if (status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: failed to unmask IRQ %d\n"), __func__,
				 status));
	}
#endif
	return;
}/* End <onebox_request_interrupt_handler> */


#if KERNEL_VERSION_BTWN_2_6_(18, 22) 
VOID onebox_dummy_interrupt_handler(PVOID pContext)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26) 
VOID onebox_dummy_interrupt_handler(struct sdio_func *function)
#endif
{
	/*dummy interrupt handler */
	return;
}

/**
 * This function is called by kernel when the driver provided 
 * Vendor and device IDs are matched. All the initialization 
 * work is done here.
 *
 * @param  Pointer to sdio_func structure.  
 * @param  Pointer to sdio_device_id structure.  
 * @return SD_SUCCESS in case of successful initialization or 
 *         a negative error code signifying failure. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
BOOLEAN __devinit onebox_probe(PSDFUNCTION pfunction, PSDDEVICE   pDevice)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
int   onebox_probe(struct sdio_func *pfunction, const struct sdio_device_id *id)
#endif
{
	ONEBOX_STATUS  status;
	PONEBOX_ADAPTER adapter = NULL;
	struct driver_assets *d_assets;

	struct onebox_coex_osi_operations *coex_osi_ops = onebox_get_coex_osi_operations();
	struct onebox_osi_host_intf_operations *osi_host_intf_ops = onebox_get_sdio_osi_host_intf_operations();
	struct onebox_osd_host_intf_operations *osd_host_intf_ops = onebox_get_sdio_osd_host_intf_operations();
	struct onebox_os_intf_operations *os_intf_ops = onebox_get_os_intf_operations_from_origin();

	/* Register gpl related os interface operations */
	onebox_register_os_intf_operations(os_intf_ops);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
		(TEXT("%s: sdio init %s\n"), __func__, sdio_func_id(pfunction)));
	/*Claim the host*/
	onebox_sdio_claim_host(pfunction);

	/*Enable the SDIO interface*/
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	status = onebox_enable_interface(pDevice);

	if (!SDIO_SUCCESS((status)))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
					("%s: Failed to enable interface for the kernels b/w 2.6.18 and 2.6.22\n"), __func__));
		onebox_sdio_release_host(pfunction);
		return status; 
	}
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	status = onebox_enable_interface(pfunction);
	if (status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Failed to enable interface\n"), __func__));
		onebox_sdio_release_host(pfunction);
		return status; 
	}
#endif

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Enabled the interface\n"), __func__));
	
	onebox_sdio_release_host(pfunction);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
			(TEXT("Initialized HAL/CORE/DEV_DEPENDENT Operations\n")));   
	d_assets = os_intf_ops->onebox_mem_zalloc(sizeof(struct driver_assets),GFP_KERNEL);
	if (d_assets == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Memory allocation for d_assets failed\n"), __func__));   
		goto fail;
	}
	os_intf_ops->onebox_memset(d_assets, 0, sizeof(struct driver_assets));

	adapter = os_intf_ops->onebox_mem_zalloc(sizeof(ONEBOX_ADAPTER),GFP_KERNEL);
	if(adapter == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Memory allocation for adapter failed\n"), __func__));   
		goto fail;
	}
	os_intf_ops->onebox_memset(adapter, 0, sizeof(ONEBOX_ADAPTER));
	adapter->host_intf_type = HOST_INTF_SDIO;
	d_assets->host_intf_type = HOST_INTF_SDIO;

	/* Initialise the Core and device dependent operations */
	adapter->coex_osi_ops = coex_osi_ops;
	adapter->osd_host_intf_ops = osd_host_intf_ops;
	adapter->osi_host_intf_ops = osi_host_intf_ops;
	adapter->os_intf_ops = os_intf_ops;

#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	adapter->pDevice    = pDevice;
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	adapter->sdio_pfunction = pfunction;
#endif
	d_assets->global_priv = (void *)adapter;
	d_assets->pfunc       = (void *)pfunction;
	adapter->d_assets = d_assets;
        
	onebox_setcontext(pfunction, (void *)adapter);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Context setting suceeded\n"), __func__));   
	/**In SDIO Case MAx PKT len can be >4K because of RX aggregation(TA to Host) */
	adapter->DataRcvPacket = (uint8*)os_intf_ops->onebox_mem_zalloc((ONEBOX_RCV_BUFFER_LEN * 4),
			GFP_KERNEL);
	if(adapter->DataRcvPacket==NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Memory allocation for receive buffer failed\n"), __func__));   
		goto fail;
	}

	os_intf_ops->onebox_init_static_mutex(&adapter->sdio_interrupt_lock);
	os_intf_ops->onebox_init_dyn_mutex(&adapter->transmit_lock);

	/* coex */
	if(adapter->coex_osi_ops->onebox_common_hal_init(d_assets, adapter)  != ONEBOX_STATUS_SUCCESS) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s Line %d: Failed to initialize common HAL\n"), __func__, __LINE__));
		goto fail;
	}

	if((pfunction->device == 0X9330)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: 9113 MODULE IS CONNECTED\n"),__func__));
		adapter->device_model = RSI_DEV_9113;
	}
	else if((pfunction->device == 0X9116)) {
		adapter->device_model = RSI_DEV_9116;
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: 9116 MODULE IS CONNECTED\n"),__func__));
	} else {
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("*** Invalid Device ID ***\n")));
		goto fail1;
	}

	coex_osi_ops->onebox_read_reg_params(adapter);

	/*Initalizing the hardware*/
	onebox_sdio_claim_host(pfunction);

	status = onebox_setupcard(adapter);   
	if (status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Failed to setup card\n"), __func__));
		os_intf_ops->onebox_mem_free(adapter->DataRcvPacket);
		goto fail1;
	}

	onebox_sdio_release_host(pfunction);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Setup card succesfully\n"), __func__));

    ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s:Vendor Id:%x, Device Id:%x\n"),__func__,
		pfunction->vendor, pfunction->device));

       
	/* Don't remove this for run time check */ 
    d_assets->device_model = adapter->device_model;
        
	if (adapter->osi_host_intf_ops->onebox_init_host_interface(adapter)!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Failed to init slave regs\n"), __func__));
		os_intf_ops->onebox_mem_free(adapter->DataRcvPacket);
		goto fail1;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Initialized SDIO slave regs\n"), __func__));


	onebox_sdio_claim_host(pfunction);
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	onebox_request_interrupt_handler(pDevice, onebox_dummy_interrupt_handler, adapter);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	onebox_request_interrupt_handler(pfunction, 
			(SD_INTERRUPT)onebox_dummy_interrupt_handler,
			adapter);
#endif
	onebox_sdio_release_host(pfunction);

	/* Initialises the Net80211 & core modules */
	if(coex_osi_ops->onebox_device_init(adapter, 1)) /* 1 is for load firmware */
	{    
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Failed in device initialization\n"), __func__));
		goto fail1;
	}    

	if (adapter->osi_host_intf_ops->onebox_master_access_msword(adapter, 0x4105)
			!= ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to set ms word to common reg\n"), __func__));
		goto fail1;              
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Setting ms word to common reg 0x41050000\n"), __func__));
	/* Release the dummy interrupt handler */
	onebox_sdio_claim_host(pfunction);
	onebox_sdio_release_irq(pfunction);
	onebox_sdio_release_host(pfunction);


	d_assets->card_state = GS_CARD_ABOARD;
	d_assets->techs[WLAN_ID].drv_state = MODULE_REMOVED;
	d_assets->techs[WLAN_ID].fw_state = FW_INACTIVE;

	d_assets->techs[BT_ID].drv_state = MODULE_REMOVED;
	d_assets->techs[BT_ID].fw_state = FW_INACTIVE;

	d_assets->techs[ZB_ID].drv_state = MODULE_REMOVED;
	d_assets->techs[ZB_ID].fw_state = FW_INACTIVE;

	/* Register the actual interrupt handler */
	onebox_sdio_claim_host(pfunction);

	/* Connect the interrupt */
	/* interrupt handler is registered only after all initialization is done
	 * to make sure interrupts are disabled till initialization is done.
	 * In firmware bootloading case, interrupts from firmware will come before
	 * all initialization is done. So this is the fix.
	 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	onebox_request_interrupt_handler(pDevice, onebox_linux_handle_interrupt, adapter);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	onebox_request_interrupt_handler(pfunction, 
			(SD_INTERRUPT)onebox_linux_handle_interrupt,
			adapter);
#endif
	onebox_sdio_release_host(pfunction);
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Registered Interrupt handler\n"), __func__));   


#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	return ONEBOX_TRUE;    
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	return ONEBOX_FALSE;    
#endif
	/*Failure in one of the steps will cause the control to be transferred here*/          
fail1:
	adapter->coex_osi_ops->onebox_common_hal_deinit(d_assets, adapter);
fail:
	/* Release the interrupt handler */
	onebox_sdio_claim_host(pfunction);
	onebox_sdio_release_irq(pfunction);

	/*Disable the interface*/
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	onebox_disable_interface(pDevice);  
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)  
	onebox_disable_interface(pfunction);  
#endif
	/* Release the host. It should be called after calling sdio_disable_func() */
	onebox_sdio_release_host(pfunction);

	reset_chip(adapter);
	onebox_sdio_claim_host(pfunction);
	/* Resetting the sdio card to make it ready for the next run */
	onebox_reset_card(pfunction);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s: card reset done successfully\n"), __func__));
	/* Release host */
	onebox_sdio_release_host(pfunction);
	if (adapter != NULL) {
		kfree(adapter);
	}
	if (!d_assets)
		kfree(d_assets);
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Failed to initialize...Exiting\n"), __func__));  
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	return ONEBOX_FALSE;    
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	return ONEBOX_TRUE;
#endif
}/* End <onebox_probe> */


/**
 * This function performs the reverse of the probe function..
 *
 * @param  Pointer to sdio_func structure.  
 * @param  Pointer to sdio_device_id structure.  
 * @return VOID. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
VOID __devexit onebox_disconnect ( PSDFUNCTION pfunction, PSDDEVICE   pDevice)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
VOID onebox_disconnect ( struct sdio_func *pfunction)
#endif
{
	struct onebox_os_intf_operations *os_intf_ops;
	struct driver_assets *d_assets;
	PONEBOX_ADAPTER adapter;
	int status;
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	adapter = pfunction->pContext;   
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	adapter = onebox_getcontext(pfunction);
#endif
	d_assets = adapter->d_assets;
	os_intf_ops = onebox_get_os_intf_operations_from_origin(); 

	FUNCTION_ENTRY(ONEBOX_ZONE_INFO);   
	adapter->coex_osi_ops->onebox_common_hal_deinit(d_assets, adapter);

	status = adapter->osi_host_intf_ops->onebox_disable_sdio_interrupt(adapter);
	onebox_sdio_claim_host(pfunction);
	onebox_sdio_release_irq(pfunction);
	onebox_sdio_release_host(pfunction);
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s:Below are the sdio stats \n"), __func__));
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("total sdio Interrupts recvd : %d\n"), adapter->sdio_int_counter));
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("total sdio Interrupts with zero status recvd : %d\n"), adapter->sdio_intr_status_zero));
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("total sdio buffer Interrupts recvd : %d\n"), adapter->buf_status_interrupts));
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("total data pending Interrupts recvd : %d\n"), adapter->total_sdio_msdu_pending_intr));
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("total sdio Unkown Interrupts recvd : %d\n"), adapter->total_sdio_unknown_intr));

	/*Disable the interface*/
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s: Disabling the interface\n"), __func__));

	reset_chip(adapter);
	onebox_sdio_claim_host(pfunction);
	/* Resetting the sdio card to make it ready for the next run */
	onebox_reset_card(pfunction);
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s: card reset done successfully\n"), __func__));
	/* Release host */
	onebox_sdio_release_host(pfunction);

	adapter->stop_card_write = 2; //stopping all writes after deinit

	os_intf_ops->onebox_mem_free(adapter);
	os_intf_ops->onebox_mem_free(d_assets);
	FUNCTION_EXIT(ONEBOX_ZONE_INFO);

	return;
}/* End <onebox_disconnect> */

ONEBOX_STATIC int onebox_sdio_disable_interrupts(struct sdio_func *pfunction)
{
	PONEBOX_ADAPTER adapter = onebox_getcontext(pfunction);
	ONEBOX_STATUS ret;
	uint8 isr_status = 0, data = 0;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			("%s: Waiting for interrupts to be cleared\n",
			 __func__));
	do {
		read_register(adapter, ONEBOX_FUNCTION_1_INTERRUPT_REGISTER,
				&isr_status);
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("."));
	} while (isr_status);
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("\nInterrupts cleared\n"));

	sdio_claim_host(pfunction);
	ret = onebox_cmd52readbyte(pfunction->card, 0x04, &data);
	if (ret < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				("%s: Failed to read INTR_EN register\n",
				 __func__));
		sdio_release_host(pfunction);
		return ret;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("INTR_EN reg content = %x\n", data));

	/* And bit0 and b1 */
	data &= 0xfc;

	ret = onebox_cmd52writebyte(pfunction->card, 0x04, data);
	if (ret < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				("%s: Failed to Write to INTR_EN register\n",
				 __func__));
		sdio_release_host(pfunction);
		return ret;
	}
	ret = onebox_cmd52readbyte(pfunction->card, 0x04, &data);
	if (ret < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				("%s: Failed to read INTR_EN register\n",
				 __func__));
		sdio_release_host(pfunction);
		return ret;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("INTR_EN reg content. = %x\n", data));

	sdio_release_host(pfunction);

	return 0;
}

ONEBOX_STATIC int onebox_sdio_enable_interrupts(struct sdio_func *pfunction)
{
	ONEBOX_STATUS ret;
	uint8 data;

	sdio_claim_host(pfunction);
	ret = onebox_cmd52readbyte(pfunction->card, 0x04, &data);
	if (ret < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				("%s: Failed to read INTR_EN register\n",
				 __func__));
		sdio_release_host(pfunction);
		return ret;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("INTR_EN reg content1 = %x\n", data));

	/* Enable b1 and b0 */
	data |= 0x03;

	ret = onebox_cmd52writebyte(pfunction->card, 0x04, data);
	if (ret < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				("%s: Failed to Write to INTR_EN register\n",
				 __func__));
		sdio_release_host(pfunction);
		return ret;
	}

	ret = onebox_cmd52readbyte(pfunction->card, 0x04, &data);
	if (ret < 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				("%s: Failed to read INTR_EN register\n",
				 __func__));
		sdio_release_host(pfunction);
		return ret;
	}
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("INTR_EN reg content1.. = %x\n", data));
	sdio_release_host(pfunction);

	return ret;
}

/**
 * This function handles the SDIO bus suspend state
 *
 * @param  SDIO device structure.
 * @return On success SD_SUCCESS is returned or negative error code on failure.
 */
ONEBOX_STATIC int onebox_suspend(struct device *dev)
{
	ONEBOX_STATUS ret = 0;
#if KERNEL_VERSION_GREATER_THAN_2_6_(31)
	struct sdio_func *pfunction = dev_to_sdio_func(dev);
#else
	struct sdio_func *pfunction = container_of(dev, struct sdio_func, dev);
#endif
	PONEBOX_ADAPTER adapter = onebox_getcontext(pfunction);

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("SDIO Bus suspend =====>\n"));

	if (!adapter) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				("%s: Device not ready\n", __func__));
		return ONEBOX_STATUS_FAILURE;
	}
	onebox_sdio_disable_interrupts(pfunction);

	/* Keep Power to the MMC while suspend */
#if KERNEL_VERSION_GREATER_THAN_2_6_(33)
	ret = sdio_set_host_pm_flags(pfunction, MMC_PM_KEEP_POWER);
	if (ret) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				("set sdio keep pwr flag failed\n"));
		return ret;
	}
#endif
	adapter->fsm_state = FSM_CARD_NOT_READY;
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("***** RSI module suspended *****\n"));

	return 0;
}

/**
 * This function handles the SDIO bus resume
 *
 * @param  SDIO device structure.
 * @return On success SD_SUCCESS is returned or negative error code on failure.
 */
ONEBOX_STATIC int onebox_resume(struct device *dev)
{
#if KERNEL_VERSION_GREATER_THAN_2_6_(31)
	struct sdio_func *pfunction = dev_to_sdio_func(dev);
#else
	struct sdio_func *pfunction = container_of(dev, struct sdio_func, dev);
#endif
	PONEBOX_ADAPTER adapter = onebox_getcontext(pfunction);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("SDIO Bus resume =====>\n"));

	adapter->fsm_state = FSM_OPEN;

	onebox_sdio_enable_interrupts(pfunction);

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("***** RSI module resumed *****\n"));

	return 0;
}

/**
 * This function sets the host block length.
 *
 * @param  Pointer to Driver adapter structure.  
 * @param  Block lenght to be set.  
 * @return On success SD_SUCCESS is returned or negative error code on failure. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
ONEBOX_STATUS  __devinit onebox_setblocklength(PONEBOX_ADAPTER adapter, uint32 length)
{
	PSDDEVICE    pDevice    = adapter->pDevice;
	int32        blklength;

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT( "%s: Setting the block length\n"), __func__));

	SDLIB_SetFunctionBlockSize(pDevice,256);
	blklength = SDDEVICE_GET_OPER_BLOCK_LEN(pDevice);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT( "%s: Operational blk length is %d\n", __func__),
				blklength));
	return ONEBOX_STATUS_SUCCESS; 
} 
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
ONEBOX_STATUS        onebox_setblocklength(PONEBOX_ADAPTER adapter, uint32 length)
{
	int32  status;

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
			(TEXT("%s: Setting the block length\n"), __func__));

	status = sdio_set_block_size(adapter->sdio_pfunction, length);
	adapter->sdio_pfunction->max_blksize = 256;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("%s: Operational blk length is %d\n"), __func__, length));
	return status;
} /* End <onebox_setblocklength> */
#endif

/**
 * This function exclusively claims a bus before a certain SDIO function.
 *
 * @param  Pointer to sdio_func structure.  
 * @return VOID. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
VOID onebox_sdio_claim_host(PSDFUNCTION pFunction)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
VOID onebox_sdio_claim_host(struct sdio_func *pfunction)
#endif
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	/*No code here*/
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	sdio_claim_host(pfunction);
#endif
	return;
}/* End <onebox_sdio_claim_host> */

/**
 * This function releases a bus after a certain SDIO function.
 *
 * @param  Pointer to sdio_func structure.  
 * @return VOID. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
VOID onebox_sdio_release_host(PSDFUNCTION pFunction)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
VOID onebox_sdio_release_host(struct sdio_func *pfunction)
#endif
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	/*No code here*/
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	sdio_release_host(pfunction);
#endif
	return;
}/* End <onebox_sdio_release_host> */

/**
 * This function sets the private data of the card function as 
 * our n/w interface.
 *
 * @param  Pointer to sdio_func structure. 
 * @param  Pointer to our network interface 
 * @return VOID. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
VOID onebox_setcontext(PSDFUNCTION pFunction, void *adapter)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
VOID onebox_setcontext(struct sdio_func *pfunction, void *adapter)
#endif
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	/*No code for 2.6.18*/
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	sdio_set_drvdata(pfunction, adapter);
#endif
	return;
}/* End <onebox_setcontext> */

/**
 * This function gets the private data of the cards function.
 *
 * @param  Pointer to sdio_func structure.  
 * @return Pointer to a our network interface. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
struct net_device * onebox_getcontext(PSDFUNCTION pfunction)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
void* onebox_getcontext(struct sdio_func *pfunction)
#endif
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	return NULL;
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	void *dev;
	dev = sdio_get_drvdata(pfunction);
	return dev;
#endif
}/* End <onebox_getcontext> */

/**
 * This function releases the irq registered with the card.
 *
 * @param  Pointer to sdio_func structure.  
 * @return 0 on sucess else a negative number with specific failure. 
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
int32 onebox_sdio_release_irq(PSDFUNCTION pFunction)
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
int32 onebox_sdio_release_irq(struct sdio_func *pfunction)
#endif
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	return 0;
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	return sdio_release_irq(pfunction);
#endif
}

int32 deregister_sdio_irq(PONEBOX_ADAPTER adapter)
{
#if KERNEL_VERSION_GREATER_THAN_2_6_(26)
	int32 s;

	onebox_sdio_claim_host(adapter->sdio_pfunction);
	s = onebox_sdio_release_irq(adapter->sdio_pfunction);
	if (s) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to Deregister irq"), __func__));
	}	
	onebox_sdio_release_host(adapter->sdio_pfunction);

	return s;
#endif
}

/**
 * This function queries and sets the card's features.
 *
 * @param  Pointer to Driver adapter structure.  
 * @return On success ONEBOX_STATUS_SUCCESS else ONEBOX_STATUS_FAILURE is returned.
 */
ONEBOX_STATUS onebox_setupcard(PONEBOX_ADAPTER adapter)
{
	ONEBOX_STATUS Status = ONEBOX_STATUS_SUCCESS;

	/* Setting sdio clock frequency to 50MHz */
#ifdef FPGA_VALIDATION
	onebox_setclock(adapter,6000);
#else
	if(!adapter->sdio_clock_speed)
	{
		adapter->sdio_clock_speed = 20000;
	}	
	onebox_setclock(adapter, adapter->sdio_clock_speed);
#endif
#ifdef RSI_IMX51 
	onebox_setclock(adapter,6000);
#endif
	if(Status!=ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unsuccessful at setting clk"), __func__));
		return Status;
	}
#if KERNEL_VERSION_BTWN_2_6_(18, 22) 
	adapter->CardCapability = SDDEVICE_GET_SDIO_CARD_CAPS(adapter->pDevice);
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Card Cap: %0x\n"), __func__,
				adapter->CardCapability));
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Common CIS Ptr:  %0x\n"), __func__,
				SDDEVICE_GET_SDIO_COMMON_CISPTR(adapter->pDevice)));
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Funcn CIS Ptr:  %0x\n"), __func__,
				SDDEVICE_GET_SDIO_FUNC_CISPTR(adapter->pDevice)));
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: CSA Ptr:  %0x\n"), __func__,
				SDDEVICE_GET_SDIO_FUNC_CSAPTR(adapter->pDevice)));
#endif
	adapter->TransmitBlockSize = 256;
	adapter->ReceiveBlockSize  = 256;
	Status = onebox_setblocklength(adapter, adapter->TransmitBlockSize);

	if(Status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to set block length\n"), __func__));
		return Status;
	}

	return Status;  
}/* End <onebox_setupcard> */

/**
 * This function sets the clock frequency
 * @param
 *  adapter     Pointer To ONEBOX_ADAPTER sturct
 * @param
 *  Frequency   Clock frequency
 */
ONEBOX_STATUS onebox_setclock(PONEBOX_ADAPTER adapter, uint32 Freq)
{
#if KERNEL_VERSION_BTWN_2_6_(18,22)
	SDCONFIG_BUS_MODE_DATA  busSettings;
#ifdef ONEBOX_WITHOUT_HARDWARE
	{
		return ONEBOX_STATUS_SUCCESS;
	}
#endif
	adapter->os_intf_ops->onebox_memset(&busSettings, 0, sizeof(SDCONFIG_BUS_MODE_DATA));
	busSettings.BusModeFlags = SDDEVICE_GET_BUSMODE_FLAGS(adapter->pDevice);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
			(TEXT("%s: Forcing SDIO clock to %dMHz\n"), __func__,
			 busSettings.ClockRate));

	busSettings.ClockRate = Freq * 1000;
	return SDLIB_IssueConfig(adapter->pDevice, SDCONFIG_BUS_MODE_CTRL,
			&busSettings, sizeof(SDCONFIG_BUS_MODE_DATA));
#else
	uint32 clock;
	struct mmc_host *host = adapter->sdio_pfunction->card->host;

	clock = Freq * 1000;
	if (clock > host->f_max) 
	{
		clock = host->f_max;
	}
	host->ios.clock = clock;
	host->ops->set_ios(host, &host->ios);
	return 0;
#endif
}

/**
 * This function reads one byte of information from a register.
 *
 * @param  Pointer to Driver adapter structure.  
 * @param  Function Number.  
 * @param  Address of the register.  
 * @param  Pointer to the data that stores the data read.  
 * @return On success ONEBOX_STATUS_SUCCESS else ONEBOX_STATUS_FAILURE. 
 */
//ONEBOX_STATUS read_register(PONEBOX_ADAPTER ext_adapter, uint32 Addr,
ONEBOX_STATUS read_register(PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data)  
{
		uint8 fun_num = 0;
	int32 status;
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	return SDLIB_IssueCMD52(adapter->pDevice,
			fun_num,
			Addr,
			data,
			1,
			ONEBOX_FALSE  
			);

#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)

	if(onebox_likely(adapter->in_sdio_litefi_irq != current))
	{
		onebox_sdio_claim_host(adapter->sdio_pfunction);
	}

	if (fun_num == 0)
	{
		*data = sdio_f0_readb(adapter->sdio_pfunction, Addr, &status);
	}
	else
	{
		*data = sdio_readb(adapter->sdio_pfunction, Addr, &status);
	} /* End if <condition> */

	if (onebox_likely(adapter->in_sdio_litefi_irq != current))
	{
		onebox_sdio_release_host(adapter->sdio_pfunction);
	}
	if (status)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	else
	{
		return ONEBOX_STATUS_SUCCESS;
	}/* End if <condition> */
#endif
}/* End <read_register> */

/**
 * This function writes one byte of information into a register.
 *
 * @param  Pointer to Driver adapter structure.  
 * @param  Function Number.  
 * @param  Address of the register.  
 * @param  Pointer to the data tha has to be written.  
 * @return On success ONEBOX_STATUS_SUCCESS else ONEBOX_STATUS_FAILURE. 
 */
int write_register(PONEBOX_ADAPTER adapter, uint8 reg_dmn,
		   uint32 Addr, uint8 *data)
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	return SDLIB_IssueCMD52(adapter->pDevice,
			reg_dmn,
			Addr,
			data,
			1,
			ONEBOX_TRUE  
			);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	int status=0;

	if (onebox_likely(adapter->in_sdio_litefi_irq != current))
	{
		onebox_sdio_claim_host(adapter->sdio_pfunction);
	}

	if (reg_dmn == 0)
	{
		sdio_f0_writeb(adapter->sdio_pfunction, *data, Addr, &status);
	}
	else
	{
		sdio_writeb(adapter->sdio_pfunction, *data, Addr, &status);
	} /* End if <condition> */
	if (onebox_likely(adapter->in_sdio_litefi_irq != current))
	{ 
		onebox_sdio_release_host(adapter->sdio_pfunction);
	}
	if (status)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	else
	{
		return ONEBOX_STATUS_SUCCESS;
	} /* End if <condition> */
#endif
}/* End <write_register> */

/**
 * This function read multiple bytes of information from the SD card.
 *
 * @param  Pointer to Driver adapter structure.  
 * @param  Function Number.  
 * @param  Address of the register.  
 * @param  Length of the data to be read.  
 * @param  Pointer to the read data.  
 * @return On success ONEBOX_STATUS_SUCCESS else ONEBOX_STATUS_FAILURE. 
 */
ONEBOX_STATUS read_register_multiple(PONEBOX_ADAPTER adapter, 
		uint32 Addr,
		uint32 Count,
		uint8 *data )
{
	uint32      status;
#if KERNEL_VERSION_BTWN_2_6_(18, 22)  
	PSDREQUEST  pReq       = NULL;
	PSDDEVICE   pDevice ; 
	uint32      num_blocks = Count / 256;
#endif
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	pDevice = adapter->pDevice;

	if (Count % 256 != 0)
	{
		++num_blocks;
	}

	pReq = SDDeviceAllocRequest(pDevice);
	if (pReq == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Request Allocation Failed\n"), __func__));
		return ONEBOX_STATUS_NO_RESOURCES;
	}

	/* If Length Less Than 256 Then Byte Mode */
	if (Count < 256)
	{
		SDIO_SET_CMD53_ARG(pReq->Argument,
				CMD53_READ,
				1,
				CMD53_BYTE_BASIS,
				CMD53_FIXED_ADDRESS,
				Addr,
				Count);
		pReq->pDataBuffer = data;
		pReq->Command     = CMD53;
		pReq->Flags       = SDREQ_FLAGS_RESP_SDIO_R5 | SDREQ_FLAGS_DATA_TRANS;
		pReq->BlockLen    = Count;
		pReq->BlockCount  = 1;    
	}
	else  /* Block Mode */
	{
		SDIO_SET_CMD53_ARG(pReq->Argument,
				CMD53_READ,
				1,
				CMD53_BLOCK_BASIS,
				CMD53_FIXED_ADDRESS,
				Addr,
				num_blocks);
		pReq->pDataBuffer = data;
		pReq->Command     = CMD53;
		pReq->Flags       = SDREQ_FLAGS_RESP_SDIO_R5 | SDREQ_FLAGS_DATA_TRANS;
		pReq->BlockLen    = 256; /*adapter->ReceiveBlockSize;*/
		pReq->BlockCount  = num_blocks;    
	} /* End if <condition> */
	status = SDDEVICE_CALL_REQUEST_FUNC(pDevice, pReq);
	if (!SDIO_SUCCESS(status))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Synch Cmd53 read failed\n"), __func__));
	}
	else
	{
		/*
		 * ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		 * "onebox_read_register_multiple: Synch Cmd53 read Success\n");
		 * */
	} /* End if <condition> */

	SDDeviceFreeRequest(pDevice,pReq);               
	return status;
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)

	if (onebox_likely(adapter->in_sdio_litefi_irq != current))
	{
		onebox_sdio_claim_host(adapter->sdio_pfunction);
	}

	status =  sdio_readsb(adapter->sdio_pfunction, data, Addr, Count);
	if (onebox_likely(adapter->in_sdio_litefi_irq != current))
	{
		onebox_sdio_release_host(adapter->sdio_pfunction);
	}

	if (status != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Synch Cmd53 read failed\n"), __func__));
	}
	return status;
#endif

}/* End <read_register_multiple> */  

/**
 * This function writes multiple bytes of information to the SD card.
 *
 * @param  Pointer to Driver adapter structure.  
 * @param  Function Number.  
 * @param  Address of the register.  
 * @param  Length of the data.  
 * @param  Pointer to the data that has to be written.  
 * @return On success ONEBOX_STATUS_SUCCESS else ONEBOX_STATUS_FAILURE. 
 */
ONEBOX_STATUS write_register_multiple(PONEBOX_ADAPTER adapter,
		uint32 Addr,
		uint8 *data,
		uint32 Count,
		netbuf_ctrl_block_t *netbuf_cb
		)
{
	int32   status = ONEBOX_STATUS_SUCCESS;
#if KERNEL_VERSION_BTWN_2_6_(18, 22)  
	PSDDEVICE    pDevice;
	PSDREQUEST   pReq       = NULL;
	uint32       num_blocks = Count / 256;
#endif
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	pDevice    = adapter->pDevice;
	if (Count % 256 != 0)
	{
		++num_blocks;
	}

	pReq = SDDeviceAllocRequest(pDevice);
	if (pReq == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Resource Allocation Failed\n"), __func__));
		return ONEBOX_STATUS_NO_RESOURCES;
	}

	/* If Length Less Than 256 Then Byte Mode */
	if (Count < 256)
	{
		SDIO_SET_CMD53_ARG(pReq->Argument,
				CMD53_WRITE,
				1,
				CMD53_BYTE_BASIS,
				CMD53_FIXED_ADDRESS,
				Addr,
				Count);
		pReq->pDataBuffer = &data[0];
		pReq->Command     = CMD53;
		pReq->Flags       = SDREQ_FLAGS_RESP_SDIO_R5 |
			SDREQ_FLAGS_DATA_TRANS   |
			SDREQ_FLAGS_DATA_WRITE;
		pReq->BlockCount = 1;    
		pReq->BlockLen   = Count;

	}
	else /* Block Mode */
	{
		SDIO_SET_CMD53_ARG(pReq->Argument,
				CMD53_WRITE,
				1,
				CMD53_BLOCK_BASIS,
				CMD53_FIXED_ADDRESS,
				Addr,
				num_blocks);
		pReq->pDataBuffer = &data[0];
		pReq->Command     = CMD53;
		pReq->Flags       = SDREQ_FLAGS_RESP_SDIO_R5 |
			SDREQ_FLAGS_DATA_TRANS   |
			SDREQ_FLAGS_DATA_WRITE;
		pReq->BlockCount = num_blocks;    
		pReq->BlockLen   = 256; /* adapter->TransmitBlockSize; */
	} /* End if <condition>i */
	do
	{
		status = SDDEVICE_CALL_REQUEST_FUNC(pDevice, pReq);
		if (!SDIO_SUCCESS(status))
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
					(TEXT("%s: Synch Cmd53 write failed %d\n"), __func__, 
					 status));
		} 
	} while (!SDIO_SUCCESS(status));
	SDDeviceFreeRequest(pDevice,pReq);               

	return status;

#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	//remove this later
	if(adapter->stop_card_write == 2)
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Stopping card writes\n"));
		return ONEBOX_STATUS_FAILURE;
	}
	else if(adapter->stop_card_write == 1)
	{
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Continue writes\n"));
		adapter->stop_card_write++;
	}

	if(onebox_likely(adapter->in_sdio_litefi_irq != current))
	{
		onebox_sdio_claim_host(adapter->sdio_pfunction);
	}

	status = sdio_writesb(adapter->sdio_pfunction, Addr, data, Count);

	if (onebox_likely(adapter->in_sdio_litefi_irq != current))
	{
		onebox_sdio_release_host(adapter->sdio_pfunction);
	}

	if (status != ONEBOX_STATUS_SUCCESS)
	{
		//dump_stack();
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Synch Cmd53 write failed %d\n"), __func__,
				 status));
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
				(TEXT("%s: Addr = %x, Count = %d \n"), __func__, Addr, Count));
		adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG, data, Count);
		adapter->stop_card_write = 2; /* To stop further card writes, as card might be removed. */
	}
	else
	{
		adapter->os_intf_ops->onebox_memcpy(adapter->prev_desc, data, FRAME_DESC_SZ);
	}
	return status;
#endif
} /* End <write_register_multiple> */

/**
 * This function registers the client driver.
 *
 * @param  VOID.  
 * @return 0 if success else a negative number. 
 */
int register_driver(void)
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	return (SDIOErrorToOSError(SDIO_RegisterFunction(&onebox_driver)));       
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)  
	return (sdio_register_driver(&onebox_driver));
#endif
}

/**
 * This function unregisters the client driver.
 *
 * @param  VOID.  
 * @return VOID. 
 */
void unregister_driver(void)
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	SDIO_UnregisterFunction(&onebox_driver);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	sdio_unregister_driver(&onebox_driver);
#endif
}

/*FUNCTION*********************************************************************
  Function Name:  onebox_open_sdbus_handle
Description:    Dummy for linux
Returned Value: None
Parameters: 
----------------------------+-----+-----+-----+------------------------------
Name                        | I/P | O/P | I/O | Purpose
----------------------------+-----+-----+-----+------------------------------
None
 ******************************************************************************/
ONEBOX_STATUS onebox_open_sdbus_handle(PONEBOX_ADAPTER adapter)
{
	/**Dummy for Linux*/
	return 0;
}

/*FUNCTION*********************************************************************
  Function Name:  onebox_remove
Description:    Dummy for linux sdio
Returned Value: None
Parameters: 
----------------------------+-----+-----+-----+------------------------------
Name                        | I/P | O/P | I/O | Purpose
----------------------------+-----+-----+-----+------------------------------
None
 ******************************************************************************/
ONEBOX_STATUS sdio_remove(void)
{
	/**Dummy for Linux*/
	return 0;
}

/**
 * This function claims the device.
 *
 * @param  Pointer to driver adapter structure.  
 * @return VOID. 
 */
void claim_device(PONEBOX_ADAPTER adapter)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
	struct sdio_func *pfunc=adapter->sdio_pfunction;
	sdio_claim_host(pfunc);
	adapter->in_sdio_litefi_irq = current;
#endif
}

/**
 * This function releases the device.
 *
 * @param  Pointer to driver adapter structure.  
 * @return VOID. 
 */
void release_device(PONEBOX_ADAPTER adapter)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
	struct sdio_func *pfunc=adapter->sdio_pfunction;
	adapter->in_sdio_litefi_irq = NULL;
	sdio_release_host(pfunc);
#endif
}

#ifndef SDIO_COMMANDS
ONEBOX_STATUS onebox_cmd52writebyte_fnx(struct mmc_card *card, unsigned int address, unsigned char byte,uint8 function_no) 
{
  return -EPERM;
}

ONEBOX_STATUS  onebox_cmd52readbyte_fnx(struct mmc_card *card, unsigned int address, unsigned char *byte,uint8 function)
{
  return -EPERM;
}
ONEBOX_STATUS configure_ahb_master (PONEBOX_ADAPTER adapter, uint32 Addr)
{
  return -EPERM;
}
ONEBOX_STATUS sdio_fnx_cmd52 (PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data, uint8 function_no, uint8 write)
{
  return -EPERM;
}
ONEBOX_STATUS sdio_fnx_cmd53 (PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data, uint8 write, uint8 Count)
{
  return -EPERM;
}
int trace_firmware (struct driver_assets *d_assets)
{
  return -EPERM;
}
#endif
