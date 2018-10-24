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
#include "onebox_linux.h"
#include "onebox_sdio_intf.h"
#include "onebox_host_intf_ops.h"
#include "onebox_zone.h"

typedef struct master_read_write
{
  uint8_t read_write;
  uint32_t address ;          
  int no_of_bytes;
}m_rw_t;

#ifdef SDIO_COMMANDS

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

#define SDIO_FUNCTION_0               0  
#define SDIO_FUNCTION_1               1
#define THREAD0_PC 0x22000400
#define THREAD1_PC 0x22000480
#define THREAD2_PC 0x22000500

#define THREAD0_IPL_OFFSET 0x14
#define THREAD1_IPL_OFFSET 0x94
#define THREAD2_IPL_OFFSET 0x114

#define THREAD0_IPL (THREAD0_PC + THREAD0_IPL_OFFSET)
#define THREAD1_IPL (THREAD0_PC + THREAD1_IPL_OFFSET)
#define THREAD2_IPL (THREAD0_PC + THREAD2_IPL_OFFSET)

#define THREAD0_R 0x22000440
#define THREAD1_R 0x220004C0
#define THREAD2_R 0x22000540
#define SD_REQUEST_MASTER                 0x10000


ONEBOX_STATUS onebox_cmd52writebyte_fnx(struct mmc_card *card, unsigned int address, unsigned char byte,uint8 function_no) ;
ONEBOX_STATUS  onebox_cmd52readbyte_fnx(struct mmc_card *card, unsigned int address, unsigned char *byte,uint8 function);

ONEBOX_STATUS  onebox_cmd52readbyte_fnx(struct mmc_card *card,
				       unsigned int address,
				       unsigned char *byte,uint8 function)
{
	struct mmc_command ioCmd;
	unsigned long   arg;
	int err;
	memset(&ioCmd,0,sizeof(ioCmd));
	SDIO_SET_CMD52_READ_ARG(arg,function,address);
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
ONEBOX_STATUS  onebox_cmd52writebyte_fnx(struct mmc_card *card, 
					 unsigned int address,
					 unsigned char byte,uint8 function_no)
{
	struct mmc_command ioCmd;
  unsigned long      arg;

	memset(&ioCmd,0,sizeof(ioCmd));
	SDIO_SET_CMD52_WRITE_ARG(arg,function_no,address,byte);
	ioCmd.opcode = SD_IO_RW_DIRECT;
	ioCmd.arg = arg;
	ioCmd.flags = MMC_RSP_R5 | MMC_CMD_AC;
	return mmc_wait_for_cmd(card->host, &ioCmd, 0);
}

ONEBOX_STATUS configure_ahb_master (PONEBOX_ADAPTER adapter, uint32 Addr)
{
  int status = 0;
  uint16 msb_address ;
  uint8 *ahb_address = NULL;
  struct mmc_card *card = adapter->sdio_pfunction->card;
  msb_address = Addr >> 16;
  msb_address = (0xFFFE & msb_address);
  ahb_address = kzalloc(8, GFP_KERNEL);
#if KERNEL_VERSION_GREATER_THAN_2_6_(26)
  if (onebox_likely(adapter->in_sdio_litefi_irq != current))
  {
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	/*No code here*/
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	sdio_claim_host(adapter->sdio_pfunction);
#endif
  }
  *ahb_address = (0x00FF & (msb_address));
  status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_MSBYTE,*ahb_address,SDIO_FUNCTION_0);
  if(status != ONEBOX_STATUS_SUCCESS)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
        (TEXT("%s: FAIL TO ACCESS MASTER_ACCESS_MSBYTE\n"), __func__));
  }
  *ahb_address = (msb_address >> 8);
  status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_LSBYTE,*ahb_address,SDIO_FUNCTION_0);
  if(status != ONEBOX_STATUS_SUCCESS)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
        (TEXT("%s: FAIL TO ACCESS MASTER_ACCESS_LSBYTE\n"), __func__));
  }
  if (onebox_likely(adapter->in_sdio_litefi_irq != current))
  {
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	/*No code here*/
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	sdio_release_host(adapter->sdio_pfunction);
#endif
  }
#endif
	kfree(ahb_address);
  	return status;
}
ONEBOX_STATUS sdio_fnx_cmd52 (PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data, uint8 function_no, uint8 write)
{
  int status = 0;
  uint32 master_mode = 0;
  struct mmc_card *card = adapter->sdio_pfunction->card;
  master_mode = (Addr & BIT(16));
  if(master_mode){
    status = configure_ahb_master(adapter, Addr);
    if(status)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
          (TEXT("%s: ACCESS TO AHB MASTER FAILED %d \n"), __func__,status));
    }
  } else {
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
          (TEXT("%s: SDIO SLAVE MODE OPERATION \n"), __func__));
  }
  if (onebox_likely(adapter->in_sdio_litefi_irq != current))
  {

#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	/*No code here*/
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	sdio_claim_host(adapter->sdio_pfunction);
#endif
  }
  Addr = (0x0001FFFF & Addr );
  if(write){
    status = onebox_cmd52writebyte_fnx(card, Addr,*data,function_no);
    if(status != ONEBOX_STATUS_SUCCESS)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
          (TEXT("%s: SDIO CMD52 (W) FAILED =%d \n"), __func__,status));
      goto cmd52_end;
    }
  } else {
    status = onebox_cmd52readbyte_fnx(card,Addr,(unsigned char *)data,function_no);
    if(status != ONEBOX_STATUS_SUCCESS)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
          (TEXT("%s: SDIO CMD52 (R) FAILED =%d \n"), __func__,status));
      goto cmd52_end;
    }
  }
cmd52_end:
  if(master_mode){
    status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_MSBYTE,0x05,SDIO_FUNCTION_0);
    if(status != ONEBOX_STATUS_SUCCESS)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
          (TEXT("%s: fail to access MASTER_ACCESS_MSBYTE\n"), __func__));
    }
    status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_LSBYTE,0x41,SDIO_FUNCTION_0);
  }
  if (onebox_likely(adapter->in_sdio_litefi_irq != current))
  {
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	/*No code here*/
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	sdio_release_host(adapter->sdio_pfunction);
#endif
  }
  return status;
};
EXPORT_SYMBOL(sdio_fnx_cmd52);

ONEBOX_STATUS sdio_fnx_cmd53 (PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data, uint8 write, uint8 Count)
{
  int status = 0;
  uint32 master_mode = 0;
  struct mmc_card *card = adapter->sdio_pfunction->card;
  master_mode = (Addr & BIT(16));
  if(master_mode){
    status = configure_ahb_master(adapter, Addr);
    if(status)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
          (TEXT("%s: ACCESS TO AHB MASTER FAILED %d \n"), __func__,status));
    }
  } else {
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
          (TEXT("%s: SDIO SLAVE MODE OPERATION \n"), __func__));
  }
  if (onebox_likely(adapter->in_sdio_litefi_irq != current))
  {

#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	/*No code here*/
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	sdio_claim_host(adapter->sdio_pfunction);
#endif
  }
  Addr = (0x0001FFFF & Addr );
  if(write){

	status = sdio_writesb(adapter->sdio_pfunction, Addr, data, Count);
    if(status != ONEBOX_STATUS_SUCCESS)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
          (TEXT("%s: SDIO CMD53 (W) FAILED =%d \n"), __func__,status));
      goto cmd52_end;
    }
  } else {
	status =  sdio_readsb(adapter->sdio_pfunction, data, Addr, Count);
    if(status != ONEBOX_STATUS_SUCCESS)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
          (TEXT("%s: SDIO CMD5R (R) FAILED =%d \n"), __func__,status));
      goto cmd52_end;
    }
  }
cmd52_end:
  if(master_mode){
    status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_MSBYTE,0x05,SDIO_FUNCTION_0);
    if(status != ONEBOX_STATUS_SUCCESS)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
          (TEXT("%s: fail to access MASTER_ACCESS_MSBYTE\n"), __func__));
    }
    status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_LSBYTE,0x41,SDIO_FUNCTION_0);
  }
  if (onebox_likely(adapter->in_sdio_litefi_irq != current))
  {
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	/*No code here*/
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	sdio_release_host(adapter->sdio_pfunction);
#endif
  }
  return status;
};
EXPORT_SYMBOL(sdio_fnx_cmd53);

int trace_firmware (struct driver_assets *d_assets)
{
  uint8 function = 1;
  int32 status;
  uint8 i,ii,iii,iv;
  static uint32 info_reg[3][18];
  static uint32 sp[3][50];
  uint32 shift_reg = 0;
  uint8 *data = NULL;
  uint8 thread_id = 0;
  uint32 address,offset;
  PONEBOX_ADAPTER adapter =  (PONEBOX_ADAPTER)d_assets->global_priv;
  struct mmc_card *card = adapter->sdio_pfunction->card;
  i=ii=iii=iv= 0;
  data = kzalloc(8, GFP_KERNEL);
  if(onebox_likely(adapter->in_sdio_litefi_irq != current))
  {
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
    /*No code here*/
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
    sdio_claim_host(adapter->sdio_pfunction);
#endif
  }
  *data = (0x00FF & (THREAD0_PC >> 16));
  status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_MSBYTE,*data,SDIO_FUNCTION_0);
  if(status != ONEBOX_STATUS_SUCCESS)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
        (TEXT("%s: fail to access MASTER_ACCESS_MSBYTE\n"), __func__));
    goto end_trace;
  }
  *data = (THREAD0_PC >> 24);
  status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_LSBYTE,*data,SDIO_FUNCTION_0);
  if(status != ONEBOX_STATUS_SUCCESS)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
        (TEXT("%s: fail to access MASTER_ACCESS_LSBYTE\n"), __func__));
    goto end_trace;
  }
  address = (0x0001FFFF & THREAD0_R );
  offset = (SD_REQUEST_MASTER | address);
  do { 
    for(ii = 0 ;ii < 2 ;ii++)
    {
      if(ii < 1)
      {
        offset = 0;
        address = ((0x0001FFFF) & (THREAD0_PC + ((iii++)*(0x80))));
        offset = (SD_REQUEST_MASTER | (address ));
      }else if(ii < 2) 
      {
        offset = 0;
        address = ((0x0001FFFF) & (THREAD0_IPL + ((iv++)*(0x80))));
        offset = (SD_REQUEST_MASTER | (address ));
      }
      for(i = 0 ;i < 4 ;i++)
      {
        status = onebox_cmd52readbyte_fnx(card, (offset + i),(unsigned char *)data,function);
        if(status) 
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s :FAILED MMC-SDIO_CMD52-R ERROR=%d\n"),__func__,status));
          break;
        }
        shift_reg = *data;
        shift_reg = (shift_reg << (i * 8));
        info_reg[thread_id][ii] |= shift_reg;
      }
      if(status)
        break;
    }
    thread_id ++;
  } while(thread_id < 3);
  thread_id = 0;
  address = 0;
  offset = 0;
  address = (0x0001FFFF & THREAD0_R );
  do { 
    offset = (SD_REQUEST_MASTER | address);
    for(ii = 2 ;ii < 18 ;ii++)
    {
      for(i = 0 ;i < 4 ;i++)
      {
        status = onebox_cmd52readbyte_fnx(card, (offset + i),(unsigned char *)data,function);
        if(status) 
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s :FAILED MMC-SDIO_CMD52-R ERROR=%d\n"),__func__,status));
          break;
        }
        shift_reg = *data;
        shift_reg = (shift_reg << (i * 8));
        info_reg[thread_id][ii] |= shift_reg;
      }
      if(status)
        break;
      offset += 0x4;
    }
    address += 0x80;
    thread_id ++;
  } while(thread_id < 3);
  thread_id = 0;
  address = 0;
  offset = 0;
  do { 
    *data = (0x00FF & ((info_reg[thread_id][17]) >> 16));
    status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_MSBYTE,*data,SDIO_FUNCTION_0);
    if(status != ONEBOX_STATUS_SUCCESS)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
          (TEXT("%s: fail to access MASTER_ACCESS_MSBYTE\n"), __func__));
      goto end_trace;
    }
    *data = 0;
    *data = ((info_reg[thread_id][17]) >> 24);
    status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_LSBYTE,*data,SDIO_FUNCTION_0);
    if(status != ONEBOX_STATUS_SUCCESS)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
          (TEXT("%s: fail to access MASTER_ACCESS_LSBYTE\n"), __func__));
      goto end_trace;
    }
    address = (0x0000FFFF & info_reg[thread_id][17] );
    offset = (SD_REQUEST_MASTER | address);
    for(ii = 0 ;ii < 50 ;ii++)
    {
      for(i = 0 ;i < 4 ;i++)
      {
        status = onebox_cmd52readbyte_fnx(card, (offset + i),(unsigned char *)data,function);
        if(status) 
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s :FAILED MMC-SDIO_CMD52-R ERROR=%d\n"),__func__,status));
          break;
        }
        shift_reg = *data;
        shift_reg = (shift_reg << (i * 8));
        sp[thread_id][ii] |= shift_reg;
      }
      if(status)
        break;
      offset += 0x4;
    }
    thread_id ++;
  } while(thread_id < 3);
  status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_MSBYTE,0x05,SDIO_FUNCTION_0);
  if(status != ONEBOX_STATUS_SUCCESS)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
        (TEXT("%s: fail to access MASTER_ACCESS_MSBYTE\n"), __func__));
    goto end_trace;
  }
  *data = (THREAD0_PC >> 24);
  status = onebox_cmd52writebyte_fnx(card, SDIO_MASTER_ACCESS_LSBYTE,0x41,SDIO_FUNCTION_0);
  if (onebox_likely(adapter->in_sdio_litefi_irq != current))
  {
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	/*No code here*/
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
	sdio_release_host(adapter->sdio_pfunction);
#endif
  }
  ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("|----------------------------------------------------------------| \n")));
  for(ii = 0; ii < 2; ii++)
  {
    if(ii == 0) {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("|Thread0_PC:%8x | Thread1_PC:%8x | Thread2_PC:%8x |\n"), info_reg[0][ii],info_reg[1][ii],info_reg[2][ii]));
    }
    else {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("|Thread0_IPL:%8x| Thread1_IPL:%8x| Thread2_IPL:%8x|\n"), info_reg[0][ii],info_reg[1][ii],info_reg[2][ii]));
    }
  }
  for(ii = 2; ii < 18; ii++)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("|Thread0R%2d:%8x | Thread1R%2d:%8x | Thread2R%2d:%8x |\n"), (ii - 2), info_reg[0][ii], (ii - 2), info_reg[1][ii], (ii - 2), info_reg[2][ii]));
  }
  ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("|----------------------------------------------------------------| \n")));
  thread_id = 0;
  do{
    for(ii = 0; ii < 50; ii++)
    {
      if(ii == 0)
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("\n\nThread%d[SP]:{"),thread_id));
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%x, "),sp[thread_id][ii]));
      if(ii == 49)
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("}\n ")));
    }
    thread_id++;
  } while(thread_id < 3);
end_trace:
  kfree(data);
  return status;
}
EXPORT_SYMBOL(trace_firmware);

ssize_t onebox_proc_sdio_cmd(struct file *filp,
					    const char __user *buff,
					    size_t len,
					    loff_t *data)
{
  char *cmd = NULL;
  uint32 i = 0;
  cmd52_t *cmd52 = NULL;
  cmd53_t *cmd53 = NULL;
  uint8 *offset = NULL;
  uint8 *Data = NULL;
 const char *temp = NULL;
  char *Command = NULL;
	gfp_t flag;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE_DATA(filp->f_inode);
#else
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE(filp->f_mapping->host)->data;
#endif
  if (!len)
    return 0;

  if (len < 9)
  {
  ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Invalid arguments please refer Usage :\n cat /proc/oneboxX/sdio_cmd \n")));
    return -EINVAL;
  }

    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    cmd = (char*)kzalloc(len,flag);
      if(cmd == NULL)
        return -ENOMEM;

  if (copy_from_user(cmd, (void __user *)buff, len)) {
    kfree(cmd);
    return -EFAULT;
  }

  if((*(uint16*)cmd == 0x3235)) {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("CMD52 \n")));
  } else if((*(uint16*)cmd == 0x3335)) {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("CMD53 \n")));
  } else{
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("INVALID CMD %c%c \n"),cmd[0],cmd[1]));
    kfree(cmd);
    return -EINVAL;
  }


  if(cmd[2] != 0x20) {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("INVALID FORMAT Please give space b/t elements\n")));
    kfree(cmd);
    return -EINVAL;
  }

  if((*(uint16*)cmd == 0x3235))  //cmd52
  {

    if (len > 22)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Invalid arguments please refer Usage\n")));
      kfree(cmd);
      return -EINVAL;
    }

    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("CMD52 \n")));

    cmd = strstrip(cmd);

    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    cmd52= (cmd52_t*) kzalloc((sizeof(cmd52_t)),flag);
    if(cmd52 == NULL){
      kfree(cmd);
      return -ENOMEM;
    }



    offset = strsep(&cmd," ");
    if(offset != NULL)
      cmd52->function_no= simple_strtol(offset,NULL,10);


    offset = strsep(&cmd," ");
    if(offset != NULL)
      cmd52->master= simple_strtol(offset,NULL,10);


    offset = strsep(&cmd," ");
    if(offset != NULL)
      cmd52->address= simple_strtol(offset,NULL,16);


    if(cmd52->master)
      cmd52->address |= SD_REQUEST_MASTER;

    offset = strsep(&cmd," ");
    if(offset != NULL)
      cmd52->write = simple_strtol(offset,NULL,10);

    if(cmd52->write){
    offset = strsep(&cmd," ");
    if(offset != NULL)
      cmd52->data = simple_strtol(offset,NULL,16);
    }

    if((cmd52->function_no > 5) || ((cmd52->function_no > 0 ) && (cmd52->master < 1 ))) {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Please provide appropriate SDIO function No[0-5] and use Master mode if funtion number is above 0 \n")));
      goto FREE;
    }
    if(((cmd52->address > (0xFFFFFFFF)) ||((!cmd52->master) && ((cmd52->address) > (0xFFFF))))) {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Please provide (32-bit) SDIO adress [0x0 - 0xFFFFFFFF] in Master mode, or 16-bit SDIO adress [0x0 - 0xFFFF] slave mode\n")));
      goto FREE;
    }

    if(sdio_fnx_cmd53(adapter, cmd52->address, &cmd52->data, cmd52->function_no, cmd52->write)){
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("SDIO CMD52 FAILED \n")));
      kfree(cmd);
      kfree(cmd52);
      return -EFAULT;
    }
    if(!cmd52->write) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("R5 = %x \n",cmd52->data));
    }
FREE:
    kfree(cmd52);
  } else if((*(uint16*)cmd == 0x3335)) {  //CMD53
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("CMD53 \n")));


    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    Command = (char*)kzalloc(len,flag);
    if(Command == NULL){
      kfree(cmd);
      return -ENOMEM;
    }

    memcpy(Command,cmd,len);
    Command = strstrip(Command);

    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    cmd53= (cmd53_t*) kzalloc((sizeof(cmd53_t)),flag);
    if(cmd53 == NULL){
      kfree(cmd);
      kfree(Command);
      return -ENOMEM;
    }


    offset = strsep(&Command," ");
    if(offset != NULL)
      cmd53->master= simple_strtol(offset,NULL,10);


    offset = strsep(&Command," ");
    if(offset != NULL)
      cmd53->address= simple_strtol(offset,NULL,16);


    offset = strsep(&Command," ");
    if(offset != NULL)
      cmd53->no_of_bytes= simple_strtol(offset,NULL,10);


    offset = strsep(&Command," ");
    if(offset != NULL)
      cmd53->read_write= simple_strtol(offset,NULL,10);

    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s Master = %d \n"),__func__,cmd53->master));
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s address = %x \n"),__func__,cmd53->address));
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s no_of_bytes = %d \n"),__func__,cmd53->no_of_bytes));
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s R/w = %d \n"),__func__,cmd53->read_write));

   if(cmd53->no_of_bytes)
   {
     flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
     ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("CMD53 M FLAG = %x \n"),flag));
     Data = (uint8*) kzalloc(cmd53->no_of_bytes,flag);

     if(Data == NULL){
       kfree(cmd);
       kfree(Command);
       return -ENOMEM;
     }
   }
    
   if(cmd53->read_write){
    for(i=0;i<cmd53->no_of_bytes;i++){
     temp = strsep(&Command," ");
     if(temp == NULL){
       break;
     }
      Data[i]= simple_strtol(temp,NULL,16);
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s Data[%d] =%x \n"),__func__,i,Data[i]));
    }
    cmd53->no_of_bytes = i+1;
   }
    
    
    if(cmd53->master)
      cmd53->address |= SD_REQUEST_MASTER;

    if(sdio_fnx_cmd53(adapter, cmd53->address, Data, cmd53->read_write,cmd53->no_of_bytes)){
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("SDIO cmd53 FAILED \n")));
      kfree(cmd);
      kfree(cmd53);
      kfree(offset);
      return -EFAULT;
    }

   if(!cmd53->read_write){
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT( "Data ")));
    for(i=0;i<cmd53->no_of_bytes;i++){
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("0x%x"),Data[i]));
    }
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\n")));
   } 
   
   if(!cmd53->read_write){
    seq_printf((struct seq_file*)filp->private_data,"CMD53 Response \n");
    for(i=0;i<cmd53->no_of_bytes;i++){
    seq_printf((struct seq_file*)filp->private_data," %02x ",cmd52->data);
    }
    seq_printf((struct seq_file*)filp->private_data,"\n");
   }
   
    
     kfree(Command); 
     kfree(cmd53); 
     kfree(Data); 
  }
  kfree(cmd);

  return len;
}

int sdio_cmd_usage (struct seq_file *seq, void *data)
{
  seq_printf(seq,"echo 52 1 1 0 0 > /proc/onebox-hal/sdio_cmds ");
	seq_printf(seq,"Usage: cmd52 \n @ Function_No [SDIO FN.NO[0-5] Default : 0 ] \n @ Master_slave operation[1/0] \n @ Address [32 bit of address in Master mode ; 16-bit in slave mode ] \n @ SDIO-WRITE/READ [1/0] @ Data [Provide Upto 8 bit Data [HEX] for SDIO write operation] \n");
  seq_printf(seq,"Usage: cmd53 \n @ Master \n @ Address [32 bit of address[HEX] ] \n @ no_of_bytes\n @ SDIO-Read/Write [0/1] \n @ Data [ HEX ] \n");
	return 0;
}


int onebox_proc_sdio_cmd_usage (struct inode *inode, struct file *file)
{

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	return single_open(file,
			   sdio_cmd_usage,
			   PDE_DATA(inode));
#else

	return single_open(file,
			   sdio_cmd_usage,
			   PDE(inode)->data);
#endif
}
#endif
int onebox_reg_value (struct seq_file *seq, void *data)
{
  PONEBOX_ADAPTER adapter =  (ONEBOX_ADAPTER*)seq->private;
  seq_printf(seq,"reg_value =0x%02x \n",adapter->reg_value);
  adapter->reg_value = 0xffffffff;
	return 0;
}
int onebox_proc_master_reg (struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	return single_open(file,
			   onebox_reg_value,
			   PDE_DATA(inode));
#else
	return single_open(file,
			   onebox_reg_value,
			   PDE(inode)->data);
#endif
}

int onebox_read_gpio_state (struct seq_file *seq, void *data)
{
  PONEBOX_ADAPTER adapter =  (ONEBOX_ADAPTER*)seq->private;
  seq_printf(seq,"gpio_reg =0x%02x gpio_state =%d \n",adapter->gpio_state,((adapter->gpio_state & 0x2000) >> 13));
  adapter->gpio_state = 0xffff;
	return 0;
}

int onebox_proc_gpio_read (struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	return single_open(file,
			   onebox_read_gpio_state,
			   PDE_DATA(inode));
#else
	return single_open(file,
			   onebox_read_gpio_state,
			   PDE(inode)->data);
#endif
}


int onebox_read_modem_pll (struct seq_file *seq, void *data)
{
  PONEBOX_ADAPTER adapter =  (ONEBOX_ADAPTER*)seq->private;
  seq_printf(seq,"modem_pll_reg =0x%02x \n",adapter->modem_pll_reg);
  adapter->modem_pll_reg = 0xffff;
	return 0;
}

int onebox_proc_modem_pll_read (struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	return single_open(file,
			   onebox_read_modem_pll,
			   PDE_DATA(inode));
#else
	return single_open(file,
			   onebox_read_modem_pll,
			   PDE(inode)->data);
#endif
}

int onebox_read_soc_pll (struct seq_file *seq, void *data)
{
  PONEBOX_ADAPTER adapter =  (ONEBOX_ADAPTER*)seq->private;
  seq_printf(seq,"soc_pll_reg =0x%02x \n",adapter->soc_pll_reg);
  adapter->soc_pll_reg = 0xffff;
	return 0;
}

int onebox_proc_soc_pll_read (struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	return single_open(file,
			   onebox_read_soc_pll,
			   PDE_DATA(inode));
#else
	return single_open(file,
			   onebox_read_soc_pll,
			   PDE(inode)->data);
#endif
}
ssize_t onebox_proc_master_cmd(struct file *filp,
					    const char __user *buff,
					    size_t len,
					    loff_t *data)
{
  char *cmd = NULL;
  uint32 i = 0;
  unsigned char *Data = NULL;
  const char *temp = NULL;
  char *offset = NULL;
  char *Command = NULL;
  char *command_p = NULL;
  m_rw_t *m_read_write = NULL;
  gfp_t flag;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE_DATA(filp->f_inode);
#else
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE(filp->f_mapping->host)->data;
#endif
  if (!len)
    return 0;
  if (len < 5)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Invalid arguments please refer Usage :\n cat /proc/oneboxX/master_read_write\n")));
    return -EINVAL;
  }


  flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
  cmd = (char*)kzalloc(len,flag);
  Command = (char *)kzalloc(len,flag); 
  if(cmd == NULL || Command == NULL)
    return -ENOMEM;
  command_p = Command;

  if (copy_from_user(cmd, (void __user *)buff, len)) {
    kfree(cmd);
    kfree(command_p);
    return -EFAULT;
  }

    memcpy(Command,cmd,len);
    Command = strstrip(Command);
    cmd[0]= simple_strtol(&cmd[0],NULL,10);


    if(cmd[0] == 1 ){

    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    m_read_write = (m_rw_t*) kzalloc((sizeof(m_rw_t)),flag);
    if(m_read_write == NULL){
      kfree(command_p);
    kfree(cmd);
      return -ENOMEM;
    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      m_read_write->read_write= simple_strtol(offset,NULL,10);

    if(m_read_write->read_write > 1 || m_read_write->read_write < 0 ) {
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -EINVAL;

    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      m_read_write->address= simple_strtol(offset,NULL,16);

    if(m_read_write->address > 0xffffffff || m_read_write->address < 0 ) {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(" Address =%d \n"),m_read_write->address));
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -EINVAL;

    }


    offset = strsep(&Command," ");
    if(offset != NULL)
      m_read_write->no_of_bytes= simple_strtol(offset,NULL,10);

    if( m_read_write->no_of_bytes < 1 || m_read_write->no_of_bytes > len) {
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -EINVAL;

    }
    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    Data = (uint8*) kzalloc(m_read_write->no_of_bytes,flag);

    if(Data == NULL){
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -ENOMEM;
    }

    for(i=0; i< m_read_write->no_of_bytes ; i++){
     temp = strsep(&Command," ");
     if(temp == NULL){
       break;
     }
      Data[i]= simple_strtol(temp,NULL,16);
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s Data[%d] =%x \n"),__func__,i,Data[i]));
    }
    m_read_write->no_of_bytes = i+1;


    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Address=%x No.of_bytes =%d \n"),__func__,m_read_write->address,m_read_write->no_of_bytes));

#ifdef USE_SDIO_INTF
	if (adapter->host_intf_type == HOST_INTF_SDIO) {
     m_read_write->address |= SD_REQUEST_MASTER;
     sdio_fnx_cmd53(adapter,m_read_write->address,Data,1,m_read_write->no_of_bytes);
  }
#endif
#ifdef USE_USB_INTF
	if (adapter->host_intf_type == HOST_INTF_USB) {
    adapter->osd_host_intf_ops->onebox_ta_write_multiple(adapter, m_read_write->address, Data, m_read_write->no_of_bytes);
	}
#endif
    kfree(cmd);
    kfree(command_p);
    kfree(m_read_write);
    kfree(Data);
    return len;
  } else if((cmd[0] == 0)) {

    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    m_read_write = (m_rw_t*) kzalloc((sizeof(m_rw_t)),flag);
    if(m_read_write == NULL){
      kfree(command_p);
    kfree(cmd);
      return -ENOMEM;
    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      m_read_write->read_write= simple_strtol(offset,NULL,10);

    if(m_read_write->read_write > 1 || m_read_write->read_write < 0 ) {
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -EINVAL;

    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      m_read_write->address= simple_strtol(offset,NULL,16);

    if(m_read_write->address > 0xffffffff || m_read_write->address < 0 ) {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT(" Address =%d \n"),m_read_write->address));
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -EINVAL;

    }


    offset = strsep(&Command," ");
    if(offset != NULL)
      m_read_write->no_of_bytes= simple_strtol(offset,NULL,10);

    if( m_read_write->no_of_bytes < 1 || m_read_write->no_of_bytes > len) {
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -EINVAL;

    }
    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    Data = (uint8*) kzalloc(m_read_write->no_of_bytes,flag);

    if(Data == NULL){
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -ENOMEM;
    }

    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Address=%x No.of_bytes =%d \n"),__func__,m_read_write->address,m_read_write->no_of_bytes));
#ifdef USE_SDIO_INTF
	if (adapter->host_intf_type == HOST_INTF_SDIO) {
     m_read_write->address |= SD_REQUEST_MASTER;
     sdio_fnx_cmd53(adapter,m_read_write->address, Data,0,m_read_write->no_of_bytes);
  }
#endif
#ifdef USE_USB_INTF
	if (adapter->host_intf_type == HOST_INTF_USB) {
    adapter->osd_host_intf_ops->onebox_ta_read_multiple(adapter, m_read_write->address, Data, m_read_write->no_of_bytes);
	}
#endif
  adapter->reg_value = *(uint32*)Data;
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("REG_VALUE: 0x")));
    for(i=0; i< m_read_write->no_of_bytes ; i++){
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%x"),Data[i]));
    }
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\n")));
	kfree(Data);
    kfree(cmd);
    kfree(command_p);
    return len;

  } else{
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("INVALID CMD %d [0- For Read ;1- For Write] \n"),cmd[0]));
    kfree(cmd);
    kfree(command_p);
    return -EINVAL;
  }
}


ONEBOX_STATUS onebox_map_ulp_gpio_to_ta_gpio ( struct driver_assets *d_assets,uint32 gpio_idx )
{


    uint32 Data = 0;
    uint32 buf = 0;
    uint32 address = 0;

    buf = UNGATE_CLOCK; 
    if( d_assets->onebox_common_master_reg_write(d_assets, M4_CLK_ADDR,buf, 4) < 0 ) 
        return ONEBOX_STATUS_FAILURE;

   address =  (EGPIO_BASE_ADDR + (10 * (gpio_idx - 64)));
   Data = 6 << 2; //GPIO MODE 6 Converts ULP_GPIO to TA_GPIO
    if( d_assets->onebox_common_master_reg_write(d_assets,address,Data, 2) < 0 ) 
        return ONEBOX_STATUS_FAILURE;

    buf = GATE_CLOCK; 
    if( d_assets->onebox_common_master_reg_write(d_assets, M4_CLK_ADDR, buf, 4) < 0 ) 
        return ONEBOX_STATUS_FAILURE;

    return ONEBOX_STATUS_SUCCESS;
}

ssize_t onebox_proc_gpio_config(struct file *filp,
					    const char __user *buff,
					    size_t len,
					    loff_t *data)
{
  char *cmd = NULL;
  uint16 Data = 0;
  char *offset = NULL;
  char *Command = NULL;
  char *command_p = NULL;
  gpio_reg_t *gpio_reg = NULL;
  gfp_t flag;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE_DATA(filp->f_inode);
#else
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE(filp->f_mapping->host)->data;
#endif
  if (!len)
    return 0;

  if (len < 1)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Invalid arguments please refer Usage \n")));
    return -EINVAL;
  }


  flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
  cmd = (char*)kzalloc(len,flag);
  Command = (char *)kzalloc(len,flag); 
  if(cmd == NULL || Command == NULL)
    return -ENOMEM;
  command_p = Command;

  if (copy_from_user(cmd, (void __user *)buff, len)) {
    kfree(cmd);
    kfree(command_p);
    return -EFAULT;
  }

    memcpy(Command,cmd,len);
    Command = strstrip(Command);
    cmd[0]= simple_strtol(&cmd[0],NULL,10);


    if(cmd[0] == 1 ){                            // GPIO_WRITE

    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    gpio_reg = (gpio_reg_t*) kzalloc((sizeof(gpio_reg_t)),flag);
    if(gpio_reg == NULL){
      kfree(command_p);
    kfree(cmd);
      return -ENOMEM;
    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      gpio_reg->read_write= simple_strtol(offset,NULL,10);

    if(gpio_reg->read_write > 1 || gpio_reg->read_write < 0 ) {
      kfree(cmd);
      kfree(command_p);
      kfree(gpio_reg);
      return -EINVAL;

    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      gpio_reg->id= simple_strtol(offset,NULL,10);

    if(gpio_reg->id < 0 || gpio_reg->address > 79 ) {
      kfree(cmd);
      kfree(command_p);
      kfree(gpio_reg);
      return -EINVAL;

    }


    offset = strsep(&Command," ");
    if(offset != NULL)
      gpio_reg->mode= simple_strtol(offset,NULL,10);

    if( gpio_reg->mode < 0 || gpio_reg->mode > 7) {
      kfree(cmd);
      kfree(command_p);
      kfree(gpio_reg);
      return -EINVAL;

    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      gpio_reg->value= simple_strtol(offset,NULL,10);

    if( gpio_reg->value < 0 || gpio_reg->value > 1) {
      kfree(cmd);
      kfree(command_p);
      kfree(gpio_reg);
      return -EINVAL;

    }

    if(gpio_reg->id >= 64 ) {
        if (onebox_map_ulp_gpio_to_ta_gpio(adapter->d_assets, gpio_reg->id) < ONEBOX_STATUS_SUCCESS) {
            len = -EFAULT;
            goto END;
        }
    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      gpio_reg->direction= simple_strtol(offset,NULL,10);

    if( gpio_reg->direction < 0 || gpio_reg->direction > 1) {
      kfree(cmd);
      kfree(command_p);
      kfree(gpio_reg);
      return -EINVAL;

    }


#define GPIO_BASE_ADDR            0x40200000
#define GPIO_REG(ID)              (GPIO_BASE_ADDR + ((ID) * 2))
//GPIO_REG(ID) = gpio_mode | (value << 4) | (direction << 5
    gpio_reg->address = GPIO_REG(gpio_reg->id);
    Data = (gpio_reg->mode|gpio_reg->value << 4 |gpio_reg->direction << 5);
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Address=%x Data =%x \n"),__func__,gpio_reg->address,Data));



#ifdef USE_SDIO_INTF
	if (adapter->host_intf_type == HOST_INTF_SDIO) {
     gpio_reg->address |= SD_REQUEST_MASTER;
     sdio_fnx_cmd53(adapter,gpio_reg->address,(uint8*)&Data,1,2);
  }
#endif
#ifdef USE_USB_INTF
	if (adapter->host_intf_type == HOST_INTF_USB) {
    adapter->osd_host_intf_ops->onebox_ta_write_multiple(adapter, gpio_reg->address,(uint8*)&Data, 2);
	}
#endif
  } else if((cmd[0] == 0)) {              // GPIO_READ

    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    gpio_reg = (gpio_reg_t*) kzalloc((sizeof(gpio_reg_t)),flag);
    if(gpio_reg == NULL){
      kfree(command_p);
    kfree(cmd);
      return -ENOMEM;
    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      gpio_reg->read_write= simple_strtol(offset,NULL,10);

    if(gpio_reg->read_write > 1 || gpio_reg->read_write < 0 ) {
      kfree(cmd);
      kfree(command_p);
      kfree(gpio_reg);
      return -EINVAL;

    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      gpio_reg->id= simple_strtol(offset,NULL,10);

    if(gpio_reg->id < 0 || gpio_reg->id > 79 ) {
      kfree(cmd);
      kfree(command_p);
      kfree(gpio_reg);
      return -EINVAL;

    }

    gpio_reg->address = GPIO_REG(gpio_reg->id);
     
    if(gpio_reg->id >= 64 ) {
        if (onebox_map_ulp_gpio_to_ta_gpio(adapter->d_assets, gpio_reg->id) < ONEBOX_STATUS_SUCCESS) {
            len = -EFAULT;
            goto END;
        }
    }

#ifdef USE_SDIO_INTF
	if (adapter->host_intf_type == HOST_INTF_SDIO) {
     gpio_reg->address |= SD_REQUEST_MASTER;
     sdio_fnx_cmd53(adapter,gpio_reg->address,(uint8*)&Data,0,2);
  }
#endif
#ifdef USE_USB_INTF
	if (adapter->host_intf_type == HOST_INTF_USB) {
    adapter->osd_host_intf_ops->onebox_ta_read_multiple(adapter, gpio_reg->address,(uint8*)&Data, 2);
	}
#endif
  adapter->gpio_state = Data;

  } else{
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("INVALID CMD %d [0- For Read ;1- For Write] \n"),cmd[0]));
    kfree(cmd);
    kfree(command_p);
    return -EINVAL;
  }
END:
    kfree(cmd);
    kfree(command_p);
    kfree(gpio_reg);
    return len;
}

ssize_t onebox_proc_modem_pll_config(struct file *filp,
					    const char __user *buff,
					    size_t len,
					    loff_t *data)
{
  char *cmd = NULL;
  int Data = 0;
  int address = 0;
  char *offset = NULL;
  char *Command = NULL;
  char *command_p = NULL;
  m_rw_t *m_read_write = NULL;
  gfp_t flag;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE_DATA(filp->f_inode);
#else
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE(filp->f_mapping->host)->data;
#endif
  if (!len)
    return 0;

  if (len < 1)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Invalid arguments please refer Usage :\n cat /proc/oneboxX/master_read_write\n")));
    return -EINVAL;
  }


  flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
  cmd = (char*)kzalloc(len,flag);
  Command = (char *)kzalloc(len,flag); 
  if(cmd == NULL || Command == NULL)
    return -ENOMEM;
  command_p = Command;

  if (copy_from_user(cmd, (void __user *)buff, len)) {
    kfree(cmd);
    kfree(command_p);
    return -EFAULT;
  }

    memcpy(Command,cmd,len);
    Command = strstrip(Command);
    cmd[0]= simple_strtol(&cmd[0],NULL,10);


    if(cmd[0] == 1 ){

    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    m_read_write = (m_rw_t*) kzalloc((sizeof(m_rw_t)),flag);
    if(m_read_write == NULL){
      kfree(command_p);
    kfree(cmd);
      return -ENOMEM;
    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      m_read_write->read_write= simple_strtol(offset,NULL,10);

    if(m_read_write->read_write > 1 || m_read_write->read_write < 0 ) {
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -EINVAL;

    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      address= simple_strtol(offset,NULL,16);

    if(address < 0 || address > 0x2000 ) {
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -EINVAL;

    }


    offset = strsep(&Command," ");
    if(offset != NULL)
      Data= simple_strtol(offset,NULL,16);

    if( Data < 0 || Data > 0xFFFF) {
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -EINVAL;

    }

#define MODEM_PLL_CONFIG_IO_BASE_ADDR 0x41138000
/* Driver executes 
 * MODEM_PLL_WRITE(ADDR,DATA) *(volatile uint16 *)(MODEM_PLL_CONFIG_IO_BASE_ADDR + ( ADDR << 2 )) = DATA 
 */

    m_read_write->address = (MODEM_PLL_CONFIG_IO_BASE_ADDR + (address * 4));

#ifdef USE_SDIO_INTF
	if (adapter->host_intf_type == HOST_INTF_SDIO) {
     m_read_write->address |= SD_REQUEST_MASTER;
     sdio_fnx_cmd53(adapter,m_read_write->address,(uint8*)&Data,1,2);
  }
#endif
#ifdef USE_USB_INTF
	if (adapter->host_intf_type == HOST_INTF_USB) {
    adapter->osd_host_intf_ops->onebox_ta_write_multiple(adapter, m_read_write->address,(uint8*)&Data, 2);
	}
#endif

    kfree(cmd);
    kfree(command_p);
    kfree(m_read_write);
    return len;
  } else if((cmd[0] == 0)) {

    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    m_read_write = (m_rw_t*) kzalloc((sizeof(m_rw_t)),flag);
    if(m_read_write == NULL){
      kfree(command_p);
    kfree(cmd);
      return -ENOMEM;
    }

    offset = strsep(&Command," ");
    if(offset != NULL)
      m_read_write->read_write= simple_strtol(offset,NULL,10);

    if(m_read_write->read_write > 1 || m_read_write->read_write < 0 ) {
      kfree(cmd);
      kfree(command_p);
      kfree(m_read_write);
      return -EINVAL;

    }


    offset = strsep(&Command," ");
    if(offset != NULL)
      address= simple_strtol(offset,NULL,16);

    printk(" address1=%x \n",address);
    m_read_write->address = (MODEM_PLL_CONFIG_IO_BASE_ADDR + (address * 4));

#ifdef USE_SDIO_INTF
	if (adapter->host_intf_type == HOST_INTF_SDIO) {
     m_read_write->address |= SD_REQUEST_MASTER;
     sdio_fnx_cmd53(adapter,m_read_write->address,(uint8*)&Data,0,2);
  }
#endif
#ifdef USE_USB_INTF
	if (adapter->host_intf_type == HOST_INTF_USB) {
    printk(" address=%x \n",m_read_write->address);
    adapter->osd_host_intf_ops->onebox_ta_read_multiple(adapter, m_read_write->address,(uint8*)&Data, 2);
	}
#endif
  ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Data =%x \n"),Data));
  adapter->reg_value = Data;

    kfree(cmd);
    kfree(command_p);
    kfree(m_read_write);
    return len;

  } else{
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("INVALID CMD %d [0- For Read ;1- For Write] \n"),cmd[0]));
    kfree(cmd);
    kfree(command_p);
    return -EINVAL;
  }
}



ssize_t onebox_proc_soc_pll_config(struct file *filp,
					    const char __user *buff,
					    size_t len,
					    loff_t *data)
{
  char *cmd = NULL;
  unsigned int Data = 0;
  unsigned int buf = 0;
  int address = 0;
  char *offset = NULL;
  char *Command = NULL;
  char *command_p = NULL;
  m_rw_t *m_read_write = NULL;
  gfp_t flag;
  struct driver_assets *d_assets = NULL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE_DATA(filp->f_inode);
#else
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE(filp->f_mapping->host)->data;
#endif

  if (!len)
    return 0;

  if (len < 1)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Invalid arguments please refer Usage :\n cat /proc/oneboxX/master_read_write\n")));
    return -EINVAL;
  }

  d_assets = adapter->d_assets;
  flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
  cmd = (char*)kzalloc(len,flag);
  Command = (char *)kzalloc(len,flag); 
  if(cmd == NULL || Command == NULL)
    return -ENOMEM;
  command_p = Command;

  if (copy_from_user(cmd, (void __user *)buff, len)) {
    kfree(cmd);
    kfree(command_p);
    return -EFAULT;
  }

    memcpy(Command,cmd,len);
    Command = strstrip(Command);
    cmd[0]= simple_strtol(&cmd[0],NULL,10);


    if(cmd[0] == 1 ){

            flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
            m_read_write = (m_rw_t*) kzalloc((sizeof(m_rw_t)),flag);
            if(m_read_write == NULL){
                    kfree(command_p);
                    kfree(cmd);
                    return -ENOMEM;
            }

            offset = strsep(&Command," ");
            if(offset != NULL)
                    m_read_write->read_write= simple_strtol(offset,NULL,10);

            if(m_read_write->read_write > 1 || m_read_write->read_write < 0 ) {
                    kfree(cmd);
                    kfree(command_p);
                    kfree(m_read_write);
                    return -EINVAL;

            }

            offset = strsep(&Command," ");
            if(offset != NULL)
                    address= simple_strtol(offset,NULL,16);

            offset = strsep(&Command," ");
            if(offset != NULL)
                    Data= simple_strtol(offset,NULL,16);

            if( Data < 0 || Data > 0xFFFF) {
                    kfree(cmd);
                    kfree(command_p);
                    kfree(m_read_write);
                    return -EINVAL;

            }

#define REG_SPI_BASE_ADDR_PLL             0x46180000
#define SPI_MEM_MAP_PLL(REG_ADR)          (REG_SPI_BASE_ADDR_PLL + 0x00008000 + (REG_ADR  << 2))
#define MODEM_PLL_CONFIG_IO_BASE_ADDR 0x41138000


            m_read_write->address = SPI_MEM_MAP_PLL(address); 
            buf = UNGATE_CLOCK; 

            if( d_assets->onebox_common_master_reg_write(d_assets, M4_CLK_ADDR,buf, 4) < 0 ) 
              goto FAILED1;

            if( d_assets->onebox_common_master_reg_write(d_assets, m_read_write->address,Data, 2) < 0 ) //SOC_PLL_WRITE
              goto FAILED1;

            buf = GATE_CLOCK; 
            if( d_assets->onebox_common_master_reg_write(d_assets, M4_CLK_ADDR, buf, 4) < 0 ) 
              goto FAILED1;

            kfree(cmd);
            kfree(command_p);
            kfree(m_read_write);
            return len;
    } else if((cmd[0] == 0)) {

            flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
            m_read_write = (m_rw_t*) kzalloc((sizeof(m_rw_t)),flag);
            if(m_read_write == NULL){
                    kfree(command_p);
                    kfree(cmd);
                    return -ENOMEM;
            }

            offset = strsep(&Command," ");
            if(offset != NULL)
                    m_read_write->read_write= simple_strtol(offset,NULL,10);

            if(m_read_write->read_write > 1 || m_read_write->read_write < 0 ) {
                    kfree(cmd);
                    kfree(command_p);
                    kfree(m_read_write);
                    return -EINVAL;

            }


            offset = strsep(&Command," ");
            if(offset != NULL)
                    address= simple_strtol(offset,NULL,16);

            if(address < 0 || address > 0x2000 ) {
                    kfree(cmd);
                    kfree(command_p);
                    kfree(m_read_write);
                    return -EINVAL;

            }


            m_read_write->address = SPI_MEM_MAP_PLL(address); /*??*/

            buf = UNGATE_CLOCK; 
            if( d_assets->onebox_common_master_reg_write(d_assets, M4_CLK_ADDR,buf, 4) < 0 ) 
              goto FAILED1;

            if( d_assets->onebox_common_master_reg_read(d_assets, m_read_write->address, &Data, 2) < 0 ) //SOC_PLL_READ
              goto FAILED1;

            adapter->reg_value = Data;

            buf = GATE_CLOCK; 
            if( d_assets->onebox_common_master_reg_write(d_assets, M4_CLK_ADDR,buf, 4) < 0 ) 
              goto FAILED1;

    kfree(cmd);
    kfree(command_p);
    kfree(m_read_write);
    return len;
    } else {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("INVALID CMD %d [0- For Read ;1- For Write] \n"),cmd[0]));
FAILED1:
    kfree(cmd);
    kfree(command_p);
    return -EFAULT;
  }
}
