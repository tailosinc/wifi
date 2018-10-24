/**
 * @file      rsi_linux_apis.c
 * @version   1.2
 * @date      2013-Feb-12
 *
 * Copyright(C) Redpine Signals 2013
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief Implements common functionality for all the commands
 *
 * @section Description
 * This file contains common api needed for all the commands
 *
 *
 */

/**
 * Includes
 */
#include "rsi_global.h"
#include "rsi_nl_app.h"
#include "rsi_api.h"
#include "rsi_wireless_copy.h"
#include "rsi_app.h"
#include <sys/ioctl.h>

/**
 * Function Declarations
 */
int32 rsi_ioctl_send_req(int32 sockfd, uint8 *buf, int32 buf_len, int32 req_type);


/*==================================================*/
/**
 * @fn          void rsi_buildFrameDescriptor(rsi_uFrameDsc *uFrameDscFrame, uint8 *cmd)
 * @brief       Creates a Frame Descriptor
 * @param[in]   rsi_uFrameDsc *uFrameDscFrame,Frame Descriptor
 * @param[in]   uint8 *cmd,Indicates type of the packet(data or management)
 * @param[out]  none
 * @return      none
 */
void rsi_buildFrameDescriptor(rsi_uFrameDsc *uFrameDscFrame, uint8 *cmd)
{
  uint8 i;
#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_buildFrameDescriptor:\n");
#endif
  for (i = 0; i < RSI_FRAME_DESC_LEN; i++)
  {
    uFrameDscFrame->uFrmDscBuf[i] = 0x00;
  }       //! zero the frame descriptor buffer
  //! data or management frame type
  //! uFrameDscFrame->uFrmDscBuf[14] = cmd[2];
  //! The first two bytes have different functions for management frames and
  //! control frames, but these are set in the pre-defined
  uFrameDscFrame->uFrmDscBuf[0] = cmd[0];
  //! arrays which are the argument passed to this function, so we just set the two values
  uFrameDscFrame->uFrmDscBuf[1] = cmd[1];
  uFrameDscFrame->uFrmDscBuf[2] = cmd[2] ;

  return;
}


/*==============================================*/
/**
 * @fn             uint8 rsi_Buffer_isFull(void)
 * @brief          Checks the buffer status of the module
 * @param[in]      none
 * @param[out]     none
 * @return         errCode
 *                 0  = Not Buffer full
 *                 1  = Buffer full
 *                 else FAILURE
 * @section description
 * This function is used to send some info which is required
 * by the kernel netdevice driver.
 */

int32 rsi_buffer_isFull(void)
{
  int32 retVal;
  rsi_linux_app_cb_t *linux_app_cbPtr = &rsi_linux_app_cb;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_buffer_isFull:\n");
#endif
  retVal = rsi_ioctl_send_req(linux_app_cbPtr->ioctl_sd,(uint8 *)&linux_app_cbPtr->rsi_status, sizeof(uint8), OID_WSC_GET_STATUS);
  if(retVal == 0)
  {
    if(linux_app_cbPtr->rsi_status & BUFFER_FULL)
      retVal = 1;
    else if(linux_app_cbPtr->rsi_status == RSI_BUSY)
      retVal = 1;
    else if(linux_app_cbPtr->rsi_status == RSI_FAIL)
      retVal = 1;
    else
      retVal = 0;
  }
  return retVal;
}

#ifdef WLAN_ENABLE
/*====================================================*/
/**
 * @fn          int16 rsi_execute_cmd(uint8 *descparam,uint8 *payloadparam,uint16 size_param)
 * @brief       Common function for all the commands.
 * @param[in]   uint8 *descparam, pointer to the frame descriptor parameter structure
 * @param[in]   uint8 *payloadparam, pointer to the command payload parameter structure
 * @param[in]   uint16 size_param, size of the payload for the command
 * @return      errCode
 *              -2 = Command issue failed
 *              0  = SUCCESS
 * @section description
 * This is a common function used to process a command to the Wi-Fi module.
 */

int16 rsi_execute_cmd(uint8 *descparam, uint8 *payloadparam, uint16 size_param)
{
  int16                                         retval = 0;
  rsi_uFrameDsc         uFrameDscFrame;
  uint8           *cmd_buff;
  rsi_app_cb_t *rsi_app = &rsi_app_cb;
#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_execute_cmd:\n");
#endif

#if (RSI_POWER_MODE == RSI_POWER_MODE_3)
  if (!rsi_app_cb.ps_ok_to_send)
  {
#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"module in sleep; Enquing\n");
#endif
    memcpy(rsi_app_cb.ps_descparam, descparam, 16);
    rsi_app_cb.ps_pkt_pending = payloadparam;
    rsi_app_cb.ps_size_param  = size_param;
    return 0;
  }
#endif
#ifdef SPI_INTERFACE
  while(rsi_buffer_isFull());
#endif

  //! Build 16 bytes, send/receive command descriptor frame
  rsi_buildFrameDescriptor(&uFrameDscFrame,descparam);

  cmd_buff = rsi_alloc_and_init_cmdbuff((uint8 *)&uFrameDscFrame,payloadparam,size_param);

#ifdef ENABLE_DEBUG_PRINTS
    uint16 length =  *(uint16 *)descparam & 0xFFF; 
    uint16 i = 0;
    RSI_DPRINT(RSI_PL0," (Length of Command packet: %d)\n", (length + 16));
    for (i=0; i<(length + 16); i++) {
        RSI_DPRINT (RSI_PL0, "0x%02x ", cmd_buff[i+24]);
        if (((i - 0)+1)%16 == 0) {
            RSI_DPRINT(RSI_PL0, "\n");
        }
    }
    RSI_DPRINT(RSI_PL0, "\n");
#endif

  if(rsi_send_usr_cmd(cmd_buff, GET_SEND_LENGTH(cmd_buff)) < 0)
  {
    retval = -2;
  }
  if(retval < 0)
  {
    RSI_DPRINT(RSI_PL13,"Unable to issue command\n");
  }
  rsi_app->expected_response = descparam[2];
  
  //! Free the command buffer
  rsi_free(cmd_buff);

  return retval;
}

/*==============================================*/
/**
 * @fn             int16 rsi_update_info(type)
 * @brief          update the info to kernel
 * @param[in]      none
 * @param[out]     none
 * @return         errCode
 *                 -2 = Failure
 *                 0  = SUCCESS
 * @section description
 * This function is used to send some info which is required
 * by the kernel netdevice driver.
 */
int16 rsi_update_info(uint8 type)
{
  int16            retval = 0;
  uint8            payload[10];

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL3,"\nUpdate info\n");
#endif

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_update_info:\n")   ;
#endif
  switch(type)
  {
  case MODULE_POWER_CYCLE:
    payload[0] = type;
    retval =rsi_execute_cmd((uint8 *)rsi_frameCmdUpdateInfo, payload, 1 /* type */);
    break;
  case UPDATE_JOIN_DONE:
    payload[0] = type;
    memcpy(&payload[1], rsi_app_cb.mac_addr, 6);
    retval =rsi_execute_cmd((uint8 *)rsi_frameCmdUpdateInfo, payload, 7 /* 1 byte type + 6 bytes mac */);
    break;
  case UPDATE_CONCURRENT_AP_JOIN_DONE:
#if RSI_CONCURRENT_MODE
    payload[0] = type;
    memcpy(&payload[1], rsi_app_cb.ap_mac_addr, 6);
    retval =rsi_execute_cmd((uint8 *)rsi_frameCmdUpdateInfo, payload, 7 /* 1 byte type + 6 bytes mac */);
#endif
    break;
  case PS_CONTINUE:
    payload[0] = type;
    retval =rsi_execute_cmd((uint8 *)rsi_frameCmdUpdateInfo, payload, 1 /* type */);
    break;
  case WKP_FROM_HOST:
    payload[0] = type;
    retval =rsi_execute_cmd((uint8 *)rsi_frameCmdUpdateInfo, payload, 1 /* type */);
    break;
  default:
    retval = -2;
    break;
  }
  return retval;
}
#endif


/*==============================================*/
/**
 * @fn          int16 rsi_boot_insn(uint8 type, uint16 *data)
 * @brief       Sends boot instructions to WiFi module
 * @param[in]   uint8 type, type of the insruction to perform
 * @param[in]   uint16 *data, pointer to data which is to be read/write
 * @param[out]  none
 * @return      errCode
 *              < 0  = Command issued failure/Invalid command 
 *                0  = SUCCESS
 * @section description 
 * This API is used to send boot instructions to WiFi module.
 */
int16 rsi_boot_insn(uint8 type, uint16* data)
{
  rsi_linux_app_cb_t *linux_app_cbPtr = &rsi_linux_app_cb;
  int16            retval = 0;
  uint32           size = 0;

  switch (type)
  {
    case REG_READ:
    {
      size = 4;
      retval = rsi_ioctl_send_req(linux_app_cbPtr->ioctl_sd,(uint8 *)data, size, OID_WSC_BOOT_READ);
    }
    break;

    case REG_WRITE:
    {

      size = 4;
      retval = rsi_ioctl_send_req(linux_app_cbPtr->ioctl_sd,(uint8 *)data, size, OID_WSC_BOOT_WRITE);
    }
    break;
    case PING_WRITE:
    {
      size = (4 * 1024);
      retval = rsi_ioctl_send_req(linux_app_cbPtr->ioctl_sd,(uint8 *)data, size, OID_WSC_BOOT_PING_WRITE);
    }
    break;

    case PONG_WRITE:
    {
      size = (4 * 1024);
      retval = rsi_ioctl_send_req(linux_app_cbPtr->ioctl_sd,(uint8 *)data, size, OID_WSC_BOOT_PONG_WRITE);
    } 
    break;

  }
  return retval;
}

