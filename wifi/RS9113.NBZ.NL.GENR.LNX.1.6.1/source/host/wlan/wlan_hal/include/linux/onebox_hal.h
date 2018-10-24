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

#ifndef __ONEBOX_HAL_H__
#define __ONEBOX_HAL_H__

#include "wlan_common.h"

#define LMAC_INSTRUCTIONS_SIZE          (16  * 1024) /* 16Kbytes */
#define FRAME_DESC_SZ                    16
#define OFFSET_EXT_DESC_SIZE    		 4
#define ONEBOX_DESCRIPTOR_SZ             160
#define ONEBOX_EXTENDED_DESCRIPTOR       12
#define ONEBOX_SD_REQUEST_MASTER         0x10000

#define ONEBOX_SDIO_INTERRUPT_ENABLE_REGISTER   0x04
#define ONEBOX_VENDOR_SPECIFIC_REGISTER_1       0xf1
#define ONEBOX_DEVICE_BUFFER_STATUS_REGISTER    0xf3
#define ONEBOX_NEXT_READ_DELAY_REGISTER_1       0xf4
#define ONEBOX_NEXT_READ_DELAY_REGISTER_2       0xf5
#define ONEBOX_VERSION_NO                       0xf8
#define ONEBOX_FUNCTION_1_INTERRUPT_REGISTER    0xf9
#define ONEBOX_RFIFO_START_LEVEL_REGISTER       0xfc
#define ONEBOX_RFIFO_AFULL_LEVEL_REGISTER       0xfd
#define ONEBOX_WFIFO_AEMPTY_LEVEL_REGISTER      0xfe
#define ONEBOX_WAKEUP_REGISTER                  0xff
#define SDIO_MASTER_ACCESS_MSBYTE               0x000FA
#define SDIO_MASTER_ACCESS_LSBYTE               0x000FB

#define ONEBOX_DESC_QUEUE_NUM_MASK  0x7

/* Interrupt Bit Related Macros */
#define SD_PKT_BUFF_SEMI_FULL         0
#define SD_PKT_BUFF_FULL              1
#define SD_PKT_MGMT_BUFF_FULL         2
#define SD_VI_STARVING                4
#define SD_VO_STARVING                5
#define SD_BE_STARVING                6
#define SD_BK_STARVING                7


#define SD_BEACON_INTERUPT            0
#define SD_BUFF_STATUS_UPDATE         1
#define SD_FW_ASSERT_IND              2
#define SD_MSDU_PACKET_PENDING        3

#define IEEE80211_IS_MODE_BEACON(_opmode) \
        ((_opmode == IEEE80211_M_IBSS) || \
         (_opmode == IEEE80211_M_HOSTAP))
/************************** END DEFINE CONSTANTS *************************/

typedef enum 
{
	BUFFER_FULL         = 0x0,
	BEACON_INTERRUPT    = 0x1,
	BUFF_STATUS_UPDATE  = 0x2,
	FIRMWARE_ASSERT_IND = 0x3,
	MSDU_PACKET_PENDING = 0x4,
	SLEEP_INDCN         = 0X5,
	WAKEUP_INDCN        = 0x6,
	SEMI_BUFFER_FULL    = 0x7,
	MGMT_BUFFER_FULL    = 0x8,
	VI_STARTVING        = 0X9,
	VO_STARTVING        = 0xA,
	BE_STARTVING        = 0xB,
	BK_STARTVING        = 0xC,
	UNKNOWN_INT         = 0XE
} SDIO_INTERRUPT_TYPE;

/************************* START DEFINE MACROS ***************************/

#define ONEBOX_GET_SDIO_INTERRUPT_TYPE(_I) \
	(_I & (1 << SD_BEACON_INTERUPT)) ?\
	BEACON_INTERRUPT: \
	(_I & (1 << SD_BUFF_STATUS_UPDATE)) ?\
	BUFF_STATUS_UPDATE: \
	(_I & (1 << SD_MSDU_PACKET_PENDING)) ? \
	MSDU_PACKET_PENDING : \
	(_I & (1 << SD_FW_ASSERT_IND )) ? \
	FIRMWARE_ASSERT_IND: UNKNOWN_INT

#define NUM_OF_RX_HAL_DATA_DESC    16

/* Rcv buffer size needs to be updated :
 */
#define ONEBOX_RCV_BUFFER_LEN      2000

#define ONEBOX_TRUE  1
#define ONEBOX_FALSE 0

/* TX Data Frame Descitptor Fields */
//DWord 0
#define HOST_HAL_TXDESC_TID_MSK         0xF0000000 /* Bits 28:31 */
#define HOST_HAL_TXDESC_TID_OFST        28
#define HOST_HAL_TXDESC_TID_FIELD_VALID  ONEBOX_BIT(26)  /* Bit 26 */

#endif
