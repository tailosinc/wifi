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

#ifndef __ONEBOX_COMMON_H__
#define __ONEBOX_COMMON_H__

#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/etherdevice.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include "onebox_datatypes.h"
#include "onebox_common.h"
#include "onebox_zone.h"

#define onebox_likely(a) likely(a)
#define USB_VENDOR_REGISTER_READ 0x15
#define USB_VENDOR_REGISTER_WRITE 0x16

#if((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))&& \
    (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)))
#include <linux/sdio/ctsystem.h>
#include <linux/sdio/sdio_busdriver.h>
#include <linux/sdio/_sdio_defs.h>
#include <linux/sdio/sdio_lib.h>
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#endif

#include "onebox_common.h"

/* Kernel version less than or equal to */
#define KERNEL_VERSION_LESS_THAN_3_6(a)  \
 (LINUX_VERSION_CODE <= KERNEL_VERSION(3,6,a)) 


#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#define ONEBOX_BIT(n)                   (1 << (n))
#define ONEBOX_ZONE_ERROR               ONEBOX_BIT(0)  /* For Error Msgs              */
#define ONEBOX_ZONE_WARN                ONEBOX_BIT(1)  /* For Warning Msgs            */
#define ONEBOX_ZONE_INFO                ONEBOX_BIT(2)  /* For General Status Msgs     */
#define ONEBOX_ZONE_INIT                ONEBOX_BIT(3)  /* For Driver Init Seq Msgs    */
#define ONEBOX_ZONE_OID                 ONEBOX_BIT(4)  /* For IOCTL Path Msgs         */
#define ONEBOX_ZONE_MGMT_SEND           ONEBOX_BIT(5)  /* For TX Mgmt Path Msgs       */
#define ONEBOX_ZONE_MGMT_RCV            ONEBOX_BIT(6)  /* For RX Mgmt Path Msgs       */
#define ONEBOX_ZONE_DATA_SEND           ONEBOX_BIT(7)  /* For TX Data Path Msgs       */
#define ONEBOX_ZONE_DATA_RCV            ONEBOX_BIT(8)  /* For RX Data Path Msgs       */
#define ONEBOX_ZONE_FSM                 ONEBOX_BIT(9)  /* For State Machine Msgs      */
#define ONEBOX_ZONE_ISR                 ONEBOX_BIT(10) /* For Interrupt Specific Msgs */
#define ONEBOX_ZONE_MGMT_DUMP           ONEBOX_BIT(11) /* For Dumping Mgmt Pkts       */
#define ONEBOX_ZONE_DATA_DUMP           ONEBOX_BIT(12) /* For Dumping Data Pkts       */
#define ONEBOX_ZONE_CLASSIFIER          ONEBOX_BIT(13) /* For Classification Msgs     */
#define ONEBOX_ZONE_PROBE               ONEBOX_BIT(14) /* For Probe Req & Rsp Msgs    */
#define ONEBOX_ZONE_EXIT                ONEBOX_BIT(15) /* For Driver Unloading Msgs   */
#define ONEBOX_ZONE_DEBUG               ONEBOX_BIT(16) /* For Extra Debug Messages    */
#define ONEBOX_ZONE_PWR_SAVE             ONEBOX_BIT(17) /* For Powersave Blk Msgs      */
#define ONEBOX_ZONE_AGGR                ONEBOX_BIT(18) /* For Aggregation Msgs        */
#define ONEBOX_ZONE_DAGGR               ONEBOX_BIT(19) /* For Deaggregation Msgs      */
#define ONEBOX_ZONE_AUTORATE            ONEBOX_BIT(20) /* For Autorate Msgs           */

#define ONEBOX_STATUS_SUCCESS       0
#define ONEBOX_STATUS_FAILURE      -1

#if 0 /* coex */
#ifdef ONEBOX_DEBUG
#define obm_dbg(__zone, __str, __arg...) \
	do { \
		if((!zone) || (zone & onebox_zone_enabled)) \
			printk(_str, __arg);
	} while(0)
#elif
#define obm_dbg(__arg) \
	((void*)0) 
#endif

#define ONEBOX_BIT(n)	(1 << (n))
#define Z_ERRO     ONEBOX_BIT(0)  /* For Error Msgs              */
#define Z_WARN     ONEBOX_BIT(1)  /* For Warning Msgs            */
#define Z_INFO     ONEBOX_BIT(2)  /* For General Status Msgs     */
#define Z_INIT     ONEBOX_BIT(3)  /* For Driver Init Seq Msgs    */
#define Z_OID      ONEBOX_BIT(4)  /* For IOCTL Path Msgs         */
#define Z_MGTX     ONEBOX_BIT(5)  /* For TX Mgmt Path Msgs       */
#define Z_MGRX
#define Z_DATX
#define Z_DARX
#define Z_FSM      ONEBOX_BIT(9)  /* For State Machine Msgs      */
#define Z_ISR      ONEBOX_BIT(10) /* For Interrupt Specific Msgs */
#define Z_MGDMP    ONEBOX_BIT(11) /* For Dumping Mgmt Pkts       */
#define Z_DADMP    ONEBOX_BIT(12) /* For Dumping Data Pkts       */
#define Z_CLAS     ONEBOX_BIT(13) /* For Classification Msgs     */
#define Z_PROB     ONEBOX_BIT(14) /* For Probe Req & Rsp Msgs    */
#define Z_EXIT     ONEBOX_BIT(15) /* For Driver Unloading Msgs   */
#define Z_DBUG     ONEBOX_BIT(16) /* For Extra Debug Messages    */
#define Z_PWRSV    ONEBOX_BIT(17) /* For Powersave Blk Msgs      */
#define Z_AGGR     ONEBOX_BIT(18) /* For Aggregation Msgs        */
#define Z_DAGGR    ONEBOX_BIT(19) /* For Deaggregation Msgs      */
#define Z_ARATE    ONEBOX_BIT(20) /* For Autorate Msgs           */

#define OB_SUCCESS       0
#define OB_FAILURE      (!0)
#endif

#define INVALID_QUEUE              0xff
#define NO_STA_DATA_QUEUES          4

#define ONEBOX_HEADER_SIZE          18 /* Size of PS_POLL */
#define TX_VECTOR_SZ                12

#define ONEBOX_TOTAL_PWR_VALS_LEN   30
#define ONEBOX_BGN_PWR_VALS_LEN     30
#define ONEBOX_AN_PWR_VALS_LEN      18

#define ONEBOX_EXTERN   extern
#define ONEBOX_STATIC   static
#define HEX_FILE       1
#define BIN_FILE       0
#define ONEBOX_SDIO_FRM_TRF_SIZE   (256 - 16)
#define WIFI_MODE_ON    1
#define RF_EVAL_MODE_ON    2
#define RF_EVAL_LPBK_CALIB 3
#define RF_EVAL_LPBK       4
#define QSPI_FLASHING   5
#define QSPI_UPDATE     6
#define SNIFFER_MODE    7
#define SWBL_FLASHING_NOSBL   8
#define SWBL_FLASHING_SBL     9
#define SWBL_FLASHING_NOSBL_FILE   10

#define FLASH_BLOCK_SIZE (32 * 1024)
#define UNIX_FILE_TYPE  8
#define DOS_FILE_TYPE   9

#define MAX_NUM_OF_SETS            15
#define  MAX_NUM_OF_BBP_VALS_PER_SET  250


typedef void (*SD_INTERRUPT) (void *pContext);

typedef int ONEBOX_STATUS;

/***************************** START MACROS ********************************/
#define ONEBOX_PRINT printk
#define ONEBOX_SPRINTF  sprintf
#ifdef ONEBOX_DEBUG_ENABLE
#define ONEBOX_DEBUG1    ONEBOX_PRINT



#define PRINT_MAC_ADDR(zone, buf) do {\
	if (zone & onebox_zone_enabled) {\
               ONEBOX_PRINT("%02x:%02x:%02x:%02x:%02x:%02x\n",\
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);\
	}\
} while (0);

#define ONEBOX_ASSERT(exp) do {\
	if (!(exp)) {\
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,\
		             ("===> ASSERTION FAILED <===\n"));\
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,\
		             ("Expression: %s\n", #exp));\
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,\
		             ("Function        : %s()\n", __FUNCTION__));\
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,\
		             ("File: %s:%d\n", __FILE__, __LINE__));\
	}\
} while (0)

#define FUNCTION_ENTRY(zone) do {\
	if (zone & onebox_zone_enabled) {\
            ONEBOX_PRINT("+%s()\n", __FUNCTION__);\
	}\
} while (0);

#define FUNCTION_EXIT(zone) do {\
	if (zone & onebox_zone_enabled) {\
             ONEBOX_PRINT("-%s()\n", __FUNCTION__);\
	}\
} while (0);
#else
 #define PRINT_MAC_ADDR(zone, buf)
 #define ONEBOX_ASSERT(exp)
 #define FUNCTION_ENTRY(zone)
 #define FUNCTION_EXIT(zone)
#endif

#ifndef DANUBE_ADDRESSING
 #define ONEBOX_HOST_VIR_TO_PHY(virt_addr)  virt_to_phys((void *)virt_addr)
 #define ONEBOX_HOST_PHY_TO_VIR(phy_addr)  phys_to_virt((uint32)phy_addr)
#else
 #define ONEBOX_HOST_VIR_TO_PHY(virt_addr) ((void *)(((uint32)virt_addr) & 0x5FFFFFFF))
 #define ONEBOX_HOST_PHY_TO_VIR(phy_addr) (((uint32)phy_addr) | 0xA0000000)
#endif

#define BB_PROG 0x00
#define RF_PROG 1

#define ONEBOX_CPU_TO_LE16(x)   cpu_to_le16(x)
#define ONEBOX_CPU_TO_LE32(x)   cpu_to_le32(x)
#define ONEBOX_CPU_TO_LE64(x)   cpu_to_le64(x)
#define ONEBOX_CPU_TO_BE16(x)   cpu_to_be16(x)

#define ONEBOX_LE16_TO_CPU(x)   le16_to_cpu(x)

#define CORE_MGMT_SOFT_QNUM 0
/* Beacon /Broadcast queues are maintained per vap ,
 * so total 8 soft queues are supported  by host */
//#define NUM_SOFT_QUEUES  6 
#define NUM_EDCA_QUEUES  8
#define MCAST_QNUM      4


#define QUEUE_DEPTH_Q0 32
#define QUEUE_DEPTH_Q1 32
#define QUEUE_DEPTH_Q2 32
#define QUEUE_DEPTH_Q3 32
#define WMM_SHORT_SLOT_TIME 9
#define SIFS_DURATION   16

/* Netbuf Control Block Flag Fields */
#define MIN_802_11_HDR_LEN              24      
#ifdef CONFIG_11W
#define MIN_802_11_HDR_LEN_MFP          32 
#endif
#define ETH_PROTOCOL_OFFSET             12
#define ETH_HDR_OFFSET                  0
#define ETH_HDR_LEN                     14

#define BK_DATA_QUEUE_WATER_MARK           600
#define BE_DATA_QUEUE_WATER_MARK           3200
#define VI_DATA_QUEUE_WATER_MARK           3900
#define VO_DATA_QUEUE_WATER_MARK           4500

#define MAX_BK_QUEUE_LEVEL          1000 
#define MAX_BE_QUEUE_LEVEL          3500
#define MAX_VI_QUEUE_LEVEL          4800
#define MAX_VO_QUEUE_LEVEL          5500

#define MAX_QUEUE_WATER_MARK		5500
#define MIN_DATA_QUEUE_WATER_MARK       1000
//#define MIN_DATA_QUEUE_WATER_MARK       1100
#define MULTICAST_WATER_MARK            500
#define MAC_80211_HDR_FRAME_CONTROL     0
#define UDP_TYPE                        17


#define EEPROM_CHECK_ADDR                  0
#define EEPROM_RFTYPE_ADDR                 73
#define EEPROM_READ_MAC_ADDR               74
#define EEPROM_2P4GHZ_GC_VALS_ADDR         80
#define EEPROM_5P1GHZ_GC_VALS_ADDR         212
#define EEPROM_2P4GHZ_MAX_TX_PWR_VALS_ADDR 450
#define EEPROM_5P1GHZ_MAX_TX_PWR_VALS_ADDR 475
#define EEPROM_5P1GHZ_GAIN_VALS_ADDR       500

/*BT specific */
#define MAX_BER_PKT_SIZE    	1030
#define GANGES_BER_QUEUE_LEN    100
/* BT device states */
#define BT_STAND_BY     		0
#define BT_INQUIRY      		1
#define BT_PAGE         		2
#define BT_INQUIRY_SCAN     	3   
#define BT_PAGE_SCAN        	4
#define BT_INQUIRY_RESP     	5   
#define BT_SLAVE_PAGE_RESP  	6
#define BT_MASTER_PAGE_RESP 	6
#define BT_CONN         		7
#define BT_PARK         		8
#define BT_SLAVE_PAGE_RESP_2    6
#define BT_MASTER_PAGE_RESP_2   6
#define BT_CLASSIC_MODE     	0
#define BT_LE_MODE      		1

#define FREQ_HOP_ENABLE     0
#define FREQ_HOP_DISABLE    1

#define Q_NUM_TO_AC(_q) (    \
         ((_q) == BK_Q_STA) ? WME_AC_BK : \
         ((_q) == BE_Q_STA) ? WME_AC_BE : \
         ((_q) == VI_Q_STA) ? WME_AC_VO : \
         WME_AC_VO)
/* zigbee queues */
//#define BT_TA_MGMT_Q            0x0
//#define ZB_INTERNAL_MGMT_Q      0x0
#define BT_TA_MGMT_Q        	  0x6
#define ZB_DATA_Q                 0x7
#define ZB_INTERNAL_MGMT_Q        0x6
#define E2E_MODE_ON                      1

/* Bluetooth Queues */
#define BT_DATA_Q       	0x7
#define BT_TA_MGMT_Q        0x6
#define BT_INT_MGMT_Q       0x6
/* EPPROM_READ_ADDRESS */

#define WLAN_MAC_EEPROM_ADDR               40 
#define WLAN_EEPROM_RFTYPE_ADDR            424
#define WLAN_MAC_MAGIC_WORD_LEN            01 
#define WLAN_HOST_MODE_LEN                 04 
#define MAGIC_WORD			   0x5A

/* Rx filter word defines */
#define PROMISCOUS_MODE         BIT(0)
#define ALLOW_DATA_ASSOC_PEER   BIT(1)
#define ALLOW_MGMT_ASSOC_PEER   BIT(2)
#define ALLOW_CTRL_ASSOC_PEER   BIT(3)
#define DISALLOW_BEACONS	BIT(4)
#define ALLOW_CONN_PEER_MGMT_WHILE_BUF_FULL BIT(5)
#define DISALLOW_BROADCAST_DATA	BIT(6)

/* sifs and slot time defines */
#define SIFS_TX_11N_VALUE 580
#define SIFS_TX_11B_VALUE 346
#define SHORT_SLOT_VALUE  360
#define LONG_SLOT_VALUE   640
#define OFDM_ACK_TOUT_VALUE 2720
#define CCK_ACK_TOUT_VALUE 9440
#define LONG_PREAMBLE    0x0000
#define SHORT_PREAMBLE    0x0001
#define REQUIRED_HEADROOM_FOR_BT_HAL	 16
#define ONEBOX_DEBUG_MESG_RECVD 1

/*Master access types*/
#define ONEBOX_MASTER_READ	11
#define ONEBOX_MASTER_WRITE	22
#define ONEBOX_MASTER_ACK	33
#ifdef RADAR_AUTO
struct radar_app
{	
    unsigned short int *ptr;
    struct radar_app *next;
};
#endif
struct ta_metadata
{
	char *name;
	unsigned int address;
};
typedef struct ta_metadata METADATA;

typedef struct
{
	uint16  major;
	uint16  minor;
	uint8 release_num;
	uint8 patch_num;
	union 
	{
		struct
		{
			uint8 fw_ver[8];
		}info;
	}ver;
}__attribute__ ((packed))version_st;


struct channel_val
{
	uint16 val[60];
	uint16 num_vals;
	uint16 vals_row;
};

/* RF Types */
typedef enum  
{ 
	ONEBOX_RF_8230 = 0,
	ONEBOX_RF_8111 
}rf_type;//core related

typedef struct config_vals
{
	uint16 internalPmuEnabled;  // 1 or 0
	uint16 ldoControlRequired;  // 1 or 0 
	uint16 crystalFrequency;      //0  9.6MHz, 1  19.2MHz, 2  38.4MHz, 4  13MHz
	                                             //5  26MHz, 6  52MHz, 8  20MHz, 9  40MHz
	uint16 crystalGoodTime;
	uint16 TAPLLEnabled;  // 1 or 0
	uint16 TAPLLFrequency;
	uint16 sleepClockSource;
	uint16 voltageControlEnabled;
	uint16 wakeupThresholdTime;
	uint16 mratio;
	uint16 inputdratio;
	uint16 outputdratio;
	uint16 host_wkup_enable;
	uint16 fr4_enable; 
	uint16 BT_coexistence;
	uint16 host_wakeup_enable;
	uint16 host_wakeup_active_high;
	uint16 Xtal_ip_enable;
	uint16 ExternalPA;
}CONFIG_VALS,*PCONFIG_VALS,BOOTUP_PARAMETERS_OLD;

//! structure to store configs related to TAPLL programming
#define RSI_CONFIG_REGISTER_LOWER  0xffff
#define RSI_CONFIG_REGISTER_UPPER (0xffff << 16)

typedef struct tapll_info_s
{
   uint32 tapll_config_register_info;
}__attribute__((__packed__)) tapll_info_t;

//! structure to store configs related to PLL960 programming
typedef struct pll960_info_s
{
	uint32 pll_config_register_1;
	uint16 pll_config_register_2;
}__attribute__((__packed__)) pll960_info_t;

//! structure to store configs related to AFEPLL programming
typedef struct afepll_info_s
{
   //! configs for pll reg 
   uint16 pll_config_register;
}__attribute__((__packed__)) afepll_info_t;

typedef struct modem_pll_info_s
{ 
                                uint16 pll_ctrl_set_reg;
                                uint16 pll_ctrl_clr_reg;
                                uint16 pll_modem_conig_reg;
                                uint16 soc_clk_config_reg;
                                uint16 adc_dac_strm1_config_reg;
                                uint16 adc_dac_strm2_config_reg;
}__attribute__((__packed__)) modem_pll_info_t;

/* structure to store configs related to pll configs for 9116 */
typedef struct pll_config_9116_s {
		modem_pll_info_t modem_pll_info_g;
}__attribute__((__packed__)) pll_config_9116_t;

/* structure to store configs related to pll configs */
typedef struct pll_config_s {
  //! tapll info is a daughter structure
  tapll_info_t tapll_info_g;
  //! pll960 info is a daughter structure
  pll960_info_t pll960_info_g;
  //! afepll info is a daughter structure
  afepll_info_t afepll_info_g;
}__attribute__((__packed__)) pll_config_t;

//! structure to store configs related to UMAC clk programming for 9116
typedef struct switch_clk_9116_s
{
  //! If set rest is valid
  uint32 switch_tass_clk : 1;
  //! If set qspi clk will be changed
  uint32 switch_qspi_clk : 1;
  //! If set sleep clk will be changed to 32MHz
  uint32 switch_slp_clk_2_32 : 1;
  //! If set program the WLAN bbp lmac clock reg val into the BBP_LMAC_CLOCK_REG
  uint32 switch_wlan_bbp_lmac_clk_reg : 1;
		//! If set program the ZB/BT BBP lmac clk reg val
		uint32 switch_zbbt_bbp_lmac_clk_reg : 1;
  //! If set config mem_ctrl for 120Mhz support
  uint32 switch_bbp_lmac_clk_reg : 1;
  //! Modem clk 160Mhz
  uint32 modem_clk_is_160mhz : 1;
  //! reserved
  uint32 reserved : 25;
		//! tass clock register 
		uint32 tass_clock_reg	:32;
		//! wlan bbp clock register
		uint32 wlan_bbp_lmac_clk_reg_val	:32;
		//!zigbee bt lmac clock register
		uint32 zbbt_bbp_lmac_clk_reg_val	:32;
		//bbp lmac clocl register
		uint32 bbp_lmac_clk_en_val	:32;
} switch_clk_9116_t;

//! structure to store configs related to UMAC clk programming

#define RSI_SWITCH_UMAC_CLK 		BIT(0)
#define RSI_SWITCH_QSPI_CLK 		BIT(1)
#define RSI_SWITCH_SLP_CLK_2_32 	BIT(2)
#define RSI_SWITCH_BBP_LMAC_CLK_REG     BIT(3)
#define RSI_SWITCH_MEM_CTRL_CFG		BIT(4)
#define RSI_LMAC_CLK_REG_VAL		0XFFFF
typedef struct switch_clk_s
{
  uint32 switch_clk_info;
  uint32 umac_clock_reg_config : 16;
  //! if switch_qspi_clk is set then this value will be programmed to reg
  uint32 qspi_uart_clock_reg_config : 16;
} switch_clk_t;

/* Device clock settings for 9116 */
typedef struct device_clk_info_9116_s
{
  pll_config_9116_t pll_config_9116_g;
  switch_clk_9116_t switch_clk_9116_g;
} device_clk_info_9116_t;


typedef struct device_clk_info_s
{
  pll_config_t pll_config_g;
  switch_clk_t switch_clk_g;
} device_clk_info_t;

/* structure contains bootup params use for configuring 9116 device */
typedef struct bootup_params_9116
{
  /* indicates whether the params are loaded by external source or by bootloading*/
  uint16 magic_number;
#define LOADED_TOKEN  0x5AA5   /* Bootup params are installed by host or OTP/FLASH (Bootloader) */
#define ROM_TOKEN     0x55AA   /* Bootup params are taken from ROM itself in MCU mode. */
  /* Add proper comment*/
  uint16 crystal_good_time;
  uint32 valid;
#define CRYSTAL_GOOD_TIME                BIT(0)
#define BOOTUP_MODE_INFO                 BIT(1) 
#define DIGITAL_LOOP_BACK_PARAMS         BIT(2)
#define RTLS_TIMESTAMP_EN                BIT(3) 
#define HOST_SPI_INTR_CFG                BIT(4)
#define WIFI_TAPLL_CONFIGS               BIT(5)
#define WIFI_PLL960_CONFIGS              BIT(6)
#define WIFI_AFEPLL_CONFIGS              BIT(7)
#define WIFI_SWITCH_CLK_CONFIGS          BIT(8) 
#define BT_TAPLL_CONFIGS                 BIT(9)
#define BT_PLL960_CONFIGS                BIT(10)
#define BT_AFEPLL_CONFIGS                BIT(11)
#define BT_SWITCH_CLK_CONFIGS            BIT(12)
#define ZB_TAPLL_CONFIGS                 BIT(13)
#define ZB_PLL960_CONFIGS                BIT(14)
#define ZB_AFEPLL_CONFIGS                BIT(15)
#define ZB_SWITCH_CLK_CONFIGS            BIT(16)
#define BUCKBOOST_WAIT_INFO              BIT(17)
#define PMU_WAKEUP_SHUTDOWN_W            BIT(18)
#define WDT_PROG_VALUES                  BIT(19)
#define WDT_RESET_DELAY_VALUE            BIT(20)
#define DCDC_OPERATION_MODE_VALID        BIT(21) 
#define PMU_SLP_CLKOUT_SEL               BIT(22)
#define SOC_RESET_WAIT_CNT               BIT(23)
  //! reserved for future use            
  uint32 reserved_for_valids;
  //!  Add proper comment
  uint16 bootup_mode_info; 
#define BT_COEXIST                       BIT(0)
#define BOOTUP_MODE                     (BIT(2) |BIT(1))
#define CUR_DEV_MODE_9116                    (bootup_params_9116.bootup_mode_info >> 1)
  //! configuration used for digital loop back
  uint16 digital_loop_back_params;
  //!  Add proper comment
  uint16 rtls_timestamp_en;
  //! reserved
  uint16 host_spi_intr_cfg;
  device_clk_info_9116_t device_clk_info_9116[1];
  //! ulp buckboost wait time 
  uint32 buckboost_wakeup_cnt;
  //! pmu wakeup wait time & WDT EN info
  uint16 pmu_wakeup_wait;
  //! ulp shutdown wait time 
  uint8 shutdown_wait_time;
  //! Sleep clock source selection 
  uint8 pmu_slp_clkout_sel;
  //! WDT programming values
  uint32 wdt_prog_value;
  //! WDT soc reset delay
  uint32 wdt_soc_rst_delay;
  //! dcdc modes configs
  uint32 dcdc_operation_mode;
  //!  update doc and make changes here, this is not in proper position
  uint32 soc_reset_wait_cnt;
  uint32 waiting_time_at_fresh_sleep;
  uint32 max_threshold_to_avoid_sleep;
  uint8 beacon_resedue_alg_en;
}BOOTUP_PARAMETERS_9116;




/* structure contains bootup params use for configuring device */
typedef struct bootup_params
{
  /* indicates whether the params are loaded by external source or by bootloading*/
  uint16 magic_number;
#define LOADED_TOKEN  0x5AA5   /* Bootup params are installed by host or OTP/FLASH (Bootloader) */
#define ROM_TOKEN     0x55AA   /* Bootup params are taken from ROM itself in MCU mode. */
  /* Add proper comment*/
  uint16 crystal_good_time;
  uint32 valid;
#define CRYSTAL_GOOD_TIME                BIT(0)
#define BOOTUP_MODE_INFO                 BIT(1) 
#define DIGITAL_LOOP_BACK_PARAMS         BIT(2)
#define RTLS_TIMESTAMP_EN                BIT(3) 
#define HOST_SPI_INTR_CFG                BIT(4)
#define WIFI_TAPLL_CONFIGS               BIT(5)
#define WIFI_PLL960_CONFIGS              BIT(6)
#define WIFI_AFEPLL_CONFIGS              BIT(7)
#define WIFI_SWITCH_CLK_CONFIGS          BIT(8) 
#define BT_TAPLL_CONFIGS                 BIT(9)
#define BT_PLL960_CONFIGS                BIT(10)
#define BT_AFEPLL_CONFIGS                BIT(11)
#define BT_SWITCH_CLK_CONFIGS            BIT(12)
#define ZB_TAPLL_CONFIGS                 BIT(13)
#define ZB_PLL960_CONFIGS                BIT(14)
#define ZB_AFEPLL_CONFIGS                BIT(15)
#define ZB_SWITCH_CLK_CONFIGS            BIT(16)
#define BUCKBOOST_WAIT_INFO              BIT(17)
#define PMU_WAKEUP_SHUTDOWN_W            BIT(18)
#define WDT_PROG_VALUES                  BIT(19)
#define WDT_RESET_DELAY_VALUE            BIT(20)
#define DCDC_OPERATION_MODE_VALID        BIT(21) 
#define PMU_SLP_CLKOUT_SEL               BIT(22)
#define SOC_RESET_WAIT_CNT               BIT(23)
  //! reserved for future use            
  uint32 reserved_for_valids;
  //!  Add proper comment
  uint16 bootup_mode_info; 
#define BT_COEXIST                       BIT(0)
#define BOOTUP_MODE                     (BIT(2) |BIT(1))
#define CUR_DEV_MODE                    (bootup_params.bootup_mode_info >> 1)
  //! configuration used for digital loop back
  uint16 digital_loop_back_params;
  //!  Add proper comment
  uint16 rtls_timestamp_en;
  //! reserved
  uint16 host_spi_intr_cfg;
  device_clk_info_t device_clk_info[3];
  //! ulp buckboost wait time 
  uint32 buckboost_wakeup_cnt;
  //! pmu wakeup wait time & WDT EN info
  uint16 pmu_wakeup_wait;
  //! ulp shutdown wait time 
  uint8 shutdown_wait_time;
  //! Sleep clock source selection 
  uint8 pmu_slp_clkout_sel;
  //! WDT programming values
  uint32 wdt_prog_value;
  //! WDT soc reset delay
  uint32 wdt_soc_rst_delay;
  //! dcdc modes configs
  uint32 dcdc_operation_mode;
  //!  update doc and make changes here, this is not in proper position
  uint32 soc_reset_wait_cnt;
  uint32 waiting_time_at_fresh_sleep;
  uint32 max_threshold_to_avoid_sleep;
  uint8 beacon_resedue_alg_en;
}BOOTUP_PARAMETERS;


typedef struct programming_seq_t
{
	uint16  num_vals;
	uint16  no_of_sets;
	uint16   set[MAX_NUM_OF_SETS][MAX_NUM_OF_BBP_VALS_PER_SET];
}PROG_SEQ_T, *PPROG_SEQ_T;

typedef struct scan_programming_seq_t
{
	uint16  num_vals;
	uint16  per_row;
	uint16  set[24][60];
}SCAN_PROG_SEQ_T,*PSCAN_PROG_SEQ_T;

typedef struct programing_values_t
{
	PROG_SEQ_T        *init_seq;
	PROG_SEQ_T        *wakeup_seq;
	PROG_SEQ_T        *power_val_set;
	SCAN_PROG_SEQ_T   *scan_seq;
	SCAN_PROG_SEQ_T   *scan_seq_amode;
	SCAN_PROG_SEQ_T   *scan_seq_40;
	SCAN_PROG_SEQ_T   *scan_seq_amode_40;
	PROG_SEQ_T        *sleep_seq;
	PROG_SEQ_T        *init_seq_amode;
	PROG_SEQ_T        *sleep_seq_amode;
	PROG_SEQ_T        *wakeup_seq_amode;
	PROG_SEQ_T        *power_val_set_amode;
	PROG_SEQ_T        *init_seq_20_to_40_bgmode;
	PROG_SEQ_T        *init_seq_40_to_20_bgmode;
	PROG_SEQ_T        *init_seq_20_to_40_amode;
	PROG_SEQ_T        *init_seq_40_to_20_amode;

}PROG_VALS_T,*PPROG_VALS_T;

typedef enum _power_mode
{ 
	ONEBOX_PWR_LOW    = 1, 
	ONEBOX_PWR_MEDIUM = 2, 
	ONEBOX_PWR_HIGH   = 3
}POWER_MODE;

/* Power Vals Set For Three Channels */
typedef struct bgn_power_val_set
{
	uint16 low;
	uint16 medium;
	uint16 high[3];
}BGNPOWER_VAL_SET,*PBGN_POWER_VAL_SET;

typedef struct an_power_val_set
{
	uint16 high[3];
}ANPOWER_VAL_SET,*PAN_POWER_VAL_SET;

typedef struct __CHANNEL_PWR_TABLE 
{
	uint16            low_pwr;
	uint16            mid_pwr;
	uint16        high_pwr[20];
}__attribute__ ((packed))CHAN_PWR_TABLE;

struct tx_stat_s 
{
	uint16 staid;
	uint16 count;
	struct 
	{
		uint16 rate_idx;
		uint16 success;
		uint16 attempts;
	} stat[4];
};

typedef struct
{
  //! no. of tx pkts
  uint16 tx_pkts;
  //! no. of rx pkts
  uint16 rx_pkts;
  //! no. of tx retries
  uint16 tx_retries;
  //! no. of pkts that pass crc
  uint16 crc_pass;
  //! no. of pkts failing crc chk
  uint16 crc_fail;
  //! no. of times cca got stuck
  uint16 cca_stk;
  //! no of times cca didn't get stuck
  uint16 cca_not_stk;
  //! no. of pkt aborts
  uint16 pkt_abort;
  //! no. of false rx starts
  uint16 fls_rx_start;
  //! cca idle time
  uint16 cca_idle;
  //! no. of greenfield pkts
  uint16 gf_pkts;
  //! no. of high throughput pkts
  uint16 ht_pkts;
  //! no. of legacy pkts
  uint16 leg_pkt;
  //!  add description
  uint16 leg_prty_fails;
  //! no. of ht pkts failing crc chk
  uint16 ht_crc_fails;
  //!  add description
  uint16 sp_rejected;
  //!  add description
  uint16 lp_rejected;
  //! Channel 1 signal power 
  uint16 ch1_sig_pow;
  //! channel 1 noise power
  uint16 ch1_noise_pow;
  //! channel 2 signal power
  uint16 ch2_sig_pow;
  //! channel 2 noise power
  uint16 ch2_noise_pow;
  //! channel 3 signal power
  uint16 ch3_sig_pow;
  //! channel 3 noise power
  uint16 ch3_noise_pow;
  //! no. of rx retries
  uint16 rx_retries;
  //! rssi value
  uint16 bea_avg_rssi;
  //! cal_rssi
  uint16 cal_rssi;
  //! lna_gain bb_gain
  uint16 lna_bb_gain;
  //! avg_val
  uint16 avg_val;
  //! xretries pkts dropped
  uint16 xretries;
  //! consecutive pkts dropped
  uint16 max_cons_pkts_dropped;
  //! 
  uint16 false_under_sat;
  //!BSS MATCHED BROADCAST PKT STATS
  uint16 bss_broadcast_pkts;
  //!BSS MATCHED MULTICAST PKT STATS
  uint16 bss_multicast_pkts;
  //!BSS and DA MATCHED MULTICAST PKT STATS
  uint16 bss_da_matched_multicast_pkts;
#ifdef WLAN_PER_PKT_RCV_RATE_STATS
  //!No.of pkts rcvd with mcs0
	uint16 pkt_rcvd_with_mcs0;
  //!No.of pkts rcvd with mcs1
	uint16 pkt_rcvd_with_mcs1;
  //!No.of pkts rcvd with mcs2
	uint16 pkt_rcvd_with_mcs2;
  //!No.of pkts rcvd with mcs3
	uint16 pkt_rcvd_with_mcs3;
  //!No.of pkts rcvd with mcs4
	uint16 pkt_rcvd_with_mcs4;
  //!No.of pkts rcvd with mcs5
	uint16 pkt_rcvd_with_mcs5;
  //!No.of pkts rcvd with mcs6
	uint16 pkt_rcvd_with_mcs6;
  //!No.of pkts rcvd with mcs7
	uint16 pkt_rcvd_with_mcs7;
  //!No.of pkts rcvd with 48M
	uint16 pkt_rcvd_with_48M;
  //!No.of pkts rcvd with 24M
	uint16 pkt_rcvd_with_24M;
  //!No.of pkts rcvd with 12M
	uint16 pkt_rcvd_with_12M;
  //!No.of pkts rcvd with 6M
	uint16 pkt_rcvd_with_6M;
  //!No.of pkts rcvd with 54M
	uint16 pkt_rcvd_with_54M;
  //!No.of pkts rcvd with 36M
	uint16 pkt_rcvd_with_36M;
  //!No.of pkts rcvd with 18M
	uint16 pkt_rcvd_with_18M;
  //!No.of pkts rcvd with 9M
	uint16 pkt_rcvd_with_9M;
  //!No.of pkts rcvd with 11M
	uint16 pkt_rcvd_with_11M;
  //!No.of pkts rcvd with 5.5M
	uint16 pkt_rcvd_with_5M;
  //!No.of pkts rcvd with 2M
	uint16 pkt_rcvd_with_2M;
  //!No.of pkts rcvd with 1M
	uint16 pkt_rcvd_with_1M;
#endif
  unsigned int eof_pkt_drop_count;
  unsigned int mask_pkt_drop_count;
  unsigned int ack_sent;
/* Channel Utilization stats */
  unsigned int utilization;
  unsigned int rssi_utilization;
  unsigned int tot_bytes;
  unsigned int rssi_bytes;
  unsigned int interval_duration;
  unsigned int false_cca_avg_rssi;
  unsigned int max_cca_avg_rssi;
  unsigned int cca_duration;
  unsigned int ed_duration;
  unsigned short int noise_rssi;
  int stop_per;
	
} per_stats;
#define PER_RECEIVE         2
#define PER_RECEIVE_STOP    6
#define PER_PACKET          8
#define SET_ENDPOINT        7
#define ANT_SEL	            9
#define CH_UTIL_START			10
#define CH_UTIL_STOP			11
#define RADAR_TEST        12
#define RADAR_ENABLE      13
#define RADAR_PKT	  14
#define ANT_TYPE	 15

#define CONFIG_ANT_SEL 	1
#define CONFIG_ANT_TYPE	2
#define PATH_ONBOARD	1
#define PATH_UFL	2

typedef struct per_params_s
{
	unsigned short enable;
	signed short power;
	unsigned int   rate;
	unsigned short length;
	unsigned short mode;
	unsigned short channel;
	unsigned short rate_flags;
	unsigned short per_ch_bw;
	unsigned short aggr_enable;
	unsigned short aggr_count;
	unsigned short no_of_pkts;
	unsigned int   delay;
	uint8_t ctry_region;
	uint8_t enable_11j;

} per_params;

typedef struct per_packet_s
{
	unsigned char enable;
	unsigned int length;
	unsigned char insert_seq;
	unsigned char packet[1536];
} per_packet;

typedef struct bb_rf_params_s
{ 
	unsigned short Data[1024];
	unsigned short no_of_fields;
	unsigned short no_of_values;
	unsigned char value;
	unsigned char soft_reset;
  unsigned char protocol_id;

} bb_rf_params_t;

struct aggr_params_s
{
	uint16 tx_limit;
	uint16 rx_limit;
};

struct master_params_s
{
	uint32 address;
	uint16 no_of_bytes;
	uint8 *data;
};

struct get_info {
	uint8_t param_name[16];
	uint8_t param_length;
	uint8_t *data;
};
struct test_mode
{
	uint16 subtype;
	uint16 args;
};

typedef struct driver_params_s
{
	uint8_t mac_addr[6];
	uint8_t fw_ver[8];
	uint8_t module_type;
}driver_params_t;

typedef struct wlan_iq_struct_s
{
  unsigned int freq;
  unsigned int rate_flags;
  unsigned int no_of_samples;
  unsigned short *iq_stats;
  unsigned int eof;
  unsigned char *pkt;
  unsigned int offset; 
} wlan_iq_struct_t;

#ifdef ONEBOX_CONFIG_PUF
#define ACTIVATION_CODE_SIZE  1192
#define MAX_KEY_SIZE    32
#define KEY_CODE_SIZE   44
#define MAX_IV_SIZE     32

#define PUF_ENROLL            0
#define PUF_START             1
#define PUF_SET_KEY           2
#define PUF_SET_INTRINSIC_KEY 3
#define PUF_GET_KEY           4
#define PUF_LOAD_KEY          5
#define PUF_AES_ENCRYPT       6
#define PUF_AES_DECRYPT       7
#define PUF_AES_MAC           8
#define PUF_BLOCK_ENROLL      9
#define PUF_BLOCK_SET_KEY     10
#define PUF_BLOCK_GET_KEY     11

#define PUF_SUB_CMD_BYTE      1
#define PUF_AC_SOURCE_BYTE    1
#define MODE_BYTE             1
#define KEY_SIZE              1
#define IV_SIZE               1
#define MODE_INFO             1
#define MODE_BIT              0
#define KEY_SIZE_BIT          1
#define KEY_SOURCE_BIT        2
#define DATA_SIZE             2
#define IV_SIZE_BIT           3

struct puf_init_params
{
  uint8 puf_sub_cmd;
  uint8 puf_ac_source;
  uint8 *activation_code;
};

struct puf_set_key_params
{
  uint8 puf_sub_cmd;
  uint8 key_index;
  uint8 key_size;
  uint8 key[MAX_KEY_SIZE];
  uint8 key_code[KEY_CODE_SIZE];
};

struct puf_get_key_params
{
  uint8 puf_sub_cmd;
  uint8 key_code[KEY_CODE_SIZE];
  uint8 key_holder;
  uint16 key_size;
  uint8 key[MAX_KEY_SIZE];
};

struct puf_aes_data_params
{
  uint8 puf_sub_cmd;
  uint8 mode;
  uint8 key_source;
  uint8 key_size;
  uint8 iv_size;
  uint8 key[MAX_KEY_SIZE];
  uint8 iv[MAX_IV_SIZE];
  uint16 data_size;
  uint8 *data;
  uint8 *enc_dec_mac_data;
};
#endif

#define PER_TRANSMIT        1
#define MAC_ADDR_LEN        6
#define PER_CONT_MODE       1
#define PER_BURST_MODE      0
#define CONTINUOUS_RUNNING  1
#define BURST_RUNNING       2
#define PER_AGGR_LIMIT_PER_PKT      1792
#define MAX_BG_SUPP_RATES   12

typedef struct wlan_priv *WLAN_ADAPTER;
//typedef struct onebox_priv *PONEBOX_ADAPTER;

#include "onebox_net80211.h"
#include "onebox_netbuf.h"
#include "onebox_coex.h"
#include "onebox_mgmt.h"
#include "onebox_hal_ops.h"
#include "onebox_os_intf_ops.h"
//#include "onebox_host_intf_ops.h"
#include "onebox_ioctl.h"
#include "onebox_autorate.h"
#include "onebox_vap.h"
#include "onebox_sdio_intf.h"
#include "onebox_eeprom.h"

struct hal_vap_info
{
		struct ieee80211vap *vap;
		uint8 vap_in_use;
		uint8 vap_id;
};

struct sta_conn_flags {
	uint8 ptk_key;
	uint8 gtk_key;
	uint16 sta_id;
	uint8 eapol4_cnfrm;
};

struct stamode {
	uint8 vap_id;
	uint8 delay_sta_support_decision_flag;
/* The below variables should be in the same order
 * do not swap them as they should be the similar to structure as 
 * sta_conn_flags structure.
 */
	uint8 ptk_key;
	uint8 gtk_key;
	uint16 sta_id;
	uint8 eapol4_cnfrm;
};

/* Adapter structure */
struct wlan_priv 
{
	/* NET80211 related variable
	 * NB: This should be the first variable if using NET80211
	 * Dont alter */
	struct ieee80211com   vap_com;
	struct radiotap_header txtap_hdr;
	struct radiotap_header rxtap_hdr;

	/* Network Interface Specific Variables */
	struct net_device     *dev; /* Stores the netdevice pointer */
	uint32           sc_nvaps;   /* # of active virtual APs */
	uint8            sc_nstavaps; /* # of active station VAPs */
	uint8            sc_nmonvaps;  /* # of monitor VAPs */
	int              sc_default_ieee80211_debug;  /* default debug flags for new VAPs */
	enum ieee80211_opmode    sc_opmode;
	unsigned int  sc_hasbmask:1;   /* bssid mask support */
	unsigned int  sc_hastsfadd:1;  /* tsf adjust support */
	unsigned int  sc_stagbeacons:1; /* use staggered beacons */
	unsigned int  sc_hastpc:1;      /* per-packet TPC support */
	unsigned int  sc_nostabeacons:1; /* no beacons for station */
	/*security support*/
	unsigned int   sc_mcastkey:1;  /* mcast key cache search */
	unsigned int   sc_splitmic:1;  /* split TKIP MIC keys */
	unsigned char   operating_band;
	unsigned char   operating_chwidth;
	unsigned char   primary_channel;
	unsigned char   rf_reset;
	unsigned char   def_chwidth;
	unsigned char   chw_flag;
	
#define ONEBOX_KEYMAX 128
#define ONEBOX_KEYBYTES (ONEBOX_KEYMAX / NBBY) /* storage space in bytes */
	
	struct ieee80211_node *sc_keyixmap[ONEBOX_KEYMAX];/* key ix->node map */
	u_int8_t sc_keymap[ONEBOX_KEYBYTES]; /* key use bit map */
	u_int8_t sec_mode[ONEBOX_VAPS_DEFAULT];

	u_int8_t wep_key[ONEBOX_VAPS_DEFAULT][4][32]; 
    /*In 4 D array, second element ONEBOX_KEYMAX is kept irrelevant (i.e . it is not being used ). 
    * So changed it to 3 D array which contains 4 vaps , each vap contains 4 keys and each key is of size 32bytes*/
	u_int8_t wep_keylen[ONEBOX_VAPS_DEFAULT][4];
	u_int8_t wep_key_idx[ONEBOX_VAPS_DEFAULT];

	/*Function Call backs*/
	void (*sc_node_cleanup)(struct ieee80211_node *);
	void (*sc_node_free)(struct ieee80211_node *);
	struct ieee80211_node *(*sc_node_alloc)(struct ieee80211vap *vap, const uint8 *mac_addr);

	ONEBOX_EVENT              sdio_scheduler_event;
	onebox_thread_handle_t    sdio_scheduler_thread_handle;

	uint32            TransmitBlockSize;
	uint32            ReceiveBlockSize;
	uint32            CardCapability;

	struct net_device_stats  stats;

	uint8       mac_addr[6];

	/* PCI OS Interface Specific Variables */
	struct pci_dev    *pdev;

	/* Netbuf List Specific Variables */
	onebox_netbuf_head_t     mgmt_tx_queue; 
	onebox_netbuf_head_t     host_tx_queue[NUM_SOFT_QUEUES]; 

	onebox_netbuf_head_t		bt_tx_queue;
	onebox_netbuf_head_t		zigb_rx_queue;
	/* WMM specific variables */
	int per_q_wt[NUM_EDCA_QUEUES];
	int wme_org_q[NUM_EDCA_QUEUES];
	int pkt_contended[NUM_EDCA_QUEUES];
	
	uint32 beacon_interval;
	
	/* AP Specific Variable */
	sta_cbl_t           sta[MAX_STATIONS_SUPPORTED];
#define NAME_MAX_LENGTH         32
	uint8               name[NAME_MAX_LENGTH];
	
	/* Tasklet Specific Variables */
	struct completion    txThreadComplete;
	atomic_t   txThreadDone;

	/* PCI HAL Specific Variables */
	void        *hal_handler; /* rsi_host_hal_info_t */
	uint32        total_bcast_mcast_pkt_send;

#ifdef THROUGHPUT_DEBUG
	uint32        prev_sec_data_vo_pkt_send;
	uint32        prev_sec_data_vi_pkt_send;
	uint32        prev_sec_data_be_pkt_send;
	uint32        prev_sec_data_bk_pkt_send;
	uint32        prev_sec_dropped;
	unsigned long prev_jiffies;
#endif

	uint32 total_tx_data_dropped[MAX_HW_QUEUES];
	uint32 total_tx_data_sent[MAX_HW_QUEUES];
	uint16 host_txq_maxlen[MAX_HW_QUEUES];

	uint32 total_mgmt_pkt_send;
	uint32 total_mgmt_pkt_freed;
	uint32 total_data_invalid_pkt_send;
	uint32 total_invalid_pkt_freed;
	uint32        total_bcast_mcast_pkt_freed;
	uint32        total_null_pkt_rcvd;
	uint32        total_unknown_interrupts;
	uint32        big_size_pkts;

	uint32        sdio_intr_status_zero;

	uint32        total_mgmt_rx_done_intr;
	uint32        onair_mgmt_pkts;
	uint32        total_data_rx_done_intr;

	uint32        num_of_sdio_multi_pkts;
	uint32        sdio_int_counter;

	uint32        total_dma_mem_allocated;
	uint32        total_mem_allocated;
	uint32        total_tx_queue_stop;
	uint32        total_tx_queue_start;
	/* SDIO Specific Variables */
	uint8         buffer_full;
	uint8         semi_buffer_full;
	uint8         mgmt_buffer_full;
	uint8         ShutDown;

	rf_type       RFType;
	uint8         bb_rf_prog_count;

#define ONEBOX_DESC_AGGR_ENAB_MASK 0x80
	uint32        buf_full_counter;
	uint32        buf_avilable_counter;
	uint32        mgmt_buf_full_counter;
	uint32        buf_semi_full_counter;
	uint32        no_buffer_fulls; 

	uint32        total_sdio_mgmt_pending_intr;
	uint32        total_sdio_msdu_pending_intr;
	uint32        total_sdio_unknown_intr;
	uint32        next_read_delay;
	uint32        sdio_high_speed_enable;
	uint32        sdio_pkt_count_q[4];
#if((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))&& \
    (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)))
	PSDDEVICE pDevice;
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
	
	struct  sdio_func *sdio_pfunction;
	struct task_struct    *in_sdio_litefi_irq;
#endif
	struct usb_interface *usb_pfunction;
	uint32	buffer_status_register;

	struct device *device;

	uint8            eeprom_type;
	uint16           power_mode;
	uint16           rf_pwr_mode;
	CONFIG_VALS      config_params;
	PROG_VALS_T      onebox_prog_val;
	BGNPOWER_VAL_SET bg_power_set[3];
	ANPOWER_VAL_SET  an_power_set[3];
	uint8 		 band_supported;

	/* Beacon related information */
	uint16         beacon_event; /*  Should be made atomic later */
	uint16         beacon_event_vap_id; /*  Should be made atomic later */
	uint32         bmiss_count;
	uint32         beacon_dbg_count;
	uint32         total_beacon_count;
	uint16         short_slot_time;
	uint16         long_slot_time;
	uint16         short_difs_rx;
	uint16         short_difs_tx;
	uint16         long_difs_rx;
	uint16         long_difs_tx;
	uint16         difs_g_delay;
	uint16         ack_timeout;

	/* Auto Rate stats from TA */
	struct autorate_stats_s    autorate_stats[32][20];
	struct onebox_ratectrl         *sc_rc; /* tx rate control support */

	struct onebox_os_intf_operations *os_intf_ops;
	struct onebox_core_operations *core_ops;
	struct onebox_net80211_operations *net80211_ops;
	struct onebox_devdep_operations *devdep_ops;
	struct ieee80211_rate_ops    *core_rate_ops;
	struct onebox_wlan_osd_operations    *wlan_osd_ops;
	ONEBOX_STATUS (*onebox_send_pkt_to_coex)(struct driver_assets *, netbuf_ctrl_block_t* netbuf_cb, uint8 hal_queue);

	struct mutex sdio_interrupt_lock;
	spinlock_t master_ops_lock ;
	struct semaphore transmit_lock;
	struct semaphore wlan_gpl_lock;
	struct semaphore bt_gpl_lock;
	struct semaphore zigb_gpl_lock;
	uint8  watch_bufferfull_count;
	uint8  firmware_path[256];

	uint8              mpdu_density;
	uint8              short_gi;
	uint32             total_retries;
	uint8              ampdu_en;
	uint8              Driver_Mode;
	struct channel_val onebox_channel_vals_bgmode[15]; 
	struct channel_val onebox_channel_vals_amode[25]; 
	struct channel_val onebox_channel_vals_bgmode_40[15]; 
	struct channel_val onebox_channel_vals_amode_40[25]; 
	uint16             power_val_set[10][20];
	      
	/*version related variables*/
	version_st        driver_ver;
	version_st        ta_ver;
	version_st        lmac_ver;

	uint32           sta_connected_bitmap[4];
	uint32           fsm_state;
	uint32           core_init_done;
	uint32           init_net80211_done;

	/* Current Channel-Power Table */
	CHAN_PWR_TABLE    TxPower11BG[3]; 
	CHAN_PWR_TABLE    TxPower11A[8];
	
	uint16 rps_rate_mcs7_pwr;
	uint16 rps_rate_mcs6_pwr;
	uint16 rps_rate_mcs5_pwr;
	uint16 rps_rate_mcs4_pwr;
	uint16 rps_rate_mcs3_pwr;
	uint16 rps_rate_mcs2_pwr;
	uint16 rps_rate_mcs1_pwr;
	uint16 rps_rate_mcs0_pwr;
	uint16 rps_rate_54_pwr;
	uint16 rps_rate_48_pwr;
	uint16 rps_rate_36_pwr;
	uint16 rps_rate_24_pwr;
	uint16 rps_rate_18_pwr;
	uint16 rps_rate_12_pwr;
	uint16 rps_rate_11_pwr;
	uint16 rps_rate_09_pwr;
	uint16 rps_rate_06_pwr;
	uint16 rps_rate_5_5_pwr;
	uint16 rps_rate_02_pwr;
	uint16 rps_rate_01_pwr;
	uint16 rps_rate_00_pwr;
	
	ONEBOX_EVENT    stats_event;
	per_stats   sta_info;
#ifdef RADAR_AUTO
	struct radar_app *radar_front;
	struct radar_app *radar_rear;
#endif
	ONEBOX_EVENT radar_event;
	uint16  radar_pkt[129];
#ifdef RADAR_AUTO
	spinlock_t radar_lock;
#endif
	uint16 radar_detected_flag;
	uint16 time_stamp_fail;
	uint16 radar_detected;
	uint16 long_radar_detected;
	uint32 no_of_pkts;
	uint32 flush_pkts;
	uint32 full_pkts;
	uint8 large_pulse;
	uint8 init_radar_timer;
	struct timer_list long_pulse_timer;
	uint16 regdomain;
	wmm_pwr_save_t    wmmpwr_save_param;
/*godavari related */
	eepromrw         eepromrw_info;
	onebox_sensitivity sensitivity;
	uint8	sdio_clock_speed;
	bool	PassiveScanEnable;
	uint8	chk_sum_offld_cfg;	
	uint8	isMobileDevice;
	uint8	eeprom_address_type;
	uint8	PermanentAddress[6];
	bool	extPAenabled;
	uint8	cur_antenna;
	uint8	extricom_enable;
	uint8   ch_bandwidth;
	uint8   mac_id;
	uint8 radio_id;
	uint8 stop_card_write;
	/* Auto rate related parms */    
	AUTO_RATE_PARAMS	auto_rate_params;
	/*Debug Variables */
	uint8 prev_desc[FRAME_DESC_SZ];
	/* Rx filter flags */
	uint16 rx_filter_word;
	uint16 sifs_tx_11n;
	uint16 sifs_tx_11b;
	uint16 slot_rx_11n;
	uint16 ofdm_ack_tout;
	uint16 cck_ack_tout;
	uint16 preamble_type;

	per_params      endpoint_params;
	per_packet      per_packet;
// PER_MODE related variables
	netbuf_ctrl_block_t *netbuf_cb_per;
	onebox_thread_handle_t    sdio_scheduler_thread_handle_per;
	uint32 			total_per_pkt_sent ;
	uint8 			recv_channel;
	uint8 			tx_running;
	uint8 			rx_running;
	bb_rf_params_t	bb_rf_params;
	bb_rf_params_t	bb_rf_read;
	uint8 			cw_type;
	uint8 			cw_sub_type;
	uint16 			no_of_per_fragments;
/* PER MODE variables end */
	uint8 			num_vaps;
	uint8 			recv_stop;
	uint8 			bb_rf_rw;
	uint8 			soft_reset;
  	spinlock_t	   	lock_bb_rf;
	ONEBOX_EVENT    bb_rf_event;
	ONEBOX_EVENT    scan_check_event;
	ONEBOX_EVENT    flash_event;
	uint8           endpoint;
	uint16          rf_lpbk_len;
	uint8           dc_calib;
	uint8           rf_lpbk_data[2048*2];   
	uint8           eeprom_erase;
	EEPROMRW        eeprom;
	uint8           qspi_flashing;
	uint32          flash_offset;
	uint8           calib_mode;
	uint8           per_lpbk_mode;
	uint16          buffer_read_len;
	uint16          buffer_recv_len;

	uint8 eapol4_cnfrm_recvd;
	/* Block data pkts while roaming decision is taken */
	uint8 sta_data_block; 
	/* power save related */
#ifdef PWR_SAVE_SUPPORT
	spinlock_t	   lock_timer;
	struct timer_list traffic_timer;
	struct timer_list channel_util_timeout;
	struct timer_list sta_support_feature_send_timeout;
#ifdef IEEE80211K
	struct timer_list rm_mcast_diagnostics_timeout;
#endif
	//uint8 ps_state;
	//uint8 sleep_request_received;
	//uint8 deep_sleep_en;
	/* ULP POWER SAVE variables */
	//uint8 sleep_entry_recvd;
	//uint16 sleep_token;
	//uint8 ulp_sleep_en;
#define SLEEP_ENTRY 1
#endif

	uint8 first_bcast;
	uint8 reorder_flag;
	uint8 buf_status_updated;
	int8 ch_power;
	uint32 pkt_count;
	uint32 beacon_interrupt;
	uint8 std_rates[MAX_BG_SUPP_RATES];
	uint16 rps_rates[MAX_BG_SUPP_RATES];
	struct aggr_params_s aggr_limit;
	//global variable from onebox_core.c
	uint8 per_flag; //static volatile
	//global varibles from onebox_core_os_intf.c
	int udp_pkts_dropped; // has to be removed = 0
	int pkts_dropped_max_queue_buffered; // has to be removed = 0
	//global varibles from onebox_wmm.c
	uint8 selected_qnum, pkt_cnt;
	//global variables from onebox_dev_ops.c
	unsigned long sdio_thread_counter ; //=0
	struct hal_vap_info hal_vap[ONEBOX_VAPS_DEFAULT];
	uint32 max_stations_supported;
#ifdef USE_WORKQUEUES
	struct workqueue_struct *int_work_queue; /* interrupt bottom half queue */
	struct work_struct      defer_work;      /* interrupt bottom half worker */
#endif
	onebox_netbuf_head_t    deferred_rx_queue; 	
#ifdef USE_TASKLETS
	struct tasklet_struct	int_bh_tasklet;
#endif /* USE_TASKLETS */
	struct stamode sta_mode;
	uint32 block_ap_queues;

	struct semaphore ic_lock_vap;
	uint8 rf_power_mode_change; //change in rf_power mode
	//struct iwreq *wrq_ptr;
	void __user *wrq_ptr;
	//store tot time and start flag for channel utilization ioctl
	uint32 ch_util_tot_time;
	uint16 ch_util_start_flag;
   	uint16 stats_interval;
	uint8  false_cca_rssi_threshold;
	int8 ant_gain[2];
	uint8 ant_path;
	uint8 ant_type;
	bool antenna_diversity;
	uint8 scan_count;
	uint8 antenna_in_use;
	struct driver_assets *d_assets;
	struct proc_dir_entry *wlan_proc_entry;
	driver_params_t driver_params;
	ONEBOX_EVENT	iap_event;
	uint8 beacon_recv_disable;
	uint8 IAPP_PKT; /* AP adjacent report packet sent as a data pkt */
	unsigned char scan_state;
	unsigned char max_pwr_enable;
	uint8 spec_mask_set;
	rsi_dev_t device_model;
#ifdef ONEBOX_CONFIG_WOWLAN
	bool wowlan_enabled;
#endif
  ONEBOX_EVENT	wlan_iqs_event;
  wlan_iq_struct_t wlan_iqs_stats;
	uint8 beacon_ssid_loaded;
	uint8 channel_change;
	uint8 start_beacon;
#ifdef ONEBOX_CONFIG_GTK_OFFLOAD
  	unsigned char gtk_en;
#endif
};

/* coex */

void deferred_rx_packet_parser(struct work_struct *work);
void deferred_rx_tasklet(unsigned long d);

//extern uint32 onebox_zone_enabled;
/***************** END DRIVER DATA STRUCTURE TEMPLATES ******************/


#define STD_RATE_MCS7 0x07
#define STD_RATE_MCS6 0x06
#define STD_RATE_MCS5 0x05
#define STD_RATE_MCS4 0x04
#define STD_RATE_MCS3 0x03            
#define STD_RATE_MCS2 0x02
#define STD_RATE_MCS1 0x01
#define STD_RATE_MCS0 0x00 
#define STD_RATE_54   0x6c
#define STD_RATE_48   0x60
#define STD_RATE_36   0x48
#define STD_RATE_24   0x30
#define STD_RATE_18   0x24
#define STD_RATE_12   0x18
#define STD_RATE_11   0x16
#define STD_RATE_09   0x12
#define STD_RATE_06   0x0C
#define STD_RATE_5_5  0x0B
#define STD_RATE_02   0x04
#define STD_RATE_01   0x02

#define RSI_RATE_00   0x00
#define RSI_RATE_1    0x0
#define RSI_RATE_2    0x2
#define RSI_RATE_5_5  0x4
#define RSI_RATE_11   0x6
#define RSI_RATE_6    0x8b
#define RSI_RATE_9    0x8f
#define RSI_RATE_12   0x8a
#define RSI_RATE_18   0x8e
#define RSI_RATE_24   0x89
#define RSI_RATE_36   0x8d
#define RSI_RATE_48   0x88
#define RSI_RATE_54   0x8c
#define RSI_RATE_MCS0 0x100
#define RSI_RATE_MCS1 0x101
#define RSI_RATE_MCS2 0x102
#define RSI_RATE_MCS3 0x103
#define RSI_RATE_MCS4 0x104
#define RSI_RATE_MCS5 0x105
#define RSI_RATE_MCS6 0x106
#define RSI_RATE_MCS7 0x107
#define RSI_RATE_MCS7_SG 0x307

struct pulse_sample
{
	uint32 time : 16;
	uint32 pw : 7;
	uint32 debug : 9;
} __attribute__((__packed__));

struct pulse_history
{
	uint32 start_time;
	uint32 end_time;
	int count;
	int missed_pls_cnt;
	uint32 pw;
	uint16 pwr;
};

#define REG_DOMAIN_US 1
#define REG_DOMAIN_EU 2
#define REG_DOMAIN_JP 3
#define REG_DOMAIN_WORLD 4

#define MAX_FCC_PATTERNS 6
#define MAX_JAPAN_PATTERNS 3
#define MAX_ETSI_PATTERNS 6
#define MAX_PATTERNS (MAX_FCC_PATTERNS + MAX_JAPAN_PATTERNS + MAX_ETSI_PATTERNS)

#define PW_TOLARANCE 40
#define PWR_TOLARANCE 10
#define MAX_UINT32 0xFFFFFFFF
#define LAST_PULSE_INDICATION 0xFFFF
#define ETSI_MIN_PRI 100

#define MIN_INDEX_US 0
#define MAX_INDEX_US 5
#define MIN_INDEX_JP MIN_INDEX_US 
#define MAX_INDEX_JP 8
#define MIN_INDEX_EU 9
#define MAX_INDEX_EU 14

#define RSI_INTERNAL_ANTENNA 0x2
#define RSI_EXTERNAL_ANTENNA 0x3

struct pulse_sample get_min_pri( struct pulse_sample *radar, uint8 count, int i);
struct pulse_sample optimize_min_pri( struct pulse_sample  *radar, uint8 count, struct pulse_sample min_radar_pulse, int i);
void   populate_multiple_pri(uint16 time );
struct pulse_sample find_next_min( struct pulse_sample  *radar, uint8 count, struct pulse_sample min_radar_pulse, int i);
ONEBOX_STATUS check_staggered_pattern(struct pulse_sample *radar_org, uint8 count, struct pulse_sample min_radar_pulse, int pattern );
uint8 check_for_clubbed_pulses(uint16 pulse, uint16 *staggered_array, uint8 staggered_depth);
void radar_timer_callback(WLAN_ADAPTER w_adapter);
void traffic_timer_callback(WLAN_ADAPTER w_adapter);
uint32 check_deep_sleep_status(WLAN_ADAPTER w_adapter);
void send_ps_params_req(struct ieee80211vap *vap, uint32 ps_en, uint32 queue_pkt);
void pwr_sve_event_handler(struct ieee80211vap *vap, uint32 ps_status, uint32 path, uint32 queue_pkt_to_head);
void send_bgscan_host_triggered_bgscan(struct ieee80211vap *vap);
void onebox_reorder_pkt(WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb);
void dump_debug_frame(uint8 *msg, int32 msg_len);
struct onebox_wlan_osd_operations *onebox_get_wlan_osd_operations_from_origin(void);
struct onebox_devdep_operations *onebox_get_devdep_wlan_operations(void);
struct onebox_net80211_operations *onebox_get_net80211_operations(void);
struct onebox_core_operations *onebox_get_core_wlan_operations(void);
struct ieee80211_rate_ops *onebox_get_ieee80211_rate_ops(void);
ONEBOX_STATUS wlan_read_pkt(WLAN_ADAPTER w_adapter,
				netbuf_ctrl_block_t *netbuf_cb);
ONEBOX_STATUS setup_wlan_procfs(WLAN_ADAPTER w_adapter);
void destroy_wlan_procfs(WLAN_ADAPTER w_adapter);
netbuf_ctrl_block_t* get_skb_cb(netbuf_ctrl_block_m_t *netbuf_cb_m);
ONEBOX_STATUS is_vap_valid(struct ieee80211vap *vap);
void onebox_print_mac_address(WLAN_ADAPTER w_adapter, uint8 *mac_addr);
void onebox_send_sta_supported_features(struct ieee80211vap *vap, WLAN_ADAPTER w_adapter);
void initialize_sta_support_feature_timeout(struct ieee80211vap *vap, WLAN_ADAPTER w_adapter);
void stop_initial_timer(struct ieee80211com *ic, struct ieee80211vap *vap);
ONEBOX_STATUS onebox_send_radar_req_frame(WLAN_ADAPTER w_adapter, uint8 radar_req, uint8 intr_clr);
ONEBOX_STATUS onebox_radar_detect_algo(WLAN_ADAPTER w_adapter, uint8* msg, uint32 length);
ONEBOX_STATUS onebox_send_block_unblock(struct ieee80211vap *vap, uint8 notify_event, uint8 quiet_enable);
void send_bgscan_params_default(struct ieee80211com *ic, uint8 band);
#ifdef RADAR_AUTO
ONEBOX_STATUS onebox_send_radar_packets_to_matlab(WLAN_ADAPTER w_adapter, uint16 radar_indication);
#endif
#if 0
#define KASSERT() do { \
	dumpo_stack();	\
}while(0)
#endif

//ONEBOX_STATUS wlan_module_init(struct driver_assets *d_assets);
ONEBOX_STATUS wlan_module_deinit(struct driver_assets *d_assets);
ONEBOX_STATUS set_per_configurations (WLAN_ADAPTER w_adapter);
ONEBOX_STATUS onebox_send_disable_programming(WLAN_ADAPTER w_adapter,unsigned char buffer);
ONEBOX_STATUS onebox_send_programming_structs(WLAN_ADAPTER w_adapter,prog_structure_t* prog_structure_p);
ONEBOX_STATUS onebox_send_wlan_iq_capture_request(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS onebox_mgmt_send_9116_rf_prog_frame(WLAN_ADAPTER w_adapter, uint16 *rf_prog_vals, uint16 num_of_sets, uint16 vals_per_set, uint8  type, uint8 protocol_id); 
#ifdef ONEBOX_CONFIG_GTK_OFFLOAD 
void onebox_send_gtk_rekey_data(struct ieee80211vap *vap, struct ieee80211_gtk_rekey_data *data);
#endif
#endif
