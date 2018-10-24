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

#ifndef __ONEBOX_MGMT_H__
#define __ONEBOX_MGMT_H__

#include "onebox_common.h"
#include "onebox_hal.h"
//#include "rsi_dev_config.h"

#define INIT_SEQ      0x00
#define CHAN_SEQ      0x01


#define LEGACY_BG_PWR_VAL_LEN   5   
#define LEGACY_AN_PWR_VAL_LEN   3
#define BG_PWR_VAL_LEN          22    
#define AN_PWR_VAL_LEN          18 

#define GLBL_IN_PWR_SAVE                  1  
#define GLBL_NO_PWR_SAVE                  2 
#define GLBL_PWR_SAVE_DISABLE_REQ_SENT    3
#define GLBL_PWR_SAVE_ENABLE_REQ_SENT     4

#define ONEBOX_EEPROM_THREE_BYTE_ADDRESS 0 
#define ONEBOX_EEPROM_ONE_BYTE_ADDRESS   1   
#define ONEBOX_SET_EXTRICOM 1

#define MAX_MGMT_PKT_SIZE 512
#define ONEBOX_PAD_HEADER         40

#define ASSERT_VERSION_MISMATCH 0xA2
/**
 * Management Frame Types
 */
/* Send Frames Types */
enum  cmd_frame_type 
{
 WAKEUP_SLEEP_REQUEST = 0x05,      
 BOOTUP_PARAMS_REQUEST = 12,     
 EEPROM_READ_TYPE = 14,         
 EEPROM_WRITE = 15,              
 CONFIRM = 33,
 TX_MISC_IND = 38, /* Reserved for common hal */
 COMMON_DEV_CONFIG = 40,
 IAP_CONFIG = 44, /* For IAP chip communication */
 STRUCT_PRGMG_STATS = 51,
 PUF_REQUEST_FRAME = 52,
};

/* Receive Frame Types */
typedef enum {
 CARD_READY_IND	= 0x0,
 TA_CONFIRM_TYPE = 0x01,
 ULP_SLEEP_NOTIFY = 0x06,
 RX_MISC_IND = 0xa
} rx_cmd_type_t;

//!Subtypes for RX_MISC_IND frame
////! Frame sub types from LMAC to Host
typedef enum {
  FW_UPGRADE_REQ
} rx_misc_ind_subtype;

//  //!Subtypes for TX_MISC_IND frame
//  //! Frame sub types from host to LMAC
typedef enum {
  FW_UPGRADE_DONE
} tx_misc_ind_subtype;


/* Receive Frame Types */
#define RESULT_CONFIRM             0x80
#define MGMT_PPE_DEBUG_FRAME       0x82
#define MIC_FAILURE_FRAME          0x86
#define PER_STATS_PACKET           0x84


#define EAPOL4_CONFIRM						1
#define PROBEREQ_CONFIRM					2
#define NULLDATA_CONFIRM					3


#define LMAC_FW_IND       0xfe
#define AUTO_RATE_STATS   0x98
#define SCANREQ_CONFIRM						6

/* Result Confirm Frame types */
#define TEMPLATE_CONFIRM          0x3A
#define RF_CONFIRM                   0x3C
#define START_BSS_CONFIRM            0x0038
#define RESET_PPE_CONFIRM            0x0036
#define SET_QOS_PARAMS_CONFIRM       0x002E
#define STATION_STATISTICS_RESP      0x0F9E 
#define MGMT_DESC_TYP_BEACONS_RESP   0xB600
#define MGMT_DESC_TYP_PD_VAL_READ    0xDC00
#define MGMT_DESC_TYP_STATS_RESP     0xA200
#define MGMT_DESC_TYP_RESET_MAC_CFM  0xD600
/*Confirm the type values */
#define MGMT_DESC_TYP_GCPD_VALS_CFM      0x0000
#define MGMT_DESC_TYP_DEEP_SLEEP_ENABLE_CFM    0x0000
#define MGMT_DESC_TYP_DEEP_SLEEP_DISABLE_CFM      0x0000
#define MGMT_DESC_TYP_WAKEUP_SLEEP_CFM      0x0000
 
/* Peer notify events */
#define ONEBOX_DELETE_PEER 0x0
#define ONEBOX_ADD_PEER    0x1

/* Macro to indicate fw thatBeacon miss handling will be done by Hardware */
#define ONEBOX_HW_BMISS_HANDLE    ONEBOX_BIT(9)

/* Flag to check whether user wants S/W or HW bmiss handling*/
#define HW_BEACON_MISS_HANDLE     ONEBOX_BIT(0)
#define BB_SENS_PROG                 0x2

/* ampdu indication events */
#define START_AMPDU_AGGR  0x1
#define STOP_AMPDU_AGGR  0x0

/* Netbuf flags */
#define ENCRYPTION_ENBL       ONEBOX_BIT(0)
#define INTERNAL_MGMT_PKT     ONEBOX_BIT(7)
#define MORE_DATA             ONEBOX_BIT(8)
#define FIRST_BCAST           ONEBOX_BIT(14)
#define LAST_BCAST            ONEBOX_BIT(15)

/* Power save Indication flags for Broadcast Multicast Pkts*/
#define MORE_DATA_PRESENT         ONEBOX_BIT(1)
#define DTIM_BEACON_GATED_FRAME   ONEBOX_BIT(10)


/* Rx Path, RF Types  */
#define RF_AL2236   0x2
#define RF_AL8230   0x3
#define ONEBOX_EXTPA_MODULE  0xa

/* Tx Path */
#define MGMT_DESC_START_AUTORATE 0x0D00

/* Queue information */
#define ONEBOX_BT_MGMT_Q 0x6
#define ONEBOX_BT_DATA_Q 0x7
#define ONEBOX_WIFI_MGMT_Q 0x4
#define ONEBOX_WIFI_DATA_Q 0x5

#define BT_DEFAULT_PS_REQ  0x12

#define BAND_2_4GHZ 0
#define BAND_5GHZ   1
/* No Seperate sets for 4.9 &5G Band, Hence same endpoint */
#define BAND_4_9GHZ 1

#ifndef PROGRAMMING_BBP_TA
#define BW_20Mhz 20
#define BW_40Mhz 40
#else
#define BW_20Mhz 0
#define BW_40Mhz 1
#endif

/* Powersave notify indication flags */
#define STA_EXITED_PS  ONEBOX_BIT(0)
#define STA_ENTERED_PS  ONEBOX_BIT(1)

/**
 * Driver FSM states
 */
#define FSM_CARD_NOT_READY            0
#define FSM_FW_LOADED		      1
#define FSM_LOAD_BOOTUP_PARAMS        2
#define FSM_EEPROM_CHECK              3 
#define FSM_EEPROM_READ_RF_TYPE       4
#define FSM_EEPROM_READ_MAC_ADDR      5
#define FSM_EEPROM_READ_2P4_PWR_VALS  6 
#define FSM_EEPROM_READ_5P1_PWR_VALS  7 
#define FSM_RESET_MAC_CFM             8
#define FSM_BB_RF_START               9
#define FSM_WAKEUP_SLEEP_VALS         10 
#define FSM_OPEN                      11
#define FSM_DEEP_SLEEP_ENABLE         12
#define FSM_MAC_INIT_DONE             13
#define FSM_AMPDU_IND_SENT            14
#define FSM_SCAN_CFM                  15
#define FSM_DEVICE_READY	      16
#define FSM_FLASH_BURN		      17

#define ONEBOX_LOAD_QOS_CONFIG_PARAM_ADDR   0x00C0
#define ONEBOX_LOAD_RATE_SYMBOLS_ADDR       0x0210
#define ONEBOX_LOAD_BSS_CONFIG_PARAM_ADDR   0x0000
#define ONEBOX_LOAD_BSS_CONFIG_OPERATIONAL_PARAM_ADDR 0x00F2
#define ONEBOX_LOAD_BEACON_DATA_ADDR        0x03C0
#define ONEBOX_LOAD_STATION_UPDATES_ADDR    0x280

/* Station Capability */
#define ONEBOX_CAP_AUTH           ONEBOX_BIT(0)
#define ONEBOX_CAP_ASSOC          ONEBOX_BIT(1)
#define ONEBOX_CAP_WEP_ENABLED    ONEBOX_BIT(2)
#define ONEBOX_CAP_WEP_128        ONEBOX_BIT(3) /* 0 - WEP 64, 1 - WEP 128 */
#define ONEBOX_CAP_RSN            ONEBOX_BIT(4)
#define ONEBOX_CAP_WPA            ONEBOX_BIT(5) /* 0 - CCMP, 1 - TKIP */
#define ONEBOX_CAP_DATA_STRUCT_OK ONEBOX_BIT(10)
#define ONEBOX_CAP_SECURITY_EN    ONEBOX_BIT(13)

/**
 * AP Capability Status Bits
 */
#define ONEBOX_AP_CAP_SECURITY_ENABLED ONEBOX_BIT(0)
#define ONEBOX_AP_CAP_WEP_ENABLED      ONEBOX_BIT(1)
#define ONEBOX_AP_CAP_WEP_128          ONEBOX_BIT(2)
#define ONEBOX_AP_CAP_WPA_ENABLED      ONEBOX_BIT(3)
#define ONEBOX_AP_CAP_WPA_TKIP         ONEBOX_BIT(4)
#define ONEBOX_AP_CAP_WPA2_CCMP        ONEBOX_BIT(5)
#define ONEBOX_AP_CAP_KEY_LOADED       ONEBOX_BIT(6)
#define ONEBOX_AP_CAP_WMM_ENABLED      ONEBOX_BIT(7)
#define ONEBOX_AP_CAP_HT_ENABLED       ONEBOX_BIT(8)
#define ONEBOX_AP_CAP_AMSDU_ENABLED    ONEBOX_BIT(9)
#define ONEBOX_AP_CAP_AMPDU_ENABLED    ONEBOX_BIT(10)

/* To enable self cts frame */
#define ONEBOX_SELF_CTS_ENABLE		   ONEBOX_BIT(1)

#define BBP_REMOVE_SOFT_RST_BEFORE_PROG  ONEBOX_BIT(0) //Take BBP out of reset
#define BBP_REMOVE_SOFT_RST_AFTER_PROG   ONEBOX_BIT(1) //Take BBP out of reset
#define BBP_REG_READ          ONEBOX_BIT(2)
#define PUT_BBP_RESET        0
#define BBP_REG_WRITE        0
#define BBP_REG_READ          ONEBOX_BIT(2)
#define RF_RESET_ENABLE       ONEBOX_BIT(3)
#define ULP_MODE              ONEBOX_BIT(0)

#define RATE_INFO_ENABLE      ONEBOX_BIT(0) 
#define ONEBOX_BROADCAST_PKT  ONEBOX_BIT(9)

#define ONEBOX_11B_MODE  0 /* Bit{8:7} */
#define ONEBOX_11G_MODE  ONEBOX_BIT(7)
#define ONEBOX_11N_MODE  ONEBOX_BIT(8)
#define ONEBOX_11AC_MODE ONEBOX_BIT(7) | (ONEBOX_BIT(8)

#define NONRSI_RF_TYPE       0
#define RSI_RF_TYPE         1

#define IEEE80211_OP_AP       0x0
#define IEEE80211_OP_STA      0x1
#define IEEE80211_OP_P2P_GO   0x2
#define IEEE80211_OP_P2P_CLIENT  0x3
#define IEEE80211_OP_IBSS     0x4

/*40 MHz specific */
#define UPPER_20_ENABLE   (0x2 << 12)
#define LOWER_20_ENABLE   (0x4 << 12)
#define FULL_40M_ENABLE   0x6 


#define ONEBOX_LMAC_CLOCK_FREQ_40MHZ	0x0
#define ONEBOX_LMAC_CLOCK_FREQ_80MHZ	0x1
#define ONEBOX_LMAC_CLOCK_FREQ_160MHZ	0x2
#define ONEBOX_LMAC_CLOCK_FREQ_320MHZ	0x3

#define ONEBOX_ENABLE_20MHZ     0x0<<3//bandwidth
#define ONEBOX_ENABLE_40MHZ     0x1<<3
#define ONEBOX_ENABLE_80MHZ     0x2<<3

#define BROADCAST_IND        ONEBOX_BIT(9)
#define ENABLE_MAC_INFO      ONEBOX_BIT(0)
#define CONTINUOUS_MODE      ONEBOX_BIT(10)


#define ENABLE_SHORTGI_RATE        ONEBOX_BIT(9)

/* MATLAB sub types*/

#define BB_READ_REQ     0x0
#define BB_WRITE_REQ    0x1
#define RF_READ_REQ     0x2
#define RF_WRITE_REQ    0x3
#define ULP_READ_REQ    0x4
#define ULP_WRITE_REQ   0x5
#define BUF_READ_REQ    0x6
#define BUF_WRITE_REQ   0x7
#define RF_LOOPBACK_M3   0xC
#define RF_LOOPBACK_M2   0xA
#define RF_RESET_REQ           0xB
#define LMAC_REG_WRITE         0xF
#define LMAC_REG_READ          0x10
#define ENDPOINT               0x11
#define EEPROM_RF_PROG_WRITE   0x9
#define EEPROM_RF_PROG_READ    0x12


/* Tx data frame format */
#define MAC_BBP_INFO  ONEBOX_BIT(0)
#define NO_ACK_IND    ONEBOX_BIT(9)
#define QOS_EN        ONEBOX_BIT(12)
/* frame type bit{11:10} */
#define NORMAL_FRAME              0x00
#define DTIM_BEACON_GATED_FRAME   ONEBOX_BIT(10) 
#define BEACON_FRAME              ONEBOX_BIT(11)
#define DTIM_BEACON               ONEBOX_BIT(10) | ONEBOX_BIT(11)
#define INSERT_TSF                ONEBOX_BIT(15)
#define INSERT_SEQ_NO             ONEBOX_BIT(2)

#define PROCESS_CONTEXT    0
#define ATOMIC_CONTEXT     1

/**Internal Mgmt frames from COMMON HAL***/
#define DEFAULT_PS_SLEEP_REQUEST  0x25

typedef enum
{
	ONEBOX_LOW_SENSITIVITY = 0,
	ONEBOX_MID_SENSITIVITY = 1,
	ONEBOX_HIGH_SENSITIVITY = 2
}onebox_sensitivity;

/* Auto rate related parms */    
typedef struct auto_rate_parms   
{ 
  uint16  failure_limit; 
/* Initial value of success limit */
  uint16  Initial_boundary;
/* max window size for a rate to pick the next best rate. For a given
 * rate if max threshold limit is reached it will fall back to the next lower rate
 * */
  uint16  max_threshold_limt;
}__attribute__ ((packed))AUTO_RATE_PARAMS;
                                 

/*
 * Each associated station information
 */
#define MAX_STATIONS_SUPPORTED  30
#define MAX_CIPHER_LEN          32

typedef enum 
{
	ONEBOX_WPA_CIPHER_NONE = 0x1,
	ONEBOX_WPA_CIPHER_WEP40 = 0x2,
	ONEBOX_WPA_CIPHER_WEP104 = 0x4,
	ONEBOX_WPA_CIPHER_TKIP = 0x8,
	ONEBOX_WPA_CIPHER_CCMP = 0x10,
	ONEBOX_WPA_CIPHER_BOTH = 0x18
}onebox_cipher_t;

typedef enum 
{
	ONEBOX_PAIRWISE_KEY = 1,
	ONEBOX_GROUP_KEY,
	ONEBOX_STA_KEY
}onebox_keytype_t;

/*********************************************TA GPIOs	 ************************************
GPIO_0	PSPI_CLK
GPIO_1	PSPI_CSN0
GPIO_2	PSPI_CSN1/Host Wakeup interrupt ( Device to Host)
GPIO_3	PSPI_DATA0
GPIO_4	PSPI_DATA1
GPIO_5	PSPI_DATA2
GPIO_6	PSPI_DATA3
GPIO_7	I2C_SCL
GPIO_8	I2C_SDA
GPIO_9	UART1_RX
GPIO_10	UART1_TX
GPIO_11	UART1_RTS/I2S_CLK
GPIO_12	UART1_CTS/I2S_WS
GPIO_13	Debug UART_RX/I2S_DIN
GPIO_14	Debug  UART_TX/I2S_DOUT
GPIO_15	LP Wakeup/Boot Bypass (Host to Device)
GPIO_16	LED 0
GPIO_17	BT Coexistance WLAN_ACTIVE/ EXT_PA ANT SEL
GPIO_18	BT Coexistance BT_PRIORITY/ EXT_PA ANTSEL_B
GPIO_19	BT Coexistance BT_ACTIVE/ EXT_PA ON_OFF
GPIO_20	RF reset
GPIO_21	Sleep indication from device (Device to Host)
****************************************************************************/
#define	 RSI_PSPI_CSN_0 							BIT(0)
#define	 RSI_PSPI_CSN_1 							BIT(1)
#define	 RSI_HOST_WAKEUP_INTR							BIT(2)
#define	 RSI_PSPI_DATA_0							BIT(3)
#define	 RSI_PSPI_DATA_1							BIT(4)
#define	 RSI_PSPI_DATA_2							BIT(5)
#define	 RSI_PSPI_DATA_3							BIT(6)
#define	 RSI_I2C_SCL								BIT(7) 
#define	 RSI_I2C_SDA 								BIT(8)
#define	 RSI_UART1_RX								BIT(9)
#define	 RSI_UART1_TX								BIT(10)
#define	 RSI_UART1_RTS_I2S_CLK							BIT(11)
#define	 RSI_UART1_CTS_I2S_WS 							BIT(12)
#define	 RSI_DEBUG_UART_RX_I2S_DIN						BIT(13) 
#define	 RSI_DEBUG_UART_TX_I2S_DOUT						BIT(14)
#define	 RSI_LP_WAKEUP_BOOT_BYPASS					 	BIT(15)
#define	 RSI_LED_0 		 					       	BIT(16)
#define	 RSI_BT_COEXISTANCE_WLAN_ACTIVE_EXT_PA_ANT_SEL_A			BIT(17)
#define	 RSI_BT_COEXISTANCE_BT_PRIORITY_EXT_PA_ANT_SEL_B			BIT(18)
#define	 RSI_BT_COEXISTANCE_BT_ACTIVE_EXT_PA_ON_OFF				BIT(19)
#define	 RSI_RF_RESET								BIT(20)
#define	 RSI_SLEEP_INDICATION_FROM_DEVICE_SOC 					BIT(21)
typedef struct rsi_soc_gpio_vals_s {
	uint32 rsi_soc_gpio_vals_s_info;
} rsi_soc_gpio;

/************************************************************************
ULP GPIOs	
ULP_GPIO_0	Motion sensor / ULP wakeup
ULP_GPIO_1	DPDT Switch Selection / Sleep indication from device (Device to Host)
ULP_GPIO_2	
ULP_ANAGPI	Pushbutton(ULP Wakeup)
************************************************************************/
#define RSI_MOTION_SENSOR_GPIO_ULP_WAKEUP 		BIT(0)
#define RSI_SLEEP_INDICATION_FROM_DEVICE		BIT(1)
#define RSI_ULP_GPIO_2					BIT(2)
#define RSI_PUSH_BUTTON_ULP_WAKEUP 			BIT(3)
typedef struct rsi_ulp_gpio_vals_s {
    uint8 rsi_ulp_gpio_vals_s_info; 
}rsi_ulp_gpio_vals;

typedef struct onebox_crypt_s 
{
	uint16                 keyLen;
	uint16                 keyId;
	onebox_keytype_t       keyType;
	uint8                  rsc[96];
	uint8                  tsc[96];
	onebox_cipher_t        cipher;
	uint8                  key[MAX_CIPHER_LEN];
}core_crypt_t;


/* Power save handshake types */
#define NO_HAND_SHAKE 			0
#define GPIO_HAND_SHAKE 		1
#define PACKET_HAND_SHAKE 		2
#define TA_GPIO				0
#define ULP_GPIO			1
#define RF_POWER_3_3			1
#define RF_POWER_1_9			0

typedef struct rsi_config_vals_s {
	uint8 lp_sleep_handshake;
  uint8 ulp_sleep_handshake;
	uint8 sleep_config_params; /* 0 for no handshake (pm will make the decision),1 for GPIO based handshake, 2 packet handshake */
	uint8 host_wakeup_intr_enable;
	uint8 host_wakeup_intr_active_high;
	uint32 lp_wakeup_threshold;
	uint32 ulp_wakeup_threshold;
	uint32 wakeup_threshold; /* should be the last field, this is nt configurable by host */
	//! This variable should be assigned as per the SoC GPIOs unused in the schematics.
	rsi_ulp_gpio_vals unused_ulp_gpio_bitmap;

	rsi_soc_gpio unused_soc_gpio_bitmap;
	//! If zero -> niether EXT PA nor EXT BT is enabled
	//! This variable should be assigned as per the ULP GPIOs unused in the schematics.
	uint8 ext_pa_or_bt_coex_en; 
  uint8 opermode;
	uint8 driver_mode;
#define EXT_PA      1
#define EXT_BT_COEX 2

	/*  
	 *  These will be added later
	 *  uint8 sleep_clock_src;
	 *  uint8 external_pa_enabled;
	 *  uint8 external_bt_coex;
	 */
} rsi_config_vals;

struct mfi_challenge {
	uint8 challenge_data[20];
};

#define UNUSED_GPIO 1
#define USED_GPIO 0

#define RSI_IAP_INIT 0
#define RSI_MFI_WRITE_CHALLENGE 1
#define RSI_MFI_READ_CERTIFICATE 2

/**USAGE OF GPIO's ***********************
 * 1-> indicates the GPIO is not used
 * 0-> indicates the GPIO is used 
 */

///* To enable a GPIO make the GPIO value as 1 */
#pragma pack(1)
typedef struct onebox_mac_frame_s 
{
	/*** Descriptor 4 Bytes ***/
	union 
	{
		uint16  desc_word[8];
		uint8   desc_byte[16];

		struct 
		{
			uint8  frame_len;
			uint8  frame_type;
		};
	};

	union 
	{
		struct 
		{
			uint8 buf[512];
		} byte;

		struct 
		{
			uint16 buf[256];
		} word;
		CONFIG_VALS bootup_params;
		CONFIG_VALS bootup_params_9116;
		rsi_config_vals dev_config_vals;
	} u;
} onebox_mac_frame_t;

#pragma pack()

typedef struct {
  uint8 enable;
  uint8 ac_vo;   /* 0 disable 1-enable */
  uint8 ac_vi;
  uint8 ac_be;
  uint8 ac_bk;
  uint8 wakeup_period; /* wakeup period in msecs or tx-triggered */
  uint8 use_mimic_apsd;
}__attribute__ ((packed)) wmm_pwr_save_t;

typedef enum _PWR_STATUS
{              
  PWR_STATUS_FAILURE = 0,
  PWR_STATUS_SUCCESS = 1,
  PWR_STATUS_GIVE_DISABLE
}PWR_STATUS;   

struct bl_header {
  uint32 flags;
  uint32 image_no;
  uint32 check_sum;
  uint32 flash_start_address;
  uint32 flash_len;
};

//#if 0
/* Bluetooth packet types & code segmentd */
//#ifdef ENABLE_BT_DRIVER
/* bluetooth dafault values */
#define BT_PKT_TX_TOUT			0xffff
#define BT_CLK_FREQ			32
#define BT_PKT_REC_TOUT			0x0C35   // slot time  
//#define BT_RX_CTRL_REG			0x1004 // Promiscuous mode
#define BT_RX_CTRL_REG		0x1000 // Normal mode
#define BT_CRC_INIT_LS_WORD		0x5555
#define BT_CRC_INIT_MS_BYTE		0x0055
//#define BT_DEVICE_ADDR_1		0x1453
//#define BT_DEVICE_ADDR_2		0x4954
//#define BT_DEVICE_ADDR_3		0x4853

#define BT_DEVICE_ADDR_3		0xc0ff
#define BT_DEVICE_ADDR_2		0xeec0
#define BT_DEVICE_ADDR_1		0xffed

/* packet types */
#define BT_PKT_TYPE_ID	 	0	/* For ID Packet */
#define BT_PKT_TYPE_SCO 	1	/* For SCO Packet */
#define BT_PKT_TYPE_ESCO	2	/* For ESCO Packet */
#define BT_PKT_TYPE_ACL		3	/* For ACL Packet */
#define BT_PKT_LE_ADV		4	/* For LE Advertising Channel Packet */
#define BT_PKT_LE_DATA		5	/* For LE Data Channel Packet */

/* rates */
#define BT_BDR 		1	/* Basic Data Rate */
#define BT_EDR		2	/* Enhanced Data Rate */

/* defines for station update */
#define BT_SCO 			0x1000
#define BT_ESCO_BDR		0x2000
#define BT_ESCO_EDR		0x2800
#define BT_ACL_EDR		0x0800

/* packet subtypes */
#define BT_PKT_SUBTYPE_0 	0
#define BT_PKT_SUBTYPE_1 	1
#define BT_PKT_SUBTYPE_2	2
#define BT_PKT_SUBTYPE_3	3
#define BT_PKT_SUBTYPE_4	4
#define BT_PKT_SUBTYPE_5	5
#define BT_PKT_SUBTYPE_6	6
#define BT_PKT_SUBTYPE_7	7
#define BT_PKT_SUBTYPE_8	8
#define BT_PKT_SUBTYPE_9	9
#define BT_PKT_SUBTYPE_10	10
#define BT_PKT_SUBTYPE_11	11
#define BT_PKT_SUBTYPE_12	12
#define BT_PKT_SUBTYPE_13	13
#define BT_PKT_SUBTYPE_14	14
#define BT_PKT_SUBTYPE_15	15

#define BT_PKT_SUBTYPE_16	16 /* ID pkt type in E2E mode */


/* station update params for E2E */
#define GENERIC_ACCESS_CODE     0x475C58CC73345E72LL
#define GENERIC_ACCESS_CODE_0   0x475C58CC
#define GENERIC_ACCESS_CODE_1   0x73345E72
#define GENERIC_HEC_SEQ         0x75
#define GENERIC_SCRAM_SEED      0x47
#define GENERIC_DEVICE_ADDR	0x9E8B00

/* scan states in BT mode */
#define INQUIRY_STATE		0
#define PAGE_STATE		1
#define CONNECTION_STATE	2

/* BT LE Mode Changes */
/* BT LE mode DATA channel pkt types */
#define BT_LE_PKT_0		0
#define BT_LE_PKT_1		1
#define BT_LE_PKT_2		2
#define BT_LE_PKT_3		3

/* BT LE mode DATA pkt types */
#define BT_LE_DATA_DATA		0
#define BT_LE_DATA_CTL		1

/* BT LE mode DATA_DATA LLID_control packets */
#define BT_LE_DATA_DATA_PKT_0	0
#define BT_LE_DATA_DATA_PKT_1	1
#define BT_LE_DATA_DATA_PKT_2	2

/* BT LE E2E Device States */
#define BT_LE_STAND_BY		0
#define BT_LE_SCAN		1
#define BT_LE_ADV		2
#define BT_LE_INITIATOR		3
#define BT_LE_CONN		4
/* End of BT_LE E2E device states */

/* Advertising Channel PDU types */
//#define ADV_IND			0
//#define ADV_DIRECT_IND		1
//#define ADV_NONCONN_IND		2
#define SCAN_REQ		3
#define SCAN_RSP		4
#define CONNECT_REQ		5
#define ADV_DISCOVER_IND	6

/* BT LE mode mgmt pkt types */
#define BT_LE_ADV_IND		0x0021
#define BT_LE_ADV_DIRECT_IND	0x0022
#define BT_LE_ADV_NONCONN_IND	0x0023
#define BT_LE_SCAN_REQ		0x0024
#define BT_LE_SCAN_RSP		0x0025
#define BT_LE_CONNECT_REQ	0x0026
#define BT_LE_ADV_DISCOVER_IND	0x0027
/* End of Advertising Channel PDU types */

/* Data Channel LLID types */
#define LLID_0			0
#define LLID_1			1
#define LLID_2			2
#define LLID_3			3

/* Opcode type decoding */
#define LL_CONNECTION_UPDATE_REQ	0
#define LL_CHANNEL_MAP_REQ		1
#define LL_TERMINATE_IND		2
#define LL_ENC_REQ			3
#define LL_ENC_RSP			4
#define LL_START_ENC_REQ		5
#define LL_START_ENC_RSP		6
#define LL_UNKNOWN_RSP			7
#define LL_FEATURE_REQ			8
#define LL_FEATURE_RSP			9
#define LL_PAUSE_ENC_REQ		10
#define LL_PAUSE_ENC_RSP		11
#define LL_VERSION_IND			12
#define LL_REJECT_IND			13


ONEBOX_STATUS usb_master_reg_write (PONEBOX_ADAPTER adapter, unsigned long reg, unsigned long value, uint16 len);
ONEBOX_STATUS sdio_master_reg_write (PONEBOX_ADAPTER adapter, unsigned long reg, unsigned long value, uint16 len);
ONEBOX_STATUS usb_master_reg_read (PONEBOX_ADAPTER adapter, uint32 reg, uint32 * value, uint16 len);
ONEBOX_STATUS sdio_master_reg_read (PONEBOX_ADAPTER adapter, uint32 reg, uint32 * value, uint16 len);
ONEBOX_STATUS onebox_sdio_master_access_msword(PONEBOX_ADAPTER adapter,
                                               uint16 ms_word);
ONEBOX_STATUS onebox_read_cardinfo(PONEBOX_ADAPTER adapter);
int32 onebox_load_lmac_instructions(PONEBOX_ADAPTER adapter);
int onebox_mgmt_pkt_to_core(PONEBOX_ADAPTER adapter,
                            uint8 *msg,
                            int32 msg_len,
                            uint8 type);
int32 onebox_mgmt_pkt_recv(PONEBOX_ADAPTER adapter, uint8 *msg);
int32 onebox_process_coex_rx_pkt(PONEBOX_ADAPTER adapter, uint8 *msg);
ONEBOX_STATUS program_bb_rf(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS set_cw_mode(PONEBOX_ADAPTER adapter, uint8 mode);
ONEBOX_STATUS eeprom_read(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS manual_flash_write(PONEBOX_ADAPTER adaprer);
ONEBOX_STATUS auto_flash_write(PONEBOX_ADAPTER adaprer, uint8 *, uint32);
ONEBOX_STATUS auto_load_flash_content(PONEBOX_ADAPTER adapter, METADATA metadata_rcv);
ONEBOX_STATUS manual_load_flash_content(PONEBOX_ADAPTER adapter, METADATA metadata_rcv);
ONEBOX_STATUS flash_read(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS send_flashing_status(PONEBOX_ADAPTER adapter, uint8 status);
ONEBOX_STATUS auto_fw_upgrade(PONEBOX_ADAPTER adapter, uint8 *flash_content, uint32 content_size);
ONEBOX_STATUS band_check(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS set_bb_rf_values(PONEBOX_ADAPTER adapter, struct iwreq *wrq);
ONEBOX_STATUS update_device_op_params(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS start_tx_rx(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS start_autorate_stats(PONEBOX_ADAPTER adapter);
void coex_sched_pkt_xmit(PONEBOX_ADAPTER adapter);
void onebox_init_chan_pwr_table(PONEBOX_ADAPTER adapter,
                                uint16   *bg_power_set,
                                uint16    *an_power_set);

ONEBOX_STATUS set_channel(PONEBOX_ADAPTER adapter, uint16 chno);
ONEBOX_STATUS bb_reset_req(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS read_reg_parameters (PONEBOX_ADAPTER adapter);
int8 calculate_rssi(PONEBOX_ADAPTER adapter ,uint8 bb_lna,uint8 average);


ONEBOX_STATUS onebox_load_radio_caps(PONEBOX_ADAPTER adapter);
uint8 onebox_qspi_bootup_params(PONEBOX_ADAPTER adapter);

PWR_STATUS onebox_load_deep_sleep (PONEBOX_ADAPTER adapter, uint16 cmd);
ONEBOX_STATUS onebox_send_reset_mac(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS onebox_send_per_ampdu_indication_frame(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS onebox_send_sleep_vals(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS onebox_coex_mgmt_frame(PONEBOX_ADAPTER adapter, uint16 *addr, uint16 len);
ONEBOX_STATUS onebox_rf_loopback(PONEBOX_ADAPTER adapter,
                                             uint16 *bb_prog_vals,
                                             uint16 num_of_vals,
					     uint8 type);
ONEBOX_STATUS onebox_ant_sel(PONEBOX_ADAPTER adapter, uint8 value, uint8 frame_type);
void onair_mgmt_dump( PONEBOX_ADAPTER adapter, netbuf_ctrl_block_t *netbuf_cb, uint8 extnd_size);
void onebox_internal_pkt_dump(PONEBOX_ADAPTER adapter, netbuf_ctrl_block_t *netbuf_cb);
void recv_onair_dump(PONEBOX_ADAPTER adapter, uint8 *buffer, uint32 len);
ONEBOX_STATUS common_hal_init(struct driver_assets *d_assets, PONEBOX_ADAPTER adapter);
void common_hal_deinit(struct driver_assets *d_assets, PONEBOX_ADAPTER adapter);
ONEBOX_STATUS onebox_configure_common_dev_params(PONEBOX_ADAPTER adapter);
void send_coex_configuration(PONEBOX_ADAPTER adapter,coex_cmd_t *coex_cmd_p );
ONEBOX_STATUS onebox_wait_for_rf_prog_frame_rsp(struct driver_assets *d_assets);


//ONEBOX_STATUS onebox_umac_init_done (PONEBOX_ADAPTER adapter);
#endif
