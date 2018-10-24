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

#include "wlan_common.h"
#include "onebox_hal.h"
#include "onebox_ps.h"

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

#define ONEBOX_PAD_HEADER         40

#define VAP_ADD 1
#define VAP_DELETE 2
#define VAP_UPDATE 3
/**
 * Management Frame Types
 */
/* Send Frames Types */
enum  cmd_frame_type 
{
 TX_DOT11_MGMT = 0,             
 RESET_MAC_REQ  = 0x01,
 RADIO_CAPABILITIES = 0x02,        
 BB_PROG_VALUES_REQUEST = 0x03,    
 RF_PROG_VALUES_REQUEST = 0x04 ,   
 WAKEUP_SLEEP_REQUEST = 0x05,      
 SCAN_REQUEST = 0x06,              
 TSF_UPDATE = 0x07,
 PEER_NOTIFY = 0x08,               
 BLOCK_UNBLOCK = 0x09,             
 SET_KEY = 10,                   
 AUTO_RATE_IND = 11,             
 BOOTUP_PARAMS_REQUEST = 12,     
 VAP_CAPABILITIES = 13,          
 EEPROM_READ_TYPE = 14,         
 EEPROM_WRITE = 15,              
 GPIO_PIN_CONFIG = 16,          
 SET_RX_FILTER = 17,             
 AMPDU_IND = 18,                 
 STATS_REQUEST_FRAME = 19,      
 BB_BUF_PROG_VALUES_REQ = 20,   
 BBP_PROG_IN_TA = 21,           
 BG_SCAN_PARAMS = 22,          
 BG_SCAN_PROBE_REQ = 23,         
 CW_MODE_REQ = 24,               
 PER_CMD_PKT = 25,
 DEV_SLEEP_REQUEST = 26,
 DEV_WAKEUP_CNF = 27,
 RF_LOOPBACK_REQ = 28,
 RF_LPBK_M3 = 29,
 RF_RESET_FRAME = 30,
 LMAC_REG_OPS = 31,
 ANT_SEL_FRAME = 32,
 CONFIRM = 33,
 WLAN_DE_REGISTER = 34,
 DEBUG_FRAME = 35,
 HW_BMISS_HANDLE = 36,
 MULTICAST_ENABLE = 37,
 TX_MISC_IND = 38, /* Reserved for common hal */
 VAP_DYNAMIC_UPDATE=39,
 COMMON_DEV_CONFIG= 40,
 RADIO_PARAMS_UPDATE=41,
 RADAR_REQUEST=42,
 WOWLAN_CONFIG_PARAMS=43,
 IAP_CONFIG = 44, /* For IAP chip communication */
 //MULTICAST_ENABLE = 64,
BEACON_RECV_DIS = 45,
 TX_POWER_REQUEST = 46,
 SCAN_PROGRESS_CHECK = 47,
 WLAN_IQ_STATS_REQ = 48,
#ifdef IEEE80211K
 SEND_MEAS_INFO = 49, 
#endif
 COEX_CONFIGURATIONS = 50,  //COEX SLOTTING CONTROL
 FEATURES_ENABLE = 51,    //WLAN_9116_FEATURES
 PROG_STRUCTURE_D = 56,
 GTK_REKEY_DATA = 57,
 DISABLE_PROGRAMMING_D = 65,
 LOG_STRUCT_PRGMG_D = 66,  //STRUCTURE PROGRAMMING STATS
};

/* MGMT Messages from TA */
typedef enum {
 CARD_READY_IND	= 0x0,
 TA_CONFIRM_TYPE = 0x01,
 RX_DOT11_MGMT = 0x02,
 RX_DOT11_DATA = 0x03,
 TX_STATUS_IND = 0x04,
 PS_NOTIFY_IND = 0x05,
 SLEEP_NOTIFY_IND = 0x06,
 DECRYPT_ERROR_IND = 0x07,
 BEACON_EVENT_IND = 0x08,
 DEBUG_IND = 0x09,
 RX_MISC_IND = 0xa,
 UNCONNECTED_PEER = 0xb,
 HW_BMISS_EVENT = 0xc,
 RATE_GC_TABLE_UPDATE = 0xd,
 RADAR_DETECTED = 0x0e,
#ifdef ONEBOX_CONFIG_GTK_OFFLOAD
 UPDATE_MCAST_PN = 0x1D,
#endif 
 TSF_SYNC_CONFIRM = 0xc0,
 ANTENNA_SELECT = 0xf,
 WLAN_IQ_STATS = 0x34,
#ifdef IEEE80211K
 MEASUREMENT_REPORT = 0x10,
#endif

} rx_cmd_type_t;

#define PUF_REQUEST_FRAME         52

/* Receive Frame Types */
#define RESULT_CONFIRM             0x80
#define MGMT_PPE_DEBUG_FRAME       0x82
#define MIC_FAILURE_FRAME          0x86
#define PER_STATS_PACKET           0x84

#define REQUIRE_TSF_SYNC_CONFIRM BIT(14)
#define EOSP_INDICATION          BIT(13)
#define REQUIRE_CONFIRM_TO_HOST  BIT(10)
#define ADD_DELTA_TSF_VAP_ID		 BIT(11)
#define FETCH_RETRY_CNT_FRM_HST	 BIT(12)

#define EAPOL_4_LEN 				113

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
#define STATION_STATISTICS_RESP      0x0F9E //
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

#define CIPHER_MISMATCH BIT(0)
#define ICV_ERROR   BIT(1)
#define MIC_FAILURE BIT(2)
#define BCAST_MIC   BIT(3)

/* Macro to indicate fw thatBeacon miss handling will be done by Hardware */
#define ONEBOX_HW_BMISS_HANDLE    ONEBOX_BIT(9)

/* Flag to check whether user wants S/W or HW bmiss handling*/
#define HW_BEACON_MISS_HANDLE     ONEBOX_BIT(0)
#define BB_SENS_PROG                 0x2

/* ampdu indication events */
#define START_AMPDU_AGGR  0x1
#define STOP_AMPDU_AGGR  0x0

/* Netbuf_cb flags */
#define ENCRYPTION_ENBL       ONEBOX_BIT(0)
#define INTERNAL_MGMT_PKT     ONEBOX_BIT(7)
#define MORE_DATA             ONEBOX_BIT(8)
#define FIRST_BCAST           ONEBOX_BIT(14)
#define LAST_BCAST            ONEBOX_BIT(15)


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
#define BW_10Mhz 2
#define BW_5Mhz	 3

#ifdef MODE_11AH
#define BW_2Mhz  2
#define BW_4Mhz 3
#endif


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
#define ONEBOX_FIXED_RATE_EN		     ONEBOX_BIT(0)

#define BBP_REMOVE_SOFT_RST_BEFORE_PROG  ONEBOX_BIT(0) //Take BBP out of reset
#define BBP_REMOVE_SOFT_RST_AFTER_PROG   ONEBOX_BIT(1) //Take BBP out of reset
#define BBP_REG_READ          ONEBOX_BIT(2)
#define PUT_BBP_RESET        0
#define BBP_REG_WRITE        0
#define BBP_REG_READ          ONEBOX_BIT(2)
#define RF_RESET_ENABLE       ONEBOX_BIT(3)
#define ULP_MODE              ONEBOX_BIT(0)


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

/*11AH Defines*/
#ifdef MODE_11AH

#define ONEBOX_ENABLE_11AH   ONEBOX_BIT(6)
#define ONEBOX_ENABLE_2MHZ 0x0
#define ONEBOX_ENABLE_4MHZ 0x8
#define FULL_2M_ENABLE  0x0
#define FULL_4M_ENABLE  0x6
#define FULL_8M_ENABLE  0xf


#endif

#define ONEBOX_ENABLE_20MHZ     0x0<<3//bandwidth
#define ONEBOX_ENABLE_40MHZ     0x1<<3
#define ONEBOX_ENABLE_80MHZ     0x2<<3

#define BROADCAST_IND        ONEBOX_BIT(9)
#define ENABLE_MAC_INFO      ONEBOX_BIT(0)
#define CONTINUOUS_MODE      ONEBOX_BIT(10)

#define EAPOL_RETRY_CNT 15 
#define PROBE_RESP_RETRY_CNT 3

#define RX_BA_INDICATION  1

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

/***MAC FLAGS in Frame DESC(16 bits) ***/
#define RATE_INFO_ENABLE          ONEBOX_BIT(0) 
/* Power save Indication flags for Broadcast Multicast Pkts*/
#define MORE_DATA_PRESENT         ONEBOX_BIT(1)
#define INSERT_SEQ_NO             ONEBOX_BIT(2)
#define ONEBOX_BROADCAST_PKT      ONEBOX_BIT(9)
#define DTIM_BEACON_GATED_FRAME   ONEBOX_BIT(10)
#define BEACON_FRAME              ONEBOX_BIT(11)
#define DTIM_BEACON               ONEBOX_BIT(10) | ONEBOX_BIT(11)
#define INSERT_TSF                ONEBOX_BIT(15)
/******End Of Mac Flags ***/

#define PROCESS_CONTEXT    0
#define ATOMIC_CONTEXT     1

#define WAC_TIMEOUT	30 * 60 * 1000	/* 30 minutes */

#define TID_MAPPING_AC(_tid) (      \
	((_tid) == 0 || (_tid) == 3) ? 0 : \
	((_tid) < 3) ? 1 : \
	((_tid) < 6) ? 5 : \
	6)

#define ACS_TIMEOUT_TIME        150
#define MAX_NUM_CHANNELS       39

typedef struct acs_stats_s{
	uint16  chan_busy_time;
	uint8   noise_floor_rssi;
}acs_stats_t;

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

typedef struct sta_cbl_s 
{
	int32 sta_id;
	uint32 max_amsdu_size;
	struct ieee80211_node *ni;
	uint8 mac_addr[ETH_ALEN];
}sta_cbl_t;

#define LP_SLEEP_TYPE 1
#define ULP_SLEEP_TYPE 2
struct sleep_params {
			uint8 ps_en;
			uint8 sleep_type; //LP or ULP type
			uint8 connected_sleep;
#ifdef DUTY_CYCLE_IOCTL
                        uint8 duty_cycle;
#else
			uint8 reserved1;
#endif
			uint16 num_beacons_per_listen_interval;
			uint16 wakeup_type;
			uint32 sleep_duration;
};

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
		struct
		{
			uint16 data[128];
		}lmac_prog_vals;
		struct 
		{
			uint16  reg_addr;
			uint16  bb_prog_vals;
		} bb_prog_req[100]; //: optimise this as some sets may not have the max vals

		struct 
		{
			uint16 rf_prog_vals[100]; //: optimise this as some sets may have less vals
		} rf_prog_req;

		struct 
		{
			uint16  bb_buf_vals[100];
		} bb_buf_req; //: optimise this as some sets may not have the max vals
		
		struct 
		{
			struct
			{
				uint16  cont_win_min_q;
				uint16  cont_win_max_q;
				uint16  aifsn_val_q;
				uint16  txop_q;
			}qos_params[MAX_HW_QUEUES];
		//Max rates should be defined as a Macro here
			uint8 num_11n_rates;
			uint8 num_11ac_rates;
			uint16 gcpd_per_rate[20];
			uint16 sifs_tx_11n;
			uint16 sifs_tx_11b;
			uint16 slot_rx_11n;
			uint16 ofdm_ack_tout;
			uint16 cck_ack_tout;
			uint16 preamble_type;
		}radio_caps;

		struct 
		{
			uint8 mac_addr[6];
			uint16 command;
			uint16 mpdu_density;
			uint16 reserved;
			uint32 sta_flags;
		}peer_notify;	
	
		struct 
		{
			uint8 mac_addr[6];
			uint16 keep_alive_period;
			uint8 bssid[6];
			uint16 reserved;
			uint32 flags;
			uint16 frag_threshold;
			uint16 rts_threshold;
			uint32 default_mgmt_rate_bbpinfo;
			uint32 default_ctrl_rate_bbpinfo;
			uint32 default_data_rate_bbpinfo;
			/* Beacon interval in terms of TU's (1024 usecs) */
			uint16 beacon_interval;
			uint16 dtim_period;
			uint16 beacon_miss_threshold;
		} vap_caps;
		struct
		{
			/* Frame body is not yet decided */	
		}ampdu_ind;
	
		struct
		{
			uint8 key[4][32];
			uint8 tx_mic_key[8];
			uint8 rx_mic_key[8];
		} set_key;
		struct
		{
			uint16 failure_limit;
			uint16 Initial_boundary;
			uint16 max_threshold_limt;
			uint16 num_supported_rates;
			uint16 aarf_rssi;
			uint16 moderate_rate_inx;
			uint16 collision_tolerance;
			uint16  supported_rates[40];
		}auto_rate;
		struct
		{
			uint16 bgscan_threshold;
			uint16 roam_threshold;
			uint16 bgscan_periodicity;
			uint8 num_bg_channels;
			uint8 two_probe;
			uint16 active_scan_duration;
			uint16 passive_scan_duration;
			uint16 channels2scan[MAX_NUM_SCAN_BGCHANS];
		}bgscan_params;
#ifdef PWR_SAVE_SUPPORT
		struct
		{
#if 0
			uint8 ps_en;
			uint8 sleep_type; //LP or ULP type
			uint8 connected_sleep;
			uint8 reserved1;
			uint16 listen_interval;
			uint16 wakeup_type;
			uint32 sleep_duration;
#endif
			struct sleep_params ps_req; 
			//uint16 reserved2;
			uint8 mimic_support;
			uint8 uapsd_acs;
			uint8 uapsd_wakeup_period;
			uint8 reserved;
			uint32 listen_interval;
			uint32 dtim_interval_duration;
			uint16 num_dtim_intervals;
		}ps_req_params;
#endif
		struct
		{
			uint16 mgmt_rate;
			uint16 flags;/**bit 0 indicates scan mode
										 *bit 1 indicates join mode
										 *bit 2 indicates RF Type
										 *bit 3 indicates Manual Bg scan
										 *bit 4 indicates Instant Bg scan
										 *bit 5 Indicates Snd multiprobe
										 *bit 6 CAC for DFS// Now we are not using
										 *bit 7 DFS_CH_Active scan
										 *Remainig bits are reserved
										 */
			uint16 channel_num;
			uint16 channel_scan_time;
			uint16 probe_req_length;
		}bgscan_probe;

    struct
    {
      uint8 macaddr[6];
      uint16 flags;
      uint16 host_wakeup_state;
    }wowlan_params;

#ifdef ONEBOX_CONFIG_GTK_OFFLOAD
    struct
    {
      uint8 kek[IEEE80211_KEK_LEN];
      uint8 kck[IEEE80211_KCK_LEN];
      uint8 replay_ctr[IEEE80211_REPLAY_CTR_LEN];
    }gtk_rekey_data;
#endif

#ifdef ONEBOX_CONFIG_PUF	  
    struct
    {
      uint8 puf_sub_cmd;
      uint8 key_index;
      uint8 key_size;
      uint8 key[MAX_KEY_SIZE];
    }puf_set_key_params;

    struct
    {
      uint8 puf_sub_cmd;
      uint8 key_code[KEY_CODE_SIZE];
      uint8 key_holder;
    }puf_get_key_params;
#endif
	
		CONFIG_VALS bootup_params;
		CONFIG_VALS bootup_params_9116;
#ifdef PWR_SAVE_SUPPORT
//		pwr_save_params ps_params;
#endif
	} u;
} onebox_mac_frame_t;

typedef struct xtended_frame_desc_s {
		uint8 confirm_frame_type;
		uint8 retry_cnt;
		uint16 reserved;
}xtended_desc_t;

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


typedef struct features_enable_s {
  unsigned char pll_mode;
  unsigned char rf_type;
  unsigned char wireless_mode;
  unsigned char enable_ppp;
  unsigned char afe_type;
  unsigned char reserved;
  uint16 reserved1;
#define DUTY_CYCLING            BIT(0)
#define END_OF_FRAME            BIT(1)
#define SIFSTRANSMITENABLE      BIT(2)
#define DPD                     BIT(3)
#define STANDBY_ASSOC_LP_CHAIN  BIT(4)
#define LMAC_BEACON_DROP        BIT(5)
#define PPE_DMEM_WRITE_FEATURE  BIT(6)
#define DROP_BYTES_FEATURE      BIT(7)
  uint32 feature_enable;
} features_enable_t;
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

int32 load_ta_instructions(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS onebox_wlan_sdio_master_access_msword(WLAN_ADAPTER w_adapter,
                                               uint16 ms_word);
ONEBOX_STATUS onebox_read_cardinfo(WLAN_ADAPTER w_adapter);
int32 onebox_load_lmac_instructions(WLAN_ADAPTER w_adapter);
int onebox_mgmt_pkt_to_core(WLAN_ADAPTER w_adapter,
                           netbuf_ctrl_block_t *netbuf_cb,
                            int32 msg_len,
                            uint8 type);
int32 onebox_mgmt_pkt_recv(WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb);
ONEBOX_STATUS program_bb_rf(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS set_cw_mode(WLAN_ADAPTER w_adapter, uint8 mode);
ONEBOX_STATUS eeprom_read(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS get_tx_power(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS flash_write(WLAN_ADAPTER adaprer);
ONEBOX_STATUS band_check(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS set_bb_rf_values(WLAN_ADAPTER w_adapter, struct iwreq *wrq);
ONEBOX_STATUS update_device_op_params(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS start_tx_rx(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS start_autorate_stats(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS	process_eeprom_write(WLAN_ADAPTER w_adapter);
int send_qos_conf_frame(WLAN_ADAPTER w_adapter, struct chanAccParams wme);
void schedule_pkt_for_tx(WLAN_ADAPTER w_adapter);
void onebox_init_chan_pwr_table(WLAN_ADAPTER w_adapter,
                                uint16   *bg_power_set,
                                uint16    *an_power_set);

ONEBOX_STATUS set_vap_capabilities(struct ieee80211vap *vap, uint8 status);
ONEBOX_STATUS set_beacon_ssid_notification(struct ieee80211vap *vap, uint8 loaded);
ONEBOX_STATUS set_channel_change_notification(struct ieee80211vap *vap, uint8 change);
//ONEBOX_STATUS set_vap_capabilities(WLAN_ADAPTER w_adapter, int unit, 
	//	enum ieee80211_opmode opmode, uint8 *bssid);
ONEBOX_STATUS hal_load_key(WLAN_ADAPTER w_adapter,
                           uint8 *data, 
                           uint16 key_len, 
                           uint16 offset,
                           uint8 key_type,
                           uint8 key_id,
			   uint32 cipher,
			   struct ieee80211vap *vap);
ONEBOX_STATUS set_channel(WLAN_ADAPTER w_adapter, uint16 chno);
ONEBOX_STATUS bb_reset_req(WLAN_ADAPTER w_adapter);
//ONEBOX_STATUS read_reg_parameters (WLAN_ADAPTER w_adapter);
int8 calculate_rssi(WLAN_ADAPTER w_adapter ,uint8 bb_lna,uint8 average);


ONEBOX_STATUS onebox_load_radio_caps(WLAN_ADAPTER w_adapter);
uint8 onebox_load_bootup_params(WLAN_ADAPTER w_adapter);

PWR_STATUS onebox_load_deep_sleep (WLAN_ADAPTER w_adapter, uint16 cmd);
ONEBOX_STATUS onebox_send_reset_mac(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS onebox_umac_init_done (WLAN_ADAPTER w_adapter);
ONEBOX_STATUS onebox_send_ampdu_indication_frame(WLAN_ADAPTER w_adapter, struct ieee80211_node *ni, uint8 event);
ONEBOX_STATUS onebox_send_vap_dynamic_update_indication_frame(struct ieee80211vap *vap);
ONEBOX_STATUS onebox_send_per_ampdu_indication_frame(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS onebox_send_sleep_vals(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS onebox_send_internal_mgmt_frame(WLAN_ADAPTER w_adapter, uint16 *addr, uint16 len);
ONEBOX_STATUS onebox_send_bgscan_params(struct ieee80211vap *vap, uint16 *data, uint8 default_val);
ONEBOX_STATUS send_bgscan_probe_req( WLAN_ADAPTER w_adapter, struct ieee80211_node *ni, uint16 cmd_flags);
ONEBOX_STATUS onebox_rf_loopback(WLAN_ADAPTER w_adapter,
                                             uint16 *bb_prog_vals,
                                             uint16 num_of_vals,
					     uint8 type);
ONEBOX_STATUS onebox_ant_sel(WLAN_ADAPTER w_adapter, uint8 value, uint8 frame_type);
ONEBOX_STATUS onebox_do_master_ops(WLAN_ADAPTER w_adapter, struct master_params_s *master , uint16 type);
ONEBOX_STATUS onebox_send_debug_frame(WLAN_ADAPTER w_adapter, struct test_mode *test);
ONEBOX_STATUS onebox_conf_beacon_recv(WLAN_ADAPTER w_adapter, uint8 value);
#ifdef RADAR_AUTO
ONEBOX_STATUS onebox_send_radar_req_frame(WLAN_ADAPTER w_adapter, uint8 radar_req, uint8 intr_clr);
#endif
ONEBOX_STATUS check_scan(WLAN_ADAPTER w_adapter);
#ifdef BYPASS_TX_DATA_PATH
ONEBOX_STATUS onebox_send_block_unblock(struct ieee80211vap *vap, uint8 notify_event, uint8 quiet_enable);
#endif
void onair_mgmt_dump( WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb, uint8 extnd_size);
void onebox_internal_pkt_dump(WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb);
void recv_onair_dump(WLAN_ADAPTER w_adapter, uint8 *buffer, uint32 len);
#ifdef ONEBOX_CONFIG_WOWLAN
ONEBOX_STATUS onebox_send_wowlan_params(struct ieee80211vap *vap,
					WLAN_ADAPTER w_adapter);
#endif
ONEBOX_STATUS onebox_send_w_9116_features(WLAN_ADAPTER w_adapter);
ONEBOX_STATUS onebox_send_structure_prog_stats_request(WLAN_ADAPTER w_adapter, programming_stats_t *programming_stats);
#endif
