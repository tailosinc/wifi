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
 SET_KEY,                   
 AUTO_RATE_IND,             
 BOOTUP_PARAMS_REQUEST,     
 VAP_CAPABILITIES,          
 EEPROM_READ_TYPE ,         
 EEPROM_WRITE,              
 GPIO_PIN_CONFIG ,          
 SET_RX_FILTER,             
 AMPDU_IND,                 
 STATS_REQUEST_FRAME ,      
 BB_BUF_PROG_VALUES_REQ ,   
 BBP_PROG_IN_TA ,           
 BG_SCAN_PARAMS  ,          
 BG_SCAN_PROBE_REQ,         
 CW_MODE_REQ,               
 PER_CMD_PKT,
 DEV_SLEEP_REQUEST,
 DEV_WAKEUP_CNF,
 RF_LOOPBACK_REQ,
 RF_LPBK_M3,
 RF_RESET_FRAME,
 LMAC_REG_OPS,
 ANT_SEL_FRAME,
 CONFIRM,
 WLAN_DE_REGISTER,
 DEBUG_FRAME,
 HW_BMISS_HANDLE,
 COEX_CONFIGURATIONS = 50,
};

/* Receive Frame Types */
#define RESULT_CONFIRM             0x80
#define MGMT_PPE_DEBUG_FRAME       0x82
#define MIC_FAILURE_FRAME          0x86
#define PER_STATS_PACKET           0x84

#define BT_DEREGISTER		   0x11
#define BT_DEFAULT_PS_REQ  0x12

#define EAPOL4_CONFIRM						1
#define PROBEREQ_CONFIRM					2
#define NULLDATA_CONFIRM					3


#define LMAC_FW_IND       0xfe
#define AUTO_RATE_STATS   0x98
#define SCANREQ_CONFIRM						6

/* MGMT Messages from TA */
#define CARD_READY_IND    				 0x0000
#define TA_CONFIRM_TYPE            0x01
#define RX_DOT11_MGMT              0x02
#define RX_DOT11_DATA              0x03
#define TX_STATUS_IND              0x04
#define PS_NOTIFY_IND              0x05
#define SLEEP_NOTIFY_IND           0x06
#define DECRYPT_ERROR_IND          0x07
#define BEACON_EVENT_IND           0x08
#define DEBUG_IND	          0x09

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

#define BAND_2_4GHZ 0
#define BAND_5GHZ   1

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

/* End of Data Channel LLID types */
/**/
#if 0
/* Bluetooth related frames */
int32 onebox_send_reset_mac(BT_ADAPTER bt_adapter);
int32 onebox_send_bt_mode_config(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_send_bt_state_config(BT_ADAPTER bt_adapter, 
				    		uint16 esco_pl_reg,
				    		uint32 LAP,
				    		uint32 UNAP,
				    		uint8 dev_state,
				    		uint32 clk_offset);
ONEBOX_STATUS onebox_send_bt_state_config_per(BT_ADAPTER bt_adapter); 
ONEBOX_STATUS onebox_send_bt_state_config_le(BT_ADAPTER bt_adapter,
                                                uint8 dev_state); 

ONEBOX_STATUS onebox_send_fhs_pkt(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_send_poll_pkt(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_send_null_pkt(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_send_dh1_pkt(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_process_dm1_pkt(BT_ADAPTER bt_adapter, uint8 *mpkt);
                         
/* LMP MESSAGES */
#define LMP_NAME_REQ              1
#define LMP_NAME_RES              2
#define LMP_ACCEPTED			  3
#define LMP_NOT_ACCEPTED		  4
#define LMP_CLK_OFFSET_REQ	 	  5
#define LMP_CLK_OFFSET_RES	  	  6
#define LMP_DETACH		  		  7
#define LMP_PARK_REQ		 	  25
#define LMP_FEATURES_REQ          39
#define LMP_FEATURES_RES          40
#define LMP_MAX_SLOT		      45
#define LMP_MAX_SLOT_REQ	      46
#define LMP_HOST_CONN_REQ         51
#define LMP_SLOT_OFFSET		      52

/* LMP Message Pkts added */
ONEBOX_STATUS onebox_send_lmp_feature_res(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_send_lmp_feature_req(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_send_lmp_name_req(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_send_lmp_name_res(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_send_lmp_host_conn_req(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_send_bt_le_connect_req(BT_ADAPTER bt_adapter);


VOID onebox_bt_pkt_rcvd(BT_ADAPTER bt_adapter,uint8  *pkt,uint16 pktLen);
VOID onebox_le_bt_pkt_rcvd(BT_ADAPTER bt_adapter,uint8  *pkt,uint16 pktLen);
ONEBOX_STATUS onebox_send_le_ll_data(BT_ADAPTER bt_adapter);

/* LE pkt template loading */
ONEBOX_STATUS onebox_send_bt_load_pkt_templates(BT_ADAPTER bt_adapter, uint8 pkt_type); 
ONEBOX_STATUS onebox_load_le_adv_ind(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_load_le_scan_req(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_load_le_scan_rsp(BT_ADAPTER bt_adapter);
ONEBOX_STATUS onebox_load_le_connect_req(BT_ADAPTER bt_adapter;
#endif
int onebox_mgmt_pkt_to_core(BT_ADAPTER bt_adapter,
                            netbuf_ctrl_block_t *netbuf_cb,
                            int32 msg_len,
                            uint8 type);
int32 onebox_bt_mgmt_pkt_recv(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb);
void onebox_internal_pkt_dump(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb);
#endif
