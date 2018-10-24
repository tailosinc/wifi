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
#ifndef _BT_UTIL_H_
#define _BT_UTIL_H_

#include <arpa/inet.h>
#include  "../rsi_nl_app.h"

/*Link Type*/
#define SCO_LINK    0
#define ACL_LINK    1
#define ESCO_LINK   2

/*Data Rate */
#define  ONE_MBPS   1
#define  TWO_MBPS   2
#define  THREE_MBPS 3

/* Hopping Mode */
#define  FREQ_NO_HOP     0
#define  FREQ_FIXED_HOP  1
#define  FREQ_RAND_HOP   2

/* LE Phy Rate */
#define  BLE_1MBPS     1
#define  BLE_2MBPS     2
#define  BLR_500KBPS   4
#define  BLR_125KBPS   8


/* BT Operating Modes */
#define BT_CLASSIC_MODE  1
#define BT_LE_MODE       2
#define BT_DUAL_MODE     3

/* BT Classic rf channels */
#define BT_CLASSIC_START_CHANNEL 0
#define BT_CLASSIC_END_CHANNEL   79

/* BT LE rf channels */
#define BT_LE_START_CHANNEL 0
#define BT_LE_END_CHANNEL   39

/* BT Classic Packet Types */
#define BT_DM1_PKT_TYPE   0x3
#define BT_DH1_PKT_TYPE   0x4
#define BT_DM3_PKT_TYPE   0xA
#define BT_DH3_PKT_TYPE   0xB
#define BT_DM5_PKT_TYPE   0xE
#define BT_DH5_PKT_TYPE   0xF
#define BT_2DH1_PKT_TYPE  0x4
#define BT_2DH3_PKT_TYPE  0xA
#define BT_2DH5_PKT_TYPE  0xE
#define BT_3DH1_PKT_TYPE  0x8
#define BT_3DH3_PKT_TYPE  0xB
#define BT_3DH5_PKT_TYPE  0xF
#define BT_HV1_PKT_TYPE   0x5
#define BT_HV2_PKT_TYPE   0x6
#define BT_HV3_PKT_TYPE   0x7
#define BT_DV_PKT_TYPE    0x8
#define BT_EV3_PKT_TYPE   0x7
#define BT_2EV3_PKT_TYPE  0x6
#define BT_3EV3_PKT_TYPE  0x7
#define BT_EV4_PKT_TYPE   0xC
#define BT_2EV5_PKT_TYPE  0xC
#define BT_EV5_PKT_TYPE   0xD
#define BT_3EV5_PKT_TYPE  0xD

/* BT Classic Max Packet Lengths */
#define BT_DM1_PAYLOAD_MAX_LEN 17
#define BT_DM3_PAYLOAD_MAX_LEN 121
#define BT_DM5_PAYLOAD_MAX_LEN 224
#define BT_DH1_PAYLOAD_MAX_LEN 27
#define BT_DH3_PAYLOAD_MAX_LEN 183
#define BT_DH5_PAYLOAD_MAX_LEN 339
#define BT_2DH1_PAYLOAD_MAX_LEN 54
#define BT_2DH3_PAYLOAD_MAX_LEN 367
#define BT_2DH5_PAYLOAD_MAX_LEN 679
#define BT_3DH1_PAYLOAD_MAX_LEN 83
#define BT_3DH3_PAYLOAD_MAX_LEN 552
#define BT_3DH5_PAYLOAD_MAX_LEN 1021
#define BT_HV1_VOICE_PAYLOAD_LEN  10
#define BT_HV2_VOICE_PAYLOAD_LEN  20
#define BT_HV3_VOICE_PAYLOAD_LEN  30
#define BT_EV3_VOICE_PAYLOAD_LEN  30
#define BT_2EV3_VOICE_PAYLOAD_LEN 60
#define BT_3EV3_VOICE_PAYLOAD_LEN 90
#define BT_EV4_VOICE_PAYLOAD_LEN 120
#define BT_EV5_VOICE_PAYLOAD_LEN 180
#define BT_2EV5_VOICE_PAYLOAD_LEN 360
#define BT_3EV5_VOICE_PAYLOAD_LEN 540

#define DISABLE_SKIP_PROG 0
#define ENABLE_SKIP_PROG  1

#define DISABLE_LOOP_BACK 0
#define ENABLE_LOOP_BACK  1

#define PLL_MODE0 0
#define PLL_MODE1 1
#define PLL_MODE2 2

#define EXTERNAL_RF 0
#define INTERNAL_RF 1


#define WLAN_HP_CHAIN             0
#define WLAN_LP_CHAIN             1
#define BT_LP_CHAIN               2
#define BT_HP_CHAIN               3


/* Duty Cycle Defines */
#define DISABLE_DUTY_CYCLE 0
#define ENABLE_DUTY_CYCLE  1
struct get_info {
	u_int8_t param_name[16];
	u_int8_t param_length;
	u_int8_t *data;
};

typedef struct bt_inf_s
{
	unsigned char hci_name[10];	
	unsigned char flag;
	struct iw_point	data;		/* Other large parameters */
}bt_inf_t;


typedef struct bt_per_stats_s
{
	unsigned short crc_fail;
	unsigned short crc_pass;
	unsigned short tx_aborts;
	unsigned short cca_stk;
	unsigned short cca_not_stk;
	unsigned short fls_rx_start;
	unsigned short cca_idle;
	unsigned short tx_dones;
	signed short int rssi;
	unsigned short id_pkts_rcvd;
}bt_per_stats_t;

typedef struct bt_ber_stats_s {
	unsigned short length;
	unsigned char data[1032];
	unsigned short num_pkts;
}__attribute__((packed)) bt_ber_stats_t;

typedef struct bt_ber_pkt_cnt_s{
		unsigned short num_pkts;
}__attribute__((packed)) bt_ber_pkt_cnt_t;


typedef struct  __attribute__((__packed__)) bt_per_params_s
{
	unsigned char  enable;
	unsigned char bt_addr[6];
	unsigned char pkt_type;
	unsigned short int  pkt_length;
	unsigned char link_type;
	unsigned char edr_indication;
	unsigned char bt_rx_rf_chnl;
	unsigned char bt_tx_rf_chnl;
	unsigned char scrambler_seed;
	unsigned int num_pkts;
	unsigned char payload_data_type;
	unsigned char mode;
	unsigned char le_chnl;
	unsigned char tx_pwr_indx;
	unsigned char transmit_mode;
	unsigned char enable_hopping;
	unsigned char ant_select;
}bt_per_params_t;

typedef struct  __attribute__((__packed__)) bt_afh_params_s
{
	unsigned char protocol_mode;
	unsigned char afh_map[10];
}bt_afh_params_t;

typedef struct  __attribute__((__packed__)) le_afh_params_s
{
	unsigned char protocol_mode;
	unsigned char afh_map[5];
}le_afh_params_t;

typedef struct  __attribute__((__packed__)) bt_br_edr_per_params_s
{
  unsigned char  enable;
  unsigned char bt_addr[6];
  unsigned short int  pkt_length;
  unsigned char pkt_type;
  unsigned char br_edr_indication;
  unsigned char bt_rx_rf_chnl;
  unsigned char bt_tx_rf_chnl;
  unsigned char link_type;	
  unsigned char scrambler_seed;
  unsigned char enable_hopping;
  unsigned char ant_select;
  unsigned char pll_mode;
  unsigned char rf_type;
  unsigned char rf_chain;
  unsigned char payload_data_type;
  unsigned char tx_pwr_indx;	
  unsigned char transmit_mode;
  unsigned char inter_pkt_gap;
  unsigned int num_pkts;
}bt_br_edr_per_params_t;

typedef struct  __attribute__((__packed__)) ble_per_params_s
{
  unsigned char  enable;
  unsigned char access_addr[4];
  unsigned char ble_rate;
  unsigned char bt_rx_rf_chnl;
  unsigned char bt_tx_rf_chnl;
  unsigned char scrambler_seed;
  unsigned char le_chnl;
  unsigned char hopping_type;
  unsigned char ant_select;	
  unsigned char pll_mode;
  unsigned char rf_type;
  unsigned char rf_chain;
  unsigned short int  pkt_length;
  unsigned char payload_data_type;
  unsigned char tx_pwr_indx;
  unsigned char transmit_mode;
  unsigned char inter_pkt_gap;
  unsigned int num_pkts;
}ble_per_params_t;

typedef struct  __attribute__((__packed__)) bt_per_recv_params_s
{
  unsigned char  enable;
  unsigned char bt_addr[6];
  unsigned char link_type;
  unsigned char pkt_type;
  unsigned short int pkt_length;
  unsigned char scrambler_seed;
  unsigned char edr_indication;
  unsigned char bt_rx_rf_chnl;
  unsigned char bt_tx_rf_chnl;
  unsigned char le_mode;
  unsigned char le_chnl;
  unsigned char enable_hopping;
  unsigned char ant_sel;
}bt_per_recv_params_t;

typedef struct  __attribute__((__packed__)) bt_br_edr_per_recv_params_s
{
  unsigned char  enable;
  unsigned char bt_addr[6];
  unsigned short int pkt_length;
  unsigned char pkt_type;
  unsigned char br_edr_indication;
  unsigned char bt_rx_rf_chnl;
  unsigned char bt_tx_rf_chnl;
  unsigned char link_type;
  unsigned char scrambler_seed;
  unsigned char enable_hopping;
  unsigned char ant_sel;
  unsigned char pll_mode;
  unsigned char rf_type;
  unsigned char rf_chain;
  unsigned char loop_back_mode;
}bt_br_edr_per_recv_params_t;

typedef struct  __attribute__((__packed__)) ble_per_recv_params_s
{
  unsigned char  enable;
  unsigned char access_addr[4];
  unsigned char ble_rate;
  unsigned char bt_rx_rf_chnl;
  unsigned char bt_tx_rf_chnl;
  unsigned char scrambler_seed;
  unsigned char le_chnl_type;
  unsigned char freq_hop_en;
  unsigned char ant_sel;
  unsigned char pll_mode;
  unsigned char rf_type;
  unsigned char rf_chain;
  unsigned char data_length_indication;
  unsigned char loop_back_mode;
  unsigned char pwrsave_options;
}ble_per_recv_params_t;


typedef struct bt_per_packet_s
{
	unsigned char  enable;
	unsigned int  length;
	unsigned char packet[1024];
}bt_per_packet_t;


typedef struct  __attribute__((__packed__)) ble_aoa_aod_transmit_per_params_s
{
  unsigned char enable;
  unsigned char pkt_length;
  unsigned char phy_rate;
  unsigned char bt_tx_rf_chnl;
  unsigned char scrambler_seed;
  unsigned char payload_data_type;
  unsigned char supp_length;
  unsigned char supp_slot_type;
  unsigned char num_antenna;
  unsigned char ant_switch_pattern;
  unsigned char le_chnl;
  unsigned char tx_pwr_indx;
  unsigned char enable_hopping;
  unsigned char ant_select;
}ble_aoa_aod_transmit_per_params_t;

typedef struct  __attribute__((__packed__)) ble_aoa_aod_receive_per_params_s
{
  unsigned char enable;
  unsigned char phy_rate;
  unsigned char bt_rx_rf_chnl;
  unsigned char scrambler_seed;
  unsigned char supp_length;
  unsigned char supp_slot_type;
  unsigned char num_antenna;
  unsigned char ant_switch_pattern;
  unsigned char le_chnl;
  unsigned char enable_hopping;
  unsigned char ant_select;
}ble_aoa_aod_receive_per_params_t;

#define ONEBOX_PRINT(fmt, args...) fprintf(stdout, fmt, ## args)
#define ONEBOX_PRINT_INFO(a, fmt) \
	if(a)\
		printf(fmt);


/* Function prototypes */
void usage();
void byteconversion(char *src,char *macaddr);
int getcmdnumber(char *command);
void get_driver_state(char *state);
int bt_ber(void);
int bt_stats(unsigned char *);
void mapconversion (char *src, char *bitmap, int length);

inline void byteconversion (char *src, char *macaddr)
{
	int ii;
	unsigned char temp[18];

	for (ii = 0; *src != '\0'; ii++)
	{
		if (*src == ':')
		{
			src++;
		}
		if ((*src >= '0') && (*src <= '9'))
		{
			*src -= '0';
			temp[ii] = *src;
		}
		else if ((*src >= 'a') && (*src <= 'f'))
		{
			*src -= 'a';
			*src += 0xa;
			temp[ii] = *src;
		}
		else if ((*src >= 'A') && (*src <= 'F'))
		{
			*src -= 'A';
			*src += 0xa;
			temp[ii] = *src;
		}
		else
		{
			ONEBOX_PRINT ("Invalid macaddr\n");
			exit (1);
		}
		src++;
	}
  if( ii != 12 )
  {
    ONEBOX_PRINT ("Invalid macaddr\n");
    exit (1);
  }
	for (ii = 0; ii < 12;)
	{
		*macaddr = (temp[ii] << 4) | (temp[ii + 1]);
		ii = ii + 2;
		macaddr++;
	}
}

void mapconversion (char *src, char *bitmap, int length)
{
	int ii;
	unsigned char temp[30];
        memset(temp, 0, 30);
	for (ii = 0; *src != '\0'; ii++)
	{
		if (*src == ':')
		{
			src++;
		}
		if ((*src >= '0') && (*src <= '9'))
		{
			*src -= '0';
			temp[ii] = *src;
		}
		else if ((*src >= 'a') && (*src <= 'f'))
		{
			*src -= 'a';
			*src += 0xa;
			temp[ii] = *src;
		}
		else if ((*src >= 'A') && (*src <= 'F'))
		{
			*src -= 'A';
			*src += 0xa;
			temp[ii] = *src;
		}
		else
		{
			ONEBOX_PRINT ("Invalid bitmap\n");
			exit (1);
		}
		src++;
	}
	for (ii = 0; ii < length;)
	{
		*bitmap = (temp[ii] << 4) | (temp[ii + 1]);
		ii = ii + 2;
		bitmap++;
	}
}




///***  MATLAB utils ****///

/*** Matlab To Driver Command Type Defines ***/

#define ONEBOX_STATUS_FAILURE           -1
#define ONEBOX_STATUS_SUCCESS           0
#define ONEBOX_STATUS                   int_32

#define ADDR0           0x315
#define ADDR1           0x316
#define ADDR2           0x317

#define BIT(n)         (1 << (n))

/* Common Frame nos for matlab_utils.c and bt_util.c */
#define BB_READ                         0x0
#define BB_WRITE                        0x1
#define RF_READ                         0x2
#define RF_WRITE                        0x3
#define BT_PER_TRANSMIT                 0x4
#define BT_RECEIVE			                0x5
#define BUFFER_READ                     0x6
#define BUFFER_WRITE                    0x7
#define BT_PER_STATS 			              0x8
#define ANT_SEL			                    0x9
#define BT_BER_PKT_CNT		              0xA
#define BT_BER_RECEIVE                  0xB
#define BT_BER_MODE			                0xC
#define BT_CW_MODE                      0xD
#define TX_STATUS                       0xE
#define GET_DRV_COEX_MODE               0xF
#define BT_PER_AFH_MAP					0x10
#define BT_STATS						0x11

#define BT_PER                          0x10
#define BT_BER                          0x11
#define BT_CW                           0x12

#define PER_BLE_TRANSMIT                0x13
#define PER_BLE_RECEIVE                 0x14
#define PER_BR_EDR_TRANSMIT             0x15
#define PER_BR_EDR_RECEIVE              0x16


#define BLE_AOA_AOD_TRANSMIT            0x17
#define BLE_AOA_AOD_RECEIVE             0x18

#define RSI_SET_BB_READ                 0x01
#define RSI_SET_BB_WRITE                0x02
#define RSI_RF_WRITE		                0x03    
#define RSI_RF_READ		                  0x04      
#define RSI_ANT_SEL		                  0x05
#define RSI_SET_CW_MODE                 0x06    
#define RSI_GET_BT_STATS                0x07
#define RSI_BT_BER                      0x08
#define RSI_SET_AFH_MAP					0x09

#define BT_CLASSIC                         1
#define BT_LE                              2


/*** Src Destn Port And IP Defines ***/
#define DESTN_PORT_NUM                  9999
#define SOURCE_PORT_NUM                 9999
#define DESTN_IP                        "192.168.70.222"
//#define DESTN_IP                        "127.0.0.1"
#define ONEBOX_MAX_PKT_LEN              6348 // Bytes
#define MAX_NUM_OF_PKTS                 100

#define IFNAMSIZ                        16


struct bb_rf_param_t
{
        unsigned char  value; //type is present here
        unsigned char  no_of_fields;
        unsigned char  no_of_values;
        unsigned char  soft_reset;
        unsigned short  Data[128];
} ;
     
#define FUNCTION_ENTRY()    NULL//            printf("+%s\n", __func__)
#define FUNCTION_EXIT()     NULL//            printf("-%s\n", __func__)

#ifdef CHIP_9116
int_32 afh_map(int classic_le_mode, int channel_bit_map);
#define IQ_BLOCK_SIZE   64
#else
int_32 afh_map(int start_channel, int end_channel);
#endif
#endif


