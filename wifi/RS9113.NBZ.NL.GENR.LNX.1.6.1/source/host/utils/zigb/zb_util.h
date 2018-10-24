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

#ifndef _ZB_UTIL_H_
#define _ZB_UTIL_H_

#include <arpa/inet.h>
#include  "../rsi_nl_app.h"

#define ONEBOX_STATUS_FAILURE           -1
#define ONEBOX_STATUS_SUCCESS           0
#define ONEBOX_STATUS                   int_32


/* Common Frame nos for matlab_utils.c and bt_util.c */
#define BB_READ                         0x0
#define BB_WRITE                        0x1
#define RF_READ                         0x2
#define RF_WRITE                        0x3
#define ZIGB_PER_TRANSMIT               0x4
#define ZIGB_PER_RECEIVE			    0x5
#define BUFFER_READ                     0x6
#define BUFFER_WRITE                    0x7
#define ZIGB_PER_STATS 		            0x8
#define ANT_SEL			                0x9
#define SET_ZIGB_CHAN			        0xA
#define TX_STATUS    			        0xB
#define ZIGB_CW_MODE    			    0xC
#define ZIGB_PER        				0xFE 




#define RSI_SET_BB_READ                 0x01
#define RSI_SET_BB_WRITE                0x02
#define RSI_RF_WRITE		            0x03    
#define RSI_RF_READ		                0x04      
#define RSI_ANT_SEL		                0x05
#define RSI_GET_ZIGB_STATS              0x06
#define RSI_SET_ZIGB_CHANNEL            0x07
#define RSI_ZIGB_STATS                  0x08
#define RSI_SET_CW_MODE                 0x09    


struct bb_rf_param_t     
{
        unsigned char  value; //type is present here
        unsigned char  no_of_fields;
        unsigned char  no_of_values;
        unsigned char  soft_reset;
        unsigned short  Data[110];
} ;

//typedef struct  __attribute__((__packed__)) zb_per_params_s
typedef struct zb_per_params_s
{
	unsigned char  enable;
	unsigned char  power;
	unsigned short pkt_length;
	unsigned char mode;
	unsigned char channel;
	unsigned short packet_count;
	unsigned short delay;

}zb_per_params_t;

typedef struct  __attribute__((__packed__)) zb_per_recv_params_s
{
	unsigned char  enable;
	unsigned char  power;
	unsigned short pkt_length;
	unsigned char mode;
	unsigned char channel;
	unsigned short packet_count;
	unsigned short delay;

}zb_per_recv_params_t;
#if 0
typedef struct per_params_s
{
	unsigned short enable;
	unsigned short power;
	unsigned short pkt_length;
	unsigned short mode;
	unsigned short channel;
	unsigned short packet_count;
	unsigned short delay;
} per_params_t;
#endif

#define ONEBOX_PRINT(fmt, args...) fprintf(stdout, fmt, ## args)
#define BIT(n)         (1 << (n))

/*** Src Destn Port And IP Defines ***/
#define DESTN_PORT_NUM                  6666
#define SOURCE_PORT_NUM                 6666
#define DESTN_IP                        "192.168.70.222"
//#define DESTN_IP                        "127.0.0.1"
#define ONEBOX_MAX_PKT_LEN              6348 // Bytes
#define MAX_NUM_OF_PKTS                 100

#define IFNAMSIZ                        16

#define FUNCTION_ENTRY()    NULL//            printf("+%s\n", __func__)
#define FUNCTION_EXIT()     NULL//            printf("-%s\n", __func__)
typedef struct zb_per_stats_s 
{
  //! no. of pkts that pass crc
  unsigned short crc_pass;
  //! no. of pkts failing crc chk
  unsigned short crc_fail;
  //! no. of pkt aborts
  unsigned short pkt_abort;
  //! no. of tx pkts
  unsigned short tx_pkts;
  //! cca idle time
  unsigned short cca_idle;
  //! no. of times RX start got stuck
  unsigned short rx_start_stuck_stk;
  //! no of times RX cca got stuck
  unsigned short rx_cca_stk;
  //! no. of time fifo occupancy counter increments
  unsigned short fifo_occupy;
  //! no. of times phy aborts got generated
  unsigned short phy_abort;
  //! no. of times RX packet timeout inter generated
  unsigned short rx_pkt_tout;
  //! RSSI value
  signed char rssi;
}per_stats_t;

void usage();
int getcmdnumber (char *command);
int_32 zb_stats( uint_8 *file_name );
int_32 zb_transmit(uint_32 *packet, uint_32 no_of_args);
int zb_receive(uint_32 *packet);

/*matlab utils declarations */
int_32 rf_read(uint_32, uint_32, uint_32, uint_32 *);
int_32 rf_write(uint_32, uint_32, uint_32, uint_32 *);
int_32 bb_read(uint_32, uint_32, uint_32);
int_32 bb_write(uint_32, uint_32, uint_32 *);
int_32 buffer_read(uint_32 *, uint_32, uint_32);
int_32 buffer_write(uint_32 *, uint_32, uint_32);
int_32 bt_transmit(uint_32 *,uint_32);
int_32 bt_receive(uint_32 *);


#endif
