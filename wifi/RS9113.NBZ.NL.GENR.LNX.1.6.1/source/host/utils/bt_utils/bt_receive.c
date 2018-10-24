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

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include "bt_util.h"



#define NO_OF_ARGS  13
#define STOP_OF_ARGS 2
int main(int argc, char **argv)
{
  int count = 0, stop_bit = 0, rc = 0;
  int i; 
  struct iwreq iwr;
  unsigned int num_1;
  unsigned short int num_2;
  bt_per_recv_params_t bt_per_recv_params;
  struct bb_rf_param_t bb_rf_params;
  unsigned char recv_pkts[1024];
	unsigned char macaddr[6];
  stop_bit = 0;
	memset(&bt_per_recv_params, 0, sizeof(bt_per_recv_params_t));
	memset(&bb_rf_params, 0, sizeof(bb_rf_params));
	printf("argc = %d\n",argc);
  if (argc == NO_OF_ARGS )
  {
    if (rsi_netlink_init() < 0)
    {
      ONEBOX_PRINT ("Netling Socket creation error\n");
      return -1;
    }
#if 0
    printf("Usage: %s  <BD_ADDR[6:4]> <BD_ADDR[4:0]> <LINK_TYPE> <PKT_TYPE> <PKT_LENGTH> <SCRAMBLER_SEED> <EDR_INDICATION> <RX_CHNL> <TX_CHNL> <MODE> <LE_CHANNEL> \n", argv[0]);
    printf("\tStart-Stop value\t- 0: Start & 1: Stop \n");
    printf("\tLINK_TYPE : SCO - 0 \t ACL - 1 \t ESCO - 2 \n");
    printf("\tEDR_INDICATION : BASIC RATE - 1 \t ENHANCED_RATE - 2 or 3 \n");
    printf("\tMODE : BR/EDR - 1 \t LE - 2 \n");
    printf("\tLE_CHANNEL : ADVERTISING - 0 \t DATA CHANNEL - 1 \n");
    printf("\tHOPPING TYPE : FIXED HOPPING - 1 \t RANDOM HOPPING - 2 \n");

    return 1;
#endif
    bt_per_recv_params.enable = 1;
    memset (macaddr, 0, 6);
    byteconversion(argv[1], macaddr);

    for( num_1 = 0; num_1 < 6; num_1++ )
    {
      bt_per_recv_params.bt_addr[5-num_1] = macaddr[num_1];
    }
    bt_per_recv_params.link_type = atoi(argv[2]);
    bt_per_recv_params.pkt_type = atoi(argv[3]);
    bt_per_recv_params.pkt_length = atoi(argv[4]);
    bt_per_recv_params.scrambler_seed = atoi(argv[5]);
    bt_per_recv_params.edr_indication = atoi(argv[6]);
    bt_per_recv_params.bt_rx_rf_chnl = atoi(argv[8]);
    bt_per_recv_params.bt_tx_rf_chnl = atoi(argv[7]);
    bt_per_recv_params.le_mode = atoi(argv[9]);
    bt_per_recv_params.le_chnl = atoi(argv[10]);
    bt_per_recv_params.enable_hopping = atoi(argv[11]);
    bt_per_recv_params.ant_sel = atoi(argv[12]);
    printf("--the type of the packet %d\n ",bt_per_recv_params.pkt_type);
    printf("--the address %x\n ",*(unsigned short int *)&bt_per_recv_params.bt_addr[4]);
    printf("--the address %x ",*(unsigned int *)&bt_per_recv_params.bt_addr[0]);
    printf("the packet type is %d \n",bt_per_recv_params.pkt_type);
    printf("the packet length is %d \n",bt_per_recv_params.pkt_length);


		bb_rf_params.value = GET_DRV_COEX_MODE;
		if( rsi_send_to_drv((uint_8 *)&bb_rf_params, BT_PER,sizeof(bb_rf_params)) < 0 )
		{
			printf("Unable to perform bt_transmit\n");
			return -1;
		}
		
    rc = recv_data((uint_8 *)&bb_rf_params.Data[0]);
    
    if( rc == 1 )
    {
      if( bt_per_recv_params.le_mode != BT_CLASSIC )
      {
        printf("Invalid le_classic_mode parameter in CLASSIC oper mode \n");
        return  -1;
      }
      if( bt_per_recv_params.bt_rx_rf_chnl < 0 || bt_per_recv_params.bt_rx_rf_chnl > 79 )
      {
        printf("Invalid rx_channel in CLASSIC oper mode\n");
        return -1;
      }
      if( bt_per_recv_params.bt_tx_rf_chnl < 0 || bt_per_recv_params.bt_tx_rf_chnl > 79 )
      {
        printf("Invalid tx_channel in CLASSIC oper mode\n");
        return -1;
      }
    }
    else if( rc == 2 )
    {
      if( bt_per_recv_params.le_mode != BT_LE )
      {
        printf("Invalid le_classic_mode parameter in LE oper mode\n");
        return  -1;
      }
      if( bt_per_recv_params.bt_rx_rf_chnl < 0 || bt_per_recv_params.bt_rx_rf_chnl > 39 )
      {
        printf("Invalid rx_channel in LE oper mode\n");
        return -1;
      }
      if( bt_per_recv_params.bt_tx_rf_chnl < 0 || bt_per_recv_params.bt_tx_rf_chnl > 39 )
      {
        printf("Invalid tx_channel in LE oper mode\n");
        return -1;
      }
    }
    else
    {
      printf("Invalid Driver mode\n");
      return -1;
    }

#ifdef CHIP_9116
    if(!((bt_per_recv_params.edr_indication >= 1) && (bt_per_recv_params.edr_indication <= 8 )))  
#else
    if(!((bt_per_recv_params.edr_indication >= 1) && (bt_per_recv_params.edr_indication < 4 )))
#endif
    {
      printf("Invalid BT_EDR Mode\n");
      return -1;
    }
    if(!((bt_per_recv_params.enable_hopping >= 0) && (bt_per_recv_params.enable_hopping < 3 )))
    {
      printf("Invalid Hopping Mode\n");
      return -1;
    }



    bb_rf_params.value = BT_RECEIVE;
    bb_rf_params.no_of_values = sizeof(bt_per_recv_params_t);
    memcpy(&bb_rf_params.Data[0], &bt_per_recv_params, sizeof(bt_per_recv_params_t));
    usleep(500000);
    if( rsi_send_to_drv((uint_8 *)&bb_rf_params, BT_PER, sizeof(bb_rf_params)) < 0 )
    {
      printf("Unable to perform bt_receive\n");
      return -1;
    }	
    printf("======== SUCCESS ============\n");
    if (rsi_netlink_deinit() < 0)
    {
      ONEBOX_PRINT ("Netling Socket creation error\n");
      return -1;
    }
  }	
  else if(argc == STOP_OF_ARGS)
  {
    /*Indicates that it is transmission*/
    if (rsi_netlink_init() < 0)
    {
      ONEBOX_PRINT ("Netling Socket creation error\n");
      return -1;
    }

    bt_per_recv_params.enable = 0;
    printf("============= BT_RECEIVE_STOP ==================");
    bb_rf_params.value = BT_RECEIVE;
    bb_rf_params.no_of_values = sizeof(bt_per_recv_params_t);
    memcpy(&bb_rf_params.Data[1], &bt_per_recv_params, sizeof(bt_per_recv_params_t));
    usleep(500000); 
    if( rsi_send_to_drv((uint_8 *)&bb_rf_params, BT_PER,sizeof(bb_rf_params)) < 0 )
    {
      printf("Unable to perform bt_receivet\n");
      return -1;
    }
    printf("======== SUCCESS ============\n");

    if (rsi_netlink_deinit() < 0)
    {
      ONEBOX_PRINT ("Netling Socket creation error\n");
      return -1;
    }    
  }
  
#if 1
	else
  {
#if 1
    printf("USAGE: %s  <dev_addr> <link_type> <pkt_type> <pkt_length> <scrambler_seed> <bt_edr_mode> <rx_channel_index> <tx_channel_index> <classic_le_mode> <le_channel_type>  <hopping_type> <ant_sel>\n", argv[0]);
    printf("\tstart-stop value\t- 0: start & 1: stop \n");
    printf("\tlink_type : sco - 0 \t acl - 1 \t esco - 2 \n");
    printf("\tedr_indication : basic rate - 1 \t enhanced_rate - 2 or 3 \n");
    printf("\tmode : br/edr - 1 \t le - 2 \n");
    printf("\tle_channel : advertising - 0 \t data channel - 1 \n");
    printf("\thopping type : fixed hopping - 1 \t random hopping - 2 \n");
    printf("\tant_sel : onchip antenna - 2 \t u.f.l - 3 \n");
    return 1;
#endif
  }
#endif
  return 0;
}
