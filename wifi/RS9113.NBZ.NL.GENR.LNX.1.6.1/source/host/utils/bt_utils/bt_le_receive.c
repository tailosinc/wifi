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
#ifdef CHIP_9116
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include "bt_util.h"



#define NO_OF_ARGS  15
#define STOP_OF_ARGS 2

int main(int argc, char **argv)
{
  int count = 0, stop_bit = 0, rc = 0;
  int i; 
  struct iwreq iwr;
  unsigned int num_1;
  ble_per_recv_params_t ble_per_recv_params;
  struct bb_rf_param_t bb_rf_params;
  unsigned char recv_pkts[1024];
  unsigned char acc_addr[4];
  stop_bit = 0;
	memset(&ble_per_recv_params, 0, sizeof(ble_per_recv_params_t));
	memset(&bb_rf_params, 0, sizeof(bb_rf_params));
	printf("argc = %d\n",argc);
  if (argc == NO_OF_ARGS)
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
    ble_per_recv_params.enable = 1;
    memset (acc_addr, 0, 4);
    mapconversion(argv[1], acc_addr,8);

    for( num_1 = 0; num_1 < 4; num_1++ )
    {
      ble_per_recv_params.access_addr[3-num_1] = acc_addr[num_1];
    }
    ble_per_recv_params.data_length_indication = atoi(argv[2]);
    ble_per_recv_params.scrambler_seed = atoi(argv[3]);
    ble_per_recv_params.ble_rate = atoi(argv[4]);
    ble_per_recv_params.bt_rx_rf_chnl = atoi(argv[5]);
    ble_per_recv_params.bt_tx_rf_chnl = atoi(argv[6]);
    ble_per_recv_params.le_chnl_type = atoi(argv[7]);
    ble_per_recv_params.freq_hop_en = atoi(argv[8]);
    ble_per_recv_params.ant_sel = atoi(argv[9]);
    ble_per_recv_params.loop_back_mode = atoi(argv[10]);
    ble_per_recv_params.pwrsave_options = atoi(argv[11]);
    ble_per_recv_params.pll_mode = atoi(argv[12]);
    ble_per_recv_params.rf_type = atoi(argv[13]);
    ble_per_recv_params.rf_chain = atoi(argv[14]);

		bb_rf_params.value = GET_DRV_COEX_MODE;
		if( rsi_send_to_drv((uint_8 *)&bb_rf_params, BT_PER,sizeof(bb_rf_params)) < 0 )
		{
			printf("Unable to perform bt_receive\n");
			return -1;
		}
		
    rc = recv_data((uint_8 *)&bb_rf_params.Data[0]);
    
    if(( rc == BT_LE_MODE ) || (rc == BT_DUAL_MODE))
    { 
      if(ble_per_recv_params.bt_rx_rf_chnl > BT_LE_END_CHANNEL)
      {
        printf("Invalid rx_channel in LE oper mode\n");
        return -1;
      }
      if(ble_per_recv_params.bt_tx_rf_chnl > BT_LE_END_CHANNEL)
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

    if(!((ble_per_recv_params.ble_rate == BLE_1MBPS) 
          || (ble_per_recv_params.ble_rate == BLE_2MBPS) 
          || (ble_per_recv_params.ble_rate == BLR_500KBPS) 
          || (ble_per_recv_params.ble_rate == BLR_125KBPS)))
    {
        printf("Invalid BLE Rate\n");
        return -1;

    }   
     
    if(ble_per_recv_params.data_length_indication > 2 )
    {
      printf("Invalid data_length_indication\n");
      return -1;
    }
    
    if(ble_per_recv_params.le_chnl_type > 2)  
    {
      printf("Invalid Channel Type\n");
      return -1;
    }
    
#if 0
    if(!((ble_per_recv_params.skip_bb_rf_programming >= DISABLE_SKIP_PROG) 
          && (ble_per_recv_params.skip_bb_rf_programming <= ENABLE_SKIP_PROG)))
    {
      printf("Invalid bb_rf pogramming\n");
      return -1;
    }
#endif
    if(!((ble_per_recv_params.loop_back_mode >= DISABLE_LOOP_BACK)
          && (ble_per_recv_params.loop_back_mode <= ENABLE_LOOP_BACK)))
    {
      printf("Invalid loop back mode\n");
      return -1;
    }

#if 0
    if(!((ble_per_recv_params.pwrsave_options >= DISABLE_DUTY_CYCLE) 
          && (ble_per_recv_params.pwrsave_options <= ENABLE_DUTY_CYCLE)))
    {
      printf("pwrsave_options Selection\n");
      return -1;
    }
#endif

    if(!((ble_per_recv_params.pll_mode >= PLL_MODE0) 
          && (ble_per_recv_params.pll_mode <= PLL_MODE2)))
    {
      printf("Invalid pll mode\n");
      return -1;
    }

    if(!((ble_per_recv_params.rf_type >= EXTERNAL_RF) 
          && (ble_per_recv_params.rf_type <= INTERNAL_RF)))
    {
      printf("Invalid rf type\n");
      return -1;
    }

    if((ble_per_recv_params.rf_chain > BT_HP_CHAIN) 
          || (ble_per_recv_params.rf_chain == WLAN_LP_CHAIN))
    {
      printf("Invalid rf chain \n");
      return -1;
    }

    bb_rf_params.value = PER_BLE_RECEIVE;
    bb_rf_params.no_of_values = sizeof(ble_per_recv_params_t);
    memcpy(&bb_rf_params.Data[0], &ble_per_recv_params, sizeof(ble_per_recv_params_t));
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

    ble_per_recv_params.enable = 0;
    printf("============= BT_RECEIVE_STOP ==================");
    bb_rf_params.value = PER_BLE_RECEIVE;
    bb_rf_params.no_of_values = sizeof(ble_per_recv_params_t);
    memcpy(&bb_rf_params.Data[1], &ble_per_recv_params, sizeof(ble_per_recv_params_t));
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
    printf("USAGE: %s  <access_addr> <data_legth_indication> <scrambler_seed> <ble_rate> <rx_channel_index> <tx_channel_index> <le_channel_type> <freq_hop_en> <ant_sel> <loop_back_mode> <pwrsave_options> <pll_mode> <rf_type> <rf_chain>\n", argv[0]);
    return 1;
#endif
  }
#endif
  return 0;
}
#endif
