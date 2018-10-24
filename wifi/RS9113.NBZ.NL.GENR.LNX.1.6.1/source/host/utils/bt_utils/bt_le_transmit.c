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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include "bt_util.h"

#define NO_OF_ARGS  18
#define STOP_OF_ARGS 2



int main(int argc, char *argv[])
{      
  int i, rc = 0;
  char *tmp_rate;
  ble_per_params_t ble_per_params;
  struct bb_rf_param_t bb_rf_params;

  unsigned int num_1;
  unsigned short int num_2;
  char acc_addr[4];


  /*Creating a Socket*/
  memset(&ble_per_params, 0, sizeof(ble_per_params_t));
  memset(&bb_rf_params, 0, sizeof(bb_rf_params));

  if(argc == NO_OF_ARGS)
  {
    /* Init netlink socket */
    if (rsi_netlink_init() < 0)
    {
      ONEBOX_PRINT ("Netling Socket creation error\n");
      return -1;
    }

    memset (acc_addr, 0, 4);
    mapconversion(argv[1], acc_addr,8);
    ble_per_params.enable = 1;

    for( num_1 = 0; num_1 < 4; num_1++ )
    {
      ble_per_params.access_addr[3-num_1] = acc_addr[num_1];
    }



    ble_per_params.pkt_length = atoi(argv[2]);
    ble_per_params.ble_rate = atoi(argv[3]);
    ble_per_params.bt_rx_rf_chnl = atoi(argv[4]);
    ble_per_params.bt_tx_rf_chnl = atoi(argv[5]);

    ble_per_params.scrambler_seed = atoi(argv[6]);
    ble_per_params.num_pkts = atoi(argv[7]);
    ble_per_params.payload_data_type = atoi(argv[8]);

    ble_per_params.le_chnl = atoi(argv[9]);
    ble_per_params.tx_pwr_indx = atoi(argv[10]);
    ble_per_params.transmit_mode = atoi(argv[11]);
    ble_per_params.hopping_type = atoi(argv[12]);
    ble_per_params.ant_select = atoi(argv[13]);	
    ble_per_params.inter_pkt_gap = atoi(argv[14]);
    ble_per_params.pll_mode = atoi(argv[15]);
    ble_per_params.rf_type = atoi(argv[16]);
    ble_per_params.rf_chain = atoi(argv[17]);


    bb_rf_params.value = TX_STATUS;
    if( rsi_send_to_drv((uint_8 *)&bb_rf_params, BT_PER,sizeof(bb_rf_params)) < 0 )
    {
      printf("Unable to perform bt_transmit\n");
      return -1;
    }
    if(recv_data((uint_8 *)&bb_rf_params.Data[0]) < 0)
    {
      printf("======== ALREADY IN TRANSMIT SO STOP TRANSMIT FIRST ============\n");
      return -1;
    }

    bb_rf_params.value = GET_DRV_COEX_MODE;
    if( rsi_send_to_drv((uint_8 *)&bb_rf_params, BT_PER,sizeof(bb_rf_params)) < 0 )
    {
      printf("Unable to perform bt_transmit\n");
      return -1;
    }

    rc = recv_data((uint_8 *)&bb_rf_params.Data[0]);	



    if(( rc == BT_LE_MODE ) || (rc == BT_DUAL_MODE))
    {
      if(ble_per_params.bt_rx_rf_chnl > BT_LE_END_CHANNEL)
      {
        printf("Invalid rx_channel in LE opermode\n");
        return -1;
      }
      if(ble_per_params.bt_tx_rf_chnl > BT_LE_END_CHANNEL)
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

    if(ble_per_params.transmit_mode > 2)
    {
      printf("Invalid transmit Mode\n");
      return -1;
    }

    if(!((ble_per_params.ble_rate == BLE_1MBPS)
        || (ble_per_params.ble_rate == BLE_2MBPS)
        || (ble_per_params.ble_rate == BLR_500KBPS)
        || (ble_per_params.ble_rate == BLR_125KBPS))
      )
    {
      printf("Invalid ble_rate \n");
      return -1;
    }


    if(ble_per_params.le_chnl > 2)
    {
      printf("Invalid le_chnl type \n");
      return -1;
    }


    if(!((ble_per_params.hopping_type >= FREQ_NO_HOP) 
          && (ble_per_params.hopping_type <= FREQ_RAND_HOP)))
    {
      printf("Invalid Hopping Type\n");
      return -1;
    }


    if(!((ble_per_params.pll_mode >= PLL_MODE0) 
          && (ble_per_params.pll_mode <= PLL_MODE2)))
    {
      printf("Invalid pll mode\n");
      return -1;
    }

    if(ble_per_params.rf_type > INTERNAL_RF)   
    {
      printf("Invalid rf_type\n");
      return -1;
    }

    if((ble_per_params.rf_chain > BT_HP_CHAIN) 
          || (ble_per_params.rf_chain == WLAN_LP_CHAIN))
    {
      printf("Invalid rf chain \n");
      return -1;
    }

    printf("the packet length is %d \n",ble_per_params.pkt_length);

    bb_rf_params.value = PER_BLE_TRANSMIT;
    bb_rf_params.no_of_values = sizeof(ble_per_params_t);
    memcpy(&bb_rf_params.Data[0], &ble_per_params, sizeof(ble_per_params_t));

    usleep(500000);

    if( rsi_send_to_drv((uint_8 *)&bb_rf_params, BT_PER,sizeof(bb_rf_params)) < 0 )
    {
      printf("Unable to perform bt_transmit\n");
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
    /* Init netlink socket */
    if (rsi_netlink_init() < 0)
    {
      ONEBOX_PRINT ("Netling Socket creation error\n");
      return -1;
    }

    ble_per_params.enable = 0;
    printf("============= BT_TRANSMIT_STOP ==================");

    bb_rf_params.value = PER_BLE_TRANSMIT;
    bb_rf_params.no_of_values = sizeof(ble_per_params_t);
    memcpy(&bb_rf_params.Data[0], &ble_per_params, sizeof(ble_per_params_t));

    usleep(500000);

    if( rsi_send_to_drv((uint_8 *)&bb_rf_params, BT_PER, sizeof(bb_rf_params)) < 0 )
    {
      printf("Unable to perform bt_transmit\n");
      return -1;
    }
    printf("======== SUCCESS ============\n");

    if (rsi_netlink_deinit() < 0)
    {
      ONEBOX_PRINT ("Netling Socket creation error\n");
      return -1;
    }

  }
  else
  {
    printf("\nUSAGE to start transmit: %s <Access_Addr> <pkt_length> <ble_rate> <rx_channel_index> <tx_channel_index> <scrambler_seed> <no_of_packets> <payload_type> <le_channel_type> <tx_power> <tx_mode> <hopping_type> <ant_sel> <inter_pkt_gap> <pll_mode> <rf_type> <rf_chain>\n",argv[0]);
    return 1;

  }
  return 0;
}
#endif
