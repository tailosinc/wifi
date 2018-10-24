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

#define NO_OF_ARGS  19
#define STOP_OF_ARGS 2

int main(int argc, char *argv[])
{      
  int i, rc = 0;
  char *tmp_rate;
  bt_br_edr_per_params_t bt_br_edr_per_params;
  struct bb_rf_param_t bb_rf_params;

  unsigned int num_1;
  unsigned short int num_2;
  unsigned char macaddr[6];


  /*Creating a Socket*/
  memset(&bt_br_edr_per_params, 0, sizeof(bt_br_edr_per_params_t));
  memset(&bb_rf_params, 0, sizeof(bb_rf_params));

  if(argc == NO_OF_ARGS)
  {
    /* Init netlink socket */
    if (rsi_netlink_init() < 0)
    {
      ONEBOX_PRINT ("Netling Socket creation error\n");
      return -1;
    }

    memset (macaddr, 0, 6);
    byteconversion(argv[1], macaddr);

    bt_br_edr_per_params.enable = 1;

    for( num_1 = 0; num_1 < 6; num_1++ )
    {
      bt_br_edr_per_params.bt_addr[5-num_1] = macaddr[num_1];
    }
    bt_br_edr_per_params.pkt_type = atoi(argv[2]);
    bt_br_edr_per_params.pkt_length = atoi(argv[3]);
    bt_br_edr_per_params.br_edr_indication = atoi(argv[4]);
    bt_br_edr_per_params.bt_rx_rf_chnl = atoi(argv[5]);
    bt_br_edr_per_params.bt_tx_rf_chnl = atoi(argv[6]);
    bt_br_edr_per_params.link_type = atoi(argv[7]);
    bt_br_edr_per_params.scrambler_seed = atoi(argv[8]);
    bt_br_edr_per_params.num_pkts = atoi(argv[9]);
    bt_br_edr_per_params.payload_data_type = atoi(argv[10]);
    bt_br_edr_per_params.tx_pwr_indx = atoi(argv[11]);
    bt_br_edr_per_params.transmit_mode = atoi(argv[12]);
    bt_br_edr_per_params.enable_hopping = atoi(argv[13]);
    bt_br_edr_per_params.ant_select = atoi(argv[14]); 
    bt_br_edr_per_params.inter_pkt_gap = atoi(argv[15]);
    bt_br_edr_per_params.pll_mode = atoi(argv[16]);
    bt_br_edr_per_params.rf_type = atoi(argv[17]);
    bt_br_edr_per_params.rf_chain = atoi(argv[18]);


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

    printf("rc value: %d\n",rc); 
    rc = recv_data((uint_8 *)&bb_rf_params.Data[0]);
    printf("rc value: %d\n",rc); 
    if(( rc == BT_CLASSIC_MODE ) || (rc == BT_DUAL_MODE))
    { 
      if( bt_br_edr_per_params.bt_rx_rf_chnl < BT_CLASSIC_START_CHANNEL 
          || bt_br_edr_per_params.bt_rx_rf_chnl > BT_CLASSIC_END_CHANNEL )
      {
        printf("Invalid rx_channel in CLASSIC oper mode\n");
        return -1;
      }

      if( bt_br_edr_per_params.bt_tx_rf_chnl < BT_CLASSIC_START_CHANNEL 
          || bt_br_edr_per_params.bt_tx_rf_chnl > BT_CLASSIC_END_CHANNEL)
      {
        printf("Invalid tx_channel in CLASSIC oper mode\n");
        return -1;
      }

      if((bt_br_edr_per_params.enable_hopping == FREQ_FIXED_HOP)
          && ((bt_br_edr_per_params.bt_tx_rf_chnl == bt_br_edr_per_params.bt_rx_rf_chnl)))
      {
        printf("Tx and Rx channels should be different in the fixed hopping\n");
        return -1;
      }

      if(bt_br_edr_per_params.link_type == ACL_LINK)     
      {
        if(bt_br_edr_per_params.br_edr_indication == ONE_MBPS)
        {
          if(bt_br_edr_per_params.pkt_type == BT_DM1_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >=0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_DM1_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_DH1_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >=0) 
                  &&(bt_br_edr_per_params.pkt_length <= BT_DH1_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_DM3_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >=0) 
                  &&(bt_br_edr_per_params.pkt_length <= BT_DM3_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_DH3_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >= BT_DH3_PAYLOAD_MAX_LEN) 
                  && (bt_br_edr_per_params.pkt_length <= BT_DH3_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_DM5_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >=0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_DM5_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_DH5_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >=0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_DH5_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
        }

        if(bt_br_edr_per_params.br_edr_indication == TWO_MBPS)
        {
          if(bt_br_edr_per_params.pkt_type == BT_2DH1_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >=0) 
                  &&(bt_br_edr_per_params.pkt_length <= BT_2DH1_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_2DH3_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >=0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_2DH3_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_2DH5_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >=0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_2DH5_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
        }

        if(bt_br_edr_per_params.br_edr_indication == THREE_MBPS)
        {
          if(bt_br_edr_per_params.pkt_type == BT_3DH1_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >=0) 
                  && (bt_br_edr_per_params.pkt_length < BT_3DH1_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_3DH3_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >=0) 
                  && (bt_br_edr_per_params.pkt_length < BT_3DH3_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_3DH5_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length >=0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_3DH5_PAYLOAD_MAX_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
        } 
      }

      if(bt_br_edr_per_params.link_type == SCO_LINK)     
      {
        if(bt_br_edr_per_params.br_edr_indication == ONE_MBPS)
        {
          if(bt_br_edr_per_params.pkt_type == BT_HV1_PKT_TYPE)
          {
            if(!(bt_br_edr_per_params.pkt_length == BT_HV1_VOICE_PAYLOAD_LEN))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_HV2_PKT_TYPE)
          {
            if(!(bt_br_edr_per_params.pkt_length == BT_HV2_VOICE_PAYLOAD_LEN))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_HV3_PKT_TYPE)
          {
            if(!(bt_br_edr_per_params.pkt_length == BT_HV3_VOICE_PAYLOAD_LEN))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
        } 
      }

      if(bt_br_edr_per_params.link_type == ESCO_LINK)     
      {
        if(bt_br_edr_per_params.br_edr_indication == ONE_MBPS)
        {
          if(bt_br_edr_per_params.pkt_type == BT_EV3_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length > 0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_EV3_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_EV4_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length > 0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_EV4_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_EV5_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length > 0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_EV5_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
        }

        if(bt_br_edr_per_params.br_edr_indication == TWO_MBPS)
        {
          if(bt_br_edr_per_params.pkt_type == BT_2EV3_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length > 0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_2EV3_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_2EV5_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length > 0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_2EV5_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
        }

        if(bt_br_edr_per_params.br_edr_indication == THREE_MBPS)
        {
          if(bt_br_edr_per_params.pkt_type == BT_3EV3_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length > 0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_3EV3_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_params.pkt_type == BT_3EV5_PKT_TYPE)
          {
            if(!((bt_br_edr_per_params.pkt_length > 0) 
                  && (bt_br_edr_per_params.pkt_length <= BT_3EV5_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
        } 
      }


      if(!((bt_br_edr_per_params.enable_hopping >= FREQ_NO_HOP) 
            && (bt_br_edr_per_params.enable_hopping <= FREQ_RAND_HOP )))
      {
        printf("Invalid Hopping Mode\n");
        return -1;
      }

      if(!((bt_br_edr_per_params.transmit_mode >= 0) && (bt_br_edr_per_params.transmit_mode < 2 )))
      {
        printf("Invalid transmit Mode\n");
        return -1;
      }

      if(!((bt_br_edr_per_params.br_edr_indication >= ONE_MBPS) && (bt_br_edr_per_params.br_edr_indication <= THREE_MBPS)))
      {
        printf("Invalid BT_EDR Mode\n");
        return -1;
      }

      if(!((bt_br_edr_per_params.link_type >= SCO_LINK) 
            && (bt_br_edr_per_params.link_type <= ESCO_LINK)))
      {
        printf("Invalid link type\n");
        return -1;
      }
      
      if(!((bt_br_edr_per_params.pll_mode >= PLL_MODE0) 
            && (bt_br_edr_per_params.pll_mode <= PLL_MODE1 )))
      {
        printf("Invalid pll mode\n");
        return -1;
      }

      if(!((bt_br_edr_per_params.rf_type >= EXTERNAL_RF) 
            && (bt_br_edr_per_params.rf_type <= INTERNAL_RF)))
      {
        printf("Invalid rf type\n");
        return -1;
      }

      if((bt_br_edr_per_params.rf_chain > BT_HP_CHAIN) 
            || (bt_br_edr_per_params.rf_chain == WLAN_LP_CHAIN))
      {
        printf("Invalid rf chain\n");
        return -1;
      }
    }
    else
    {
      printf("Invalid Driver mode\n");
      return -1;
    }




    printf("the packet type is %d \n",bt_br_edr_per_params.pkt_type);
    printf("the packet length is %d \n",bt_br_edr_per_params.pkt_length);

    bb_rf_params.value = PER_BR_EDR_TRANSMIT;
    bb_rf_params.no_of_values = sizeof(bt_br_edr_per_params_t);
    memcpy(&bb_rf_params.Data[0], &bt_br_edr_per_params, sizeof(bt_br_edr_per_params_t));

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

    bt_br_edr_per_params.enable = 0;
    printf("============= BT_TRANSMIT_STOP ==================");

    bb_rf_params.value = PER_BR_EDR_TRANSMIT;
    bb_rf_params.no_of_values = sizeof(bt_br_edr_per_params_t);
    memcpy(&bb_rf_params.Data[0], &bt_br_edr_per_params, sizeof(bt_br_edr_per_params_t));

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
    printf("\nUSAGE to start transmit: %s <dev_addr> <pkt_type>  <pkt_length> <br_edr_mode> <rx_channel_index> <tx_channel_index> <link_type> <scrambler_seed> <no_of_packets> <payload_type> <tx_power> <tx_mode> <hopping_type> <ant_sel> <inter_pkt_gap> <pll_mode> <rf_type> <rf_chain>\n",argv[0]); 
    return 1;
  }
  return 0;
}
#endif
