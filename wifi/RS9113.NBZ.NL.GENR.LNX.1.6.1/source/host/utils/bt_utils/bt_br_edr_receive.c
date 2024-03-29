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

/*
#ifdef 9118 
#define NO_OF_ARGS  14 //for 9118
#else
#define NO_OF_ARGS  11	//for 9113
#endif
*/

#define NO_OF_ARGS  15 //for 9118
#define STOP_OF_ARGS 2



int main(int argc, char **argv)
{
	int count = 0, stop_bit = 0, rc = 0;
	int i; 
	struct iwreq iwr;
	unsigned int num_1;
	unsigned short int num_2;
	bt_br_edr_per_recv_params_t bt_br_edr_per_recv_params;
	struct bb_rf_param_t bb_rf_params;
	unsigned char recv_pkts[1024];
	unsigned char macaddr[6];
	stop_bit = 0;
	memset(&bt_br_edr_per_recv_params, 0, sizeof(bt_br_edr_per_recv_params_t));
	memset(&bb_rf_params, 0, sizeof(bb_rf_params));
	printf("argc = %d\n",argc);
	if (argc == NO_OF_ARGS )
  {
    if (rsi_netlink_init() < 0)
    {
      ONEBOX_PRINT ("Netling Socket creation error\n");
      return -1;
    }

    bt_br_edr_per_recv_params.enable = 1;
    memset (macaddr, 0, 6);
    byteconversion(argv[1], macaddr);

    for( num_1 = 0; num_1 < 6; num_1++ )
    {
      bt_br_edr_per_recv_params.bt_addr[5-num_1] = macaddr[num_1];
    }
    bt_br_edr_per_recv_params.link_type = atoi(argv[2]);
    bt_br_edr_per_recv_params.pkt_type = atoi(argv[3]);
    bt_br_edr_per_recv_params.pkt_length = atoi(argv[4]);
    bt_br_edr_per_recv_params.scrambler_seed = atoi(argv[5]);
    bt_br_edr_per_recv_params.br_edr_indication = atoi(argv[6]);
    bt_br_edr_per_recv_params.bt_rx_rf_chnl = atoi(argv[7]);	//verify why in the old file these are in reverse order
    bt_br_edr_per_recv_params.bt_tx_rf_chnl = atoi(argv[8]);
    bt_br_edr_per_recv_params.enable_hopping = atoi(argv[9]);
    bt_br_edr_per_recv_params.ant_sel = atoi(argv[10]);
    bt_br_edr_per_recv_params.loop_back_mode = atoi(argv[11]);
    bt_br_edr_per_recv_params.pll_mode = atoi(argv[12]);
    bt_br_edr_per_recv_params.rf_type = atoi(argv[13]);
    bt_br_edr_per_recv_params.rf_chain = atoi(argv[14]);



    printf("--the type of the packet %d\n ",bt_br_edr_per_recv_params.pkt_type);
    printf("--the address %x\n ",*(unsigned short int *)&bt_br_edr_per_recv_params.bt_addr[4]);
    printf("--the address %x ",*(unsigned int *)&bt_br_edr_per_recv_params.bt_addr[0]);
    printf("the packet type is %d \n",bt_br_edr_per_recv_params.pkt_type);
    printf("the packet length is %d \n",bt_br_edr_per_recv_params.pkt_length);


    bb_rf_params.value = GET_DRV_COEX_MODE;
    if( rsi_send_to_drv((uint_8 *)&bb_rf_params, BT_PER,sizeof(bb_rf_params)) < 0 )
    {
      printf("Unable to perform bt_transmit\n");
      return -1;
    }

    rc = recv_data((uint_8 *)&bb_rf_params.Data[0]);

    if((rc == BT_CLASSIC_MODE) || (rc == BT_DUAL_MODE))
    {
      if(bt_br_edr_per_recv_params.bt_rx_rf_chnl < BT_CLASSIC_START_CHANNEL 
          || bt_br_edr_per_recv_params.bt_rx_rf_chnl > BT_CLASSIC_END_CHANNEL)
      {
        printf("Invalid rx_channel in CLASSIC oper mode\n");
        return -1;
      }

      if( bt_br_edr_per_recv_params.bt_tx_rf_chnl < BT_CLASSIC_START_CHANNEL 
          || bt_br_edr_per_recv_params.bt_tx_rf_chnl > BT_CLASSIC_END_CHANNEL)
      {
        printf("Invalid tx_channel in CLASSIC oper mode\n");
        return -1;
      }

      if((bt_br_edr_per_recv_params.enable_hopping == FREQ_FIXED_HOP) 
          && ((bt_br_edr_per_recv_params.bt_tx_rf_chnl == bt_br_edr_per_recv_params.bt_rx_rf_chnl)))
      {
        printf("Tx and Rx channels should be different in the fixed hopping\n");
        return -1;
      }

      if(bt_br_edr_per_recv_params.link_type == SCO_LINK)     
      {
        if(bt_br_edr_per_recv_params.br_edr_indication == ONE_MBPS)
        {
          if(bt_br_edr_per_recv_params.pkt_type == BT_HV1_PKT_TYPE)
          {
            if(!(bt_br_edr_per_recv_params.pkt_length == BT_HV1_VOICE_PAYLOAD_LEN))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_recv_params.pkt_type == BT_HV2_PKT_TYPE)
          {
            if(!(bt_br_edr_per_recv_params.pkt_length == BT_HV2_VOICE_PAYLOAD_LEN))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_recv_params.pkt_type == BT_HV3_PKT_TYPE)
          {
            if(!(bt_br_edr_per_recv_params.pkt_length == BT_HV3_VOICE_PAYLOAD_LEN))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
        } 
      }

      if(bt_br_edr_per_recv_params.link_type == ESCO_LINK)     
      {
        if(bt_br_edr_per_recv_params.br_edr_indication == ONE_MBPS)
        {
          if(bt_br_edr_per_recv_params.pkt_type == BT_EV3_PKT_TYPE)
          {
            if(!((bt_br_edr_per_recv_params.pkt_length > 0) 
                  && (bt_br_edr_per_recv_params.pkt_length <= BT_EV3_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_recv_params.pkt_type == BT_EV4_PKT_TYPE)
          {
            if(!((bt_br_edr_per_recv_params.pkt_length > 0) 
                  && (bt_br_edr_per_recv_params.pkt_length <= BT_EV4_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_recv_params.pkt_type == BT_EV5_PKT_TYPE)
          {
            if(!((bt_br_edr_per_recv_params.pkt_length > 0) 
                  && (bt_br_edr_per_recv_params.pkt_length <= BT_EV5_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
        }

        if(bt_br_edr_per_recv_params.br_edr_indication == TWO_MBPS)
        {
          if(bt_br_edr_per_recv_params.pkt_type == BT_2EV3_PKT_TYPE)
          {
            if(!((bt_br_edr_per_recv_params.pkt_length > 0) 
                  && (bt_br_edr_per_recv_params.pkt_length <= BT_2EV3_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_recv_params.pkt_type == BT_2EV5_PKT_TYPE)
          {
            if(!((bt_br_edr_per_recv_params.pkt_length > 0) 
                  && (bt_br_edr_per_recv_params.pkt_length <= BT_2EV5_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
        }

        if(bt_br_edr_per_recv_params.br_edr_indication == THREE_MBPS)
        {
          if(bt_br_edr_per_recv_params.pkt_type == BT_3EV3_PKT_TYPE)
          {
            if(!((bt_br_edr_per_recv_params.pkt_length > 0) 
                  && (bt_br_edr_per_recv_params.pkt_length <= BT_3EV3_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
          if(bt_br_edr_per_recv_params.pkt_type == BT_3EV5_PKT_TYPE)
          {
            if(!((bt_br_edr_per_recv_params.pkt_length > 0) 
                  && (bt_br_edr_per_recv_params.pkt_length <= BT_3EV5_VOICE_PAYLOAD_LEN)))
            {
              printf("Invalid packet length\n");
              return -1;
            }
          }
        } 
      }

      if(!((bt_br_edr_per_recv_params.br_edr_indication >= ONE_MBPS) 
            && (bt_br_edr_per_recv_params.br_edr_indication <= THREE_MBPS  )))
      {
        printf("Invalid BT_EDR Mode\n");
        return -1;
      }
      if(!((bt_br_edr_per_recv_params.enable_hopping >= FREQ_NO_HOP) 
            && (bt_br_edr_per_recv_params.enable_hopping <= FREQ_RAND_HOP )))
      {
        printf("Invalid Hopping Mode\n");
        return -1;
      }
      if(!((bt_br_edr_per_recv_params.link_type >= SCO_LINK) 
            && (bt_br_edr_per_recv_params.link_type <= ESCO_LINK)))
      {
        printf("Invalid link type Mode\n");
        return -1;
      }
      
      if(!(bt_br_edr_per_recv_params.loop_back_mode >= DISABLE_LOOP_BACK
            || bt_br_edr_per_recv_params.loop_back_mode <= ENABLE_LOOP_BACK))   //condition fails only for 0,1
      {
        printf("Invalid loop_back_mode value\n");
        return -1;
      }

      if( !(bt_br_edr_per_recv_params.pll_mode >= PLL_MODE0 
            || bt_br_edr_per_recv_params.pll_mode <= PLL_MODE2) )   //condition fails only for 0,1,2
      {
        printf("Invalid pll mode\n");
        return -1;
      }

      if( !(bt_br_edr_per_recv_params.rf_type >= EXTERNAL_RF 
            || bt_br_edr_per_recv_params.rf_type <= INTERNAL_RF) )   //condition fails only for 0,1
      {
        printf("Invalid rf_type\n");
        return -1;
      }

      if((bt_br_edr_per_recv_params.rf_chain > BT_HP_CHAIN) 
            || (bt_br_edr_per_recv_params.rf_chain == WLAN_LP_CHAIN))   //condition fails only for 0,1,2,3
      {
        printf("Invalid rf chain \n");
        return -1;
      }
    }
    else
    {
      printf("Invalid Driver mode\n");
      return -1;
    }
    printf("the packet type is %d \n",bt_br_edr_per_recv_params.pkt_type);
    printf("the packet length is %d \n",bt_br_edr_per_recv_params.pkt_length);

    bb_rf_params.value = PER_BR_EDR_RECEIVE;
    bb_rf_params.no_of_values = sizeof(bt_br_edr_per_recv_params_t);
    memcpy(&bb_rf_params.Data[0], &bt_br_edr_per_recv_params, sizeof(bt_br_edr_per_recv_params_t));
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

		bt_br_edr_per_recv_params.enable = 0;
		printf("============= BT_RECEIVE_STOP ==================");
		bb_rf_params.value = PER_BR_EDR_RECEIVE;
		bb_rf_params.no_of_values = sizeof(bt_br_edr_per_recv_params_t);
		memcpy(&bb_rf_params.Data[1], &bt_br_edr_per_recv_params, sizeof(bt_br_edr_per_recv_params_t));
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
	else
	{
		printf("USAGE: %s  <dev_addr> <link_type> <pkt_type> <pkt_length> <scrambler_seed> <br_edr_mode> <rx_channel_index> <tx_channel_index> <hopping_type> <ant_sel> <loop_back_mode> <pll_mode> <rf_type> <rf_chain>\n", argv[0]);
		return 1;

	}
	return 0;
}
#endif
