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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include "bt_util.h"

#define NO_OF_ARGS   14
#define STOP_OF_ARGS 2
#define WAIT_TIME    500000




int main(int argc, char *argv[])
{      
  int rc = 0;

  ble_aoa_aod_transmit_per_params_t ble_aoa_aod_transmit_per_params;
  struct bb_rf_param_t bb_rf_params;

  /*Creating a Socket*/
  memset(&ble_aoa_aod_transmit_per_params, 0, sizeof(ble_aoa_aod_transmit_per_params_t));
  memset(&bb_rf_params, 0, sizeof(bb_rf_params));

  if(argc == NO_OF_ARGS)
  {
    /* Init netlink socket */
    if (rsi_netlink_init() < 0)
    {
      ONEBOX_PRINT ("Netlink Socket creation error\n");
      return -1;
    }

    ble_aoa_aod_transmit_per_params.enable = 1;
    ble_aoa_aod_transmit_per_params.pkt_length = atoi(argv[1]);
    ble_aoa_aod_transmit_per_params.phy_rate   = atoi(argv[2]);
    ble_aoa_aod_transmit_per_params.bt_tx_rf_chnl = atoi(argv[3]);
    ble_aoa_aod_transmit_per_params.scrambler_seed = atoi(argv[4]);
    ble_aoa_aod_transmit_per_params.payload_data_type = atoi(argv[5]);
    ble_aoa_aod_transmit_per_params.supp_length = atoi(argv[6]);
    ble_aoa_aod_transmit_per_params.supp_slot_type = atoi(argv[7]);
    ble_aoa_aod_transmit_per_params.num_antenna = atoi(argv[8]);
    ble_aoa_aod_transmit_per_params.ant_switch_pattern = atoi(argv[9]);
    ble_aoa_aod_transmit_per_params.le_chnl = atoi(argv[10]);
    ble_aoa_aod_transmit_per_params.tx_pwr_indx = atoi(argv[11]);
    ble_aoa_aod_transmit_per_params.enable_hopping = atoi(argv[12]);
    ble_aoa_aod_transmit_per_params.ant_select = atoi(argv[13]);

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

    if( rc == 2 )
    {
      if( ble_aoa_aod_transmit_per_params.bt_tx_rf_chnl < 0 
          || ble_aoa_aod_transmit_per_params.bt_tx_rf_chnl > 39 )
      {
        printf("Invalid channel in LE opermode\n");
        return -1;
      }
      if(ble_aoa_aod_transmit_per_params.phy_rate > 2)
      {
        printf("Invalid LE Phy rate \n");
        return -1;
      }
    }
    else
    {
      printf("Invalid Driver mode\n");
      return -1;
    }

    if(!((ble_aoa_aod_transmit_per_params.enable_hopping >= 0) && (ble_aoa_aod_transmit_per_params.enable_hopping < 3 )))
    {
      printf("Invalid Hopping Mode\n");
      return -1;
    }

    printf("the packet length is %d \n",ble_aoa_aod_transmit_per_params.pkt_length);

    bb_rf_params.value = BLE_AOA_AOD_TRANSMIT;
    bb_rf_params.no_of_values = sizeof(ble_aoa_aod_transmit_per_params_t);
    memcpy(&bb_rf_params.Data[0], &ble_aoa_aod_transmit_per_params, sizeof(ble_aoa_aod_transmit_per_params_t));

    usleep(WAIT_TIME);

    if( rsi_send_to_drv((uint_8 *)&bb_rf_params, BT_PER,sizeof(bb_rf_params)) < 0 )
    {
      printf("Unable to perform bt_transmit\n");
      return -1;
    }
    printf("======== SUCCESS ============\n");

    if (rsi_netlink_deinit() < 0)
    {
      ONEBOX_PRINT ("Netlink Socket creation error\n");
      return -1;
    }
  }
  else if(argc == STOP_OF_ARGS)
  {
    /* Init netlink socket */
    if (rsi_netlink_init() < 0)
    {
      ONEBOX_PRINT ("Netlink Socket creation error\n");
      return -1;
    }

    ble_aoa_aod_transmit_per_params.enable = 0;
    printf("============= BT_TRANSMIT_STOP ==================");

    bb_rf_params.value = BLE_AOA_AOD_TRANSMIT;
    bb_rf_params.no_of_values = sizeof(ble_aoa_aod_transmit_per_params_t);
    memcpy(&bb_rf_params.Data[0], &ble_aoa_aod_transmit_per_params, sizeof(ble_aoa_aod_transmit_per_params_t));

    usleep(WAIT_TIME);

    if( rsi_send_to_drv((uint_8 *)&bb_rf_params, BT_PER, sizeof(bb_rf_params)) < 0 )
    {
      printf("Unable to perform bt_transmit\n");
      return -1;
    }
    printf("======== SUCCESS ============\n");

    if (rsi_netlink_deinit() < 0)
    {
      ONEBOX_PRINT ("Netlink Socket creation error\n");
      return -1;
    }

  }
  else
  {
    printf("\nUSAGE to start transmit: %s <pkt_length> <phy_rate> <channel_index> <scrambler_seed> <payload_type> <supp_length> <supp_slot_type> <num_antenna> <ant_switch_pattern> \n <le_channel_type> <tx_power> <hopping_type> <ant_sel>\n",argv[0]);
    return 0;
  }
  return 0;
}
