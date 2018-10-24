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
#include <linux/errno.h>
#include <termios.h>
#include "zb_util.h"


//#endif

int main(int argc, char *argv[])
{      
	//struct iwreq iwr;
	//int tx_pktlen;
    char chan_number,tx_mode; 
	unsigned short tx_pwr,tx_pktlen;
	int sockfd, i;
	int tx_is_in_progress = 0;
	zb_per_params_t zb_per_params;
  	struct bb_rf_param_t bb_rf_params;

	unsigned short tx_rx_switch = 0;
	signed int ret = 0;

	memset(&zb_per_params, 0, sizeof(zb_per_params_t));
	memset(&bb_rf_params, 0, sizeof(bb_rf_params));

	memset(&zb_per_params, 0, sizeof(zb_per_params_t));

	if (argc == 7)
	{

		/* Init netlink socket */
		if (rsi_netlink_init() < 0)
		{
			//ONEBOX_PRINT ("Netlink Socket creation error\n");
			printf("Netlink Socket creation error\n");
			return -1;
		}

		//Enabling the PER
		zb_per_params.enable = 1;
		zb_per_params.power = atoi(argv[1]);
		if((zb_per_params.power > 18) || (zb_per_params.power < 0))
		{
			printf("Invalid power given,must be in between 0 to 18 only \n");
			exit(0);
		}

		tx_pktlen = atoi(argv[2]);
		if((tx_pktlen > 127) || (tx_pktlen < 6))
		{
			printf("Invalid length,Give the length > 6 and < 128 \n");
			exit(0);
		}

		zb_per_params.pkt_length = tx_pktlen;

		zb_per_params.mode = atoi(argv[3]);
		if((zb_per_params.mode !=0) && (zb_per_params.mode != 1))
		{
			printf("Invalid mode,give the 0 or 1 only \n");
			exit(0);
		}

		chan_number = atoi(argv[4]);
		if ((chan_number < 11) || (chan_number > 26))
		{
			printf("Inavlid channel number\n");
			
			return;
		}
		zb_per_params.channel = chan_number;

		zb_per_params.packet_count = atoi(argv[5]); 

		zb_per_params.delay = atoi(argv[6]); // in milliseconds
		//per_params.delay = 0; // in milliseconds

		printf("\nenable: %d\n",zb_per_params.enable);
		printf("\ntx_power: %d\n",zb_per_params.power);
		printf("\ntx_packet_length: %d\n",zb_per_params.pkt_length);
		printf("\ntx_mode: %d\n",zb_per_params.mode);
		printf("\ntx_channel: %d\n",zb_per_params.channel);
		printf("\ntx_pkt_count: %d\n",zb_per_params.packet_count);
		printf("\ntx_pkt_delay: %d\n",zb_per_params.delay);
		// printf("\ntx_power: %d\n",zb_per_params.power);
		//memset(&iwr, 0, sizeof(iwr));
		//strncpy(iwr.ifr_name, "zigbee0", IFNAMSIZ);
		bb_rf_params.value = TX_STATUS;
		if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER,sizeof(bb_rf_params)) < 0 )
		{
			printf("Unable to perform zb_transmit\n");
			return -1;
		}
		if(recv_data((uint_8 *)&bb_rf_params.Data[0]) < 0)
		{
			printf("======== ALREADY IN TRANSMIT SO STOP TRANSMIT FIRST ============\n");
			return -1;
		}

		bb_rf_params.value = ZIGB_PER_TRANSMIT;
		bb_rf_params.no_of_values = sizeof(zb_per_params_t);
		memcpy(&bb_rf_params.Data[0], &zb_per_params, sizeof(zb_per_params_t));

		usleep(500000);

		if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER,sizeof(bb_rf_params)) < 0 )
		{
			printf("Unable to perform zb_transmit\n");
			return -1;
		}


		printf("============= ZB_TRANSMIT_SUCCESS ================== \n");

		if (rsi_netlink_deinit() < 0)
		{
			//ONEBOX_PRINT ("Netling Socket creation error\n");
			printf("Netling Socket creation error\n");
			return -1;
		}
	}
	else if(argc == 2)
	{
		/* Init netlink socket */
		if (rsi_netlink_init() < 0)
		{
			printf("Netlink Socket creation error\n");
			return -1;
		}

		zb_per_params.enable = 0;
		printf("============= ZB_TRANSMIT_STOP ==================");

		bb_rf_params.value = ZIGB_PER_TRANSMIT;
		bb_rf_params.no_of_values = sizeof(zb_per_params_t);
		memcpy(&bb_rf_params.Data[0], &zb_per_params, sizeof(zb_per_params_t));

		usleep(500000);

		if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER, sizeof(bb_rf_params)) < 0 )
		{
			printf("Unable to perform zb_transmit\n");
			return -1;
		}
		printf("======== SUCCESS ============\n");

		if (rsi_netlink_deinit() < 0)
		{
			printf("Netling Socket creation error\n");
			return -1;
		}

	}
	else
	{
		printf("\nUSAGE to start transmit: %s <tx_power> <pkt_length> <tx_mode> <channel_index> <no_of_packets> <delay>\n",argv[0]);
		printf("\ntx_mode:- 0 - Burst, 1 - Continuous mode \n");
		//	printf("\n All the paramters are neglected except channel and filename in receive\n");
		return 1;

	}
        return 0;
}
