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
#include <signal.h>
#include <sys/socket.h>
#include <linux/types.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include "onebox_util.h"

#define PER_MODE

void sniffer_usage(char *argv)
{
	printf("Onebox dump stats application\n");   
	ONEBOX_PRINT
		("Usage: %s base_interface sniffer_mode channel_number channel_BW \n",argv);
	ONEBOX_PRINT
		("Usage: %s base_interface ch_utilization start/stop_bit(1 - start,0 - stop) Stats_interval(in ms) RSSI_threshold False_CCA_RSSI_threshold drop_pkt(1 - enable,0 - disable) discard_payload(1 - enable,0 - disable) Noise_floor_enable\n",argv);
	printf("\tChannel BW  		- 0: 20MHz, 2: Upper 40MHz, 4: Lower 40MHz & 6: Full 40MHz \n");

	return ;
}

int sniffer_getcmdnumber (char *command)
{
	if (!strcmp (command, "sniffer_mode"))
	{
		return SNIFFER_MODE;
	}
	if (!strcmp (command, "ch_utilization"))
	{
		return CH_UTILIZATION;
	}
}

int channel_width_validation(unsigned short ch_width)
{

	if((ch_width != 0) && (ch_width != 2) && (ch_width != 4) && (ch_width != 6))
	{
		return 1;
	}
	return 0;

}

int freq_validation(int freq,unsigned short ch_width)
{
	unsigned int valid_channels_5_Ghz_40Mhz[]   = { 38, 42, 46, 50, 54, 58, 62, 102,\
		106, 110, 114, 118, 122, 126, 130, 134, 138,\
			151, 155, 159, 163 };
	unsigned int valid_channels_5_Ghz[]   = { 36, 40, 44, 48, 52, 56, 60, 64, 100,\
		104, 108, 112, 116, 120, 124, 128, 132, 136,\
			140, 149, 153, 157, 161, 165 };
	int i;
	if (freq == 0xFF)
	{
		/* Pass 0xFF so as to skip channel programming */
	}
	else if((freq >= 36 && freq <= 165 && !ch_width))
	{
		for(i = 0; i < 24; i++)
		{
			if(freq == valid_channels_5_Ghz[i])
			{
				break;
			}
		}
		if(i == 24)
		{
			return 1;
		}
	}
	else if((freq >= 36 && freq <= 165 && ch_width))
	{
		for(i = 0; i < 21; i++)
		{
			if(freq == valid_channels_5_Ghz_40Mhz[i])
			{
				break;
			}
		}
		if(i == 21)
		{
			return 1;
		}
	}
	else if(!(freq <= 14))
	{
		return 1;
	}
	return 0;

}

int stop_flag;
void handler(int signo)
{
  if (signo == SIGINT)
    printf("received SIGINT %d\n",signo);
		stop_flag = 1;
}

int main(int argc, char **argv)
{
	char ifName[32];
	int cmdNo = -1;
	int sockfd;
	int freq;
	int rssi_threshold;
	int first_time = 0;
	int count = 0, start_bit = 0;
	int drop_pkt = 0;
	int discard_payload = 0, stats_interval = 0, noise_floor_enable = 0;
	unsigned short false_cca_rssi_threshold;
	int i; 
	unsigned short ch_width = 0;
	unsigned ch_util[2] = {0};
	struct iwreq iwr;
	per_stats *sta_info = NULL;

	if (argc < 3)
	{
		sniffer_usage(argv[0]);
	}
	else if (argc <= 50)
	{
		/* Get interface name */
		if (strlen(argv[1]) < sizeof(ifName)) {
			strcpy (ifName, argv[1]);
		} else{
			ONEBOX_PRINT("length of given interface name is more than the buffer size\n");	
			return -1;
		}

		cmdNo = sniffer_getcmdnumber (argv[2]);
		//printf("cmd is %d \n",cmdNo);
	}
  
	signal(SIGINT,handler);

	/* Creating a Socket */
	sockfd = socket(PF_INET, SOCK_DGRAM, 0);
	if(sockfd < 0)
	{
		printf("Unable to create a socket\n");
		return sockfd;
	}

	switch (cmdNo)
	{
		case SNIFFER_MODE:     //to use in sniffer mode      
			{
				if(argc != 5)
				{
					printf("Invalid number of arguments \n");
					sniffer_usage(argv[0]);
					return;
				}
				freq = atoi(argv[3]);
				ch_width = atoi(argv[4]);
				if(channel_width_validation(ch_width))
				{
					printf("Invalid Channel BW values \n");
					return;
				}
				if(freq_validation(freq,ch_width))
				{
					printf("Invalid Channel issued by user\n");
					return;
				}
				memset(&iwr, 0, sizeof(iwr));
				iwr.u.data.flags = (unsigned short)PER_RECEIVE;                                
				/*Indicates that it is receive*/
				strncpy (iwr.ifr_name, ifName, IFNAMSIZ);
				iwr.u.data.flags |= (unsigned short)freq << 8;
				iwr.u.data.pointer = &ch_width;


				if(ioctl(sockfd, ONEBOX_HOST_IOCTL, &iwr) < 0)
				{
					printf("Unable to issue ioctl\n");
					return -1;
				}

				break;
			}
		case CH_UTILIZATION:     //to use for channel utilization      
			{
				if(argc != 10)
				{
					printf("Invalid number of arguments \n");
					sniffer_usage(argv[0]);
					return;
				}

        start_bit = atoi(argv[3]);
        if(start_bit != 0 && start_bit != 1)
        {
                printf("Invalid (1 - start)/(0 - stop bit)  \n");
                return;
				}
        rssi_threshold = atoi(argv[5]);
				if(rssi_threshold == 0)
				{
					rssi_threshold = 79;
				}
				if(rssi_threshold < 40 || rssi_threshold > 79)
				{
					printf("Invalid Rssi Threshold should be 40 to 79 \n");
					return;
				}
				drop_pkt = atoi(argv[7]);
				if(drop_pkt != 0 && drop_pkt != 1)
				{
					printf("Invalid (1 - enable)/(0 - disable)  \n");
					return;
				}
				discard_payload = atoi(argv[8]);
				if(discard_payload != 0 && discard_payload != 1)
				{
						printf("Invalid (1 - enable)/(0 - disable)  \n");
						return;
				}
				stats_interval = atoi(argv[4]);
				if(stats_interval == 0)
				{
						printf("STATS INTERVAL Default value is 1 sec   \n");
						stats_interval = 1000;
				}

				false_cca_rssi_threshold = atoi(argv[6]);
				if(false_cca_rssi_threshold > rssi_threshold ) 
				{
						printf("False CCA threshold should not be greater than rssi_threshold\n");
						return;
				}
				noise_floor_enable = atoi(argv[9]);
				if(noise_floor_enable != 0 && noise_floor_enable != 1)
				{
						printf("Invalid (1 - enable)/(0 - disable)  \n");
						return;
				}

				while(1)
				{
					sta_info = malloc(sizeof(per_stats));
					if(stop_flag)
					{
						printf("in check \n");
						start_bit = 0;
					}

					memset(&iwr, 0, sizeof(iwr));
					/*Indicates that it is channel utilization start of stop*/
					if(start_bit == 1)
					{
						if(first_time == 0)
						{

								ch_util[0] = rssi_threshold;
								ch_util[0] |= ((noise_floor_enable & 0XFF) << 11);
								ch_util[0] |= ((discard_payload & 0XFF) << 10);
								ch_util[0] |= ((drop_pkt & 0XFF) << 9);
								ch_util[0] |= (start_bit << 8);
								ch_util[0] |= (stats_interval << 16);
								ch_util[1] = (false_cca_rssi_threshold & 0xFF);
								first_time++;

								iwr.u.data.pointer = ch_util;
								iwr.u.data.flags = CH_UTIL_START;
								strncpy (iwr.ifr_name, ifName, IFNAMSIZ);
								iwr.u.data.length = sizeof(ch_util);
						}
						else
						{
								iwr.u.data.pointer = sta_info;// get sta_info here
								iwr.u.data.length = sizeof(per_stats);
						}
						iwr.u.data.flags = (unsigned short)CH_UTIL_START;
					}
					else                                
					{
							iwr.u.data.flags = (unsigned short)CH_UTIL_STOP;
							ch_util[0] = rssi_threshold;
							ch_util[0] |= ((noise_floor_enable & 0XFF) << 11);
							ch_util[0] |= ((discard_payload & 0XFF) << 10);
							ch_util[0] |= ((drop_pkt & 0XFF) << 9);
							ch_util[0] |= (start_bit << 8);
							ch_util[0] |= (stats_interval << 16);
							ch_util[1] = (false_cca_rssi_threshold & 0xFF);
							iwr.u.data.pointer = ch_util;
					}
					strncpy (iwr.ifr_name, ifName, IFNAMSIZ);
					//printf("&&&&777 %x \n", ch_util);
					if(ioctl(sockfd, ONEBOX_HOST_IOCTL, &iwr) < 0)
					{
							printf("Unable to issue ioctl \n");
							free(sta_info);
							return -1;
					}

					if(start_bit == 1)
					{
						if((count % 20) == 0)
						{
								printf(" TOT_UTIL(%) ");
								printf(" TOT_BITS(Kbps) ");
								printf(" NON_WIFI_COUNT ");
								printf(" NON_WIFI_DUR ");
								printf(" NON_WIFI_Avg_RSSI ");
								printf(" NON_WIFI_MAX_RSSI ");
								printf(" NOISE_FLOOR ");
								printf(" CRC_FAILS ");
								printf(" CRC_PASS\n");
						}
						if(count != 0 && count != 1)
						{
								printf(" %7.3f %%\t",(((float)sta_info->utilization)/((float)sta_info->interval_duration))*100);//(tot_on_air_occupancy / tot_time ));
								printf(" %4d \t",(int)((sta_info->tot_bytes*8)/1000));//(tot_on_air_occupancy / tot_time ));
								printf(" %13d \t",(int)(sta_info->cca_not_stk + sta_info->cca_idle ));
								printf(" %15.3f \t",(( ((float)sta_info->ed_duration) + (float)sta_info->cca_duration)/1000));
								printf(" %4d \t",(int)sta_info->false_cca_avg_rssi);
								printf(" %16d \t",(int)sta_info->max_cca_avg_rssi);
								printf(" %6d \t",(int)sta_info->noise_rssi);
								printf(" %3d \t",(int)sta_info->crc_fail);//(tot_on_air_occupancy / tot_time ));
								printf(" %6d \n",(int)sta_info->crc_pass);//(tot_on_air_occupancy / tot_time ));
						}
					}
					else
					{
						free(sta_info);
						break;
					}
					free(sta_info);
					count++;
				}

				break;
			}
		default:
			printf("Invalid Command\n");
			break;
	}

	close(sockfd);
	return 0;
}
