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
#include <linux/types.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include "onebox_util.h"

#define PER_MODE
//#define PER_BASIC_STATS
#ifdef RSSI_AVG
int kk;
#endif
int main(int argc, char **argv)
{
	int sockfd;
	unsigned int freq;
	int first_time = 1;
	int count = 0, stop_bit = 0;
	int i; 
	unsigned short ch_width = 0;
	FILE *pFile;
	struct iwreq iwr;
#ifdef RSSI_AVG
	int rssi_buff[10];
	int jj=0,avg_loop;
	int rssi_total=0,rssi_avg=0; 
#endif
	unsigned int valid_channels_5_Ghz[]   = { 36, 40, 44, 48, 52, 56, 60, 64, 100,\
									  		  104, 108, 112, 116, 120, 124, 128, 132, 136,\
									          140, 149, 153, 157, 161, 165 
											};
	unsigned int valid_channels_5_Ghz_40Mhz[]   = { 38, 42, 46, 50, 54, 58, 62, 102,\
                							  		106, 110, 114, 118, 122, 126, 130, 134, 138,\
									                151, 155, 159, 163 
											      };
	unsigned int valid_channels_4_9_Ghz_20Mhz[]   = { 184, 188, 192, 196, 8, 12, 16, 0xff, 0x00 };
	unsigned int valid_channels_4_9_Ghz_10Mhz[]   = { 183, 185, 187, 189, 7, 9, 11, 0xff, 0x00 };
	unsigned char enable_40 = 0;
	unsigned char enable_11j = 0;
	unsigned char rate_flags = 0;

	if (argc != 6)
	{
		printf("Onebox dump stats application\n");   
		printf("Usage: %s <rpine_interface> <file_name> <channel num> <start-stop> <Channel BW>\n", argv[0]);
		printf("\tFile_name   		- File to dump status\n");
		printf("\tChannel num 		- Channel to operate\n");
		printf("\tStart-Stop value\t- 0: Start & 1: Stop \n");
		printf("\tChannel BW  		- 0: 20MHz, 2: Upper 40MHz, 4: Lower 40MHz 6: Full 40MHz & 8: 20MHz for 11J Channel\n");

		return 1;
	}

	/* Creating a Socket */
	sockfd = socket(PF_INET, SOCK_DGRAM, 0);
	if(sockfd < 0)
	{
		printf("Unable to create a socket\n");
		return sockfd;
	}

	freq = atoi(argv[3]);
	stop_bit = atoi(argv[4]);
	rate_flags = atoi(argv[5]);
	ch_width = rate_flags & (0x07);
	enable_11j = (rate_flags & BIT(3) );
	
	if( ch_width == BW_U40 || ch_width == BW_L40 || ch_width == BW_F40 )
	{
			enable_40 = 1;
	}

	if(!enable_11j )
	{
			if (freq == 0xFF)
			{
					/* Pass 0xFF so as to skip channel programming */
			}
			else if((freq >= 36 && freq <= 165 && ch_width == BW_20))
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
							printf("Invalid Channel issued by user\n");
							exit(0);
					}
			}
			else if((freq >= 36 && freq <= 165 && enable_40))
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
							printf("Invalid Channel issued by user\n");
							exit(0);
					}
			}
			else if(!(freq <= 14))
			{
					printf("Invalid Channel issued by user\n");
					exit(0);
			}
	}
	else
	{
			if(ch_width == BW_20)
			{
					for(i = 0; i < sizeof(valid_channels_4_9_Ghz_20Mhz)/sizeof(valid_channels_4_9_Ghz_20Mhz[0]); i++)
					{
							if(freq == valid_channels_4_9_Ghz_20Mhz[i])
							{
									break;
							}
					}
					if(i == sizeof(valid_channels_4_9_Ghz_20Mhz)/sizeof(valid_channels_4_9_Ghz_20Mhz[0]))
					{
							printf("Invalid Channel issued by user\n");
							exit(0);
					}
			}
			else if( ch_width == BW_10 )
			{
					for(i = 0; i < sizeof(valid_channels_4_9_Ghz_10Mhz)/sizeof(valid_channels_4_9_Ghz_10Mhz[0]); i++)
					{
							if(freq == valid_channels_4_9_Ghz_10Mhz[i])
							{
									break;
							}
					}
					if(i == sizeof(valid_channels_4_9_Ghz_10Mhz)/sizeof(valid_channels_4_9_Ghz_10Mhz[0]))
					{
							printf("Invalid Channel issued by user\n");
							exit(0);
					}
			}
			else if( ch_width == BW_5 )
			{
						printf("5MHz BW is not supported\n");
						exit(0);
			}
			else
			{
					printf("Invalid BW Configuration\n");
					exit(0);
			}
	}

	pFile = fopen(argv[2], "w");
	if(pFile == NULL)
	{
		printf("Unable to create a file\n");
		return -1;
	}

	while(1)
	{
		per_stats *sta_info = malloc(sizeof(per_stats));
#if 0
		if(sleep(1)!=0)
		{
			printf("Unable to sleep\n");            
			free(sta_info);        
			break;
		}
#endif
		memset(&iwr, 0, sizeof(iwr));
		/*Indicates that it is receive*/
		iwr.u.data.flags = (unsigned short)PER_RECEIVE; 
		strncpy(iwr.ifr_name, argv[1], 7);
		iwr.u.data.pointer = sta_info;
		iwr.u.data.length = sizeof(per_stats);
		iwr.u.data.flags |= (unsigned short)freq << 8;
		*(unsigned short *)iwr.u.data.pointer = (unsigned short)rate_flags;

		if (stop_bit)
		{
			iwr.u.data.flags |= (unsigned short)PER_RCV_STOP;
		}
		if(ioctl(sockfd, ONEBOX_HOST_IOCTL, &iwr) < 0)
		{
			printf("Unable to issue ioctl\n");
			printf("Make sure that wlan protocol is enabled before issuing receive command\n");
			printf("Usage: ./onebox_util rpineX enable_protocol 1\n");
			free(sta_info);
			break;
		}
		else
		{
#ifdef RSSI_AVG
			if (jj < 10){
				rssi_buff[jj] = sta_info->cal_rssi;
				jj = jj+1;
			}
			else {
				rssi_total = 0;
				rssi_buff[kk]=sta_info->cal_rssi;
				for (avg_loop=0;avg_loop<10;avg_loop++){
					rssi_total = rssi_total + rssi_buff[avg_loop];
				}
				rssi_avg = rssi_total/10; 		
				kk++;
				sta_info->cal_rssi = rssi_avg;
				if (kk == 10){
					kk = 0;
				}
			}
#endif	
			freq = 0;
			if (stop_bit || (sta_info->stop_per == 1))
			{
				printf(" RECEIVE STOPPED \n");
				break;
			} 
			if(first_time)
			{
				first_time = 0;
				free(sta_info);
				fflush(pFile);
				continue;
			}
			if((count % 20) == 0)
			{
#ifdef PER_MODE
#ifdef PER_BASIC_STATS
				printf(" %8s %8s %8s %8s\n",
						"crc_pass","crc_fail","false_cca","cal_rssi");
				fprintf(pFile, "%8s %8s %8s %8s\n",
						"crc_pass","crc_fail","false_cca","cal_rssi");
#else
				printf("%12s %12s %12s %12s %12s %12s %12s %12s %12s\n","crc_pass","crc_fail","non_wifi_pkt","false_ed","pkt_abort","fls_rx_start","false_cca","cal_rssi","ack_sent");
				fprintf(pFile,"\n%12s %12s %12s %12s %12s %12s %12s %12s %12s\n","crc_pass","crc_fail","non_wifi_pkt","false_ed","pkt_abort","fls_rx_start","false_cca","cal_rssi","ack_sent");
#endif
			}
#ifdef PER_BASIC_STATS
			printf("%7d %7d %7d %7d\n",
					sta_info->crc_pass,
					sta_info->crc_fail,
					sta_info->cca_idle,
					sta_info->cal_rssi);
			fprintf(pFile, "%7d %7d %7d %7d\n",
					sta_info->crc_pass,
					sta_info->crc_fail,
					sta_info->cca_idle,
					sta_info->cal_rssi);
#else
			printf("%12d %12d %12d %12d %12d %12d %12d %12d %12d\n",
					sta_info->crc_pass,
					sta_info->crc_fail,
					sta_info->cca_stk,  /*non_wifi_pkt*/
					sta_info->cca_not_stk,  /*false_ed*/
					sta_info->pkt_abort,
					sta_info->fls_rx_start,
					sta_info->cca_idle,
					sta_info->cal_rssi,
          sta_info->ack_sent
          );
			fprintf(pFile,"%12d %12d %12d %12d %12d %12d %12d %12d %12d\n",
					sta_info->crc_pass,
					sta_info->crc_fail,
					sta_info->cca_stk,
					sta_info->cca_not_stk,
					sta_info->pkt_abort,
					sta_info->fls_rx_start,
					sta_info->cca_idle,
					sta_info->cal_rssi,
          sta_info->ack_sent
        );
#endif
#else
				printf(" %20s %8s %10s %10s %10s \n",
						"tx_pkts","retries","pkts_drop","rssi","cons_drops");
				fprintf(pFile, "%20s %10s %10s %10s %10s \n",
						"tx_pkts","retries","pkts_drop","rssi","cons_drops");
			}
			printf("%20d %9d %9d %9d %9d \n",
					sta_info->tx_pkts,
					sta_info->tx_retries,
					sta_info->xretries,
					sta_info->bea_avg_rssi,
					sta_info->max_cons_pkts_dropped);
			fprintf(pFile, "%20d %9d %9d %9d %9d\n",
					sta_info->tx_pkts,
					sta_info->tx_retries,
					sta_info->xretries,
					sta_info->bea_avg_rssi,
					sta_info->max_cons_pkts_dropped
					);
#endif

			++count;    
			free(sta_info);
		}    
		fflush(pFile);
	}
	close(sockfd);
	fclose(pFile);
	return 0;
}
