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
	int count = 0;
	int i,ii; 
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
  wlan_iq_struct_t wlan_iq_struct;

	if (argc != 6)
	{
		printf("Onebox dump stats application\n");   
		printf("Usage:\n./wlan_iq_capture <filename> <channel_number> <bandwidth> <num_samples> \n");
		printf("\t< filename >   		- File to dump status\n");
		printf("\t< channel_number >		- Channel to operate\n");
		printf("\t< bandwidth > 		- 0: 20MHz, 2: Upper 40MHz, 4: Lower 40MHz 6: Full 40MHz & 8: 20MHz for 11J Channel\n");
		printf("\t< num_samples > 	number of IQ samples\n");
    printf("\t< start/stop > \n");

		return 1;
	}

	/* Creating a Socket */
	sockfd = socket(PF_INET, SOCK_DGRAM, 0);
	if(sockfd < 0)
	{
		printf("Unable to create a socket\n");
		return sockfd;
	}

		memset(&wlan_iq_struct,0,sizeof(wlan_iq_struct_t));
	freq = atoi(argv[2]);
	rate_flags = atoi(argv[3]);
	ch_width = rate_flags & (0x07);
	enable_11j = (rate_flags & BIT(3) );
  wlan_iq_struct.no_of_samples = atoi(argv[4])  ;


	
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

	pFile = fopen(argv[1], "w");
	if(pFile == NULL)
	{
		printf("Unable to create a file\n");
		return -1;
	}

		wlan_iq_struct.iq_stats =(unsigned short*)malloc(wlan_iq_struct.no_of_samples*4);
		memset(&iwr, 0, sizeof(iwr));
		memset(wlan_iq_struct.iq_stats,0,wlan_iq_struct.no_of_samples*4);
		/*Indicates that it is receive*/
    wlan_iq_struct.freq = freq;
    wlan_iq_struct.rate_flags = rate_flags;
		strncpy(iwr.ifr_name, "rpine0", 7);
		iwr.u.data.pointer = &wlan_iq_struct;
		iwr.u.data.length = sizeof(wlan_iq_struct_t);

		if(ioctl(sockfd, WLAN_I_Q_STATS, &iwr) < 0)
		{
			printf("Unable to issue ioctl\n");
			printf("Make sure that wlan protocol is enabled before issuing receive command\n");
			printf("Usage: ./onebox_util rpine0 enable_protocol 1\n");
      free(wlan_iq_struct.iq_stats);
			return -1;
		}
		else
    {
      printf(" WLAN_IQ_STATS %d samples \n",wlan_iq_struct.no_of_samples);
      fprintf(pFile,"WLAN_IQ_STATS %d samples\n",wlan_iq_struct.no_of_samples);
      wlan_iq_struct.no_of_samples =wlan_iq_struct.no_of_samples * 2;
          for(i = 0 ; i < wlan_iq_struct.no_of_samples ; i++)
          {
            printf("0x%04x, ",wlan_iq_struct.iq_stats[i]);
            fprintf(pFile,"0x%04x,\n",wlan_iq_struct.iq_stats[i]);
          }    

            printf("\n");
    }

  free((void *)wlan_iq_struct.iq_stats);
	close(sockfd);
	fclose(pFile);
	return 0;
}
