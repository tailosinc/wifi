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
#include <linux/types.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include "onebox_util.h"
#include "../wlan/supplicant/linux/src/utils/common.h"

int cal_rate(char* );
int char_to_hwaddr(const char *txt, unsigned char *addr,unsigned int length);
static int hex2num(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

/**
 * hwaddr_compact_aton - Convert ASCII string to MAC address (no colon delimitors format)
 * @txt: MAC address as a string (e.g., "001122334455")
 * @addr: Buffer for the MAC address (ETH_ALEN = 6 bytes)
 * Returns: 0 on success, -1 on failure (e.g., string not a MAC address)
 */
int char_to_hwaddr(const char *txt, u8 *addr,unsigned int length)
{
	int i;

	for (i = 0; i < (length*2); i++) {
		int a, b;

		a = hex2num(*txt++);
		if (a < 0)
			return -1;
		b = hex2num(*txt++);
		if (b < 0)
			return -1;
		*addr++ = (a << 4) | b;
	}

	return 0;
}

//main
int main(int argc, char *argv[])
{      
	struct iwreq iwr;
	int sockfd, i;
	int status;
	int file_size;
	per_packet_t per_packet;
	unsigned short length;
	unsigned char pkt[1536*2] = {0};
	FILE *fp , *fp1 = NULL;
	/*Creating a Socket*/
	sockfd = socket(PF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0)
	{
		printf("Unable to create a socket\n");
		return sockfd;
	}
	memset(&per_packet, 0, sizeof(per_packet_t));
	if (argc == 4 || argc == 3)
	{
		memset(&iwr, 0, sizeof(iwr));
		strncpy(iwr.ifr_name, "rpine0", 6);
		per_packet.enable = atoi(argv[1]);
		if(!(per_packet.enable == 0 || per_packet.enable == 1))
		{
			printf("Invalid Enable field,Enter either 0 or 1 \n");
			exit(0);
		}
		per_packet.length = atoi(argv[2]);
		if(argc == 4)
			per_packet.insert_seq = atoi(argv[3]);
		else
			per_packet.insert_seq = 0;
		if(!(per_packet.insert_seq == 0 || per_packet.insert_seq == 1))
		{
			printf("Invalid sequence number field,Enter either 0 or 1 \n");
			exit(0);
		}
		if(per_packet.length > 1536)
		{
			printf("Invalid length,Give the length <= 1536 \n");
			exit(0);
		}
		fp = fopen("per_packet.txt","r");
		if (fp == NULL) {
			printf("Unable to open file per_packet.txt\n");
			exit(0);
		}
		fp1 = fopen("per_packet.txt","r");
		if (fp1 == NULL) {
			fclose(fp);
			fp = NULL;
			printf("Unable to open file per_packet.txt\n");
			exit(0);
		}
		fseek(fp1,0,SEEK_END);
		file_size = ftell(fp1);
		if(per_packet.length > ((file_size - 1)/2))
		{
			printf("Please enter length less than or equal to content length in per_packet.txt file \n");
			exit(0);
		}
		for(i = 0; i < (per_packet.length * 2); i++)
		{
			pkt[i] = getc(fp); /* returns unsigned char cast to int */
		}
		status = char_to_hwaddr(&pkt[0], &per_packet.packet[0], per_packet.length);
		/* Filling the iwreq structure */ 
		memset(&iwr, 0, sizeof(iwr));
		strncpy(iwr.ifr_name, "rpine0", 6);
		/*Indicates that it is transmission*/
		iwr.u.data.flags = (unsigned short)PER_PACKET;
		iwr.u.data.pointer = (unsigned char *)&per_packet;
		iwr.u.data.length  = sizeof(per_packet);

		if(ioctl(sockfd, ONEBOX_HOST_IOCTL, &iwr) < 0)
		{
			perror(argv[0]);
			printf("Please ensure OneBox Driver is running with valid arguments \tor stop existing transmit utility\n");
		}
		else
		{
			printf("Tx Packet Configuration is DONE\n");
		}
	fclose(fp);
	fclose(fp1);
	}

	else if(argc == 2)
	{
		i = atoi(argv[1]);
		if(i == 0)
		{
			memset(&iwr, 0, sizeof(iwr));
			strncpy(iwr.ifr_name, "rpine0", 7);
			/*Indicates that it is transmission*/
			iwr.u.data.flags = (unsigned short)PER_PACKET;
			iwr.u.data.pointer = (unsigned char *)&per_packet;
			iwr.u.data.length  = sizeof(per_packet);

			if(ioctl(sockfd, ONEBOX_HOST_IOCTL, &iwr) < 0)
			{
				perror(argv[0]);
				printf("&&IOCTL sent is failed \n");
			}
			else
			{
				printf("Default configuration is set\n");
			}
		}
		else
		{
			printf("Please enter either 0 or 1 as an argument, instead of %d to stop..\n",i);
		}
	}

	else
	{
		printf("\nUSAGE   : ./transmit_packet Enable Length \n");
		printf("\nEnable  : 1-enable\n");
		printf("        : 0-Disable \n");
		printf("\nLength  : Length in number of bytes\n");
		printf("\nSEQ_NO  : 1-Takes sequence number from per_packet.txt file\n");
		printf("        : 0-Ignore sequence number from per_packet.txt file\n");
		return 0;
	}
	return 0;
}
