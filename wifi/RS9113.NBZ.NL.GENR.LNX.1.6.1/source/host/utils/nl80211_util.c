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
#include <linux/types.h>
#include <linux/wireless.h>
#include "onebox_util.h"

void ioctl_get( char *buffer, char *argv[] );
void ioctl_set(char *buffer, char *argv[] );

int main(int argc, char *argv[])
{
	int sockfd;
	int count=0, val;
	unsigned int ii=0,OID,len; 
	struct iwreq iwr;
	struct iw_freq ifrq;
	unsigned char buffer[200], sub_oid=0;
	unsigned int num_args=0, min =0, fixed =0, max=0;
	int status;

	if (argc == 2)
	{
		printf("nl80211_util:Invalid number of arguments\n");   
		return 1;
	}
	else if(argc == 1)
	{
		printf("NL80211 Private IOCTL Application\n");
		printf("Usage: ./nl80211_util <interface name> <ioctl name> <input> \n");
		return 1;
	}

	len = strlen(argv[2]);
	num_args = argc - 3;
	memset(buffer, 0, 200);

	if((strncmp(argv[2], "set", 3) == 0) && argc > 3)
	{
		if(strncmp(argv[2], "set_short_gi", strlen("set_short_gi")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_SHORT_GI;
			buffer[4] = atoi(argv[3]);
			if(buffer[4] < 0 || buffer[4] > 3)
			{
				printf("Enter a valid value\n");
				printf("Usage: 0 - Disable Short_Gi\n");
				printf("Usage: 1 - Enaable Short_Gi for 20MHz\n");
				printf("Usage: 2 - Enaable Short_Gi for 40MHz\n");
				printf("Usage: 3 - Enaable Short_Gi for both 20/40MHz\n");
				return 0;
			}
			ioctl_set(buffer, argv);
			return 0;
		}
		else if(strncmp(argv[2], "set_mode", strlen("set_mode")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_MODE;
			//sub_oid = 2;
			buffer[1] = strlen(argv[3]);
			if(buffer[1])
			{
				memcpy(&buffer[5], argv[3], strlen(argv[3]));
				ioctl_set(buffer, argv);
				//break;
			}
			else
			{
				printf("Please Specify the mode to Set\n");
			}
			return 0;
		}
		else if(strncmp(argv[2], "set_rate", strlen("set_rate")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_RATE;
			buffer[1] = 1;
			buffer[4] = atoi(argv[3]);
			//printf("The rate is %d\n", buffer[1]);
			ioctl_set(buffer, argv);
			return 0;
		}
		else if(strncmp(argv[2], "set_deauth", strlen("set_deauth")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_ROAMING;
			buffer[1] = 1;
			buffer[4] = atoi(argv[3]);
			ioctl_set(buffer, argv);
			return 0;
		}
		else if(strncmp(argv[2], "set_preamble", strlen("set_preamble")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_SHPREAMBLE;
			if(strncmp(argv[3], "auto", strlen("auto")) == 0)
			{
				buffer[1] = 1;
				printf("The preamble value set is auto %d\n", buffer[1]);
			}
			else
			{
				printf("The preamble value set is long %d\n", buffer[1]);
				buffer[1] = 0;
			}
			ioctl_set(buffer, argv);
			return 0;
		}
		else if(strncmp(argv[2], "set_bmiss_th", strlen("set_bmiss_th")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_BEACON_MISS_THRESH;
			buffer[1] = atoi(argv[3]);
			printf("The threshold val is %d\n", buffer[1]);
			ioctl_set(buffer, argv);
			return 0;
		}
#if 0
		else if(strncmp(argv[2], "set_country", strlen("set_country")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_CHANGE_COUNTRY_IE;
		}
		else if(strncmp(argv[2], "set_hide_ssid", strlen("set_hide_ssid")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_HIDESSID;
		}
		else if(strncmp(argv[2], "set_wmm", strlen("set_wmm")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_WMM;
		}
		else if(strncmp(argv[2], "set_ampdu_dens",len) == 0)
		{
			buffer[0] = IEEE80211_PARAM_AMPDU_DENSITY;
		}
		else if(strncmp(argv[2], "set_pureg", strlen("set_pureg")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_PUREG;
		}
		else if(strncmp(argv[2], "set_puren", strlen("set_puren")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_PUREN;
		}
#endif
		else
		{
			printf("Invalid set Ioctl cmd\n");
			printf("In valid Set IOctl command \n Set Ioctls USAGE: \n");
			printf("./nl80211_util vap_name set_short_gi value(0/1/2)\n");
			printf("./nl80211_util vap_name set_mode $mode_name\n");
			printf("./nl80211_util vap_name set_rate $rate_value (Eg: If user wants to set data rate as 6Mbps then value should be given as 12)\n");
			//printf("./nl80211_util vap_name set_privacy\n");
		}
	}
	else if(strncmp(argv[2], "get", 3) == 0)
	{
		if(strncmp(argv[2],"get_short_gi", strlen("get_short_gi")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_SHORT_GI;
			ioctl_get(buffer, &argv[0]);
			printf(" The short_gi is %d \n", buffer[0]);
			return 0;
		}
		else if(strncmp(argv[2],"get_mode", strlen("get_mode")) == 0)
		{
			buffer[1] = 1;
			buffer[0] = IEEE80211_PARAM_MODE;
			ioctl_get(buffer, &argv[0]);
			printf(" The mode is %s \n", buffer);
			return 0;
		}
		else if(strncmp(argv[2],"get_name", strlen("get_name")) == 0)
		{
			buffer[1] = 1;
			buffer[0] = IEEE80211_PARAM_NAME;
			ioctl_get(buffer, &argv[0]);
			printf(" The name is %s \n", buffer);
			return 0;
		}
		else if(strncmp(argv[2],"get_rate", strlen("get_rate")) == 0)
		{
			buffer[1] = 1;
			buffer[0] = IEEE80211_PARAM_RATE;
			ioctl_get(buffer, &argv[0]);
			//printf("The Data rate is %d \n" , buffer[0]);
			printf("The Data rate is %0.1f \n", (float )buffer[0]/2);
			return 0;
		}
		else if(strncmp(argv[2],"get_privacy", strlen("get_privacy")) == 0)
		{
			buffer[0] = IEEE80211_PARAM_PRIVACY;
			ioctl_get(buffer, &argv[0]);
			if(buffer[0] == PRIVACY_ENABLED)
			{
				printf("The privacy is Enabled\n" );
			}
			else if(buffer[0] == PRIVACY_DISABLED)
			{
				printf("The privacy is Disabled\n" );
			}
			return 0;
		}
		else if(strncmp(argv[2],"get_opt_bw", strlen("get_opt_bw")) == 0)
		{
			buffer[1] = 1;
			buffer[0] = IEEE80211_PARAM_BW;
			ioctl_get(buffer, &argv[0]);
			if(buffer[0] == BW_40Mhz)
			{
				printf("Device is in 40Mhz Mode\n");
			}
			else if(buffer[0] == BW_20Mhz)
			{
				printf("Device is in 20Mhz Mode\n");
			}
		}
		else
		{
			printf("In valid Get IOctl command \n Get Ioctls USAGE: \n");
			printf("./nl80211_util vap_name get_short_gi\n");
			printf("./nl80211_util vap_name get_mode\n");
			printf("./nl80211_util vap_name get_name\n");
			printf("./nl80211_util vap_name get_rate\n");
			printf("./nl80211_util vap_name get_privacy\n");
			printf("./nl80211_util vap_name get_opt_bw\n");
		}
	}
    
  
  return 0;
}

void ioctl_get(char *buffer, char *argv[] )
{

	int sockfd;
	struct iwreq iwr;

	sockfd = socket(AF_INET,SOCK_DGRAM,0);

	if(sockfd<0)
	{
		printf("Unable to create a socket\n");
		return; 
	}

	memset(&iwr,0,sizeof(iwr));
	strncpy(iwr.ifr_name, argv[1], IFNAMSIZ);
	iwr.u.data.pointer = &buffer[0];

	if (ioctl(sockfd, NL80211_GET_IOCTL, &iwr) < 0)
	{
		perror(argv[0]);
		printf("!!Please ensure Driver is running\n");     
		return ;
	}
	return ;
	//memcpy(&val, iwr.u.data.pointer, 2);

}
void ioctl_set(char *buffer, char *argv[] )
{

	int sockfd;
	struct iwreq iwr;

	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	if(sockfd<0)
	{
		printf("Unable to create a socket\n");
		return; 
	}
	memset(&iwr,0,sizeof(iwr));
	strncpy(iwr.ifr_name, argv[1], IFNAMSIZ);
	iwr.u.data.pointer = &buffer[0];

	if (ioctl(sockfd, NL80211_SET_IOCTL, &iwr) < 0)
	{
		perror(argv[0]);
		printf("!!Please ensure Driver is running\n");     
		return ;
	}
	return ;

}


#if 0
  switch(OID)
  {
    case OID_RPS_SET_GENERIC_PARAMS:
    {
      if(argc > 3)
      {
        iwr.u.data.length = 1;
        buffer[0] = sub_oid;         
        if((strncmp(argv[3], "0x", 2) == 0) || (strncmp(argv[3], "0X", 2) == 0))
        {
          buffer[1] = strtol(argv[3], NULL, 16);
        }
        else
        {
          buffer[1] = atoi(argv[3]);   
        }
        if (ioctl(sockfd,OID_RPS_SET_GENERIC_PARAMS,&iwr) < 0)
        {
           perror(argv[0]);
           printf("!!Please ensure Lite-Fi and Driver are running\n");     
           return 0;
        }
      }
      else
      {
        printf("!!Invalid number of arguments %d\n",argc);     
        return 0;
      }
      break;
    }  
    case OID_RPS_GET_GENERIC_PARAMS:
    {
      if(argc < 4)
      {
        iwr.u.data.flags = sub_oid;
        if (ioctl(sockfd,OID_RPS_GET_GENERIC_PARAMS,&iwr) < 0)
        {
           perror(argv[0]);
           printf("!!Please ensure Lite-Fi and Driver are running\n");     
           return 0;
        }
        else
        {
          printf("%s:%d\n",argv[2],buffer[0]);
        }    
      }
      else
      {
        printf("!!Invalid number of arguments: %d\n",argc);     
        return 0;
      }
      break;
    } 
    case OID_RPS_SET_PROTOCOL_TYPE: 
    {
      if(argc > 3)
      {
        iwr.u.data.length = 1;
        buffer[0] = *argv[3];
        if (ioctl(sockfd,OID_RPS_SET_PROTOCOL_TYPE,&iwr) < 0)
        {
           perror(argv[0]);
           printf("!!Please ensure Lite-Fi and Driver are running\n");     
           return 0;
        }
      }
      else
      {
        printf("!!Invalid number of arguments %d\n",argc);     
        return 0;
      }
      break;
    }  
    case OID_RPS_GET_PROTOCOL_TYPE:
    {
      if(argc < 4)
      {
        if (ioctl(sockfd,OID_RPS_GET_PROTOCOL_TYPE,&iwr) < 0)
        {
           perror(argv[0]);
           printf("!!Please ensure Lite-Fi and Driver are running\n");     
           return 0;
        }
        else
        {
          printf("%s:%c\n",argv[2],buffer[0]);
        }    
      }
      else
      {
        printf("!!Invalid number of arguments: %d\n",argc);     
        return 0;
      }
      break;
    } 
    case OID_RPS_PER_TX_MODE:
    case OID_RPS_SET_LISTEN_INTERVAL: 
    case OID_RPS_RM_ALL_CHNL_RMRPT: 
    case OID_RPS_SET_MANUAL_BGSCAN: 
    case OID_RPS_GPIO_CONFIG: 
    case OID_RPS_SET_IBSS_ROLE:
    case OID_RPS_SET_REG_PWR_SAVE:
    case OID_RPS_SET_WMM_PWR_SAVE:
    case OID_RPS_SET_BGSCAN_PARAMS:
    case OID_RPS_SET_STATS_PARAMS:
    case OID_RPS_SET_WMM_ADDTS_REQ:
    case OID_RPS_RM_OFF_CHNL_RMRPT: 
    case OID_RPS_SET_EEPROM_READ:
    {
      if(argc > 3)
      {
        iwr.u.data.length = num_args;
        for(ii =0; ii< iwr.u.data.length; ii++)
        {
          if((strncmp(argv[ii+3], "0x", 2) == 0) || (strncmp(argv[ii+3], "0X", 2) == 0))
          {
            buffer[ii] = strtol(argv[ii+3], NULL, 16);
          }
          else
          {
            buffer[ii] = atoi(argv[ii +3]);
          }
        }
        if(ioctl(sockfd,OID,&iwr) < 0)
        {
           perror(argv[0]);
           printf("!!Please ensure Lite-Fi and Driver are running\n");     
           return 0;
        }
      }
      else
      {
        printf("!!Invalid number of arguments %d\n",argc);     
        return 0;
      }
      break;
    }  
    case OID_RPS_GET_WMM_PWR_SAVE:
    case OID_RPS_GET_REG_PWR_SAVE:
    case OID_RPS_GET_STATS:
    case OID_RPS_GET_AP_IP_ADDR:
    {
      if(argc < 4)
      {
        if (ioctl(sockfd,OID,&iwr) < 0)
        {
           perror(argv[0]);
           printf("!!Please ensure Lite-Fi and Driver are running\n");     
           return 0;
        }
        else
        { 
          printf("%s:",argv[2]);
          for(ii =0; ii< iwr.u.data.length; ii++)
          {
            printf("  %d",buffer[ii]);
          }
          printf("\n");
        }    
      }
      else
      {
        printf("!!Invalid number of arguments: %d\n",argc);     
        return 0;
      }
      break;
    } 
    case OID_RPS_SET_BGSCAN_SSID_FLD: 
    {
      if(argc > 3)
      {
        if(strlen(argv[3]) < 33)
        {
          iwr.u.data.length = strlen(argv[3]);
          strncpy((char *)&buffer[0],argv[3],strlen(argv[3]));
        /*  for(ii =0; ii< iwr.u.data.length; ii++)
          {
            printf("%c",buffer[ii]);
          }*/
          if (ioctl(sockfd,OID_RPS_SET_BGSCAN_SSID_FLD,&iwr) < 0)
          {
             perror(argv[0]);
             printf("!!Please ensure Lite-Fi and Driver are running\n");     
             return 0;
          }
        }
        else
        {
          printf("!!Too big SSID\n");     
          return 0;
        }
      }
      else
      {
        printf("!!Invalid number of arguments %d\n",argc);     
        return 0;
      }
      break;
    }  
    default:
      printf("unknown command is called\n");
    break;
  }
#endif
