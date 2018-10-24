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
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/types.h>
#include <linux/if.h>
#include <stdlib.h>
#include <linux/wireless.h>
#include <unistd.h>
#include <fcntl.h>
#include <inttypes.h>
#include "onebox_util.h"

/******************************************************************************
 * main()
 *****************************************************************************/
int main (int argc, char *argv[])
{
	int sfd;
	struct iwreq wrq;
	char ifName[32];
	char VapName[32];
	char driver_state[100];
	int proto_state;
	int ret = 0;
	int cmdNo = -1;
	int qnum;
	int cw_mode = 3;
	int cw_type = 0;
	int channel = 1;
	char str[70];
	int fds;
	char fname[20];
	int loop_count = 0;
	int len;
	int beaconvalue;
	int rsi_cmd_flags;
	int flash_file[100];
	unsigned char *data;
	unsigned char ii;
	unsigned char rsi_flags;
	unsigned char verbose = 0;
	unsigned int pkt;
	unsigned char macaddr[IEEE80211_ADDR_LEN];
	struct rsi_clone_params cp;
	struct onebox_wmm_params wmm;
	struct ieee80211_stats stat;
	struct minstrel_node *ar_stats = NULL;
	struct minstrel_rate *mr = NULL;
	struct bb_rf_param_t  bb_rf_params;
	unsigned int bb_addr = 0, bb_addr2 = 0, bb_val = 0, bb_values[2];
	unsigned int rf_addr = 0,rf_val = 0;
	struct bgscan_params bgscan;
	struct bgscan_ssid bg_ssid;
	struct uapsd_params uapsd_req;
	struct ps_req_params ps_req;
	struct master_params_s master;
	struct test_mode test;
	struct aggr_params_s aggr_params;
	int rxfilter_frame = 0;
	int rf_pwr_mode = 0;
	int country;
	char country_name[2];
        char pwrval = 0;
	struct get_info getinfo;
	struct wowlan_config wowlan;
	signed char ant_gain[2];
	unsigned char enable;
	int ant_path, ant_type;
#ifdef IEEE80211K
	struct msrmnt_req_params msrmnt_req;
#endif
	int i,j,k = 0, args;
	int ep_timout = 0, ep_delay = 0, ep_type;
	unsigned cmd_flags =0;
	unsigned value_incr_dec = 0;
	char *decr = NULL;
	EEPROMRW  eeprom;
	unsigned int valid_channels_4_9_Ghz_20Mhz[]   = { 184, 188, 192, 196, 8, 12, 16};
	unsigned char scan_state = 0;
	unsigned char spec_mask = 0;
	unsigned char use_rates[10] = { 0 };
  w_9116_features_t w_9116_features;
  int enable_disable = 0;
  int protocol_id = 0;
  unsigned char value = 0; 
    unsigned int rf_type = 0;
  unsigned char gtk_en = 0;
  programming_stats_t programming_stats;
  prog_structure_t prog_structure;
    ipmu_params_t *ipmu_params;
    gpio_reg_t *gpio_registers;
  //! PUF related variables and structure initalizations
  unsigned char puf_sub_cmd = 0;
  struct puf_init_params_s puf_start;
  struct puf_init_params_s puf_enroll;
  struct puf_set_key_params_s puf_set_key;
  struct puf_get_key_params_s puf_get_key;
  struct puf_aes_data_params_s puf_aes_data;
  FILE *acfp, *kcfp, *aesdatafp, *aesencdatafp, *aesdecdatafp;
  int kc = 0, ac = 0;
  unsigned int data_size = 0;
  //! PUF End initializations

	if (argc < 3)
	{
		usage ();
		return ONEBOX_STATUS_FAILURE;
	}
	else if (argc <= 50)
	{
		/* Get interface name */
		if (strlen(argv[1]) < sizeof(ifName)) {
			strcpy (ifName, argv[1]);
		} else{
			ONEBOX_PRINT("length of given interface name is more than the buffer size\n");	
			return ONEBOX_STATUS_FAILURE;
		}

		cmdNo = getcmdnumber (argv[2], ifName);
		if(cmdNo == ONEBOX_STATUS_FAILURE)
			return ONEBOX_STATUS_FAILURE;
		//printf("cmd is %d \n",cmdNo);
	}

	/* Open socket */
	if ((sfd = socket (AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		ONEBOX_PRINT ("socket creation error\n");
		return 2;
	}

	switch (cmdNo)
	{
		case RSI_VAPCREATE:     //VAP creation      
		{
			while (1)
			{
				get_driver_state (driver_state, ifName);
				if(!strncmp(driver_state, "FSM_OPEN", 8))
				{
					break;
				}
				if (strncmp (driver_state, "FSM_MAC_INIT_DONE", 17) && (strncmp (driver_state, "FSM_DEEP_SLEEP_ENABLE", 21)))
				{

					/* Driver should complete the initialization */
					usleep (200000);
					loop_count++;
					ONEBOX_PRINT("Waiting for driver to finish initialization, %d\n",
					               loop_count);
					if (loop_count >= 20)
					{
						ONEBOX_PRINT("Exiting: Driver Initialization not completed even after waiting for %d00ms\n",
						            loop_count);
						close(sfd);
						return ONEBOX_STATUS_FAILURE;
					}
				}
				else
				{
					break;
				}
			}
			//ONEBOX_PRINT ("Driver initialization is done\n");
			memset (&cp, 0, sizeof (cp));

			if (strlen(argv[3]) < sizeof(VapName)) {
				strcpy (VapName, argv[3]);
			} else {
				ONEBOX_PRINT("length of argv[3] is more than the buffer size\n");	
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}

			strncpy (cp.icp_name, VapName, IFNAMSIZ);
			cp.icp_opmode = getopmode (argv[4]);
			if (cp.icp_opmode == RSI_M_STA)
			{
				if(argc < 6)
				{
					printf("Failed to create vap in station mode\n");
					printf("please see the usage for creating vap in client mode\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				if(argc == 6 && ((!strcmp(argv[5],"sw_bmiss")) || (!strcmp(argv[5], "SW_BMISS"))))
				{
					cp.icp_flags = RSI_NO_STABEACONS;
					ONEBOX_PRINT("Software Beacon miss handling enabled\n");
				}
				else
				{
					ONEBOX_PRINT("Hardware Beacon miss handling enabled...\n");
				}
			}
			cp.icp_flags |= RSI_CLONE_BSSID;
			strncpy (wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.pointer = &cp;
			if (ioctl (sfd, RSIIOCVAPCREATE, &wrq) < 0)
			{
				ONEBOX_PRINT ("Error While starting Device....  \n");
				ONEBOX_PRINT("Device might not be detected or Driver might not be installed properly\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			else
			{
				ONEBOX_PRINT ("VAP created Successfully\n");
			}

			loop_count = 0;
			while (1)
			{
				get_driver_state (driver_state, ifName);
				if (0)
				//if (strncmp (driver_state, "FSM_OPEN", 8))
				{
					usleep (100000);
					loop_count++;
					ONEBOX_PRINT("Vap creation not completed, waiting for %d00ms\n",
					             loop_count);
					if (loop_count >= 20)
					{
						ONEBOX_PRINT("Exiting: Vap creation failed, waiting for %d00ms\n",
						             loop_count);
						close(sfd);
						return ONEBOX_STATUS_FAILURE;
					}
				}
				else
				{
					break;
				}
			}
		}
		break;
		case RSI_VAPDEL:          //VAP deletion
		{
			if(argc == 4) {
				memset (&cp, 0, sizeof (cp));

				if (strlen(argv[3]) < sizeof(VapName)) {
					strcpy (VapName, argv[3]);
				} else {
					ONEBOX_PRINT("length of argv[3] is more than the buffer size\n");	
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}

				strncpy (cp.icp_name, VapName, IFNAMSIZ);
				strncpy (wrq.ifr_name, ifName, IFNAMSIZ);
				wrq.u.data.pointer = &cp;
				if (ioctl (sfd, RSIIOCVAPDEL, &wrq) < 0)
				{
					ONEBOX_PRINT ("onebox_util: error in deleting '%s'\n", VapName);
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				else
				{
					ONEBOX_PRINT ("onebox_util: VAP '%s' has been removed!\n", VapName);
				}
			}
			else
				ONEBOX_PRINT ("Usage: onebox_util base_interface delete_vap vap_name \n");
		}
		break;
		case RSI_ENABLE_PROTOCOL:
		{
			args = argc - 3;
			strcpy (ifName, argv[1]);
			memset (&wrq, 0, sizeof (struct iwreq));
			strncpy (wrq.ifr_name, ifName, IFNAMSIZ);
			
			switch (args) {
			case 1:
				ep_type = atoi(argv[3]);
				ep_timout = 2; 
				ep_delay = 1000;
				break;
			case 2:
				ep_type = atoi(argv[3]);
				ep_timout = atoi(argv[4]);
				ep_delay = 1000;
				break;
			case 3:
				ep_type = atoi(argv[3]);
				ep_timout = atoi(argv[4]);
				ep_delay = atoi(argv[5]);
				if (ep_delay < 50)
					ep_delay = 50;
				break;
			default:
				ONEBOX_PRINT
				  ("Invalid Arguments.\n"
				   "Usage: onebox_util base_interface "
				   "enable_protocol $value $pollcount $polldelay\n"
				   "(1- WLAN,\n 2-BT,\n 4-Zigbee,\n 3-WLAN+BT,\n "
				   "5-WLAN+Zigbee)\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}

			if (!ep_type || ep_type > 5) {
				ONEBOX_PRINT
				  ("Invalid protocol.\n"
				   "Usage: onebox_util base_interface "
				   "enable_protocol $value $polltimeout $polldelay\n"
				   "(1- WLAN,\n 2-BT,\n 4-Zigbee,\n 3-WLAN+BT,\n "
				   "5-WLAN+Zigbee)\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			
			wrq.u.data.flags = 1;
			wrq.u.data.length = ep_type;
			loop_count = 0;
			while (1) {
				usleep(ep_delay * 1000);
				loop_count++;
				if (loop_count >= ep_timout)
				{
					ONEBOX_PRINT("Exiting: Driver init is not yet completed, Please check the coex mode / protocol selected\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				get_driver_state (driver_state, ifName);
				
				if(!strncmp(driver_state,"FSM_MAC_INIT_DONE",17) )
				{
					break;
				}
				else if(!strncmp (driver_state, "FSM_FW_LOADED", 13) )
				{
					if (wrq.u.data.length & WLAN_PROTOCOL)
					{	
						printf("WLAN protocol selected\n");
					}
					if(wrq.u.data.length & BT_PROTOCOL)
					{
						printf("BT protocol selected\n");
		
					}
					if(wrq.u.data.length & ZIGB_PROTOCOL)
					{
						printf("ZIGB protocol selected\n");
					}
					break;
				}
				else
				{
					if(wrq.u.data.length & BT_PROTOCOL)
					{
					 	/*BT protocol only enabled */
						proto_state = get_bt_state();
						if(proto_state == 0)
						{
							printf("No need to call enable protocol for BT only mode\n");
							goto bt_zigbee;
						}
					}
					if(wrq.u.data.length & ZIGB_PROTOCOL)
					{
					 	/*zigb protocol only enabled */
						proto_state = get_zigb_state();
						if(proto_state == 0)
						{
							printf("No need to call enable protocol for zigb only mode\n");
							goto bt_zigbee;
						}
					}
				}
			}
			if (ioctl (sfd, RSIIOCPROTOMODE, &wrq) < 0)
			{
			  	ONEBOX_PRINT ("Error while starting/enabling protocols OR\nTrying to insert already enabled protocol\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}

			ONEBOX_PRINT ("Driver initialization is done\n");

		bt_zigbee:
			break;
		}
		case RSI_DISABLE_PROTOCOL:
		{
			strcpy (ifName, argv[1]);
			memset (&wrq, 0, sizeof (struct iwreq));
			strncpy (wrq.ifr_name, ifName, IFNAMSIZ);

			if (argc != 4) {
				ONEBOX_PRINT 
					("Invalid Number of Arguments.\n");
				ONEBOX_PRINT
					("Usage: onebox_util base_interface disable_protocol $value\n"
					 "(1- WLAN,\n 2-BT,\n 4-Zigbee,\n 3-WLAN+BT,\n 5-WLAN+Zigbee)\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			wrq.u.data.flags = 0;
			wrq.u.data.length = atoi(argv[3]);

			if (wrq.u.data.length > 5) {
				ONEBOX_PRINT 
					("Error while Disabling protocols Select protocol "
					 "number correctly.\n");
				ONEBOX_PRINT
					("Usage: onebox_util base_interface disable_protocol $value\n"
					 "(1- WLAN,\n 2-BT,\n 4-Zigbee,\n 3-WLAN+BT,\n 5-WLAN+Zigbee)\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			if (ioctl (sfd, RSIIOCPROTOMODE, &wrq) < 0)
			{
				ONEBOX_PRINT ("Error while Disabling protocols\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			break;
		}
		case VAPSTATS:
		{
			if (strlen(argv[1]) < sizeof(VapName)) {
				strcpy (VapName, argv[1]);
			} else {
				ONEBOX_PRINT("length of argv[1] is more than the buffer size\n");	
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}

			strncpy (wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.pointer = &stat;
			if ((argc == 3) || 
			    ((argc == 4) && (!strcmp (argv[3], "-v"))) || 
			    ((argc == 5) && (!strcmp (argv[3], "-f"))) || 
			    (argc == 6  &&
			    !strcmp (argv[3],"-v") &&
			    !strcmp (argv[4],"-f")))
			{
				if (ioctl (sfd, RSIVAPSTATS, &wrq) < 0)
				{
					ONEBOX_PRINT ("Error in getting vap stats\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				else
				{
					if ((argc == 4) || (argc == 6))
					{
						verbose = 1;
					}
					if ((argc == 5) || (argc == 6))
					{
						if (strlen(argv[argc - 1]) < sizeof(fname)) {
							strcpy (fname, argv[argc - 1]);
						} else {
							ONEBOX_PRINT("length of argv[argc - 1] is more than the buffer size\n");	
							close(sfd);
							return ONEBOX_STATUS_FAILURE;
						}

						ONEBOX_PRINT ("The  vap stats are written into  %s\n",fname);
						fds = open (fname, O_RDWR | O_CREAT, 0660);
						dup2(fds, 1);
					}
					printvapstats(&stat, verbose);
				}
			}
			else
			{
				usage();
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
		}
		break;
		case STATIONSTATS:
		{
			memset(macaddr, 0, sizeof (macaddr));
			byteconversion (argv[3], macaddr);
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.length = sizeof (struct ieee80211_nodestats);
			wrq.u.data.pointer = malloc (sizeof (struct ieee80211_nodestats) * 32);
			memcpy (wrq.u.data.pointer, macaddr, 6);
			if ((argc == 4) || (argc == 5 && !strcmp (argv[4], "-v")) ||
			    (argc == 6 && !strcmp (argv[4], "-f")) ||
			    (argc == 7 && !strcmp (argv[4], "-v") && !strcmp (argv[5], "-f")))
			{
				if (ioctl (sfd, RSISTATIONSTATS, &wrq) < 0)
				{
					ONEBOX_PRINT ("error in Receiving stationstats\n");
					free(wrq.u.data.pointer);
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				else
				{
					struct ieee80211_nodestats *ni_stats;
					ni_stats = (struct ieee80211_nodestats *) (wrq.u.data.pointer +
					                                           STATION_STATS_OFFSET);
					if ((argc == 5) || (argc == 7))
					{
						verbose = 1;
					}
					if ((argc == 6) || (argc == 7))
					{
						if (strlen(argv[argc - 1]) < sizeof(fname)) {
							strcpy (fname, argv[argc - 1]);
						} else {
							ONEBOX_PRINT("length of argv[argc - 1] is more than the buffer size\n");	
							free(wrq.u.data.pointer);
							close(sfd);
							return ONEBOX_STATUS_FAILURE;
						}
						fds = open (fname, O_RDWR | O_CREAT, 0660);
						ONEBOX_PRINT ("The  station stats are written into  %s\n",
						               fname);
						dup2(fds, 1);
					}
					printstationstats (ni_stats, verbose);
				}
			}
			else
			{
				usage ();
				ret = ONEBOX_STATUS_FAILURE;
			}
			free(wrq.u.data.pointer);
		}
		break;
		case STAINFO:
		{
			memset (macaddr, 0, sizeof (macaddr));
			byteconversion (argv[3], macaddr);
			strncpy (wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.length = sizeof (struct ieee80211req_sta_info);
			wrq.u.data.pointer = malloc (sizeof (struct ieee80211req_sta_info) * 32);
			memcpy (wrq.u.data.pointer, macaddr, 6);
			if ((argc == 4) || (argc == 5 && !strcmp (argv[4], "-v")) ||
			    (argc == 6 && !strcmp (argv[4], "-f")) ||
			    (argc == 7 && !strcmp (argv[4], "-v") && !strcmp (argv[5], "-f")))
			{
				if (ioctl (sfd, RSISTAINFO, &wrq) < 0)
				{
					ONEBOX_PRINT ("error in Receiving station info\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				else
				{
					struct ieee80211req_sta_info *sta_info;
					sta_info = (struct ieee80211req_sta_info *) (wrq.u.data.pointer +
					                                             STATION_STATS_OFFSET);
					if (sta_info->isi_len == 0)
					{
						ONEBOX_PRINT ("no stations connected yet\n");
						break;
					}
					len = 0;
					if ((argc == 5) || (argc == 7))
					{
						verbose = 1;
					}
					if ((argc == 6) || (argc == 7))
					{
						if (strlen(argv[argc - 1]) < sizeof(fname)) {
							strcpy (fname, argv[argc - 1]);
						} else {
							ONEBOX_PRINT("length of argv[argc - 1] is more than the buffer size\n");	
							close(sfd);
							return ONEBOX_STATUS_FAILURE;
						}
						fds = open (fname, O_RDWR | O_CREAT, 0660);
						ONEBOX_PRINT("The  stationinfo stats are written into  %s\n",
						             fname);
						dup2(fds, 1);
					}
					for (ii = 0; ii < 32; ii++) /*MAX number of STA's per VAP is 32*/ 
					{
						if (ii > 0)
						{
							sta_info = (struct ieee80211req_sta_info *) ((uint8_t *) sta_info
							                                                          + len);
						}
						if (sta_info->isi_len == 0)
						{
							break;
						}
						len = sta_info->isi_len;
						printstationinfo (sta_info, verbose);
					}
				}
			}
			else
			{
				usage ();
			}
			free(wrq.u.data.pointer);
		}
		break;
		case RSI_WMM_PARAMS:
		{
			memset (&wmm, 0, sizeof (struct onebox_wmm_params));
			if (strlen(argv[1]) < sizeof(VapName)) {
				strcpy (VapName, argv[1]);
			} else {
				ONEBOX_PRINT("length of argv[1] is more than the buffer size\n");	
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}

			strncpy (wmm.icp_name, VapName, IFNAMSIZ);
			if ((strncasecmp (argv[6], "Self", 4) == 0))
			{
				wmm.isbss = 0;
			}
			else
			{
				wmm.isbss = 1;
			}
			if ((strncasecmp (argv[5], "vo_q", 4) == 0))
			{
				wmm.wmm_ac = WME_VO_Q;
			}
			else if ((strncasecmp (argv[5], "vi_q", 4) == 0))
			{
				wmm.wmm_ac = WME_VI_Q;
			}
			else if ((strncasecmp (argv[5], "bk_q", 4) == 0))
			{
				wmm.wmm_ac = WME_BK_Q;
			}
			else
			{
				wmm.wmm_ac = WME_BE_Q;
			}
			wmm.wmm_val = atoi (argv[4]);	// value to be set 
			if ((strncasecmp (argv[3], "cwmin", 5) == 0))
			{
				wmm.wmm_param = IEEE80211_IOC_WME_CWMIN;
			}
			else if ((strncasecmp (argv[3], "cwmax", 5) == 0))
			{
				wmm.wmm_param = IEEE80211_IOC_WME_CWMAX;
			}
			else if ((strncasecmp (argv[3], "aifs", 4) == 0))
			{
				wmm.wmm_param = IEEE80211_IOC_WME_AIFS;
			}
			else if ((strncasecmp (argv[3], "txop", 4) == 0))
			{
				wmm.wmm_param = IEEE80211_IOC_WME_TXOPLIMIT;
			}
			else if ((strncasecmp (argv[3], "acm", 3) == 0))
			{
				wmm.wmm_param = IEEE80211_IOC_WME_ACM;
			}
			else
			{
				wmm.wmm_param = IEEE80211_IOC_WME_ACKPOLICY;
			}
			wmm.wmm_end_param = atoi (argv[7]);
			/* Issue ioctl to set wme parameters */
			ret = snprintf (str, 70, "%s %s %s %d %d %d %d %d", "iwpriv", ifName,
			          	"setwmmparams", wmm.wmm_param, wmm.wmm_val, wmm.wmm_ac,
			           	wmm.isbss, wmm.wmm_end_param);
			system (str);
			if (ret < -1 || ret >= 70)
				ONEBOX_PRINT("Error in %s %d\n", __func__, __LINE__);
		}
		break;
		case RSI_RESET_PER_Q_STATS:
		{
			/* Note: If qnum entered is not valid all the four edca queues are reset */
			strncpy (wrq.ifr_name, ifName, IFNAMSIZ);
			qnum = atoi (argv[3]);
			wrq.u.data.pointer = &qnum;
			wrq.u.data.flags = RESET_PER_Q_STATS;  
			if (ioctl (sfd, ONEBOX_HOST_IOCTL, &wrq) < 0)
			{
				ONEBOX_PRINT ("Error while resetting stats\n");
				ret = 3;
			}
		}
		break;
		case RSI_AGGR_LIMIT:
		{
			if(argc > 5)
			{
				ONEBOX_PRINT("Aggr limit changes usage: ./onbeox_util interface_name aggr_limit tx_limit rx_limit\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			aggr_params.tx_limit = atoi(argv[3]);
			aggr_params.rx_limit = atoi(argv[4]);
			wrq.u.data.pointer = &aggr_params;
			wrq.u.data.length = sizeof(struct aggr_params_s); 
			wrq.u.data.flags= AGGR_LIMIT; 
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			if (ioctl (sfd, ONEBOX_HOST_IOCTL, &wrq) < 0)
			{  
				printf("Error : setting aggregation parameters\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}  
			else
			{
				printf(" Aggr params set are tx limit %d rx_limit %d\n", atoi(argv[3]), atoi(argv[4]));
				break;
			}
		}
		break;
		case RSI_SET_BEACON:
			if(argc == 4)
			{
				beaconvalue = atoi(argv[3]);
				if ((beaconvalue <= IEEE80211_BINTVAL_MIN_AP) || ( beaconvalue >= IEEE80211_BINTVAL_MAX)) 
				{
					ONEBOX_PRINT("Enter a valid value between 52 and 1000\n");
					break;
				} 
				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
				wrq.u.data.length = beaconvalue;
				wrq.u.data.flags = SET_BEACON_INVL;
				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("Error while setting beacon interval value\n");
					ret = ONEBOX_STATUS_FAILURE;
				}
			}
			else 
				ONEBOX_PRINT( "Usage: onebox_util base_interface set_beacon_intvl value \n");
		break;
		case RSI_USEONLY_RATES:
			args = argc - 3;
			ONEBOX_PRINT( "onebox_util: useonly rates\n");

			if (!args) {
				ONEBOX_PRINT( "Usage: onebox_util base_interface"
				   " useonly_rates value\n");
				break;
			}

			for (i = 0; i < args; i++) 
				use_rates[i] = atoi(argv[i + 3]);

			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.flags = SET_USEONLY_RATES;
			wrq.u.data.pointer = &use_rates;
			wrq.u.data.length = args;
			if (ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) {
				ONEBOX_PRINT("Error issuing supported rates IOCTL\n");
				ret = ONEBOX_STATUS_FAILURE;
			}
		break;
		case RSI_STA_STATE:
				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
				wrq.u.data.flags = CHECK_STA_STATE;
				wrq.u.data.pointer = &scan_state;
				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("ERROR! Unable to check STA State\n");
					ret = ONEBOX_STATUS_FAILURE;
					return ret;
				}
				switch (scan_state)
				{
						case 0: printf("INIT\n");
										break;
						case 1: printf("SCAN\n");
										break;
						case 2: printf("AUTH\n");
										break;
						case 3: printf("ASSOC\n");
										break;
						case 4:	printf("INVALID STATE\n"); 
										break;
						case 5: printf("RUN\n");
										break;
						case 0xff: printf("DOWN\n");
											 break;
						default: printf("INVALID STATE\n");
										 break;
				}
		break;
    		case RSI_SCAN_STOP:
				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
				bgscan.bgscan_threshold = 0;
				bgscan.roam_threshold = 0;
				bgscan.bgscan_periodicity = 0;
				bgscan.active_scan_duration = 30;
				bgscan.passive_scan_duration = 100;

				bgscan.two_probe = 0;
				bgscan.num_bg_channels = 1;
				bgscan.channels2scan[0] = 1;

				wrq.u.data.flags = SET_HOST_SCAN;
				wrq.u.data.pointer = &bgscan;
				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("Error while issuing scan stop IOCTL\n");
					return ONEBOX_STATUS_FAILURE;
				}
		break;
    		case RSI_HOST_SCAN_2G:
				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
				if(argc != 5)
				{
					ONEBOX_PRINT
						("Usage: onebox_util base_interface host_scan_2g  periodicity scan_duration \n" );
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				bgscan.bgscan_threshold = 0;
				bgscan.roam_threshold = 0;
				bgscan.bgscan_periodicity = atoi(argv[3]);
				bgscan.active_scan_duration = atoi(argv[4]);
				if (!bgscan.active_scan_duration ||
						(bgscan.active_scan_duration > 255) ||
						(bgscan.active_scan_duration < 0)) {
					ONEBOX_PRINT ("Active scan duration should be greater than 0 and less than 256\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				bgscan.passive_scan_duration = 100;
				bgscan.two_probe = 0;
				bgscan.num_bg_channels = 14;
				for(j=0; j<bgscan.num_bg_channels; j++)
				{
						 bgscan.channels2scan[j] = (j+1);
				}

				wrq.u.data.flags = SET_HOST_SCAN;
				wrq.u.data.pointer = &bgscan;
				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("Error while issuing host scan IOCTL in 2G band\n");
					return ONEBOX_STATUS_FAILURE;
				}
				break;
		case RSI_HOST_SCAN:
				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
				if((argc < 6) ||(argc != (atoi(argv[5]) + 6)) )
				{
					ONEBOX_PRINT
						("Usage: onebox_util base_interface host_scan  periodicity scan_duration num_of_channels  channels_to_scan \n" );
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				/*bgscan parameters order is
				 * bgscan threshold
				 * roam threshold
				 * bgscan periodicity
				 * active scan duration
				 * passive scan duration
				 * two_probe(en/dis)
				 * num of bg channels
				 * channels to scan
				 */
				for(i = 3; i <= 4; i++) {
					if((atoi(argv[i]) < 0 ))
					{
						printf("scan arguments can't be negative vals\n");
						close(sfd);
						return ONEBOX_STATUS_FAILURE;
					}
				}
				bgscan.bgscan_threshold = 0;
				bgscan.roam_threshold = 0;
				bgscan.bgscan_periodicity = atoi(argv[3]);
				bgscan.active_scan_duration = atoi(argv[4]);
				if (!bgscan.active_scan_duration ||
						(bgscan.active_scan_duration > 255) ||
						(bgscan.active_scan_duration < 0)) {
					ONEBOX_PRINT ("Active scan duration should be greater than 0 and less than 256\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				bgscan.passive_scan_duration = 100;
				if (!bgscan.passive_scan_duration ||
						(bgscan.passive_scan_duration > 255) ||
						(bgscan.passive_scan_duration < 0)) {
					ONEBOX_PRINT ("Passive scan duration should be greater than 0 and less than 256\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}

				bgscan.two_probe = 0;
				if(!(bgscan.two_probe && (bgscan.two_probe ==1) || (bgscan.two_probe == 0)))
				{
					printf("Accepted value for two_probe argumentis either 0 or 1\n");
					break;
				}
				bgscan.num_bg_channels = atoi(argv[5]);
				if((bgscan.num_bg_channels > MAX_NUM_BGCHAN)) {
					ONEBOX_PRINT("Max scan channels allowed is %d, Try entering channels less than %d\n", MAX_NUM_BGCHAN, MAX_NUM_BGCHAN);
					return ONEBOX_STATUS_FAILURE;
				}
				else if((bgscan.num_bg_channels <= 0)) {
					ONEBOX_PRINT("Min number of channels allowed is 1\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}

				for(i = 6, j=0; i < 6 + bgscan.num_bg_channels; i++, j++)
				{
					 if( !(strcmp(argv[i], "8J")) || (!strcmp(argv[i], "8j")))
					 {
							 bgscan.channels2scan[j] = 8;
							 bgscan.channels2scan[j] |= BIT(14); 
							 printf("11J Channel is 8\n");
					 }
					 else if( !(strcmp(argv[i], "12J")) || (!strcmp(argv[i], "12j")))
					 {
							 bgscan.channels2scan[j] = 12;
               						 bgscan.channels2scan[j] |= BIT(14); 
							 printf("11J Channel is 12\n");
					 }
					 else if( !(strcmp(argv[i], "16J")) || (!strcmp(argv[i], "16j")))
					 {
							 bgscan.channels2scan[j] = 16;
               						 bgscan.channels2scan[j] |= BIT(14); 
							 printf("11J Channel is 16\n");
					 }
					 else if(((atoi(argv[i]) >= 1) && (atoi(argv[i]) <= 14)) || 
						((atoi(argv[i]) >= 36) && (atoi(argv[i]) <= 64)) || 
						((atoi(argv[i]) >= 100) && (atoi(argv[i]) <= 140)) || 
						((atoi(argv[i]) >= 149) && (atoi(argv[i]) <= 165)) ||
						((atoi(argv[i]) >= 184) && (atoi(argv[i]) <= 196)) )
					 {
						 bgscan.channels2scan[j] = atoi(argv[i]);
						 if ((bgscan.channels2scan[j] >= 36) && (bgscan.channels2scan[j] <= 64))
						 {      
							 k = ((bgscan.channels2scan[j] - 36) % 4);
						 }
						 else if ((bgscan.channels2scan[j] >= 100) && (bgscan.channels2scan[j] <= 140))
						 {
							 k = ((bgscan.channels2scan[j] - 100) % 4);
						 } 
						 else if(bgscan.channels2scan[j] >= 149 && bgscan.channels2scan[j] <= 165)
						 {
							 k = ((bgscan.channels2scan[j] - 149) % 4);
						 } 
						 else if(bgscan.channels2scan[j] >= 184 && bgscan.channels2scan[j] <= 196 )
						 {
							 k = ((bgscan.channels2scan[j] - 184) % 4);
						 } 
						 if(k)
						 {
							 printf("Invalid Channel given %d by user", atoi(argv[i]));
							 close(sfd);
							 return ONEBOX_STATUS_FAILURE;
						 }	

						 if((bgscan.channels2scan[j] >= 52) && (bgscan.channels2scan[j] <=  140))
						 {
							 /* DFS Channels */
							 bgscan.channels2scan[j] |= BIT(15); 
						 }
					 }
					else
					{
						printf("Invalid Channel given %d by user", atoi(argv[i]));
						close(sfd);
						return ONEBOX_STATUS_FAILURE;
					}
				}

				wrq.u.data.flags = SET_HOST_SCAN;
				wrq.u.data.pointer = &bgscan;
				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("Error while issuing host scan IOCTL \n");
					return ONEBOX_STATUS_FAILURE;
				}
				break;
		case RSI_SET_BGSCAN:
				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
				if((argc < 10) ||(argc != (atoi(argv[9]) + 10)) )
				{
					ONEBOX_PRINT
						("Usage: onebox_util base_interface set_bgscan_params bgscan_threshold rssi_tolerance_threshold periodicity active_scan_duration passive_scan_durations two_probe_en(0/1) num_bg_channels  channels_to_scan \n" );
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				printf("arguments are argv[3] %d argv[4] %d\n", atoi(argv[3]), atoi(argv[4]));
				/*bgscan parameters order is
				 * bgscan threshold
				 * roam threshold
				 * bgscan periodicity
				 * active scan duration
				 * passive scan duration
				 * two_probe(en/dis)
				 * num of bg channels
				 * channels to scan
				 */
				for(i = 3; i <= 7; i++) {
					if((atoi(argv[i]) < 0 ))
					{
						printf("Bg scan arguments can't be negative vals\n");
						close(sfd);
						return ONEBOX_STATUS_FAILURE;
					}
				}
				bgscan.bgscan_threshold = atoi(argv[3]);
				bgscan.roam_threshold = atoi(argv[4]);
				bgscan.bgscan_periodicity = atoi(argv[5]);
				bgscan.active_scan_duration = atoi(argv[6]);
				if (!bgscan.active_scan_duration ||
						(bgscan.active_scan_duration > 255) ||
						(bgscan.active_scan_duration < 0)) {
					ONEBOX_PRINT ("Active scan duration should be greater than 0 and less than 256\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				bgscan.passive_scan_duration = atoi(argv[7]);
				if (!bgscan.passive_scan_duration ||
						(bgscan.passive_scan_duration > 255) ||
						(bgscan.passive_scan_duration < 0)) {
					ONEBOX_PRINT ("Passive scan duration should be greater than 0 and less than 256\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}

				bgscan.two_probe = atoi(argv[8]);
				if(!(bgscan.two_probe && (bgscan.two_probe ==1) || (bgscan.two_probe == 0)))
				{
					printf("Accepted value for two_probe argumentis either 0 or 1\n");
					break;
				}
				bgscan.num_bg_channels = atoi(argv[9]);
				if((bgscan.num_bg_channels > MAX_NUM_BGCHAN)) {
					ONEBOX_PRINT("Max bg channels allowed is %d, Try entering channels less than %d\n", MAX_NUM_BGCHAN, MAX_NUM_BGCHAN);
					ret = ONEBOX_STATUS_FAILURE;
					break;
				}
				else if((bgscan.num_bg_channels <= 0)) {
					ONEBOX_PRINT("Min number of bgscan channels allowed is 1\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}

				for(i = 10, j=0; i < 10 + bgscan.num_bg_channels; i++, j++)
				{
					 if( !(strcmp(argv[i], "8J")) || (!strcmp(argv[i], "8j")))
					 {
							 bgscan.channels2scan[j] = 8;
							 bgscan.channels2scan[j] |= BIT(14); 
							 printf("11J Channel is 8\n");
					 }
					 else if( !(strcmp(argv[i], "12J")) || (!strcmp(argv[i], "12j")))
					 {
							 bgscan.channels2scan[j] = 12;
               						 bgscan.channels2scan[j] |= BIT(14); 
							 printf("11J Channel is 12\n");
					 }
					 else if( !(strcmp(argv[i], "16J")) || (!strcmp(argv[i], "16j")))
					 {
							 bgscan.channels2scan[j] = 16;
               						 bgscan.channels2scan[j] |= BIT(14); 
							 printf("11J Channel is 16\n");
					 }
					 else if(((atoi(argv[i]) >= 1) && (atoi(argv[i]) <= 14)) || 
						((atoi(argv[i]) >= 36) && (atoi(argv[i]) <= 64)) || 
						((atoi(argv[i]) >= 100) && (atoi(argv[i]) <= 140)) || 
						((atoi(argv[i]) >= 149) && (atoi(argv[i]) <= 165)) ||
						((atoi(argv[i]) >= 184) && (atoi(argv[i]) <= 196)) )
					 {
						 bgscan.channels2scan[j] = atoi(argv[i]);
						 if ((bgscan.channels2scan[j] >= 36) && (bgscan.channels2scan[j] <= 64))
						 {      
							 k = ((bgscan.channels2scan[j] - 36) % 4);
						 }
						 else if ((bgscan.channels2scan[j] >= 100) && (bgscan.channels2scan[j] <= 140))
						 {
							 k = ((bgscan.channels2scan[j] - 100) % 4);
						 } 
						 else if(bgscan.channels2scan[j] >= 149 && bgscan.channels2scan[j] <= 165)
						 {
							 k = ((bgscan.channels2scan[j] - 149) % 4);
						 } 
						 else if(bgscan.channels2scan[j] >= 184 && bgscan.channels2scan[j] <= 196 )
						 {
							 k = ((bgscan.channels2scan[j] - 184) % 4);
						 } 
						 if(k)
						 {
							 printf("Invalid Channel given %d by user", atoi(argv[i]));
							 close(sfd);
							 return ONEBOX_STATUS_FAILURE;
						 }	

						 if((bgscan.channels2scan[j] >= 52) && (bgscan.channels2scan[j] <=  140))
						 {
							 /* DFS Channels */
							 bgscan.channels2scan[j] |= BIT(15); 
						 }
					 }
					else
					{
						printf("Invalid Channel given %d by user", atoi(argv[i]));
						close(sfd);
						return ONEBOX_STATUS_FAILURE;
					}
				}

				wrq.u.data.flags = SET_BGSCAN_PARAMS;
				wrq.u.data.pointer = &bgscan;
				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("Error while issuing bgscan IOCTL\n");
					ret = ONEBOX_STATUS_FAILURE;
				}
				break;
		case RSI_DO_BGSCAN:
				printf("<==== Do bgscan ioctl ====>\n");
				if(argc != 3)
				{
					ONEBOX_PRINT
						("Usage: onebox_util base_interface do_bgscan\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;

				}
				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
				wrq.u.data.flags = DO_BGSCAN;
				//if(atoi(argv[3]))
				cmd_flags |= BIT(4); //host triggered bgscan
				printf("The cmd_flags is %02x\n", cmd_flags);
				wrq.u.data.pointer = &cmd_flags;
				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("Error while Issuing do_bgscan IOCTL\n");
					ret = ONEBOX_STATUS_FAILURE;
				}
				break;
		case RSI_BGSCAN_SSID:
			printf("<==== BGSCAN SSID IOCTL ====>\n");
			if(argc != 4)
			{
				ONEBOX_PRINT
					("Usage: onebox_util base_interface bgscan_ssid $ssid_name)\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;

			}
			bg_ssid.ssid_len = strlen(argv[3]);
			if(bg_ssid.ssid_len > 32) {
				ONEBOX_PRINT("The ssid should be lessthan or equal to 32 characters\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			memcpy(bg_ssid.ssid, argv[3], bg_ssid.ssid_len);
			
			printf("ssid is %s  len is %d\n", bg_ssid.ssid, strlen(argv[3]));
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.flags = BGSCAN_SSID;
			wrq.u.data.pointer = &bg_ssid;
			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
			{
				ONEBOX_PRINT("Error while Issuing bgscan_ssid IOCTL\n");
				ret = ONEBOX_STATUS_FAILURE;
			}
		break;
		case RSI_SET_ENDPOINT:
			if(argc == 4)
			{
				rsi_flags = atoi(argv[3]); // store endpoint value
				if (rsi_flags > 9) 
				{
					ONEBOX_PRINT("Enter a valid value between 0 and 9\n");
					ret = ONEBOX_STATUS_FAILURE;
					break;
				} 
				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
				wrq.u.data.flags = rsi_flags << 8; //endpoint type
				wrq.u.data.flags |= SET_ENDPOINT; //endpoint type
				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("Error while setting endpoint\n");
					ret = ONEBOX_STATUS_FAILURE;
				}
			}
		break;
		case RSI_ANT_TYPE:
			if(argc == 5)
			{
				ant_path = atoi(argv[3]); // store ant_sel value
				ant_type = atoi(argv[4]); // store ant_sel value
				if (ant_type > 3 || ant_type < 1) 
				{
					ONEBOX_PRINT("Enter a valid value for antenna type between 1 and 3\n"
					"\t \t	Value = 1 => Type 1 antenna \n"
					"\t \t	Value = 2 => Type 2 antenna \n"
					"\t \t	Value = 3 => Type 3 antenna \n");
					ret = ONEBOX_STATUS_FAILURE;
					break;
				} 
				if( ant_path != 1 && ant_path != 2 )
				{
					ONEBOX_PRINT("Enter a valid value \n"
					"\t \t	Value = 1 => On board antenna path \n"
					"\t \t	Value = 2 => U.FL Path \n");
					ret = ONEBOX_STATUS_FAILURE;
					break;
				}

				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
				wrq.u.data.flags = ant_path << 8; //ANT_SEL type
				wrq.u.data.flags |= ANT_TYPE; //endpoint type
				wrq.u.data.pointer = &ant_type;
				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("Error while setting Antenna Selection\n");
					ret = ONEBOX_STATUS_FAILURE;
				}
			} else {
					printf(" ERROR , usage is ./onebox_util <base_interface> ant_type <ant_path> <ant_type>\n");
					return ONEBOX_STATUS_FAILURE;
			}
		break;
		case RSI_SPECTRAL_MASK:
			if(argc == 4){
				spec_mask = atoi(argv[3]);	
				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
				wrq.u.data.flags = 0;
				wrq.u.data.flags |= SPECTRAL_MASK; //Spectral Mask Type
				wrq.u.data.pointer = &spec_mask;
			
				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("Error while setting spectral mask\n");
					ret = ONEBOX_STATUS_FAILURE;
				}
			} else {
					printf(" ERROR , usage is ./onebox_util <base_interface> spectral_mask <mask_value>\n");
					return ONEBOX_STATUS_FAILURE;
			}
		break;
		case RSI_ANT_SEL:
			if(argc == 4)
			{
				rsi_flags = atoi(argv[3]); // store ant_sel value
				if (rsi_flags > 5 || rsi_flags < 2) 
				{
					ONEBOX_PRINT("Enter a valid value between 2 and 5\n"
					"\t \t	Value = 2; ant_sel => tx_on, ant_sel_b => rx_on \n"
					"\t \t	Value = 3; ant_sel => rx_on, ant_sel_b => tx_on \n"
					"\t \t	Value = 4; ant_sel => 0, ant_sel_b => 0 \n"
					"\t \t	Value = 5; ant_sel => 0, ant_sel_b => 1 \n");
					ret = ONEBOX_STATUS_FAILURE;
					break;
				} 
				ONEBOX_PRINT("ANT_SEL REG value is 0x%x for the selected value %d\n", (rsi_flags << 5), rsi_flags);

				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
				wrq.u.data.flags = rsi_flags << 8; //ANT_SEL type
				wrq.u.data.flags |= ANT_SEL; //endpoint type
				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("Error while setting Antenna Selection\n");
					ret = ONEBOX_STATUS_FAILURE;
				}
			} else {
					printf("ERROR, usage: ./onebox_util <base_interface> ant_sel <value>\n");
			}
			break;
		case RSI_FLASH_VERIFY:
			if(argc != 4)
			{
				rsi_cmd_flags = 1024;
			}
			else
			{
				rsi_cmd_flags = atoi(argv[3]);
			}

			if (verify_flash(ifName, sfd, rsi_cmd_flags) < 0)
			{  
				ONEBOX_PRINT("Error while verifying flash content\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			break;
		case RSI_EEPROM_READ:
			//		printf("<<<< RSI EEPROM READ >>>>>\n");
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			eeprom.length = atoi (argv[3]);
			eeprom.offset = atoi (argv[4]);
			rsi_cmd_flags = eeprom.length;
			wrq.u.data.flags = EEPROM_READ_IOCTL;
			wrq.u.data.pointer = &eeprom;
			wrq.u.data.length = eeprom.length;
			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
			{
				ONEBOX_PRINT("Unable to read EEPROM\n");
				ret = ONEBOX_STATUS_FAILURE;
			}
			else
			{
				for (ii = 0; ii < rsi_cmd_flags; ii++ )
					printf("Received EEPROM value[%d] is: 0x%x\n",ii, eeprom.data[ii]);
			}
			//			  free(eeprom);        
			break;
		case RSI_RF_READ:
			printf("<<<<<<RF_READ request IOCTL>>>>>>\n");
			if( 4 == argc )
			{
				rf_addr = strtol(argv[3],NULL,16);
				//printf("rf_addr: 0x%x\n",rf_addr);

				bb_rf_params.value = 2;
				bb_rf_params.no_of_values = 1;
				bb_rf_params.no_of_fields = 5;
				bb_rf_params.soft_reset = 0;
				bb_rf_params.Data[1] = 0x20;
				bb_rf_params.Data[2] = 0;
				bb_rf_params.Data[3] = rf_addr | BIT(15);
				bb_rf_params.Data[4] = 0;
				bb_rf_params.Data[5] = 0;
        bb_rf_params.protocol_id = 0xff;



				//	bb_rf_params.Data[2] = 1; //no of registers to read
			}
			else
			{
				printf("Invalid No of Arguments\n");
				ONEBOX_PRINT
					("Usage: onebox_util base_interface rf_read  addr \n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}	
			wrq.u.data.pointer = &bb_rf_params;
			wrq.u.data.length = 1; 
			strncpy(wrq.ifr_name,"rpine0",IFNAMSIZ);
			if(ioctl(sfd,ONEBOX_SET_BB_RF,&wrq)<0)
			{  
				printf("Error reading from RF\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}  
			else
			{ 
				printf("Reading RF Successful:\n");
				printf(" data[0] is 0x%x\n",bb_rf_params.Data[0]); 
				printf(" data[1] is 0x%x\n",bb_rf_params.Data[1]); 
			}
			break;
		case RSI_RF_WRITE:

			printf("<<<<<<RF_WRITE request IOCTL>>>>>>\n");
			if( 5 == argc )
			{
				rf_addr = strtol(argv[3],NULL,16);
				rf_val = strtol(argv[4],NULL,16);
				//printf("rf_addr: 0x%x\n",rf_addr);
				//printf("rf_val: 0x%x\n",rf_val);

				bb_rf_params.value = 3;
				bb_rf_params.no_of_values = 1;
				bb_rf_params.no_of_fields = 5;
				bb_rf_params.soft_reset = 0;
				bb_rf_params.Data[1] = 0x20;
				bb_rf_params.Data[2] = rf_val;
				bb_rf_params.Data[3] = rf_addr;
				bb_rf_params.Data[4] = 0;
				bb_rf_params.Data[5] = 0;
        bb_rf_params.protocol_id = 0xff;
			}
			else
			{
				printf("Invalid No of Arguments\n");
				ONEBOX_PRINT
					("Usage: onebox_util base_interface rf_write  addr   data \n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}

			wrq.u.data.pointer = &bb_rf_params;
			wrq.u.data.length = 5; 
			strncpy(wrq.ifr_name,"rpine0",IFNAMSIZ);
			if(ioctl(sfd,ONEBOX_SET_BB_RF,&wrq)<0)
			{  
				printf("Error writing to RF\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}  
			else
			{

				printf("Writing RF Successful:\n");

			}

			break;
		case PROTOCOL_RF_READ:
			printf("<<<<<<PROTOCOL RF_READ request IOCTL>>>>>>\n");
			if( 4 == argc )
			{
				rf_addr = strtol(argv[3],NULL,16);
				bb_rf_params.value = 2;
				bb_rf_params.no_of_values = 1;
				bb_rf_params.no_of_fields = 5;
				bb_rf_params.soft_reset = 0;
				bb_rf_params.Data[1] = 0x20;
				bb_rf_params.Data[2] = 0;
				bb_rf_params.Data[3] = rf_addr | BIT(15);
				bb_rf_params.Data[4] = 0;
				bb_rf_params.Data[5] = 0;
			}
			else
			{
				printf("Invalid No of Arguments\n");
				ONEBOX_PRINT
					("Usage: onebox_util base_interface protocol_rf_read  addr \n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}	
			wrq.u.data.flags = PROTOCOL_RF_READ;
			wrq.u.data.pointer = &bb_rf_params;
			wrq.u.data.length = 1; 
			strncpy(wrq.ifr_name,"rpine0",IFNAMSIZ);
			if(ioctl(sfd,ONEBOX_SET_BB_RF,&wrq)<0)
			{  
				printf("Error reading from RF\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}  
			else
			{ 
				printf("Reading RF Successful:\n");
				printf(" data[0] is 0x%x\n",bb_rf_params.Data[0]); 
				printf(" data[1] is 0x%x\n",bb_rf_params.Data[1]); 
			}
			break;
		case PROTOCOL_RF_WRITE:
			printf("<<<<<< PROTOCOL RF_WRITE request IOCTL>>>>>>\n");
			if( 5 == argc )
			{
				rf_addr = strtol(argv[3],NULL,16);
				rf_val = strtol(argv[4],NULL,16);
				bb_rf_params.value = 3;
				bb_rf_params.no_of_values = 1;
				bb_rf_params.no_of_fields = 5;
				bb_rf_params.soft_reset = 0;
				bb_rf_params.Data[1] = 0x20;
				bb_rf_params.Data[2] = rf_val;
				bb_rf_params.Data[3] = rf_addr;
				bb_rf_params.Data[4] = 0;
				bb_rf_params.Data[5] = 0;
			}
			else
			{
				printf("Invalid No of Arguments\n");
				ONEBOX_PRINT
					("Usage: onebox_util base_interface protocol_rf_write  addr  data  \n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}
			wrq.u.data.flags = PROTOCOL_RF_WRITE;
			wrq.u.data.pointer = &bb_rf_params;
			wrq.u.data.length = 5; 
			strncpy(wrq.ifr_name,"rpine0",IFNAMSIZ);
			if(ioctl(sfd,ONEBOX_SET_BB_RF,&wrq)<0)
			{  
				printf("Error writing to RF\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}  
			else
			{
				printf("Writing RF Successful:\n");
			}
			break;
#if 0
		case RSI_EEPROM_WRITE:
		printf("<<<< RSI EEPROM WRITE >>>>>\n");
		strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
		eeprom.length = atoi (argv[3]);
		data = (unsigned char *)flash_file;
		eeprom.length = (unsigned int)sizeof(flash_file);
		ONEBOX_PRINT("length of file is %d\n",eeprom.length);
		eeprom.offset = atoi (argv[4]);
		while (1)
		{  
			if (eeprom.length > MAX_EEPROM_LEN)
			{  
				len = MAX_EEPROM_LEN;
				rsi_cmd_flags = 1;
			}
			else
			{
				len = eeprom.length;
			}  
			//					for (ii = 0; ii < len; ii++ )
			//            eeprom.data[ii] = atoi (argv[ii+5]);
			memcpy (eeprom.data, (unsigned char *)data, len);
			//            eeprom.data = (unsigned char *)flash_file;
			wrq.u.data.flags = EEPROM_WRITE_IOCTL;
			wrq.u.data.pointer = &eeprom;
			wrq.u.data.length = (len + 10);
			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
			{
				ONEBOX_PRINT("Error while getting PD value\n");
				break;
			}
			else
			{
				printf("Write length is %d, offset is: 0x%x\n", eeprom.length, eeprom.offset);
				if (rsi_cmd_flags)
				{  
					eeprom.length -= MAX_EEPROM_LEN;
					eeprom.offset += MAX_EEPROM_LEN;
					data = (unsigned char *)data + MAX_EEPROM_LEN; 
					rsi_cmd_flags = 0;
				}
				else
				{
					break;
				}  
			}
		}
		break;
#endif	
		case RSI_PS_REQUEST:
		{ 
#if 1
#ifdef DUTY_CYCLE_IOCTL
			if((argc <= 15) || (argc > 16))
#else
			if((argc <= 14) || (argc > 15))
#endif
			{
#ifdef DUTY_CYCLE_IOCTL
				ONEBOX_PRINT
					("Usage: onebox_util base_interface set_ps_params $ps_en/ps_dis(0/1) $sleep_type(1/2) $tx_threshold $rx_threshold $tx_hysterisis $rx_hysterisis $monitor_interval $sleep_duration $listen_interval_duration $num_beacons_per_listen_interval $dtim_interval_duration $num_dtims_per_sleep $duty_cycle\n");
#else
				ONEBOX_PRINT
					("Usage: onebox_util base_interface set_ps_params $ps_en/ps_dis(0/1) $sleep_type(1/2) $tx_threshold $rx_threshold $tx_hysterisis $rx_hysterisis $monitor_interval $sleep_duration $listen_interval_duration $num_beacons_per_listen_interval $dtim_interval_duration $num_dtims_per_sleep\n");
#endif
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
#else
			if((argc <= 11) || (argc > 12))
			{
				ONEBOX_PRINT
					("Usage: onebox_util base_interface set_ps_params $ps_en/ps_dis(0/1) $sleep_type(1/2) $tx_threshold $rx_threshold $tx_hysterisis $rx_hysterisis $monitor_interval $sleep_duration $num_beacons_per_listen_interval\n");
				return;
			}
#endif
			memset(&ps_req, 0, sizeof(struct ps_req_params));
			ps_req.ps_en = atoi(argv[3]);
			ps_req.sleep_type = atoi(argv[4]);

			if(!((ps_req.sleep_type == LP_SLEEP_TYPE) || (ps_req.sleep_type == ULP_SLEEP_TYPE)))
			{
				printf("Entered wrong value \n");
				printf("Please Enter 1- For LP mode\n 2-ULP Mode\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}

			if(ps_req.ps_en) {

				ps_req.tx_threshold = atoi(argv[5]);
				ps_req.rx_threshold = atoi(argv[6]);

				if (ps_req.tx_threshold > 10)
				{
					printf("Making tx threshold  to default value of 1Mbps\n");
					printf("Supported TX threshold is 0 to 10Mbps\n");
					ps_req.tx_threshold = 1;

				}

				if (ps_req.rx_threshold > 10)
				{
					printf("Making rx threshold  to default value of 1Mbps\n");
					printf("Supported RX threshold is 0 to 10Mbps\n");
					ps_req.rx_threshold = 1;

				}

				ps_req.tx_hysterisis = atoi(argv[7]);
				ps_req.rx_hysterisis = atoi(argv[8]);

				if(!ps_req.rx_threshold) {
					/**In case of Fast PSP Profile**/
					ps_req.rx_hysterisis = 0;
				}else	if(ps_req.rx_threshold && (ps_req.rx_hysterisis > ps_req.rx_threshold)) {
					printf("Making RX Hysterisis value to RX threshold as hysterisis value should not be greater than rx threshold\n");
					ps_req.rx_hysterisis = ps_req.rx_threshold;
				}
				

				if(!ps_req.tx_threshold) {
						/**In case of Fast PSP Profile**/
						ps_req.tx_hysterisis = 0;
				}else if(ps_req.tx_threshold && (ps_req.tx_hysterisis > ps_req.tx_threshold)) {
					printf("Making TX Hysterisis value to TX threshold as hysterisis value should not be greater than tx threshold\n");
					ps_req.tx_hysterisis = ps_req.tx_threshold;
				}

				ps_req.monitor_interval = atoi(argv[9]);
				if((ps_req.monitor_interval && !((ps_req.monitor_interval >= 10) && (ps_req.monitor_interval <= 30000))))
				{
					printf("Supported Monitor Interval Range is 10 to 30000ms\n");
					printf("*** Making monitor interval to default value of 50 milliseconds Which is recommended ***\n");
					ps_req.monitor_interval = 50;
				}
				
				ps_req.deep_sleep_wakeup_period = atoi(argv[10]);
				if((ps_req.sleep_type == ULP_SLEEP_TYPE) && !ps_req.deep_sleep_wakeup_period) {
					ps_req.deep_sleep_wakeup_period = 10;
				}

#if 1
				ps_req.listen_interval = atoi(argv[11]);
				ps_req.num_beacons_per_listen_interval = atoi(argv[12]);
				
				if (((int)(atoi(argv[12])) > 4095) || ((int)(atoi(argv[12])) < 0)) {
					printf("Please enter a valid value for num_beacons_per_listen_interval\n");
					printf("Supported Range is 0 to 4095 beacons\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}

				if (((int)(atoi(argv[14])) > 10) || ((int)(atoi(argv[14])) < 0)) {
					printf("Please enter a valid value for num_dtims_per_sleep\n");
					printf("Supported Range is 0 to 10 dtim intervals\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}

				if (((int)(atoi(argv[13])) > 10000) || ((int)(atoi(argv[13])) < 0)) {
					printf("Please enter a valid value for dtim_interval_duration\n");
					printf("Supported Range is 0 to 10000ms\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				ps_req.dtim_interval_duration = atoi(argv[13]);
				ps_req.num_dtims_per_sleep = atoi(argv[14]);	
	
			
				if (!ps_req.listen_interval && !ps_req.num_beacons_per_listen_interval && !ps_req.dtim_interval_duration && !ps_req.num_dtims_per_sleep) {
					printf("Listen_interval and num_beacons_per_listen_interval and dtim_interval_duration and num_dtims_per_sleep for sleep not provided!!\n");
					printf("Wakeup period is set as one dtim!!\n");
					ps_req.num_dtims_per_sleep = 1;
				}
#ifdef DUTY_CYCLE_IOCTL
                                ps_req.duty_cycle = atoi(argv[15]);
#endif
#else
				if (((int)(atoi(argv[11])) > 4095) || ((int)(atoi(argv[11])) < 0)) {
					printf("Please enter a valid value for num_beacons_per_listen_interval\n");
					printf("Supported Range is 0 to 4095 beacons\n");
					return;
				}
				ps_req.num_beacons_per_listen_interval = atoi(argv[11]);
#endif
			}
			wrq.u.data.flags = PS_REQUEST;
			wrq.u.data.pointer = &ps_req;
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
			{
				ONEBOX_PRINT("Error while issuing ps_req ioctl\n");
				ret = ONEBOX_STATUS_FAILURE;
			}
			
			break;
		}
		case RSI_UAPSD_REQ:
		{
			uint8_t sp_len = 0;
			ONEBOX_PRINT("ARGC %d\n", argc);
			if(argc != 6)
			{
				ONEBOX_PRINT
					("Usage: onebox_util base_interface set_uapsd_params $UAPSD_ACS $sp_len $uapsd_wakeup_period \n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			ONEBOX_PRINT("UAPSD IOCTL is issued\n");
			memset(&uapsd_req, 0, sizeof(struct uapsd_params));
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.flags = UAPSD_REQ;
			wrq.u.data.pointer = &uapsd_req;
			uapsd_req.acs = strtod(argv[3],NULL);
			if((uapsd_req.acs) && (uapsd_req.acs != DEFAULT_UAPSD_ACS))
			{
				ONEBOX_PRINT("ACS support for present release is 15, So defaulting the value to 15\n");
				uapsd_req.acs = DEFAULT_UAPSD_ACS;
			}
			//uapsd_req.mimic_support = atoi(argv[5]);
			sp_len = atoi(argv[4]);
			uapsd_req.uapsd_wakeup_period = atoi(argv[5]);

			if(sp_len > MAX_SP_LEN)
			{
				ONEBOX_PRINT("Max SP Length can be 3, so making default value.\n");
				sp_len = DEFAULT_SP_LEN;
			}
			
			uapsd_req.acs |= (sp_len << SP_LEN_POS);
			ONEBOX_PRINT("WME Info is 0x%02x.\n", uapsd_req.acs);
			
			if((uapsd_req.uapsd_wakeup_period) && ((uapsd_req.uapsd_wakeup_period < WAKEUP_PERIOD_MIN) || (uapsd_req.uapsd_wakeup_period > WAKEUP_PERIOD_MAX)))
			{
				ONEBOX_PRINT("UAPSD wakeup period value should be in between (10 to 100), So defaulting the value to 10\n");
				uapsd_req.uapsd_wakeup_period = WAKEUP_PERIOD_MIN;
			}
			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
			{
				ONEBOX_PRINT("Error while issuing ps_req ioctl\n");
				ret = ONEBOX_STATUS_FAILURE;
			}
			break;
		}
		case RSI_RESET_ADAPTER:
		{
			if(argc > 3)
			{
				ONEBOX_PRINT
					("Usage: onebox_util base_interface reset_adapter\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			printf("Issuing reset adapter\n");
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.flags = RESET_ADAPTER;
			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
			{
				ONEBOX_PRINT("Error while issuing reset adapter ioctl\n");
				ret = ONEBOX_STATUS_FAILURE;
			}
			break;
		}
		case RSI_RX_FILTER:
		{
			if(argc != 10)
			{
				ONEBOX_PRINT("Usage: ./onebox_util base_interface set_rx_filter\n \
					PROMISCOUS_MODE (set - 1/remove - 0)\n \
					ALLOW_DATA_ASSOC_PEER(set - 1/remove - 0)\n \
					ALLOW_MGMT_ASSOC_PEER(set - 1/remove - 0)\n \
					ALLOW_CTRL_ASSOC_PEER(set - 1/remove - 0)\n \
					DISALLOW_BEACONS(set - 1/remove - 0)\n \
					ALLOW_CONN_PEER_MGMT_WHILE_BUF_FULL(set - 1/remove - 0)\n \
					DISALLOW_BROADCAST_DATA(set - 1/remove - 0))\n ");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.flags = RX_FILTER;
			wrq.u.data.pointer = &rxfilter_frame;
			for(i = 3; i < 10; i++)
			{	
				if((atoi(argv[i]) < 0) || (atoi(argv[i]) > 1)){
					printf("Invalid values given by user at location %d \n", i);
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				if(atoi(argv[i]))
					rxfilter_frame |= BIT(i-3);
			}
			
			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
			{
				ONEBOX_PRINT("Error while issuing rx filter ioctl\n");
				ret = ONEBOX_STATUS_FAILURE;
			}
			break;
		}                                                                              
		case RSI_TX_RX_RF_PWR_MODE:
		{
			if (argc != 5)
			{
				ONEBOX_PRINT("Usage: ./onebox_util base_interface set_rf_tx_rx_pwr_mode \n \
					      tx_value (0 - High, 1- Medium, 2 - Low) \n \
					      rx_value (0 - High, 1 - Medium, 2 - Low) \n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.flags = RF_PWR_MODE;
			wrq.u.data.pointer = &rf_pwr_mode;
		
			if (atoi(argv[3]) == 0 || atoi(argv[3]) == 1 || atoi(argv[3]) == 2) 
			{
				rf_pwr_mode = atoi(argv[3]);
			}	
			else
			{
				printf("TX RF power mode invalid!\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}

			if (atoi(argv[4]) == 0 || atoi(argv[4]) == 1 || atoi(argv[4]) == 2)
			{
				rf_pwr_mode |= (atoi(argv[4]) << 4);
			}
			else
			{
				printf("RX RF power mode invalid!\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
	
			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
			{
				ONEBOX_PRINT("Error while issuing rf pwr mode ioctl\n");
				ret = ONEBOX_STATUS_FAILURE;
			}
			break;
		}
		case RSI_SET_CW_MODE:
		if (argc != 6)
		{
			ONEBOX_PRINT("Usage: ./onebox_util base_interface channel cw_mode cw_type\n \
					cw_mode (0 - cw mode start for 802.11n channels , 1- cw mode start for 802.11j channels, 2 - stop cw mode)\n \
					cw_type (2 - 5MHz single tone, 5 - DC tone)\n");
			close(sfd);
			return ONEBOX_STATUS_FAILURE;
		}

		channel = atoi (argv[3]);
		cw_mode = atoi (argv[4]);
		cw_type = atoi (argv[5]);
		/*cw_mode=0 is single tone mode and cw_mode=1 is DC mode*/
		if ((cw_mode == 0) ||(cw_mode == 1) )
		{
			switch (cw_type)
			{
				case 1:
				case 2:
				case 3:
				case 4:
				case 5:
					if( cw_type == 2 )
						ONEBOX_PRINT("SET_CW_MODE: Setting the Single tone of 5MHz\n");
					else if( cw_type == 5 )
						ONEBOX_PRINT("SET_CW_MODE: Setting the DC tone at the center frequency\n");
					wrq.u.data.flags = (cw_mode << 8) | (cw_type << 12);
					if(!cw_mode)
					{
						if (channel)
						{
							wrq.u.data.flags |= channel;
						}
						else
						{
							wrq.u.data.flags |= 1; // set channel value to 1 if channel is 0
						}	
					}
					else
					{
						for(i = 0; i < sizeof(valid_channels_4_9_Ghz_20Mhz)/sizeof(valid_channels_4_9_Ghz_20Mhz[0]); i++)
						{
							if(channel == valid_channels_4_9_Ghz_20Mhz[i])
							{	
								wrq.u.data.flags |= channel;
								break;
							}
						}
						if(!(wrq.u.data.flags & channel))
						{
							printf("Invalid 11J Channel issued by user for 20Mhz BW\n");
							exit(0);
						}

					}
					strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
					ONEBOX_PRINT("SET_CW_MODE: channel %d , cw-mode : %d , cw_type : %d\n ",channel, cw_mode, cw_type);
					if(ioctl(sfd, ONEBOX_SET_CW_MODE, &wrq) < 0) 
					{
						ONEBOX_PRINT("Unable to issue CW mode Command %s\n",ifName);
						ret = ONEBOX_STATUS_FAILURE;
					}
					break;	
				default:
					ONEBOX_PRINT(" SET_CW_MODE: Invalid type in CW mode \n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE; 
			}  
		}
		else if( cw_mode == 2 )
		{
			ONEBOX_PRINT("SET_CW_MODE: Disable CW mode \n");
			wrq.u.data.flags = (cw_mode << 8) | (cw_type << 12);
			if (channel)
			{
				wrq.u.data.flags |= channel;
			}
			else
			{
				wrq.u.data.flags |= 1; // set channel value to 1 if channel is 0
			}	
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			ONEBOX_PRINT("SET_CW_MODE: channel %d , cw-mode : %d , cw_type : %d\n ",channel, cw_mode, cw_type);
			if(ioctl(sfd, ONEBOX_SET_CW_MODE, &wrq) < 0) 
			{
				ONEBOX_PRINT("Unable to issue CW stop command %s\n",ifName);
				ret = ONEBOX_STATUS_FAILURE;
			}
		}
		else 
		{
			ONEBOX_PRINT("SET_CW_MODE: Invalid option for cw_mode \n");
			ONEBOX_PRINT("Usage: ./onebox_util base_interface channel cw_mode cw_type\n \
					cw_mode (0 - cw mode start for 802.11n channels , 1- cw mode start for 802.11j channels, 2 - stop cw mode)\n \
					cw_type (2 - 5MHz single tone, 5 - DC tone)\n");
		}
		break;
		case RSI_SET_BB_WRITE:
		if(argc > 3)
		{
			bb_addr = strtol(argv[3],NULL,16);
			bb_val = strtol(argv[4],NULL,16);
			ONEBOX_PRINT("BB addr: 0x%x value 0x%x\n",bb_addr, bb_val);

			bb_rf_params.value = 1;
			bb_rf_params.no_of_values = 2;
			bb_rf_params.soft_reset = 3;
			bb_rf_params.Data[1] = bb_addr;
			bb_rf_params.Data[2] = bb_val;
		}
		if (argc > 5)
		{
			bb_rf_params.no_of_values = (argc - 3);
			for (ii = 3; ii< bb_rf_params.no_of_values - 2+3;ii+=2)
			{
				bb_addr = strtol(argv[ii+2],NULL,16);
				bb_val = strtol(argv[ii+3],NULL,16);
				bb_rf_params.Data[ii] = bb_addr;
				bb_rf_params.Data[ii+1] = bb_val;
				ONEBOX_PRINT("BB addr: 0x%x value 0x%x\n",bb_addr, bb_val);

			}

		}
		wrq.u.data.pointer = &bb_rf_params;
		wrq.u.data.length = 1; 
		strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
		if(ioctl(sfd, ONEBOX_SET_BB_RF, &wrq) < 0)
		{  
			printf("%s %d Error writing to BB\n", __func__, __LINE__);
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}  
		else
			printf("SUCCESS Writing to BB: \n");
		break;
		case RSI_SET_ED_THRESHOLD:

		decr = strstr(argv[3], "decr");
		value_incr_dec = atoi(argv[4]);

		bb_addr = 0x10E;
		bb_addr2 = 0x15D;

		bb_rf_params.value = 0; //BB_READ_TYPE
		bb_rf_params.no_of_values = 4;
		bb_rf_params.soft_reset = 0;
		bb_rf_params.Data[1] = bb_addr;
		bb_rf_params.Data[2] = bb_addr2; // NO_of registers to read
		wrq.u.data.pointer = &bb_rf_params;
		wrq.u.data.length = 1; 
		strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

		if(ioctl(sfd, ONEBOX_SET_BB_RF, &wrq) < 0)
		{  
			printf("%s %d Unable to set ED threshold\n", __func__, __LINE__);
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}  
		printf("The Value Read is 0x%02x\n", bb_rf_params.Data[0]);
		printf("The Value Read is 0x%02x\n", bb_rf_params.Data[1]);
		for(ii =0; ii <= 1; ii++) {

			if(decr) {
				if(bb_rf_params.Data[ii] < (value_incr_dec/2))
				{
					bb_values[ii] = 0;
				}
				else
				{
					bb_values[ii] = (bb_rf_params.Data[ii] - (value_incr_dec/2));
				}
			} else {
				if((bb_rf_params.Data[ii] + (value_incr_dec/2)) > 27) {
					bb_values[ii] = 27;
				} else {
					bb_values[ii] = (bb_rf_params.Data[ii] + (value_incr_dec/2));
				}
			}
			printf("The New Value that is to be written is 0x%02x\n", bb_values[ii]);
		}


		bb_rf_params.value = 1; //BB_WRITE_TYPE
		bb_rf_params.no_of_values = 4;
		bb_rf_params.soft_reset = 3;
		bb_rf_params.Data[1] = bb_addr;
		bb_rf_params.Data[2] = bb_values[0]; 
		bb_rf_params.Data[3] = bb_addr2;
		bb_rf_params.Data[4] = bb_values[1]; 
		wrq.u.data.pointer = &bb_rf_params;
		wrq.u.data.length = 1; 
		strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

		if(ioctl(sfd, ONEBOX_SET_BB_RF, &wrq) < 0)
		{  
			printf("%s %d Error writing to BB\n", __func__, __LINE__);
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}  


		break;

		case RSI_SET_BB_READ:
		if(argc > 3)
		{
			bb_addr = strtol(argv[3],NULL,16);
//			ONEBOX_PRINT("BB addr: 0x%x \n",bb_addr);

			bb_rf_params.value = 0; //BB_READ_TYPE
			bb_rf_params.no_of_values = 2;
			bb_rf_params.soft_reset = 0;
			bb_rf_params.Data[1] = bb_addr;
			bb_rf_params.Data[2] = 1; // NO_of registers to read
		}
		wrq.u.data.pointer = &bb_rf_params;
		wrq.u.data.length = 1; 
		strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
		if(ioctl(sfd, ONEBOX_SET_BB_RF, &wrq) < 0)
		{  
			printf("%s %d Error reading from BB\n", __func__, __LINE__);
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}  
		else
		{
//			printf("SUCCESS Reading from BB: \n");
			printf(" BB_read value is 0x%x\n",bb_rf_params.Data[0]);

			break;
		}
		case RSI_MASTER_READ:
		{
			if(argc != 5)
			{
				ONEBOX_PRINT
					("Usage: onebox_util base_interface master_read $32bit_address $no_bytes_to_read )\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			master.address = strtol(argv[3], NULL, 16);
			master.no_of_bytes = atoi(argv[4]);
			if(master.no_of_bytes >= 4)
			{
				if(master.no_of_bytes % 4){
					ONEBOX_PRINT("Enter valid no of bytes to read either 1, 2 or multiples of 4 )\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
				}
				
			}else if((master.no_of_bytes != 1) && (master.no_of_bytes != 2))
			{
				ONEBOX_PRINT("Enter valid no of bytes to read either 1, 2 or multiples of 4 )\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.flags = MASTER_READ;
			wrq.u.data.pointer = &master;
			master.data = malloc(master.no_of_bytes);
			wrq.u.data.length = sizeof(struct master_params_s); 
			if(ioctl(sfd, RSIIOCPROTOMODE, &wrq) < 0) 
			{
				ONEBOX_PRINT("Error while issuing master_read ioctl\n");
				ret = ONEBOX_STATUS_FAILURE;
			}else if(master.data != NULL)
			{
				printf("data received");
				for(i=0; i < master.no_of_bytes; i++)
				{
					if(!(i%16))
						printf("\n");
					printf("0x%x ", master.data[i]);
				}
				printf("\n");
			}
			free(master.data);
		break;
		}
		case RSI_MASTER_WRITE:
		{
			if(argc != 6 )
			{
				ONEBOX_PRINT
					("Usage: onebox_util base_interface master_write $32bit_address $no_bytes_to_write $data)\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			master.address = strtol(argv[3], NULL, 16);
			master.no_of_bytes = atoi(argv[4]);
			if((master.no_of_bytes !=1) && (master.no_of_bytes != 2) && (master.no_of_bytes != 4))
			//if(master.no_of_bytes !=1)
			{
				ONEBOX_PRINT("Enter valid no of bytes to write either 1, 2 or 4 )\n");
				close(sfd);
				//ONEBOX_PRINT("only 1 byte can be written )\n");
				return ONEBOX_STATUS_FAILURE;
			}
			pkt = strtol(argv[5], NULL, 16);
			master.data =(unsigned char *)&pkt;
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.flags = MASTER_WRITE;
			wrq.u.data.pointer = &master;
			wrq.u.data.length = sizeof(struct master_params_s); 
			if(ioctl(sfd, RSIIOCPROTOMODE, &wrq) < 0) 
			{
				ONEBOX_PRINT("Error while issuing master_write ioctl\n");
				ret = ONEBOX_STATUS_FAILURE;
			}
		break;
		}
		case RSI_TEST_MODE:
		{
			if(argc < 4)
			{
				ONEBOX_PRINT("Usage: onebox_util base_interface test_mode subtype arguments\n");
				close(sfd);
				return ONEBOX_STATUS_FAILURE;
			}
			test.subtype = atoi(argv[3]);
			test.args = atoi(argv[4]);
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.flags = TEST_MODE;
			wrq.u.data.pointer = &test;
			wrq.u.data.length = sizeof(struct test_mode);
			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
			{
				ONEBOX_PRINT("Error while issuing test_mode ioctl\n");
				ret = ONEBOX_STATUS_FAILURE;
			}
		break;
		}
		case RSI_SET_COUNTRY:
		if(argc == 4)
		{
			country = atoi(argv[3]);
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.length = country;
			wrq.u.data.flags = SET_COUNTRY;
			loop_count=0;
			while(1)
			{
				get_driver_state (driver_state, ifName);
				if (strncmp (driver_state, "FSM_MAC_INIT_DONE", 17))
				{

					/* Driver should complete the initialization */
					usleep (200000);
					loop_count++;
					ONEBOX_PRINT("Waiting for driver to finish initialization, %d\n",
							loop_count);
					if (loop_count >= 2)
					{
						ONEBOX_PRINT("Exiting: Driver Initialization not completed even after waiting for %dms\n",
								loop_count);
						close(sfd);
						return ONEBOX_STATUS_FAILURE;
					}
				}
				else
				{
					break;
				}
			}
			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
			{
				ONEBOX_PRINT("Error while setting country information value\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}
			printf("The country Code is %02d\n", country);
		}
		break;
		case RSI_GET_INFO:
			if(argc == 4)
			{
				strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

				wrq.u.data.flags = GET_INFO;
				wrq.u.data.pointer = &getinfo;

				strcpy (getinfo.param_name, argv[3]);
				getinfo.param_length = sizeof(argv[3]);
				
				if(!strcmp(argv[3], "country")) {
					getinfo.data = country_name;
				}	else if(!strcmp(argv[3], "country_code")) {
					getinfo.data = (void *)&country;
				} else {
					ONEBOX_PRINT("Invalid Getinfo Element \n");
					ONEBOX_PRINT
							("Usage: onebox_util base_interface get_info country\n");
					ONEBOX_PRINT
							("Usage: onebox_util base_interface get_info country_code\n");
					close(sfd);
					return ONEBOX_STATUS_FAILURE;
					//break;
				}

				if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
				{
					ONEBOX_PRINT("Error while getting the required country info\n");
					ret = ONEBOX_STATUS_FAILURE;
					break;
				}

				if(!strcmp(argv[3], "country")) {
						printf("The country code is %s\n", country_name);
				}
				else if(!strcmp(argv[3], "country_code")) {
						printf("The country code is %d\n", country);
				}
			} else {
					printf("ERROR, Usage is : ./onebox_util <base_interface> get_info country/country_code \n");
			}
		break;
		case RSI_SET_SCAN_TYPE:
			if(argc !=4) {
				ONEBOX_PRINT("Invalid Arguments\n");
				ONEBOX_PRINT("Usage: onebox_util base_interface set_scan_type $value\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}
			if((atoi(argv[3]) > 3) || (atoi(argv[3]) <= 0 )) {
				printf("Enter value '1'(2.4 Ghz only) '2'(5Ghz only) or '3'(Both 2.4Ghz and 5Ghz) \n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
			wrq.u.data.flags = SET_SCAN_TYPE;
			int band_to_scan = atoi(argv[3]);
			printf("Scan value is %d\n", band_to_scan);
			wrq.u.data.length = band_to_scan;
			loop_count=0;
			while(1)
			{
				get_driver_state (driver_state, ifName);
				if (strncmp (driver_state, "FSM_MAC_INIT_DONE", 17))
				{

					/* Driver should complete the initialization */
					usleep (200000);
					loop_count++;
					ONEBOX_PRINT("Waiting for driver to finish initialization, %d\n",
							loop_count);
					if (loop_count >= 2)
					{
						ONEBOX_PRINT("Exiting: Driver Initialization not completed even after waiting for %dms\n",
								loop_count);
						close(sfd);
						return ONEBOX_STATUS_FAILURE;
					}
				}
				else
				{
					break;
				}
			}
			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
			{
					ONEBOX_PRINT("Error while Setting the required Scan type\n");
					ret = ONEBOX_STATUS_FAILURE;
					break;
			}
		break;
		case RSI_WOWLAN_CONFIG:
		if (argc != 6) {
			ONEBOX_PRINT("Invalid Arguments\n");
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}
		byteconversion (argv[3], wowlan.macaddr);
		ONEBOX_PRINT("MAC ADDR : %02x:%02x:%02x:%02x:%02x\n", 
				wowlan.macaddr[0], wowlan.macaddr[1], 
				wowlan.macaddr[2], wowlan.macaddr[3],
				wowlan.macaddr[4],wowlan.macaddr[5]);
		memset (macaddr, 0, sizeof (macaddr));

		wowlan.host_wakeup_state = atoi(argv[4]);
		if((wowlan.host_wakeup_state < 0) || (wowlan.host_wakeup_state > 1)) {
			ONEBOX_PRINT("HOST WAKEUP: 0- Host Disable, 1- Host wakeup Enable\n");
			break;
		}
		wowlan.flags = atoi(argv[5]);

		printf("Flags %02x\n",wowlan.flags);
		strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
		wrq.u.data.flags = SET_WOWLAN_CONFIG;
		wrq.u.data.length = sizeof(wowlan);
		wrq.u.data.pointer = &wowlan;
		if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
		{
			ONEBOX_PRINT("Error while Configuring WOWLAN Parameters\n");
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}
		break;
		case RSI_SET_EXT_ANT_GAIN:
		if(argc !=5) {
			ONEBOX_PRINT("Invalid Arguments\n");
			ONEBOX_PRINT("Usage: onebox_util base_interface set_ext_ant_gain $2.4GHz_ant_gain $5GHz_ant_gain\n");
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}

		wrq.u.data.flags = SET_EXT_ANT_GAIN;
		ant_gain[0] = atoi(argv[3]);
		ant_gain[1] = atoi(argv[4]);
		wrq.u.data.pointer = ant_gain;
		wrq.u.data.length = sizeof(ant_gain); 
		strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

		if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0)
		{  
			printf("%s %d Error while Setting EXT ANTENNA GAIN\n", __func__, __LINE__);
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}  
		break;
		case RSI_MAX_POWER:
		if( argc == 4 ) {
			wrq.u.data.flags = ENABLE_MAX_POWER;
			enable = (unsigned char )atoi(argv[3]);
			if( enable != 0 && enable != 1) {
					printf("Invalid Usage: Use: ./onebox_util <base_interface> max_power <1(enable) / 0(disable)>\n");
					return ONEBOX_STATUS_FAILURE;
			}
			wrq.u.data.pointer = &enable;
			wrq.u.data.length = sizeof(unsigned char); 
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0)
			{  
				printf("Error while configuring beacon_recv\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}
		} else {
			printf("Invalid Usage: Use: ./onebox_util <base_interface> max_power <1(enable) / 0(disable)>\n");
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}

		break;
		case RSI_BEACON_RECV_DIS:
		if( argc == 4 ) {
			wrq.u.data.flags = CONF_BEACON_RECV;
			enable = (unsigned char )atoi(argv[3]);
			if( enable != 0 && enable != 1) {
					printf("Invalid Usage: Use: ./onebox_util <base_interface> beacon_recv_dis <1(disable) / 0(enable)>\n");
					return ONEBOX_STATUS_FAILURE;
			}
			wrq.u.data.pointer = &enable;
			wrq.u.data.length = sizeof(unsigned char); 
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0)
			{  
				printf("Error while configuring beacon_recv\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}
		} else {
			printf("Invalid Usage: Use: ./onebox_util <base_interface> beacon_recv_dis <1(disable) / 0(enable)>\n");
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}
		break;
		case RSI_GET_TXPOWER:
		{
			wrq.u.data.flags = GET_TXPOWER;
			wrq.u.data.pointer = &pwrval;
			wrq.u.data.length = sizeof(unsigned char); 
			strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

			if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0)
			{  
				printf("Error while reading txpower from driver\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}
			else
			{
				snprintf(str,70,"iwconfig");
				system(str);
			}
		}
		break;
#ifdef IEEE80211K
	case RSI_SET_MSRMNT_PARAMS:									//This ioctl is used to accept the required parameters for rrm request form user for example unicat mac addr ,bssid mac addr 
		if(argc == 3){
			ONEBOX_PRINT("Invalid Arguments\n");
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}
		msrmnt_req.frame_type = atoi(argv[3]);
		if(msrmnt_req.frame_type == CHANNEL_LOAD)
		{	
			if(argc != 5){
				ONEBOX_PRINT("Mac address of destination is not specified\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}
			byteconversion(argv[4], msrmnt_req.ucast_macaddr);	
    }
		else if(msrmnt_req.frame_type == FRAME_REQUEST)
		{
				if(argc != 6){
				ONEBOX_PRINT("Invalid Arguments provide frame_type ucast_mac frame_req_mac\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}
			byteconversion(argv[4], msrmnt_req.ucast_macaddr);		
			byteconversion(argv[5], msrmnt_req.frame_req_macaddr);			
		}
		else if(msrmnt_req.frame_type == BEACON_REPORT)
		{
				if(argc != 6){
				ONEBOX_PRINT("Invalid Arguments Provide: frame_typ ucast_macaddr beacon_mac_addr ssid_filed\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}
			byteconversion(argv[4], msrmnt_req.ucast_macaddr);		
			byteconversion(argv[5], msrmnt_req.bssid_macaddr);			
			
			if(argc == 7)
			{
				msrmnt_req.ssid_len = strlen(argv[6]);
				memcpy(msrmnt_req.ssid_beacon_rpt, argv[6], msrmnt_req.ssid_len);		
			}
			else
			{
				msrmnt_req.ssid_len = 0;
			}
		}
		else if(msrmnt_req.frame_type == MCAST_DIG_REQUEST)
		{
				if(argc != 6){
				ONEBOX_PRINT("Invalid Arguments provide frame_type ucast_mac frame_req_mac\n");
				ret = ONEBOX_STATUS_FAILURE;
				break;
			}
			byteconversion(argv[4], msrmnt_req.ucast_macaddr);		
			byteconversion(argv[5], msrmnt_req.frame_req_macaddr);			
		}


		else if(msrmnt_req.frame_type < 3)
		{
			byteconversion(argv[4], msrmnt_req.ucast_macaddr);		
		}
		strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
		wrq.u.data.flags = SET_MSRMNT_PARAMS;
		wrq.u.data.length = sizeof(msrmnt_req);
		wrq.u.data.pointer = &msrmnt_req;
		if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
		{
			ONEBOX_PRINT("Error while Configuring 11K Measurment Request Parameters\n");
			ret = ONEBOX_STATUS_FAILURE;
			break;
		}
		break;
#endif	
    case RSI_GTK_OFFLOAD:
    	if (argc == 4) {
	        gtk_en = atoi(argv[3]);
	        strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
	        wrq.u.data.flags = GTK_OFFLOAD;
	        wrq.u.data.pointer = &gtk_en;
	        if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0)
	        {
	          ONEBOX_PRINT("Error while enabling/disabling gtk offload\n");
	          ret = ONEBOX_STATUS_FAILURE;
	        }
	      } else {
	        printf("Invalid Usage: Use: ./onebox_util <base_interface> gtk_offload <0/1>\n");
	        return ONEBOX_STATUS_FAILURE;
	      }
    break;
		case WLAN_9116_FEATURE :
		{
      if(argc < 11 ){
        printf("Usage \n");
        printf("./onebox_util rpine0 w_9116_features pll_mode rf_type wireless_mode enable_ppe afe_type dpd SIFSTransmitenable pwrsave_options\n");
        printf("\x1B[31m" "PLL_MODE:" "\x1B[0m" "0-PLLMODE0, 1-PLLMODE1, 2-PLLMODE2 \n"); 
        printf("RF_TYPE[ONLY FOR 2GHz]: 0-External_RF_8111, 1-Internal_RF_9116, 2-AVIACOM_RF \n");
        printf("WIRELESS_MODE:12 – for LP chain enable, else LP chain disable \n");
        printf("ENABLE_PPE: 0-Disable_per_packet_TX_programming, 1-Enable_per_packet_TX_programming_mode_1, 2-Enable_per_packet_TX_programming_mode_2 \n");
        printf("AFE: ? Default=0\n");
        printf("DPD: ? Default=0\n");
        printf("SIFSTransmitenable\n");
        printf("pwrsave_options[0-3]: 0 -Disable [duty cycling & end of frame], 1-[Duty cycling Enabled], 2-[End_of_Frame] 3- Enable [Duty_cycling & End_of_frame] \n");
        break;
      }
      w_9116_features.pll_mode = atoi(argv[3]);
      w_9116_features.rf_type = atoi(argv[4]);
      w_9116_features.wireless_mode = atoi(argv[5]);
      w_9116_features.enable_ppe = atoi(argv[6]);
      w_9116_features.afe_type = atoi(argv[7]);
      w_9116_features.dpd = atoi(argv[8]);
      w_9116_features.SIFSTransmitenable = atoi(argv[9]);
      w_9116_features.pwrsave_options = atoi(argv[10]);
      if(w_9116_features.pll_mode > 2 || w_9116_features.pll_mode < 0 ) {
        printf("Invalid pll_mode Configuration\n");
        printf("\x1B[31m" "PLL_MODE:" "\x1B[0m" "0-PLLMODE0, 1-PLLMODE1, 2-PLLMODE2 \n");
        break;
      }else if( w_9116_features.rf_type > 2 || w_9116_features.rf_type < 0 ) {
        printf("Invalid rf_type Configuration\n");
        printf("RF_TYPE[ONLY FOR 2GHz]: 0-External_RF_8111, 1-Internal_RF_9116, 2-AVIACOM_RF \n");
        break;
      }else if( w_9116_features.enable_ppe > 2 || w_9116_features.enable_ppe < 0 ) {
        printf("Invalid enable_ppe Configuration\n");
        printf("ENABLE_PPE: 0-Disable_per_packet_TX_programming, 1-Enable_per_packet_TX_programming_mode_1, 2-Enable_per_packet_TX_programming_mode_2 \n");
        break;
      }else if( w_9116_features.dpd > 1 || w_9116_features.dpd < 0 ) {
        printf("Invalid dpd Configuration\n");
        printf("DPD: ? \n");
        break;
      }else if( w_9116_features.SIFSTransmitenable > 1 || w_9116_features.SIFSTransmitenable< 0 ) {
        printf("Invalid SIFSTransmitenable Configuration\n");
        printf("SIFSTransmitenable: ? \n");
        break;
      }else if(w_9116_features.pwrsave_options< 0 ) {
        printf("Invalid pwrsave_options Configuration\n");
        printf("pwrsave_options[0-3]: 0 -Disable [duty cycling & end of frame], 1-[Duty cycling Enabled], 2-[End_of_Frame] 3- Enable [Duty_cycling & End_of_frame] \n");
        break;
      }
      wrq.u.data.flags = WLAN_9116_FEATURE ;
      wrq.u.data.pointer = &w_9116_features;
      wrq.u.data.length = sizeof(w_9116_features); 
      strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
      if(ioctl(sfd, RSIIOCPROTOMODE, &wrq) < 0)
      {  
        printf("Error while sending W_9116_features \n");
        ret = ONEBOX_STATUS_FAILURE;
        break;
      }
    }
		break;
		case LOG_STRUCT_PRGMG:
        {
            if(argc < 5 ){
                printf("Usage \n");
                printf("./onebox_util rpine0 prgm_stats enable_disable interval[Decimal]\n");
                break;
            }
            memset(&programming_stats, 0, sizeof(programming_stats_t));
            programming_stats.start_stop = atoi(argv[3]);
            programming_stats.interval =  strtoul(argv[4], NULL, 10);  
            wrq.u.data.flags = LOG_STRUCT_PRGMG ;
            wrq.u.data.pointer = &(programming_stats);
            wrq.u.data.length = sizeof(programming_stats_t);
            strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
            if(ioctl(sfd, RSIIOCPROTOMODE , &wrq) < 0)
            {  
                printf("Error start/stop structure Programming stats from driver\n");
                ret = ONEBOX_STATUS_FAILURE;
                break;
            }
        }
		break;
		case DISABLE_PROGRAMMING:
        {
            if(argc < 4 ){
                printf("Usage \n");
                printf("./onebox_util rpine0  disable_programming value\n");
                printf("<value> – \nBIT(0)     SKIP_BBP_STRUCT_PROG \nBIT(1)    SKIP_RF_STRUCT_PROG \nBIT(2)    SKIP_AFE_STRUCT_PROG \nBIT(3)    SKIP_PERIODIC_PROG \nBIT(4)    SKIP_DC_CALIB_PROG \n");
                break;
            }
            value = atoi(argv[3]);
            wrq.u.data.flags = DISABLE_PROGRAMMING ;
            wrq.u.data.pointer = &value;
            wrq.u.data.length = sizeof(value);
            strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
            if(ioctl(sfd, RSIIOCPROTOMODE, &wrq) < 0)
            {  
                printf("Error in DISABLE_PROGRAMMING from driver\n");
                ret = ONEBOX_STATUS_FAILURE;
                break;
            }
        }
		break;
		case PROG_STRUCTURE:
        {
            if(argc < 5 ){
                printf("Usage \n");
                printf("./onebox_util rpine0  prog_structure <prog_type> <TA_RAM_ADDRESS> \n");
                break;
            }
            memset(&prog_structure, 0, sizeof(prog_structure_t));
            prog_structure.prog_type = atoi(argv[3]);
            prog_structure.TA_RAM_ADDRESS =strtol(argv[4], NULL, 16);// atoi(argv[4]);
            prog_structure.structure_present = 0; 
            prog_structure.bb_rf_flags = 0;
            prog_structure.len = 0;
            wrq.u.data.flags = PROG_STRUCTURE ;
            wrq.u.data.pointer = &prog_structure;
            wrq.u.data.length = sizeof(prog_structure_t);
            strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
            if(ioctl(sfd, RSIIOCPROTOMODE, &wrq) < 0)
            {  
                printf("Error in PROG_STRUCTURE from driver\n");
                ret = ONEBOX_STATUS_FAILURE;
                break;
            }
        }
		break;
        case IPMU_R_W:
        {
            if(argc < 5 ){
                printf("Usage \n");
                printf("./onebox_util rpine0 ipmu_reg <mode> <address> <value>\n");
                printf("<mode>: [ 0 and 2] are for READ \n");
                printf("<mode>: [ 1 and 3] are for WRITE \n");
                break;
            }
            ipmu_params = (ipmu_params_t*)malloc(sizeof(ipmu_params_t));
            memset(ipmu_params, 0, sizeof(ipmu_params_t));
            ipmu_params->mode = atoi(argv[3]);
            ipmu_params->address =strtol(argv[4], NULL, 16);// atoi(argv[4]);
            if(ipmu_params->mode == 1 || ipmu_params->mode == 3 )
            {
                ipmu_params->value = atoi(argv[5]);
            }
            wrq.u.data.flags = IPMU_R_W ;
            wrq.u.data.pointer = ipmu_params;
            wrq.u.data.length = sizeof(ipmu_params_t);
            strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
            if(ioctl(sfd, RSIIOCPROTOMODE, &wrq) < 0) {  
                printf("Unable to issue IPMU READ\n");
                free(ipmu_params);
                ret = ONEBOX_STATUS_FAILURE;
                break;
            } else if (ipmu_params->mode == 0 || ipmu_params->mode == 2 ) {
                printf("IPMU_REG%d: %08x\n",ipmu_params->mode,ipmu_params->value);
            }
            free(ipmu_params);
        }
        break;
        case GPIO_R_W:
        {
            if(argc < 5 ) {
                printf("Usage \n");
                printf("./onebox_util rpine0 gpio_reg read/write  IDX  gpio_mode  value  direction\n");
                printf("./onebox_util rpine0 gpio_reg read/write  IDX  \n");
                break;
            }
            gpio_registers = (gpio_reg_t*)malloc(sizeof(gpio_reg_t));
            if (gpio_registers == NULL)
                return ONEBOX_STATUS_FAILURE;
            memset(gpio_registers, 0, sizeof(gpio_reg_t));
            gpio_registers->read_write = atoi(argv[3]);
            gpio_registers->id = atoi(argv[4]);
            if(gpio_registers->read_write == 1) {
                if(argc < 8 ) {
                    printf("Usage \n");
                    printf("./onebox_util rpine0 gpio_reg read/write  IDX  gpio_mode  value  direction\n");
                    free(gpio_registers);
                    break;
                }
                gpio_registers->mode = atoi(argv[5]);
                gpio_registers->value = atoi(argv[6]);
                gpio_registers->direction = atoi(argv[7]);
            }
            wrq.u.data.flags = GPIO_R_W ;
            wrq.u.data.pointer = gpio_registers;
            wrq.u.data.length = sizeof(gpio_reg_t);
            strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
            if(ioctl(sfd, RSIIOCPROTOMODE, &wrq) < 0)
            {  
                printf("Unable to issue GPIO_R_W \n");
                free(gpio_registers);
                ret = ONEBOX_STATUS_FAILURE;
                break;
            } else {
                if(!gpio_registers->read_write) 
                    printf("GPIO_REG: %04x GPIO_STATE = %d \n",gpio_registers->value,((gpio_registers->value & BIT(13)) >> 13));
            }
            free(gpio_registers);
        }
        break;
    case RSI_PUF_REQUEST:
    {
      if (argc >= 4) {
        wrq.u.data.flags = PUF_REQUEST;
        puf_sub_cmd = atoi(argv[3]);
        if ((puf_sub_cmd < 0) || (puf_sub_cmd > 11)) {
          printf("Enter puf_sub_cmd between 0 and 11 \n");
          return ONEBOX_STATUS_FAILURE;
        }
        switch (puf_sub_cmd)
        {
          case PUF_ENROLL:    //! PUF Enroll operation, if activation code is to be saved in host, after ioctl it creates a file and saves the activation code
            puf_enroll.puf_sub_cmd = puf_sub_cmd;
            if (argc == 5) {
              puf_enroll.puf_ac_source = atoi(argv[4]);
              if (puf_enroll.puf_ac_source) {
                puf_enroll.activation_code = malloc(ACTIVATION_CODE_SIZE);
              }
              wrq.u.data.pointer = &puf_enroll;
              wrq.u.data.length = sizeof(struct puf_init_params_s);
              strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
              if (ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
              {
                printf("PUF Enroll Operation Failed\n");
                free(puf_enroll.activation_code);
                return ONEBOX_STATUS_FAILURE;
              }
              if (puf_enroll.puf_ac_source) {
                acfp = fopen("puf_ac.txt", "w");
                if (acfp == NULL) {
                  printf("Unable to open file for writing activation code\n");
                  ret = ONEBOX_STATUS_FAILURE;
                } else {
                  printf("PUF Enroll Operation Successfull\n");
                  printf("Got activation code writing into file: puf_ac.txt\n");
                  for (ac = 0; ac < ACTIVATION_CODE_SIZE; ac++) {
                    fprintf(acfp, "0x%x,\n", puf_enroll.activation_code[ac]);
                  }
                }
                fclose(acfp);
                free(puf_enroll.activation_code);
              }
            } else {
              printf("Invalid Usage: Use: ./onebox_util <base_interface> puf_req <sub_cmd> <puf_act_code_src>\n");
              return ONEBOX_STATUS_FAILURE;
            }
            return ret;
          case PUF_START:   //! PUF Start operation, if activation code is available in host, it copies ac from file and performs ioctl
            puf_start.puf_sub_cmd = puf_sub_cmd;
            if (argc >= 5) {
              puf_start.puf_ac_source = atoi(argv[4]);
              if (puf_start.puf_ac_source) {
                if (argc == 6) {
                  acfp = fopen(argv[5], "r");
                  if (acfp == NULL) {
                    printf("Unable to open file %s \n", argv[5]);
                    fclose(acfp);
                    return ONEBOX_STATUS_FAILURE;
                  } else {
                    unsigned char tempac[8];
                    puf_start.activation_code = malloc(ACTIVATION_CODE_SIZE);
                    for (kc = 0; kc < ACTIVATION_CODE_SIZE; kc++) {
                      fscanf(acfp, "%s", tempac);
                      if ((tempac[0] == '0') && (tempac[1] == 'x')) 
                        sscanf(&tempac[2], "%x", &puf_start.activation_code[kc]); 
                    }
                  }
                } else {
                  printf("Invalid Usage: Use: ./onebox_util <base_interface> puf_req <sub_cmd> <puf_act_code_src> <puf_act_code_file>\n");
                  return ONEBOX_STATUS_FAILURE;
                }
              }
            } else {
              printf("Invalid Usage: Use: ./onebox_util <base_interface> puf_req <sub_cmd> <puf_act_code_src> <puf_act_code_file>\n");
              return ONEBOX_STATUS_FAILURE;
            }
            wrq.u.data.pointer = &puf_start;
            wrq.u.data.length = sizeof(struct puf_init_params_s);
            strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

            if (ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
            {
              printf("PUF Start Operation Failed\n");
              ret = ONEBOX_STATUS_FAILURE;
            } else {
              printf("PUF Start Operation Successfull\n");
            }
            free(puf_start.activation_code);
            return ret;
          case PUF_SET_KEY:           //! PUF Set Key/IntrinsicKey operation, takes key argument and performs ioctl and then saves key code in file 
          case PUF_SET_INTRINSIC_KEY:
            if (argc >= 6) {
              puf_set_key.puf_sub_cmd = puf_sub_cmd;
              puf_set_key.key_index = atoi(argv[4]);
              puf_set_key.key_size = atoi(argv[5]);
              if ((puf_set_key.key_index > 15) || (puf_set_key.key_size > 1)) {
                printf("Enter key_index value <= 15, key_size value 0 or 1 \n");
                return ONEBOX_STATUS_FAILURE;
              }
              if (argc == 7) {
                if (puf_sub_cmd == PUF_SET_KEY) {
                  if (strlen(argv[6]) != ((puf_set_key.key_size + 1) * 16)) {
                    printf("Enter key of given key_size value %d -> %d bytes key \n", puf_set_key.key_size, ((puf_set_key.key_size + 1) * 16));
                    return ONEBOX_STATUS_FAILURE;
                  }
                  memcpy(puf_set_key.key, argv[6], ((puf_set_key.key_size + 1) * 16));
                }
              } else {
                printf("Invalid Usage: Use: ./onebox_util <base_interface> puf_req <sub_cmd> <key_index> <key_size> <key_input> \n");
                return ONEBOX_STATUS_FAILURE;
              }
              wrq.u.data.pointer = &puf_set_key;  
              wrq.u.data.length = sizeof(struct puf_set_key_params_s);
            } else {
              printf("Invalid Usage: Use: ./onebox_util <base_interface> puf_req <sub_cmd> <key_index> <key_size> <key_input> \n");
              return ONEBOX_STATUS_FAILURE;
            }
            strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

            if (ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
            {
              printf("PUF Set Key Operation Failed\n");
              return ONEBOX_STATUS_FAILURE;
            } else {
              char keycodefile[128];
              if (puf_sub_cmd == PUF_SET_KEY) {
                sprintf(keycodefile, "puf_keycode_%d.txt", puf_set_key.key_index);
              } else if(puf_sub_cmd == PUF_SET_INTRINSIC_KEY) {
                sprintf(keycodefile, "puf_keycode_intr_%d.txt", puf_set_key.key_index);
              }
              kcfp = fopen(keycodefile, "w");
              if (kcfp == NULL) {
                printf("Unable to create file for storing keycode \n");
                ret = ONEBOX_STATUS_FAILURE;
              } else {
                printf("PUF Set Key Operation Successfull\n");
                printf("Writing KeyCode to file puf_keycode_%d.txt\n", puf_set_key.key_index);
                printf(" -- Generated Key Code -- \n");
                for (kc = 0; kc < KEY_CODE_SIZE; kc++) {
                  fprintf(kcfp, "0x%x,\n", puf_set_key.key_code[kc]);
                  printf("%x\t", puf_set_key.key_code[kc]);
                }
                printf("\n");
              }            
              fclose(kcfp);
            }
            return ret;
          case PUF_GET_KEY:       //! PUF Get Key/Load Key operation, gets keycode from file input and performs ioctl
          case PUF_LOAD_KEY:
            if (argc >= 5) {
              puf_get_key.puf_sub_cmd = puf_sub_cmd;
              kcfp = fopen(argv[4], "r");
              if (kcfp == NULL) {
                printf("Unable to open file %s \n", argv[4]);
                return ONEBOX_STATUS_FAILURE;
              } else {
                unsigned char tempkc[8];
                for (kc = 0; kc < KEY_CODE_SIZE; kc++) {
                  fscanf(kcfp, "%s", tempkc);
                  if ((tempkc[0] == '0') && (tempkc[1] == 'x')) 
                    sscanf(&tempkc[2], "%x", &puf_get_key.key_code[kc]); 
                }
                if (puf_sub_cmd == PUF_LOAD_KEY) {
                  if (argc == 6) {
                    puf_get_key.key_holder = atoi(argv[5]);
                  } else {
                    printf("Invalid Usage: Use: ./onebox_util <base_interface> puf_req <sub_cmd> <key_code_file> <key_holder> \n");
                    return ONEBOX_STATUS_FAILURE;
                  }
                }
                wrq.u.data.pointer = &puf_get_key;  
                wrq.u.data.length = sizeof(struct puf_get_key_params_s);
              }
              fclose(kcfp);
              strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

              if (ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
              {
                printf("PUF GET/LOAD Key Operation Failed\n");
                return ONEBOX_STATUS_FAILURE;
              } else {
                printf("PUF GET/LOAD Key Operation Successful\n");
                if (puf_get_key.puf_sub_cmd == PUF_GET_KEY) {
                  printf(" -- Extracted Key - %d Bytes-- \n", puf_get_key.key_size);
                  printf("%s\n", puf_get_key.key);
                }
              }
            } else {
              printf("Invalid Usage: Use: ./onebox_util <base_interface> puf_req <sub_cmd> <key_code_file> \n");
              return ONEBOX_STATUS_FAILURE;
            }
            return ret;
          case PUF_AES_ENCRYPT:     //! PUF AES Encryption operation, takes input of plain text from file and performs ioctl and saves encrypted data in file 
          case PUF_AES_DECRYPT:     //! PUF AES Decryption operation, takes input of encrypted data from file and performs ioctl and saves decrypted data in file
          case PUF_AES_MAC:         //! PUF AES MAC operation, takes input of plain text from file and performs ioctl and display MAC data for the given input data
            if (argc == 12) { 
              puf_aes_data.puf_sub_cmd = puf_sub_cmd;
              if (puf_sub_cmd == PUF_AES_MAC) {
                puf_aes_data.mode = 1;
              } else {
                puf_aes_data.mode = atoi(argv[4]);
              }
              if (puf_aes_data.mode > 1) {
                printf("Enter proper mode 0 - AES ECB, 1 - AES CBC \n");
                return ONEBOX_STATUS_FAILURE;
              }
              puf_aes_data.key_source = atoi(argv[5]);
              if (puf_aes_data.key_source > 1) {
                printf("Enter proper key_source 0 - PUF, 1 - AES \n");
                return ONEBOX_STATUS_FAILURE;
              }
              puf_aes_data.key_size = atoi(argv[6]);
              if (puf_aes_data.key_size > 1) {
                printf("Enter proper key_size 0 - 128 bit, 1 - 256 bit \n");
                return ONEBOX_STATUS_FAILURE;
              }
              if (puf_aes_data.key_source) {
                if (strlen(argv[7]) != ((puf_aes_data.key_size + 1) * 16)) {
                  printf("Enter key of given key_size value %d -> %d bytes key \n", puf_aes_data.key_size, ((puf_aes_data.key_size + 1) * 16));
                  return ONEBOX_STATUS_FAILURE;
                }
                memcpy(puf_aes_data.key, argv[7], ((puf_aes_data.key_size + 1) * 16));
              }
              puf_aes_data.data_size = atoi(argv[8]);
              if ((puf_aes_data.data_size < 16) || (puf_aes_data.data_size > 128)) {
                printf("Enter data size between 16/32 bytes & 128 bytes \n");
                return ONEBOX_STATUS_FAILURE;
              }
              if(puf_aes_data.data_size % ((puf_aes_data.key_size + 1) * 16)) {
                printf("Enter data size that are multiples of key_size \n");
                return ONEBOX_STATUS_FAILURE;
              }
              puf_aes_data.data = malloc(puf_aes_data.data_size);
              aesdatafp = fopen(argv[9], "r");
              if (aesdatafp == NULL) {
                printf("Unable to open file %s \n", argv[9]);
                free(puf_aes_data.data);
                return ONEBOX_STATUS_FAILURE;
              } else {
                if (puf_sub_cmd == PUF_AES_ENCRYPT || puf_sub_cmd == PUF_AES_MAC) {
                  fread(puf_aes_data.data, sizeof(unsigned char), puf_aes_data.data_size, aesdatafp);
                } else {
                  unsigned char tempac[8];
                  for (ac = 0; ac < puf_aes_data.data_size; ac++) {
                    fscanf(aesdatafp, "%s", tempac);
                    if ((tempac[0] == '0') && (tempac[1] == 'x')) 
                      sscanf(&tempac[2], "%x", &puf_aes_data.data[ac]); 
                  }
                }
                fclose(aesdatafp);
              }
              if (puf_aes_data.mode) {
                puf_aes_data.iv_size = atoi(argv[10]);
                if (puf_aes_data.iv_size > 1) {
                  printf("Enter proper iv size 0 - 128 bit, 1 - 256 bit \n");
                  free(puf_aes_data.data);
                  return ONEBOX_STATUS_FAILURE;
                }
                if (strlen(argv[11]) != ((puf_aes_data.iv_size + 1) * 16)) {
                  printf("Enter key of given key_size value %d -> %d bytes key \n", puf_aes_data.iv_size, ((puf_aes_data.iv_size + 1) * 16));
                  free(puf_aes_data.data);
                  return ONEBOX_STATUS_FAILURE;
                }
                memcpy(puf_aes_data.iv, argv[11], ((puf_aes_data.iv_size + 1) * 16));
              }
              if (puf_sub_cmd == PUF_AES_MAC) { 
                puf_aes_data.enc_dec_mac_data = malloc(16);
              } else {
                puf_aes_data.enc_dec_mac_data = malloc(puf_aes_data.data_size);
              }
              wrq.u.data.pointer = &puf_aes_data;  
              wrq.u.data.length = sizeof(struct puf_aes_data_params_s);

              strncpy(wrq.ifr_name, ifName, IFNAMSIZ);

              if (ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
              {
                printf("PUF AES Operation Failed\n");
                free(puf_aes_data.data);
                free(puf_aes_data.enc_dec_mac_data);
                return ONEBOX_STATUS_FAILURE;
              } else {
                if (puf_aes_data.puf_sub_cmd == PUF_AES_ENCRYPT) {
                  printf("PUF AES Encryption Operation Successfull\n");
                  printf("Writing Encrypted data to file - aes_enc_data.txt\n");
                  aesencdatafp = fopen("aes_enc_data.txt","w");
                  if (aesencdatafp == NULL) {
                    printf("Unable to open file for saving encrypted data\n");
                    free(puf_aes_data.data);
                    free(puf_aes_data.enc_dec_mac_data);
                    return ONEBOX_STATUS_FAILURE;
                  } else {
                    for (ac = 0; ac < puf_aes_data.data_size; ac++) {
                      fprintf(aesencdatafp, "0x%x,\n", puf_aes_data.enc_dec_mac_data[ac]);
                    }
                    fclose(aesencdatafp);
                  }
                } else if (puf_aes_data.puf_sub_cmd == PUF_AES_DECRYPT) {
                  printf("PUF AES Decryption Operation Successfull\n");
                  printf("Writing Decrypted data to file - aes_dec_data.txt\n");
                  aesdecdatafp = fopen("aes_dec_data.txt","w");
                  if (aesdecdatafp == NULL) {
                    printf("Unable to open file for saving data\n");
                    free(puf_aes_data.data);
                    free(puf_aes_data.enc_dec_mac_data);
                    return ONEBOX_STATUS_FAILURE;
                  } else {
                    fwrite(puf_aes_data.enc_dec_mac_data, sizeof(unsigned char), puf_aes_data.data_size, aesdecdatafp);
                    fclose(aesdecdatafp);
                  }
                } else if (puf_aes_data.puf_sub_cmd == PUF_AES_MAC) {
                  printf("PUF MAC Generation Operation Successfull\n");
                  printf(" -- Generated MAC for given data -- \n");
                  for (kc = 0; kc < 16; kc++) {
                    printf("%x\t", puf_aes_data.enc_dec_mac_data[kc]);
                  }
                  printf("\n");
                }
              }
              free(puf_aes_data.data);
              free(puf_aes_data.enc_dec_mac_data);
            } else {
              printf("Invalid Usage: Use: ./onebox_util <base_interface> puf_req <sub_cmd> <aes_mode> <key_source> <key_size> <key_input> <data_size> <data_file> <iv_size> <iv_input> \n");
              return ONEBOX_STATUS_FAILURE;
            }
            break;
          case PUF_BLOCK_ENROLL:      //! PUF Block Enroll/Set Key/Get Key operation, perform ioctl of required block opertion
          case PUF_BLOCK_SET_KEY:
          case PUF_BLOCK_GET_KEY:
            wrq.u.data.pointer = &puf_sub_cmd;
            wrq.u.data.length = sizeof(puf_sub_cmd);
            strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
            if (ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
            {
              printf("PUF Block Operation Failed\n");
              ret = ONEBOX_STATUS_FAILURE;
            }
            return ret;
          default: 
            printf("Invalid Usage: Use: ./onebox_util <base_interface> puf_req <sub_cmd> <...>\n");
            return ONEBOX_STATUS_FAILURE; 
        }
      } else {
        printf("Invalid Usage: Use: ./onebox_util <base_interface> puf_req <sub_cmd> <...>\n");
        return ONEBOX_STATUS_FAILURE;
      }
      break;
    }
		default:
		return ONEBOX_STATUS_FAILURE;
	}
	close(sfd);
	return ret;
}

int
getcmdnumber (char *command, char *ifName)
{
	if (!strcmp (command, "reset_per_q_stats") && !strncmp (ifName, "rpine",5))
	{
		return RSI_RESET_PER_Q_STATS;
	}
#ifdef IEEE80211K
  if (!strcmp(command, "set_msrmnt_req_params") && !strncmp (ifName, "rpine",5))
	{
		return RSI_SET_MSRMNT_PARAMS;
	}
#endif
	if (!strcmp (command, "setwmmparams") && strncmp (ifName, "rpine",5))
	{
		return RSI_WMM_PARAMS;
	}
	if (!strcmp (command, "create_vap") && !strncmp (ifName, "rpine",5))
	{
		return RSI_VAPCREATE;
	}
	if (!strcmp (command, "delete_vap") && !strncmp (ifName, "rpine",5))
	{
		return RSI_VAPDEL;
	}
	if (!strcmp (command, "print_vap_stats") && strncmp (ifName, "rpine",5))
	{
		return VAPSTATS;
	}
	if (!strcmp (command, "enable_protocol") && !strncmp (ifName, "rpine",5))
	{
		return RSI_ENABLE_PROTOCOL;
	}
	if (!strcmp (command, "disable_protocol") && !strncmp (ifName, "rpine",5))
	{
		return RSI_DISABLE_PROTOCOL;
	}
	if (!strcmp (command, "print_station_stats") && strncmp (ifName, "rpine",5))
	{
		return STATIONSTATS;
	}
	if (!strcmp (command, "aggr_limit") && !strncmp (ifName, "rpine",5))
	{
		return RSI_AGGR_LIMIT;
	}
	if (!strcmp (command, "print_station_info") && strncmp (ifName, "rpine",5))
	{
		return STAINFO;
	}
	if(!strcmp(command, "set_beacon_intvl") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_SET_BEACON;
	}
	if(!strcmp(command, "useonly_rates") && !strncmp(ifName, "rpine",5)) 
	{
		return RSI_USEONLY_RATES;
	}
	if(!strcmp(command, "cw_mode") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_SET_CW_MODE;
	}
	if(!strcmp(command, "bb_write") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_SET_BB_WRITE;
	}
	if(!strcmp(command, "endpoint") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_SET_ENDPOINT;
	}
	if(!strcmp(command, "ant_sel") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_ANT_SEL;
	}
	if(!strcmp(command, "ant_type") && !strncmp (ifName, "rpine",5))
	{
		return RSI_ANT_TYPE;
	}
	if(!strcmp(command, "get_txpwr") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_GET_TXPOWER;
	}
	if(!strcmp(command, "verify_flash") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_FLASH_VERIFY;
	}
	if(!strcmp(command, "eeprom_write") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_EEPROM_WRITE;
	}
	if(!strcmp(command, "eeprom_read") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_EEPROM_READ;
	}
	if(!strcmp(command, "bb_read") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_SET_BB_READ;
	}
	if(!strcmp(command, "set_bgscan_params") && !strncmp (ifName, "rpine",5)) 
	{
		printf("<<< set bgscan>>>\n");
		return RSI_SET_BGSCAN;
	}
	if(!strcmp(command, "host_scan_2g") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_HOST_SCAN_2G;
	}
	if(!strcmp(command, "host_scan") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_HOST_SCAN;
	}
	if(!strcmp(command, "host_scan_stop") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_SCAN_STOP;
	}
	if(!strcmp(command, "check_sta_state") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_STA_STATE;
	}
	if(!strcmp(command, "do_bgscan") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_DO_BGSCAN;
	}
	if(!strcmp(command, "bgscan_ssid") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_BGSCAN_SSID;
	}
	if(!strcmp(command, "set_ps_params") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_PS_REQUEST;
	}
	if(!strcmp(command, "set_uapsd_params") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_UAPSD_REQ;
	}
	if(!strcmp(command, "reset_adapter") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_RESET_ADAPTER;
	}
	if(!strcmp(command, "set_rx_filter") && !strncmp (ifName, "rpine",5))
	{
		return RSI_RX_FILTER;
	}
	if(!strcmp(command, "master_read") && !strncmp (ifName, "rpine",5))
	{
		return RSI_MASTER_READ;
	}
	if(!strcmp(command, "master_write") && !strncmp (ifName, "rpine",5))
	{
		return RSI_MASTER_WRITE;
	}
	if(!strcmp(command, "test_mode") && !strncmp (ifName, "rpine",5))
	{
		return RSI_TEST_MODE;
	}
	if(!strcmp(command, "set_rf_tx_rx_pwr_mode") && !strncmp (ifName, "rpine",5))
	{
		return RSI_TX_RX_RF_PWR_MODE;
	}
	if(!strcmp(command, "set_country") && !strncmp (ifName, "rpine",5))
	{
		return RSI_SET_COUNTRY;
	}
	if(!strcmp(command, "get_info") && !strncmp (ifName, "rpine",5))
	{
		return RSI_GET_INFO;
	}
	if(!strcmp(command, "set_scan_type") && !strncmp (ifName, "rpine",5))
	{
		return RSI_SET_SCAN_TYPE;
	}
	if(!strcmp(command, "rf_write") && !strncmp (ifName, "rpine",5))
	{
		return RSI_RF_WRITE;
	}
	if(!strcmp(command, "rf_read") && !strncmp (ifName, "rpine",5))
	{
		return RSI_RF_READ;
	}
	if(!strcmp(command, "protocol_rf_write") && !strncmp (ifName, "rpine",5))
	{
		return PROTOCOL_RF_WRITE;
	}
	if(!strcmp(command, "protocol_rf_read") && !strncmp (ifName, "rpine",5))
	{
		return PROTOCOL_RF_READ;
	}
	if(!strcmp(command, "ed_threshold") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_SET_ED_THRESHOLD;
	}
	if(!strcmp(command, "wowlan") && !strncmp (ifName, "rpine",5))
	{
		return RSI_WOWLAN_CONFIG;
	}
	if(!strcmp(command, "set_ext_ant_gain") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_SET_EXT_ANT_GAIN;
	}
	if(!strcmp(command, "beacon_recv_dis") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_BEACON_RECV_DIS;
	}
	if(!strcmp(command, "max_power") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_MAX_POWER;
	}
	if(!strcmp(command, "spectral_mask") && !strncmp (ifName, "rpine",5)) 
	{
		return RSI_SPECTRAL_MASK;
	}
	if(!strcmp(command, "w_9116_features") && !strncmp (ifName, "rpine",5)) 
	{
		return WLAN_9116_FEATURE ;
	}
	if(!strcmp(command, "prgm_stats") && !strncmp (ifName, "rpine",5)) 
	{
		return LOG_STRUCT_PRGMG ;
	}
	if(!strcmp(command, "disable_programming") && !strncmp (ifName, "rpine",5)) 
	{
		return DISABLE_PROGRAMMING ;
	}
	if(!strcmp(command, "prog_structure") && !strncmp (ifName, "rpine",5))
	{
		return PROG_STRUCTURE;
	}
	if(!strcmp(command, "ipmu_reg") && !strcmp (ifName, "rpine0"))
	{
		return IPMU_R_W;
	}
	if(!strcmp(command, "gpio_reg") && !strcmp (ifName, "rpine0"))
	{
		return GPIO_R_W;
	}
	if(!strcmp(command, "gtk_offload") && !strcmp (ifName, "rpine0"))
	{
		return RSI_GTK_OFFLOAD;
	}
	if(!strcmp(command, "puf_req") && !strcmp (ifName, "rpine0"))
	{
		return RSI_PUF_REQUEST;
	}	
	else
	{
		ONEBOX_PRINT ("Error: Wrong command , Please follow usage...\n");
		usage();
		return ONEBOX_STATUS_FAILURE;
	}
}

/** This function is used to destroy the VAP
 * @param  socket descriptor
 * @param  interface name
 * @return void
 */
void
destroy_vap (int sock, char *ifname)
{
	struct iwreq ifr;
	//ONEBOX_PRINT( "Going to delete[%x] VAP %s\n",RSIIOCVAPDEL,ifname);
	strncpy (ifr.ifr_name, ifname, IFNAMSIZ);
	if (ioctl (sock, RSIIOCVAPDEL, &ifr) < 0)
	{
		ONEBOX_PRINT ("Error In deleting VAP\n");
	}
	else
	{
		ONEBOX_PRINT ("VAP Deleted\n");
	}
	return;
}

/** This function prints the vap statistics which are useful while debugging
 * @param  Pointer to the ieee80211_stats structure
 * @param  Verbose - used for printing the detailed information related to stats
 * @return void 
 */
void
printvapstats (struct ieee80211_stats *stat, unsigned char verbose)
{
	ONEBOX_PRINT ("Received the statsistics from device....");
	ONEBOX_PRINT ("\nrx_badversion       : %010d", stat->is_rx_badversion);
	ONEBOX_PRINT_INFO (verbose, " (rx frame with badversion)");
	ONEBOX_PRINT ("\nrx_tooshort         : %010d", stat->is_rx_tooshort);
	ONEBOX_PRINT_INFO (verbose, " (rx frame too short)");
	ONEBOX_PRINT ("\nrx_wrongbss         : %010d", stat->is_rx_wrongbss);
	ONEBOX_PRINT_INFO (verbose, " (rx from wrong bss)");
	ONEBOX_PRINT ("\nrx_dup              : %010d", stat->is_rx_dup);
	ONEBOX_PRINT_INFO (verbose, " (rx discard 'cuz dup)");
	ONEBOX_PRINT ("\nrx_wrongdir         : %010d", stat->is_rx_wrongdir);
	ONEBOX_PRINT_INFO (verbose, " (rx w/wrong direction)");
	ONEBOX_PRINT ("\nrx_mcastecho        : %010d", stat->is_rx_mcastecho);
	ONEBOX_PRINT_INFO (verbose, " (rx discard 'cuz mcast echo)");
	ONEBOX_PRINT ("\nrx_notassoc         : %010d", stat->is_rx_notassoc);
	ONEBOX_PRINT_INFO (verbose, " (rx failed sta not associated)");

	ONEBOX_PRINT ("\nrx_noprivacy        : %010d", stat->is_rx_noprivacy);
	ONEBOX_PRINT_INFO (verbose, " (rx w/ wep but privacy off)");

	ONEBOX_PRINT ("\nrx_unencrypted      : %010d", stat->is_rx_unencrypted);
	ONEBOX_PRINT_INFO (verbose, " (rx w/o wep but privacy on)");
	ONEBOX_PRINT ("\nrx_wepfail          : %010d", stat->is_rx_wepfail);
	ONEBOX_PRINT_INFO (verbose, " (rx wep processing failed)");
	ONEBOX_PRINT ("\nrx_mgtdiscard       : %010d", stat->is_rx_mgtdiscard);
	ONEBOX_PRINT_INFO (verbose, " (rx discard mgt frames)");
	ONEBOX_PRINT ("\nrx_beacon           : %010d", stat->is_rx_beacon);
	ONEBOX_PRINT_INFO (verbose, " (rx beacon frames)");
	ONEBOX_PRINT ("\nrx_rstoobig         : %010d", stat->is_rx_rstoobig);
	ONEBOX_PRINT_INFO (verbose, " (irx rate set truncated)");
	ONEBOX_PRINT ("\nrx_missing          : %010d", stat->is_rx_elem_missing);
	ONEBOX_PRINT_INFO (verbose, " (rx required element missing)");
	ONEBOX_PRINT ("\nrx_elem_toobig      : %010d", stat->is_rx_elem_toobig);
	ONEBOX_PRINT_INFO (verbose, " (rx required element too big)");
	ONEBOX_PRINT ("\nrx_elem_toosmall    : %010d", stat->is_rx_elem_toosmall);
	ONEBOX_PRINT_INFO (verbose, " (rx required element too small)");
	ONEBOX_PRINT ("\nrx_elem_unknown     : %010d", stat->is_rx_elem_unknown);
	ONEBOX_PRINT_INFO (verbose, " (rx required element unknown)");
	ONEBOX_PRINT ("\nrx_badchan          : %010d", stat->is_rx_badchan);
	ONEBOX_PRINT_INFO (verbose, " (rx frame w/ invalid chan)");
	ONEBOX_PRINT ("\nrx_chanmismatch     : %010d", stat->is_rx_chanmismatch);
	ONEBOX_PRINT_INFO (verbose, " (rx frame chanmiosmatch)");
	ONEBOX_PRINT ("\nrx_nodealloc        : %010d", stat->is_rx_nodealloc);
	ONEBOX_PRINT_INFO (verbose, " (rx frame dropped)");
	ONEBOX_PRINT ("\nrx_ssidmismatch     : %010d", stat->is_rx_ssidmismatch);
	ONEBOX_PRINT_INFO (verbose, " (rx frame mismatched)");
	ONEBOX_PRINT ("\nrx_auth_unsupp'd    : %010d",stat->is_rx_auth_unsupported);
	ONEBOX_PRINT_INFO (verbose, " (rx w/unsupported auth alg)");
	ONEBOX_PRINT ("\nrx_auth_fail        : %010d", stat->is_rx_auth_fail);
	ONEBOX_PRINT_INFO (verbose, " (rx station authentication failure)");
	ONEBOX_PRINT ("\nrx_auth_countermeasures : %010d",stat->is_rx_auth_countermeasures);
	ONEBOX_PRINT_INFO (verbose, " (rx auth discard 'cuz CM)");
	ONEBOX_PRINT ("\nrx_assoc_bss        : %010d", stat->is_rx_assoc_bss);
	ONEBOX_PRINT_INFO (verbose, " (rx assoc from wrong bssid)");
	ONEBOX_PRINT ("\nrx_assoc_notauth    : %010d", stat->is_rx_assoc_notauth);
	ONEBOX_PRINT_INFO (verbose, " (rx assoc w/0 auth)");
	ONEBOX_PRINT ("\nrx_assoc_capmismatch: %010d", stat->is_rx_assoc_capmismatch);
	ONEBOX_PRINT_INFO (verbose, " (rx assoc w/ cap mismatch)");
	ONEBOX_PRINT ("\nrx_assoc_norate     : %010d", stat->is_rx_assoc_norate);
	ONEBOX_PRINT_INFO (verbose, " (rx assoc w/ no rate match)");
	ONEBOX_PRINT ("\nrx_assoc_badwpa     : %010d", stat->is_rx_assoc_badwpaie);
	ONEBOX_PRINT_INFO (verbose, " (rx assoc w/ bad  WPA IE)");
	ONEBOX_PRINT ("\nrx_deauth           : %010d", stat->is_rx_deauth);
	ONEBOX_PRINT_INFO (verbose, " (rx deauthentication)");
	ONEBOX_PRINT ("\nrx_disassoc         : %010d", stat->is_rx_disassoc);
	ONEBOX_PRINT_INFO (verbose, " (rx deassociation)");
	ONEBOX_PRINT ("\nrx_badsubtype       : %010d", stat->is_rx_badsubtype);
	ONEBOX_PRINT_INFO (verbose, " (rx frame w/unknown subtype)");
	ONEBOX_PRINT ("\nrx_nobuf            : %010d", stat->is_rx_nobuf);
	ONEBOX_PRINT_INFO (verbose, " (rx failed for lack of buf)");
	ONEBOX_PRINT ("\nrx_decryptcrc       : %010d", stat->is_rx_decryptcrc);
	ONEBOX_PRINT_INFO (verbose, " (rx decrystat failed on crc)");
	ONEBOX_PRINT ("\nrx_ahdemo_mg        : %010d", stat->is_rx_ahdemo_mgt);
	ONEBOX_PRINT_INFO (verbose, " (rx discard ahdemomgt frame)");
	ONEBOX_PRINT ("\nrx_bad_auth         : %010d", stat->is_rx_bad_auth);
	ONEBOX_PRINT_INFO (verbose, " (rx bad auth request)");
	ONEBOX_PRINT ("\nrx_unauth           : %010d", stat->is_rx_unauth);
	ONEBOX_PRINT_INFO (verbose, " (rx on unauthorized port)");
	ONEBOX_PRINT ("\nrx_badkeyid         : %010d", stat->is_rx_badkeyid);
	ONEBOX_PRINT_INFO (verbose, " (rx w/ incorrect keyid)");
	ONEBOX_PRINT ("\nrx_ccmpreplay       : %010d", stat->is_rx_ccmpreplay);
	ONEBOX_PRINT_INFO (verbose, " (rx seq# violation error(CCMP))");
	ONEBOX_PRINT ("\nrx_ccmpformat       : %010d", stat->is_rx_ccmpformat);
	ONEBOX_PRINT_INFO (verbose, " (rx format bad  (CCMP))");
	ONEBOX_PRINT ("\nrx_ccmpmic          : %010d", stat->is_rx_ccmpmic);
	ONEBOX_PRINT_INFO (verbose, " (rx MIC check faile(CCMP))");
	ONEBOX_PRINT ("\nrx_tkipreplay       : %010d", stat->is_rx_tkipreplay);
	ONEBOX_PRINT_INFO (verbose, " (rx seq# violation error(TKIP))");
	ONEBOX_PRINT ("\nrx_tkipformat       : %010d", stat->is_rx_tkipformat);
	ONEBOX_PRINT_INFO (verbose, " (rx format bad  (TKIP))");
	ONEBOX_PRINT ("\nrx_tkipmic          : %010d", stat->is_rx_tkipmic);
	ONEBOX_PRINT_INFO (verbose, " (rx MIC check faile(TKIP))");
	ONEBOX_PRINT ("\nrx_tkipicv          : %010d", stat->is_rx_tkipicv);
	ONEBOX_PRINT_INFO (verbose, " (rx ICV check faile(TKIP))");
	ONEBOX_PRINT ("\nrx_badcipher        : %010d", stat->is_rx_badcipher);
	ONEBOX_PRINT_INFO (verbose, " (rx failed 'cuz key type)");
	ONEBOX_PRINT ("\nrx_nocipherctx      : %010d", stat->is_rx_nocipherctx);
	ONEBOX_PRINT_INFO (verbose, " (rx failed 'cuz key !setup)");
	ONEBOX_PRINT ("\nrx_acl              : %010d", stat->is_rx_acl);
	ONEBOX_PRINT_INFO (verbose, " (rx discard 'cuz acl policy)");
	ONEBOX_PRINT ("\nrx_nobuf            : %010d", stat->is_tx_nobuf);
	ONEBOX_PRINT_INFO (verbose, " (Txfailed for lock of buf)");
	ONEBOX_PRINT ("\nrx_nonode           : %010d", stat->is_tx_nonode);
	ONEBOX_PRINT_INFO (verbose, " (tx fail;ed for no node)");
	ONEBOX_PRINT ("\nrx_unknownmgt       : %010d", stat->is_tx_unknownmgt);
	ONEBOX_PRINT_INFO (verbose, " (Tx of unknowm mgt frame)");
	ONEBOX_PRINT ("\ntx_badcipher        : %010d", stat->is_tx_badcipher);
	ONEBOX_PRINT_INFO (verbose, " (Tx failed 'cuz key type)");
	ONEBOX_PRINT ("\ntx_nodefkey         : %010d", stat->is_tx_nodefkey);
	ONEBOX_PRINT_INFO (verbose, " (Tx failed 'cuz no def key)");
	ONEBOX_PRINT ("\ntx_noheadroom       : %010d", stat->is_tx_noheadroom);
	ONEBOX_PRINT_INFO (verbose, " (Tx failed 'cuz no space)");
	ONEBOX_PRINT ("\ntx_fragframes       : %010d", stat->is_tx_fragframes);
	ONEBOX_PRINT_INFO (verbose, " (Tx frames fragmented)");
	ONEBOX_PRINT ("\ntx_frags            : %010d", stat->is_tx_frags);
	ONEBOX_PRINT_INFO (verbose, " (Tx frames created)");
	ONEBOX_PRINT ("\ntx_ctl              : %010d", stat->is_tx_ctl);
	ONEBOX_PRINT_INFO (verbose, " (tx ctrl frames)");
	ONEBOX_PRINT ("\nscan_active         : %010d", stat->is_scan_active);
	ONEBOX_PRINT_INFO (verbose, " (active scans atarted)");
	ONEBOX_PRINT ("\nscan_passive        : %010d", stat->is_scan_passive);
	ONEBOX_PRINT_INFO (verbose, " (passive scans atarted)");
	ONEBOX_PRINT ("\nnode_timeout        : %010d", stat->is_node_timeout);
	ONEBOX_PRINT_INFO (verbose, " (nodes timeout inactivity)");
	ONEBOX_PRINT ("\ncrypto_nomem        : %010d", stat->is_crypto_nomem);
	ONEBOX_PRINT_INFO (verbose, " (no memory for crystato ctx)");
	ONEBOX_PRINT ("\ncrypto_tkip         : %010d", stat->is_crypto_tkip);
	ONEBOX_PRINT_INFO (verbose, " (TKIP crystato done in s/w)");
	ONEBOX_PRINT ("\ncrypto_tkipenmic    : %010d", stat->is_crypto_tkipenmic);
	ONEBOX_PRINT_INFO (verbose, " (TKIP en-MIC done in s/w)");
	ONEBOX_PRINT ("\ncrypto_tkipdemic    : %010d", stat->is_crypto_tkipdemic);
	ONEBOX_PRINT_INFO (verbose, " (TKIP de-MIC done in s/w)");
	ONEBOX_PRINT ("\ncrypto_tkipcm       : %010d", stat->is_crypto_tkipcm);
	ONEBOX_PRINT_INFO (verbose, " (TKIP icounter measures)");
	ONEBOX_PRINT ("\ncrypto_ccmp         : %010d", stat->is_crypto_ccmp);
	ONEBOX_PRINT_INFO (verbose, " (CCMPP crystato done in s/w)");
	ONEBOX_PRINT ("\ncrypto_wep          : %010d", stat->is_crypto_wep);
	ONEBOX_PRINT_INFO (verbose, " (wep crystato done in s/w)");
	ONEBOX_PRINT ("\ncrypto_setkey_cipher: %010d", stat->is_crypto_setkey_cipher);
	ONEBOX_PRINT_INFO (verbose, " (cipher rejected key)");
	ONEBOX_PRINT ("\ncrypto_setkey_nokey : %010d", stat->is_crypto_setkey_nokey);
	ONEBOX_PRINT_INFO (verbose, " (no key index for setkey)");
	ONEBOX_PRINT ("\ntx_nonode           : %010d", stat->is_tx_nonode);
	ONEBOX_PRINT_INFO (verbose, " (tx failed for no node)");
	ONEBOX_PRINT ("\ncrypto_delkey       : %010d", stat->is_crypto_delkey);
	ONEBOX_PRINT_INFO (verbose, " (driver key delete failed)");
	ONEBOX_PRINT ("\ncrypto_badcipher    : %010d", stat->is_crypto_badcipher);
	ONEBOX_PRINT_INFO (verbose, " (unknown cipher)");
	ONEBOX_PRINT ("\ncrypto_nocipher     : %010d", stat->is_crypto_nocipher);
	ONEBOX_PRINT_INFO (verbose, " (cipher not available)");
	ONEBOX_PRINT ("\ncrypto_attachfail   : %010d", stat->is_crypto_attachfail);
	ONEBOX_PRINT_INFO (verbose, " (cipher attach failed)");
	ONEBOX_PRINT ("\ncrypto_swfallback   : %010d", stat->is_crypto_swfallback);
	ONEBOX_PRINT_INFO (verbose, " (cipher fallback to s/w)");
	ONEBOX_PRINT ("\ncryptokeyfail       : %010d", stat->is_crypto_keyfail);
	ONEBOX_PRINT_INFO (verbose, " (driver key alloc failed) ");
	ONEBOX_PRINT ("\ncrypto_enmicfail    : %010d", stat->is_crypto_enmicfail);
	ONEBOX_PRINT_INFO (verbose, " (en-MIC failed)");
	ONEBOX_PRINT ("\nibss_capmismatch    : %010d", stat->is_ibss_capmismatch);
	ONEBOX_PRINT_INFO (verbose, " (merge failed-cap mismatch)");
	ONEBOX_PRINT ("\nibss_norate         : %010d", stat->is_ibss_norate);
	ONEBOX_PRINT_INFO (verbose, " (merge failed-rate mismatch)");
	ONEBOX_PRINT ("\nps_unassoc          : %010d", stat->is_ps_unassoc);
	ONEBOX_PRINT_INFO (verbose, " (ps-poll for unassoc. sta)");
	ONEBOX_PRINT ("\nps_badaid           : %010d", stat->is_ps_badaid);
	ONEBOX_PRINT_INFO (verbose, " (ps-poll w/ incorrect aid)");
	ONEBOX_PRINT ("\nps_qempty           : %010d", stat->is_ps_qempty);
	ONEBOX_PRINT_INFO (verbose, " (ps-poll w/ nothing to send)");
	ONEBOX_PRINT ("\nff_badhdr           : %010d", stat->is_ff_badhdr);
	ONEBOX_PRINT_INFO (verbose, " (fast frame rx'd w/ bad hdr)");
	ONEBOX_PRINT ("\nff_tooshort         : %010d", stat->is_ff_tooshort);
	ONEBOX_PRINT_INFO (verbose, " (fast frame rx decap error)");
	ONEBOX_PRINT ("\nff_split            : %010d", stat->is_ff_split);
	ONEBOX_PRINT_INFO (verbose, " (fast frame rx split error)");
	ONEBOX_PRINT ("\nff_decap            : %010d", stat->is_ff_decap);
	ONEBOX_PRINT_INFO (verbose, " (fast frames decap'd)");
	ONEBOX_PRINT ("\nff_encap            : %010d", stat->is_ff_encap);
	ONEBOX_PRINT_INFO (verbose, " (fast frames encap'd for tx)");
	ONEBOX_PRINT ("\nrx_badbintval       : %010d", stat->is_rx_badbintval);
	ONEBOX_PRINT_INFO (verbose, " (rx frame w/ bogus bintval");
	ONEBOX_PRINT ("\ndemicfail           : %010d", stat->is_rx_demicfail);
	ONEBOX_PRINT_INFO (verbose, " (rx demic failed)");
	ONEBOX_PRINT ("\nrx_defrag           : %010d", stat->is_rx_defrag);
	ONEBOX_PRINT_INFO (verbose, " (rx defragmentation failed)");
	ONEBOX_PRINT ("\nrx_mgmt             : %010d", stat->is_rx_mgmt);
	ONEBOX_PRINT_INFO (verbose, " (rx management frames)");
	ONEBOX_PRINT ("\nrx_action           : %010d", stat->is_rx_action);
	ONEBOX_PRINT_INFO (verbose, " (rx action mgt frames)");
	ONEBOX_PRINT ("\namsdu_tooshort      : %010d", stat->is_amsdu_tooshort);
	ONEBOX_PRINT_INFO (verbose, " (A-MSDU rx decap error)");
	ONEBOX_PRINT ("\namsdu_split         : %010d", stat->is_amsdu_split);
	ONEBOX_PRINT_INFO (verbose, " (A-MSDU rx split error)");
	ONEBOX_PRINT ("\namsdu_decap         : %010d", stat->is_amsdu_decap);
	ONEBOX_PRINT_INFO (verbose, " (A-MSDU decap'd)");
	ONEBOX_PRINT ("\namsdu_encap         : %010d", stat->is_amsdu_encap);
	ONEBOX_PRINT_INFO (verbose, " (A-MSDU encap'd for tx )");
	ONEBOX_PRINT ("\nampdu_bar_oow       : %010d", stat->is_ampdu_bar_oow);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU BAR before ADDBA)");
	ONEBOX_PRINT ("\nampdu_bar_move      : %010d", stat->is_ampdu_bar_move);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU BAR moved window)");
	ONEBOX_PRINT ("\nampdu_bar_rx        : %010d", stat->is_ampdu_bar_rx);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU BAR frames handled)");
	ONEBOX_PRINT ("\nampdu_rx_flush      : %010d", stat->is_ampdu_rx_flush);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU frames flushed)");
	ONEBOX_PRINT ("\nampdu_rx_oor        : %010d", stat->is_ampdu_rx_oor);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU frames out-of-order)");
	ONEBOX_PRINT ("\nampdu_rx_copy       : %010d", stat->is_ampdu_rx_copy);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU frames copied down)");
	ONEBOX_PRINT ("\nAMPDU frames        : %010d", stat->is_ampdu_rx_drop);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU frames dropped)");
	ONEBOX_PRINT ("\ntx_badstate         : %010d", stat->is_tx_badstate);
	ONEBOX_PRINT_INFO (verbose, " (tx discard state != RUN)");
	ONEBOX_PRINT ("\ntx_notassoc         : %010d", stat->is_tx_notassoc);
	ONEBOX_PRINT_INFO (verbose, " (tx failed, sta not assoc)");
	ONEBOX_PRINT ("\ntx_classify         : %010d", stat->is_tx_classify);
	ONEBOX_PRINT_INFO (verbose, " (tx classification failed)");
	ONEBOX_PRINT ("\ndwds_mcast          : %010d", stat->is_dwds_mcast);
	ONEBOX_PRINT_INFO (verbose, " (discard mcast over dwds)");
	ONEBOX_PRINT ("\ndwds_qdrop          : %010d", stat->is_dwds_qdrop);
	ONEBOX_PRINT_INFO (verbose, " (dwds pending frame q full)");
	ONEBOX_PRINT ("\nht_assoc_nohtcap    : %010d", stat->is_ht_assoc_nohtcap);
	ONEBOX_PRINT_INFO (verbose, " (non-HT sta rejected)");
	ONEBOX_PRINT ("\nht_assocdowngrade   : %010d", stat->is_ht_assoc_downgrade);
	ONEBOX_PRINT_INFO (verbose, " (HT sta forced to legacy)");
	ONEBOX_PRINT ("\nht_assoc_norate     : %010d", stat->is_ht_assoc_norate);
	ONEBOX_PRINT_INFO (verbose, " (HT assoc w/ rate mismatch )");
	ONEBOX_PRINT ("\nampdu_rx_age        : %010d", stat->is_ampdu_rx_age);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU sent up 'cuz of age)");
	ONEBOX_PRINT ("\nampdu_rx_move       : %010d", stat->is_ampdu_rx_move);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU MSDU moved window)");
	ONEBOX_PRINT ("\naddba_reject        : %010d", stat->is_addba_reject);
	ONEBOX_PRINT_INFO (verbose, " (ADDBA reject 'cuz disabled)");
	ONEBOX_PRINT ("\naddba_norequest     : %010d", stat->is_addba_norequest);
	ONEBOX_PRINT_INFO (verbose, " (ADDBA response w/o ADDBA)");
	ONEBOX_PRINT ("\naddba_badtoken      : %010d", stat->is_addba_badtoken);
	ONEBOX_PRINT_INFO (verbose, " (ADDBA response w/ wrong)");
	ONEBOX_PRINT ("\naddba_badpolicy     : %010d", stat->is_addba_badpolicy);
	ONEBOX_PRINT_INFO (verbose, " (ADDBA resp w/ wrong policy)");
	ONEBOX_PRINT ("\nampdu_stop          : %010d", stat->is_ampdu_stop);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU stream stopped)");
	ONEBOX_PRINT ("\nampdu_stop_failed   : %010d", stat->is_ampdu_stop_failed);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU stream not running)");
	ONEBOX_PRINT ("\nampdu_rx_reorder    : %010d", stat->is_ampdu_rx_reorder);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU held for rx reorder)");
	ONEBOX_PRINT ("\nscan_bg             : %010d", stat->is_scan_bg);
	ONEBOX_PRINT_INFO (verbose, " (background scans started)");
	ONEBOX_PRINT ("\ndeauth_code         : %010d", stat->is_rx_deauth_code);
	ONEBOX_PRINT_INFO (verbose, " (last rx'd deauth reason)");
	ONEBOX_PRINT ("\ndisassoc_code       : %010d", stat->is_rx_disassoc_code);
	ONEBOX_PRINT_INFO (verbose, " (last rx'd disassoc reason)");
	ONEBOX_PRINT ("\nauthfail_code       : %010d", stat->is_rx_authfail_code);
	ONEBOX_PRINT_INFO (verbose, " (last rx'd auth fail reason)");
	ONEBOX_PRINT ("\nbeacon_ misss       : %010d", stat->is_beacon_miss);
	ONEBOX_PRINT_INFO (verbose, " (beacon miss notification)");
	ONEBOX_PRINT ("\nrx_badstate         : %010d", stat->is_rx_badstate);
	ONEBOX_PRINT_INFO (verbose, " (rx discard state != RUN)");
	ONEBOX_PRINT ("\nff_flush            : %010d", stat->is_ff_flush);
	ONEBOX_PRINT_INFO (verbose, " (ff's flush'd from stageq)");
	ONEBOX_PRINT ("\ntx_ctl              : %010d", stat->is_tx_ctl);
	ONEBOX_PRINT_INFO (verbose, " (tx ctrl frames)");
	ONEBOX_PRINT ("\nampdu_rexmt         : %010d", stat->is_ampdu_rexmt);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU frames rexmt ok)");
	ONEBOX_PRINT ("\nampdu_rexmt_fail    : %010d", stat->is_ampdu_rexmt_fail);
	ONEBOX_PRINT_INFO (verbose, " (A-MPDU frames rexmt fail)");
	ONEBOX_PRINT ("\nmesh_wrongmesh      : %010d", stat->is_mesh_wrongmesh);
	ONEBOX_PRINT_INFO (verbose, " (dropped 'cuz not mesh sta)");
	ONEBOX_PRINT ("\nmesh_nolink         : %010d", stat->is_mesh_nolink);
	ONEBOX_PRINT_INFO (verbose, " (dropped 'cuz link not estab*)");
	ONEBOX_PRINT ("\nmesh_fwd_ttl        : %010d", stat->is_mesh_fwd_ttl);
	ONEBOX_PRINT_INFO (verbose, " (mesh not fwd'd 'cuz ttl 0)");
	ONEBOX_PRINT ("\nmesh_fwd_nobuf      : %010d", stat->is_mesh_fwd_nobuf);
	ONEBOX_PRINT_INFO (verbose, " (mesh not fwd'd 'cuz no mbuf)");
	ONEBOX_PRINT ("\nmesh_fwd_tooshort   : %010d", stat->is_mesh_fwd_tooshort);
	ONEBOX_PRINT_INFO (verbose, " (mesh not fwd'd 'cuz no hdr)");
	ONEBOX_PRINT ("\nmesh_fwd_disabled   : %010d", stat->is_mesh_fwd_disabled);
	ONEBOX_PRINT_INFO (verbose, " (mesh not fwd'd 'cuz disabled)");
	ONEBOX_PRINT ("\nmesh_fwd_nopath     : %010d", stat->is_mesh_fwd_nopath);
	ONEBOX_PRINT_INFO (verbose, " (mesh not fwd'd 'cuz path unknown)");
	ONEBOX_PRINT ("\nhwmp_wrongseq       : %010d", stat->is_hwmp_wrongseq);
	ONEBOX_PRINT_INFO (verbose, " (wrong hwmp seq no. )");
	ONEBOX_PRINT ("\nhwmp_rootreqs       : %010d", stat->is_hwmp_rootreqs);
	ONEBOX_PRINT_INFO (verbose, " (root PREQs sent )");
	ONEBOX_PRINT ("\nhwmp_rootrann       : %010d", stat->is_hwmp_rootrann);
	ONEBOX_PRINT_INFO (verbose, " (root RANNs sent)");
	ONEBOX_PRINT ("\nmesh_badae          : %010d", stat->is_mesh_badae);
	ONEBOX_PRINT_INFO (verbose, " (dropped 'cuz invalid AE )");
	ONEBOX_PRINT ("\nmesh_rtaddfailed    : %010d", stat->is_mesh_rtaddfailed);
	ONEBOX_PRINT_INFO (verbose, " (route add failed)");
	ONEBOX_PRINT ("\nmesh_notproxy       : %010d", stat->is_mesh_notproxy);
	ONEBOX_PRINT_INFO (verbose, " (dropped 'cuz not proxying)");
	ONEBOX_PRINT ("\nbadalign            : %010d", stat->is_rx_badalign);
	ONEBOX_PRINT_INFO (verbose, " (dropped 'cuz misaligned)");
	ONEBOX_PRINT ("\nhwmp_proxy          : %010d", stat->is_hwmp_proxy);
	ONEBOX_PRINT_INFO (verbose, " (PREP for proxy route)");
	ONEBOX_PRINT ("\n");
}


/** This function prints the station statistics for the particular station selected by user
 * These stats are useful while debugging
 * @param  Pointer to the ieee80211_nodestats structure
 * @param  Verbose - used for printing the detailed information related to stats
 * @return void 
 */
void
printstationstats (struct ieee80211_nodestats *ni_stats,
                   unsigned char verbose)
{
	ONEBOX_PRINT ("\nns_rx_data          : %010d", ni_stats->ns_rx_data);
	ONEBOX_PRINT_INFO (verbose, " (rx data frames)");
	ONEBOX_PRINT ("\nns_rx_mgmt          : %010d", ni_stats->ns_rx_mgmt);
	ONEBOX_PRINT_INFO (verbose, " (rxmanagement frames)");
	ONEBOX_PRINT ("\nns_rx_ctrl          : %010d", ni_stats->ns_rx_ctrl);
	ONEBOX_PRINT_INFO (verbose, " (rx control frames)");
	ONEBOX_PRINT ("\nns_rx_ucast         : %010d", ni_stats->ns_rx_ucast);
	ONEBOX_PRINT_INFO (verbose, " (rx unicast frames)");
	ONEBOX_PRINT ("\nns_rx_mcast         : %010d", ni_stats->ns_rx_mcast);
	ONEBOX_PRINT_INFO (verbose, " (rx multi/broadcast frames)");
	ONEBOX_PRINT ("\nns_rx_bytes         : %010ld", ni_stats->ns_rx_bytes);
	ONEBOX_PRINT_INFO (verbose, " (rx data count (bytes))");
	ONEBOX_PRINT ("\nns_rx_beacon        : %010ld", ni_stats->ns_rx_beacons);
	ONEBOX_PRINT_INFO (verbose, " (rx beacon frames)");
	ONEBOX_PRINT ("\nns_rx_proberesp     : %010d", ni_stats->ns_rx_proberesp);
	ONEBOX_PRINT_INFO (verbose, " (rx probe response frames)");
	ONEBOX_PRINT ("\nns_rx_dup           : %010d", ni_stats->ns_rx_dup);
	ONEBOX_PRINT_INFO (verbose, " (rx discard 'cuz dup)");
	ONEBOX_PRINT ("\nns_rx_noprivacy     : %010d", ni_stats->ns_rx_noprivacy);
	ONEBOX_PRINT_INFO (verbose, " (rx w/ wep but privacy off)");
	ONEBOX_PRINT ("\nns_rx_wepfail       : %010d", ni_stats->ns_rx_wepfail);
	ONEBOX_PRINT_INFO (verbose, " (rx wep processing failed)");
	ONEBOX_PRINT ("\nns_rx_demicfail     : %010d", ni_stats->ns_rx_demicfail);
	ONEBOX_PRINT_INFO (verbose, " (rx demic failed)");
	ONEBOX_PRINT ("\nns_rx_decap         : %010d", ni_stats->ns_rx_decap);
	ONEBOX_PRINT_INFO (verbose, " (rx decapsulation failed)");
	ONEBOX_PRINT ("\nns_rx_defrag        : %010d", ni_stats->ns_rx_defrag);
	ONEBOX_PRINT_INFO (verbose, " (rx defragmentation failed)");
	ONEBOX_PRINT ("\nns_rx_disassoc      : %010d", ni_stats->ns_rx_disassoc);
	ONEBOX_PRINT_INFO (verbose, " (rx disassociation)");
	ONEBOX_PRINT ("\nns_rx_deauth        : %010d", ni_stats->ns_rx_deauth);
	ONEBOX_PRINT_INFO (verbose, " (rx deauthentication)");
	ONEBOX_PRINT ("\nns_rx_action        : %010d", ni_stats->ns_rx_action);
	ONEBOX_PRINT_INFO (verbose, " (rx action)");
	ONEBOX_PRINT ("\nns_rx_decryptcrc    : %010d", ni_stats->ns_rx_decryptcrc);
	ONEBOX_PRINT_INFO (verbose, " (rx decrypt failed on crc)");
	ONEBOX_PRINT ("\nns_rx_unauth        : %010d", ni_stats->ns_rx_unauth);
	ONEBOX_PRINT_INFO (verbose, " (rx on unauthorized port)");
	ONEBOX_PRINT ("\nns_rx_unencrypted   : %010d", ni_stats->ns_rx_unencrypted);
	ONEBOX_PRINT_INFO (verbose, " (rx unecrypted w/ privacy)");
	ONEBOX_PRINT ("\nns_rx_drop          : %010d", ni_stats->ns_rx_drop);
	ONEBOX_PRINT_INFO (verbose, " (rx discard other reason)");

	ONEBOX_PRINT ("\nns_tx_data          : %010d", ni_stats->ns_tx_data);
	ONEBOX_PRINT_INFO (verbose, " (tx data frames)");
	ONEBOX_PRINT ("\nns_tx_mgmt          : %010d", ni_stats->ns_tx_mgmt);
	ONEBOX_PRINT_INFO (verbose, " (tx management frames)");
	ONEBOX_PRINT ("\nns_tx_ctrl          : %010d", ni_stats->ns_tx_ctrl);
	ONEBOX_PRINT_INFO (verbose, " (tx control frames)");
	ONEBOX_PRINT ("\nns_tx_ucast         : %010d", ni_stats->ns_tx_ucast);
	ONEBOX_PRINT_INFO (verbose, " (tx unicast frames)");
	ONEBOX_PRINT ("\nns_tx_mcast         : %010d", ni_stats->ns_tx_mcast);
	ONEBOX_PRINT_INFO (verbose, " (tx multi/broadcast frames)");
	ONEBOX_PRINT ("\nns_tx_bytes         : %010ld", ni_stats->ns_tx_bytes);
	ONEBOX_PRINT_INFO (verbose, " (tx data count (bytes))");
	ONEBOX_PRINT ("\nns_tx_probereq      : %010d", ni_stats->ns_tx_probereq);
	ONEBOX_PRINT_INFO (verbose, " (tx probe request frames)");

	ONEBOX_PRINT ("\nns_tx_novlanta      : %010d", ni_stats->ns_tx_novlantag);
	ONEBOX_PRINT_INFO (verbose, " (tx discard 'cuz no tag)");
	ONEBOX_PRINT ("\nns_tx_vlanmismatch  : %010d", ni_stats->ns_tx_vlanmismatch);
	ONEBOX_PRINT_INFO (verbose, " (tx discard 'cuz bad tag)");

	ONEBOX_PRINT ("\nns_ps_discard       : %010d", ni_stats->ns_ps_discard);
	ONEBOX_PRINT_INFO (verbose, " (ps discard 'cuz of age)");

	/////////////* MIB-related state */

	ONEBOX_PRINT ("\nns_tx_assoc         : %010d", ni_stats->ns_tx_assoc);
	ONEBOX_PRINT_INFO (verbose, " ([re]associations)");
	ONEBOX_PRINT ("\nns_tx_assoc_fail    : %010d", ni_stats->ns_tx_assoc_fail);
	ONEBOX_PRINT_INFO (verbose, " ([re]association failures)");
	ONEBOX_PRINT ("\nns_tx_auth          : %010d", ni_stats->ns_tx_auth);
	ONEBOX_PRINT_INFO (verbose, " ([re]authentications)");
	ONEBOX_PRINT ("\nns_tx_auth_fail     : %010d", ni_stats->ns_tx_auth_fail);
	ONEBOX_PRINT_INFO (verbose, " ([re]authentication failures)");
	ONEBOX_PRINT ("\nns_tx_deauth        : %010d", ni_stats->ns_tx_deauth);
	ONEBOX_PRINT_INFO (verbose, " (deauthentications)");
	ONEBOX_PRINT ("\nns_tx_deauth_code   : %010d", ni_stats->ns_tx_deauth_code);
	ONEBOX_PRINT_INFO (verbose, " (last deauth reason)");
	ONEBOX_PRINT ("\nns_tx_disassoc      : %010d", ni_stats->ns_tx_disassoc);
	ONEBOX_PRINT_INFO (verbose, " (disassociation)");
	ONEBOX_PRINT ("\nns_tx_disassoc_code : %010d", ni_stats->ns_tx_disassoc_code);
	ONEBOX_PRINT_INFO (verbose, " (disassociation reason code)");
	ONEBOX_PRINT ("\n");
}

/** This function prints the station information for the list of connected stations.
 * @param  Pointer to the ieee80211_nodestats structure
 * @param  Verbose - used for printing the detailed information related to stats
 * @return void 
 */
void
printstationinfo (struct ieee80211req_sta_info *sta_info,
                  unsigned char verbose)
{
	int i;
	ONEBOX_PRINT ("The station info for ");
	ONEBOX_PRINT ("\nmacaddr   =  %02x:%02x:%02x:%02x:%02x:%02x",
	              sta_info->isi_macaddr[0], sta_info->isi_macaddr[1],
	              sta_info->isi_macaddr[2], sta_info->isi_macaddr[3],
	              sta_info->isi_macaddr[4], sta_info->isi_macaddr[5]);
	ONEBOX_PRINT ("\nisi_len         : %010d", sta_info->isi_len);
	ONEBOX_PRINT_INFO (verbose, "(total length (mult of 4))");
	ONEBOX_PRINT ("\nisi_off         : %010d", sta_info->isi_ie_off);
	ONEBOX_PRINT_INFO (verbose, "(offset to IE data)");
	ONEBOX_PRINT ("\nisi_ie_len      : %010d", sta_info->isi_ie_len);
	ONEBOX_PRINT_INFO (verbose, " (IE length)");
	ONEBOX_PRINT ("\nisi_jointime    : %d", sta_info->isi_jointime);
	ONEBOX_PRINT ("\nisi_freq        : %010d", sta_info->isi_freq); /* MHz */
	ONEBOX_PRINT ("\nisi_flags       : %010x", sta_info->isi_flags);
	ONEBOX_PRINT_INFO (verbose, " (channel flags)");
	ONEBOX_PRINT ("\nisi_state       : %010x", sta_info->isi_state);
	ONEBOX_PRINT_INFO (verbose, " (state flags)");
	ONEBOX_PRINT ("\nisi_authmode    : %010d", sta_info->isi_authmode);
	ONEBOX_PRINT_INFO (verbose, " (authentication algorithm)");
	ONEBOX_PRINT ("\nisi_rssi        : %d", sta_info->isi_rssi);
	ONEBOX_PRINT_INFO (verbose, " (receive signal strength)");
	ONEBOX_PRINT ("\nisi_noise       : %010d", sta_info->isi_noise);
	ONEBOX_PRINT_INFO (verbose, " (noise floor)");
	ONEBOX_PRINT ("\nisi_capinfo     : %010d", sta_info->isi_capinfo);
	ONEBOX_PRINT_INFO (verbose, " (capabilities)");
	ONEBOX_PRINT ("\nisi_erp         : %010d", sta_info->isi_erp);
	ONEBOX_PRINT_INFO (verbose, " (ERP element)");
	ONEBOX_PRINT ("\nisi_nrates      : %010d", sta_info->isi_nrates);
	ONEBOX_PRINT_INFO (verbose, " (negotiated rates)");
	for (i = 0; i < IEEE80211_RATE_MAXSIZE; i++)
	{
		ONEBOX_PRINT ("\nisi_rates[%02d]   : %010d", i, sta_info->isi_rates[i]);
	}
	ONEBOX_PRINT ("\nisi_txrate      : %d", sta_info->isi_txrate);
	ONEBOX_PRINT_INFO (verbose, " (legacy/IEEE rate or MCS)");
	ONEBOX_PRINT ("\nisi_associd     : %010x", sta_info->isi_associd);
	ONEBOX_PRINT_INFO (verbose, " (assoc response)");
	ONEBOX_PRINT ("\nisi_txpower     : %010d", sta_info->isi_txpower);
	ONEBOX_PRINT_INFO (verbose, " (current tx power)");
	ONEBOX_PRINT ("\nisi_vlan        : %010d", sta_info->isi_vlan);
	ONEBOX_PRINT_INFO (verbose, "(vlan tag)");

	ONEBOX_PRINT("\nNB: [IEEE80211_NONQOS_TID] holds seq#'s for non-QoS stations ");

	for (i = 0; i < IEEE80211_TID_SIZE; i++)
	{
		ONEBOX_PRINT ("\nisi_txseqs[%02d]  : %010d", i,
		                       sta_info->isi_txseqs[i]);
		ONEBOX_PRINT_INFO (verbose, "(tx seq #/TID)");
	}
	for (i = 0; i < IEEE80211_TID_SIZE; i++)
	{
		ONEBOX_PRINT ("\nisi_rxseqs[%02d]  : %010x", i,
		                       sta_info->isi_rxseqs[i]);
		ONEBOX_PRINT_INFO (verbose, "(rx seq #/TID)");
	}
	ONEBOX_PRINT ("\nisi_inact       : %010d", sta_info->isi_inact);
	ONEBOX_PRINT_INFO (verbose, "(inactivity timer)");
	ONEBOX_PRINT ("\nisi_txmbps      : %010d", sta_info->isi_txmbps);
	ONEBOX_PRINT_INFO (verbose, "(current tx rate in .5 Mb/s)");
	ONEBOX_PRINT ("\nisi_pad         : %010d", sta_info->isi_pad);
	ONEBOX_PRINT_INFO (verbose, "(time of assoc/join)");
	ONEBOX_PRINT ("\n***************MIMO info for 11n sta's*****************");
	for (i = 0; i < IEEE80211_MAX_CHAINS; i++)
	{
		ONEBOX_PRINT ("\nmimo.rssi[%02d]   : %010x", i,
		                    sta_info->isi_mimo.rssi[i]);
		ONEBOX_PRINT_INFO (verbose, "(per-antenna rssi)");
	}
	for (i = 0; i < IEEE80211_MAX_CHAINS; i++)
	{
		ONEBOX_PRINT ("\nmimo.noise[%02d]  : %010x", i,
		                   sta_info->isi_mimo.noise[i]);
		ONEBOX_PRINT_INFO (verbose, "(per-antenna noise floor)");
	}
	ONEBOX_PRINT ("\nmimo.pad[0]     : %010d", sta_info->isi_mimo.pad[0]);
	ONEBOX_PRINT ("\nmimo.pad[1]     : %010d", sta_info->isi_mimo.pad[1]);
	ONEBOX_PRINT ("\nmimo.evm[0]     : %010d", sta_info->isi_mimo.evm[0]);
	ONEBOX_PRINT_INFO (verbose, "(EVM data)");
	ONEBOX_PRINT ("\nmimo.evm[1]     : %010d", sta_info->isi_mimo.evm[1]);
	ONEBOX_PRINT_INFO (verbose, "(EVM data)");
	ONEBOX_PRINT ("\nmimo.evm[2]     : %010d", sta_info->isi_mimo.evm[2]);
	ONEBOX_PRINT_INFO (verbose, "(EVM data)");
	ONEBOX_PRINT ("\n**********************11s info *************************");
	ONEBOX_PRINT ("\nisi_peerid      : %010d", sta_info->isi_peerid);
	ONEBOX_PRINT ("\nisi_localid     : %010d", sta_info->isi_localid);
	ONEBOX_PRINT ("\nisi_peerstate   : %010d", sta_info->isi_peerstate);
	ONEBOX_PRINT ("\n");

}

/** This function converts the mac address from  Ascii to Hex. 
 * @param  Pointer to the mac address entered by the user
 * @param  location where the mac address is stored after conversion to HEX
 * @return void 
 */
void byteconversion (char *src, char *macaddr)
{
	int ii;
	unsigned char temp[18];

	for (ii = 0; *src != '\0'; ii++)
	{
		if (*src == ':')
		{
			src++;
		}
		if ((*src >= '0') && (*src <= '9'))
		{
			*src -= '0';
			temp[ii] = *src;
		}
		else if ((*src >= 'a') && (*src <= 'f'))
		{
			*src -= 'a';
			*src += 0xa;
			temp[ii] = *src;
		}
		else if ((*src >= 'A') && (*src <= 'F'))
		{
			*src -= 'A';
			*src += 0xa;
			temp[ii] = *src;
		}
		else
		{
			ONEBOX_PRINT ("Invalid macaddr\n");
			exit (1);
		}
		src++;
	}
	for (ii = 0; ii < 12;)
	{
		*macaddr = (temp[ii] << 4) | (temp[ii + 1]);
		ii = ii + 2;
		macaddr++;
	}
}
/** This function gets the Zigbee state. 
 * @param  pointer to the Zigbee interface mac address of Redpine module 
 */
int
get_zigb_state ( )
{
	char status;
	FILE *fp;
	int lastchar=0;
	char str_out[100],str_zigb[100];
	int fs;
	int ii;
	char str[70];
	char command[100],cmd_tmp[100];

	status = snprintf(cmd_tmp, 100, "%s %s", "ifconfig -a",
			"| grep -i zigb | cut -c1-4");

	fp = popen
		(cmd_tmp,
		 "r");
	lastchar = fread (str_zigb, 1, 100, fp);
	str_zigb[lastchar] = '\0';
	pclose (fp);
	if(strncasecmp(str_zigb,"zigb",4))
	{
		return -1;	
	}

	status = snprintf(command, 100, "%s %s", "ifconfig -a",
			"| grep -i 00:23:A7 | sed -e 's/^[ \t]*//' | cut -f2 -d ' ' | cut -c1-8");

	fp = popen
		(command,
		 "r");
	lastchar = fread (str_out, 1, 100, fp);
	str_out[lastchar] = '\0';
	pclose (fp);
	if(!strncasecmp(str_out,"00:23:A7",8)) 
        { 
 		return 0;	
	}

	
}


/** This function gets the Bluetooth state. 
 * @param  pointer to the bluetooth interface mac address of Redpine module 
 */
int  get_bt_state ( )
{
	char status;
	FILE *fp;
	int lastchar;
	char str_out[100];
	int fs;
	int ii;
	char str[70];
	char command[150];

	status = snprintf(command, 150, "%s %s", "hciconfig -a",
			"| grep -i 00:23:A7 | cut -f3 -d ' ' | cut -c1-8");
	fp = popen
		(command,
		 "r");
	lastchar = fread (str_out, 1, 100, fp);
	str_out[lastchar] = '\0';
	pclose (fp);
	if(!strncasecmp(str_out,"00:23:A7",8)) 
        { 
 		return 0;	
	}
	else
		return -1;
	
}

/** This function gets the driver state. 
 * @param  pointer to the driver state string 
 * @return void
 */
void
get_driver_state (char *ptr, char *ifname)
{
	char status;
	FILE *fp;
	int lastchar;
	char str_out[100];
	int fs;
	int ii;
	char str[70];
	char command[150];

#ifdef RSI_CONFIG_ANDROID
	status = snprintf (command, 150, "%s%s%s ", "cat /proc/", ifname,
			"/stats | grep -i FSM_ | busybox cut -f2 -d ' ' | busybox cut -f1 -d '('");
	fp = popen
		(command,
		 "r");
#else
	status = snprintf (command, 150, "%s%s%s ", "cat /proc/", ifname,
			"/stats | grep -i FSM_ | cut -f2 -d ' ' | cut -f1 -d '('");
	fp = popen
		(command,
		 "r");
#endif
	lastchar = fread (str_out, 1, 100, fp);
	str_out[lastchar] = '\0';
	pclose (fp);
	

	for (ii = 0; str_out[ii] != '\0'; ii++)
	{
		*ptr = str_out[ii];
		ptr++;
	}
}

/** This function checks vap name given to delete. 
 * @param  pointer to vap name string 
 * @return void
 */
int
check_vap (const char *ptr)
{
	char status;
	FILE *fp;
	char str_out[20];
	char command[50];

	sprintf(command,"ifconfig %s | grep -i 00:23:A7 | wc -l", ptr);
	fp = popen(command,"r");
	fread (str_out, 1, 1, fp);
	pclose (fp);
	if((atoi(str_out) == 0) || !strncmp(ptr, "rpine0", 6))
		return 0;
	else
		return 1;
}
/** This function gives the information about the operating mode being used
 * in the driver
 * @param  string to be mapped for extracting the operating mode
 * @ return operating mode selected.
 */
int
getopmode (const char *s)
{
	int opmode;
	if ((strncasecmp (s, "STA", 3) == 0))
	{
		opmode = RSI_M_STA;
		ONEBOX_PRINT ("Creating VAP in station mode\n");
	}
	else if ((strncasecmp (s, "P2P_GO", 6) == 0))
	{
		opmode = RSI_M_P2P_GO;
		ONEBOX_PRINT ("Creating VAP in P2P GO mode\n");
	}
	else if ((strncasecmp (s, "P2P", 3) == 0))
	{
		opmode = RSI_M_P2P;
		ONEBOX_PRINT ("Creating VAP in P2P mode\n");
	}
	else if ((strncasecmp (s, "MON", 3) == 0))
	{
		opmode = RSI_M_MONITOR;
		ONEBOX_PRINT ("Creating VAP in monitor mode\n");
	}
	else if ((strncasecmp (s, "AP", 2) == 0))
	{
		ONEBOX_PRINT ("Creating VAP in AP mode\n");
		opmode = RSI_M_HOSTAP;
	}
	else
	{
		// NB: AP mode is the default 
		ONEBOX_PRINT("unknown operating mode, AP mode is selected by default %s", s);
		opmode = RSI_M_HOSTAP;
	}
	return opmode;
}

/** This function verifies the flashed values with the burned values
 * @param  void
 * @return void 
 */
int verify_flash( char *ifName, int sfd, int len)
{
	struct iwreq wrq;
	int max_len = 256;
	int file_size;
	int count = 0, kk = 0;
	eepromrw eeprom;
	char temp[100],*temp_buf;
	char *start;
	FILE *pFile, *pFile1;
	unsigned char *buf_file;
	int ret_val = 0;

	pFile = fopen("../release/RS9113_RS8111_calib_values.txt", "rw+");
	if(pFile == NULL)
	{
		printf("Unable to create a file\n");
		return ONEBOX_STATUS_FAILURE;
	}
	printf("<<<< RSI EEPROM VERIFY >>>>>\n");

	fseek(pFile, 0L, SEEK_END);
	file_size = ftell(pFile);
	if (!file_size)
		return ONEBOX_STATUS_FAILURE;

	buf_file = malloc (file_size/6);
	if (buf_file == NULL)
		return ONEBOX_STATUS_FAILURE;

	temp_buf = buf_file;
	fseek(pFile, 0L, SEEK_SET);

	for (kk = 0; kk < file_size ; kk+=6)
	{  
		fscanf(pFile,"%s",temp);
		start = temp;
		if((start[0] == '0') && (start[1] == 'x'))
		{
			start+=2;
			strncpy(temp, start, 2);
		}
		sscanf(temp,"%x",buf_file);
		start++;
		buf_file++;
	}

	file_size = file_size/6;
	buf_file = temp_buf;
	pFile1 = fopen("../release/log_burn.txt", "w");
	if (pFile1 == NULL)
		goto end1;

	strncpy(wrq.ifr_name, ifName, IFNAMSIZ);
	eeprom.length = len;
	eeprom.offset = 0;
	ONEBOX_PRINT("File SIZE burnt to flash %d \n",file_size);
	kk = 0;

	while (len > 0)
	{  
		wrq.u.data.flags = EEPROM_READ_IOCTL;
		if (len >= 256)
			eeprom.length = max_len;
		else
			eeprom.length = len;
		wrq.u.data.pointer = &eeprom;
		wrq.u.data.length = eeprom.length;
		if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
		{
			ONEBOX_PRINT(" Failed Error while reading Flash value\n");
		}
		else
		{
			for ( kk = 0; kk < max_len; count++, kk++ )
			{
				if (pFile1) /* reqd? already checking above */
					fprintf(pFile1, "0x%x,\n", eeprom.data[kk]);
				if (count >= file_size)
				{  
					ONEBOX_PRINT("!!!!!! File comparion done SUCCESSFULLY !!!!!!\n");
					goto end;
				}  
				if (eeprom.data[kk] != buf_file[count])
				{
					printf(" EEPROM value[%d] is: 0x%x buf_file[%d] :0x%x\n",kk, eeprom.data[kk],count,buf_file[count]);
					ONEBOX_PRINT("!!!!!! Failed !!!!!!!!!!!!\n");
					ONEBOX_PRINT("!!!!!! Flash values do not MATCH!!!!!!!!!!!!\n");
					ret_val = -1;
					goto end;
				}
			}
		}
		len = len - max_len;
		eeprom.offset += max_len;
		memset (&eeprom.data[0], 0, sizeof (eeprom.data));
	}

end:
	fclose(pFile1);
	pFile1 = NULL;
end1:
	fclose(pFile);
	pFile = NULL;
	free(buf_file);
	temp_buf = NULL; /* dangling pointer */
	return ret_val;
}

/** This function gives the usage of the onebox utility
 * @param  void
 * @return void 
 */
void usage ()
{
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface create_vap vap_name AP/ap \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface create_vap vap_name STA/sta sw_bmiss/hw_bmiss(not case sensitive) \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface create_vap vap_name MON/mon \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface create_vap vap_name P2P/p2p \n");
	ONEBOX_PRINT( "Usage: onebox_util base_interface set_beacon_intvl value \n");
	ONEBOX_PRINT( "Usage: onebox_util base_interface cw_mode channel mode sub_type \n");
	ONEBOX_PRINT( "Usage: onebox_util base_interface endpoint type \n");
	ONEBOX_PRINT ("Usage: onebox_util base_interface delete_vap vap_name \n");
	ONEBOX_PRINT ("Usage: onbeox_util base_interface aggr_limit tx_limit rx_limit \n");
	ONEBOX_PRINT ("Usage: onebox_util base_interface master_read $32bit_address $no_bytes_to_read )\n");
	ONEBOX_PRINT ("Usage: onebox_util base_interface master_write $32bit_address $no_bytes_to_write $data)\n");
	ONEBOX_PRINT ("Usage: onebox_util base_interface test_mode subtype arguments\n");
	ONEBOX_PRINT ("Usage: onebox_util base_interface beacon_recv_dis <1(disable) / 0(enable)>\n");
	ONEBOX_PRINT ("Usage: ./onebox_util <base_interface> spectral_mask <mask_value>\n");
	ONEBOX_PRINT ("Usage: onebox_util vap_name print_vap_stats\n");
	ONEBOX_PRINT ("Usage: onebox_util vap_name print_vap_stats [-v] \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util vap_name print_vap_stats [-f] filename \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util vap_name print_vap_stats [-v] [-f] filename\n");
	ONEBOX_PRINT ("Usage: onebox_util vap_name print_station_stats macaddr\n");
	ONEBOX_PRINT
	        ("Usage: onebox_util vap_name print_station_stats macaddr [-v] \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util vap_name print_station_stats macaddr [-f] filename \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util vap_name print_station_stats macaddr [-v] [-f] filename\n");
	ONEBOX_PRINT ("Usage: onebox_util vap_name print_station_info macaddr \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util vap_name print_station_info macaddr [-v] \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util vap_name print_station_info macaddr [-f] filename \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util vap_name print_station_info macaddr [-v] [-f] filename\n");
	ONEBOX_PRINT
	        ("Usage: onebox_util vap_name setwmmparams  wmm_param_name value access_category self/broadcast update_params\n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface reset_per_q_stats qnum \n");
	ONEBOX_PRINT
		("Usage: onebox_util base_interface set_bgscan_params bgscan_threshold rssi_tolerance_threshold periodicity active_scan_duration passive_scan_durations two_probe_en(0/1) num_bg_channels  channels_to_scan \n" );
	ONEBOX_PRINT
		("Usage: onebox_util base_interface do_bgscan\n");
	ONEBOX_PRINT
		("Usage: onebox_util base_interface bgscan_ssid $ssid_name)\n");
	ONEBOX_PRINT
		("Usage: onebox_util base_interface set_ps_params $ps_en/ps_dis(0/1) $sleep_type(1/2) $tx_threshold $rx_threshold $tx_hysterisis $rx_hysterisis $monitor_interval $listen_interval $sleep_duration)\n");
	ONEBOX_PRINT
		("Usage: onebox_util base_interface set_uapsd_params $UAPSD_ACS(0xF) $sp_len $uapsd_wakeup_period )\n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface eeprom read/write length offset  \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface reset_adapter\n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface bb_read addr  \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface bb_write addr data \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface ant_sel  value \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface set_rx_filter rx_filter_word \n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface set_rf_tx_rx_pwr_mode tx_value rx_value\n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface set_country <Country_Code>\n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface get_info country\n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface get_info country_code\n");
	ONEBOX_PRINT
		("Usage: onebox_util base_interface set_scan_type $value\n");
	ONEBOX_PRINT
		("Usage: onebox_util base_interface get_txpwr\n");

	ONEBOX_PRINT
		("Usage: onebox_util base_interface wowlan macaddr host_wakeup(0-Disable/1-Enable) WOWLAN-Flags(Bit0 -ALLOW All Packets \n BIT1 - Allow only Unicast\n BIT2 - Allow only Unicast with the macaddress specified \nBIT3 - Allow EAPOL Packets\n  ) \n");
	ONEBOX_PRINT
		("Usage: onebox_util base_interface set_ext_ant_gain $2.4GHz_ant_gain $5GHz_ant_gain\n");
	ONEBOX_PRINT
		("Usage: onebox_util base_interface enable_protocol $value(1- WLAN, 2-BT, 4-Zigbee, 3-WLAN+BT, 5-WLAN+Zigbee)\n");
	ONEBOX_PRINT
		("Usage: onebox_util base_interface disable_protocol $value(1- WLAN, 2-BT, 4-Zigbee, 3-WLAN+BT, 5-WLAN+Zigbee)\n");
	ONEBOX_PRINT
	        ("Usage: onebox_util base_interface ant_type $ant_path $ant_type_value \n");
  ONEBOX_PRINT
          ("Usage: onebox_util base_interface w_9116_features pll_mode rf_type wireless_mode enable_ppe afe_type dpd SIFSTransmitenable pwrsave_options\n");
  ONEBOX_PRINT
          ("Usage: onebox_util rpine0 prgm_stats enable_disable interval\n");
  ONEBOX_PRINT
          ("Usage: onebox_util base_interface rf_write  addr  data   \n");
  ONEBOX_PRINT
          ("Usage: onebox_util base_interface rf_read  addr  \n");
  ONEBOX_PRINT
          ("Usage: onebox_util base_interface protocol_rf_write  addr  data   \n");
  ONEBOX_PRINT
          ("Usage: onebox_util base_interface protocol_rf_read  addr \n");
  ONEBOX_PRINT
          ("Usage: onebox_util rpine0  disable_programming value\n");
  ONEBOX_PRINT 
          ("Usage ./onebox_util rpine0  prog_structure <prog_type> <TA_RAM_ADDRESS> \n");
  ONEBOX_PRINT 
          ("Usage ./onebox_util rpine0 ipmu_reg <mode> <address> <value>\n");
  ONEBOX_PRINT 
          ("Usage ./onebox_util rpine0 gtk_offload <value>\n");
  ONEBOX_PRINT
          ("Usage: ./onebox_util rpine0 puf_req $puf_sub_cmd(0 - PUF_Enroll, 1 - PUF_Start, 2 - Set_Key, 3 - Set_Intrinsic_Key, 4 - Get_Key, 5 - Load_Key, 6 - AES_Encrypt, 7 - AES_Decrypt, 8 - AES_MAC, 9 - Disable_Enroll, 10 - Disable_Set_Key, 11 - Disable_Get_key) <variable params>\n");
	return ;
}
