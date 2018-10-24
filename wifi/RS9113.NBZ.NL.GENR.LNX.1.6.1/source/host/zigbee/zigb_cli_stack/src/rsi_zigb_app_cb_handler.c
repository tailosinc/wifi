/**
 *  @file     rsi_zigb_app_cb_handler.c
 *  @version  1.0
 *  @date     2014-Aug-23
 *
 *  Copyright(C) Redpine Signals 2014
 *  All rights reserved by Redpine Signals.
 *
 *  @section License
 *  This program should be used on your own responsibility.
 *  Redpine Signals assumes no responsibility for any losses
 *  incurred by customers or third parties arising from the use of this file.
 *
 *  @brief API: Definitions of various data and interface handlers
 *
 *  @section Description
 *  This file contain definitions for different ZigBee INTERFACE callbacks. 
 *  These are various data and mgmt handler functions
 *
 *  @section Improvements
 *
 */


/**
 * Includes
 * */


#include "rsi_common_types.h"
#include "rsi_zigb_api.h"
#include "rsi_zigb_interfaces.h"
#include "rsi_zigb_app.h"
#include "rsi_zigb_callbacks.h"
#include "rsi_frame.h"
#include "semaphore.h"
#include <time.h>
#include <stdarg.h>
#include <stdio.h>
#ifdef COEX_APP_CHANGES
#include "rsi_zigb_onoff.h"
#endif
#ifdef LINUX_PLATFORM
#include "stdio.h"

#endif
void log_and_print_info(const char *fmt, ...);
extern char f_name; 
extern FILE *fLogPtr;

typedef struct Queue_Tag
{
	/*! Holds the head of the queue */
	uint8_t write_index;
	/*! Holds the tail of the queue */
	uint8_t read_index;
	/*! Holds the total capacity of the current queue */
	uint8_t max_used;
	/*! Holds the current number of entries in the queue */
	uint8_t current_capacity;
	uint8_t que_list[24]; 
	char name[12];
} __attribute__((__packed__))queue_t;

typedef struct Confirmation_Details_Tag
{
	uint16_t dest_address;
	uint8_t dest_addr_mode;
	uint8_t dest_endpoint;
	uint8_t src_endpoint;
	uint8_t aps_counter;
} __attribute__((__packed__))Confirmation_Details_t;

typedef struct NSDU_Handle_Table_Tag
{
	Confirmation_Details_t conf_details;
	uint8_t nsdu_handle;
	uint8_t action_on_nlde_conf;
	uint8_t buffer_id;
} __attribute__((__packed__))NSDU_Handle_Table_t;

typedef struct Msdu_Handle_Table_Tag
{
	uint8_t msdu_handle;
	uint8_t module;
	uint8_t NBT_index;
	uint16_t nwk_dest_addr;
	uint16_t nwk_src_addr;
} __attribute__((__packed__))Msdu_Handle_Table_t;


#ifdef COEX_APP_CHANGES
lightDeviceInfo_t lightDevInfo_coex;
#endif
ZigBeeNWKStatusInfo nwkStatusInfo;
extern rsi_zigb_app_info_t rsi_zigb_app_info;
extern rsi_zigb_app_cb_t rsi_zigb_app_cb;
extern uint16_t matchDescReqSent;
extern void rsi_zb_app_clear_event(uint32_t event_num);
extern int rsi_zb_app_set_event(uint32_t event_num);
extern void measure_throughput(uint32_t  pkt_length, uint32_t tx_rx);
extern void signal_call(int signum);
extern uint8_t g_endpoint;
extern uint16_t g_short_addr;
extern sem_t  sem_udp_send;
///////////////Call back Handlers called from Stack. These are to be filled by App developer

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_scan_complete_handler (uint32_t channel, 
 *                                                       uint8_t status )
 * @brief       Scan complete handler 
 * @param[in]   Channel
 * @param[in]   Status of channel whether beacons are found
 * @return      none
 * @section description
 * This API is used to handle ZigBee scan complete state
 * It provides infromation of the channel whether beacons are found or not
 * Updating few app_info variables 
 *
 * ===========================================================================*/

void rsi_zigb_app_scan_complete_handler ( uint32_t channel, uint8_t status )
{
	rsi_zigb_app_info.scan_done_cb.channel = channel; 
	rsi_zigb_app_info.scan_done_cb.scan_status = status; 
	rsi_zigb_app_info.status_var.scanReqSent = 0;
	log_and_print_info("Scan Completed [Channel %d: Status: %d]\n",channel,status);
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_energy_scan_result_handler( uint32_t channel,
 *                                                       uint8_t *pEnergyValue)
 * @brief       Energy Scan complete handler 
 * @param[in]   Channel
 * @param[in]   Energy Value (RSSI)
 * @return      none
 * @section description
 * This API is used to handle ZigBee Energy scan complete state
 * Here Energy in each channel is received, for the provided channels 
 * issued by user to scan
 *
 * ===========================================================================*/
void rsi_zigb_app_energy_scan_result_handler( uint32_t channel,uint8_t *pEnergyValue)
{
	int i=0;
	char temp[2000];
	char temp1[200];
	sprintf(temp,"Application Energy Scan Result[Channel %d]\n",channel);
	for (i=0;i<16;i++)
	{
		sprintf(temp1,"Energy Value[%d]: %d\n",i,pEnergyValue[i]);
		strcat(temp,temp1);
	}
	log_and_print_info(temp);
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_network_found_handler(ZigBeeNetworkDetails)
 * @brief       Network found indication handler 
 * @param[in]   NetworkInformation data 
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle network found indication frame  
 * Infromation about the found network is updated 
 *
 * ===========================================================================*/

void rsi_zigb_app_network_found_handler(ZigBeeNetworkDetails networkInformation)
{
	ZigBeeNetworkDetails *nwk_details = &(rsi_zigb_app_info.nwkinfo);
	uint8_t i;
	char temp[1000];
	char temp1[300];

	rsi_zigb_app_cb.num_nwk_found++;
	/* Currently we are checking for any coordinator, if you know the specific 
	 * extended panid, then check here for specific panid */
	rsi_zigb_mcpy((uint8_t *)&networkInformation, (uint8_t *)nwk_details, sizeof(ZigBeeNetworkDetails));
	sprintf(temp,"\tExtended Pan: ");
	for (i=0;i<8;i++)
	{
		sprintf(temp1,"0x%X ",networkInformation.extendedPanId[i]);
		strcat(temp,temp1);
	}
	log_and_print_info("Network Found:-\n\tchannel: %d\n\tshort panid: %X\n%s\n",networkInformation.channel,networkInformation.shortPanId,temp);
	
	
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_stack_status_handler(ZigBeeNWKStatusInfo *statusInfo)
 * @brief       Stack status Indication
 * @param[in]   Network status Information 
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle network/stack status
 * Infromation about network status (If connection successful of failed) 
 *
 * ===========================================================================*/

char *stack_status[]={   "ZigBee NWK Is Up",
	"ZigBee NWK Is Down", 
	"ZigBee Join Failed", 
	"ZigBee Cannot Join As Router", 
	"ZigBee Changed Node ID", 
	"ZigBee Changed PANId", 
	"ZigBee Changed Channel", 
	"ZigBee No Beacons", 
	"ZigBee Received KeyIn Clear", 
	"ZigBee NoNWKKey Received", 
	"ZigBee NoLinkKey Received", 
	"ZigBee PreconfiguredKey Required", 
	"ZigBee ChangedManager Address"};



void rsi_zigb_app_stack_status_handler(ZigBeeNWKStatusInfo *statusInfo)
{
	rsi_zigb_app_info.stack_status = *statusInfo;
	log_and_print_info("%s\n", stack_status[rsi_zigb_app_info.stack_status]);
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_incoming_many_to_one_route_req_handler(uint16_t SourceAddr,
 *                                                  uint8_t * pSrcIEEEAddr,uint8_t PathCost )
 * @brief       Many to one route request handler
 * @param[in]   Source short Addr
 * @param[in]   Source IEEE address
 * @param[in]   Path cost
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle Many to one route request
 * We have to decide which route to accept based on path cost 
 *
 * ===========================================================================*/

void rsi_zigb_app_incoming_many_to_one_route_req_handler( uint16_t SourceAddr, uint8_t * pSrcIEEEAddr,uint8_t PathCost )
{
	char temp[2000];
	char temp1[200],i;
	sprintf(temp,"Received Many to one route req\n");
    sprintf(temp1,"\tSorceAddr: 0x%X\n",SourceAddr);
	strcat(temp,temp1);
	sprintf(temp1,"\tPathCost: %X\n",PathCost);
	strcat(temp,temp1);
    sprintf(temp1,"\tSource IEEE Address:  ");
	strcat(temp,temp1);
    for (i=0;i<8;i++)
	{
		sprintf(temp1,"0x%X ",pSrcIEEEAddr[i]);
		strcat(temp,temp1);
	}
    log_and_print_info("%s\n",temp);

}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_handle_data_indication(
 *                                   APSDE_Data_Indication_t * pDataIndication )
 * @brief       Handle data indication frame
 * @param[in]   Data indication info struct 
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle received data indication frame
 *
 * ===========================================================================*/

void rsi_zigb_app_handle_data_indication(APSDE_Data_Indication_t *pDataIndication)
{
#ifdef COEX_APP_CHANGES
	uint16_t* lightAddress;
#endif
	rsi_zigb_app_info_t *app_info = &rsi_zigb_app_info;

	if ( pDataIndication->cluster_id == 0x8003)//0x8003: power descriptor response
	{
		if (pDataIndication->a_asdu[1] == 0x00)
		{
			rsi_zigb_mcpy( pDataIndication->a_asdu,
						   app_info->zb_resp_info.powerDescResp, 
						   pDataIndication->asdulength);
			app_info->status_var.powerDescRspStatus = 0x00;
		}
	}

	if ( pDataIndication->cluster_id == 0x8002)//0x8003: node descriptor response
	{
		if (pDataIndication->a_asdu[1] == 0x00)
		{
			rsi_zigb_mcpy( pDataIndication->a_asdu,
						   app_info->zb_resp_info.nodeDescResp, 
						   pDataIndication->asdulength);
			app_info->status_var.nodeDescRspStatus = 0x00;
		}
	}

	if ( pDataIndication->cluster_id == 0x8021)//0x8021: bind response
	{
		if (pDataIndication->a_asdu[1] == 0x84)
		{
			rsi_zigb_mcpy( pDataIndication->a_asdu,
						   app_info->zb_resp_info.bindResp, 
						   pDataIndication->asdulength);
			app_info->status_var.bindRspStatus = 0x00;
		}
	}

	if ( pDataIndication->cluster_id == 0x8022)//0x8022: unbind response
	{
		if (pDataIndication->a_asdu[1] == 0x84)
		{
			rsi_zigb_mcpy( pDataIndication->a_asdu,
						   app_info->zb_resp_info.unbindResp, 
						   pDataIndication->asdulength);
			app_info->status_var.unbindRspStatus = 0x00;
		}
	}

	if ( pDataIndication->cluster_id == 0x8005)//0x8005: active endpoint response
	{
		if (pDataIndication->a_asdu[1] == 0x00)
		{
			rsi_zigb_mcpy( pDataIndication->a_asdu,
						   app_info->zb_resp_info.actepResp, 
						   pDataIndication->asdulength);
			app_info->status_var.actepRspStatus = 0x00;
		}
	}

	if ( pDataIndication->cluster_id == 0x8001)//0x8001: ieee addr response
	{
		if (pDataIndication->a_asdu[1] == 0x00)
		{
			rsi_zigb_mcpy( pDataIndication->a_asdu,
						   app_info->zb_resp_info.ieeeaddrResp, 
						   pDataIndication->asdulength);
			app_info->status_var.ieeeaddrRspStatus = 0x00;
		}
	}

	if ( pDataIndication->cluster_id == 0x8004)//0x8004: simple desc response
	{
		if (pDataIndication->a_asdu[1] == 0x00)
		{
			rsi_zigb_mcpy( pDataIndication->a_asdu,
						   app_info->zb_resp_info.simpledespResp, 
						   pDataIndication->asdulength);
			app_info->status_var.simpledescRspStatus = 0x00;
		}
	}

	if ( pDataIndication->cluster_id == 0x8000)//0x8000: network addr response
	{
		if (pDataIndication->a_asdu[1] == 0x00)
		{
			rsi_zigb_mcpy( pDataIndication->a_asdu,
						   app_info->zb_resp_info.networkaddrResp, 
						   pDataIndication->asdulength);
			app_info->status_var.networkaddrRspStatus = 0x00;
		}
	}

	if ( pDataIndication->cluster_id == 0x8006)//0x8006: Match decs response
	{
		if (pDataIndication->a_asdu[1] == 0x00)
		{
#ifdef ZB_DEBUG 
			rsi_os_printf(RSI_PL1,"\n Match Desc Response rcvd \n");
#endif  
			rsi_zigb_mcpy( pDataIndication->a_asdu, 
						   app_info->zb_resp_info.matchDescResp,
						   pDataIndication->asdulength);
			app_info->status_var.matchDescRspStatus = 0x00;
#ifdef COEX_APP_CHANGES
#ifndef ZB_API_TEST
			lightAddress =  (uint16_t *)&app_info->zb_resp_info.matchDescResp[2];
			lightDevInfo_coex.shortaddress = *lightAddress;    
			lightDevInfo_coex.endpoint = app_info->zb_resp_info.matchDescResp[5];
			rsi_zigb_delay(1);
			rsi_zigb_app_send_data( g_Client_To_Server_c, g_Cluster_Specific_Cmd_c, lightDevInfo_coex.endpoint, lightDevInfo_coex.shortaddress , g_TOGGLE_c, 0x0006, 0x00, 0x0);
#endif
#endif
		}
	}
	measure_throughput(100,1);
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_handle_data_confirmation (
 *                                   APSDE_Data_Confirmation_t* pDataConfirmation )
 * @brief       Handle data confirmation frame
 * @param[in]   Buffer Index of actual data from the pointer
 * @param[in]   Data confirmation info struct 
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle received data confirmation frame for the 
 * data request sent
 *
 * ===========================================================================*/
uint32_t dataConfcnt_success = 0;
uint32_t dataConfcnt_Failed = 0;
uint32_t data_send_faile_buffer_full_or_error = 0;

extern uint32_t tx_packet_counter;

extern sem_t  sem_cmd_rsp;
void rsi_zigb_app_handle_data_confirmation (APSDE_Data_Confirmation_t *pDataConfirmation)
{
	uint8_t ret;
	APSDE_Data_Confirmation_t *data_cnf = &(rsi_zigb_app_info.data_conf);      
	rsi_zigb_mcpy((uint8_t *)pDataConfirmation,(uint8_t *)data_cnf, sizeof(APSDE_Data_Confirmation_t));
	if (data_cnf->status ==0)
	{
		dataConfcnt_success++;
		log_and_print_info("Data Confirmation Received successfully[%d]-Tx Packet No:%d,Failed due to A7: %d, Failed due to Buffer Full or Errors:%d\n", dataConfcnt_success,
						                           tx_packet_counter,
						                           dataConfcnt_Failed,
						                           data_send_faile_buffer_full_or_error);
	}
	else
	{
		dataConfcnt_Failed++;
		log_and_print_info("Failed to get Data Confirmation status[%X]-Tx Packet No:%d\n", data_cnf->status,tx_packet_counter);
	}
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_child_join_handler(uint16_t short_address,
 *                                                   BOOL joining)
 * @brief       Child join handler 
 * @param[in]   Short_addr of child
 * @param[in]   Status of child joining/leaving 
 * @return      none
 * @section description
 * This API is used to handle child join/leave status
 *
 * ===========================================================================*/
extern uint16_t g_short_addr;
void rsi_zigb_app_child_join_handler(uint16_t short_address, BOOL joining)
{
	g_short_addr=rsi_zigb_app_cb.short_addr = short_address;
	if (joining==1)
	{
		log_and_print_info("Join Completed successfully with ShortAddr: 0x%X\n\t",short_address);
	}
	else
	{
		log_and_print_info("Failed to Join with ShortAddr: 0x%X\n\t",short_address);
	}
}

void log_and_print_info(const char *fmt, ...)
{
	struct timespec ts;
	char temp_buffer[3000];
	char temp_buffer1[1000];

	clock_gettime(CLOCK_REALTIME, &ts);
	va_list arg;
	memset(&arg,0,sizeof(va_list));
	va_start(arg, fmt);
	vsprintf(temp_buffer, fmt, arg);
	va_end(arg);

	//sprintf(temp_buffer1,"[%d:%d]- %s",(ts.tv_sec%3600),(ts.tv_nsec/1000000),temp_buffer);
	if (fLogPtr)
	{
		fprintf(fLogPtr,"%s\n",temp_buffer);
	}
	printf("%s\n",temp_buffer);
}

/*===========================================================================
 *
 * @fn          void rsi_zigb_app_cb_handler(uint8_t cmd_id, uint8_t *buffer)
 * @brief       Handler for asyncronous data and interface pkts
 * @param[in]   cmd type
 * @param[in]   Buffer 
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to handle different interface pkts
 * For eg: In this handler Scancomplete , network info/status, data Indication 
 * and Confirmation Pkts will be handled
 *
 * ===========================================================================*/

void rsi_zigb_app_cb_handler(uint8_t cmd_id, uint8_t *buffer,uint16_t length)
{
	uint8_t i = 0;
	rsi_zigb_app_info_t *app_info = &rsi_zigb_app_info;
	struct timespec ts;
	char temp_buffer[2000];
	char temp_buffer1[1000];
	clock_gettime(CLOCK_REALTIME, &ts);


	switch (cmd_id)
	{
		case APPSCANCOMPLETEHANDLER:
			{
				uint32_t channel = *(uint32_t *)buffer;
				uint8_t status = *(buffer + 4);

				rsi_zigb_app_scan_complete_handler (channel, status);
			}
			break;

		case APPENERGYSCANRESULTHANDLER:
			{
				uint32_t channel = *(uint32_t *)buffer;
				uint8_t pEnergyValue[16];

				rsi_zigb_mcpy((buffer + 4), pEnergyValue, 16);

				rsi_zigb_app_energy_scan_result_handler(channel, pEnergyValue);
			}
			break;

		case APPNETWORKFOUNDHANDLER:
			{
				ZigBeeNetworkDetails networkInformation;
				networkInformation.shortPanId = rsi_zigb_bytes2R_to_uint16(buffer);
				buffer += SHORT_PANID_SIZE;
				networkInformation.channel = *buffer++;

				rsi_zigb_mcpy(buffer, networkInformation.extendedPanId, EXTENDED_PANID_SIZE);
				buffer += EXTENDED_PANID_SIZE;

				networkInformation.stackProfile = *buffer++;
				networkInformation.nwkUpdateId = *buffer++;
				networkInformation.allowingJoining = (BOOL)*buffer++;

				rsi_zigb_app_network_found_handler(networkInformation);
			}
			break;

		case APPZIGBEESTACKSTATUSHANDLER:
			{
				ZigBeeNWKStatusInfo statusInfo;
				statusInfo = (ZigBeeNWKStatusInfo)*buffer;
				rsi_zigb_app_stack_status_handler(&statusInfo);
			}
			break;

		case APPINCOMINGMANYTOONEROUTEREQUESTHANDLER:
			{
				uint8_t pSrcIEEEAddr[8], PathCost;
				uint16_t SourceAddr;

				SourceAddr = rsi_zigb_bytes2R_to_uint16(buffer);
				buffer += 2;

				rsi_zigb_mcpy(buffer, pSrcIEEEAddr, EXTENDED_ADDR_SIZE);
				buffer += EXTENDED_ADDR_SIZE;

				PathCost = *buffer;

				rsi_zigb_app_incoming_many_to_one_route_req_handler(SourceAddr, pSrcIEEEAddr, PathCost);
			}
			break;

		case APPHANDLEDATAINDICATION:
			{  
				APSDE_Data_Indication_t pDataIndication;
				pDataIndication.dest_addr_mode = *buffer++;

				if (pDataIndication.dest_addr_mode == g_SHORT_ADDR_MODE_c)
				{
					pDataIndication.dest_address.short_address = rsi_zigb_bytes2R_to_uint16(buffer);
					buffer += 2;
				}
				else if (pDataIndication.dest_addr_mode == g_EXTENDED_ADDR_MODE_c)
				{
					for (i =0; i < 8; i++)
					{
						pDataIndication.dest_address.IEEE_address[i] = *buffer++;
					}
				}

				pDataIndication.dest_endpoint = *buffer++;
				pDataIndication.src_addr_mode = *buffer++;

				if (pDataIndication.src_addr_mode == g_SHORT_ADDR_MODE_c)
				{
					pDataIndication.src_address.short_address = rsi_zigb_bytes2R_to_uint16(buffer);
					buffer += 2;
				}
				else if (pDataIndication.src_addr_mode == g_EXTENDED_ADDR_MODE_c)
				{
					for (i =0; i < 8; i++)
					{
						pDataIndication.src_address.IEEE_address[i] = *buffer++;
					}
				}
				pDataIndication.src_endpoint = *buffer++;
				pDataIndication.profile_id = rsi_zigb_bytes2R_to_uint16(buffer);
				buffer += 2;

				pDataIndication.cluster_id = rsi_zigb_bytes2R_to_uint16(buffer);
				buffer += 2;

				pDataIndication.asdulength = *buffer++;
				pDataIndication.was_broadcast = *buffer++;
				pDataIndication.security_status = *buffer++;
				pDataIndication.link_quality = *buffer++;
				(pDataIndication.a_asdu) = app_info->zb_resp_info.asdu_pkt;

				for (i = 0; i < pDataIndication.asdulength; i++)
				{
					pDataIndication.a_asdu[i] = buffer[i];
				}
#ifndef ZB_API_TEST      
				rsi_zigb_app_handle_data_indication(&pDataIndication );
#else
				rsi_zigb_api_test_data_indication_handler(&pDataIndication);
#endif      

			}
			break;

		case APPHANDLEDATACONFIRMATION:
			{
				APSDE_Data_Confirmation_t DataConfirmation;
				DataConfirmation.dest_addr_mode = *buffer++;

				if (DataConfirmation.dest_addr_mode == g_SHORT_ADDR_MODE_c)
				{
					DataConfirmation.dest_address.short_address = rsi_zigb_bytes2R_to_uint16(buffer);
					buffer += 2;
				}
				else if (DataConfirmation.dest_addr_mode == g_EXTENDED_ADDR_MODE_c)
				{
					for (i =0; i < 8; i++)
					{
						DataConfirmation.dest_address.IEEE_address[i] = *buffer++;
					}
				}

				DataConfirmation.dest_endpoint = *buffer++;
				DataConfirmation.src_endpoint = *buffer++;
				DataConfirmation.status = *buffer++;

				rsi_zigb_app_handle_data_confirmation(&DataConfirmation);
				if (sem_post(&sem_udp_send) == -1)
				{
					printf("Failed to Wakeup Semaphore \n ");
				}
			}
			break;

		case APPCHILDJOINHANDLER:
			{
				BOOL Joining;
				uint16_t Short_address = rsi_zigb_bytes2R_to_uint16(buffer);
				buffer += 2;
				Joining = *buffer;
				rsi_zigb_app_child_join_handler(Short_address, Joining);
			}
			break;
		case ZIGBEE_HEART_BEAT:
			{
				unsigned long heart_beat_counter=0;
				heart_beat_counter=(buffer[0]<<24);
				heart_beat_counter|=(buffer[1]<<16);
				heart_beat_counter|=(buffer[2]<<8);
				heart_beat_counter|=buffer[3];

				log_and_print_info("Received Link Status Count: %d\n",heart_beat_counter);

			}
			break;

		case ZIGBEE_DEBUG:
			{
				if (buffer[1] != 0)
				{
					printf("Failed to Execute command %X[Error %X]\n",ZIGBEE_DEBUG,buffer[1]);
				}
				else
				{
					switch (buffer[0])
					{
						case SET_ZIGBEE_DEBUG_MODE:
							if (length <5)
							{
								log_and_print_info("Zigbee Debug Mode is Changed\n");
							}
							else
							{
								memset(temp_buffer,0,(length+10));
								memcpy(temp_buffer,&buffer[2],(length-2));
                                log_and_print_info(temp_buffer);
							}
							break;
						case GET_MEMORY_TABLE:
							{
								int mem_table_size,i;
								unsigned short *allocated_line_number_ptr;
								unsigned short *freed_line_number_ptr;
								unsigned char *allocated_buffer_status;
								char  temp_string[200];
                                				log_and_print_info("Received Memory Table of Size %d\n",buffer[2]);
								mem_table_size=buffer[2];
								allocated_buffer_status=&buffer[3];
								allocated_line_number_ptr=(unsigned short *)(&buffer[3+mem_table_size]);
								freed_line_number_ptr=(unsigned short *)(&buffer[3+3*mem_table_size]);
                                				log_and_print_info("Buffer ID\tStatus\tAllocated By\tFreed By\n");
								for (i=0 ; i < mem_table_size; i++ )
								{
                                    					log_and_print_info("   %d\t\t   %d\t      %d\t\t   %d\n",i,allocated_buffer_status[i],allocated_line_number_ptr[i],freed_line_number_ptr[i]);
								}
							}
							break;
						case GET_TRANSACTION_TABLE:
							{
								int txn_table_size,i;
								NSDU_Handle_Table_t *ptr,*ptr1;

                                				log_and_print_info("Received Transaction Table of Size %d\n",buffer[2]);
								ptr=(NSDU_Handle_Table_t *)&buffer[3];
								txn_table_size=buffer[2];
                                				log_and_print_info("Buffer ID\tNsduHandle\tBufferId\tAction On\tAPS Counter\n");
								for (i=0 ; i < txn_table_size; i++ )
								{
									ptr1=&ptr[i];
									log_and_print_info("   %d\t\t  %d\t\t  %d\t\t   %d\t\t    %d\n",i,ptr1->nsdu_handle,ptr1->buffer_id,ptr1->action_on_nlde_conf,ptr1->conf_details.aps_counter);
								}

							} 
							break;
						case GET_APP_QUEUE_TABLE:
						case GET_MSC_QUEUE_TABLE:
						case GET_MAC_QUEUE_TABLE:
							{
								queue_t *pQueueName;
								int que_table_size,i,j;
								que_table_size=buffer[2];
								pQueueName=(queue_t *)&buffer[3];
                                				log_and_print_info("\nReceived Que Table of Size %d\n",que_table_size);
								for (j=0;j<que_table_size;j++)
								{
									sprintf(temp_buffer,"W_indx:%d R_indx:%d Que[%s] :Max-%d ,Cur-%d ",pQueueName->write_index,pQueueName->read_index,pQueueName->name,pQueueName->max_used,pQueueName->current_capacity,pQueueName->max_used);
                                    for (i=0;i<24;i++)
									{
										sprintf(temp_buffer1,"%02d ",pQueueName->que_list[i]);
										strcat(temp_buffer,temp_buffer1);
									}
                                    					log_and_print_info("%s\n",temp_buffer);
									pQueueName++;
								}
							}
							break;
						case GET_SAS_INFO:
							{
								Startup_Attribute_Set_t *pSasInfo;
								pSasInfo=(Startup_Attribute_Set_t *)&buffer[3];
								sprintf(temp_buffer,"\nReceived SAS Information\n");
								sprintf(temp_buffer1,"Extended PAN ID: %02X %02X %02X %02X %02X %02X %02X %02X\n", pSasInfo->a_extended_pan_id[0],pSasInfo->a_extended_pan_id[1],
											pSasInfo->a_extended_pan_id[2],pSasInfo->a_extended_pan_id[3],pSasInfo->a_extended_pan_id[4], pSasInfo->a_extended_pan_id[5],
										     pSasInfo->a_extended_pan_id[6],pSasInfo->a_extended_pan_id[7]);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Network Key:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",pSasInfo->a_network_key[0],
										pSasInfo->a_network_key[1],pSasInfo->a_network_key[2],pSasInfo->a_network_key[3],pSasInfo->a_network_key[4],pSasInfo->a_network_key[5],
										pSasInfo->a_network_key[6],pSasInfo->a_network_key[7],pSasInfo->a_network_key[8],pSasInfo->a_network_key[9],pSasInfo->a_network_key[10],
										pSasInfo->a_network_key[11],pSasInfo->a_network_key[12],pSasInfo->a_network_key[12],pSasInfo->a_network_key[14],pSasInfo->a_network_key[15]);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Channel Mask:%X\n",pSasInfo->channel_mask);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"End Device Bind Timeout:%d\n",pSasInfo->end_device_bind_timeout);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"PAN ID:%X\n",pSasInfo->a_pan_id);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Pre Link Key:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",pSasInfo->a_preconfigured_link_key[0],
										pSasInfo->a_preconfigured_link_key[1],pSasInfo->a_preconfigured_link_key[2],pSasInfo->a_preconfigured_link_key[3],pSasInfo->a_preconfigured_link_key[4],pSasInfo->a_preconfigured_link_key[5],
										pSasInfo->a_preconfigured_link_key[6],pSasInfo->a_preconfigured_link_key[7],pSasInfo->a_preconfigured_link_key[8],pSasInfo->a_preconfigured_link_key[9],pSasInfo->a_preconfigured_link_key[10],
										pSasInfo->a_preconfigured_link_key[11],pSasInfo->a_preconfigured_link_key[12],pSasInfo->a_preconfigured_link_key[12],pSasInfo->a_preconfigured_link_key[14],pSasInfo->a_preconfigured_link_key[15]);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Trust Centre Address: %02X %02X %02X %02X %02X %02X %02X %02X\n", pSasInfo->a_trust_center_address[0],pSasInfo->a_trust_center_address[1],
											pSasInfo->a_trust_center_address[2],pSasInfo->a_trust_center_address[3],pSasInfo->a_trust_center_address[4], pSasInfo->a_trust_center_address[5],
										     pSasInfo->a_trust_center_address[6],pSasInfo->a_trust_center_address[7]);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Trust Centre Master Key:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",pSasInfo->a_trustcenter_master_key[0],
										pSasInfo->a_trustcenter_master_key[1],pSasInfo->a_trustcenter_master_key[2],pSasInfo->a_trustcenter_master_key[3],pSasInfo->a_trustcenter_master_key[4],pSasInfo->a_trustcenter_master_key[5],
										pSasInfo->a_trustcenter_master_key[6],pSasInfo->a_trustcenter_master_key[7],pSasInfo->a_trustcenter_master_key[8],pSasInfo->a_trustcenter_master_key[9],pSasInfo->a_trustcenter_master_key[10],
										pSasInfo->a_trustcenter_master_key[11],pSasInfo->a_trustcenter_master_key[12],pSasInfo->a_trustcenter_master_key[12],pSasInfo->a_trustcenter_master_key[14],pSasInfo->a_trustcenter_master_key[15]);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Indirect Poll Rate:%d\n",pSasInfo->indirect_poll_rate);
								strcat(temp_buffer,temp_buffer1);
								
								sprintf(temp_buffer1,"Max rejoin Interval:%d\n",pSasInfo->max_rejoin_interval);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Parent Retry Threshoold:%d\n",pSasInfo->parent_retry_threshold);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"End Device Bind Timeout:%d\n",pSasInfo->end_device_bind_timeout);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Scan Attempts:%d\n",pSasInfo->scan_attempts);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Time Between Scan:%d\n",pSasInfo->time_between_scans);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Startup Control:%d\n",pSasInfo->startup_control);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Use Insecure Join:%d\n",pSasInfo->use_insecure_join);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Protocol Versio:%d\n",pSasInfo->protocol_version);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Stack Profile:%d\n",pSasInfo->stack_profile);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Security Timeout Period:%d\n",pSasInfo->security_timeout_period);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"APS ack wait duration:%d\n",pSasInfo->APS_ack_wait_duration);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Short Addres:%X\n",pSasInfo->a_short_address);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Concentrator Flag:%X\n",pSasInfo->concentrator_flag);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Concentrator RADIUS:%X\n",pSasInfo->concentrator_radius);
								strcat(temp_buffer,temp_buffer1);
								sprintf(temp_buffer1,"Concentrator Discovery Time:%X\n",pSasInfo->concentrator_discovery_time);
								strcat(temp_buffer,temp_buffer1);
								log_and_print_info(temp_buffer);
							}
							break;
						case ZIGBEE_DEBUG_TRACE:
							{
								memset(temp_buffer,0,(length+10));
								memcpy(temp_buffer,&buffer[2],(length-2));
								log_and_print_info(temp_buffer);
							}
							break;
						case GET_MSDU_TABLE:
							{
								int txn_table_size,i;
								Msdu_Handle_Table_t *ptr,*ptr1;
								log_and_print_info("Received MSDU Table of Size %d\n",buffer[2]);
								ptr=(Msdu_Handle_Table_t *)&buffer[3];
								txn_table_size=buffer[2];
								log_and_print_info("S.No\tMsduHandle\tModule\tNBT_index\tNWK Dest Addr\tNWK Src Addr\n");
								for (i=0 ; i < txn_table_size; i++ )
								{
									ptr1=&ptr[i];
                                    log_and_print_info(" %d\t  %d\t  %d\t\t  %d\t\t   %X\t\t    %X\n",i ,ptr1->msdu_handle,ptr1->module,ptr1->NBT_index,ptr1->nwk_dest_addr,ptr1->nwk_src_addr);
								}

							} 
							break;
						default:
							buffer[0]=0xFF;
							//length=1;
							break;
					}

				}

			}
			break;
		case ZIGBEESENDUNICASTDATA:
			{
				uint8_t status = *(buffer + 0);
				if (status == 0)
				{
					log_and_print_info("Data sent to card successfully\n");
				}
				else
				{
					data_send_faile_buffer_full_or_error++;
					if (status == 4)
					{
						log_and_print_info("Failed to Send Unicast Data due to buffer Full\n");
					}
					else
					{
						log_and_print_info("Failed to Send Unicast Data due to Error %X\n",status);
					}
					sleep(1);
				    sem_post(&sem_udp_send);
				}
			}
			break;
		default:
			break;
	}
	fflush(fLogPtr);
}
