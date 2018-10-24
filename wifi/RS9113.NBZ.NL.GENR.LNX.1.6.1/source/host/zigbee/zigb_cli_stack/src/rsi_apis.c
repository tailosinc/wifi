
#include "rsi_frame.h"
#include "rsi_zigb_config.h"
#include "rsi_zigb_global.h"
#include "rsi_zigb_api.h"
#include "rsi_zigb_app.h"
#include "platform_specific.h"
#include "rsi_serial.h"
unsigned long Packet_Received=0;
unsigned long Packet_Sent=0;

unsigned long unidirectional_packet_Sent=0;


extern rsi_zigb_app_cb_t   rsi_zigb_app_cb;
extern rsi_zigb_app_info_t rsi_zigb_app_info;
extern void measure_throughput(uint32_t  pkt_length, uint32_t tx_rx);

/* Basic, Identify and ON_OFF switch configuration clusters on Server side. */
const cluster_id_t a_In_Cluster_List_For_Custom_Device[] = {
	g_BASIC_CLUSTER_c,
	g_IDENTIFY_CLUSTER_c,
	g_ON_OFF_SWITCH_CONFIGURATION_CLUSTER_c
};

/* ON_OFF cluster on Client side */
const cluster_id_t a_Out_Cluster_List_For_Custom_Device[] = {
	g_ON_OFF_CLUSTER_c
};

/* Simple Descriptor */
Simple_Descriptor_t SimpleDesc =  
{
	HA_PROFILE_ID,
	DEV_ID_ON_OFF_SWITCH,
	0x00,
	sizeof(a_In_Cluster_List_For_Custom_Device)/  sizeof(cluster_id_t),
	(cluster_id_t*)a_In_Cluster_List_For_Custom_Device,
	sizeof(a_Out_Cluster_List_For_Custom_Device)/ sizeof(cluster_id_t),
	(cluster_id_t*)a_Out_Cluster_List_For_Custom_Device
};

/* Default values of the Startup Attribute Set */
APP_Startup_Attribute_Set_t APP_Startup_Attribute_Set_Default =
{
	g_EXTENDED_PAN_ID_c,
	g_CHANNEL_MASK_c,
	g_STARTUP_CONTROL_c,
	g_USE_INSECURE_JOIN_c,
	g_SCAN_ATTEMPTS_c,
	g_PARENT_RETRY_THRESHOLD_c,
	g_TRUST_CENTER_ADDRESS_c,
	g_NETWORK_KEY_c,
	g_TIME_BETWEEN_SCANS_c,
	g_REJOIN_INTERVAL_c,
	g_MAX_REJOIN_INTERVAL_c,
	g_POLL_RATE_c,
	g_PANID_c,
	g_NETWORK_MANAGER_ADDRESS_c,
	g_TC_MASTER_KEY_c,
	g_PRECONFG_LINK_KEY_c,
	g_END_DEVICE_TIMEOUT_c
};

/* Default value of the ZDO configuration table */
ZDO_Configuration_Table_t g_Table_Default =
{
	g_PERMIT_JOIN_DURATION_c,
	g_NWK_SECURE_ALL_FRAMES_c,
	g_FORMATION_ATTEMPTS_c,
	g_SCAN_DURATION_c,
	g_JOIN_ATTEMPTS_c,
	g_PRECONFIGURED_KEY_c,
	g_TRUST_CENTER_SHORT_ADDRESS_c,
	g_AUTOMATIC_POLL_ALLOWED_c,
	g_AUTHENTICATION_POLL_RATE_c,
	g_SWITCH_KEY_TIMEOUT_c,
	g_NWK_SECURITY_LEVEL_c,
	g_APS_ACK_POLL_TIME_OUT_c,
	g_MANUFACTURER_CODE_c,
};

/*==================================================*/
/**
 * @fn          void rsi_buildFrameDescriptor(rsi_zigb_uFrameDsc *uFrameDscFrame, uint8_t *cmd)
 * @brief       Creates a Frame Descriptor
 * @param[in]   rsi_zigb_uFrameDsc *uFrameDscFrame,Frame Descriptor
 * @param[in]   uint8_t *cmd,Indicates type of the packet(data or management)
 * @param[out]  none
 * @return      none
 */

uint8_t wifi_packet =0;
int16_t rsi_serial_frame_write(uint8_t *packet_buffer, int16_t length)
{
	int16_t retval = 0;
	uint16_t ii = 0;
	struct timespec ts;
	char temp_buffer[200];

	clock_gettime(CLOCK_REALTIME, &ts);
	

	if (length ==82)
	{
		unidirectional_packet_Sent++;
		sprintf(temp_buffer,"UDP TX PACKET ID:%d",unidirectional_packet_Sent);
		DebugDumpBytes(packet_buffer,length,temp_buffer);
	}
	else
	{
		Packet_Sent++;
		sprintf(temp_buffer,"Management TX PACKET [%d:%d]",Packet_Sent,Packet_Received);
		DebugDumpBytes(packet_buffer,length,temp_buffer);
	}
	retval = write(rsi_linux_app_cb.ttyfd, packet_buffer, length);
	/* Return success */
	return retval;
}

void rsi_zigb_build_frame_descriptor(rsi_zigb_uFrameDsc *uFrameDscFrame, uint8_t *cmd, uint8_t size_param)
{
	uint8_t i;
	for (i = 0; i < RSI_ZIGB_FRAME_DESC_LEN; i++)
	{
		uFrameDscFrame->uFrmDscBuf[i] = 0;
	}

	/* Update the ZigBee Descriptor */

	//! Length of the frame 
	uFrameDscFrame->uFrmDscBuf[0]  = size_param;
	//! ZigBee Host Queue Type
	if (wifi_packet)
	{
		uFrameDscFrame->uFrmDscBuf[1]  = 0x40;
		//! Common Opermode
		uFrameDscFrame->uFrmDscBuf[2]  = 0x10;
		//! Direction
		uFrameDscFrame->uFrmDscBuf[13] = 0;
		wifi_packet =0;
	}
	else
	{
		uFrameDscFrame->uFrmDscBuf[1]  = 0x10;
		//! Direction
		uFrameDscFrame->uFrmDscBuf[13] = 1;
	}
	//! Interface Type
	uFrameDscFrame->uFrmDscBuf[14] = cmd[0];
	//! Command Type
	uFrameDscFrame->uFrmDscBuf[15] = cmd[1];
	return;
}

/*=================================================*/
/**
 * @fn          uint8_t *rsi_alloc_and_init_cmdbuff(uint8_t *Desc,
 *                                       uint8_t *payload,
 *                                       uint16_t payload_size)
 * @brief       To allocate and initialise the command buffer.
 * @param[in]   pkt_struct_t *Pkt
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to allocate a buffer for the command to send and
 * initializing it with all the header bytes, Desc and payload filled.
 */
/*uint8_t *rsi_alloc_and_init_cmdbuff(uint8_t *Desc, uint8_t *payload, uint16_t payload_size)
{
	uint8_t          *cmd_buff; 

	rsi_os_printf(RSI_PL14,"\nrsi_alloc_and_init_cmdbuff\n");
	cmd_buff = rsi_malloc(payload_size + RSI_FRAME_DESC_LEN );

	memcpy(cmd_buff, Desc, RSI_FRAME_DESC_LEN);

	if (payload_size)
		memcpy(cmd_buff + RSI_FRAME_DESC_LEN, payload, payload_size);

	return cmd_buff;
}*/

/*====================================================*/
/**
 * @fn          int16_t rsi_execute_cmd(uint8_t *descparam,uint8_t *payloadparam,uint16_t size_param)
 * @brief       Common function for all the commands.
 * @param[in]   uint8_t *descparam, pointer to the frame descriptor parameter structure
 * @param[in]   uint8_t *payloadparam, pointer to the command payload parameter structure
 * @param[in]   uint16_t size_param, size of the payload for the command
 * @return      errCode
 *              -2 = Command issue failed
 *              0  = SUCCESS
 * @section description
 * This is a common function used to process a command to the Wi-Fi module.
 */

/*int16_t rsi_zigb_execute_cmd(uint8_t *descparam, uint8_t *payloadparam, uint16_t size_param)
{
	int16_t                                         retval = 0;
	rsi_zigb_uFrameDsc         uFrameDscFrame;
	uint8_t           *cmd_buff;

	rsi_os_printf(RSI_PL1,"\nrsi_zigb_execute_cmd:\n");
	//! Build 16 bytes, send/receive command descriptor frame
	rsi_zigb_build_frame_descriptor(&uFrameDscFrame, descparam, size_param);
	cmd_buff = rsi_alloc_and_init_cmdbuff((uint8_t *)&uFrameDscFrame,payloadparam,size_param);

	if (rsi_serial_frame_write(cmd_buff, cmd_buff[0]+16) < 0)
	{
		retval = -2;
	}
	if (retval < 0)
	{
		rsi_os_printf(RSI_PL1,"Unable to issue command\n");
	}

	//! Free the command buffer
	rsi_free(cmd_buff);

	return retval;
}*/

/*=================================================*/
/**
 * @fn          void rsi_enqueue_to_rcv_q(pkt_struct_t *Pkt)
 * @brief       To enqueue the packet to receive queue
 * @param[in]   pkt_struct_t *Pkt
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to enqueue the received packet from kernel
 * to receive queue.
 */
void rsi_enqueue_to_rcv_q(rsi_pkt_t *pkt)
{
	pkt_queue_t *rcv_q = &rsi_linux_app_cb.rcv_queue;

//! check queue is empty
	if (!rcv_q->pending_pkt_count)
	{
		//! if empty then add packet as first packet (head & tail point to first packet)
		rcv_q->head = rcv_q->tail = pkt;
	}
	else
	{
		//! if not empty append the packet to list at tail
		rcv_q->tail->next = pkt;

		//! Make packet as tail
		rcv_q->tail = pkt;
	}

	//! increment packet pending count
	rcv_q->pending_pkt_count++;
#ifdef ZB_ENABLE
	rsi_zigb_app_cb.pkt_pending = RSI_TRUE;
#endif  

	return;
}

/*=================================================*/
/**
 * @fn          pkt_struct_t *rsi_dequeue_from_rcv_q(void)
 * @brief       To dequeue the packet to receive queue
 * @param[in]   none
 * @param[out]  none
 * @return      pkt_struct_t *Pkt, dequeued packet pointer
 * @section description
 * This API is used to dequeue the packets from receive queue
 * when packets are pending in queue.
 */
rsi_pkt_t *rsi_dequeue_from_rcv_q(void)
{
	rsi_pkt_t *pkt = NULL;
	pkt_queue_t *rcv_q = &rsi_linux_app_cb.rcv_queue;

	//! check queue is empty
	if (!rcv_q->pending_pkt_count)
	{
		//! return NULL if queue is empty
		return NULL;
	}

	//! dequeue the packet from queue 
	pkt = rcv_q->head;

	//! update the queue head and decrement pending count
	rcv_q->head = rcv_q->head->next; 

	//! Decrease pending packet count
	rcv_q->pending_pkt_count--;

	//! if pending count is zero, then reset head and tail 
	if (!rcv_q->pending_pkt_count)
	{
		rcv_q->head = NULL;
		rcv_q->tail = NULL;
	}
#ifdef ZB_ENABLE
	rsi_zigb_app_cb.pkt_pending = RSI_FALSE;
#endif

	return pkt;
}

/*===========================================================================
 *
 * @fn          void rsi_zb_app_init(void)
 * @brief       Initialize ZigBee structues and variables
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to Initialize ZigBee specific variables and structures
 * to default values
 *
 * ===========================================================================*/

uint8_t rsi_zigb_channel_ext(uint32_t channel_Mask)
{
	uint8_t i;
	for (i=0 ; i<16 ; i++)
	{
		if ((channel_Mask >> i) & (0x00000800))
			return(i+11);
	}
	return(11);
}

void rsi_zb_app_init(void)
{
	rsi_zigb_app_cb_t *app_cb_ptr = &rsi_zigb_app_cb;
	rsi_zigb_app_info_t *app_info = &rsi_zigb_app_info;
#ifdef ZB_API_TEST  
	rsi_zigb_apitest_t  *api_test_info = &rsi_zigb_apitest;
#endif  

	app_cb_ptr->pkt_pending = RSI_FALSE;
	app_cb_ptr->fsm_state = 0;//FSM_CARD_NOT_READY;
	app_info->status_var.scanReqSent = 0;

	/*Initializing status variables */
	app_info->status_var.matchDescRspStatus = 0x0f;
	app_info->status_var.powerDescRspStatus = 0x0f;
	app_info->status_var.nodeDescRspStatus = 0x0f;
	app_info->status_var.bindRspStatus = 0x0f;
	app_info->status_var.unbindRspStatus = 0x0f;
	app_info->status_var.actepRspStatus = 0x0f;
	app_info->status_var.ieeeaddrRspStatus = 0x0f;
	app_info->status_var.simpledescRspStatus = 0x0f;
	app_info->status_var.networkaddrRspStatus = 0x0f;

	app_info->DeviceSimpleDesc = &SimpleDesc;

	app_info->channel_mask = g_CHANNEL_MASK_c;
	app_info->scan_duration = g_SCAN_DURATION_c;
	app_cb_ptr->channel = rsi_zigb_channel_ext(g_CHANNEL_MASK_c);
	app_cb_ptr->power = 10;

}


/*===========================================================================
 *
 * @fn          uint8_t rsi_zigb_zcl_create_command (uint8_t direction, uint8_t *p_asdu,
									  void* p_ZCL_Cmd_Data, uint8_t ZCL_Cmd_Data_Length,
									  uint8_t trans_seq_num)
 * @brief       Prepares the ZigBee Cluster command 
 * @param[in]   Direction
 * @param[in]   p_asdu - buffer pointer of data
 * @param[in]   p_ZCL_Cmd_Data - Cluster data
 * @param[in]   length of ZCL data
 * @param[in]   Seq num
 * @param[out]  none
 * @return      Final data length
 * @section description
 * This API is used to prepare the ZigBee Cluster command pkt
 *
 * ===========================================================================*/

uint8_t rsi_zigb_zcl_create_command (uint8_t direction, uint8_t *p_asdu,
									 void* p_ZCL_Cmd_Data, uint8_t ZCL_Cmd_Data_Length,
									 uint8_t trans_seq_num)
{
	App_ZCL_Request_t *p_received_data = ( App_ZCL_Request_t *)p_ZCL_Cmd_Data;
	uint8_t data_length = 0;
	BOOL manufacture_specific = p_received_data->manufacture_specific;
	BOOL disable_default_response = p_received_data->disable_default_response;
	uint8_t *p_ZCL_Header_Payload = p_asdu;

	if ( !( p_received_data->command_type & 0x01 ))
	{
		((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->frame_control = g_Generic_Cmd_c ;
	}
	else
	{
		((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->frame_control = g_Cluster_Specific_Cmd_c;
	}
	((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->frame_control |= direction;

	data_length++;

	if ( disable_default_response )
	{
		((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->frame_control |= g_Disable_Default_Response_c;
	}
	if ( manufacture_specific )
	{
		((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->frame_control |= g_Manufacture_Specific_Bit_c ;
		rsi_zigb_mcpy((uint8_t*)p_received_data->a_manufacturer_code, (uint8_t*)((ZCL_Header_And_Payload_t*)p_ZCL_Header_Payload)->a_manufacture_code ,2 );
		data_length += sizeof(uint16_t);
		ZCL_Cmd_Data_Length -= 0x03;
	}
	else
	{
		ZCL_Cmd_Data_Length -= 0x05;
	}
	*( p_ZCL_Header_Payload + data_length ) = trans_seq_num;

	data_length++;

	*( p_ZCL_Header_Payload +  data_length ) = p_received_data->ZCL_command_received.command_identifier;

	ZCL_Cmd_Data_Length--;
	data_length++;

	rsi_zigb_mcpy((uint8_t*)&( p_received_data->ZCL_command_received.Foundation_Commands ) ,
				  p_ZCL_Header_Payload + data_length, ZCL_Cmd_Data_Length);

	data_length += ZCL_Cmd_Data_Length;

	return data_length;
}

/*===========================================================================
 *
 * @fn          uint8_t rsi_zigb_app_send_data( uint8_t direction, uint8_t commandType, 
 *                                              uint8_t destEP, uint16_t dest_short_address,
												uint8_t commandId, uint16_t cluster, 
												uint8_t dataLength,uint8_t* payloadPointer )
 * @brief       Prepares ZigBee data pkt
 * @param[in]   Direction
 * @param[in]   Command type
 * @param[in]   Destination End Point
 * @param[in]   Destination Short address
 * @param[in]   ZCL Command ID received
 * @param[in]   Cluster type
 * @param[in]   Data length
 * @param[in]   Payload pointer
 * @param[out]  none
 * @return      Status
 * @section description
 * This API is used to prepare the ZigBee Data pkt with cluster information
 *
 * ===========================================================================*/

uint8_t rsi_zigb_app_send_data( uint8_t direction, uint8_t commandType, uint8_t destEP, uint16_t dest_short_address,
								uint8_t commandId, uint16_t cluster, uint8_t dataLength,uint8_t* payloadPointer )
{
	uint8_t status;
	Address DestAddress;
	ZigBeeAPSDEDataRequest_t APSDERequest;
	App_ZCL_Request_t *pZcl_hdr;
	uint8_t *pAsdu;

	/*+1 is added for Command id*/
	uint8_t ZCLHeaderLength = ((sizeof(App_ZCL_Request_t) - sizeof(ZCL_Command_t)) + 1);

	//DestAddress.short_address = 0x00;
	DestAddress.short_address = dest_short_address;

	APSDERequest.ProfileId = HA_PROFILE_ID;
	APSDERequest.DestEndpoint = destEP;
	APSDERequest.ClusterId = cluster;
	APSDERequest.AsduLength = dataLength;
	APSDERequest.SrcEndpoint = ONOFF_SWITCH_END_POINT;
	APSDERequest.TxOptions = g_APS_Tx_Opt_Use_NWK_Key_c | g_APS_Tx_Opt_Ack_Req_c;
	APSDERequest.Radius = DEFAULT_RADIUS;

	pZcl_hdr = (App_ZCL_Request_t*)APSDERequest.aPayload;
	pZcl_hdr->command_type = commandType;
	pZcl_hdr->disable_default_response = g_Disable_Default_Response_c;
	pZcl_hdr->manufacture_specific = 0;
	pZcl_hdr->ZCL_command_received.command_identifier = commandId;
	pAsdu = APSDERequest.aPayload + ZCLHeaderLength;

	if(payloadPointer)
	{
		rsi_zigb_mcpy(payloadPointer,pAsdu, dataLength );
	}

	APSDERequest.AsduLength =  rsi_zigb_zcl_create_command ( direction,
															 APSDERequest.aPayload,
															 (App_ZCL_Request_t*)&APSDERequest.aPayload,
															 dataLength + ZCLHeaderLength ,
															 0);
	measure_throughput(100,0);
	status = rsi_zigb_send_unicast_data(ZigBee_Outgoing_Direct,
										DestAddress  , &APSDERequest);

	return status;
}
/*=============================================================================*/
/**
 * @fn              uint16 rsi_bytes2R_to_uint16(uint8 *dBuf)
 * @brief           Convert a 2 byte array to uint16, first byte in array is LSB
 * @param[in]       uint8 *dBuf,pointer to a buffer to get the data from
 * @param[out]      none
 * @return          uint16, converted data
 */
uint16_t rsi_bytes2R_to_uint16(uint8_t *dBuf)
{
	uint16_t        val;    
#ifdef RSI_LITTLE_ENDIAN
	val = dBuf[1];
	val <<= 8;
	val |= dBuf[0] & 0x000000ff;
#else
	val = dBuf[0];
	val <<= 8;
	val |= dBuf[1] & 0x000000ff;
#endif
	return val;
}

extern unsigned long Packet_Sent;
//extern unsigned long Packet_Received;

/*===========================================================================
 *
 * @fn          rsi_zigb_uCmdRsp *rsi_zigb_app_frame_process(uint8_t *buffer)
 * @brief       Processes the received ZigBee pkt
 * @param[in]   buffer- buffer pointer to fill the descriptor
 * @param[out]  none
 * @return      Buffer pointer after parsing descriptor
 * @section description
 * This API is used to parse ZigBee specific descriptor and return paylod with
 * cmd_id and intf_id of the pkt
 *
 * ===========================================================================*/

int LastReceivedPacketNumber=0;
extern int enable_dut_debug;
rsi_zigb_uCmdRsp *rsi_zigb_app_frame_process(uint8_t *buffer)
{
	uint16_t payload_length = 0;
	uint16_t seq_no = 0;
	uint8_t *buf_ptr, cmd_id = 0, intf_id = 0;
	rsi_zigb_uCmdRsp *temp_ptr;
	struct timespec ts;
	char temp_buffer[200];
	int i=0;


	clock_gettime(CLOCK_REALTIME, &ts);
	/* Point the buffer pointer to interface and cmd id
 * We can just verify seq num, direction */
	temp_ptr = (rsi_zigb_uCmdRsp *)(buffer + RSI_ZIGB_SEQ_NUM_OFFSET);
	cmd_id = temp_ptr->cmd_id;

	if ((cmd_id !=ZIGBEE_HEART_BEAT) && (cmd_id !=ZIGBEE_DEBUG))
	{
		if ((16+buffer[0]) == 86)
		{
			unsigned long packet_id;
			packet_id=(buffer[45]<<24);
			packet_id|=(buffer[46]<<16);
			packet_id|=(buffer[47]<<8);
			packet_id|=buffer[48];
			if (LastReceivedPacketNumber != 0)
			{
				if (LastReceivedPacketNumber != (packet_id-1))
				{
					log_and_print_info("Expected %d Receivd %d\n",(LastReceivedPacketNumber+1),packet_id);
					if (packet_id ==0 )
					{
						if (enable_dut_debug==0)
						{
							enable_dut_debug=1;
							sprintf(temp_buffer,"UDP RX PACKET NO:%d",packet_id);
							DebugDumpBytes(buffer,(16+buffer[0]),temp_buffer);
							enable_dut_debug=0;
						}
					}
				}
				if (LastReceivedPacketNumber == packet_id)
				{
					log_and_print_info("\n\nSame packet number received agian\n\n");
				}

			}
			sprintf(temp_buffer,"UDP RX PACKET NO:%d",packet_id);
			DebugDumpBytes(buffer,(16+buffer[0]),temp_buffer);
			LastReceivedPacketNumber=packet_id;
		}
		else
		{
			sprintf(temp_buffer,"Management RX Packet[%d]",Packet_Received);
			DebugDumpBytes(buffer,(16+buffer[0]),temp_buffer);
		}
	}
	/* Step-1:- Get the payload length */
	payload_length = ((uint16_t )buffer[0] & 0xfff);

	if (payload_length >= RSI_ZIGB_MAX_PAYLOAD_SIZE)
	{
		rsi_os_printf(RSI_PL1,"Error: RSI_ZB_PAYLOAD_SIZE_ERROR \n");
		return NULL; 
	}
//#ifdef ZB_ONLY
	/* Check Queue no and differentiate protocol */
	if (((buffer[1] & 0xF0) == 0x40) && ((buffer[2] == 0x10)||(buffer[2] == 0x89)))
	{
		/* Incrementing this buffer to get the Sequence number
		* starting address */
		temp_ptr->intf_id = 7;
		temp_ptr->cmd_id = ZIGBEE_OPER_MODE_RSP;
		return temp_ptr;
	}
//#endif  

	/* Incrementing this buffer to get the Sequence number
	 * starting address */
	buf_ptr = (buffer + RSI_ZIGB_SEQ_NUM_OFFSET);

	/* Step-2:- Sequence number Verification 
	 * Currently we are just taking it as token number
	 * only  for debugging*/
	seq_no = *buf_ptr++; 

	if (seq_no >= RSI_ZIGB_MAX_PAYLOAD_SIZE)
	{
		rsi_os_printf(RSI_PL1,"Error: RSI_ZB_ERROR_SEQUENCE_NUMBER \n");
		return NULL; 
	}
	/* Step-3:- Direction check */
	if (*buf_ptr++ != DIR_DEVICE_TO_HOST)
	{

		return NULL; 
	}

	/* Step-4:- Get the Interface Id */
	intf_id = *buf_ptr++;

	if (intf_id > INTERNAL_MANAGEMENT_INTERFACE)
	{
		rsi_os_printf(RSI_PL1,"Error: RSI_ZB_INTERFACE_ERROR \n");
		return NULL; 
	}

	/* Step-5:- Get the Command Id */
//  cmd_id = *buf_ptr++;

	if (!cmd_id)
	{
		rsi_os_printf(RSI_PL1,"Error: RSI_ZB_COMMAND_ERROR \n");
		return NULL; 
	}

	/* Return buffer only from Dir present in desc 
	 * rest of desc is discarded*/
	return temp_ptr;
}
