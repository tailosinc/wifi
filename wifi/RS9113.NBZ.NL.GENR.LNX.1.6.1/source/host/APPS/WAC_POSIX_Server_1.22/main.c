#include <stdio.h>
#include "WACServerAPI.h"
#include "dns_sd.h"
#include "platform_specific.h"
#include "rsi_nl_app.h"
#include "rsi_app.h"
#include<signal.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>

#ifndef ONEBOX_EMB 
#define DRV_PARAMS		50
#define ONEBOX_HOST_IOCTL     SIOCIWLASTPRIV - 0x0B
typedef struct driver_parma_s
{
	unsigned char mac_addr[6];
	unsigned char fw_ver[8];
	unsigned char module_type;
}driver_param_t;
#endif

WACContext_t WAC_incontext;
void sigint_handler(int sig);
void WAC_start_handler()
{
  return;

}

uint8_t fw_version[] = "1.5.0";
uint8_t hw_version[] = "1.5.0";
uint8_t serial_number[] = "13";
uint8_t name[20];
uint8_t model[] = "RS9113";
uint8_t manufacturer[] = "REDPINE";
uint8_t *support[2]= {{"12"},{"AA"}};
const char ProgramName[] = "mDNSClientPosix";

rsi_app_cb_t rsi_app_cb;
rsi_linux_app_cb_t   rsi_linux_app_cb;
pthread_t wac_rcv_thd;

#ifdef ONEBOX_EMB 
void * wac_read_pkt_thread(void * arg )
{
  pkt_struct_t *rcvPktPtr;
  int32 rsp_len;
  uint8 *volatile descPtr    = NULL;

  char *s = arg;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nRecvThreadBody:\n");
#endif
  RSI_DPRINT(RSI_PL13,"%s\n",s);
  while(1)
  {
    rcvPktPtr = (pkt_struct_t*)rsi_malloc(RSI_MAX_PAYLOAD_SIZE + RSI_RXPKT_HEAD_ROOM);
    if(rcvPktPtr == NULL)
    {
#ifdef RSI_DEBUG_PRINT
      RSI_DPRINT(RSI_PL13,"Allocation failed to recv packet\n");
#endif
      return NULL;
    }
    rcvPktPtr->data = (uint8 *)(((uint8 *)rcvPktPtr) +  RSI_RXPKT_HEAD_ROOM);

    rsp_len = recv(rsi_linux_app_cb.nl_sd, rcvPktPtr->data , RSI_MAX_PAYLOAD_SIZE, 0);
   if(rsp_len < 0)
    {
      perror("recv");
#if ((defined LINUX_PLATFORM) && (defined RSI_DEBUG_PRINT))
      fprintf(stderr," RecvThreadBody ERROR NUMBER = %d \n",errno);
#endif
      if(errno == ENOBUFS || errno == ESPIPE)
      {
        //! Handling for No buffer space available Error
        rsi_free(rcvPktPtr);
        continue;
      }
      return NULL;
    }
    descPtr    = (rcvPktPtr->data + RSI_NL_HEAD_SIZE);
    if(descPtr[2] != rsi_app_cb.expected_response)
    {
        rsi_free(rcvPktPtr);
        continue;
    }
 
    pthread_mutex_lock(&rsi_linux_app_cb.mutex1);
    rsi_enqueue_to_rcv_q(rcvPktPtr);
    rsi_app_cb.pkt_pending = 1;
    pthread_mutex_unlock(&rsi_linux_app_cb.mutex1);
  }
}

int16 rsi_frame_read(uint8 *packet_buffer)
{
  /* Variables */
  rsi_linux_app_cb_t *linux_app_cbPtr = &rsi_linux_app_cb;
  pkt_struct_t *rspPtr = NULL;

  /* Length of the data to copy */
  int16 length = 0;

  /* Pointer to the Packet file descriptor */
   uint8 * volatile descPtr    = NULL;

#ifdef RSI_DEBUG_PRINT
  int i;
#endif

  /* Do actual deque from the RX queue */
  pthread_mutex_lock(&linux_app_cbPtr->mutex1);
  rspPtr = rsi_dequeue_from_rcv_q();
  rsi_app_cb.pkt_pending = 0;
  pthread_mutex_unlock(&linux_app_cbPtr->mutex1);
  /* Assign pointers to appropriate addresses */

  
  descPtr    = (rspPtr->data + RSI_NL_HEAD_SIZE);
  /* Calculate length of the packet from the first two bytes of the frame descriptor */
  length = rsi_bytes2R_to_uint16(descPtr);
  length &= 0x0FFF;
  length += RSI_FRAME_DESC_LEN;

  /* Debug: Print the length & contents of the packet */
#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL0,"RX Len of the packet: %d\n", length);
  for (i=0; i<length; i++) {
      RSI_DPRINT (RSI_PL0, "0x%x ", descPtr[i]);
      if ((i % 16 == 0) && (i != 0)) {
          RSI_DPRINT(RSI_PL0, "\n");
      }
  }
  RSI_DPRINT(RSI_PL0, "\n");
#endif

  memset(packet_buffer, 0, length);
  memcpy(packet_buffer, descPtr, length);
  rsi_free(rspPtr);

  /* Return success */
  return RSI_SUCCESS;
}

rsi_uCmdRsp *rsi_parse_response(uint8 *rsp)
{
  rsi_uCmdRsp             *temp_uCmdRspPtr = NULL;
  uint8                   temp_rspCode;
  uint16                  temp_status;
  uint8                   *descPtr = rsp ;
  uint8                   *payloadPtr = rsp + RSI_FRAME_DESC_LEN;
  uint8 ch;
#ifdef RSI_DEBUG_PRINT
  uint8 i;
#endif

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL13,"Recieved Packet PRINT3 \n");
#endif
  /* Check whether it is any rxpkt or just a status indication */
#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL13,"Recieved Packet PRINT4 \n");
#endif
  descPtr = rsp; 

  /* Retrieve response code from the received packet */
  temp_status  = rsi_bytes2R_to_uint16(descPtr + RSI_STATUS_OFFSET);
  temp_rspCode = rsi_bytes2R_to_uint16(descPtr + RSI_RSP_TYPE_OFFSET);

#ifdef RSI_DEBUG_PRINT
  for(i=0;i<16;i++)
  {
    RSI_DPRINT(RSI_PL13,"received rspcode: 0x%x \n",descPtr[i]);

  }
#endif    

  if(temp_rspCode)
  {
#ifdef RSI_DEBUG_PRINT
    RSI_DPRINT(RSI_PL13,"received status : 0x%x \n",temp_status);
    RSI_DPRINT(RSI_PL13,"received rspcode: 0x%x \n",temp_rspCode);
#endif    
  }

  rsi_uint16_to_2bytes((payloadPtr - 2), temp_status);
  rsi_uint16_to_2bytes((payloadPtr - 4), temp_rspCode);

  temp_uCmdRspPtr = (rsi_uCmdRsp *)(payloadPtr - 4);

  return temp_uCmdRspPtr;
}

/*=================================================*/
/**
 *@fn           int16 rsi_register_interrupt_irq(uint8 mask)
 * @brief       Sends the register interrupt irq to the Wi-Fi module via SPI
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              SPI:
 *              -1 = SPI busy / Timeout
 *              -2 = SPI Failure
 *              -3 = BUFFER FULL
 *              0  = SUCCESS
 *              UART/USB/USB-CDC:
 *              -2 = Command issue failed
 *              0  = SUCCESS
 * @section description 
 * This API is used to register the interrupt irq.
 */
int16 rsi_register_interrupt_irq(void)
{
  int16     retval;
  /* set unblock interrupt frame */  
  uint8      rsi_frameRegisterInterruptIrq[RSI_BYTES_3] = {0x01,  0x40, 0xEE};


#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL3,"\r\n\nunblocking interrupt");
#endif
  retval = rsi_execute_cmd((uint8 *)rsi_frameRegisterInterruptIrq, NULL, 0);
  return retval;
}
#else

int32 get_rsi_intf_name( uint8 *intf_name )
{
	FILE *fptr;
	uint8 str[100];
	system("ifconfig -a > ifconfig.txt");
	fptr = fopen("ifconfig.txt", "r");

	if( fptr == NULL )
	{
		perror("fopen");
		return -1;
	}

	while(fgets( str, 100, fptr))
	{
		if( strstr(str, "rpine" ) )
		{
			memcpy(intf_name, str, 6);
			fclose(fptr);	
			system("rm -rf ifconfig.txt");
			return 0;
		}
	}

	fclose(fptr);	
	system("rm -rf ifconfig.txt");
	return -1;

}
#endif

int16 rsi_start_ap()
{
  int16           retval     =  0;
#ifdef ONEBOX_EMB
  uint8   response_type,buffer[1000];  


  printf("\n WAITING FOR  CARD READY \n");
  
  while(!rsi_app_cb.pkt_pending);
  retval = rsi_frame_read(rsi_app_cb.read_packet_buffer);                                                            
  rsi_app_cb.uCmdRspFrame = rsi_parse_response(rsi_app_cb.read_packet_buffer);
  if(retval == 0)
  {
    response_type           = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->rspCode);
    rsi_app_cb.error_code   = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->status);
  }
  if(rsi_app_cb.error_code || (response_type != 0x89))
  {
    printf("\n RESPONSE : CARD READY  Failed \n");
    return rsi_app_cb.error_code;
  }
  printf("\n CARD READY RECEIVED\n");

  printf("\n REQUEST  : opermode command given \n");
  rsi_uOperMode *oper=(rsi_uOperMode *)buffer;
  rsi_uint32_to_4bytes(oper->operModeFrameSnd.oper_mode, 6);
  rsi_uint32_to_4bytes(oper->operModeFrameSnd.feature_bit_map, RSI_FEATURE_BIT_MAP);
  rsi_uint32_to_4bytes(oper->operModeFrameSnd.tcp_ip_feature_bit_map, RSI_TCP_IP_FEATURE_BIT_MAP);
  rsi_uint32_to_4bytes(oper->operModeFrameSnd.custom_feature_bit_map, RSI_CUSTOM_FEATURE_BIT_MAP);
  retval = rsi_execute_cmd((uint8 *)rsi_frameCmdOperMode,(uint8 *)oper, sizeof(rsi_uOperMode));
  while(!rsi_app_cb.pkt_pending);
  retval = rsi_frame_read(rsi_app_cb.read_packet_buffer);                                                            
  rsi_app_cb.uCmdRspFrame = rsi_parse_response(rsi_app_cb.read_packet_buffer);
  if(retval == 0)
  {
    response_type           = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->rspCode);
    rsi_app_cb.error_code   = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->status);
  }
  if(rsi_app_cb.error_code)
  {
    printf("\n RESPONSE : Opermode Failed ,ERROR code : 0x%x \n",rsi_app_cb.error_code);
    return rsi_app_cb.error_code;
  }
  printf("\n RESPONSE : Opermode Success \n");



  printf("\n REQUEST  : Band command given \n");
  rsi_uBand *band =(rsi_uBand *)buffer;
  band->bandFrameSnd.bandVal  =  RSI_BAND;
  retval = rsi_execute_cmd((uint8 *)rsi_frameCmdBand,(uint8 *)band, sizeof(rsi_uBand));
  while(!rsi_app_cb.pkt_pending);
  retval = rsi_frame_read(rsi_app_cb.read_packet_buffer);                                                            
  rsi_app_cb.uCmdRspFrame = rsi_parse_response(rsi_app_cb.read_packet_buffer);
  if(retval == 0)
  {
    response_type           = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->rspCode);
    rsi_app_cb.error_code   = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->status);
  }
  if(rsi_app_cb.error_code)
  {
    printf("\n RESPONSE : Band Failed , ERROR Code : 0x%x \n",rsi_app_cb.error_code);
    return rsi_app_cb.error_code;
  }
  printf("\n RESPONSE : Band Success \n");



  printf("\n REQUEST  : INIT command given \n");

  retval = rsi_execute_cmd((uint8 *)rsi_frameCmdInit,NULL,0);
  while(!rsi_app_cb.pkt_pending);
  retval = rsi_frame_read(rsi_app_cb.read_packet_buffer);                                                            
  rsi_app_cb.uCmdRspFrame = rsi_parse_response(rsi_app_cb.read_packet_buffer);
  if(retval == 0)
  {
    response_type           = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->rspCode);
    rsi_app_cb.error_code   = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->status);
  }
  if(rsi_app_cb.error_code)
  {
    printf("\n RESPONSE : Init Failed , ERROR Code : 0x%x \n",rsi_app_cb.error_code);
    return rsi_app_cb.error_code;
  }
  memcpy(rsi_app_cb.mac_addr, rsi_app_cb.uCmdRspFrame->uCmdRspPayLoad.initResponse.macAddress, 6);

  printf("\n RESPONSE : Init Success \n");


  printf("\n REQUEST  : Access point configuration command given \n");
  rsi_apconfig *ap = (rsi_apconfig *)buffer;
  rsi_uint16_to_2bytes(ap->dtim_period,RSI_DTIM_PERIOD);
  rsi_uint16_to_2bytes(ap->beacon_interval,RSI_BEACON_INTERVAL);
  rsi_uint16_to_2bytes(ap->channel_no,RSI_AP_CHANNEL_NUM);
  rsi_uint16_to_2bytes(ap->max_sta_support,MAX_NO_OF_CLIENTS);
  strcpy((char *)ap->ssid,RSI_AP_SSID);
  ap->security_type = RSI_SECURITY_TYPE;
  ap->ap_keepalive_type = AP_KEEPALIVE_TYPE;
  ap->ap_keepalive_period = AP_KEEPALIVE_PERIOD;
  ap->encryp_mode = RSI_ENCRYPTION_MODE;
  strcpy((char *)ap->psk,RSI_PSK); 
  retval = rsi_execute_cmd((uint8 *)rsi_frameCmdAPconf, (uint8 *)ap, sizeof(rsi_apconfig));

  while(!rsi_app_cb.pkt_pending);
  retval = rsi_frame_read(rsi_app_cb.read_packet_buffer);                                                            
  rsi_app_cb.uCmdRspFrame = rsi_parse_response(rsi_app_cb.read_packet_buffer);
  if(retval == 0)
  {
    response_type           = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->rspCode);
    rsi_app_cb.error_code   = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->status);
  }
  if(rsi_app_cb.error_code)
  {
    printf("\n RESPONSE :   Access point configuration Failed , ERROR Code : 0x%x \n",rsi_app_cb.error_code);
    return rsi_app_cb.error_code;
  }

  printf("\n RESPONSE :  Access point configuration passed \n");


  printf("\n REQUEST  : Join command given \n");
  rsi_uJoin *join = (rsi_uJoin *)buffer;            

  join->joinFrameSnd.securityType = RSI_SECURITY_MODE;
  join->joinFrameSnd.dataRate     = RSI_DATA_RATE;
  join->joinFrameSnd.powerLevel   = RSI_POWER_LEVEL;
  join->joinFrameSnd.join_feature_bitmap = RSI_JOIN_FEAT_BIT_MAP;
  join->joinFrameSnd.ssid_len     = (sizeof(RSI_AP_SSID) - 1);
  rsi_uint32_to_4bytes(join->joinFrameSnd.listen_interval, RSI_LISTEN_INTERVAL);

  if(SEND_PSK_IN_JOIN)
  {
    strcpy((char *)join->joinFrameSnd.psk, RSI_PSK);
  }
  strcpy((char *)join->joinFrameSnd.ssid, RSI_AP_SSID);


  retval = rsi_execute_cmd((uint8 *)rsi_frameCmdJoin,(uint8 *)join,sizeof(rsi_uJoin));

  while(!rsi_app_cb.pkt_pending);
  retval = rsi_frame_read(rsi_app_cb.read_packet_buffer);                                                            
  rsi_app_cb.uCmdRspFrame = rsi_parse_response(rsi_app_cb.read_packet_buffer);
  if(retval == 0)
  {
    response_type           = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->rspCode);
    rsi_app_cb.error_code   = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->status);
  }
  if(rsi_app_cb.error_code)
  {
    printf("\n RESPONSE :   Join Failed , ERROR Code : 0x%x \n",rsi_app_cb.error_code);
    return rsi_app_cb.error_code;
  }

  printf("\n RESPONSE : Join passed \n");

  retval = rsi_update_info(UPDATE_JOIN_DONE);
  system("ifconfig rsi_wlan0 192.168.2.1");
  system("/sbin/service dhcpd start");
  system("ifconfig rsi_wlan0 inet6 add 2001:0db8:0:f101::1/64");
  system("/etc/init.d/mdns start");
#else
	printf("In %s %d\n", __func__, __LINE__);
	system("sh onebox_insert.sh");
	sleep(2);
	system("./onebox_util rpine0 enable_protocol 1");
	sleep(1);
	system("./onebox_util rpine0 create_vap wifi0 ap");
	printf("In %s %d\n", __func__, __LINE__);
#endif
  return 0;
}

void main()
{

  WACPlatformParameters_t *WACPlatformParameters_ptr;
  int16_t retval = RSI_SUCCESS;
  void *ret_val;
#ifdef ONEBOX_EMB
  retval = rsi_nl_socket_init();
  if (retval != RSI_SUCCESS) {
    return RSI_FAIL;
  }

  rsi_app_cb.pkt_pending = 0;

  rsi_fill_genl_nl_hdrs_for_cmd();
  if(pthread_mutex_init(&rsi_linux_app_cb.mutex1, NULL) != 0) {
    return RSI_FAIL;
  }

  if(pthread_create(&wac_rcv_thd, NULL, wac_read_pkt_thread, 0)) {
    return RSI_FAIL;
  }
  signal(SIGINT, sigint_handler);
  rsi_register_interrupt_irq();
  rsi_app_cb.expected_response = 0x89;
#endif
  retval = rsi_start_ap();
  if(retval != RSI_SUCCESS)
  {
    printf("RETVAL not success in %s %d\n", __func__, __LINE__);
    return ;
    
  }

  signal(SIGINT, sigint_handler);
#ifndef ONEBOX_EMB
	uint8_t intf_name[7] = {'\0'};
	driver_param_t driver_params;
	int32_t sfd;
	struct iwreq wrq;
	memset(&wrq, 0, sizeof(struct iwreq));
	memset(&driver_params, 0, sizeof(driver_param_t));

	if(get_rsi_intf_name(intf_name) < 0 )
	{
		printf("Unable to Find rpine interface.\n");
		return ;
	}
	printf("Interface name is %s\n", intf_name);

	if ((sfd = socket (AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		printf("socket creation error\n");
		return ;
	}
	wrq.u.data.flags = DRV_PARAMS;
	strncpy (wrq.ifr_name, intf_name, 6);
	wrq.u.data.pointer = &driver_params;
	wrq.u.data.length = sizeof(driver_param_t); 
	
	if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
	{
		perror("ioctl");
		return ;
	}

//	snprintf(fw_version,10,"%d.%d.%d.%d.%d", *(uint16_t *)(&driver_params.fw_ver[0]),
//										  *(uint16_t *)(&driver_params.fw_ver[2]),
//										  				 driver_params.fw_ver[4],
//														 driver_params.fw_ver[5],
//														 driver_params.fw_ver[0]);
	printf("Firmware Version is %s\n", fw_version);
#endif

  strcpy(name, MFI_DEVICE_TO_CONFIGURE);
  WAC_incontext.platformParams  = malloc(sizeof(WACPlatformParameters_t));
  memset(WAC_incontext.platformParams,0,sizeof(WACPlatformParameters_t));
  WACPlatformParameters_ptr = WAC_incontext.platformParams;
  //! Copy MAC address of the module
#ifdef ONEBOX_EMB
	memcpy(&WACPlatformParameters_ptr->macAddress[0],rsi_app_cb.mac_addr,6);
#else
	memcpy(&WACPlatformParameters_ptr->macAddress[0], driver_params.mac_addr ,6);
#endif
  //! Unconfigured Accesory
  WACPlatformParameters_ptr->isUnconfigured = 1;
  //! Doesnot support Airplay
  WACPlatformParameters_ptr->supportsAirPlay = 0;
  //! Doesnot support Airprint
  WACPlatformParameters_ptr->supportsAirPrint = 0;
  //! Supports 2.4 GHz
  WACPlatformParameters_ptr->supports2_4GHzWiFi = 1;
  if (driver_params.module_type)
  	WACPlatformParameters_ptr->supports5GHzWiFi = 1;
  //! Supports Wake on Wireless
  WACPlatformParameters_ptr->supportsWakeOnWireless = 1;
  //! Copy Fw version
  WACPlatformParameters_ptr->firmwareRevision = &fw_version;
  WACPlatformParameters_ptr->hardwareRevision = &hw_version;
  WACPlatformParameters_ptr->serialNumber = &serial_number;
  WACPlatformParameters_ptr->name = &name;
  WACPlatformParameters_ptr->model = &model;
  WACPlatformParameters_ptr->manufacturer = &manufacturer;
  WACPlatformParameters_ptr->numSupportedExternalAccessoryProtocols = 1;
  WACPlatformParameters_ptr->supportedExternalAccessoryProtocols =  support;

  WACServerStart(&WAC_incontext,WAC_start_handler);

#ifdef ONEBOX_EMB
  pthread_join(wac_rcv_thd,&retval);
#endif
  //pthread_join(WAC_incontext.serverEngineThread,&retval);
  pthread_join(WAC_incontext.serverEngineThread,&ret_val);
  sigint_handler(1);

}

void sigint_handler(int sig)
{
	printf("\nkilling process 0x%x\n",getpid());
#ifdef ONEBOX_EMB
	rsi_linux_app_cb_t *linux_app_cbPtr = &rsi_linux_app_cb;
	pthread_cancel(wac_rcv_thd);
	close(linux_app_cbPtr->ioctl_sd);
	close(linux_app_cbPtr->nl_sd);
#else
	//system("sh remove_all.sh");
#endif
	exit(0);
}








