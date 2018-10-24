
#include "PlatformApplyConfiguration.h"
#include "rsi_global.h"
#include "rsi_app.h"
#include "rsi_nl_app.h"

#ifdef ONEBOX_EMB
extern rsi_app_cb_t rsi_app_cb;
#endif

OSStatus PlatformJoinDestinationWiFiNetwork( const char * const inSSID, const uint8_t * const inWiFiPSK, size_t inWiFiPSKLen )
{
#ifdef ONEBOX_EMB
  uint8  response_type;
  int16  retval     =  0;
  uint8 buffer[1000] ={0};

  rsi_uOperMode *oper = NULL;
  rsi_uJoin *join = NULL;            
  rsi_uBand *band = NULL;
  rsi_uScan *scan = NULL;

  system("/etc/init.d/mdns start");
  printf("\n JOINING THE DESTINATION NETWORK \n");

  printf("\n Changing the module from Accesspoint to station mode \n");
  printf("\n REQUEST  : Setting opermode to station \n");
  oper=(rsi_uOperMode *)buffer;
  rsi_uint32_to_4bytes(oper->operModeFrameSnd.oper_mode, 0);
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





  //! Setting Band to 2.4GHZ
  printf("\n REQUEST  : Band command given \n");
  band =(rsi_uBand *)buffer;
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
  printf("\n RESPONSE : Band passed \n");



  //! Setting INIT command
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

 do{
  //! Sending scan request 
  printf("\n REQUEST  : Scanning for Accesspoint  %s \n",inSSID);
  scan = (rsi_uScan *)buffer;
  scan->scanFrameSnd.channel[0] = 0;
  strcpy((char *)scan->scanFrameSnd.ssid, inSSID);
  scan->scanFrameSnd.scan_feature_bitmap = 0;
  retval = rsi_execute_cmd((uint8 *)rsi_frameCmdScan,(uint8 *)scan, sizeof(rsi_uScan));
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
    printf("\n RESPONSE : SCAN  Failed  \n");
  }
 }while(rsi_app_cb.error_code);
  printf("\n RESPONSE : Scan Response received \n");
  //! Joining to the network
  printf("\n REQUEST  : Joining to the access point %s \n",inSSID);
  join = (rsi_uJoin *)buffer;            
  join->joinFrameSnd.securityType = RSI_SECURITY_MODE;
  join->joinFrameSnd.dataRate     = RSI_DATA_RATE;
  join->joinFrameSnd.powerLevel   = RSI_POWER_LEVEL;
  join->joinFrameSnd.join_feature_bitmap = RSI_JOIN_FEAT_BIT_MAP;
  join->joinFrameSnd.ssid_len     = strlen(inSSID);
  rsi_uint32_to_4bytes(join->joinFrameSnd.listen_interval, RSI_LISTEN_INTERVAL);
  if(inWiFiPSKLen)
  {
    strcpy((char *)join->joinFrameSnd.psk,inWiFiPSK);
  }
  strcpy((char *)join->joinFrameSnd.ssid, inSSID);
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
    printf("\n RESPONSE : Join Failed \n");
    return rsi_app_cb.error_code;

  }
    printf("\n RESPONSE : Join Success \n");
  retval = rsi_update_info(UPDATE_JOIN_DONE);
  system("dhclient -r rsi_wlan0");
  system("dhclient -v rsi_wlan0");
#else
	system("sh remove_all.sh");
	sleep(1);
	system("sh onebox_insert.sh");
	sleep(1);
  	system("/etc/init.d/mdns start");
	system("./onebox_util rpine0 enable_protocol 1");
	sleep(1);
	printf("Join to the destination Network\n");
	system("./onebox_util rpine0 set_scan_type 3");

	system("./onebox_util rpine0 create_vap wifi0 sta sw_bmiss");
	sleep(1);
  //system("ifconfig rsi_wlan0 192.168.0.11");
  FILE *fp;
  fp = fopen("./mfi_sta_settings.conf", "w+");
  fprintf(fp, "ctrl_interface=/var/run/wpa_supplicant\n");
  fprintf(fp, "update_config=1\n");
  fprintf(fp, "uuid=12345678-9abc-def0-1234-56789abcdef0\n");
  fprintf(fp, "device_name=RS9113_n-Link\n");
  fprintf(fp, "manufacturer=Redpine Signals INC\n");
  fprintf(fp, "model_name=OneBox-Mobile\n");
  fprintf(fp, "model_number=9113\n");
  fprintf(fp, "serial_number=02\n");
  fprintf(fp, "device_type=1-0050F204-1\n");
  fprintf(fp, "os_version=01020300\n");
  fprintf(fp, "config_methods=display push_button keypad\n");
  fprintf(fp, "ap_scan=1\n");
  fprintf(fp, "\n");
  fprintf(fp, "network={\n");
  fprintf(fp, "ssid=\"%s\"\n", inSSID);
  if (inWiFiPSKLen) {
	fprintf(fp, "pairwise=CCMP TKIP\n");
	fprintf(fp, "group=CCMP TKIP\n");
	fprintf(fp, "key_mgmt=WPA-PSK\n");
	fprintf(fp, "psk=\"%s\"\n", inWiFiPSK);
	fprintf(fp, "proto=WPA2 WPA\n");
        fprintf(fp, "priority=2\n");
        fprintf(fp, "}");

 	fprintf(fp, "\nnetwork={\n");
 	fprintf(fp, "ssid=\"%s\"\n", inSSID);
	fprintf(fp, "wep_key0=%s\n", inWiFiPSK);
	fprintf(fp, "wep_tx_keyidx=1\n");
	fprintf(fp, "key_mgmt=NONE\n");
         
  } else {
	fprintf(fp, "key_mgmt=NONE\n");
  }
  fprintf(fp, "priority=2\n");
  fprintf(fp, "}");
  fprintf(fp, "\n");
  fclose(fp);
#ifndef ONEBOX_NL80211
  system("./wpa_supplicant -i wifi0 -D bsd -c mfi_sta_settings.conf -ddddt > log &");
#else
  system("./wpa_supplicant -i wifi0 -D nl80211 -c mfi_sta_settings.conf -ddddt > log &");
#endif
  sleep(3);
  system("dhclient -r wifi0");
  system("dhclient -v wifi0");
#endif
  return RSI_SUCCESS;
}

OSStatus PlatformApplyName( const char * const inName )
{

  return RSI_SUCCESS;

}

OSStatus PlatformApplyAirPlayPlayPassword( const char * const inPlayPassword )
{

  return RSI_SUCCESS;
}


