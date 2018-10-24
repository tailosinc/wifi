#include "PlatformSoftwareAccessPoint.h"

#include "rsi_global.h"
#include "rsi_app.h"
#include "rsi_nl_app.h"

#ifdef ONEBOX_EMB
extern rsi_app_cb_t rsi_app_cb;
extern pthread_t wac_rcv_thd;
extern rsi_linux_app_cb_t   rsi_linux_app_cb;
extern void * wac_read_pkt_thread(void * arg );
#else
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>
#define ONEBOX_HOST_IOCTL     SIOCIWLASTPRIV - 0x0B
#define ADD_APPLE_IE 46
#endif

OSStatus PlatformSoftwareAccessPointStart( const uint8_t *inIE, size_t inIELen )
{
#ifdef ONEBOX_EMB
  uint8   rsi_frameMfiAddBeaconIe[RSI_BYTES_3] = {0x00, 0x40, 0xB5},response_type;
   uint8   rsi_frameMfiIapInit[RSI_BYTES_3] = {0x00, 0x40, 0xB7};
  int16           retval     =  0;
  uint8 buffer[1000];
  uint8  *ie = NULL;
  ie = malloc(inIELen+1);
  ie[0]  = inIELen;
  memcpy(&ie[1], inIE, inIELen);

  rsi_uint16_to_2bytes(rsi_frameMfiAddBeaconIe, (((inIELen + 1) & 0x0FFF) | 0x4000));
  printf("\n REQUEST  : PlatformSoftwareAccessPointStart  \n ");
  retval = rsi_execute_cmd((uint8 *)rsi_frameMfiAddBeaconIe, 
      ie,inIELen + 1);

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
    printf("\n RESPONSE : PlatformSoftwareAccessPointStart Failed , ERROR code : 0x%x\n",rsi_app_cb.error_code);
    return rsi_app_cb.error_code;
  }
#else
	int32_t sfd;
	struct iwreq wrq;
	memset(&wrq, 0, sizeof(struct iwreq));
	wrq.u.data.flags = ADD_APPLE_IE;
	strncpy (wrq.ifr_name, "rpine0", 6);
	wrq.u.data.pointer = inIE;
	wrq.u.data.length = inIELen; 

	if ((sfd = socket (AF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("socket creation error\n");
		return ;
	}
	
	if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
	{
		perror("ioctl");
		return ;
	}

	if (close(sfd) < 0) {
		printf("ERROR CLOSING THE SOCKET\n\n");
	}
#ifndef ONEBOX_NL80211
	system("./wpa_supplicant -i wifi0 -D bsd -c wpa_supplicant_open.conf -ddddt > log &");
#else
	system("./hostapd hostapd_open.conf -ddddt > log &");
#endif
	sleep(5);
	system("sh dhcp_server.sh wifi0");

#endif
   printf("\n RESPONSE : PlatformSoftwareAccessPointStart Success \n");
   return 0;
}

OSStatus PlatformSoftwareAccessPointStop( void )
{
#ifdef ONEBOX_EMB
  int16  retval     =  0;
  int rc1 = 0;
  uint8  response_type;
  rsi_linux_app_cb_t *linux_app_cbPtr = &rsi_linux_app_cb;

  system("/etc/init.d/mdns stop");
  system("/etc/init.d/mdns stop");
  system("sudo netstat -ap | grep :5353");
  sleep(5);
  printf("\n  REQUEST : PlatformSoftwareAccessPointStop \n");
  retval = rsi_execute_cmd((uint8 *)rsi_frameCmdReset, 
      NULL,0);
  pthread_cancel(wac_rcv_thd);
  close(linux_app_cbPtr->ioctl_sd);
  close(linux_app_cbPtr->nl_sd);
  sleep(5);
  /* Open a socket for issueing ioctls */
  if ((linux_app_cbPtr->ioctl_sd = socket (AF_INET, SOCK_DGRAM, 0)) < 0)
  {
#ifdef RSI_DEBUG_PRINT
    RSI_DPRINT (RSI_PL3,"socket creation error\n");
#endif
    return 2;
  }

  retval = rsi_nl_socket_init();
  if (retval == -1) {
    return 2;
  }

  rsi_fill_genl_nl_hdrs_for_cmd();
  if( (rc1 = pthread_create( &wac_rcv_thd, NULL, wac_read_pkt_thread, (void*)NULL)) )
  {
    return -1;
  }
  retval = rsi_register_interrupt_irq();
  rsi_app_cb.expected_response = rsi_frameCmdReset[2];
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
    return rsi_app_cb.error_code;
    printf("\n RESPONSE :  PlatformSoftwareAccessPointStop Failed \n");
  }
#else

  system("/etc/init.d/mdns stop");
  //system("/etc/init.d/mdns stop");
  system("sudo netstat -ap | grep :5353");
  //sleep(5);
  printf("\n  REQUEST : PlatformSoftwareAccessPointStop \n");
  system("killall -9 wpa_supplicant");
  system("rm -rf /var/run/wpa_supplicant");
  system("./onebox_util rpine0 delete_vap wifi0");
#endif

  printf("\n RESPONSE :  PlatformSoftwareAccessPointStop Success \n");
  return 0;
}


