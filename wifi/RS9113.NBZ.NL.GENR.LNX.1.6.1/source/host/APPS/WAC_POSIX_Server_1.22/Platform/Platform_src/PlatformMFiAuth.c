#include "PlatformMFiAuth.h"
#include "rsi_global.h"
#include "rsi_app.h"
#include "rsi_nl_app.h"

#ifdef ONEBOX_EMB
extern rsi_app_cb_t rsi_app_cb;
#else
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>
#define ONEBOX_HOST_IOCTL     SIOCIWLASTPRIV - 0x0B
#define IAP_INIT 47
#define IAP_MFI_CHALLENGE 48
#define IAP_READ_CERTIFICATE 49
#endif

OSStatus PlatformMFiAuthInitialize( void )
{
#ifdef ONEBOX_EMB
  int16           retval     =  0;
  uint8   rsi_frameMfiIapInit[RSI_BYTES_3] = {0x00, 0x40, 0xB7},response_type,buffer[1000];  
  printf("\n REQUEST  : PlatformMFiAuthInitialize  \n ");
  retval = rsi_execute_cmd((uint8 *)rsi_frameMfiIapInit, 
      NULL,0);
  while(!rsi_app_cb.pkt_pending);
  retval = rsi_frame_read(rsi_app_cb.read_packet_buffer);                                                            
  rsi_app_cb.uCmdRspFrame = rsi_parse_response(rsi_app_cb.read_packet_buffer);
  if(retval == 0)
  {
    response_type           = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->rspCode);
    rsi_app_cb.error_code   = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->status);
  }

  if(rsi_app_cb.error_code || (response_type != 0xB7))
  {
    printf("\n RESPONSE : PlatformMFiAuthInitialize Failed , ERROR code : 0x%x\n",rsi_app_cb.error_code);
    return -1;
  }
  printf("\n RESPONSE : PlatformMFiAuthInitialize Success \n");
  return rsi_app_cb.error_code;
#else
	int32_t sfd;
	struct iwreq wrq;

	//rsi_iap_init has to be called here
	memset(&wrq, 0, sizeof(struct iwreq));
	if ((sfd = socket (AF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("socket creation error\n");
		return -1;
	}
	wrq.u.data.flags = IAP_INIT;
	strncpy (wrq.ifr_name, "rpine0", 6);
	wrq.u.data.length = 0; 
	
	if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
	{
		perror("ioctl");
		return -1;
	}

	if (close(sfd) < 0) {
		printf("ERROR CLOSING THE SOCKET\n\n");
	}

	printf("\n RESPONSE : PlatformMFiAuthInitialize Success \n");
	return 0;
#endif
}

void PlatformMFiAuthFinalize( void )
{
  printf("\n PlatformMFiAuthFinalize \n ");
  system("/sbin/service dhcpd stop");
  //system("/etc/init.d/mdns stop");
#ifdef ONEBOX_EMB
  system("dhclient -r rsi_wlan0");
#else
  //system("dhclient -r wifi0");
#endif
  
  printf("\n ################################################## \n");

  printf("\n ##########ACCESSORY CONFIGURATION SUCCESS ######## \n");

  printf("\n ################################################## \n");

}

OSStatus PlatformMFiAuthCopyCertificate( uint8_t **outCertificatePtr, size_t *outCertificateLen )
{
#ifdef ONEBOX_EMB
  uint8   rsi_frameMfiRequestCert[RSI_BYTES_3] = {0x00, 0x40, 0xB6},response_type;
  int16           retval     =  0,cert_length = 0;

  
  printf("\n REQUEST  : PlatformMFiAuthCopyCertificate \n ");
  retval = rsi_execute_cmd((uint8 *)rsi_frameMfiRequestCert, 
      NULL,0);

  memset(rsi_app_cb.read_packet_buffer,0,16);
  rsi_app_cb.pkt_pending = 0;
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
    printf("\n RESPONSE : PlatformMFiAuthCopyCertificate Failed , ERROR code : 0x%x\n",rsi_app_cb.error_code);
    return -1;
  }
  printf("\n RESPONSE : PlatformMFiAuthCopyCertificate Success \n");

  cert_length = rsi_bytes2R_to_uint16(&rsi_app_cb.read_packet_buffer[16]);
  *outCertificatePtr = malloc(cert_length);
  *outCertificateLen = cert_length; 
  memcpy(*outCertificatePtr,&rsi_app_cb.read_packet_buffer[18],cert_length);

  return rsi_app_cb.error_code;
#else
	int16 cert_length = 0;
	int32_t sfd;
	struct iwreq wrq;

	memset(&wrq, 0, sizeof(struct iwreq));
	wrq.u.data.length = 908;
	wrq.u.data.flags = IAP_READ_CERTIFICATE;
	wrq.u.data.pointer = malloc(920);
	strncpy (wrq.ifr_name, "rpine0", 6);

	if ((sfd = socket (AF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("socket creation error\n");
		return -1;
	}
	
	if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) {
		perror("ioctl");
		return -1;
	}

	cert_length = wrq.u.data.length;

	if (cert_length)
		*outCertificatePtr = malloc(cert_length);
	*outCertificateLen = cert_length; 

	memcpy(*outCertificatePtr, wrq.u.data.pointer, cert_length);
	
	if (close(sfd) < 0) {
		printf("ERROR CLOSING THE SOCKET\n\n");
	}
	free(wrq.u.data.pointer);

	return 0;
#endif
}

OSStatus PlatformMFiAuthCreateSignature( const void *inDigestPtr,
                                         size_t     inDigestLen,
                                         uint8_t    **outSignaturePtr,
                                         size_t     *outSignatureLen )


{
#ifdef ONEBOX_EMB
  int16           retval     =  0,signature_length;
  uint8  *digest = NULL;
  
  uint8   rsi_frameMfiCreateSignature[RSI_BYTES_3] = {0x00, 0x40, 0xB8},response_type;

  digest = malloc(inDigestLen + 4);
  memcpy(&digest[4],inDigestPtr,inDigestLen );
	
  rsi_uint32_to_4bytes(digest, (inDigestLen));
  rsi_uint16_to_2bytes(rsi_frameMfiCreateSignature, (((inDigestLen + 4) & 0x0FFF) | 0x4000));
  printf("\n REQUEST  : PlatformMFiAuthCreateSignature \n ");

	
  retval = rsi_execute_cmd((uint8 *)rsi_frameMfiCreateSignature, 
      (uint8*)digest,(inDigestLen + 4));
  while(!rsi_app_cb.pkt_pending);
  retval = rsi_frame_read(rsi_app_cb.read_packet_buffer);                                                            
  rsi_app_cb.uCmdRspFrame = rsi_parse_response(rsi_app_cb.read_packet_buffer);
  if(retval == 0)
  {
    response_type           = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->rspCode);
    rsi_app_cb.error_code   = rsi_bytes2R_to_uint16(rsi_app_cb.uCmdRspFrame->status);
  }
  signature_length = rsi_bytes2R_to_uint16(&rsi_app_cb.read_packet_buffer[16]);
  *outSignaturePtr = malloc(signature_length);
  *outSignatureLen = signature_length; 
  memcpy(*outSignaturePtr,&rsi_app_cb.read_packet_buffer[18],signature_length);
  
  if(rsi_app_cb.error_code)
  {
    printf("\n RESPONSE : PlatformMFiAuthCreateSignature Failed , ERROR code : 0x%x\n",rsi_app_cb.error_code);
    return -1;
  }
  printf("\n RESPONSE : PlatformMFiAuthCreateSignature Success \n");
  return rsi_app_cb.error_code;
#else
	int16 retval = 0, signature_length;
	uint8 *digest = NULL;
	int32_t sfd;
	struct iwreq wrq;
	char *challenge_signature;

	memset(&wrq, 0, sizeof(struct iwreq));
	wrq.u.data.flags = IAP_MFI_CHALLENGE;
	strncpy (wrq.ifr_name, "rpine0", 6);
	wrq.u.data.length = inDigestLen;
	challenge_signature = malloc(128 + inDigestLen);
	memcpy(challenge_signature, inDigestPtr, inDigestLen);

	wrq.u.data.pointer = challenge_signature;
	
	if ((sfd = socket (AF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("socket creation error\n");
		return -1;
	}
	
	if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) {
		perror("ioctl");
		return -1;
	}

	signature_length = 128;
	*outSignatureLen = signature_length;
	*outSignaturePtr = malloc(signature_length);
	memcpy(*outSignaturePtr, wrq.u.data.pointer, signature_length);

	if (close(sfd) < 0) {
		printf("ERROR CLOSING THE SOCKET\n\n");
	}
	free(challenge_signature);

	printf("\n RESPONSE : PlatformMFiAuthCreateSignature Success \n");
	return 0;

#endif
}


