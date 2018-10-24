#include<stdio.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>
#include "header.h"
#include"Configuration.h"

#if IAP_CHIP == 1
int IAP_initialise()
{
	int32_t sfd;
	struct iwreq wrq;
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
	printf("IAP CHIP INITIALISATION DONE");
	return 0;
}

int MFiAuthCopyCertificate( uint8_t **outCertificatePtr, size_t *outCertificateLen )
{

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
}
int MFiAuthCreateSignature( const void *inDigestPtr,
                                         size_t     inDigestLen,
                                         uint8_t    **outSignaturePtr,
                                         size_t     *outSignatureLen )


{
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

}
#endif


