#include <stdio.h>
#include <stdlib.h>
#include "Configuration.h"

typedef unsigned char           uint8;
typedef unsigned short          uint16;
typedef unsigned int            uint32;
typedef signed char             int8;
typedef short                   int16;
typedef long                    int32;


#define RSI_SUCCESS                0
#define RSI_FAIL                   1

#if IAP_CHIP == 1
#define ONEBOX_HOST_IOCTL     SIOCIWLASTPRIV - 0x0B
#define IAP_INIT 47
#define IAP_MFI_CHALLENGE 48
#define IAP_READ_CERTIFICATE 49
#endif

int Homekit_main();
#if IAP_CHIP == 1
int IAP_initialise(void);
int MFiAuthCopyCertificate( uint8_t **outCertificatePtr, size_t *outCertificateLen );
int MFiAuthCreateSignature( const void *inDigestPtr,
                                         size_t     inDigestLen,
                                         uint8_t    **outSignaturePtr,
                                         size_t     *outSignatureLen );
#endif


