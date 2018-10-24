#include "PlatformRandomNumber.h"

#include <stdlib.h>

OSStatus PlatformCryptoStrongRandomBytes( void *inBuffer, size_t inByteCount )
{
  int n,key;
  char *inbuff = NULL;
  inbuff = (char *)inBuffer;
  printf("\n PlatformCryptoStrongRandomBytes \n ");


  for (int n = 0;n < inByteCount;n++)
  {            
    key = rand();          
    inbuff[n] = key;
  }
  return 0;
}

