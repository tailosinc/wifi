#include "PlatformBonjour.h"

OSStatus PlatformInitializemDNSResponder( void )
{
  int32_t status = 0;
  system("/etc/init.d/mdns start");
  return status;
}


OSStatus PlatformMayStopmDNSResponder( void )
{
  //printf("\n Stopping mDNS Responder\n");
  //system("/etc/init.d/mdns stop");
  return 0;
}
