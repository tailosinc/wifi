#include <stdio.h>
#include <stdlib.h>
#include "header.h"

int16 rsi_start_station()
{
  system("sh start_sta.sh");
  sleep(5);
  system("dhclient -r");
  system("dhclient -v wifi0");
  printf("\n !!!! Connected to Given AP!!!! \n");//Have to check the status for connection
  return 0;
}
int main()
{
  int16_t retval = RSI_SUCCESS;
  retval = rsi_start_station();
  if(retval != RSI_SUCCESS)
  {
    return ;
    
  }
#if IAP_CHIP == 1
  IAP_initialise();
#endif
  printf("\n !!!! Home kit Accessory Started !!!! \n");
  system("/etc/init.d/mdns start");
  printf("Started the MDNS \n");
  retval = Homekit_main(); 
  return 0;
}

