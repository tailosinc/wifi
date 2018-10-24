#include<stdio.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<string.h>
#include<stdlib.h>
#include<netinet/in.h>
#include<pthread.h>
#include "rsi_api_routine.h"
#include <sys/stat.h>

test_params_t test_params;


/**********************************************************************
 * Create Log file with file name as MAC ADDR and card no storing 
 * Calibration Values. 
 * ********************************************************************/
int rsi_create_log(int type, int card_no)
{

  unsigned char file_name[13];
  int i = 0;
  FILE *f_log;
  FILE *f_calib;
  unsigned char ch;
  unsigned char create_log = 0;

  char command[60];

  char token[80];
  FILE *rsi_config = NULL;

  rsi_config = fopen("RSI_Config.txt","r");
  if (rsi_config == NULL)
  {
    printf("Unable to open CONFIG file\n");
    return RSI_STATUS_FAILURE;
  }  

  if (rsi_get_token(rsi_config,"RSI_MAC_LOG",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%x",&create_log);

  if( create_log == 0 )
  {
    return RSI_STATUS_SUCCESS;
  }

  memset(command,'\0',60);

  printf("CREATING LOG FILE\n");
  memset(file_name, '\0' , 13);
  sprintf(file_name, "%x" , test_params.wlan_mac_id);

  for(i = 0; i <2; i++)
  {
    memmove(&file_name[2*(i+1)+i +1],&file_name[2*(i+1) + i], strlen(&file_name[2*(i+1) + i ]));
    file_name[2*(i+1) + i] = '_';
  }
  if ( 5 == type)
  {
    strcat(file_name,"_W_");
  }

  else
  {
    strcat(file_name,"_S_");
  }

  if(card_no)
  {
    sprintf(&file_name[strlen(file_name)],"%d",card_no);	
  }
  if(!( mkdir("rsi_mac_log", S_IRWXU | S_IRWXG )   == -1 ))
  {
    printf("successfully created the directory\n");
  }


  sprintf(command,"sed -e '1300,$d' ../RS9113_RS8111_calib_values.txt >> rsi_mac_log/%s",file_name);



  system(command);
#if 0	
  f_log = fopen(file_name, "w");
  if( NULL == f_log ) 
  {
    printf("WARNING Unable To Create Log File\n");
    return RSI_STATUS_FAILURE;
  }	
  f_calib = fopen("../RS9113_RS8111_calib_values.txt", "r");
  if( NULL == f_calib ) 
  {
    printf("WARNING Unable To Create Log File\n");
    return RSI_STATUS_FAILURE;
  }	
  while(1)
  {
    ch = fgetc(f_calib);

    if( feof(f_calib) )	 
      break;
    else
      fputc(ch,f_log);
  }	
  fclose(f_log);
  fclose(f_calib);
#endif
  return RSI_STATUS_SUCCESS;
}


/**********************************************************************
 * Update next MAC ADDRESS in the file
 * ********************************************************************/

int rsi_save_mac()
{
  FILE *rsi_last_mac, *rsi_current_mac;
  rsi_last_mac = fopen("RSI_LastMac.txt","w");
//  rsi_current_mac = fopen("RSI_CurrentMac.txt","w");
  if (rsi_last_mac == NULL)
  {
    printf("Unable to create LAST MAC file\n");
    return RSI_STATUS_FAILURE;
  }  
 // if (rsi_current_mac == NULL)
 // {
 //   printf("Unable to create current MAC file\n");
 //   return RSI_STATUS_FAILURE;
 // }  
  fprintf(rsi_last_mac,"RSI_LAST_WLAN_MAC_ID=%x\n",test_params.wlan_mac_id);
  fprintf(rsi_last_mac,"RSI_LAST_ZIGBEE_MAC_ID=%x\n",test_params.zigbee_mac_id);
  fprintf(rsi_last_mac,"RSI_LAST_BT_MAC_ID=%x\n",test_params.bt_mac_id);
 // fprintf(rsi_current_mac,"RSI_LAST_WLAN_MAC_ID=%x\n",test_params.wlan_mac_id -4);
 // fprintf(rsi_current_mac,"RSI_LAST_ZIGBEE_MAC_ID=%x\n",test_params.zigbee_mac_id-1);
 // fprintf(rsi_current_mac,"RSI_LAST_BT_MAC_ID=%x\n",test_params.bt_mac_id-1);
 // fclose(rsi_current_mac);
  fclose(rsi_last_mac);
  return RSI_STATUS_SUCCESS;
}

/**********************************************************************
 * Read the value of config parameters
 * ********************************************************************/
int rsi_get_token(FILE *info_file, const char *check_str, char *res)
{
  char temp[1024];
  char *start;

  fseek(info_file,0,SEEK_SET);
  while(!feof(info_file))
  {
    fscanf(info_file,"%s",temp);
    start = temp; /* skip all  spaces */
    while(isspace(*start)) start++;
    if((start[0] == '/') && (start[1] == '/'))
    {
      continue;
    }
    if((start = strstr(start,check_str)))
    {
      start+=strlen(check_str);
      start++;/* skip = */
      strcpy(res,start);
      return RSI_STATUS_SUCCESS;
    }
  }
  printf("\nWARNING ERROR IN READING CONFIG FILE.... SOME CONFIGURATIONS MISSING\n");
  return RSI_STATUS_FAILURE;
}

/**********************************************************************
 * Read config file parameters
 * ********************************************************************/
int rsi_read_config(void)
{
  char token[80];
  FILE *rsi_config, *rsi_last_mac;
  
  rsi_config = fopen("RSI_Config.txt","r");
  rsi_last_mac = fopen("RSI_LastMac.txt","rw");
    
  if (rsi_last_mac == NULL) 
  {
    printf("Unable to open LAST MAC file\n");
    if (rsi_config == NULL)
    {
      printf("Unable to open CONFIG file\n");
      return RSI_STATUS_FAILURE;
    }  
  }  
  if (rsi_get_token(rsi_config,"RSI_VENDOR_ID",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
    
  
  sscanf(token,"%x",&test_params.vendor_id);
  printf("Vebdor ID %x\n",test_params.vendor_id);
  
  //get last Mac Id from LiteFiLastMac.txt
  if (rsi_get_token(rsi_last_mac,"RSI_LAST_WLAN_MAC_ID",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%x",&test_params.wlan_mac_id);
  printf("WLAN MAC ID %x\n",test_params.wlan_mac_id);
 
  if (rsi_get_token(rsi_last_mac,"RSI_LAST_BT_MAC_ID",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%x",&test_params.bt_mac_id);
  printf("BT MAC ID %x\n",test_params.bt_mac_id);
  
  if (rsi_get_token(rsi_last_mac,"RSI_LAST_ZIGBEE_MAC_ID",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%x",&test_params.zigbee_mac_id);
  printf("ZIGBEE MAC ID %x\n",test_params.zigbee_mac_id);
  
//  memcpy(&test_params.wlan_mac_id , ,sizeof(token));

#if 1
  if (rsi_get_token(rsi_config,"RSI_WLAN_START_MAC_ID",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%x",&test_params.start_mac_id);
  printf("WLAN START MAC ID %x\n",test_params.start_mac_id);
  
  if (rsi_get_token(rsi_config,"RSI_WLAN_END_MAC_ID",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%x",&test_params.end_mac_id);
  printf("WLAN END MAC ID %x\n",test_params.end_mac_id);
#endif  
  if (rsi_get_token(rsi_config,"RSI_NUM_OF_WLAN_MAC_IDS",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%x",&test_params.num_wlan_macs);
  printf("NUM OF WLAN MAC ID's %x\n",test_params.num_wlan_macs);
  
  if (rsi_get_token(rsi_config,"RSI_BT_START_MAC_ID",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%x",&test_params.bt_start_mac_id);
  printf("BT START MAC ID %x\n",test_params.bt_start_mac_id);
  
  if (rsi_get_token(rsi_config,"RSI_BT_END_MAC_ID",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%x",&test_params.bt_end_mac_id);
  printf("BT END MAC ID %x\n",test_params.bt_end_mac_id);
  

  if (rsi_get_token(rsi_config,"RSI_ZIGBEE_START_MAC_ID",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%x",&test_params.zigbee_start_mac_id);
  printf("ZIGBEE START MAC ID %x\n",test_params.zigbee_start_mac_id);
  
  if (rsi_get_token(rsi_config,"RSI_ZIGBEE_END_MAC_ID",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%x",&test_params.zigbee_end_mac_id);
  printf("ZIGBEE END MAC ID %x\n",test_params.zigbee_end_mac_id);
  
  if (rsi_get_token(rsi_config,"RSI_FLASH_SIZE",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%x",&test_params.flash_size);
  //  printf("flash_size %x\n",test_params.flash_size);

  if (rsi_get_token(rsi_config,"RSI_SERVER_MAC",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
  sscanf(token,"%s",&test_params.server_mac[0]);

  fclose(rsi_last_mac);
  fclose(rsi_config);
  return RSI_STATUS_SUCCESS;
}

int fd_sock;

void *read_sock()
{
  FILE *get_addr = NULL;
  FILE *fconfig = NULL;
  FILE *fcpy = NULL;
  char string[200];
  char write_buffer[20];
  char read_buffer[20];
  short *ret_status;

  char addr[40];
  char *msg_states[] = {"WL_ST","WL_ED","BT_ST","BT_ED","ZB_ST","ZB_ED","NO_MAC"};
  char msg_state_sel,i;
  unsigned int wlan_addr_st;
  unsigned int wlan_addr_ed;
  unsigned int bt_addr_ed;
  unsigned int bt_addr_st;
  unsigned int zigbee_addr_st;
  unsigned int zigbee_addr_ed;
  char while_loop = 0;
  bzero(addr,40);
  ret_status = (short *)malloc(2);
  bzero(write_buffer,20);
  bzero(read_buffer,20);
  printf("enter the message to write: \n");
  system("ifconfig >> abc");
  if((get_addr = fopen("abc","r")) == NULL )
  {
    perror("fopen");
    return;
  }
  do
  {
    fscanf(get_addr,"%s",addr);
    if( strcmp("HWaddr",addr) == 0 )
    {
      fscanf(get_addr,"%s",addr);
      printf("Found MAC iD\n");
      break;

    }
    else if(strcmp("ether",addr) == 0 )
    {
      fscanf(get_addr,"%s",addr);
      break;

    }
  }while(1);
  fclose(get_addr);
  system("rm -rf abc");
  write(fd_sock, &addr , strlen(addr));
  while_loop = 1;
  while(while_loop )
  {
    read(fd_sock, &read_buffer, 20);
    for( i = 0; i < sizeof(msg_states)/sizeof(msg_states[0]); i++ )
    {
      if( strcmp(read_buffer,msg_states[i]) == 0 )
      {
        break;
      }
    }
    msg_state_sel = i;
    memcpy(write_buffer,"GOTIT",sizeof("GOTIT"));
    write( fd_sock, &write_buffer , strlen(write_buffer));
    switch (msg_state_sel)
    {
      case 0: printf("%s\n",read_buffer);
              read(fd_sock, &read_buffer, 20);
              memcpy(&wlan_addr_st,read_buffer,4);
              printf("wlan_addr_st = %x\n",wlan_addr_st);
              break;
      case 1: printf("%s\n",read_buffer);
              read(fd_sock, &read_buffer, 20);
              memcpy(&wlan_addr_ed,read_buffer,4);
              printf("wlan_addr_ed = %x\n",wlan_addr_ed);
              break;
      case 2: printf("%s\n",read_buffer);
              read(fd_sock, &read_buffer, 20);
              memcpy(&bt_addr_st,read_buffer,4);
              printf("bt_addr_st = %x\n",bt_addr_st);
              break;
      case 3: printf("%s\n",read_buffer);
              read(fd_sock, &read_buffer, 20);
              memcpy(&bt_addr_ed,read_buffer,4);
              printf("bt_addr_ed = %x\n",bt_addr_ed);
              break;
      case 4: printf("%s\n",read_buffer);
              read(fd_sock, &read_buffer, 20);
              memcpy(&zigbee_addr_st,read_buffer,4);
              printf("zigbee_addr_st = %x\n",zigbee_addr_st);
              break;
      case 5: printf("%s\n",read_buffer);
              read(fd_sock, &read_buffer, 20);
              memcpy(&zigbee_addr_ed,read_buffer,4);
              printf("zigbee_addr_ed = %x\n",zigbee_addr_ed);
              while_loop = 0;
              break;
      case 6: printf("\033[1m\033[31m \n\t\tMAC ADDR EXHAUSTED IN SERVER\033[0m\n");
              *ret_status = -1;
              pthread_exit(ret_status);
              break;
      default:
              while_loop = 0;
              break;
    }
    memcpy(write_buffer,"GOTIT",sizeof("GOTIT"));
    write( fd_sock, &write_buffer , strlen(write_buffer));
    bzero(read_buffer,20);
  }


  fconfig = fopen("RSI_Config.txt","r");
  fcpy = fopen("TEST.txt","w");
  if((fconfig == NULL) || (fcpy == NULL))
  {
    perror("fopen");
    return;
  }
  bzero(string,200);

  while(fgets(string,200,fconfig) != NULL )
  {
    if( strstr(string,"RSI_WLAN_START_MAC_ID") != NULL )
    {
      fprintf(fcpy,"%s","RSI_WLAN_START_MAC_ID=");
      fprintf(fcpy,"%x\n",wlan_addr_st);
    }
    else if(strstr(string,"RSI_WLAN_END_MAC_ID") != NULL )
    {
      fprintf(fcpy,"%s","RSI_WLAN_END_MAC_ID=");
      fprintf(fcpy,"%x\n",wlan_addr_ed);
    }
    else if(strstr(string,"RSI_BT_START_MAC_ID") != NULL )
    {
      fprintf(fcpy,"%s","RSI_BT_START_MAC_ID=");
      fprintf(fcpy,"%x\n",bt_addr_st);
    }
    else if(strstr(string,"RSI_BT_END_MAC_ID") != NULL )
    {
      fprintf(fcpy,"%s","RSI_BT_END_MAC_ID=");
      fprintf(fcpy,"%x\n",bt_addr_ed);
    }
    else if(strstr(string,"RSI_ZIGBEE_START_MAC_ID") != NULL )
    {
      fprintf(fcpy,"%s","RSI_ZIGBEE_START_MAC_ID=");
      fprintf(fcpy,"%x\n",zigbee_addr_st);
    }
    else if(strstr(string,"RSI_ZIGBEE_END_MAC_ID") != NULL )
    {
      fprintf(fcpy,"%s","RSI_ZIGBEE_END_MAC_ID=");
      fprintf(fcpy,"%x\n",zigbee_addr_ed);
    }
    else
    {
      fprintf(fcpy,"%s",string);
    }
  }

  fclose(fcpy);
  fclose(fconfig);
  system("cp -f TEST.txt RSI_Config.txt");

  test_params.wlan_mac_id = wlan_addr_st;
  test_params.zigbee_mac_id = zigbee_addr_st;
  test_params.bt_mac_id = bt_addr_st;

  if (rsi_save_mac() != RSI_STATUS_SUCCESS)
  {
    printf("\nWARNING ERROR IN SAVING LAST MAC FILE \n");
  }

  //  while ( strcmp( read_buffer,"bye" ) != 0)
  //  {
  //    printf("server : %s\n", read_buffer);
  //    bzero(read_buffer,20);
  //    read( fd_sock ,&read_buffer, 20);
  //  }
  *ret_status = 0;
  pthread_exit(ret_status);

}
int alloc_new_mac_id()
{
  pthread_t read_thread, write_thread;
  struct sockaddr_in sock_addr;

  short *rec_status;

  fd_sock = socket(AF_INET,SOCK_STREAM, 0);

  sock_addr.sin_family = AF_INET;
  sock_addr.sin_port = htons(1111);

  if(inet_pton( AF_INET, test_params.server_mac, & sock_addr.sin_addr ) == -1 )
  {
    perror("inet_pton");
    return RSI_STATUS_FAILURE;
  }

  printf("Socket created successfully\n");

  if (connect(fd_sock,(const struct sockaddr *)&sock_addr,sizeof(sock_addr)) == -1 )
  {
    perror("\n\t\033[1m\033[31mPlease check if Server is up!!!\n\t");
    return RSI_STATUS_FAILURE;
  }


  printf(" connection successful\n");

  pthread_create( &read_thread, NULL, &read_sock, NULL);
  //  pthread_create( &write_thread, NULL, &write_sock, NULL);
  pthread_join(read_thread, (void **)&rec_status);
  close( fd_sock );
  if( *rec_status == -1 )
  {
    return RSI_STATUS_FAILURE;
  }
  return RSI_STATUS_SUCCESS;


  //  pthread_join( write_thread, NULL);
}

