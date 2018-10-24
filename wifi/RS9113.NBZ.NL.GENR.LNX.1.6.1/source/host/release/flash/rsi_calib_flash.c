#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "rsi_api_routine.h"
#include <time.h>

extern test_params_t test_params;
unsigned int mbr_dump[] = {
#include "WC/mbr_hex_dump"
}; 
unsigned int mbr_dump_host[] = {
#include "WC/mbr_host_hex_dump"
};  

#define MBR_DUMP_SIZE 16
#define MBR_DUMP_SIZE1 24

char module[] = "RS9113-0301-WBZ0";
//char module[16];

#define WLAN_MAC_ID_LS_BYTE   0x38
#define ZIGBEE_MAC_ID_LS_BYTE 0x09

#define FIPS_OFFSET_1   4042753
time_t start_createlog,stop_createlog,start_wbz,stop_wbz,start_wbz_w,stop_wbz_w;
struct eeprom_values
{
  unsigned int bootloader[4];
  unsigned int  sw_feature_enable;
  unsigned char flash_vendor_type;
  unsigned char flash_variant;
  unsigned int  flash_size;
  unsigned char padding[14];
  unsigned char wlan_magic_word;
  unsigned int  wlan_enables;
  unsigned char wlan_mac_id[6];
  unsigned char bt_magic_word;
  unsigned int bt_enables;
  unsigned char bt_mac_id[6];
  unsigned char zigbee_magic_word;
  unsigned int zb_enables;
  unsigned char zigbee_mac_id[8];
  unsigned char common_magic_word[2];
  unsigned short	eeprom_version;
  unsigned char	module_num[16];
  unsigned char	sdio_ids[4];
  unsigned char	num_wlan_mac;
  unsigned char	digital_chip_version[2];
  unsigned char	module_version[2];
  unsigned char	RF_chip_version[2];
  unsigned char	MfgSoftware_version[2];
  unsigned short usb_vendorid; 
  unsigned short usb_deviceid;
  unsigned char reserved[40];
  unsigned char reserved1[272];
}__attribute__ ((packed))values  __attribute__ ((section(".eeprom_value_section"))) =
{
  .bootloader = {0x00000000},
  .sw_feature_enable = 0,
  .flash_vendor_type = 0,
  .flash_variant = 0,
  .flash_size = 0,
  .wlan_magic_word = 0x5A,
  .wlan_enables   = 0,
  .wlan_mac_id = {0x00, 0x23, 0xA7, 0x00, 0x00, 0x00},
  .bt_magic_word = 0x5A,
  .bt_enables = 0,
  .bt_mac_id = {0x00, 0x23, 0xA7, 0x00, 0x00, 0x00},
  .zigbee_magic_word = 0x5A,
  .zb_enables = 0,
  .zigbee_mac_id = {0x00, 0x23, 0xA7, 0xff, 0xff, 0x00, 0x00, 0x00},
  .common_magic_word = {0x5A, 0},
  .eeprom_version = 0,
  .module_num = {"RS9113-0003-WBZ0"},
  .sdio_ids = 0,
  .num_wlan_mac = 4,
  .digital_chip_version = {1,2},        //Digital Chip version 1.1
  .module_version = {4,5},        //Module version 0.0
  .RF_chip_version = {1,3},        //RF Chip version 0.0
  .MfgSoftware_version = {1,0},        //RF Chip version 0.0
  .usb_vendorid  = 0x1618, 
  .usb_deviceid  = 0x9113, 
  // .usb_vendorid  = 0x041b, 
  // .usb_deviceid  = 0x0301, 
  .reserved = {},
  .reserved1 = {}
};


void append(FILE *head, FILE *tail)
{
    char buf[1024];
    size_t n;
    while ((n = fread(buf, 1, sizeof buf, tail)) > 0)
        if (fwrite(buf, 1, n, head) != n)
            abort();
    if (ferror(tail))
        abort();
}

/*Copying MAC address 
 * @Parameters: Destination
 *            : Source
 *            : No of bytes to be copied*/
void rsi_copy_mac_info(char *mac, int test_mac, int bytes)
{
  unsigned char *temp;

  temp = (unsigned char *)&test_mac;
  bytes--;
  do 
  {
    *mac = temp[bytes];
    mac++;
  } while (bytes--);
} 
int version_append_func(int arg,int chip_ver)  
{
  int i=0;
  char chip_version[2]={0,0};
  chip_version[i++]= chip_ver/10;
  chip_version[i]= chip_ver%10;
  if(arg == 1)
  {
    memset(values.digital_chip_version,'\0',2);
    memcpy(&values.digital_chip_version[0], chip_version, 2);
  }
  else if(arg == 2)
  {
    memset(values.module_version,'\0',2);
    memcpy(&values.module_version[0], chip_version, 2);
  }
  else if(arg == 3)
  {
    memset(values.RF_chip_version,'\0',2);
    memcpy(&values.RF_chip_version[0], chip_version, 2);
  }
  else if(arg == 4)
  {
    memset(values.MfgSoftware_version,'\0',2);
    memcpy(&values.MfgSoftware_version[0], chip_version, 2);
  }
  return 0;
}
int wbz_append(int type,int module_type,int fips_mode,int ant_type) //type- 1:Nlink 2:Wise connect
{
  int ret;
  char str[50],str1[50];
  strcpy(str,"sh WC/wbz_append.sh");
  strcat(str, " ");
  printf("\n*********WBZ function file*******\n");
  printf("\ntype : %d module_type : %d fips_mode : %d\n",type,module_type,fips_mode);

  memset(module,'\0',sizeof(module));

  if( type == 1 )
  {
    switch( module_type )// N-Link
    {
      case 2:
        if(ant_type == 0)
        {
          sprintf(module,"%s","RS9113-NBZ-S0N");
          printf("\nReq char is : %s\n",module);
        }
        else if(ant_type == 1)
        {
          sprintf(module,"%s","RS9113-NBZ-S1N");
          printf("\nReq char is : %s\n",module);
        }
        break;
      case 3:
        if(ant_type == 0)
        {
          sprintf(module,"%s","RS9113-NBZ-D0N");
          printf("\nReq char is : %s\n",module);
        }
        else if(ant_type == 1)
        {
          sprintf(module,"%s","RS9113-NBZ-D1N");
          printf("\nReq char is : %s\n",module);
        }
        break;
    }
  }
  else if( type == 2 )// WISE
  {
    switch( module_type )
    {
      case 2: // SINGLE BAND
        switch (fips_mode )
        {
          case 0:
          case 1:
          case 2:
            if(ant_type == 0)
            {
              sprintf(module,"%s","RS9113-NBZ-S0W");
              printf("\nReq char is : %s\n",module);
            }
            else if(ant_type == 1)
            {
              sprintf(module,"%s","RS9113-NBZ-S1W");
              printf("\nReq char is : %s\n",module);
            }
            break;
          case 3: //WISE-MCU
            sprintf(module,"%s","RS10002-NBZ-S0M");
            printf("\nReq char is : %s\n",module);
            break;             
        }
        break;
      case 3:// DUAL BAND
        {
          switch (fips_mode )
          {
            case 0:
              if(ant_type == 0)
              {
                sprintf(module,"%s","RS9113-NBZ-D0W");
                printf("\nReq char is : %s\n",module);
              }
              else if(ant_type == 1)
              {
                sprintf(module,"%s","RS9113-NBZ-D1W");
                printf("\nReq char is : %s\n",module);
              }
              break;
            case 1:
              sprintf(module,"%s","RS9113-N00-D0F-D");
              printf("\nReq char is : %s\n",module);
              break;
            case 2:
              sprintf(module,"%s","RS9113-N00-D0F");
              printf("\nReq char is : %s\n",module);
              break;
            case 4: //WISE-MCU
              sprintf(module,"%s","RS10003-NBZ-D0M");
              printf("\nReq char is : %s\n",module);
              break;
          }
        }
        break;
    }
  }


  memcpy(&values.module_num[0], module, 16);
  return 0;
}

int main (int argc, char *argv[])
{
  int i = 0, err =0, k = 0;
  FILE *pFile;
  unsigned char *temp;
  unsigned char query;
  int status = RSI_STATUS_SUCCESS;
  int type, update_mac,module_type,module_append,ant_type,chip_ver, mfg_sw_ver;
  int card_no = 0;

  unsigned char fips_key[] = {
#include "FIPS_KEY"
  };
  FILE *fptr_write = NULL;
  FILE *fptr_read = NULL;
  unsigned char value[10];

  //char module_1[] = "RS9113-WBZ-0002-"; //BL:0 & Mod_type:2  
  //char module_2[] = "RS9113-WBZ-0003-"; //BL:0 & Mod_type:3 
  //char module_3[] = "RS9113-WB0-0201-"; //BL:1 & Mod_type:2 
  //char module_4[] = "RS9113-WB0-0301-"; //BL:1 & Mod_type:3 

  if (argc == 3)
  {  
    update_mac = atoi(argv[1]);
    //    printf("\nCard_number1: %d\n",card_no);	
    //card_no    = atoi(argv[2]);
  }
  else if (argc == 12)
  {
    type = atoi(argv[1]);
    module_type = atoi(argv[5]);
    module_append = atoi(argv[6]);
    ant_type = atoi(argv[11]);
    if((atoi(argv[2]) == 0) || (atoi(argv[3]) == 0))
    {
      printf("\nWARNING Incorrect arguments passed to flash application\n");

    }
    values.eeprom_version = atoi(argv[2]);

    values.flash_size = atoi(argv[3]);
    values.flash_vendor_type = atoi(argv[4]);
    version_append_func(1,atoi(argv[7]));          //Digital Chip Version
    version_append_func(2,atoi(argv[8]));          //Module Version
    version_append_func(3,atoi(argv[9]));          //RF Chip Version
    version_append_func(4,atoi(argv[10]));         //MfgSoftware Version
    printf("\nReceived arguments from script file argv[0]:%d, argv[1]:%d ,argv[2]:%d ,argv[3]:%d ,argv[4]:%d %d %d %d %d %d\n"
        ,atoi(argv[0]),atoi(argv[1]),atoi(argv[2]),atoi(argv[3]),atoi(argv[4]),atoi(argv[5]),atoi(argv[6]),atoi(argv[7]),atoi(argv[8]),atoi(argv[9]));
  } 
  else if( argc == 2 )
  {
    if(!strcmp("append_fips", argv[1]))
    {
      printf(" Appending the FIPS Key\n");

      fptr_read  = fopen("non_rf_values3.txt", "r");
      fptr_write = fopen("RS9113_RS8111_calib_values.txt", "w"); 

      if( (fptr_read == NULL) || (fptr_write == NULL) )
      {
        printf("WARNING ERROR OPENING FILE\n");
        return RSI_STATUS_FAILURE;
      }
      i = 0;
      memset( value, 0, 10 );
      while( fgets(value, 10, fptr_read) != NULL )
      {
        i++;
        switch( i )
        {
          case FIPS_OFFSET_1:
            for(k = 0; k < sizeof(fips_key)/sizeof(fips_key[0]) -1; k++ )
            {
              fprintf(fptr_write, "0x%.2x,", fips_key[k]);
              fgets(value, 10, fptr_read);
            }
            fprintf(fptr_write, "0x%.2x,", fips_key[k]);
            i += k;
            break;
          default:
            fprintf(fptr_write,"%s",value);
            break;
        }

      }
      if( i < FIPS_OFFSET_1 )
      {
        for( i; i < FIPS_OFFSET_1 - 1; i++ )
        {
          //fprintf(fptr_write, "0x%.2x,\n", fips_key[k]);
          fprintf(fptr_write, "0x00,\n");
        }
        for(k = 0; k < sizeof(fips_key)/sizeof(fips_key[0]); k++ )
        {
          fprintf(fptr_write, "0x%.2x,\n", fips_key[k]);
        }
      }
      fclose(fptr_write);
      fclose(fptr_read);
    }
    else
    {
      printf("WARNING INVALID ARG\n");
      return RSI_STATUS_FAILURE;
    }
    return RSI_STATUS_SUCCESS;
  }
  else
  {
    printf("\nWARNING SENDING INCORRECT NUMBER OF ARGUMENTS\n");
    return RSI_STATUS_FAILURE;
  } 
  if (type == 1 || type == 2)
  {  
    pFile = fopen("non_rf_values.txt", "w");
    //	pFile = fopen(argv[1], "w");
    if(pFile == NULL)
    {
      printf("WARNING Unable to create a file\n");
      return RSI_STATUS_FAILURE;
    }

READ_CONFIG:
    if (rsi_read_config() != RSI_STATUS_SUCCESS)
    {
      printf("\nWARNING READING CONFIG FILE FAILURE\n");
      return RSI_STATUS_FAILURE;
    }


    if((test_params.wlan_mac_id < test_params.start_mac_id) || 
        ((test_params.wlan_mac_id + test_params.num_wlan_macs) > test_params.end_mac_id) || 
        (test_params.wlan_mac_id & 0x03))
    {
	  write(2,"\n\t\033[1m\033[31m WLAN MAC ADDR EXHAUSTED\n", sizeof("\n\t\033[1m\033[31m WLAN MAC ADDR EXHAUSTED\n"));

      printf("\nWLAN MAC ADDRESS FOUND OUT OF RANGE\n");
      printf("Requesting Server for New pool\n");
      if ( alloc_new_mac_id() != RSI_STATUS_SUCCESS )
      {
        printf("\nWARNING MAC ADDRESS EXHAUSTED\n");
        return RSI_STATUS_FAILURE;
      }
      goto READ_CONFIG;
    }
    if((test_params.zigbee_mac_id < test_params.zigbee_start_mac_id) || (test_params.zigbee_mac_id > test_params.zigbee_end_mac_id))
    {
      printf("\nWARNING ZIGBEE MAC ADDRESS FOUND OUT OF RANGE\n");
	  write(2,"\n\t\033[1m\033[31m WARNING ZIGBEE MAC ADDRESS FOUND OUT OF RANGE\n", sizeof("\n\t\033[1m\033[31m WARNING ZIGBEE MAC ADDRESS FOUND OUT OF RANGE\n"));
      return RSI_STATUS_FAILURE;
    }
    if((test_params.bt_mac_id < test_params.bt_start_mac_id) || (test_params.bt_mac_id > test_params.bt_end_mac_id))
    {
      printf("\nWARNING BT MAC ADDRESS FOUND OUT OF RANGE\n");
	  write(2,"\n\t\033[1m\033[31m WARNING BT MAC ADDRESS FOUND OUT OF RANGE\n", sizeof("\n\t\033[1m\033[31m WARNING BT MAC ADDRESS FOUND OUT OF RANGE\n"));
      return RSI_STATUS_FAILURE;
    }


    //#ifdef RSI_LOG
#if 1
    printf("wlan_mac 0x%x \n",test_params.wlan_mac_id);
    printf("zigbee_mac 0x%x \n",test_params.zigbee_mac_id);
    printf("bt_mac 0x%x \n",test_params.bt_mac_id);
#endif  
    rsi_copy_mac_info(&values.wlan_mac_id[3], test_params.wlan_mac_id, 3);
    rsi_copy_mac_info(&values.bt_mac_id[3], test_params.bt_mac_id, 3);
    rsi_copy_mac_info(&values.zigbee_mac_id[5], test_params.zigbee_mac_id, 3);

    start_wbz = time(NULL); 	
    wbz_append(type,module_type,module_append,ant_type);
    stop_wbz = time(NULL); 	
    printf("\n==========>Elapsed time for wbz function:%2f Sec\n",difftime(stop_wbz,start_wbz));
    //if(module_type == 2)                      //indicates 201 module
    //{
    //	  wbz_append(type,module_type,module_append);
    //	  //memcpy(&values.module_num[0], module_1, 16);
    //}	
    //else 	   //indicates 301 module
    //{
    //	  wbz_append(type,module_type,module_append);
    //	  //memcpy(&values.module_num[0], module_2, 16);
    //}
    //! FOR WISE-CONNECT modules  
    if (type == 2)
    { 

      printf("WISE CONNECT MODULE \n");
      memcpy(&values.bootloader[0], mbr_dump, MBR_DUMP_SIZE);
      start_wbz_w = time(NULL); 	
      wbz_append(type,module_type,module_append,ant_type);
      stop_wbz_w = time(NULL); 	
      printf("\n==========>Elapsed time for wbz function:%2f Sec\n",difftime(stop_wbz_w,start_wbz_w));
      //  values.flash_size = 0x800; //2MB flash
      //if(module_type == 2)                      //indicates 201 module
      //{
      //    wbz_append(type,module_type,module_append);
      //    //memcpy(&values.module_num[0], module_3, 16);
      //}
      //else	                                    //indicates 301 module
      //{
      //    wbz_append(type,module_type,module_append);
      //    //memcpy(&values.module_num[0], module_4, 16);
      //}
    }
    else
    {
      printf("HOSTED MODULE \n");
      memcpy(&values.bootloader[0], mbr_dump_host, MBR_DUMP_SIZE);
    }
#ifdef RSI_LOG
    printf("wlan_mac 0x%x \n", *(unsigned int *)&values.wlan_mac_id[3]);
    printf("BT_mac 0x%x \n", *(unsigned int *)&values.bt_mac_id[3]);
    printf("Zigbee_mac 0x%x \n", *(unsigned int *)&values.zigbee_mac_id[5]);
    printf("flash_size %x\n",values.flash_size);
#endif  
    temp = (unsigned char *)&values.bootloader[0]; 
    for (i = 0; i< sizeof(values); i++)
      fprintf(pFile, "0x%x,\n",temp[i]);
    err = fclose(pFile);
    printf("ERROR No: %d \n",err);
    if(pFile == NULL)
      printf("PFILE is NULL\n");
    test_params.wlan_mac_id += 4;
    test_params.zigbee_mac_id += 1;
    test_params.bt_mac_id += 1;
#if 0
    printf("DO YOU WANT TO FLASH: (y or n ) ");
    scanf("%c",&query);
    if (query == 'y')
    {
      printf("PROCEEDING TO BURN FLASH \n");
    }
    else 
    {
      printf("WARNING SKIPPING FLASH BURNING \n");
      return RSI_STATUS_FAILURE;
    }
#endif
    return RSI_STATUS_SUCCESS;
  }
  else if (update_mac == 4 || update_mac == 5)
  {
    if (rsi_read_config() != RSI_STATUS_SUCCESS) 
    {
      printf("\nWARNING READING CONFIG FILE FAILURE\n");
      return RSI_STATUS_FAILURE;
    }

#if 1
    if(argc == 3 )
    {
      card_no = atoi(argv[2]);
      //      printf("\nCard_number: %d\n",card_no);	
    }
#endif
    start_createlog = time(NULL);	
    if(rsi_create_log( type, card_no ) != RSI_STATUS_SUCCESS)
    {
      printf("WARNING Creating Log Failed\n");
      return RSI_STATUS_FAILURE;
    }
    stop_createlog = time(NULL);	
    printf("\n=========>Elapsed time for creating the log:%2f Sec\n",difftime(stop_createlog,start_createlog));
    printf("\nUPDATING LAST MAC FILE \n");
    test_params.wlan_mac_id += 4;
    test_params.zigbee_mac_id += 1;
    test_params.bt_mac_id += 1;
    if (rsi_save_mac() != RSI_STATUS_SUCCESS)
    {
      printf("\nWARNING ERROR IN SAVING LAST MAC FILE \n");
      return RSI_STATUS_FAILURE;
    }
    return RSI_STATUS_SUCCESS;
  }
  return RSI_STATUS_SUCCESS;
}	
