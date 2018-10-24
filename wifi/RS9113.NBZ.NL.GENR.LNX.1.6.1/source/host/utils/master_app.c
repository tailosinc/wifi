#include<stdio.h>
#include<math.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <linux/wireless.h>
#include <linux/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <inttypes.h>
#include "onebox_util.h"
//#include "manuf_util.h"
#include <time.h>
#include <sys/stat.h>
#include<sys/types.h>
#include <pthread.h>
#include <dirent.h>

#define TYPE_OFFSET             2
#define XO_OFFSET               4 
#define TX_IQ_2G_20             5 
#define TX_IQ_2G_40             13
#define TX_IQ_5G_1_20           97
#define TX_IQ_5G_1_40           105
#define TX_IQ_5G_2_20           189
#define TX_IQ_5G_2_40           197
#define TX_IQ_5G_3_20           281
#define TX_IQ_5G_3_40           289
#define TX_IQ_5G_4_20           373
#define TX_IQ_5G_4_40           381

#define GAIN_OFFSET_2G          546
#define GAIN_OFFSET_5G_1        624
#define GAIN_OFFSET_5G_2        700
#define GAIN_OFFSET_5G_3        776
#define GAIN_OFFSET_5G_4        852

#define RF_TEMP_CONST_INT       853
#define RF_TEMP_CONST_FRA       854
#define ULP_TEMP_CONST_INT       855
#define ULP_TEMP_CONST_FRA       856

#define RSI_STATUS_SUCCESS 0
#define RSI_STATUS_FAILURE -1

#define XO_CTUNE                BIT(0)
#define GAIN_OFF_ESTI           BIT(1)
#define TX_IQ_IMB               BIT(2)
#define RX_IQ_IMB               BIT(3)
#define GPIO_TEST               BIT(4)
#define CALIB_DONE              BIT(5)
#define RF_TEMP_CALIB           BIT(6)
#define ULP_TEMP_CALIB          BIT(7)
#define VRMS_2G                 BIT(8)
#define VRMS_5G                 BIT(9)
#define ENABLE_FLOOR 1
//#define DEBUG_PRINT 1
struct timeval starttime,endtime,timediff;
struct timeval starttime_temp,endtime_temp,timediff_temp;
float x_rf = 0;
float x_ulp = 0;
short int chan,ch_cnt=0;
uint_8 rx_band_2G_en=0;
uint_8 rx_band_5G_1_en=0;
uint_8 rx_band_5G_2_en=0;
uint_8 rx_band_5G_3_en=0;
uint_8 rx_band_5G_4_en=0;

typedef struct calib_params_s
{
  uint_8 ctune_val;
  uint_16 cfo_val;
  int_8 gain_offset_2G;
  int_8 gain_offset_5G_1;
  int_8 gain_offset_5G_2;
  int_8 gain_offset_5G_3;
  int_8 gain_offset_5G_4;
  uint_16 tx_dci_2G;
  uint_16 tx_dcq_2G;
  uint_16 tx_betacos_2G;
  uint_16 tx_betasin_2G;
  uint_16 tx_dci_5G;
  uint_16 tx_dcq_5G;
  uint_16 tx_betacos_5G;
  uint_16 tx_betasin_5G;
  int_32 pow_db_2G;
  int_32 pow_db_5G_1;
  int_32 pow_db_5G_2;
  int_32 pow_db_5G_3;
  int_32 pow_db_5G_4;
  int_16 tx_g_2G;
  int_16 tx_ph_2G;
  int_16 tx_g_5G;
  int_16 tx_ph_5G;
  int_32 isq_avg;
  int_32 qsq_avg;
  int_32 iq_avg;
  int_32 isq_avg_5G;
  int_32 qsq_avg_5G;
  int_32 iq_avg_5G;
  uint_32 var;
  uint_32 var_5g;
  uint_32 var_5g_2;
  uint_32 var_5g_3;
  uint_32 var_5g_4;
  int_8 rf_int;
  int_8 rf_fra;
  int_8 ulp_int;
  int_8 ulp_fra;
  uint_16 adapter_state;
  uint_32 time_ms;
  int_32 gdb_est_2G;
  int_32 ph_deg_approx_2G;
  int_32 gdb_est_5G;
  int_32 ph_deg_approx_5G;
  uint_32 variance_2G;
  uint_32 variance_5G_1;
  uint_32 variance_5G_2;
  uint_32 variance_5G_3;
  uint_32 variance_5G_4;
  int_8 rx_vrms_offset_2G;
  int_8 rx_vrms_offset_5G_1;
  int_8 rx_vrms_offset_5G_2;
  int_8 rx_vrms_offset_5G_3;
  int_8 rx_vrms_offset_5G_4;
  int_16 rx_channel[5]; 
}calib_params_t;

int_16 rx_betacos = 0, rx_betasin = 0;
int_16 rx_betacos_5G = 0, rx_betasin_5G = 0;

double rx_ph = 0,rx_g = 0;

typedef struct module_params_s
{
  unsigned short type;
  unsigned short boot_load;
  unsigned short eeprom_version;
  unsigned short eeprom_type;
  unsigned short module_append;
  unsigned short eeprom_size;
  unsigned short digi_chip_ver;
  unsigned short module_ver;
  unsigned short rf_chip_ver;
  unsigned short flash_check;
  unsigned short ant_type;
  unsigned short mfg_sw_ver;
  unsigned short gpio_bypass;
}module_params_t;

module_params_t module_params;

/*Taken out these parameters from firmware for giving control to user
 * use this structure for any other similar handling in future */
typedef struct config_values_s
{
  signed short cable_loss_2g; 
  signed short cable_loss_5g_1; 
  signed short cable_loss_5g_2;
  signed short cable_loss_5g_3;
  signed short cable_loss_5g_4;
  unsigned short cable_loss_type;
  unsigned short rx_gain_offset_2g; 
  unsigned short rx_gain_offset_5g_1; 
  unsigned short rx_gain_offset_5g_2;
  unsigned short rx_gain_offset_5g_3;
  unsigned short rx_gain_offset_5g_4;
  unsigned short expected_power_2g; 
  unsigned short expected_power_5g_1;
  unsigned short expected_power_5g_2;
  unsigned short expected_power_5g_3;
  unsigned short expected_power_5g_4;
  int_16 rx_channel[5]; 
}config_values_t;

config_values_t config_values;

typedef struct mac_add_s
{
  unsigned char wlan[6];
  unsigned char bt[6];
  unsigned char zb[8];
}mac_add_t;

mac_add_t mac_add;

int pipe_fd[2];

int onebox_open_interface(void)
{
  FUNCTION_ENTRY();
  int sockfd = 0;
  sockfd = socket(PF_INET, SOCK_DGRAM, 0);    
  if (sockfd < 0)
  {
    printf("Unable to create a socket\n");
    return sockfd;
  }
  FUNCTION_EXIT();
  return sockfd;
}
int rsi_read_lastmac(void)
{
  unsigned char token[80],temp[8],value[2];
  FILE *rsi_lastmac;
  int i;
  rsi_lastmac = fopen("../release/RS9113_RS8111_calib_values.txt","r");

  if (rsi_lastmac == NULL)
  {
    printf("Unable to open calib values.txt file\n");
    return RSI_STATUS_FAILURE;
  }  

  fseek(rsi_lastmac, (45*6), SEEK_SET);
  memset(temp,0,sizeof(temp));
  for(i=0;i<6;i++)
  {
    fscanf(rsi_lastmac,"%x%c\n",&temp[i],&value[0]);
    //printf("temp: %s",temp[i]);
  }
  sscanf(temp,"%s",&mac_add.wlan);
  memcpy(&mac_add.wlan, temp, sizeof(temp));

  fseek(rsi_lastmac, (56*6), SEEK_SET);
  memset(temp,0,sizeof(temp));
  for(i=0;i<6;i++)
  {
    fscanf(rsi_lastmac,"%x%c\n",&temp[i],&value[0]);
    //printf("temp: %s",temp[i]);
  }
  sscanf(temp,"%s",&mac_add.bt);
  memcpy(&mac_add.bt, temp, sizeof(temp));

  fseek(rsi_lastmac, (67*6),SEEK_SET); 
  memset(temp,0,sizeof(temp));
  for(i=0;i<8;i++)
  {
    fscanf(rsi_lastmac,"%x%c\n",&temp[i],&value[0]);
    //printf("temp: %s",temp[i]);
  }
  sscanf(temp,"%s",&mac_add.zb);
  memcpy(&mac_add.zb, temp, sizeof(temp));
  fclose(rsi_lastmac);
  return RSI_STATUS_SUCCESS;
}
void prepare_log_file( calib_params_t *calib_values_log,uint_8 fail ,uint_16 module_type)
{
  return;
	time_t cur_time;
	unsigned char i = 0;
	unsigned char mac_prefix[] ="0023A7";
	unsigned char file_name[100],str[100];
	unsigned char path_name[100] = "flash/test_5_log/";
	FILE *fptr = NULL, *fmac = NULL;
	unsigned char ch;
	uint_32 mac_id = 0;
	DIR *dp;
	struct dirent *ep;
	int k=0;


	if(!( mkdir("flash/test_5_log", S_IRWXU | S_IRWXG )   == -1 ))
	{
		//printf("successfully created the directory\n");
	}

	dp = opendir("../release/flash/WC/");
	if( dp == NULL )
	{
		perror("opendir");
		return;
	}


	memset(file_name,0,sizeof(file_name));

	if( fail )
	{
		cur_time  = time(NULL);

		memcpy(file_name,ctime(&cur_time),strlen(ctime(&cur_time)));

		for(i = 0; i < strlen(ctime(&cur_time)); i++ )
		{
			if(file_name[i] == ' ')
			{
				file_name[i] = '_';
			}
		}
		file_name[i - 1 ] = '\0';

	}
	else if( fail == 0 )
	{
		fmac = fopen("flash/RSI_LastMac.txt","r");
		if(fmac == NULL)
		{
			printf("Error Unable to open RSI_LastMac.txt");
			return;
		}
		fgets(file_name,50,fmac);
		i = 0;
		ch = file_name[i];
		while(ch != '=')
		{
			i++;
			ch = file_name[i];
		}
		memmove(file_name,&file_name[i + 1],strlen(file_name));
		file_name[strlen(file_name) - 1 ] = '\0';

		mac_id = strtol(file_name,NULL,16);
		mac_id -=4;
		sprintf(file_name,"%.6X",mac_id);
		memset(str,'\0',sizeof(str));
#ifdef NO_FIPS
		sprintf(str,"%s","RS9113_N00_");
#else
		sprintf(str,"%s","RS9113_NBZ_");
#endif
		switch(module_params.type)
		{
			case 2:
				strcat(str,"S");
				break;
			case 3:
				strcat(str,"D");
				break;
		}
		switch(module_params.ant_type)
		{
			case 0:
				strcat(str,"0");
				break;
			case 1:
				strcat(str,"1");
				break;
		}
#ifdef NO_FIPS
		strcat(str,"F");
#else
		switch(module_params.boot_load)
		{
			case 0:
				strcat(str,"N");
				break;
			case 1:
				strcat(str,"W");
				break;
		}
#endif
#ifdef DRAGGER
		strcat(str,"_DRG");
#elif WISE_MCU
		strcat(str,"_WISE_MCU");
#elif WISE_MCU_8X8
		strcat(str,"_WISE_MCU_8X8");
#endif

		strcat(str,"_");
		strcat(mac_prefix,file_name);
		strcat(str,mac_prefix);
		strcpy(file_name,str);
		fclose(fmac);
	}
	strcat(path_name,file_name);
	strcat(path_name,".txt");
	fptr = fopen(path_name,"w");

	if( fptr == NULL )
	{
		printf("Error while opening Log File\n");
		return;
	}
	if( module_type == 2 )
	{
		fprintf(fptr,"\t\t\tCalibration Log For 02 Module\r\n\r\n");
	}
	else
	{
		fprintf(fptr,"\t\t\tCalibration Log For 03 Module\r\n\r\n");
	}

	if( rsi_read_lastmac() == RSI_STATUS_FAILURE )
	{
		printf("Unable to read Last MAC File\n");
	}
	if( !fail )
	{
		fprintf(fptr,"------------------\r\n");
		fprintf(fptr,"WLAN MAC_ID      :::: %02x:%02x:%02x:%02x:%02x:%02x\r\n",mac_add.wlan[0],mac_add.wlan[1],mac_add.wlan[2],mac_add.wlan[3],mac_add.wlan[4],mac_add.wlan[5]);
		fprintf(fptr,"BT MAC_ID        :::: %02x:%02x:%02x:%02x:%02x:%02x\r\n",mac_add.bt[0],mac_add.bt[1],mac_add.bt[2],mac_add.bt[3],mac_add.bt[4],mac_add.bt[5]);
		fprintf(fptr,"ZB MAC_ID        :::: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\r\n",mac_add.zb[0],mac_add.zb[1],mac_add.zb[2],mac_add.zb[3],mac_add.zb[4],mac_add.zb[5],mac_add.zb[6],mac_add.zb[7]);
		fprintf(fptr,"------------------\r\n");
		fprintf(fptr,"\r\n\r\n");
	}
	else
	{
		fprintf(fptr,"---------------------------------\r\n");
		//fprintf(fptr,"\r\n\r\n");
	}
	fprintf(fptr,"Ctune_Val           =    0x%x\r\n",(uint_8)calib_values_log->ctune_val);
	fprintf(fptr,"Cfo_Val             =    0x%x\r\n",(uint_8)calib_values_log->cfo_val);
	fprintf(fptr,"TX_Gain_Off_2G      =    %d\r\n",(int_8)calib_values_log->gain_offset_2G);
	fprintf(fptr,"RX_gain_est         =    %lf\r\n",(float)calib_values_log->gdb_est_2G/10000);
	fprintf(fptr,"RX_phase_est        =    %lf\r\n",(float)calib_values_log->ph_deg_approx_2G/10000);
	fprintf(fptr,"RX_betacos_2G       =    0x%x\r\n",(uint_16)rx_betacos);
	fprintf(fptr,"RX_betasin_2G       =    0x%x\r\n",(uint_16)rx_betasin);
	fprintf(fptr,"TX_DCI_2G           =    0x%x\r\n",(uint_16)calib_values_log->tx_dci_2G);
	fprintf(fptr,"TX_DCQ_2G           =    0x%x\r\n",(uint_16)calib_values_log->tx_dcq_2G);
	fprintf(fptr,"TX_betacos_2G       =    0x%x\r\n",(uint_16)calib_values_log->tx_betacos_2G);
	fprintf(fptr,"TX_betasin_2G       =    0x%x\r\n",(uint_16)calib_values_log->tx_betasin_2G);
	fprintf(fptr,"TX_phase_2G         =    %lf\r\n",(float)calib_values_log->tx_ph_2G/10000);
	fprintf(fptr,"TX_gain_2G          =    %lf\r\n",(float)calib_values_log->tx_g_2G/10000);
	for(k=0 ; k<ch_cnt ; k++)
	{
		if(((calib_values_log->rx_channel[k] & 0x00FF) >= 1) && ((calib_values_log->rx_channel[k] & 0x00FF) <= 14))
		{
			fprintf(fptr,"RX_VRMS_2G          =    %d\r\n",(unsigned int)calib_values_log->var);
			fprintf(fptr,"RX_Gain_Off_2G      =    %d\r\n",calib_values_log->rx_vrms_offset_2G);
			break;
		}
	}
	if(k == ch_cnt)
	{
		fprintf(fptr,"RX_Gain_Off_2G      =    NA\r\n");
		fprintf(fptr,"RX_VRMS_2G          =    NA\r\n");
	}
	if( module_type == 3 )
	{
		fprintf(fptr,"TX_Gain_Off_5G_1    =    %d\r\n",(int_8)calib_values_log->gain_offset_5G_1);
		fprintf(fptr,"TX_Gain_Off_5G_2    =    %d\r\n",(int_8)calib_values_log->gain_offset_5G_2);
		fprintf(fptr,"TX_Gain_Off_5G_3    =    %d\r\n",(int_8)calib_values_log->gain_offset_5G_3);
		fprintf(fptr,"TX_Gain_Off_5G_4    =    %d\r\n",(int_8)calib_values_log->gain_offset_5G_4);
		fprintf(fptr,"RX_betacos_5G       =    0x%x\r\n",(uint_16)rx_betacos_5G);
		fprintf(fptr,"RX_betasin_5G       =    0x%x\r\n",(uint_16)rx_betasin_5G);
		fprintf(fptr,"TX_DCI_5G           =    0x%x\r\n",(uint_16)calib_values_log->tx_dci_5G);
		fprintf(fptr,"TX_DCQ_5G           =    0x%x\r\n",(uint_16)calib_values_log->tx_dcq_5G);
		fprintf(fptr,"TX_betacos_5G       =    0x%x\r\n",(uint_16)calib_values_log->tx_betacos_5G);
		fprintf(fptr,"TX_betasin_5G       =    0x%x\r\n",(uint_16)calib_values_log->tx_betasin_5G);
		fprintf(fptr,"TX_phase_5G         =    %lf\r\n",(float)calib_values_log->tx_ph_5G/10000);
		fprintf(fptr,"TX_gain_5G          =    %lf\r\n",(float)calib_values_log->tx_g_5G/10000);
		for(k=0 ; k<ch_cnt ; k++)
		{
			if(((calib_values_log->rx_channel[k] & 0x00FF) >= 36) && ((calib_values_log->rx_channel[k] & 0x00FF) < 60))
			{
				fprintf(fptr,"RX_VRMS_5G_1        =    %d\r\n",(unsigned int)calib_values_log->var_5g);
				fprintf(fptr,"RX_Gain_Off_5G_1    =    %d\r\n",calib_values_log->rx_vrms_offset_5G_1);
				break;
			}
		}
		if(k == ch_cnt)
		{
			fprintf(fptr,"RX_VRMS_5G_1        =    NA\r\n");
			fprintf(fptr,"RX_Gain_Off_5G_1    =    NA\r\n");
		}
		for(k=0 ; k<ch_cnt ; k++)
		{
			if(((calib_values_log->rx_channel[k] & 0x00FF) >= 60) && ((calib_values_log->rx_channel[k] & 0x00FF) < 100))
			{
				fprintf(fptr,"RX_VRMS_5G_2        =    %d\r\n",(unsigned int)calib_values_log->var_5g_2);
				fprintf(fptr,"RX_Gain_Off_5G_2    =    %d\r\n",calib_values_log->rx_vrms_offset_5G_2);
				break;
			}
		}
		if(k == ch_cnt)
		{
			fprintf(fptr,"RX_VRMS_5G_2        =    NA\r\n");
			fprintf(fptr,"RX_Gain_Off_5G_2    =    NA\r\n");
		}
		for(k=0 ; k<ch_cnt ; k++)
		{
			if(((calib_values_log->rx_channel[k] & 0x00FF) >= 100) && ((calib_values_log->rx_channel[k] & 0x00FF) < 149))
			{
				fprintf(fptr,"RX_VRMS_5G_3        =    %d\r\n",(unsigned int)calib_values_log->var_5g_3);
				fprintf(fptr,"RX_Gain_Off_5G_3    =    %d\r\n",calib_values_log->rx_vrms_offset_5G_3);
				break;
			}
		}
		if(k == ch_cnt)
		{
			fprintf(fptr,"RX_VRMS_5G_3        =    NA\r\n");
			fprintf(fptr,"RX_Gain_Off_5G_3    =    NA\r\n");
		}
		for(k=0 ; k<ch_cnt ; k++)
		{
			if(((calib_values_log->rx_channel[k] & 0x00FF) >= 149) && ((calib_values_log->rx_channel[k] & 0x00FF) <= 165))
			{
				fprintf(fptr,"RX_VRMS_5G_4        =    %d\r\n",(unsigned int)calib_values_log->var_5g_4);
				fprintf(fptr,"RX_Gain_Off_5G_4    =    %d\r\n",calib_values_log->rx_vrms_offset_5G_4);
				break;
			}
		}
		if(k == ch_cnt)
		{
			fprintf(fptr,"RX_VRMS_5G_4        =    NA\r\n");
			fprintf(fptr,"RX_Gain_Off_5G_4    =    NA\r\n");
		}
	}
	fprintf(fptr,"RF_temp_coef        =    %f\r\n",(calib_values_log->rf_int)+(float)(calib_values_log->rf_fra)/100);
	fprintf(fptr,"ULP_temp_coef       =    %f\r\n",(calib_values_log->ulp_int)+(float)(calib_values_log->ulp_fra)/100);

	fprintf(fptr,"\r\n\r\n");
	fprintf(fptr,"RSI_EEPROM_SIZE     =    %d\r\n",module_params.eeprom_size);
	fprintf(fptr,"RSI_EEPROM_TYPE     =    %s\r\n",module_params.eeprom_type ? "MICRON" : "EON");
	fprintf(fptr,"RSI_EEPROM_VER      =    %d\r\n",module_params.eeprom_version);
	fprintf(fptr,"RSI_DIGI_CHIP_VER   =    %d\r\n",module_params.digi_chip_ver);
	fprintf(fptr,"RSI_MODULE_VER      =    %d\r\n",module_params.module_ver);
	fprintf(fptr,"RSI_RF_CHIP_VER     =    %d\r\n",module_params.rf_chip_ver);
	fprintf(fptr,"RSI_MFG_SW_VER      =    %d\r\n",module_params.mfg_sw_ver);
	fprintf(fptr,"\r\n\r\n");

	if( fail == 2 )
	{
		fprintf(fptr,"Unable to flash Calibration Values\r\n");
		fprintf(fptr,"\r\n\t\t\t* * * * * *\r\n");
		fprintf(fptr,"\t\t\t  FAILED\r\n");
		fprintf(fptr,"\t\t\t* * * * * *\r\n\r\n");
	}
	else if( fail == 1 )
	{
		switch(calib_values_log->adapter_state)
		{
			case XO_CTUNE:
				//    printf ("\033[1m\033[31m \n\n\tXO Ctune FAILED\033[0m\r\n");
				fprintf(fptr,"XO Ctune FAILED\r\n");
				break;
			case GAIN_OFF_ESTI:
				//    printf ("\033[1m\033[31m \n\n\tGAIN_OFFSET ESTIMATION FAILED\033[0m\r\n");
				fprintf(fptr,"GAIN_OFFSET ESTIMATION FAILED\r\n");
				break;
			case TX_IQ_IMB:
				//    printf ("\033[1m\033[31m \n\n\tTXIQ FAILED\033[0m\r\n");
				fprintf(fptr,"TXIQ FAILED\r\n");
				break;
			case RX_IQ_IMB:
				//     printf ("\033[1m\033[31m \n\n\tRXIQ FAILED\033[0m\r\n");
				fprintf(fptr,"RXIQ FAILED\r\n");
				break;	
			case VRMS_2G:
				//     printf ("\033[1m\033[31m \n\n\tRXIQ FAILED\033[0m\r\n");
				fprintf(fptr,"Bad Receive VRMS_2G\r\n");
				break;
			case VRMS_5G:
				//     printf ("\033[1m\033[31m \n\n\tRXIQ FAILED\033[0m\r\n");
				fprintf(fptr,"Bad Receive VRMS_5G\r\n");
				break;
			case GPIO_TEST:
				//     printf ("\033[1m\033[31m \n\n\tGPIO_TEST FAILED\033[0m\r\n");
				fprintf(fptr,"GPIO TEST FAILED\r\n");
				break;
			case RF_TEMP_CALIB:
				//     printf ("\033[1m\033[31m \n\n\tRF_TEMP CALIB FAILED\033[0m\r\n");
				fprintf(fptr,"RF_TEMP CALIB FAILED\r\n");
				break;
			case ULP_TEMP_CALIB:
				//     printf ("\033[1m\033[31m \n\n\tULP_TEMP CALIB FAILED\033[0m\r\n");
				fprintf(fptr,"ULP_TEMP CALIB FAILED\r\n");
				break;
			case CALIB_DONE:
				break;
			default:
				//     printf ("\033[1m\033[31m \n\t\tUART_LINK FAILED\033[0m\r\n");
				fprintf(fptr,"UART LINK FAILED\r\n");
				break;

		}
		fprintf(fptr,"\r\n\t\t\t* * * * * *\r\n");
		fprintf(fptr,"\t\t\t  FAILED\r\n");
		fprintf(fptr,"\t\t\t* * * * * *\r\n\r\n");
	}
	else if( fail == 3 )
	{
		fprintf(fptr,"OTP Burning Failed\r\n");
		fprintf(fptr,"\r\n\t\t\t* * * * * *\r\n");
		fprintf(fptr,"\t\t\t  FAILED\r\n");
		fprintf(fptr,"\t\t\t* * * * * *\r\n\r\n");
	}

	if( module_params.boot_load )
	{
		fprintf(fptr,"Module is in Embedded Mode\r\n\r\n");
		fprintf(fptr,"Firmware Details");
		while(ep = readdir(dp))
		{
			if( strstr(ep->d_name,".rps"))
			{
#ifdef DRAGGER
				if(strstr(ep->d_name,"RS9113.NBZ.WC.DRG"))
#elif NO_FIPS
					if(strstr(ep->d_name,"RS9113.NBZ.WC.FIPS"))
#elif WISE_MCU
					if(strstr(ep->d_name,"RS9113.NBZ.WC.GEN"))
#elif WISE_MCU_8X8
					if(strstr(ep->d_name,"RS9113.NBZ.WC.GEN"))
#else
						if(strstr(ep->d_name,"RS9113.NBZ.WC.GEN"))
#endif
						{
							fprintf(fptr,"-----> %s\r\n\r\n",ep->d_name);
						}
			}
		}
		fprintf(fptr,"Bootloader Details");
#ifdef BL_RELEASE
		closedir(dp);
		dp = opendir("../../../../release/flash/WC/");
		if( dp == NULL )
		{
			perror("opendir");
			return;
		}
		while(ep = readdir(dp))
		{
			if(strstr(ep->d_name,"RS9113_WC_BL_"))
			{
				fprintf(fptr,"-----> %s\r\n\r\n",ep->d_name);
			}
		}
#else
		closedir(dp);
		dp = opendir("../release/flash/WC/");
		if( dp == NULL )
		{
			perror("opendir");
			return;
		}
		while(ep = readdir(dp))
		{
			if( strstr(ep->d_name,".rps"))
			{
				if(strstr(ep->d_name,"RS9113_WC_BL_"))
				{
					fprintf(fptr,"-----> %s\r\n\r\n",ep->d_name);
				}
			}
		}
#endif
	}
	else
	{
		fprintf(fptr,"Module is in Hosted Mode\r\n\r\n");
		fprintf(fptr,"Bootloader Details");
#ifdef BL_RELEASE
		closedir(dp);
		dp = opendir("../../../../release/flash/WC/");
		if( dp == NULL )
		{
			perror("opendir");
			return;
		}
		while(ep = readdir(dp))
		{
			if(strstr(ep->d_name,"RS9113_WC_BL_"))
			{
				fprintf(fptr,"-----> %s\r\n\r\n",ep->d_name);
			}
		}
#else
		while(ep = readdir(dp))
		{
			if( strstr(ep->d_name,".rps"))
			{
				if(strstr(ep->d_name,"RS9113_WC_BL_"))
				{
					fprintf(fptr,"-----> %s\r\n\r\n",ep->d_name);
				}
			}
		}
#endif
	}
	if( fail == 0 )
	{
		fprintf(fptr,"Calibration Process Successful\r\n");
		fprintf(fptr,"\r\n\t\t\t* * * * * *\r\n");
		fprintf(fptr,"\t\t\t  PASSED\r\n");
		fprintf(fptr,"\t\t\t* * * * * *\r\n\r\n");
	}

	gettimeofday(&endtime,0x0);
	if( starttime.tv_sec == endtime.tv_sec )
	{
		fprintf(fptr,"Time taken in Milli Seconds is %f\r\n\r\n", (double)(endtime.tv_usec - starttime.tv_usec )/1000 );
	}
	else
	{
		fprintf(fptr,"Time taken in Milli Seconds is %f\r\n\r\n", (double)(((endtime.tv_sec - starttime.tv_sec )*1000000 - starttime.tv_usec) + endtime.tv_usec )/1000 );
	}
	cur_time = time(NULL);

	memcpy(file_name,ctime(&cur_time),strlen(ctime(&cur_time)));

	for(i = 0; i < strlen(ctime(&cur_time)); i++ )
	{
		// if(file_name[i] == ' ')
		// {
		//   if(file_name[i + 1] == ' ')
		//   {
		//   }
		// }
	}
	file_name[i - 1 ] = '\0';

	fprintf(fptr,"Date: %s\r\n",file_name);

	closedir(dp);
	fclose(fptr);

}
int flash_buffer(uint_16 card_no, uint_16 module_type, uint_16 boot_load, calib_params_t *calib_values_slave)
{
  int   count,ret; 
  char str[80],ptr[80],str1[30];
  uint_32 buff;
  //  uint_8 eeprom_version = module_params.eeprom_version;
  //  uint_16 eeprom_size = module_params.eeprom_size;
  //  uint_8 eeprom_type = module_params.eeprom_type;
  //  uint_8 module_append = module_params.module_append;
  //  uint_16 dig_chip_version = module_params.digi_chip_ver;
  //  uint_16 module_version = module_params.module_ver;
  //  uint_16 rf_chip_version = module_params.rf_chip_ver;

  //  printf("\nversion is %d, eeprom_size %d, eeprom_type: %d card_no:%d module_type:%d module_append:%d\n", eeprom_version, eeprom_size,eeprom_type,card_no,module_type,module_append);

  strcpy(str, "sh ../release/flash/run_calib.sh");
  strcpy(ptr, "sh ../release/flash/run_calib_wc.sh");

  strcat(str, " ");
  strcat(ptr, " ");

  sprintf(str1,"%d",module_params.eeprom_version);
  strcat(str,str1);
  strcat(ptr,str1);

  strcat(str," ");
  strcat(ptr," ");

  sprintf(str1,"%d",module_params.eeprom_size);
  strcat(str,str1);
  strcat(ptr,str1);

  strcat(str," ");
  strcat(ptr," ");

  sprintf(str1,"%d",module_params.eeprom_type);
  strcat(str,str1);
  strcat(ptr,str1);

  strcat(str," ");
  strcat(ptr," ");

  sprintf(str1,"%d",module_type);
  strcat(str,str1);
  strcat(ptr,str1);

  strcat(str," ");
  strcat(ptr," ");

  sprintf(str1,"%d",card_no);
  strcat(str,str1);
  strcat(ptr,str1);

  strcat(str," ");
  strcat(ptr," ");

  sprintf(str1,"%d",module_params.module_append);
  strcat(str,str1);
  strcat(ptr,str1);

  strcat(str," ");
  strcat(ptr," ");

  sprintf(str1,"%d",module_params.digi_chip_ver);
  strcat(str,str1);
  strcat(ptr,str1);

  strcat(str," ");
  strcat(ptr," ");

  sprintf(str1,"%d",module_params.module_ver);
  strcat(str,str1);
  strcat(ptr,str1);

  strcat(str," ");
  strcat(ptr," ");

  sprintf(str1,"%d",module_params.rf_chip_ver);
  strcat(str,str1);
  strcat(ptr,str1);

  strcat(str," ");
  strcat(ptr," ");

  sprintf(str1,"%d",module_params.flash_check);
  strcat(str,str1);
  strcat(ptr,str1);

  strcat(str," ");
  strcat(ptr," ");
#ifdef BL_RELEASE
  sprintf(str1,"%d",1);
  strcat(str,str1);
  strcat(ptr,str1);
#else
  sprintf(str1,"%d",0);
  strcat(str,str1);
  strcat(ptr,str1);
#endif
  strcat(str," ");
  strcat(ptr," ");
#ifdef WISE_MCU
  if( module_type == 2)
  {
	  sprintf(str1,"%d",3);
	  strcat(str,str1);
	  strcat(ptr,str1);
  }
  else if( module_type == 3 )
  {
	  sprintf(str1,"%d",4);
	  strcat(str,str1);
	  strcat(ptr,str1);
  }
#elif WISE_MCU_8X8
  {
	  sprintf(str1,"%d",3);
	  strcat(str,str1);
	  strcat(ptr,str1);
  } 
#elif DRAGGER
  sprintf(str1,"%d",1);
  strcat(str,str1);
  strcat(ptr,str1);
#elif NO_FIPS
  sprintf(str1,"%d",2);
  strcat(str,str1);
  strcat(ptr,str1);
#else
  sprintf(str1,"%d",0);
  strcat(str,str1);
  strcat(ptr,str1);
#endif

  strcat(str," ");
  strcat(ptr," ");

  sprintf(str1,"%d",module_params.mfg_sw_ver);
  strcat(str,str1);
  strcat(ptr,str1);

  strcat(str," ");
  strcat(ptr," ");

#ifdef ANT_0
  sprintf(str1,"%d",0);
  strcat(str,str1);
  strcat(ptr,str1);
#elif ANT_1
  sprintf(str1,"%d",1);
  strcat(str,str1);
  strcat(ptr,str1);
#endif

#if 1
  //script_start = time(NULL);	
  if(!boot_load)
    ret=system(str);
  else
    ret=system(ptr);

  //	script_stop = time(NULL);	
  //printf("\n=========>Time taken for returning from script file: %2f Sec\n",difftime(script_stop,script_start));
  //	printf("\nreturn : %d\n",ret);
  if(ret == 0)
  {
    printf("\nSuccess in returning from script file\n");
    //buff = 0;
  }
  else if(ret == 256)
  {
    printf ("\033[1m\033[31m \n\tError Return1 : Unable To Create Flash File\033[0m\n");
    //buff = 1 << 24;
    return -1;
  }
  else if(ret == 512)
  {
    printf ("\033[1m\033[31m \n\tError Return2 : Upgrade Failed\033[0m\n");
    //buff = 2 << 24;
    return -1;
  }
  else
  {
    printf ("\033[1m\033[31m \n\tError Return3 : Upgrade Failed\033[0m\n");
    //buff = (ret/256) << 24;
    return -1;
  }
#endif
  prepare_log_file(calib_values_slave,0,module_type);
#if 0        
  if(!type)
    system("sh ../release/flash/run_calib.sh ver e_size");
  else
    system("sh ../release/flash/run_calib_wc.sh ver e_size");
#endif
  return 0;
}
#if 0
int process_otp_write()
{
  uint_8 otp_buffer[] = {
#include "buffer_otp.txt"
  };
  uint_16 length; 
  struct bb_rf_param_t  bb_rf_params;
  uint_16 addr_otp = 0, block_size = 100, no_of_blocks,write_block, i,ii,k;
  struct iwreq iwr;
  uint_32 sockfd = 0;

  memset(&bb_rf_params,0,sizeof(struct bb_rf_param_t));
  memset(&iwr,0,sizeof(iwr));

  length = (sizeof(otp_buffer)/sizeof(otp_buffer[0]));
  no_of_blocks = (length/block_size);

  sockfd = onebox_open_interface();
  k = 0;

  for(i = 0; i <=no_of_blocks; i++ )
  {
    if( i == no_of_blocks )
    {
      write_block = (length % block_size);
      bb_rf_params.soft_reset = 1;/*Used to indicate last Frame */
    }
    else
    {
      write_block = block_size;
      bb_rf_params.soft_reset = 0;
    }
    bb_rf_params.Data[0] = addr_otp;
    for( ii = 1; ii <=write_block; ii++, k++)
    {
      bb_rf_params.Data[ii] = otp_buffer[k];
    }
    addr_otp += write_block;
    // printf("write_block = %d\n",write_block);
    // printf(" k = %d\n",k);
    // printf(" addr_otp = %d\n",addr_otp);
    bb_rf_params.value = OTP_WRITE;
    bb_rf_params.no_of_values = write_block;
    strncpy(iwr.ifr_name,"rpine1",IFNAMSIZ);
    iwr.u.data.pointer = &bb_rf_params;
    iwr.u.data.length = write_block; 

    if(ioctl(sockfd, ONEBOX_SET_BB_RF, &iwr) == -1 )
    {
      //printf("OTP_WRITE_FAILED\n");
      return -1;
    }
    else
    {
      //printf("write_successful\n");
    }

  }

}
#endif
int prepare_buffer(calib_params_t *calib_values_burn, uint_8 module_type)
{
  FILE *fptr = NULL;
  FILE *fcpy = NULL;
  int i =0,j = 0;
  uint_8 value[10];
  uint_8 split_data[10];
  unsigned long long int tx_iq = 0;
  unsigned long long int rx_iq = 0;



  if( (fptr = fopen("calib_values.txt","r" )) == NULL )
  {
    perror("fopen");
    return -1;
  }
  if( (fcpy = fopen("../release/flash/RS9113_RS8111_calib_values.txt","w" )) == NULL )
  {
    perror("fopen");
    return -1;
  }

  i = 0;
  while( fgets(value,10,fptr) != NULL )
  {
    i++;
    switch( i )
    {
      case TYPE_OFFSET:
        if( 2 == module_type )
        {
#ifdef WISE_MCU_8x8
          fprintf(fcpy,"0x11,\n");
#elif WISE_MCU
          fprintf(fcpy,"0x09,\n");
#else
          fprintf(fcpy,"0x01,\n");
#endif
        }
        else if( 3 == module_type)
        {
        
#ifndef WISE_MCU
			fprintf(fcpy,"0x03,\n");
#else
			fprintf(fcpy,"0x0B,\n");
#endif                        
		}

#if 0
        if( 2 == module_type )
        {
#ifndef WISE_MCU
          fprintf(fcpy,"0x01,\n");
#else
          fprintf(fcpy,"0x09,\n");
#endif
        }
        else if( 3 == module_type)
        {
#ifndef WISE_MCU
          fprintf(fcpy,"0x03,\n");
#else
          fprintf(fcpy,"0x0B,\n");
#endif
        }
#endif
        break;
      case XO_OFFSET:
        fprintf(fcpy,"0x%x%s\n",(uint_8)calib_values_burn->ctune_val,",");
        break;
      case TX_IQ_2G_20:
      case TX_IQ_2G_40:
        bzero(split_data,10);
        tx_iq = 0;
        rx_iq = 0;
        rx_iq = rx_betasin << 10;
        rx_iq |= rx_betacos;
        tx_iq = calib_values_burn->tx_dcq_2G << 20;
        //        printf("size is %d\n",sizeof(tx_iq));
        //        printf("%x\n",tx_iq);
        tx_iq = (unsigned long long)tx_iq << 10;
        //        printf("%llx\n",(unsigned long long)tx_iq);
        tx_iq |= calib_values_burn->tx_dci_2G << 20;
        tx_iq |= calib_values_burn->tx_betasin_2G <<10;
        tx_iq |= calib_values_burn->tx_betacos_2G;
        //        printf("%llx\n",tx_iq);
        memcpy(split_data,&tx_iq,sizeof(tx_iq));
        memcpy(&split_data[5],&rx_iq,sizeof(rx_iq));
        for( j = 0; j<7 ; j++ )
        {
          fprintf(fcpy,"0x%x%s\n",(uint_8)split_data[j],",");
          fgets(value,10,fptr);
        }
        fprintf(fcpy,"0x%x%s\n",(uint_8)split_data[j],",");
        i += 7;
        break;
      case TX_IQ_5G_1_20:
      case TX_IQ_5G_1_40:
      case TX_IQ_5G_2_20:
      case TX_IQ_5G_2_40:
      case TX_IQ_5G_3_20:
      case TX_IQ_5G_3_40:
      case TX_IQ_5G_4_20:
      case TX_IQ_5G_4_40:
        // if( module_params.type == 3 )
        if(1)
        {
          bzero(split_data,10);
          rx_betacos_5G = 0x100;
          rx_betasin_5G = 0x00;
          tx_iq = 0;
          rx_iq = 0;
          rx_iq = rx_betasin_5G << 10;
          rx_iq |= rx_betacos_5G;
          tx_iq = calib_values_burn->tx_dcq_5G << 20;
          tx_iq = (unsigned long long)tx_iq << 10;
          tx_iq |= calib_values_burn->tx_dci_5G << 20;
          tx_iq |= calib_values_burn->tx_betasin_5G <<10;
          tx_iq |= calib_values_burn->tx_betacos_5G;
          memcpy(split_data,&tx_iq,sizeof(tx_iq));
          memcpy(&split_data[5],&rx_iq,sizeof(rx_iq));
          for( j = 0; j<7 ; j++ )
          {
            fprintf(fcpy,"0x%x%s\n",(uint_8)split_data[j],",");
            fgets(value,10,fptr);
          }
          fprintf(fcpy,"0x%x%s\n",(uint_8)split_data[j],",");
          i += 7;
        }
        else
        {
          fprintf(fcpy,"%s\n","0xff,");
        }
        break;
      case GAIN_OFFSET_2G:
        fprintf(fcpy,"0x%x%s\n",(uint_8)calib_values_burn->gain_offset_2G,",");
        break;
      case GAIN_OFFSET_5G_1:
        // if( module_params.type == 3 )
        if(1)
        {
          fprintf(fcpy,"0x%x%s\n",(uint_8)calib_values_burn->gain_offset_5G_1,",");
        }
        else
        {
          fprintf(fcpy,"%s\n","0xff,");
        }
        break;
      case GAIN_OFFSET_5G_2:
        // if( module_params.type == 3 )
        if(1)
        {
          fprintf(fcpy,"0x%x%s\n",(uint_8)calib_values_burn->gain_offset_5G_2,",");
        }
        else
        {
          fprintf(fcpy,"%s\n","0xff,");
        }
        break;
      case GAIN_OFFSET_5G_3:
        // if( module_params.type == 3 )
        if(1)
        {
          fprintf(fcpy,"0x%x%s\n",(uint_8)calib_values_burn->gain_offset_5G_3,",");
        }
        else
        {
          fprintf(fcpy,"%s\n","0xff,");
        }
        break;
      case GAIN_OFFSET_5G_4:
        // if( module_params.type == 3 )
        if(1)
        {
          fprintf(fcpy,"0x%x%s\n",(uint_8)calib_values_burn->gain_offset_5G_4,",");
        }
        else
        {
          fprintf(fcpy,"%s\n","0xff,");
        }
        break;
      case RF_TEMP_CONST_INT:
        fprintf(fcpy,"0x%x%s\n",(uint_8)calib_values_burn->rf_int,",");
        break;
      case RF_TEMP_CONST_FRA:
        fprintf(fcpy,"0x%x%s\n",(uint_8)calib_values_burn->rf_fra,",");
        break;
      case ULP_TEMP_CONST_INT:
        fprintf(fcpy,"0x%x%s\n",(uint_8)calib_values_burn->ulp_int,",");
        break;
      case ULP_TEMP_CONST_FRA:
        fprintf(fcpy,"0x%x%s\n",(uint_8)calib_values_burn->ulp_fra,",");
        break;
      default:
        fprintf(fcpy,"%s",value);
        break;
    }
  }
  fclose(fptr);
  fclose(fcpy);
  system("cp calib_values.txt abc.txt");

  return 0;
}

int_32 rf_write(int sfd,uint_32 soft_rst, uint_32 num_of_regs, uint_32 vals_per_reg, uint_32 *BUFFER)
{
  uint_16 val, i, j=0 ,index =0, k,ii, rf_len;
  uint_8 blocks, count,count1, write_block = 20;
  struct iwreq wrq;        
  struct bb_rf_param_t  bb_rf_params;

  val = (num_of_regs * vals_per_reg);
  rf_len = val; 
  blocks = (num_of_regs/(write_block)); 
  count = (num_of_regs%(write_block)); 
  for ( i=0; i<=blocks ; i++)
  {
    if (i == blocks)
    {
      count1 = count;  
    } 
    else 
    {
      count1 = write_block;  
    }     
    index = j;
    for (k=1,ii=1 ; ii<count1*5; j+=5,k++,ii+=5)
    {
      bb_rf_params.Data[ii] = *(uint_16 *)&BUFFER[j];
      bb_rf_params.Data[ii+1] = *(uint_16 *)&BUFFER[j+3];
      bb_rf_params.Data[ii+2] = *(uint_16 *)&BUFFER[j+2];
      bb_rf_params.Data[ii+3] = *(uint_16 *)&BUFFER[j+1];
      bb_rf_params.Data[ii+4] = *(uint_16 *)&BUFFER[j+4];
    }
    rf_len = (count1 );
    strncpy(wrq.ifr_name,"rpine1",IFNAMSIZ);
    bb_rf_params.value = RF_WRITE;
    bb_rf_params.no_of_fields = vals_per_reg;
    bb_rf_params.no_of_values = rf_len;
    bb_rf_params.soft_reset = (uint_8)soft_rst;
    wrq.u.data.pointer = &bb_rf_params;
    wrq.u.data.length = (rf_len); 
    if(ioctl(sfd,ONEBOX_SET_BB_RF,&wrq)<0)
    {  
      printf("Error writing to RF\n");
  	  return ONEBOX_STATUS_FAILURE;
    }  
    else
    { 
      return ONEBOX_STATUS_SUCCESS;
    }
  }  
}


int_32 rf_read(int sfd,uint_32 soft_rst, uint_32 num_of_regs, uint_32 vals_per_reg, uint_32 *BUFFER,uint_32 *read_word)
{

  uint_16 val, i, j = 0, index = 0, ii, k, kk = 0,  rf_len;
  uint_8 blocks, count,count1, write_block = 20;
  struct iwreq wrq;        
  struct bb_rf_param_t  bb_rf_params, bb_rf_print;
  uint_32 swap;

  val = (num_of_regs * vals_per_reg);
  rf_len = val; 
  blocks = (num_of_regs/(write_block)); 
  count = (num_of_regs%(write_block)); 
  for ( i=0; i<=blocks ; i++)
  {
    if (i == blocks)
    {
      count1 = count;  
    } 
    else 
    {
      count1 = write_block;  
    }      
    index = j;
    for (ii=1,k=1 ; ii<count1*5; j+=5,k++,ii+=5)
    {
      bb_rf_params.Data[ii] = *(uint_16 *)&BUFFER[j];
      bb_rf_params.Data[ii+1] = *(uint_16 *)&BUFFER[j+3];
      bb_rf_params.Data[ii+2] = (*(uint_16 *)&BUFFER[j+2] | BIT(15));
      bb_rf_params.Data[ii+3] = *(uint_16 *)&BUFFER[j+1];
      bb_rf_params.Data[ii+4] = *(uint_16 *)&BUFFER[j+4];
    }
    rf_len = (count1 );
    strncpy(wrq.ifr_name,"rpine1",IFNAMSIZ);
    bb_rf_params.value = RF_READ;
    bb_rf_params.no_of_fields = vals_per_reg;
    bb_rf_params.no_of_values = rf_len;
    bb_rf_params.soft_reset = (uint_8)soft_rst;
    wrq.u.data.pointer = &bb_rf_params;
    wrq.u.data.length = sizeof(bb_rf_params);



    if(ioctl(sfd,ONEBOX_SET_BB_RF,&wrq)<0)
    {  
      printf("Error reading from RF\n");
      return ONEBOX_STATUS_FAILURE;
    }  
    else
    { 
	  for(ii=0;ii<rf_len;ii+=5)
      { 
        swap = bb_rf_params.Data[ii];
        bb_rf_print.Data[ii] = ((swap & 0x00FF) << 8) | ((swap & 0xFF00) >> 8);
        kk++;
        swap = bb_rf_params.Data[ii+1];
        bb_rf_print.Data[ii+1] = ((swap & 0x00FF) << 8) | ((swap & 0xFF00) >> 8);
        kk++;
      }
    }
	*read_word |= bb_rf_params.Data[0];
	*read_word |= bb_rf_params.Data[1] << 16;
	return ONEBOX_STATUS_SUCCESS;
  }  
}
 
int_32  rsi_rf_temp_coeff(int sfd)
{
	struct bb_rf_param_t  bb_rf_params, bb_rf_print;
	double k = 1316-27;
	double b = 0;
	double c = 0;
	float x = 0;
	float cnt_f1 = 0,cnt_f2 = 0;
	int i=0,ii=0,j=0;

	uint_32 soft_rst;
	uint_32 vals_per_reg;
	uint_32 num_of_regs;
	uint_16 rf_read_vals[] = {0x00,0x00,0x00,0x00,0x00};
	uint_32 BUFFER[5];
	uint_32 *rf_vals;
	uint_16 rf_reg_write1[] = {0x20, 0x0, 0x5589, 0xfe00, 0xa};
	uint_16 rf_reg_write2[] = {0x20, 0x0, 0x56e0, 0x0, 0xa};

	uint_16 rf_reg_read1[] = {0x20, 0x0, 0xd700, 0x0, 0x1};
	rf_vals = (uint_32 *)rf_read_vals;


	soft_rst = 0;
	vals_per_reg = 5;
	num_of_regs = 1;
	for(j=0;j<4;j++)
  {
    BUFFER[j]=rf_reg_write1[j];
  }
	if(rf_write(sfd,soft_rst,num_of_regs,vals_per_reg,BUFFER)<0)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	sleep(2);
	
	for(j=0;j<4;j++)
  {
    BUFFER[j]=rf_reg_write2[j];
  }
	if(rf_write(sfd,soft_rst,num_of_regs,vals_per_reg,BUFFER)<0)
	{
		return ONEBOX_STATUS_FAILURE;
	}
	sleep(1);

	for(j=0;j<4;j++)
  {
    BUFFER[j]=rf_reg_read1[j];
  }
	if(rf_read(sfd,soft_rst,num_of_regs,vals_per_reg,BUFFER,rf_vals)<0)
	{
		return ONEBOX_STATUS_FAILURE;
	}


	cnt_f2 = ((rf_read_vals[0] & 0xffc) >> 2) ;
	cnt_f1 = (((rf_read_vals[1] & 0x3f) << 4) +((rf_read_vals[0] & 0xf000) >> 12));
	if( cnt_f1 == 255 )
	{
		b = 553350 - (2*k*cnt_f2);
		c = 55752435 - (553350*cnt_f2) + k*(cnt_f2*cnt_f2);

		b = (double)b/100;
		c = (double)c/100;
		k = (double)k/100;
		x = (-(b*100) - (sqrt((b*b)-(4*k*c))*100))/(2*k*100);
		x_rf = x;
	}
	return 0;
}


int_32 ulp_write(int sfd,uint_32 soft_rst, uint_32 num_of_regs, uint_32 vals_per_reg, uint_32 *BUFFER)
{
  uint_16 val, i, j = 0, index = 0, ii, k, ulp_len;
  uint_8 blocks, count,count1, write_block = 20;
  struct iwreq wrq;        
  struct bb_rf_param_t  bb_rf_params;

  val = (num_of_regs * vals_per_reg);
  ulp_len = val; 
  blocks = (num_of_regs/(write_block)); 
  count = (num_of_regs%(write_block)); 
  for ( i=0; i<=blocks ; i++)
  {
    if (i == blocks)
    {
      count1 = count;  
    } 
    else 
    {
      count1 = write_block;  
    }      
    index = j;
    for (ii=1,k=1 ; ii<count1*5; j+=5,k++,ii+=5)
    {
      bb_rf_params.Data[ii] = *(uint_16 *)&BUFFER[j];
      bb_rf_params.Data[ii+1] = *(uint_16 *)&BUFFER[j+1];
      bb_rf_params.Data[ii+2] = *(uint_16 *)&BUFFER[j+2];
      bb_rf_params.Data[ii+3] = *(uint_16 *)&BUFFER[j+3];
      bb_rf_params.Data[ii+4] = *(uint_16 *)&BUFFER[j+4];
    }
    ulp_len = (count1);
    strncpy(wrq.ifr_name,"rpine1",IFNAMSIZ);
    bb_rf_params.value = 5;
    bb_rf_params.no_of_values = ulp_len;
    bb_rf_params.soft_reset = (uint_8)soft_rst;
    bb_rf_params.no_of_fields = vals_per_reg;
    wrq.u.data.pointer = &bb_rf_params;
    wrq.u.data.length = (ulp_len); 
    if(ioctl(sfd,ONEBOX_SET_BB_RF,&wrq)<0)
    {  
      printf("Error writing to ULP\n");
      return ONEBOX_STATUS_FAILURE;
    }  
    else
    {
      return ONEBOX_STATUS_SUCCESS;
    }
  }  
}


int_32 ulp_read(int sfd,uint_32 soft_rst, uint_32 num_of_regs, uint_32 vals_per_reg, uint_32 *BUFFER,uint_32 *read_word)
{

  uint_16 val, i, j = 0, index = 0, ii , k, m, ulp_len, swap;
  uint_8 blocks, count,count1, write_block = 20;
  struct iwreq wrq;        
  struct bb_rf_param_t  bb_rf_params;

  val = (num_of_regs * vals_per_reg);
  ulp_len = val; 
  blocks = (num_of_regs/(write_block)); 
  count = (num_of_regs%(write_block)); 
  for ( i=0; i<=blocks ; i++)
  {
    if (i == blocks)
    {
      count1 = count;  
    } 
    else 
    {
      count1 = write_block;  
    }      
    index = j;
    for (ii=1,k=1 ; ii<count1*5; j+=5,k++,ii+=5)
    {
      bb_rf_params.Data[ii] = *(uint_16 *)&BUFFER[j];
      bb_rf_params.Data[ii+1] = *(uint_16 *)&BUFFER[j+1];
      bb_rf_params.Data[ii+2] = *(uint_16 *)&BUFFER[j+2];
      bb_rf_params.Data[ii+3] = *(uint_16 *)&BUFFER[j+3];
      bb_rf_params.Data[ii+4] = *(uint_16 *)&BUFFER[j+4];
    }
    ulp_len = (count1 );
    strncpy(wrq.ifr_name,"rpine1",IFNAMSIZ);
    bb_rf_params.value = 4;
    bb_rf_params.no_of_values = ulp_len;
    bb_rf_params.soft_reset = (uint_8)soft_rst;
    bb_rf_params.no_of_fields = vals_per_reg;
    wrq.u.data.pointer = &bb_rf_params;
    wrq.u.data.length = (ulp_len); 
    if(ioctl(sfd,ONEBOX_SET_BB_RF,&wrq)<0)
    {  
      printf("Error reading from ULP\n");
	  return ONEBOX_STATUS_FAILURE;
    }  
    else
    { 
      for(m=0,k=0;m<ulp_len;m+=5,k+=2)
      {
        bb_rf_params.Data[k+1] = (bb_rf_params.Data[m+2]);
        swap = bb_rf_params.Data[k+1];
        bb_rf_params.Data[k+1] = ((swap & 0x00FF) << 8) | ((swap & 0xFF00) >> 8);
        bb_rf_params.Data[k] =(bb_rf_params.Data[m+3]);
        swap = bb_rf_params.Data[k];
        bb_rf_params.Data[k] = ((swap & 0x00FF) << 8) | ((swap & 0xFF00) >> 8);
      }
      for(m=0;m<k;m++)
      {
        if (!bb_rf_params.Data[m])
          bb_rf_params.Data[m] = 0x0000;

      }
      read_word[0] = 0;
      read_word[0] |= bb_rf_params.Data[2];
      read_word[0] |= bb_rf_params.Data[3] << 16;
	  return ONEBOX_STATUS_SUCCESS;
    }
  }  
}

int_32 rsi_ulp_temp_coeff(sfd)
{
  uint_8 iter = 0;
  double k = 0;
  double b = 0;
  double c = 0;
  float x = 0;
  uint_16 cnt_f1 ;
  uint_16 cnt_f2 ;
  int i;

	uint_32 soft_rst;
	uint_32 vals_per_reg;
  uint_32 num_of_regs;
	uint_32 BUFFER[5];
	uint_32 ulp_read_vals = 0;
	uint_32 *ulp_vals;
	ulp_vals = &ulp_read_vals;

	uint_16 ulp_reg_write1[] = {0x15f, 0x0, 0x1, 0x0, 0x1};
	uint_16 ulp_reg_write2[] = {0x15f, 0x0, 0x1, 0x8, 0x1};
	uint_16 ulp_reg_write3[] = {0x15f, 0x0, 0x1, 0xc, 0x1};

	uint_16 ulp_reg_read1[] = {0x33e, 0x0, 0x0, 0x0, 0x1};
	uint_16 ulp_reg_read2[] = {0x33f, 0x0, 0x0, 0x0, 0x1};

	soft_rst = 0;
	vals_per_reg = 5;
  num_of_regs = 1;
  for(i=0;i<5;i++)
  {
    BUFFER[i] = ulp_reg_write1[i];
  }
	if(ulp_write(sfd,soft_rst,num_of_regs,vals_per_reg,BUFFER)<0)
	{
	  return ONEBOX_STATUS_FAILURE;
	}
//	sleep(1);

  for(i=0;i<5;i++)
  {
    BUFFER[i] = ulp_reg_write2[i];
  }
	if(ulp_write(sfd,soft_rst,num_of_regs,vals_per_reg,BUFFER)<0)
	{
	  return ONEBOX_STATUS_FAILURE;
	}
//	sleep(1);

  for(i=0;i<5;i++)
  {
    BUFFER[i] = ulp_reg_write3[i];
  }
	if(ulp_write(sfd,soft_rst,num_of_regs,vals_per_reg,BUFFER)<0)
	{
	  return ONEBOX_STATUS_FAILURE;
	}

	usleep(500);

  for(i=0;i<5;i++)
  {
    BUFFER[i] = ulp_reg_read1[i];
  }
	if(ulp_read(sfd,soft_rst,num_of_regs,vals_per_reg,BUFFER,ulp_vals))
	{
	  return ONEBOX_STATUS_FAILURE;
	}
	sleep(1);

  for(i=0;i<5;i++)
  {
    BUFFER[i] = ulp_reg_read2[i];
  }
	if(ulp_read(sfd,soft_rst,num_of_regs,vals_per_reg,BUFFER,ulp_vals)<0)
	{
	  return ONEBOX_STATUS_FAILURE;
	}
//	sleep(1);

	cnt_f1 = ((ulp_read_vals >> 12) & 0xff);
	cnt_f2 = ((ulp_read_vals >> 2) & 0x3ff);


	if( cnt_f1 == 255 )
	{
		k = (1702-27);
		b = 800190 - (2*k*cnt_f2);
		c = 92010375 - (800190*cnt_f2) + k*(cnt_f2*cnt_f2);

		b = (double)b/100;
		c = (double)c/100;
		k = (double)k/100;

		x = (-(b*100) - (sqrt(((b*b)-(4*k*c)))*100))/(2*k*100); 
		x_ulp= x;
	}
	else if( (cnt_f1 == 0 ) || (cnt_f1 > 255 ) )
	{
	}
	return 0;
}

void get_values(calib_params_t *calib_values)
{

  printf("Enter the Following values in Hexadecimal Format\n");
  printf("enter the xo value : ");
  scanf("%x",&calib_values->ctune_val);

  printf("enter tx_dci_2G : ");
  scanf("%x",&calib_values->tx_dci_2G);

  printf("enter tx_dcq_2G : ");
  scanf("%x",&calib_values->tx_dcq_2G);

  printf("enter tx_betacos_2G : ");
  scanf("%x",&calib_values->tx_betacos_2G);

  printf("enter tx_betasin_2G : ");
  scanf("%x",&calib_values->tx_betasin_2G);

  printf("enter tx_dci_5G : ");
  scanf("%x",&calib_values->tx_dci_5G);

  printf("enter tx_dcq_5G : ");
  scanf("%x",&calib_values->tx_dcq_5G);

  printf("enter tx_betacos_5G : ");
  scanf("%x",&calib_values->tx_betacos_5G);

  printf("enter tx_betasin_5G : ");
  scanf("%x",&calib_values->tx_betasin_5G);

  printf("enter rx_betacos_2G : ");
  scanf("%x",&rx_betacos);

  printf("enter rx_betasin_2G : ");
  scanf("%x",&rx_betasin);

  printf("enter rx_betacos_5G : ");
  scanf("%x",&rx_betacos_5G);

  printf("enter rx_betasin_5G : ");
  scanf("%x",&rx_betasin_5G);

  printf("Enter the Following values in decimal Format\n");
  
  printf("enter gain_offset_2G : ");
  scanf("%hd",&calib_values->gain_offset_2G);

  printf("enter gain_offset_5G band 1 : ");
  scanf("%hd",&calib_values->gain_offset_5G_1);

  printf("enter gain_offset_5G band 2: ");
  scanf("%hd",&calib_values->gain_offset_5G_2);

  printf("enter gain_offset_5G band 3: ");
  scanf("%hd",&calib_values->gain_offset_5G_3);

  printf("enter gain_offset_5G band 4: ");
  scanf("%hd",&calib_values->gain_offset_5G_4);

  printf("enter Integer part of rf temp Coefficient: ");
  scanf("%hd",&calib_values->rf_int);

  printf("enter Fractional part of rf temp Coefficient: ");
  scanf("%hd",&calib_values->rf_fra);

  printf("enter Integer part of ulp temp Coefficient: ");
  scanf("%hd",&calib_values->ulp_int);

  printf("enter Fractional part of ulp temp Coefficient: ");
  scanf("%hd",&calib_values->ulp_fra);
}

void *master_func( void *arg )
{
  int sfd;
  struct iwreq wrq;
  int flag_vrms = 0;
  uint_8 mod_type = 0;
  uint_8 gpio_flag = 0x01;   //Making GPIO-9,10 as High
  int k=0,kk=0;
  uint_8 flag_2g=0;
  uint_8 flag_5g=0;
  unsigned int valid_channels_5_Ghz[]   = { 36, 40, 44, 48, 52, 56, 60, 64, 100,\
									  		  104, 108, 112, 116, 120, 124, 128, 132, 136,\
									          140, 149, 153, 157, 161, 165 
											};

  sfd = onebox_open_interface();
  calib_params_t calib_values;

  memset (&calib_values, 0, sizeof (calib_params_t));
  memset (&wrq, 0, sizeof (struct iwreq));
  if( *((char *)arg) == 2 )
  {
    calib_values.ctune_val = 0 ;
    mod_type =0;
  }
  else if(*((char *)arg) == 3 )
  {
    calib_values.ctune_val = 1 ;
    mod_type =1;
  }
    get_values(&calib_values);
#if 0
  calib_values.cfo_val = module_params.gpio_bypass;
#ifdef SOCK_LOSS
  calib_values.cfo_val = 0xFF;
#endif


  if(rsi_rf_temp_coeff(sfd)<0)
  {
    write(pipe_fd[1],"CLOSE",6);
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    pthread_exit(0);
  }


  for(k=0;k<ch_cnt;k++)
  {
	  if(mod_type == 1)
	  {
		  if((config_values.rx_channel[k] >= 1) && (config_values.rx_channel[k] <= 14))
		  {
			  calib_values.rx_channel[k] = (uint_8)config_values.rx_channel[k];
			  rx_band_2G_en = 1;
        flag_2g = 1;
		  }
		  else 
      {
        for(kk = 0; kk < 24; kk++)
        {
          if(config_values.rx_channel[k] == valid_channels_5_Ghz[kk])
          {
              calib_values.rx_channel[k] = (uint_8)config_values.rx_channel[k];

              if(((calib_values.rx_channel[k] >= 36) && (calib_values.rx_channel[k] < 60)))
              {
                rx_band_5G_1_en = 1;
                flag_5g = 1;
              }
              else if(((calib_values.rx_channel[k] >= 60) && (calib_values.rx_channel[k] < 100)))
              {
                rx_band_5G_2_en = 1;
                flag_5g = 1;
              }
              else if(((calib_values.rx_channel[k] >= 100) && (calib_values.rx_channel[k] < 149)))
              {
                rx_band_5G_3_en = 1;
                flag_5g = 1;
              }
              else if(((calib_values.rx_channel[k] >= 149) && (calib_values.rx_channel[k] <= 165)))
              {
                rx_band_5G_4_en = 1;
                flag_5g = 1;
              }
            break;
          }
        }
        if(kk == 24)
        {
          printf("\033[1m\033[38m""\n\tInvalid Channel issued by user in config_internal.c\n""\033[0m"); 
          write(pipe_fd[1],"CLOSE",6);
          close(pipe_fd[0]);
          close(pipe_fd[1]);
          pthread_exit(0);
        }
      }
	  }
	  else if( mod_type == 0  )
	  {
		  if((config_values.rx_channel[k] >= 1) && (config_values.rx_channel[k] <= 14))
		  {
			  calib_values.rx_channel[0] = (uint_8)config_values.rx_channel[k];
			  rx_band_2G_en = 1;
        calib_values.rx_channel[1] = 0 ;
        calib_values.rx_channel[2] = 0 ;
        calib_values.rx_channel[3] = 0 ;
        calib_values.rx_channel[4] = 0 ;
        flag_2g = 1;
			  break;
		  }
      else
      {
        for(kk = 0; kk < 24; kk++)
        {
          if(config_values.rx_channel[k] == valid_channels_5_Ghz[kk])
          {
            break;
          }
        }
        if(kk == 24)
        {
          printf("\033[1m\033[38m""\n\tInvalid Channel issued by user in config_internal.c\n""\033[0m"); 
          write(pipe_fd[1],"CLOSE",6);
          close(pipe_fd[0]);
          close(pipe_fd[1]);
          pthread_exit(0);
        }
      }
	  }
  }

	if(mod_type == 1)
  {
    if((flag_2g == 0)||(flag_5g ==0))
    {
      printf("\033[1m\033[38m""\n\tChannel configuration missing in config_internal.c\n""\033[0m"); 
      write(pipe_fd[1],"CLOSE",6);
      close(pipe_fd[0]);
      close(pipe_fd[1]);
      pthread_exit(0);
    }
  }
  else if(mod_type == 0)
  {
    if(flag_2g == 0)
    {
      printf("\033[1m\033[38m""\n\tChannel configuration missing in config_internal.c\n""\033[0m"); 
      write(pipe_fd[1],"CLOSE",6);
      close(pipe_fd[0]);
      close(pipe_fd[1]);
      pthread_exit(0);
    }
  }

  strncpy(wrq.ifr_name, "rpine1", 6);
  wrq.u.data.flags = gpio_flag << 8; 
  wrq.u.data.flags |= SET_GPIO; 
  wrq.u.data.length = 1;
  if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
  {
    printf ("\033[1m\033[31m \n\t  Error while Initiating GPIO's \033[0m\n");
    printf ("\033[1m\033[31m \n\tMake Sure Slave Driver is Inserted\033[0m\n");
    write(pipe_fd[1],"CLOSE",6);
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    pthread_exit(0);
    // return;
  }

  strncpy(wrq.ifr_name, "rpine0", 6);
  wrq.u.data.flags = gpio_flag << 8; 
  wrq.u.data.flags |= SET_GPIO; 
  wrq.u.data.length = 1;
  if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
  {
    printf ("\033[1m\033[31m \n\t  Error while Initiating GPIO's \033[0m\n");
    printf ("\033[1m\033[31m \n\tMake Sure Master Driver is Inserted\033[0m\n");
    write(pipe_fd[1],"CLOSE",6);
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    pthread_exit(0);
  }
  
  strncpy(wrq.ifr_name, "rpine0", 6);
  wrq.u.data.flags = START_CALIB; 
  wrq.u.data.pointer = &calib_values;
  wrq.u.data.length = sizeof(calib_params_t);						 /////3;
  if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
  {
    printf ("\033[1m\033[31m \n\t  Error while Initiating Calibration\033[0m\n");
    printf ("\033[1m\033[31m \n\tMake Sure Master Driver is Inserted\033[0m\n");
    write(pipe_fd[1],"CLOSE",6);
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    pthread_exit(0);
  }
  else
  {
	  strncpy(wrq.ifr_name, "rpine0", 6);
	  gpio_flag = 0x00;
	  wrq.u.data.flags = gpio_flag << 8; 
	  wrq.u.data.flags |= SET_GPIO; 
	  if(ioctl(sfd, ONEBOX_HOST_IOCTL, &wrq) < 0) 
	  {
		  printf ("\033[1m\033[31m \n\t  Error while Disabling GPIO's \033[0m\n");
		  write(pipe_fd[1],"CLOSE",6);
		  close(pipe_fd[0]);
		  close(pipe_fd[1]);
		  pthread_exit(0);
	  }
	 
	  
	  if(rsi_ulp_temp_coeff(sfd)<0)
	  {
      write(pipe_fd[1],"CLOSE",6);
      close(pipe_fd[0]);
      close(pipe_fd[1]);
      pthread_exit(0);
	  }

#ifndef SOCK_LOSS
	  printf ("\033[1m\033[35m \n\tXO CALIBRATION\033[0m\n\n");
	  printf("\033[1m\033[38m""cfo_value     \t= %d\n""\033[0m",calib_values.cfo_val);
	  printf("\033[1m\033[38m""ctune_val     \t= %d\n""\033[0m",calib_values.ctune_val);
#endif

	  printf ("\033[1m\033[35m \n\tGAIN OFFSET CALIBRATION\033[0m\n");   



	  calib_values.pow_db_2G = ((float)calib_values.pow_db_2G/10000);
      calib_values.pow_db_5G_1 = ((float)calib_values.pow_db_5G_1/10000); 
      calib_values.pow_db_5G_2 = ((float)calib_values.pow_db_5G_2/10000);
      calib_values.pow_db_5G_3 = ((float)calib_values.pow_db_5G_3/10000);
      calib_values.pow_db_5G_4 = ((float)calib_values.pow_db_5G_4/10000);


    calib_values.pow_db_2G   = calib_values.pow_db_2G  - (config_values.expected_power_2g    - (config_values.rx_gain_offset_2g   + config_values.cable_loss_2g));
    if(mod_type)
    {
      calib_values.pow_db_5G_1 = calib_values.pow_db_5G_1 - (config_values.expected_power_5g_1 - (config_values.rx_gain_offset_5g_1 + config_values.cable_loss_5g_1));
      calib_values.pow_db_5G_2 = calib_values.pow_db_5G_2 - (config_values.expected_power_5g_2 - (config_values.rx_gain_offset_5g_2 + config_values.cable_loss_5g_2));
      calib_values.pow_db_5G_3 = calib_values.pow_db_5G_3 - (config_values.expected_power_5g_3 - (config_values.rx_gain_offset_5g_3 + config_values.cable_loss_5g_3));
      calib_values.pow_db_5G_4 = calib_values.pow_db_5G_4 - (config_values.expected_power_5g_4 - (config_values.rx_gain_offset_5g_4 + config_values.cable_loss_5g_4));
    }
    calib_values.gain_offset_2G   = floor(calib_values.pow_db_2G); 
#ifdef SINGLE_BAND
    calib_values.gain_offset_2G   += 2; 
#endif
    if(mod_type)
    {
      calib_values.gain_offset_5G_1 = floor(calib_values.pow_db_5G_1); 
      calib_values.gain_offset_5G_2 = floor(calib_values.pow_db_5G_2); 
      calib_values.gain_offset_5G_3 = floor(calib_values.pow_db_5G_3); 
      calib_values.gain_offset_5G_4 = floor(calib_values.pow_db_5G_4); 
#ifdef SOCK_LOSS
      calib_values.gain_offset_2G   += 1; 
      calib_values.gain_offset_5G_1 += 3;
      calib_values.gain_offset_5G_2 += 2;
      calib_values.gain_offset_5G_3 += 2;
      calib_values.gain_offset_5G_4 += 2;
#endif
    }
#ifndef SOCK_LOSS
    printf("\033[1m\033[38m""GAIN_OFF_2G   \t= %d\n""\033[0m",(int_8)calib_values.gain_offset_2G); 
//    printf("\033[1m\033[38m""VARIANCE_2G   \t= %d\n""\033[0m",(unsigned int)(sqrt((double)calib_values.variance_2G/1000)*1000)); 
#else
    printf("\033[1m\033[38m""LOSS IN 2G   \t= %d\n""\033[0m",(int_8)calib_values.gain_offset_2G); 
#endif
    //printf("\033[1m\033[38m""VARIANCE_2G   \t= %lf\n""\033[0m",((double)calib_values.variance_2G/1000)); 
#ifndef SOCK_LOSS
    if( mod_type )
    {
      printf("\033[1m\033[38m""GAIN_OFF_5G_1 \t= %d\n""\033[0m",(int_8)calib_values.gain_offset_5G_1);
      printf("\033[1m\033[38m""GAIN_OFF_5G_2 \t= %d\n""\033[0m",(int_8)calib_values.gain_offset_5G_2);
      printf("\033[1m\033[38m""GAIN_OFF_5G_3 \t= %d\n""\033[0m",(int_8)calib_values.gain_offset_5G_3);
      printf("\033[1m\033[38m""GAIN_OFF_5G_4 \t= %d\n\n""\033[0m",(int_8)calib_values.gain_offset_5G_4);

//  fprintf(fptr,"VRMS_2G          =    %d\r\n",(unsigned int)(sqrt((double)calib_values_log->var/1000)*1000));
//    printf("\033[1m\033[38m""VARIANCE_5G_1   \t= %d\n""\033[0m",(unsigned int)(sqrt((double)calib_values.variance_5G_1/1000)*1000)); 
//    printf("\033[1m\033[38m""VARIANCE_5G_2   \t= %d\n""\033[0m",(unsigned int)(sqrt((double)calib_values.variance_5G_2/1000)*1000)); 
//    printf("\033[1m\033[38m""VARIANCE_5G_3   \t= %d\n""\033[0m",(unsigned int)(sqrt((double)calib_values.variance_5G_3/1000)*1000)); 
//    printf("\033[1m\033[38m""VARIANCE_5G_4   \t= %d\n""\033[0m",(unsigned int)(sqrt((double)calib_values.variance_5G_4/1000)*1000)); 
    //  printf("\033[1m\033[38m""VARIANCE_5G_2   \t= %lf\n""\033[0m",((double)calib_values.variance_5G_2)/1000);//*0.7071); 
    //  printf("\033[1m\033[38m""VARIANCE_5G_3   \t= %lf\n""\033[0m",((double)calib_values.variance_5G_3)/1000);//*0.7071); 
    //  printf("\033[1m\033[38m""VARIANCE_5G_4   \t= %lf\n""\033[0m",((double)calib_values.variance_5G_4)/1000);//*0.7071); 
    }
#else
      printf("\033[1m\033[38m""LOSS IN 5G_1 \t= %d\n""\033[0m",(int_8)calib_values.gain_offset_5G_1);
      printf("\033[1m\033[38m""LOSS IN 5G_2 \t= %d\n""\033[0m",(int_8)calib_values.gain_offset_5G_2);
      printf("\033[1m\033[38m""LOSS IN 5G_3 \t= %d\n""\033[0m",(int_8)calib_values.gain_offset_5G_3);
      printf("\033[1m\033[38m""LOSS IN 5G_4 \t= %d\n\n""\033[0m",(int_8)calib_values.gain_offset_5G_4);
#endif

#ifndef SOCK_LOSS
	  printf ("\033[1m\033[35m \n\tTXIQ CALIBRATION\033[0m\n");
	  printf("\033[1m\033[38m""tx_dci_2G     \t= %x\t""\033[0m",calib_values.tx_dci_2G);     
	  if( mod_type )
	  {
		  printf("\033[1m\033[38m""tx_dci_5G     \t= %x\n""\033[0m",calib_values.tx_dci_5G);     
	  }
	  else
	  {
		  printf("\n");
	  }
	  printf("\033[1m\033[38m""tx_dcq_2G     \t= %x\t""\033[0m",calib_values.tx_dcq_2G);     
	  if( mod_type )
	  {
		  printf("\033[1m\033[38m""tx_dcq_5G     \t= %x\n""\033[0m",calib_values.tx_dcq_5G);     
	  }
	  else
	  {
		  printf("\n");
	  }
	  printf("\033[1m\033[38m""tx_betacos_2G \t= %x\t""\033[0m",calib_values.tx_betacos_2G);     
	  if( mod_type )
	  {
		  printf("\033[1m\033[38m""tx_betacos_5G \t= %x\n""\033[0m",calib_values.tx_betacos_5G);     
	  }
	  else
	  {
		  printf("\n");
	  }
	  printf("\033[1m\033[38m""tx_betasin_2G \t= %x\t""\033[0m",calib_values.tx_betasin_2G);     
	  if( mod_type )
	  {
		  printf("\033[1m\033[38m""tx_betasin_5G \t= %x\n""\033[0m",calib_values.tx_betasin_5G);     
	  }
	  else
	  {
		  printf("\n");
	  }
	  //    printf("tx_g_2G \t= %f\t\t",(float)calib_values.tx_g_2G/10000);     
	  //    printf("tx_g_5G \t= %f\n",(float)calib_values.tx_g_5G/10000);     
	  //    printf("tx_ph_2G \t= %f\t\t",(float)calib_values.tx_ph_2G/10000);     
	  //    printf("tx_ph_5G \t= %f\n",(float)calib_values.tx_ph_5G/10000);     
	  //    printf("pow_db_2G \t= %lf\t\t\n",(float)calib_values.pow_db_2G/10000);
	  //    printf("pow_db_5G_1 \t= %lf\t\t",(float)calib_values.pow_db_5G_1/10000);
	  //    printf("pow_db_5G_2 \t= %lf\n",(float)calib_values.pow_db_5G_2/10000);
	  //    printf("pow_db_5G_3 \t= %lf\t\t",(float)calib_values.pow_db_5G_3/10000);
	  //    printf("pow_db_5G_4 \t= %lf\n",(float)calib_values.pow_db_5G_4/10000);
	  printf ("\033[1m\033[35m \n\tRF/ULP TEMP CALIBRATION\033[0m\n");
	  
	  if(x_rf < 5.260695)
	  {
		  x_rf = 5.260695;
	  }
	  calib_values.rf_int = (int_8)x_rf;
	  if( x_rf > 0 )
	  {
		  calib_values.rf_fra = (int_8)((x_rf - (int_8)x_rf)*100);
	  }
	  else
	  {
		  calib_values.rf_fra = (int_8)(-(x_rf - (int_8)x_rf)*100);
	  }

	  if(x_ulp < 0.052437)
	  {
		  x_ulp = 0.052437;
	  }
	  calib_values.ulp_int = (int_8)x_ulp;
	  if( x_ulp > 0 )
	  {
		  calib_values.ulp_fra = (int_8)((x_ulp  - (int_8)x_ulp)*100);
	  }
	  else
	  {
		  calib_values.ulp_fra = (int_8)(-(x_ulp  - (int_8)x_ulp)*100);
	  }
	  printf("\033[1m\033[38m""rf_int  = %d\t""\033[0m",calib_values.rf_int);
	  printf("\033[1m\033[38m""rf_fra  = %.2d\n""\033[0m",calib_values.rf_fra);
	  printf("\033[1m\033[38m""ulp_int = %d\t""\033[0m",calib_values.ulp_int);
	  printf("\033[1m\033[38m""ulp_fra = %.2d\n""\033[0m",calib_values.ulp_fra);

	  calib_values.gdb_est_2G = (int_32)100000*(log10(((double)calib_values.isq_avg/1000000)/((double)calib_values.qsq_avg/1000000)));
	  calib_values.ph_deg_approx_2G = (int_32)10000*asin(((double)calib_values.iq_avg/1000000)/(sqrt(((double)calib_values.qsq_avg/1000000)*((double)calib_values.isq_avg/1000000))))*(180/3.414);

	  rx_g = pow(10,((-1)*((double)calib_values.gdb_est_2G/10000)/20));
	  rx_ph = ((double)calib_values.ph_deg_approx_2G/10000)*(3.414/180);


	  rx_betacos = round(rx_g*cos(rx_ph) * 256);
	  rx_betasin = round(rx_g*sin(rx_ph) * 512);

	  rx_betacos &= 0x3ff;
	  rx_betasin &= 0x3ff;

	  calib_values.gdb_est_5G = (int_32)100000*(log10(((double)calib_values.isq_avg_5G/1000000)/((double)calib_values.qsq_avg_5G/1000000)));
	  calib_values.ph_deg_approx_5G = (int_32)10000*asin(((double)calib_values.iq_avg_5G/1000000)/(sqrt(((double)calib_values.qsq_avg_5G/1000000)*((double)calib_values.isq_avg_5G/1000000))))*(180/3.414);

	  rx_g = pow(10,((-1)*((double)calib_values.gdb_est_5G/10000)/20));
	  rx_ph = ((double)calib_values.ph_deg_approx_5G/10000)*(3.414/180);

	  rx_betacos_5G = round(rx_g*cos(rx_ph) * 256);
	  rx_betasin_5G = round(rx_g*sin(rx_ph) * 512);

	  rx_betacos_5G &= 0x3ff;
	  rx_betasin_5G &= 0x3ff;


	  printf ("\033[1m\033[35m \n\tRXIQ CALIBRATION\033[0m\n");
#if 0	  
	  printf("\033[1m\033[38m""isq_avg \t= %lf\n""\033[0m",(float)calib_values.isq_avg/1000000);
	  printf("\033[1m\033[38m""qsq_avg  \t= %lf\n""\033[0m",(float)calib_values.qsq_avg/1000000);
	  printf("\033[1m\033[38m""iq_avg  \t= %lf\n\n""\033[0m",(float)calib_values.iq_avg/1000000);

	  printf("\033[1m\033[38m""isq_avg_5G \t= %lf\n""\033[0m",(float)calib_values.isq_avg_5G/1000000);
	  printf("\033[1m\033[38m""qsq_avg_5G  \t= %lf\n""\033[0m",(float)calib_values.qsq_avg_5G/1000000);
	  printf("\033[1m\033[38m""iq_avg_5G  \t= %lf\n\n""\033[0m",(float)calib_values.iq_avg_5G/1000000);

#endif
	  printf("\033[1m\033[38m""gdb_est 2G \t= %lf\n""\033[0m",(float)calib_values.gdb_est_2G/10000);
	  printf("\033[1m\033[38m""ph_deg 2G \t= %lf\n\n""\033[0m",(float)calib_values.ph_deg_approx_2G/10000);
	  if(mod_type == 1)
	  {	
		  printf("\033[1m\033[38m""gdb_est 5G\t= %lf\n""\033[0m",(float)calib_values.gdb_est_5G/10000);
		  printf("\033[1m\033[38m""ph_deg 5G \t= %lf\n\n""\033[0m",(float)calib_values.ph_deg_approx_5G/10000);
	  }	  
	   
	  //    printf("\033[1m\033[38m""Variance\t= %d\n\n",calib_values.var);
	  if(rx_band_2G_en == 1)
	  {
		  printf("\033[1m\033[38m""VRMS_2G    \t= %d\n\n",calib_values.var);
	  }
	  if( mod_type )
	  {
		  if(rx_band_5G_1_en == 1)
		  {
			  printf("\033[1m\033[38m""VRMS_5G_1    \t= %d\n\n",calib_values.var_5g);
		  }
		  if(rx_band_5G_2_en == 1)
		  {
			  printf("\033[1m\033[38m""VRMS_5G_2    \t= %d\n\n",calib_values.var_5g_2);
		  }
		  if(rx_band_5G_3_en == 1)
		  {
			  printf("\033[1m\033[38m""VRMS_5G_3    \t= %d\n\n",calib_values.var_5g_3);
		  }
		  if(rx_band_5G_4_en == 1)
		  {
			  printf("\033[1m\033[38m""VRMS_5G_4    \t= %d\n\n",calib_values.var_5g_4);
		  }
	  }

	  if( mod_type )
	  {
		  for(k=0;k<ch_cnt;k++)
		  {

			  if(((calib_values.rx_channel[k] & 0x00FF) >= 1) && ((calib_values.rx_channel[k] & 0x00FF) <= 14))
			  {
				  calib_values.rx_vrms_offset_2G = ((calib_values.rx_channel[k] & 0xFF00) >> 8);
			  }

			  else if(((calib_values.rx_channel[k] & 0x00FF) >= 36) && ((calib_values.rx_channel[k] & 0x00FF) < 60 ))
			  {
				  calib_values.rx_vrms_offset_5G_1 = (calib_values.rx_channel[k] & 0xFF00) >> 8 ;
			  }
			  else if(((calib_values.rx_channel[k] & 0x00FF) >= 60) && ((calib_values.rx_channel[k] & 0x00FF) < 100 ))
			  {
				  calib_values.rx_vrms_offset_5G_2 = (calib_values.rx_channel[k] & 0xFF00) >> 8 ;
			  }
			  else if(((calib_values.rx_channel[k] & 0x00FF) >= 100) && ((calib_values.rx_channel[k] & 0x00FF) < 149  ))
			  {
				  calib_values.rx_vrms_offset_5G_3 = (calib_values.rx_channel[k] & 0xFF00) >> 8 ;
			  }
			  else if(((calib_values.rx_channel[k] & 0x00FF) >= 149) && ((calib_values.rx_channel[k] & 0x00FF) <= 165  ))
			  {
				  calib_values.rx_vrms_offset_5G_4 = (calib_values.rx_channel[k] & 0xFF00) >> 8 ;
			  }
		  }

	  }
	  else 
	  {
		  for(k=0;k<ch_cnt;k++)
		  {
			  if(((calib_values.rx_channel[k] & 0x00FF) >= 1) && ((calib_values.rx_channel[k] & 0x00FF) <= 14))
			  {
				  calib_values.rx_vrms_offset_2G = (calib_values.rx_channel[k] & 0xFF00) >> 8;
			  }
		  }

	  }

	  printf("\033[1m\033[38m""rx_betasin_2G\t= %x\t\n""\033[0m",rx_betasin);     
	  printf("\033[1m\033[38m""rx_betacos_2G\t= %x\n\n""\033[0m",rx_betacos);     
	  if(mod_type == 1)
	  {
		  printf("\033[1m\033[38m""rx_betasin_5G\t= %x\t\n""\033[0m",rx_betasin_5G);     
		  printf("\033[1m\033[38m""rx_betacos_5G\t= %x\n\n""\033[0m",rx_betacos_5G);     
	  }
	  printf("\033[1m\033[38m""time_ms  \t= %d\n\n""\033[0m",calib_values.time_ms);
	  //if((calib_values.adapter_state != CALIB_DONE) && (calib_values.adapter_state != GPIO_TEST))
	  if((calib_values.adapter_state != CALIB_DONE))
	  {
		  printf ("\033[1m\033[31m \n\tCALIBRATION PROCESS INCOMPLETE IN FW\033[0m\n");

		  switch(calib_values.adapter_state)
		  {
			  case XO_CTUNE:
				  printf ("\033[1m\033[31m \n\t\tXO Ctune FAILED\033[0m\n");
				  break;
			  case GAIN_OFF_ESTI:
				  printf ("\033[1m\033[31m \n\t\tGAIN_OFFSET ESTIMATION FAILED\033[0m\n");
				  break;
			  case TX_IQ_IMB:
				  printf ("\033[1m\033[31m \n\t\tTXIQ FAILED\033[0m\n");
				  break;
			  case RX_IQ_IMB:
				  printf ("\033[1m\033[31m \n\t\tRXIQ FAILED\033[0m\n");
				  break;
			  case GPIO_TEST:
				  printf ("\033[1m\033[31m \n\t\tGPIO_TEST FAILED\033[0m\n");
				  break;
			  default:
				  calib_values.adapter_state = 0;
				  printf ("\033[1m\033[31m \n\t\tUART_LINK FAILED\033[0m\n");
				  printf ("\033[1m\033[31m \nKindly Make sure the Master and Slave Driver are Inserted\n");
				  break;

		  }
		  write(pipe_fd[1],"FAIL",5);
		  usleep(500);
		  write(pipe_fd[1],(calib_params_t *)&calib_values,sizeof(calib_params_t));

		  close(pipe_fd[0]);
		  close(pipe_fd[1]);
      system("sh remove_all.sh");
      usleep(3000);
		  pthread_exit(0);
	  }
#endif
  }
#if 1
#ifndef SOCK_LOSS
  if( (calib_values.cfo_val > 280) || (calib_values.ctune_val == 0 ) )
  {
    printf("ctune_val \t= %x\n",calib_values.ctune_val);     
    printf("cfo_value \t= %d\n\n",calib_values.cfo_val);     
    printf ("\033[1m\033[31m \n\tXO Ctune FAILED\033[0m\n");
    calib_values.adapter_state = XO_CTUNE;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((((float)(calib_values.tx_g_5G/10000) < -0.3)  || ((float)(calib_values.tx_g_5G/10000) > 0.3))  || 
      (((float)(calib_values.tx_g_2G/10000) < -0.3)  || ((float)(calib_values.tx_g_2G/10000) > 0.3) ))
  {
    printf ("\033[1m\033[31m \n\tTXIQ FAILED\033[0m\n");
    if( mod_type )
    {
      printf("tx_g_5G = %lf\n",(float)(calib_values.tx_g_5G/10000));
    }
    printf("tx_g_2G = %lf\n",(float)(calib_values.tx_g_2G/10000));
    calib_values.adapter_state = TX_IQ_IMB;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((((float)(calib_values.tx_ph_2G/10000) < -3) || ((float)(calib_values.tx_ph_2G/10000) > 3 )) || 
      (((float)(calib_values.tx_ph_5G/10000) < -3) || ((float)(calib_values.tx_ph_5G/10000) > 3 )))    
  {
    printf ("\033[1m\033[31m \n\tTXIQ FAILED\033[0m\n");
    if( mod_type )
    {
      printf("tx_ph_5G = %lf\n",(float)(calib_values.tx_ph_5G/10000));
    }
    printf("tx_ph_2G = %lf\n",(float)(calib_values.tx_ph_2G/10000));
    calib_values.adapter_state = TX_IQ_IMB;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((calib_values.gain_offset_2G > 3 ) || (calib_values.gain_offset_2G < -5 ))
  {
    printf("gain_offset_2G \t= %d\n",(int_8)calib_values.gain_offset_2G);
    printf ("\033[1m\033[31m \n\tGAIN_OFFSET ESTIMATION FAILED\033[0m\n");
    calib_values.adapter_state = GAIN_OFF_ESTI;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((calib_values.gain_offset_5G_1 > 4 ) || (calib_values.gain_offset_5G_1 < -4 ))
  {
    printf("gain_offset_5G_1 \t= %d\n",(int_8)calib_values.gain_offset_5G_1);
    printf ("\033[1m\033[31m \n\tGAIN_OFFSET ESTIMATION FAILED\033[0m\n");
    calib_values.adapter_state = GAIN_OFF_ESTI;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((calib_values.gain_offset_5G_2 > 4 ) || (calib_values.gain_offset_5G_2 < -4 )) 

  {
    printf("gain_offset_5G_2 \t= %d\n",(int_8)calib_values.gain_offset_5G_2);
    printf ("\033[1m\033[31m \n\tGAIN_OFFSET ESTIMATION FAILED\033[0m\n");
    calib_values.adapter_state = GAIN_OFF_ESTI;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((calib_values.gain_offset_5G_3 > 4 ) || (calib_values.gain_offset_5G_3 < -4 ))
  {
    printf("gain_offset_5G_3 \t= %d\n",(int_8)calib_values.gain_offset_5G_3);
    printf ("\033[1m\033[31m \n\tGAIN_OFFSET ESTIMATION FAILED\033[0m\n");
    calib_values.adapter_state = GAIN_OFF_ESTI;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((calib_values.gain_offset_5G_4 > 4 ) || (calib_values.gain_offset_5G_4 < -4 ))
  {
    printf("gain_offset_5G_4 \t= %d\n",(int_8)calib_values.gain_offset_5G_4);
    printf ("\033[1m\033[31m \n\tGAIN_OFFSET ESTIMATION FAILED\033[0m\n");
    calib_values.adapter_state = GAIN_OFF_ESTI;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((calib_values.rf_int == 0) && (calib_values.rf_fra == 0 ))
  {
    printf ("\033[1m\033[31m \n\tRF TEMPERATURE CALIBRATION FAILED\033[0m\n");
    calib_values.adapter_state = RF_TEMP_CALIB;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((calib_values.ulp_int == 0) && (calib_values.ulp_fra == 0 ))
  {
    printf ("\033[1m\033[31m \n\tULP TEMPERATURE CALIBRATION FAILED\033[0m\n");
    calib_values.adapter_state = ULP_TEMP_CALIB;
    write(pipe_fd[1],"FAIL",5);
  }
  else if( calib_values.adapter_state == GPIO_TEST)
  {
    printf ("\033[1m\033[31m \n\t\tGPIO_TEST FAILED\033[0m\n");
  }
  else if((rx_band_2G_en == 1)&&(( fabs(20*log10((float)210/calib_values.var)) > 5 )  ||(fabs(calib_values.rx_vrms_offset_2G)>5)))
  {
    printf ("\033[1m\033[31m \n\t\tBAD Receive VRMS in 2G\033[0m\n");
    calib_values.adapter_state = VRMS_2G;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((((float)calib_values.gdb_est_2G/10000) < -0.9)  || (((float)calib_values.gdb_est_2G/10000) > 0.9)) 
  {
    printf ("\033[1m\033[31m \n\tRXIQ FAILED in 2G\033[0m\n");
    calib_values.adapter_state = RX_IQ_IMB;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((((float)calib_values.ph_deg_approx_2G/10000) < -2)  || (((float)calib_values.ph_deg_approx_2G/10000) > 2)) 
  {
    printf ("\033[1m\033[31m \n\tRXIQ FAILED in 2G\033[0m\n");
    calib_values.adapter_state = RX_IQ_IMB;
    write(pipe_fd[1],"FAIL",5);
  }
  else if( mod_type )
  {
 
  if((((float)calib_values.gdb_est_5G/10000) < -0.9)  || (((float)calib_values.gdb_est_5G/10000) > 0.9)) 
  {
    printf ("\033[1m\033[31m \n\tRXIQ FAILED in 5G\033[0m\n");
    calib_values.adapter_state = RX_IQ_IMB;
    write(pipe_fd[1],"FAIL",5);
  }
  else if((((float)calib_values.ph_deg_approx_5G/10000) < -2)  || (((float)calib_values.ph_deg_approx_5G/10000) > 2)) 
  {
    printf ("\033[1m\033[31m \n\tRXIQ FAILED in 5G\033[0m\n");
    calib_values.adapter_state = RX_IQ_IMB;
    write(pipe_fd[1],"FAIL",5);
  }	  
    if((rx_band_5G_1_en == 1)&&(( fabs(20*log10((float)210/calib_values.var_5g)) > 5 )  ||(fabs(calib_values.rx_vrms_offset_5G_1)>5)))
    {
      printf ("\033[1m\033[31m \n\t\tBAD Receive VRMS in 5G_1\033[0m\n");
      flag_vrms = 1;
    }

    if((rx_band_5G_2_en == 1)&&(( fabs(20*log10((float)210/calib_values.var_5g_2)) > 5 )  ||(fabs(calib_values.rx_vrms_offset_5G_2)>5)))
    {
      printf ("\033[1m\033[31m \n\t\tBAD Receive VRMS in 5G_2\033[0m\n");
      flag_vrms = 1;
    }

    if((rx_band_5G_3_en == 1)&&(( fabs(20*log10((float)210/calib_values.var_5g_3)) > 5 )  ||(fabs(calib_values.rx_vrms_offset_5G_3)>5)))
    {
      printf ("\033[1m\033[31m \n\t\tBAD Receive VRMS in 5G_3\033[0m\n");
      flag_vrms = 1;
    }

    if((rx_band_5G_4_en == 1)&&(( fabs(20*log10((float)210/calib_values.var_5g_4)) > 5 )  ||(fabs(calib_values.rx_vrms_offset_5G_4)>5)))
    {
      printf ("\033[1m\033[31m \n\t\tBAD Receive VRMS in 5G_4\033[0m\n");
      flag_vrms = 1;
    }
    if(flag_vrms == 1)
    {
      flag_vrms = 0;
      calib_values.adapter_state = VRMS_5G;
      write(pipe_fd[1],"FAIL",5);
    }
  }
#endif
  usleep(500);
  write(pipe_fd[1],(calib_params_t *)&calib_values,sizeof(calib_params_t));
#endif
#endif
  write(pipe_fd[1],(calib_params_t *)&calib_values,sizeof(calib_params_t));
  close(pipe_fd[0]);
  close(pipe_fd[1]);
  system("sh remove_all.sh");
  usleep(3000);
  pthread_exit(0);
}


void *slave_func( void *arg)
{
  calib_params_t *calib_values_slave;
  uint_8 read_buffer[200];
  float ulp_const;
  float rf_const;

  memset(read_buffer,0,sizeof(read_buffer));
  read(pipe_fd[0],read_buffer,200);

  if( strcmp(read_buffer,"FAIL") == 0 )
  {
    printf ("\033[1m\033[31m \nCalibration Process Failed\033[0m\n");

    memset(read_buffer,0,sizeof(read_buffer));
    read(pipe_fd[0],read_buffer,200);
    calib_values_slave = (calib_params_t *)read_buffer;
#if DEBUG_PRINT
    printf ("\033[1m\033[35m \n\tXO CALIBRATION\033[0m\n\n");
    printf("\033[1m\033[38m""cfo_value     \t= %d\n""\033[0m",calib_values_slave->cfo_val);
    printf("\033[1m\033[38m""ctune_val     \t= %d\n""\033[0m",calib_values_slave->ctune_val);

    printf ("\033[1m\033[35m \n\tGAIN OFFSET CALIBRATION\033[0m\n");    
    printf("\033[1m\033[38m""GAIN_OFF_2G   \t= %d\n""\033[0m",(int_8)calib_values_slave->gain_offset_2G); 
    printf("\033[1m\033[38m""GAIN_OFF_5G_1 \t= %d\n""\033[0m",(int_8)calib_values_slave->gain_offset_5G_1);
    printf("\033[1m\033[38m""GAIN_OFF_5G_2 \t= %d\n""\033[0m",(int_8)calib_values_slave->gain_offset_5G_2);
    printf("\033[1m\033[38m""GAIN_OFF_5G_3 \t= %d\n""\033[0m",(int_8)calib_values_slave->gain_offset_5G_3);
    printf("\033[1m\033[38m""GAIN_OFF_5G_4 \t= %d\n""\033[0m",(int_8)calib_values_slave->gain_offset_5G_4);

    printf ("\033[1m\033[35m \n\tTXIQ CALIBRATION\033[0m\n");
    printf("\033[1m\033[38m""tx_dci_2G     \t= %x\t""\033[0m",calib_values_slave->tx_dci_2G);     
    printf("\033[1m\033[38m""tx_dci_5G     \t= %x\n""\033[0m",calib_values_slave->tx_dci_5G);     
    printf("\033[1m\033[38m""tx_dcq_2G     \t= %x\t""\033[0m",calib_values_slave->tx_dcq_2G);     
    printf("\033[1m\033[38m""tx_dcq_5G     \t= %x\n""\033[0m",calib_values_slave->tx_dcq_5G);     
    printf("\033[1m\033[38m""tx_betacos_2G \t= %x\t""\033[0m",calib_values_slave->tx_betacos_2G);     
    printf("\033[1m\033[38m""tx_betacos_5G \t= %x\n""\033[0m",calib_values_slave->tx_betacos_5G);     
    printf("\033[1m\033[38m""tx_betasin_2G \t= %x\t""\033[0m",calib_values_slave->tx_betasin_2G);     
    printf("\033[1m\033[38m""tx_betasin_5G \t= %x\n""\033[0m",calib_values_slave->tx_betasin_5G);     
#endif
    prepare_log_file(calib_values_slave,1,module_params.type);

    close(pipe_fd[0]);
    close(pipe_fd[1]);
    pthread_exit(0);
  }
  else if(strcmp(read_buffer,"CLOSE") == 0 )
  {
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    pthread_exit(0);
  }
  calib_values_slave = (calib_params_t *)read_buffer;
#if 0
  printf ("\033[1m\033[35m \n\tXO CALIBRATION\033[0m\n\n");
  printf("\033[1m\033[38m""cfo_value     \t= %d\n""\033[0m",calib_values_slave->cfo_val);
  printf("\033[1m\033[38m""ctune_val     \t= %d\n""\033[0m",calib_values_slave->ctune_val);

  printf ("\033[1m\033[35m \n\tGAIN OFFSET CALIBRATION\033[0m\n");    
  printf("\033[1m\033[38m""GAIN_OFF_2G   \t= %d\n""\033[0m",(int_8)calib_values_slave->gain_offset_2G); 
  printf("\033[1m\033[38m""GAIN_OFF_5G_1 \t= %d\n""\033[0m",(int_8)calib_values_slave->gain_offset_5G_1);
  printf("\033[1m\033[38m""GAIN_OFF_5G_2 \t= %d\n""\033[0m",(int_8)calib_values_slave->gain_offset_5G_2);
  printf("\033[1m\033[38m""GAIN_OFF_5G_3 \t= %d\n""\033[0m",(int_8)calib_values_slave->gain_offset_5G_3);
  printf("\033[1m\033[38m""GAIN_OFF_5G_4 \t= %d\n""\033[0m",(int_8)calib_values_slave->gain_offset_5G_4);

  printf ("\033[1m\033[35m \n\tTXIQ CALIBRATION\033[0m\n");
  printf("\033[1m\033[38m""tx_dci_2G     \t= %x\t""\033[0m",calib_values_slave->tx_dci_2G);     
  printf("\033[1m\033[38m""tx_dci_5G     \t= %x\n""\033[0m",calib_values_slave->tx_dci_5G);     
  printf("\033[1m\033[38m""tx_dcq_2G     \t= %x\t""\033[0m",calib_values_slave->tx_dcq_2G);     
  printf("\033[1m\033[38m""tx_dcq_5G     \t= %x\n""\033[0m",calib_values_slave->tx_dcq_5G);     
  printf("\033[1m\033[38m""tx_betacos_2G \t= %x\t""\033[0m",calib_values_slave->tx_betacos_2G);     
  printf("\033[1m\033[38m""tx_betacos_5G \t= %x\n""\033[0m",calib_values_slave->tx_betacos_5G);     
  printf("\033[1m\033[38m""tx_betasin_2G \t= %x\t""\033[0m",calib_values_slave->tx_betasin_2G);     
  printf("\033[1m\033[38m""tx_betasin_5G \t= %x\n""\033[0m",calib_values_slave->tx_betasin_5G);     
  close(pipe_fd[0]);
  close(pipe_fd[1]);
  pthread_exit(0);
#endif

#if 0
  printf ("\033[1m\033[35m \n\tOTP WRITING\033[0m\n");
  if( process_otp_write() == -1 )
  {
    printf ("\033[1m\033[31m \n\t   OTP Write Failed\033[0m\n");
    prepare_log_file(calib_values_slave,3 ,module_params.type);
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    pthread_exit(0);
  }
#endif
  if( prepare_buffer(calib_values_slave, module_params.type) == -1 )
  {
    printf ("\033[1m\033[31m \n\t   Buffer Preparation Failed\033[0m\n");
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    pthread_exit(0);
  }  
  if( flash_buffer(1, module_params.type, module_params.boot_load, calib_values_slave) == -1 )
  {
    printf ("\033[1m\033[31m \n\t   Calib Flashing failed\033[0m\n");
    prepare_log_file(calib_values_slave,2 ,module_params.type);
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    pthread_exit(0);
  }

  close(pipe_fd[0]);
  close(pipe_fd[1]);

  pthread_exit(0);
}

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
int rsi_slave_ready()
{
  FILE *fptr = NULL;
  uint_8 array[100];
  memset(array,0,100);
  struct timeval enter_time,exit_time;
  gettimeofday(&enter_time,0x0);
  while(1)
  {

    gettimeofday(&exit_time,0x0);
    if( (exit_time.tv_sec - enter_time.tv_sec) >= 4 )
    {
      printf("\033[1m\033[31m \n\n\tUnable to insert Slave Driver\n\033[0m");
      printf("\033[1m\033[31m     Verify enumeration of Slave Driver\n\033[0m\n");
      return RSI_STATUS_FAILURE;
    }
    fptr = fopen("/proc/onebox-mobile_slave/stats","r");
    if( ( fptr == NULL))
    {
      //  perror("fopen");
      continue;
    }
    while(fgets(array,100,fptr))
    {
      if(strstr(array,"FSM_MAC_INIT_DONE(13)"))
      {
        fclose(fptr);
        return RSI_STATUS_SUCCESS;
      }
    }
    fclose(fptr);
  }
}

int rsi_read_config(void)
{
  char token[80];
  int i=0,j=0;
  FILE *rsi_config,*config_file;

  rsi_config = fopen("../release/flash/RSI_Config.txt","r");

  if (rsi_config == NULL)
  {
    printf("Unable to open CONFIG file\n");
    return RSI_STATUS_FAILURE;
  }  
#if 0
  config_file = fopen("../release/flash/config_internal.c","r");
  if (config_file == NULL)
  {
    printf("Unable to open CONFIG_int.c file\n");
    return RSI_STATUS_FAILURE;
  }  
#endif
#if 0
  if (rsi_get_token(rsi_config,"RSI_EMBEDDED_MODE",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&module_params.boot_load);
#endif
#ifdef NLINK
  module_params.boot_load = 0;
#elif WISE
  module_params.boot_load = 1;
#endif
  //  printf("EMBEDDE MODE  %d\n",module_params.boot_load);
#if 0
  if (rsi_get_token(rsi_config,"RSI_MODULE_TYPE",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&module_params.type);
#endif
#ifdef SINGLE_BAND
  module_params.type = 2;
#elif DUAL_BAND
  module_params.type = 3;
#endif

#ifdef ANT_0
  module_params.ant_type = 0;
#elif ANT_1
  module_params.ant_type = 1;
#endif


  //  printf("MODULE_TYPE  %d\n",module_params.type);

  if (rsi_get_token(rsi_config,"RSI_FLASH_SIZE",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&module_params.eeprom_size);
  //  printf("FLASH_SIZE  %d\n",module_params.eeprom_size);

  if (rsi_get_token(rsi_config,"RSI_EEPROM_TYPE",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&module_params.eeprom_type);
  //  printf("EEPROM_TYPE  %d\n",module_params.eeprom_type);

  if (rsi_get_token(rsi_config,"RSI_MODULE_APPEND",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&module_params.module_append);
  //  printf("MODULE_APPEND  %hd\n",module_params.module_append);

  if (rsi_get_token(rsi_config,"RSI_EEPROM_VERSION",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&module_params.eeprom_version);
  //  printf("EEPROM_VERSION  %hd\n",module_params.eeprom_version);

  if (rsi_get_token(rsi_config,"RSI_DIGI_CHIP_VERSION",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&module_params.digi_chip_ver);
  // printf("DIGI_CHIP_VERSION  %hd\n",module_params.digi_chip_ver);

  if (rsi_get_token(rsi_config,"RSI_RF_CHIP_VERSION",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&module_params.rf_chip_ver);
  //  printf("DIGI_RF_VERSION  %d\n",module_params.rf_chip_ver);
#ifdef WISE_MCU
  if (rsi_get_token(rsi_config,"RSI_MODULE_VERSION_WMCU",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
#elif WISE_MCU_8x8
  if (rsi_get_token(rsi_config,"RSI_MODULE_VERSION_WMCU",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
#else
  if (rsi_get_token(rsi_config,"RSI_MODULE_VERSION",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;
#endif

  sscanf(token,"%hd",&module_params.module_ver);
  //  printf("MODULE_VERSION  %d\n",module_params.module_ver);

  if (rsi_get_token(rsi_config,"RSI_FLASH_CHECK",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&module_params.flash_check);
  //  printf("FLASH_CHECK  %d\n",module_params.flash_check);

  if (rsi_get_token(rsi_config,"RSI_MFG_SW_VERSION",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&module_params.mfg_sw_ver);

  if (rsi_get_token(rsi_config,"RSI_GPIO_BYPASS",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&module_params.gpio_bypass);

  if (rsi_get_token(rsi_config,"CABLE_LOSS_TYPE",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&config_values.cable_loss_type);

  if(config_values.cable_loss_type == 0) //For socket board
  {
    if (rsi_get_token(rsi_config,"CABLE_LOSS_2G",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  else                                 //For EVB
  {
    if (rsi_get_token(rsi_config,"CABLE_LOSS_EVB_2G",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }

  sscanf(token,"%hd",&config_values.cable_loss_2g);

  if(config_values.cable_loss_type == 0)
  {
    if (rsi_get_token(rsi_config,"CABLE_LOSS_5G_1",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  else
  { 
    if (rsi_get_token(rsi_config,"CABLE_LOSS_EVB_5G_1",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  sscanf(token,"%hd",&config_values.cable_loss_5g_1);

  if(config_values.cable_loss_type == 0)
  {
    if (rsi_get_token(rsi_config,"CABLE_LOSS_5G_2",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  else
  { 
    if (rsi_get_token(rsi_config,"CABLE_LOSS_EVB_5G_2",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }

  sscanf(token,"%hd",&config_values.cable_loss_5g_2);

  if(config_values.cable_loss_type == 0)
  {
    if (rsi_get_token(rsi_config,"CABLE_LOSS_5G_3",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  else
  { 
    if (rsi_get_token(rsi_config,"CABLE_LOSS_EVB_5G_3",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  sscanf(token,"%hd",&config_values.cable_loss_5g_3);

  if(config_values.cable_loss_type == 0)
  {
    if (rsi_get_token(rsi_config,"CABLE_LOSS_5G_4",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  else
  { 
    if (rsi_get_token(rsi_config,"CABLE_LOSS_EVB_5G_4",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }

  sscanf(token,"%hd",&config_values.cable_loss_5g_4);
#if 0
  if(config_values.cable_loss_type == 0)
  {
    if (rsi_get_token(config_file,"RX_GAIN_OFFSET_2G",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  else
  {
    if (rsi_get_token(config_file,"RX_GAIN_OFFSET_EVB_2G",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }

  sscanf(token,"%hd",&config_values.rx_gain_offset_2g);

  if(config_values.cable_loss_type == 0)
  {
    if (rsi_get_token(config_file,"RX_GAIN_OFFSET_5G_1",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  else
  {
    if (rsi_get_token(config_file,"RX_GAIN_OFFSET_EVB_5G_1",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }

  sscanf(token,"%hd",&config_values.rx_gain_offset_5g_1);

  if(config_values.cable_loss_type == 0)
  {
    if (rsi_get_token(config_file,"RX_GAIN_OFFSET_5G_2",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  else
  {
    if (rsi_get_token(config_file,"RX_GAIN_OFFSET_EVB_5G_2",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }

  sscanf(token,"%hd",&config_values.rx_gain_offset_5g_2);

  if(config_values.cable_loss_type == 0)
  {
    if (rsi_get_token(config_file,"RX_GAIN_OFFSET_5G_3",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  else
  {
    if (rsi_get_token(config_file,"RX_GAIN_OFFSET_EVB_5G_3",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }

  sscanf(token,"%hd",&config_values.rx_gain_offset_5g_3);

  if(config_values.cable_loss_type == 0)
  {
    if (rsi_get_token(config_file,"RX_GAIN_OFFSET_5G_4",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }
  else
  {
    if (rsi_get_token(config_file,"RX_GAIN_OFFSET_EVB_5G_4",token) != RSI_STATUS_SUCCESS)
      return RSI_STATUS_FAILURE;
  }

  sscanf(token,"%hd",&config_values.rx_gain_offset_5g_4);


  if (rsi_get_token(config_file,"EXPECTED_POWER_2G",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&config_values.expected_power_2g);

  if (rsi_get_token(config_file,"EXPECTED_POWER_5G_1",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&config_values.expected_power_5g_1);

  if (rsi_get_token(config_file,"EXPECTED_POWER_5G_2",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&config_values.expected_power_5g_2);

  if (rsi_get_token(config_file,"EXPECTED_POWER_5G_3",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&config_values.expected_power_5g_3);

  if (rsi_get_token(config_file,"EXPECTED_POWER_5G_4",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%hd",&config_values.expected_power_5g_4);


  if (rsi_get_token(config_file,"CHANNEL",token) != RSI_STATUS_SUCCESS)
    return RSI_STATUS_FAILURE;

  sscanf(token,"%c",&chan);
  if(chan == '{')
  {
	  while(chan != '}')
	  {
		  i++;
		  sscanf(token+i,"%d",&chan);
      config_values.rx_channel[j] = chan;
      j++;
      ch_cnt=j;
      if(ch_cnt > 5)
      {
        printf("\033[1m\033[38m""\n\tNumber of channels input must be less than 5\n""\033[0m"); 
        return RSI_STATUS_FAILURE;
      }
		  if(chan == '}')
			  break;
		  while((chan != ',') && (chan != '}'))
		  {
			  i++;
			  sscanf(token+i,"%c",&chan);
		  }
	  }
  }
  fclose(config_file);
#endif
  fclose(rsi_config);
  return RSI_STATUS_SUCCESS;
}

int main(int argc, char *argv[])
{
  pthread_t master_thread, slave_thread;
  int ret;
  gettimeofday(&starttime,0x0);

  //  if( argc != 4 )
  //  {
  //    printf ("\033[1m\033[31m \n\tUSAGE: ./master_app <mod_type (0-02) (1-03)> <card_no> <boot_load (1-Yes) (0-NO)>\033[0m\n");
  //    return -1;
  //  }

  if( pipe(pipe_fd) == -1 )
  {
    perror("pipe");
    return;
  }
#if 0
  system("sh remove_all.sh");
  usleep(3000);
  system("sh insert.sh");
  if( rsi_slave_ready () == RSI_STATUS_FAILURE)
  {
    //printf("Slave Driver is not Ready\n");
    system("sh remove_all.sh");
    return RSI_STATUS_FAILURE;
  }
#endif
  if( rsi_read_config() == RSI_STATUS_FAILURE )
  {
    printf("Unable to read Config File\n");
    return RSI_STATUS_FAILURE;
  }

  pthread_create(&master_thread,NULL,&master_func,&module_params.type);
#ifndef SOCK_LOSS
  pthread_create(&slave_thread, NULL, &slave_func,NULL);
  pthread_join(master_thread,NULL);
  pthread_join(slave_thread,NULL);
#else
  pthread_join(master_thread,NULL);
#endif
  gettimeofday(&endtime,0x0);

  if( starttime.tv_sec == endtime.tv_sec )
  {
    printf("time taken in ms is %f\n", (double)(endtime.tv_usec - starttime.tv_usec )/1000 );
  }
  else
  {
    printf("time taken in ms is %f\n",(double)(((endtime.tv_sec - starttime.tv_sec )*1000000 - starttime.tv_usec) + endtime.tv_usec )/1000);

  }

  //  printf("it took %lu sec and %lu ms\n", starttime.tv_sec ,starttime.tv_usec);
  //  printf("it took %lu sec and %lu ms\n", endtime.tv_sec, endtime.tv_usec);

  return 0;
}

