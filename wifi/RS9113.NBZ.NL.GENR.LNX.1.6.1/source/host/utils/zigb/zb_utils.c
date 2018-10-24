/*
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	1. Redistributions of source code must retain the above copyright
 * 	   notice, this list of conditions and the following disclaimer.
 *
 * 	2. Redistributions in binary form must reproduce the above copyright
 * 	   notice, this list of conditions and the following disclaimer in the
 * 	   documentation and/or other materials provided with the distribution.
 *
 * 	3. Neither the name of the copyright holder nor the names of its
 * 	   contributors may be used to endorse or promote products derived from
 * 	   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/types.h>
#include <linux/if.h>
#include <stdlib.h>
#include <linux/wireless.h>
#include <unistd.h>
#include <fcntl.h>
#include <inttypes.h>
#include "zb_util.h"



/******************************************************************************
 * main()
 *****************************************************************************/

int main (int argc, char *argv[])
{
  int sfd;
  struct iwreq wrq;
  char ifName[32],chan_number;
  int ret = 0;
  int cmdNo = -1;
  int channel = 1;
  int cw_mode = 3;
  int cw_type = 0;
  int len;
  int rsi_cmd_flags;
  unsigned int nl_type = 0;
  unsigned char ii;
  unsigned char rsi_flags;
  unsigned char verbose = 0;
  struct bb_rf_param_t  bb_rf_params;
  unsigned int bb_addr = 0, bb_val = 0;
  unsigned int rf_addr = 0,rf_val = 0;
  int i,j,k = 0;
  unsigned cmd_flags =0;
  unsigned int cw_buffer_1[6] = {0x311,0,0x321,10,0x322,3};
  unsigned int cw_buffer_2[12] = {0x322,0x07,0x302,0x8ffb,0x303,0x4101,0x318,0x80,0x312,0xD2,0x301,0x40};
  unsigned int cw_buffer_3[12] = {0x322,0x07,0x302,0x8ffb,0x303,0x4101,0x318,0x80,0x312,0xD2,0x301,0x60};
  unsigned int cw_buffer_4[2] = {0x322,0x0a};


  memset(&bb_rf_params, 0, sizeof(bb_rf_params));

  if (argc < 2)
  {
    usage();
  }
  else if (argc <= 50)
  {
    cmdNo = getcmdnumber( argv[1] );
  }

  /* Init netlink socket */
  if (rsi_netlink_init() < 0)
  {
    ONEBOX_PRINT ("Netling Socket creation error\n");
    return -1;
  }

  switch (cmdNo)
  {
    case RSI_ANT_SEL:
      if(argc == 3)
      {
        rsi_flags = atoi(argv[2]); // store ant_sel value
        if (rsi_flags > 7 || rsi_flags < 2) 
        {
          ONEBOX_PRINT("Enter a valid value between 2 and 7\n"
              "\t \t	Value = 0,1 => Invalid \n"
              "\t \t	Value = 2; ant_sel => tx_on, ant_sel_b => rx_on \n"
              "\t \t	Value = 3; ant_sel => rx_on, ant_sel_b => tx_on \n"
              "\t \t	Value = 4; ant_sel => 0, ant_sel_b => 0 \n"
              "\t \t	Value = 5; ant_sel => 0, ant_sel_b => 1 \n"
              "\t \t	Value = 6; ant_sel => 1, ant_sel_b => 0 \n"
              "\t \t	Value = 7; ant_sel => 1, ant_sel_b => 1 \n ");
          break;
        } 
        ONEBOX_PRINT("ANT_SEL REG value is 0x%x for the selected value %d\n", (rsi_flags << 5), rsi_flags);

        bb_rf_params.value = ANT_SEL;
        bb_rf_params.no_of_fields = rsi_flags;

        if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER, sizeof(bb_rf_params)) < 0 )
        {
          printf("Unable to perform Ant_sel\n");
        }
      }
      else
      {
        printf("Usage : %s ant_sel <value> \n",argv[0] );
      }
      break;
    case RSI_RF_READ:
      if( 3 == argc )
      {
        rf_addr = strtol(argv[2], NULL, 16);
        //printf("rf_addr: 0x%x\n",rf_addr);

        bb_rf_params.value = 2;
        bb_rf_params.no_of_fields = 5;
        bb_rf_params.no_of_values = 1;
        bb_rf_params.soft_reset = 0;
        bb_rf_params.Data[0] = 0x20;
        bb_rf_params.Data[1] = 0;
        bb_rf_params.Data[2] = rf_addr | BIT(15);
        bb_rf_params.Data[3] = 0;
        bb_rf_params.Data[4] = 0;
      }
      else
      {
        printf("Invalid No of Arguments\n");
        ONEBOX_PRINT
          ("Usage: zb_util rf_read  addr \n");
        break;
      }	
      if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER, sizeof(bb_rf_params)) < 0 )
      {
        printf("Unable to perform RF_READ\n");
        break;
      }
      else
      { 
        memset(&bb_rf_params.Data[0], 0, sizeof(bb_rf_params));
        if( recv_data((uint_8 *)&bb_rf_params.Data[0]) < 0 )
        {
          return -1;
        }
        printf("data[0] = %x\n", bb_rf_params.Data[0]);
        printf("data[1] = %x\n", bb_rf_params.Data[1]);
      }
      break;
    case RSI_RF_WRITE:

      printf("<<<<<<RF_WRITE request IOCTL>>>>>>\n");
      if( 4 == argc )
      {
        rf_addr = strtol(argv[2], NULL, 16);
        rf_val = strtol(argv[3], NULL, 16);
        //printf("rf_addr: 0x%x\n",rf_addr);
        //printf("rf_val: 0x%x\n",rf_val);

        bb_rf_params.value = 3;
        bb_rf_params.no_of_fields = 3;
        bb_rf_params.no_of_values = 1;
        bb_rf_params.soft_reset = 0;
        bb_rf_params.Data[0] = rf_val;
        bb_rf_params.Data[1] = rf_addr;
        bb_rf_params.Data[2] = 0;
        bb_rf_params.Data[3] = 0;
      }
      else
      {
        printf("Invalid No of Arguments\n");
        ONEBOX_PRINT
          ("Usage: onebox_util base_interface rf_write  addr   data \n");
        break;
      }
      if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER, sizeof(bb_rf_params)) < 0 )
      {
        printf("Unable to perform RF_WRITE\n");
        break;
      }
      else
      {

        printf("Writing RF Successful:\n");

      }
      break;
    case RSI_SET_BB_WRITE:
      if(argc == 4)
      {
        bb_addr = strtol(argv[2], NULL, 16);
        bb_val = strtol(argv[3], NULL, 16);
        ONEBOX_PRINT("BB addr: 0x%x value 0x%x\n",bb_addr, bb_val);

        bb_rf_params.value = 1;
        bb_rf_params.no_of_values = 2;
        bb_rf_params.soft_reset = 0;
        bb_rf_params.Data[0] = bb_addr;
        bb_rf_params.Data[1] = bb_val;
      }
      else if (argc > 4)
      {
        if( argc % 2 )
        {
          printf("Usage: %s bb_write addr_1 data_1 addr_2 data_2 ... \n");
          return ONEBOX_STATUS_FAILURE;
        }
        bb_rf_params.no_of_values = (argc - 3);
        for (ii = 2; ii< bb_rf_params.no_of_values - 2+3; ii+=2)
        {
          bb_addr = strtol(argv[ii+2], NULL, 16);
          bb_val = strtol(argv[ii+3], NULL, 16);
          bb_rf_params.Data[ii] = bb_addr;
          bb_rf_params.Data[ii+1] = bb_val;
          ONEBOX_PRINT("BB addr: 0x%x value 0x%x\n",bb_addr, bb_val);
        }
      }
      else
      {
        printf("Usage: %s bb_write addr_1 data_1 addr_2 data_2 ... \n");
        return ONEBOX_STATUS_FAILURE;
      }
      if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER,sizeof(bb_rf_params)) < 0 )
      {
        printf("Unable to perform BB_WRITE\n");
      }
      else
        printf("SUCCESS Writing to BB: \n");
      break;
    case RSI_SET_BB_READ:
      if(argc == 3)
      {
        bb_addr = strtol(argv[2], NULL, 16);
        //			ONEBOX_PRINT("BB addr: 0x%x \n",bb_addr);

        bb_rf_params.value = 0; //BB_READ_TYPE
        bb_rf_params.no_of_values = 1;
        bb_rf_params.soft_reset = 3;
        bb_rf_params.Data[0] = bb_addr;
      }
      else
      {
        printf("Usage: %s bb_read addr \n");
        return ONEBOX_STATUS_FAILURE;
      }
      if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER, sizeof(bb_rf_params)) < 0 )
      {
        printf("Unable to perform BB_READ\n");
        break;
      }
      else
      {
        memset(&bb_rf_params.Data[0], 0, sizeof(bb_rf_params));
        if(recv_data((uint_8 *)&bb_rf_params.Data[0]) < 0 )
        {
          return -1;
        }
        printf("bb_data = %x\n",bb_rf_params.Data[0]);
      }
      break;
    case RSI_GET_ZIGB_STATS:
      {
        if( argc == 3 )
        {
          if( zb_stats(argv[2]) == ONEBOX_STATUS_FAILURE)
          {
            return ONEBOX_STATUS_FAILURE;
          }
        }
        else
        {
          printf("Usage: ./zb_util zb_stats <file_name>\n");  
        }
      }
      break;
	case RSI_SET_ZIGB_CHANNEL:
	  {
		  if(argc == 3)
		  {

			  chan_number = atoi(argv[2]);

			  if ((chan_number < 11) || (chan_number > 26))
			  {
				  printf("Inavlid channel number\n");
				  exit(0);
			  }
			  bb_rf_params.value = TX_STATUS;
			  if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER,sizeof(bb_rf_params)) < 0 )
			  {
				  printf("Unable to perform TX_STAUS\n");
				  return -1;
			  }
			  if(recv_data((uint_8 *)&bb_rf_params.Data[0]) == -1)
			  {
				  printf("======== ALREADY IN TRANSMIT SO STOP TRANSMIT FIRST ============\n");
				  return -1;
			  }

			  bb_rf_params.value = SET_ZIGB_CHAN; //BB_READ_TYPE
			  bb_rf_params.no_of_values = 1;
			  bb_rf_params.soft_reset = 0;
			  bb_rf_params.Data[0] = chan_number;
		  }
		  else
		  {
			  printf("Usage: %s ./zb_util set_channel <channel_index> \n", argv[0]);
			  return ONEBOX_STATUS_FAILURE;
		  }
		  if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER, sizeof(bb_rf_params)) < 0 )
		  {
			  printf("Unable to perform set_channel\n");
			  break;
		  }
		  else
		  {
			  memset(&bb_rf_params.Data[0], 0, sizeof(bb_rf_params));
			  if(recv_data((uint_8 *)&bb_rf_params.Data[0]) < 0 )
			  {
				  return -1;
			  }
			  printf("Channel set successfully\n");
		  }
	  }
	  break;
	case RSI_ZIGB_STATS:
	  {
		  if(argc == 3)
		  {
			  if( zb_stats(argv[2]) == ONEBOX_STATUS_FAILURE)
			  {
				  return ONEBOX_STATUS_FAILURE;
			  }

		  }
		  else
		  {
			  printf("Usage: ./zb_util zigb_stats <filename>\n");  
		  }
	  }
	  break;
	case RSI_SET_CW_MODE:
	  {
		  if(argc == 5)
		  {
			  memset(&bb_rf_params, 0, sizeof(bb_rf_params));
			  channel = atoi (argv[2]);
			  cw_mode = atoi (argv[3]);
			  cw_type = 1 ; //atoi (argv[4]);
			  if( channel < 11 || channel > 26 )
			  {
				  printf("Enter a valid channel between 11-26\n");
				  break;
			  }
			  /*cw_mode=0 is single tone mode and cw_mode=1 is DC mode*/
			  if (!cw_mode)
			  {
				  // bb_write(3,0,cw_buffer_1);
				  switch (cw_type)
				  {
					  case 0:
						  //ONEBOX_PRINT("SET_CW_MODE: Setting the Single tone of 500KHz\n");
						  break;
					  case 1:
					  case 2:
					  case 3:
					  case 4:
					  case 5:
						  ONEBOX_PRINT("SET_CW_MODE: DC Tone\n");
						  bb_rf_params.value = ZIGB_CW_MODE;
						  bb_rf_params.no_of_values = cw_mode;
						  bb_rf_params.no_of_fields = cw_type;
						  bb_rf_params.soft_reset = channel;
						  // ONEBOX_PRINT("SET_CW_MODE: Setting the Single tone of 1MHz \n flag :%d\n ",);
						  // ONEBOX_PRINT("SET_CW_MODE: channel %d , cw-mode : %d , cw_type : %d\n ",channel, cw_mode, cw_type);
						  if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER, sizeof(bb_rf_params)) < 0 )
						  {
							  printf("Unable to send continuous wave\n");
							  break;
						  }  
						  if( atoi(argv[4]) == 2 )
						  {               
							  printf("Send buffer\n");
							  bb_write(6, 0, cw_buffer_2);
						  }
						  else
						  {
							  bb_write(6, 0, cw_buffer_3);
						  }
						  break;
					  default:
						  ONEBOX_PRINT(" SET_CW_MODE: Invalid type in CW mode \n");
						  return -1; 
				  }  
			  }
			  else
			  {
				  ONEBOX_PRINT("SET_CW_MODE: Disable CW mode \n");
				  bb_rf_params.value = ZIGB_CW_MODE;
				  bb_rf_params.no_of_values = cw_mode;
				  bb_rf_params.no_of_fields = cw_type;
				  if (channel)
				  {
					  bb_rf_params.soft_reset = channel;
				  }
				  else
				  {
					  bb_rf_params.soft_reset = 1;
				  }	
				  // ONEBOX_PRINT("SET_CW_MODE: Setting the Single tone of 1MHz \n flag :%d\n ",wrq.u.data.flags);
				  // ONEBOX_PRINT("SET_CW_MODE: channel %d , cw-mode : %d , cw_type : %d\n ",channel, cw_mode, cw_type);
				  if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER, sizeof(bb_rf_params)) < 0 )
				  {
					  printf("Unable to send continuous wave\n");
					  break;
				  }
				  printf("cw[0] = %x\n",cw_buffer_4[0]);
				  bb_write(1, 0, cw_buffer_4);
			  }
			  break;
		  }
		  else
		  {
			  ONEBOX_PRINT("%s cw_mode <channel(11-26)> <(start-0)/(stop-2)> <(Ant_Sel -2/3)>", argv[0]);
		  }
	  }
	  break;

    default:
      break;
  }
  if (rsi_netlink_deinit() < 0)
  {
    ONEBOX_PRINT ("Netling Socket creation error\n");
    return -1;
  }
  return ret;
}

int_32 zb_stats( uint_8 *file_name )
{
 
  int first_time = 1;
  uint_32 nl_type = 0;
  uint_32 nl_len = 0;
  int count = 0, stop_bit = 0;
  int i; 
  FILE *pFile;
  struct iwreq iwr;
   per_stats_t *sta_info;
  struct bb_rf_param_t bb_rf_params;

  memset(&bb_rf_params, 0, sizeof(bb_rf_params));

  pFile = fopen(file_name, "w");
  if(pFile == NULL)
  {
    printf("Unable to create a file\n");
    return ONEBOX_STATUS_FAILURE;
  }

  while(1)
  {
    sta_info = (per_stats_t *)malloc(300);

#if 1
    //unsigned int sta_info[100];
    if(sleep(1)!=0)
    {
      printf("Unable to sleep\n");            
      free(sta_info);        
      break;
    }

    bb_rf_params.value = ZIGB_PER_STATS;

    if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER, sizeof(bb_rf_params)) < 0 )
    {
      printf("Unable to perform receive stats\n");
    }
    else
    {
      if(recv_data((uint_8*)sta_info) < 0 )
      {
        free(sta_info);
        return -1;
      }
      if(first_time)
      {
        first_time = 0;
        free(sta_info);
        fflush(pFile);
        continue;
      }
      if((count % 20) == 0)
      {
#if 0
        printf("%12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s\n",
            "crc_pass","crc_fail","pkt_abort","tx_pkts","cca_idle","rx_start_stuck_stk","rx_cca_stk","fifo_occupy","phy_abort","rx_pkt_tout","rssi");
        fprintf(pFile, "%12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s\n",
            "crc_pass","crc_fail","pkt_abort","tx_pkts","cca_idle","rx_start_stuck_stk","rx_cca_stk","fifo_occupy","phy_abort","rx_pkt_tout","rssi");
#else
        printf("%12s %12s %12s\n",
				"crc_pass","crc_fail","rssi");
        fprintf(pFile,"\n%12s %12s %12s\n",
				"crc_pass","crc_fail","rssi");
#endif
      }
#if 0
      printf("%12d %12d %12d %12d %12d %12d %12d %12d %12d %12d %12d\n",
          sta_info->crc_pass,
          sta_info->crc_fail,
          sta_info->pkt_abort,
          sta_info->tx_pkts,
          sta_info->cca_idle,
          sta_info->rx_start_stuck_stk,
          sta_info->rx_cca_stk,
          sta_info->fifo_occupy,
          sta_info->phy_abort,
          sta_info->rx_pkt_tout,
		  sta_info->rssi);
      fprintf(pFile, "%12d %12d %12d %12d %12d %12d %12d %12d %12d %12d %12d\n",
          sta_info->crc_pass,
          sta_info->crc_fail,
          sta_info->pkt_abort,
          sta_info->tx_pkts,
          sta_info->cca_idle,
          sta_info->rx_start_stuck_stk,
          sta_info->rx_cca_stk,
          sta_info->fifo_occupy,
          sta_info->phy_abort,
          sta_info->rx_pkt_tout,
		  sta_info->rssi);
#else
      printf("%12d %12d %12d \n",
          sta_info->crc_pass,
          sta_info->crc_fail,
		      sta_info->rssi);
      fprintf(pFile, "%12d %12d %12d\n",
          sta_info->crc_fail,
          sta_info->crc_pass,
		      sta_info->rssi);
#endif

      ++count;    
      free(sta_info);
    }    
    fflush(pFile);
#endif
  }
}




int_32 bb_write(uint_32 bb_len, uint_32 soft_rst, uint_32 * BUFFER)
{
  uint_16 val, i,j = 0, index = 0, ii, k;
  uint_8 blocks, count,count1, write_block = 50;
  val = (uint_16)bb_len;
  struct bb_rf_param_t  bb_rf_params;

  memset(&bb_rf_params, 0, sizeof(bb_rf_params));
  blocks = (bb_len/(write_block)); 
  count = (bb_len%(write_block)); 
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
    printf(" addr[%d]= %08x, Data= %08x\n",k,BUFFER[0],BUFFER[0+1]);
    //	printf("index = %d :\n",index);
    for (ii=0,k=1 ; ii<count1*2; j+=2,ii+=2,k++)
    {
      bb_rf_params.Data[ii] = *(uint_16 *)&BUFFER[j];
      bb_rf_params.Data[ii+1] = *(uint_16 *)&BUFFER[j+1];

      		printf(" addr[%d]= %08x, Data= %08x\n",k,bb_rf_params.Data[j],bb_rf_params.Data[j+1]);
      		printf(" addr[%d]= %08x, Data= %08x\n",k,BUFFER[j],BUFFER[j+1]);
    }

    val = (count1 * 2);
    //	printf("writing BB len is:%d\n",val);
    bb_rf_params.value = 1;
    bb_rf_params.no_of_values = val;
    bb_rf_params.soft_reset = (uint_8)soft_rst;
    //	printf("writing BB value is:%d\n",bb_rf_params.value);
    //	printf("writing BB no_of_vals is:%d\n",bb_rf_params.no_of_values);
    // printf("writing BB reset is:%d\n",bb_rf_params.soft_reset);
    if( rsi_send_to_drv((uint_8 *)&bb_rf_params, ZIGB_PER,sizeof(bb_rf_params)) < 0 )
    {
      printf("Unable to perform Ant_sel\n");
      return -1;
    }

  }  
  return 0;
}
int
getcmdnumber (char *command)
{
	if(!strcmp(command, "cw_mode")) 
	{
		return RSI_SET_CW_MODE;
	}
	if(!strcmp(command, "bb_write")) 
	{
		return RSI_SET_BB_WRITE;
	}
	if(!strcmp(command, "ant_sel")) 
	{
		return RSI_ANT_SEL;
	}
	if(!strcmp(command, "bb_read")) 
	{
		return RSI_SET_BB_READ;
	}
	if(!strcmp(command, "rf_write"))
	{
		return RSI_RF_WRITE;
	}
	if(!strcmp(command, "rf_read"))
	{
		return RSI_RF_READ;
	}
	if(!strcmp(command, "zb_stats"))
	{
		return RSI_GET_ZIGB_STATS;
	}
	if(!strcmp(command, "set_channel"))
	{
		return RSI_SET_ZIGB_CHANNEL;
	}
	if(!strcmp(command, "zigb_stats"))
	{
		return RSI_ZIGB_STATS;
	}
	else
	usage();
	return 0;
}


/** This function gives the usage of the onebox utility
 * @param  void
 * @return void 
 */
void usage()
{
  ONEBOX_PRINT( "Usage: ./zb_util  bb_read addr  \n");
  ONEBOX_PRINT( "Usage: ./zb_util  bb_write addr data \n");
  ONEBOX_PRINT( "Usage: ./zb_util  ant_sel  value \n");
  ONEBOX_PRINT( "Usage: ./zb_util  rf_read  addr \n");
  ONEBOX_PRINT( "Usage: ./zb_util  rf_write addr data\n");
  ONEBOX_PRINT( "Usage: ./zb_util  set_channel channel\n");
  ONEBOX_PRINT( "Usage: ./zb_util  zb_stats <File_name>\n");
  ONEBOX_PRINT( "Usage: ./zb_util  cw_mode <channel(11-26)> <start-0/stop-2> <ant_sel(2/3)> \n");
	return ;
}
