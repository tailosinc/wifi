#include "platform_specific.h"
#include "rsi_serial.h"
#include "rsi_common_types.h"
#include "rsi_zigb_global.h"
#include "rsi_zigb_api.h"
#include "rsi_zigb_app.h"
#include "rsi_zigb_config.h"
#include "rsi_zigb_interfaces.h"
#include "rsi_zigb_oper_mode.h"
#include "rsi_frame.h"
#include <sys/time.h>
#include <stdio.h>
#include <semaphore.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <signal.h>
#include <linux/genetlink.h>
#include <termios.h>
#include <linux/types.h>                /* for "caddr_t" et al          */
#include <linux/socket.h>               /* for "struct sockaddr" et al  */
#include <linux/if.h>
#include <sys/ioctl.h>
#include <time.h>
#include<stdarg.h>           
#include <termios.h>


sem_t  sem_cmd_rsp;
sem_t  sem_cmd_wait;
sem_t  sem_udp_send;
void log_and_print_info(const char *fmt, ...);
extern uint8_t wifi_packet;

extern int16_t rsi_zigb_oper_mode(void);

//#include "gs_common.h"
#define MAX_NUM_ARG 15

rsi_linux_app_cb_t   rsi_linux_app_cb;

extern rsi_zigb_app_cb_t   rsi_zigb_app_cb;
rsi_zigb_app_info_t rsi_zigb_app_info;
extern APP_Startup_Attribute_Set_t APP_Startup_Attribute_Set_Default;
extern ZDO_Configuration_Table_t g_Table_Default;
extern Simple_Descriptor_t SimpleDesc;

int32_t rsi_zigb_framework_init(void);
int16_t rsi_frame_read(uint8_t *packet_buffer);
rsi_zigb_app_cb_t     *app_cb_ptr = &rsi_zigb_app_cb;
rsi_zigb_app_info_t   *app_info = &rsi_zigb_app_info;
ZigBeeNetworkDetails  *nwk_details = &(rsi_zigb_app_info.nwkinfo); 
uint8_t  device_type; 
rsi_zigb_uCmdRsp      *resp;
uint8_t g_endpoint =1;
uint16_t g_short_addr=0;

extern rsi_pkt_t *rsi_dequeue_from_rcv_q(void);
extern void rsi_queues_init(pkt_queue_t *queue);
extern void rsi_platform_based_init(void);
uint32_t tx_packet_counter=0;
uint32_t dataConfcnt;
#define THROUPUT_DISPLAY_MASK 0x10000

typedef struct
{
	char *cmd;
	int (*func)(int argc, char *argv[] , char *rsp_buf);
	char *doc;
} client_command;

struct cmdInfo
{
	int argc;
	int multi_command;
	char *argv[MAX_NUM_ARG];
};


int stop_all_threads=0;
int currentPos=0;
int rsi_debug_print=0;
int command_line_interface(int app_mode);
int keyhit;
unsigned long debug_enable_mask = 0x20000;
int enable_dut_debug=1;
static struct termios old, new;
char rsp_buffer[5000]="";
extern int transmit_socket;
extern int client_interface_mode;
char temp_buffer[1000];
struct cmdInfo gCmdBuf;
uint32_t rsi_app_async_event_map = 0;
void signal_call(int signum);
FILE *fLogPtr;
/*############################ Time variables ###############################*/
static uint32_t last_tx_print_time;
static uint32_t last_rx_print_time;

uint32_t total_tx_bytes;
uint32_t total_rx_bytes;
uint32_t secs;
/*############################ Time Variables ###############################*/
int send_response(const char *fmt, ...)
{
	int SockWrite;
	va_list arg;
	memset(&arg,0,sizeof(va_list));
	va_start(arg, fmt);
	vsprintf(temp_buffer, fmt, arg);
	va_end(arg);
	log_and_print_info(temp_buffer);
	return 0;
}



int check_help_print_request(int argc, char *argv[])
{
	if (argc)
	{
		if (( strcmp(argv[0],"?")==0 ) || (strcmp(argv[0],"help")== 0) )
		{
			return 1;
		}
	}
	return 0;
}

void DebugDumpBytes(unsigned char *buffer, unsigned short length, char *pDescription)
{
	log_and_print_info("<---------Dumping %d Bytes : %s ------>\n", length, pDescription);
	if ((enable_dut_debug==1) && (length>0))
	{
        if (debug_enable_mask &0x20000)
		{
			char stream[60];
			char byteOffsetStr[10];
			unsigned long i;
			unsigned short offset, count, byteOffset;
			count = 0;
			offset = 0;
			byteOffset = 0;
			for (i = 0; i < length; i++)
			{
				sprintf(stream + offset, "%2.2X ", buffer[i]);
				count ++;
				offset += 3;

				if (count == 16)
				{
					count = 0;
					offset = 0;
					sprintf(byteOffsetStr,"%4.4X",byteOffset);
					log_and_print_info("[%s]: %s\n", byteOffsetStr, stream);
					memset(stream,0, 60);
					byteOffset += 16;
				}
			}

			if (offset != 0)
			{
				sprintf(byteOffsetStr,"%4.4X",byteOffset);
				log_and_print_info("[%s]: %s\n", byteOffsetStr, stream);
			}
			log_and_print_info("<------------------------------------------------------------------>\n");
		}
	}
}

/*==============================================*/
/**
 * @fn         rsi_zb_app_set_event
 * @brief      set the specific event.
 * @param[in]  event_num, specific event number.
 * @return     none.
 * @section description
 * This function is used to set/raise the specific event.
 */
int rsi_zb_app_set_event(uint32_t event_num)
{
	//uint32_t temp_event = rsi_zb_app_get_event();
	//if(temp_event != event_num)
	{
		rsi_app_async_event_map |= BIT(event_num);
		return 0;
	}
	return 1;
}

/*==============================================*/
/**
 * @fn         rsi_zb_app_clear_event
 * @brief      clear the specific event.
 * @param[in]  event_num, specific event number.
 * @return     none.
 * @section description
 * This function is used to clear the specific event.
 */
void rsi_zb_app_clear_event(uint32_t event_num)
{
	rsi_app_async_event_map &= ~BIT(event_num);
	return;
}

/*==============================================*/
/**
 * @fn         rsi_zb_app_get_event
 * @brief      returns the first set event based on priority
 * @param[in]  none.
 * @return     int32_t
 *             > 0  = event number
 *             -1   = not received any event
 * @section description
 * This function returns the highest priority event among all the set events
 */
int32_t rsi_zb_app_get_event(void)
{
	uint32_t  ix;

	for (ix = 0; ix < 32; ix++)
	{
		if (rsi_app_async_event_map & (1 << ix))
		{
			return ix;
		}
	}

	return(-1);
}

void initTermios(int echo)
{
	tcgetattr(0, &old);	/* grab old terminal i/o settings */
	new = old; /* make new settings same as old settings */
	new.c_lflag &= ~ICANON;	/* disable buffered i/o */
	new.c_lflag &= echo ? ECHO : ~ECHO;	/* set echo mode */
	tcsetattr(0, TCSANOW, &new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void)
{
	tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo)
{
	char ch;
	initTermios(echo);
	ch = getchar();
	resetTermios();
	return ch;
}

/* Read 1 character without echo */
char getch(void)
{
	return getch_(0);
}

int kbhit(void)
{
	struct termios oldt, newt;
	int ch = 0;
	int oldf = 0;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
		printf("\ncli>");
		putchar(ch);
		return ch;
	}


	return 0;
}
#define MAX_CMD_LIST 100
unsigned long command_list[MAX_CMD_LIST+2];

int get_string(char *ptr)
{
	int count;
	gets(ptr);
	count=strlen(ptr);
	return count;

}

int get_command_from_user(char *buffer)
{
	int count=0;
	printf("\ncli>");
	count=get_string(buffer);
	return count;

}
void split_command_and_arquments(char *cmdBuf)
{
	int i=0,j=0;
	char *p;
	char temp[2048];
	int length;
	p=temp;
	for (i=0;i<MAX_NUM_ARG;i++)
	{
		if (gCmdBuf.argv[i])
		{
			free((void *)gCmdBuf.argv[i]);
			gCmdBuf.argv[i]=0;
		}
	}

	i=0;
	while ( 1 )
	{
		if ( *cmdBuf==0 )
		{
			break;
		}
		if ( *cmdBuf==' ' )
		{
			if ( j )
			{
				p[j]=0;
				length=strlen(temp);
				p=(char *)malloc(length+10);
				strcpy(p,temp);
				gCmdBuf.argv[i]=(char *)p;
				j=0;
				i++;
				p=temp;
			}
			cmdBuf++;
			continue;
		}
		p[j]=*cmdBuf;
		cmdBuf++;
		j++;

	}
	if ( j==0 )
	{
		i--;
	}
	else
	{
		p[j]=0;
		length=strlen(temp);
		gCmdBuf.argv[i]=malloc(length+10);
		strcpy((char *)gCmdBuf.argv[i],temp);
	}
	i++;
	gCmdBuf.argc=i;
}
static const char *cmd_setdebugmask_help =
"\nUsage:\n"
"\n\tsetdebugmask <mask>\n"
"\tmask: Mask in Hexadecimal\n"
"\tsetdebugmask\n"
"\nExample:\n"
"\tsetdebugmask FFFFF :Change Debug Mask to 0xFFFFF\n";
int cmd_setdebugmask(int argc, char *argv[],char *rsp)
{

	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_setdebugmask_help);
		return -1;
	}
	if ( argc <1 )
	{
		sprintf(rsp,"Command Require one argument\n%s\n",cmd_setdebugmask_help);
		return -1;
	}
	debug_enable_mask=strtol(argv[0],NULL,16);
	printf("Debug Mask is Set to 0x%X\n",debug_enable_mask);

	return 0;
}
static const char *cmd_init_stack_help =
"\nUsage:\n"
"\tInit_stack\n"
"\nExample:\n"
"\tInit_stack\t(it inits stack for zigbee)\n";
int cmd_init_stack(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_init_stack_help);
		return -1;
	}
	rsi_zigb_init_stack();
	return 1;
}
static const char *cmd_reset_stack_help =
"\nUsage:\n"
"\tReset_stack\n"
"\nExample:\n"
"\tReset_stack\t(it Resets stack for zigbee)\n";
int cmd_reset_stack(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_reset_stack_help);
		return -1;
	}
	rsi_zigb_reset_stack();
	return 1;
}
static const char *cmd_update_sas_help =
"\nUsage:\n"
"\tUpdate_sas\n"
"\nExample:\n"
"\tUpdate_zdo\t(it updates SAS value for zigbee)\n";
int cmd_update_sas(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_update_sas_help);
		return -1;
	}
	rsi_zigb_update_sas(&APP_Startup_Attribute_Set_Default);
	return 1;
}
static const char *cmd_update_zdo_help =
"\nUsage:\n"
"\tUpdate_zdo\n"
"\nExample:\n"
"\tUpdate_zdo\t(it updates ZDO values for zigbee)\n";
int cmd_update_zdo(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_update_zdo_help);
		return -1;
	}
	rsi_zigb_update_zdo_configuration(&g_Table_Default);
	return 1;
}
static const char *cmd_get_self_ieee_addr_help =
"\nUsage:\n"
"\tGet_self_ieee_addr\n"
"\nExample:\n"
"\tGet_self_ieee_addr\t(it gets self ieee address of device)\n";
int cmd_get_self_ieee_addr(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_get_self_ieee_addr_help);
		return -1;
	}
	rsi_zigb_get_self_ieee_address();
	return 1;
}
static const char *cmd_get_device_type_help =
"\nUsage:\n"
"\tGet_device_type\n"
"\nExample:\n"
"\tGet_device_type\t(it gets device type)\n";
int cmd_get_device_type(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_get_device_type_help);
		return -1;
	}
	rsi_zigb_get_device_type();
	return 1;
}
static const char *cmd_set_simple_desc_help =
"\nUsage:\n"
"\tSet_simple_desc\n"
"\nExample:\n"
"\tSet_simple_desc\t(it set the simple descriptor for the given endpoint)\n";
int cmd_set_simple_desc(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_set_simple_desc_help);
		return -1;
	}
	rsi_zigb_set_simple_descriptor(ONOFF_SWITCH_END_POINT, app_info->DeviceSimpleDesc);
	return 1;
}
static const char *cmd_scan_help =
"\nUsage:\n"
"\tScan <channel>\n"
"\tchannel :11 to 26\n"
"\nExample:\n"
"\tScan 21 \t(it scans the zigbee network)\n";
int cmd_scan_network(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_scan_help);
		return -1;
	}
	if ( argc <1 )
	{
		printf("Command Require one argument for valid channel\n");
		return -1;
	}
	uint32_t channel=atoi(argv[0]);
	channel = 1 << channel;	
	rsi_zigb_initiate_scan(g_MAC_ACTIVE_SCAN_TYPE_c, channel, g_SCAN_DURATION_c);
	return 1;
}
static const char *cmd_form_network_help =
"\nUsage:\n"
"\tForm_network <channel>\n"
"\tchannel :11 to 26\n"
"\nExample:\n"
"\tForm_network 21 \t(it Forms the zigbee network)\n";
int cmd_form_network(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_form_network_help);
		return -1;
	}
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	uint8_t ext_pan_id[8]={0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01};
	uint8_t channel = atoi(argv[0]);
	return rsi_zigb_form_network(channel, app_cb_ptr->power, ext_pan_id);
}
static const char *cmd_join_network_help =
"\nUsage:\n"
"\tJoin_network\n"
"\nExample:\n"
"\tJoin_network\t(it joins with the scanned network for zigbee)\n";
int cmd_join_network(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_join_network_help);
		return -1;
	}
	device_type=1;
	rsi_zigb_join_network(device_type, nwk_details->channel, 0x12, nwk_details->extendedPanId);  
	return 1;
}
static const char *cmd_permit_join_help =
"\nUsage:\n"
"\tPermit_join\n"
"\nExample:\n"
"\tPermit_join\t(it set the permit join with FF duration)\n";
int cmd_permit_join(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_permit_join_help);
		return -1;
	}
	rsi_zigb_permit_join(0xFF);
	return 1;
}
static const char *cmd_get_rssi_help =
"\nUsage:\n"
"\tGet_rssi\n"
"\nExample:\n"
"\tGet_rssi\t(it gets the rssi value from firmware)\n";
int cmd_get_rssi(int argc, char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_get_rssi_help);
		return -1;
	}
	rsi_zigb_get_rssi();
	return 1;
}
static const char *cmd_get_dmsg_help =
"\nUsage:\n"
"\td\n"
"\nExample:\n"
"\td\t(it gets debug details from firmware)\n";
int cmd_get_dmsg(int argc, char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_get_dmsg_help);
		return -1;
	}
	fprintf(fLogPtr,"\nPacket sent=%d Acknowledgement Received Successfully=%d\n",tx_packet_counter,dataConfcnt);
	rsi_zigb_get_debug_info(GET_MEMORY_TABLE,0);
	sleep(1);
	rsi_zigb_get_debug_info(GET_TRANSACTION_TABLE,0);
	sleep(1);
	rsi_zigb_get_debug_info(GET_MAC_QUEUE_TABLE,0);
	sleep(1);
	rsi_zigb_get_debug_info(GET_APP_QUEUE_TABLE,0);
	sleep(1);
	rsi_zigb_get_debug_info(GET_MSC_QUEUE_TABLE,0);
	sleep(1);
	rsi_zigb_get_debug_info(GET_SAS_INFO,0);
	
}
static const char *cmd_get_msdu_help =
"\nUsage:\n"
"\tm\n"
"\nExample:\n"
"\tm\t(it gets Msdu table details from firmware)\n";
int cmd_get_msdu(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_get_msdu_help);
		return -1;
	}
	rsi_zigb_get_debug_info(GET_MSDU_TABLE,0);
	return 1;

}
static const char *cmd_enable_dbg_help =
"\nUsage:\n"
"\te <option>\n"
"\toption : 0-disable 1-enable\n"
"\nExample:\n"
"\te 1\t(it enable debug prints in firmware)\n";
int cmd_enable_dbg(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_enable_dbg_help);
		return -1;
	}
	uint8_t state;
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	state = atoi(argv[0]);
	rsi_zigb_get_debug_info(SET_ZIGBEE_DEBUG_MODE,state);
	return 1;
}
void rsi_change_heart_beat_report(unsigned char flag);
static const char *cmd_set_hb_help =
"\nUsage:\n"
"\tset_hb <state>\n"
"\tstate : 1\n"
"\nExample:\n"
"\tset_hb 1\t(When link status packet transmitted it prints)\n";
int cmd_set_heart_beat_update(int argc, char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_set_hb_help);
		return -1;
	}
	uint8_t state;
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	state = atoi(argv[0]);
	rsi_change_heart_beat_report(state);
	return 1;
}


struct timespec ts_rx_thread,ts_send_data,ts_cmd_wait;
int stop_sending_udp_packet=0;
void *udp_send_handler(void* args)
{
	int ret,i;
	char tx_data_buffer[200];
	for (i=0;i<100;i++)
	{
		tx_data_buffer[i]=(i%256);
	}
	tx_data_buffer[0]=0xAA;
	tx_data_buffer[1]=0x55;
	tx_data_buffer[2]=0xBB;
	tx_data_buffer[3]=0x55;
	while (1)
	{
		if (stop_sending_udp_packet== 0)
		{
			tx_packet_counter++;
			tx_data_buffer[4]=(unsigned char)((tx_packet_counter>>24)&0xFF);
			tx_data_buffer[5]=(unsigned char)((tx_packet_counter>>16)&0xFF);
			tx_data_buffer[6]=(unsigned char)((tx_packet_counter>>8)&0xFF);
			tx_data_buffer[7]=(unsigned char)(tx_packet_counter&0xFF);
			rsi_zigb_app_send_data( g_Client_To_Server_c, g_Cluster_Specific_Cmd_c, g_endpoint, g_short_addr , g_TOGGLE_c, 0x0006, 45, tx_data_buffer);
		}
		if (clock_gettime(CLOCK_REALTIME, &ts_send_data) == -1)
		{
			printf("Failed to get Time");
		}
		ts_send_data.tv_sec += 2;		//Timeout 1 seconds
		ret = sem_timedwait(&sem_udp_send, &ts_send_data);
		if (stop_sending_udp_packet== 2)
		{
			break;
		}
	}

}
static const char *cmd_leave_nwk_help =
"\nUsage:\n"
"\tLeave_network\n"
"\nExample:\n"
"\tLeave_network\t(it leave the zigbee network)\n";
int cmd_leave_network(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_leave_nwk_help);
		return -1;
	}
	rsi_zigb_leave_network();
	return 1;
}
static const char *cmd_nwk_state_help =
"\nUsage:\n"
"\tNetwork_state\n"
"\nExample:\n"
"\tNetwork_state\t(it read the cuurent network state)\n";
int cmd_network_state(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_nwk_state_help);
		return -1;
	}
	rsi_zigb_network_state();
	return 1;
}
static const char *cmd_get_self_short_addr_help =
"\nUsage:\n"
"\tGet_self_short_address\n"
"\nExample:\n"
"\tGet_self_short_address\t(it gets the self short address of the device)\n";
int cmd_short_address(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_get_self_short_addr_help);
		return -1;
	}
	rsi_zigb_get_self_short_address();
	return 1;
}
static const char *cmd_set_max_in_help =
"\nUsage:\n"
"\tSet_max_incoming_txr_size <size>\n"
"\tsize :max possible Incoming tx frame size\n"
"\nExample:\n"
"\tSet_max_incoming_txr_size 10\t(It set the maximum incoming tx frame size to ZigBee stack)\n";
int cmd_set_max_incoming_txr_size(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_set_max_in_help);
		return -1;
	}
	uint16_t size;
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	size=atoi(argv[0]);
	rsi_zigb_set_maxm_incoming_txfr_size(size);
	return 1;
}
static const char *cmd_set_max_out_help =
"\nUsage:\n"
"\tSet_max_outgoing_txr_size <size>\n"
"\tsize :max possible outgoing tx frame size\n"
"\nExample:\n"
"\tSet_max_outgoing_txr_size 10\t(It set the maximum outgoing tx frame size to ZigBee stack)\n";
int cmd_set_max_outgoing_txr_size(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_set_max_out_help);
		return -1;
	}
	uint16_t size;
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	size=atoi(argv[0]);
	rsi_zigb_set_maxm_out_going_txfr_size(size);
	return 1;
}
static const char *cmd_set_channel_help =
"\nUsage:\n"
"\tSet_Operating_Channel <channel>\n"
"\tchannel :11 to 26\n"
"\nExample:\n"
"\tSet_Operating_Channel 25\t(It sets the operating channel in ZigBee stack)\n";
int cmd_set_oper_channel(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_set_channel_help);
		return -1;
	}
	uint8_t channel;
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	channel = atoi(argv[0]);
	rsi_zigb_set_operating_channel(channel);
	return 1;
}
static const char *cmd_get_short_addr_help =
"\nUsage:\n"
"\tGet_short_addr <ieee_addr>\n"
"\tieee_addr : ieee address to query\n"
"\nExample:\n"
"\tGet_short_addr 1234567812345678\t(it gets the short address for specific 64-bit IEEE address)\n";
int cmd_get_short_addr_for_specified_ieee_addr(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_get_short_addr_help);
		return -1;
	}
	uint8_t *pIEEEaddr;
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	pIEEEaddr =atoi(argv[0]);
	rsi_zigb_get_short_addr_for_specified_ieee_addr(pIEEEaddr);
	return 1;
}
static const char *cmd_get_ieee_addr_help =
"\nUsage:\n"
"\tGet_ieee_addr <short_addr>\n"
"\tshort_addr : short address to query\n"
"\nExample:\n"
"\tGet_ieee_addr 1234\t(it gets the 64-bit extended address for the given short address)\n";
int cmd_get_ieee_addr_for_specified_short_addr(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_get_ieee_addr_help);
		return -1;
	}
	uint16_t *shortaddr;
	//uint8_t *ieeeadd;
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	shortaddr =atoi(argv[0]);
	rsi_zigb_get_ieee_addr_for_specified_short_addr(shortaddr);
	return 1;
}

pthread_t udp_send_thread;
int thread_running=0;
static const char *cmd_send_data_help =
"\nUsage:\n"
"\tSend_data\n"
"\nExample:\n"
"\tSend_data\t(Coordinator sends data to joined child,Child sends data to coordinator)\n";
int cmd_send_data(int argc, char *argv[],char *rsp)
{
	int err;

	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_send_data_help);
		return -1;
	}
	if (thread_running==1)
	{
		return;
	}
	if ((err = pthread_create(&udp_send_thread, NULL, &udp_send_handler, NULL)))
	{
		rsi_os_printf(RSI_PL1,"\n ERROR occured in thread creation\n");
	}
	return 1;
}
static const char *cmd_read_neighbor_table_help =
"\nUsage:\n"
"\tRead_neigh_table_entry <index>\n"
"\tindex :index of neighbor table\n"
"\nExample:\n"
"\tRead_neigh_table_entry 1\t(It read the neighbour rout table entry)\n";
int cmd_read_neighbor_table_entry(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_read_neighbor_table_help);
		return -1;
	}
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	uint8_t index;
	index =atoi(argv[0]);
	return rsi_zigb_read_neighbor_table_entry(index); 
}
static const char *cmd_get_rout_entry_help =
"\nUsage:\n"
"\tGet_route_table_entry <index>\n"
"\tindex :index of rout table\n"
"\nExample:\n"
"\tGet_route_table_entry 1\t(It gets the rout table entry)\n";
int cmd_get_route_table_entry(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_get_rout_entry_help);
		return -1;
	}
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	uint8_t index;
	index = atoi(argv[0]);
	rsi_zigb_get_route_table_entry(index);
	return 1;
}
static const char *cmd_neigh_count_help =
"\nUsage:\n"
"\tGet_neigh_table_count\n"
"\nExample:\n"
"\tGet_neigh_table_count\t(It get neighbour table entry count)\n";
int cmd_get_neighbor_table_entry_count(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_neigh_count_help);
		return -1;
	}
	return rsi_zigb_get_neighbor_table_entry_count();
}
static const char *cmd_get_child_detail_help =
"\nUsage:\n"
"\tGet_child_details <index>\n"
"\tindex :index of the child of interest\n"
"\nExample:\n"
"\tGet_child_details 1\t(Sends the child short address index read command to ZigBee module)\n";
int cmd_get_child_details(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_get_child_detail_help);
		return -1;
	}
	if (argc<1)
	{
		printf("Require one more arguement\n");
		return -1;
	}
	uint8_t index;
	index =atoi(argv[0]);
	rsi_zigb_get_child_details(index);
	return 1;
}
static const char *cmd_poll_data_help =
"\nUsage:\n"
"\tPoll_for_data\n"
"\nExample:\n"
"\tPoll_for_data\t(End device polls for data)\n";
int cmd_end_device_poll_for_data(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_poll_data_help);
		return -1;
	}
	rsi_zigb_end_device_poll_for_data();
	return 1;
}
static const char *cmd_read_child_count_help =
"\nUsage:\n"
"\tRead_count_child\n"
"\nExample:\n"
"\tRead_count_child\t(it reads number of child joined)\n";
int cmd_read_count_of_child_devices(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_read_child_count_help);
		return -1;
	}
	rsi_zigb_read_count_of_child_devices();
	return 1;
}
static const char *cmd_device_ance_help =
"\nUsage:\n"
"\tZDP_device_announcement\n"
"\nExample:\n"
"\tZDP_device_announcement\t(it sends device announcement on air)\n";
int cmd_zdp_send_device_announcement(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_device_ance_help);
		return -1;
	}
	rsi_zigb_zdp_send_device_announcement();
	return 1;
}
static const char *cmd_get_buffer_queue_help =
"\nUsage:\n"
"\tGet_buffer_queue <query_type>\n"
"\tquery_type :0 to 6\n"
"\nExample:\n"
"\tGet_buffer_queue 3\t(it gets buffer and queue details from firmware)\n";
int cmd_get_buffer_queue(int argc, char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_get_buffer_queue_help);
		return -1;
	}
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	uint8_t query_type = atoi(argv[0]);
	uint8_t data = atoi(argv[1]);
	rsi_zigb_get_debug_info(query_type,data);
	return 1;
}
static const char *cmd_pause_data_help =
"\nUsage:\n"
"\tp\n"
"\nExample:\n"
"\tp\t(it pauses data transfermation with other device)\n";
int cmd_pause_data(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_pause_data_help);
		return -1;
	}
	stop_sending_udp_packet=1;
	if (sem_post(&sem_udp_send) == -1)
	{
		printf("Failed to Wakeup Semaphore \n ");
	}
	else
	{
		printf("UDP packet sending stopped\n");
	}
	return 1;
}
static const char *cmd_resume_data_help =
"\nUsage:\n"
"\tr\n"
"\nExample:\n"
"\tr\t(it resumes data transfermation with other device)\n";
int cmd_resume_data(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_resume_data_help);
		return -1;
	}
	stop_sending_udp_packet=0;
	if (sem_post(&sem_udp_send) == -1)
	{
		printf("Failed to Wakeup Semaphore \n ");
	}
	else
	{
		printf("UDP packet sending started...\n");
	}
	return 1;
}
static const char *cmd_rejoin_help =
"\nUsage:\n"
"\trj\n"
"\nExample:\n"
"\trj\t(it rejoins with zigbee network)\n";
int cmd_rejoin(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_rejoin_help);
		return -1;
	}
	rsi_zigb_rejoin_network(1); 
	return 1;
}
static const char *cmd_oper_mode_help =
"\nUsage:\n"
"\tOper_mode\n"
"\nExample:\n"
"\tOper_mode\t(it sets oper mode for zigbee)\n";
int cmd_opermode(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_oper_mode_help);
		return -1;
	}
	rsi_zigb_oper_mode();
	return 1;
}
static const char *cmd_disconnect_help =
"\nUsage:\n"
"\tdc\n"
"\nExample:\n"
"\tdc\t(it disconnects from zigbee network)\n";
int cmd_disconnect(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_disconnect_help);
		return -1;
	}
	rsi_zigb_leave_network(); 
	return 1;
}
static const char *cmd_start_coord_help =
"\nUsage:\n"
"\tstart_coord <channel>\n"
"\tchannel :11 to 26\n"
"\nExample:\n"
"\tstart_coord 21 \t(it starts the zigbee network)\n";
int cmd_start_coord(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_start_coord_help);
		return -1;
	}
	if (argc<1)
	{
		printf("Require one more arguemnet\n");
		return -1;
	}
	uint8_t ext_pan_id[8]={0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01};
	uint8_t channel = atoi(argv[0]);

	rsi_zigb_oper_mode();
	usleep(30000);
	printf("\nOper mode done press any key for next command\n");
	getchar();	
	rsi_zigb_init_stack();
	usleep(30000);
	printf("Init stack done press any key for next command\n");
	getchar();
	rsi_zigb_reset_stack();
	usleep(30000);
	printf("Reset stack done press any key for next command]\n");
	getchar();
	rsi_zigb_update_sas(&APP_Startup_Attribute_Set_Default);
	usleep(30000);
	printf("SAS done press any key for next command\n");
	getchar();
	rsi_zigb_update_zdo_configuration(&g_Table_Default);
	usleep(30000);
	printf("ZDO configuration done press any key for next command\n");
	getchar();
	rsi_zigb_form_network(channel, app_cb_ptr->power, ext_pan_id);
	usleep(30000);
	printf("Wait for zigbee network is up then press any key for Permit join command\n");
	getchar();
	rsi_zigb_permit_join(0xFF);
	usleep(30000);
	printf("Permit join done wait for any one child to join and then press any key to send data");
	getchar();
	cmd_send_data(0,0,0);	
}
static const char *cmd_start_rout_help =
"\nUsage:\n"
"\tstart_rout <channel>\n"
"\tchannel :11 to 26\n"
"\nExample:\n"
"\tstart_rout 21 \t(it start the zigbee network)\n";
int cmd_start_rout(int argc,char *argv[],char *rsp)
{
	if (check_help_print_request(argc,argv))
	{
		strcpy(rsp,cmd_start_rout_help);
		return -1;
	}
	if ( argc <1 )
	{
		printf("Command Require one argument for valid channel\n");
		return -1;
	}
	uint8_t device_type = 1;
	uint32_t channel=atoi(argv[0]);
	channel = 1 << channel;

	rsi_zigb_oper_mode();
	usleep(30000);
	printf("\nOper mode done press any key for next command\n");
	getchar();	
	rsi_zigb_init_stack();
	usleep(30000);
	printf("Init stack done press any key for next command\n");
	getchar();
	rsi_zigb_reset_stack();
	usleep(30000);
	printf("Reset stack done press any key for next command]\n");
	getchar();
	rsi_zigb_update_sas(&APP_Startup_Attribute_Set_Default);
	usleep(30000);
	printf("SAS done press any key for next command\n");
	getchar();
	rsi_zigb_update_zdo_configuration(&g_Table_Default);
	usleep(30000);
	printf("ZDO configuration done press any key for next command\n");
	getchar();
	rsi_zigb_initiate_scan(g_MAC_ACTIVE_SCAN_TYPE_c, channel, g_SCAN_DURATION_c);
	usleep(30000);
	printf("Scanning...\n After scan complete handler press any key for next command\n");
	getchar();
	rsi_zigb_join_network(device_type, nwk_details->channel, 0x12, nwk_details->extendedPanId);
	usleep(30000);
	printf("Wait for join complete and zigbee network is up and then press any key to send data\n");
	getchar();
	cmd_send_data(0,0,0);	
}
client_command commands_zigbee[] = 
{
	{ "setdebugmask",cmd_setdebugmask, "Set Debug Mask"},
	{ "Oper_mode",cmd_opermode,"Opermode"},
	{ "Init_stack",cmd_init_stack,"stack init"},
	{ "Reset_stack",cmd_reset_stack,"stack reset"},
	{ "Update_sas",cmd_update_sas,"update_sas"},
	{ "Update_zdo",cmd_update_zdo,"update_zdo"},
	{ "Get_self_ieee_addr",cmd_get_self_ieee_addr,"get_self_ieee_addr"},
	{ "Get_device_type",cmd_get_device_type,"get_device_type"},
	{ "Set_simple_desc",cmd_set_simple_desc,"set_simple_desc"},
	{ "Scan",cmd_scan_network,"scan_network"},					//needs one argument for channel
	{ "Form_network",cmd_form_network,"form_network"},				//needs one argument for channel
	{ "Join_network",cmd_join_network,"join_network"},				//Gets channel and extended pan id from scan results
	{ "Permit_join",cmd_permit_join,"permit_join"},
	{ "Send_data",cmd_send_data,"send_data"},
	{"e",cmd_enable_dbg,"Enable debug Trace from Zigbee"},				//Enable debug prints by e
	{"k",cmd_pause_data,"Pause the data"},						//To pause the unicast data transfermation
	{"p",cmd_pause_data,"Pause the data"},
	{"r",cmd_resume_data,"Reseume data"},						//To resume the unicast data transfermation
	{"rj",cmd_rejoin,"Rejoin"},
	{"dc",cmd_disconnect,"Disconnect from network"},
	{"d",cmd_get_dmsg,"Get_dmesg"},
	{"m",cmd_get_msdu,"Get_Msdu_table from firmware"},
	{"set_hb",cmd_set_heart_beat_update,"Change Heart Beat reporting"},
	{"Get_buffer_queue",cmd_get_buffer_queue,"get_buffer_queue"},
	{"Leave_network",cmd_leave_network,"leave_network"},
	{"Network_state",cmd_network_state,"network_state"},
	{"Get_self_short_address",cmd_short_address,"get_self_short_address"},
	{"Get_rssi",cmd_get_rssi,"get_rssi"},
	{"Set_max_incoming_txr_size",cmd_set_max_incoming_txr_size,"set_max_incoming_txfr_size"},
	{"Set_max_outgoing_txr_size",cmd_set_max_outgoing_txr_size,"set_max_outgoing_txfr_size"},
	{"Set_Operating_Channel",cmd_set_oper_channel,"set_operating_channel"},
	{"Get_short_addr",cmd_get_short_addr_for_specified_ieee_addr,"get_short_addr_for_specified_ieee_addr"},
	{"Get_ieee_addr",cmd_get_ieee_addr_for_specified_short_addr,"get_ieee_addr_for_specified_short_addr"},
	{"Read_neigh_table_entry",cmd_read_neighbor_table_entry,"read_neighbour_table_entry"},
	{"Get_route_table_entry",cmd_get_route_table_entry,"get_route_table_entry"},
	{"Get_neigh_table_count",cmd_get_neighbor_table_entry_count,"get_neighbour_table_entry_count"},
	{"Get_child_details",cmd_get_child_details,"get_child_details"},
	{"Poll_for_data",cmd_end_device_poll_for_data,"end_device_poll_for_data"},
	{"Read_count_child",cmd_read_count_of_child_devices,"Read_count_child_devices"},
	{"ZDP_device_announcement",cmd_zdp_send_device_announcement,"zdp_send_device_announcement"},
	{"start_coord",cmd_start_coord,"start coord"},
	{"start_rout",cmd_start_rout,"start rout"},
	{ NULL, NULL, NULL}
};

void print_zigbee_cmd_help(char *rsp)
{
	int i=0;
	char temp[5000];
	strcat(rsp,"Commands Supported:-\n");
	strcat(rsp,"----------------------\n");
	while ( 1 )
	{
		if ( commands_zigbee[i].cmd == NULL )
			break;
		sprintf(temp,"\t%-30s :  %s\n", commands_zigbee[i].cmd,commands_zigbee[i].doc);
		strcat(rsp,temp);
		i++;
	}
	strcat(rsp,"\n");
	printf("\n%s",rsp);

}

int handle_zigbee_commands(int argc, char *argv[],char *rsp)
{
	int detected=0;
	int i=0;
	char *cmdStr;
	int ret_status=-1;
	int ret;

	cmdStr=argv[0];

	for ( i = 0; commands_zigbee[i].cmd; i++ )
	{
		if ( strcmp(commands_zigbee[i].cmd, cmdStr) !=0  )
		{
			continue;
		}
		detected=1;
		argc--;
		argv++;
		printf("Received Command is %s\n",cmdStr);
		printf("Argument count %d\n", argc);
		ret_status = commands_zigbee[i].func(argc,argv,rsp);
		if (ret_status==1)
		{
			if (clock_gettime(CLOCK_REALTIME, &ts_cmd_wait) == -1)
			{
				printf("Failed to get Time");
			}
			ts_cmd_wait.tv_sec += 10;		//Timeout 10 seconds
			ret = sem_timedwait(&sem_cmd_wait, &ts_cmd_wait);
			ret_status=0;
		}
		break;
	}

	if ( detected==0 )
	{
		return -3;
	}
	return ret_status;
}
int handle_command(char *buff_ptr,char *rsp)
{
	int ret_status=0;
	char **argv;
	int argc;
	if ( strlen(buff_ptr) )
	{
		split_command_and_arquments(buff_ptr);
	}
	argv=gCmdBuf.argv;
	argc=gCmdBuf.argc;
	gCmdBuf.multi_command=0;
	ret_status=handle_zigbee_commands(argc,argv,rsp);
	if (ret_status==-3 ||ret_status == -1)
	{
		printf("Command \'%s\' Not Supported \n%s\n",buff_ptr,rsp);
	}
	return ret_status;

}

int interactive_mode_state=0;
extern unsigned long Packet_Received;

void *response_thread_handler(void* args)
{

	uint8_t ret,rsi_status,cmd_type,rx_length;
	pkt_queue_t *rcv_q = &rsi_linux_app_cb.rcv_queue;
	while (1)
	{
		rx_length = rsi_frame_read(app_cb_ptr->read_packet_buffer);
		if (rx_length )
		{
			Packet_Received++;
			resp = rsi_zigb_app_frame_process(app_cb_ptr->read_packet_buffer);
			rsi_os_printf(RSI_PL1,"cmd_id : %X intf_id : %X, Length:%d\n", resp->cmd_id,resp->intf_id,rx_length);
			cmd_type = resp->cmd_id;
			rsi_zigb_app_cb_handler(cmd_type, (uint8_t *)&resp->uCmdRspPayLoad.uRspData,rx_length);
		}
		if (rcv_q->pending_pkt_count == 0)
		{
			if (clock_gettime(CLOCK_REALTIME, &ts_rx_thread) == -1)
			{
				printf("Failed to get Time");
			}
			ts_rx_thread.tv_sec += 10;		//Timeout 10 seconds
			ret = sem_timedwait(&sem_cmd_rsp, &ts_rx_thread); 
		}


	}

}


rsi_zigb_app_cb_t   rsi_zigb_app_cb;
int client_interface_mode=0;
int ServerPortno=0;
char command_buffer[1000];
//int send_serial_data(uint8_t *packet_buffer, int16_t length);
void handle_interactive_mode(void)
{
	int count;
	interactive_mode_state=1;
	printf("Starting the Interactive Mode\n");
	while (1)
	{
    	count=get_command_from_user(command_buffer);
		if (strcmp(command_buffer,"quit")==0)
		{
			break;
		}
		//send_serial_data(command_buffer,count);//reyaz
	}
	interactive_mode_state=0;
	printf("Exiting from the Interactive Mode\n");
}

//int main_thread_run(int mode)
int main_thread_run()
{
	int count,ret_status=0;
	RSI_ZB_STATUS  rsi_status = RSI_ZB_SUCCESS;
	struct timeval start_time,end_time,time_diff;
	unsigned long time_val;
	


	while (1)
	{
		memset(rsp_buffer,0,sizeof(rsp_buffer));
		count=get_command_from_user(command_buffer);
		if (count==0)
		{
			continue;
		}
		if (strcmp(command_buffer,"quit")==0)
		{
			break;
		}
		if (strcmp(command_buffer,"q")==0)
		{
			break;
		}
		if (strcmp(command_buffer,"exit")==0)
		{
			break;
		}
		if (strcmp(command_buffer,"x")==0)
		{
			break;
		}
		if (strcmp(command_buffer,"interactive")==0)
		{
			handle_interactive_mode();
		}
		if (strcmp(command_buffer,"?")==0)
		{
			print_zigbee_cmd_help(rsp_buffer);
			continue;
		}

		if (strcmp(command_buffer,"help")==0)
		{
			print_zigbee_cmd_help(rsp_buffer);
			continue;
		}

		if (strcmp(command_buffer,"h")==0)
		{
			print_zigbee_cmd_help(rsp_buffer);
			continue;
		}

		gettimeofday(&start_time, NULL);
		memset(rsp_buffer,0,sizeof(rsp_buffer));
		printf("Command Received: %s\n",command_buffer);
		ret_status=handle_command(command_buffer,rsp_buffer);

	}
}

int Configutre_serial_port(int port,int baud_rate)
{

	//! UART initialisations
	//if (rsi_serial_init(port,baud_rate)) 
	//if (rsi_serial_init(port))
	{
		rsi_os_printf(0xFFFF,"\n ERROR occured in UART initialisation\n");
		return -1;
	}
	//! initialise the UART recv queue
	rsi_queues_init(&rsi_linux_app_cb.rcv_queue);

	//! platform based initalisations
	rsi_platform_based_init();

	return 0;
}

int32_t zigb_main(uint8_t );

/*=================================================*/
/**
 *@fn            int16_t rsi_zigb_oper_mode(rsi_uOperMode *uOperMode)
 * @brief        Sends the OPERATING MODE command to the Wi-Fi module via SPI
 * @param[in]    uint8_t mode value to configure 0 for legacy client mode , 
  *              1 for wifi-direct mode , 2 for enterprise security mode.
 * @param[out]   none
 * @return       errCode
 *               -2 = Command execution failure
 *               -1 = Buffer Full
 *               0  = SUCCESS
 * @section description 
 * This API is used to select the Legacy client mode or P2P mode or Enterprise Security Mode.
 */

int16_t rsi_zigb_oper_mode(void)
{
	int16_t    retval;
	uint8_t uOperMode[16] = {0x00,0x00,0x03,0x00,0x01,0x00,0x00,0x00,0x06,0x00,0x08,0x00,0x00,0x00,0x00,0x00};
	uint8_t rsi_zigb_frameCmdOperMode[2] = {0x00,0x00};
	rsi_os_printf(RSI_PL3,"\r\n\nOperating Mode");

	wifi_packet=1;
	//! Write descriptor and payload
	retval = rsi_zigb_execute_cmd(rsi_zigb_frameCmdOperMode,uOperMode, 16);
	wifi_packet=0;
	return retval;
}

/*===========================================================================
 *
 * @fn          int main()
 * @brief       Function to Initiate ZigBee App
 * @param[in]   none
 * @param[out]  none
 * @return      status
 * @section description
 * This main is used to initiate ZigBee Application
 *
 * ===========================================================================*/
char f_name[100]; 
//extern char *serial_device[];
int main(int argc, char *argv[])
{
	char *f_ptr;

	char *p;
	int com_port,rc1;
	int baud_rate;
	pthread_t     thread1;
	char *message1 = "Recv Thread for serial dev";

	printf("**************************************************************\n");
	printf("*                                                            *\n");
	printf("************  Redpine Zigbee Command Line App*****************\n");
	printf("*                                                            *\n");
	printf("**************************************************************\n");
	if ( argc <=2 )
	{
		printf("Please specify the COM port and Baudrate\n");
	}

	p= argv[1];
	com_port=atoi(p);
	p= argv[2];
	baud_rate=atoi(p);  

	rsi_linux_app_cb.serial_type = com_port;
	rsi_zb_app_init();
	if (sem_init(&sem_cmd_rsp, 0, 0) == -1)
	{
		printf("Failed to initialize semaphore\n");
		exit(0);
	}
	if (sem_init(&sem_cmd_wait, 0, 0) == -1)
	{
		printf("Failed to initialize semaphore\n");
		exit(0);
	}
	if (sem_init(&sem_udp_send, 0, 0) == -1)
	{
		printf("Failed to initialize semaphore\n");
		exit(0);
	}


	//printf("Configutring Serial Port %s with Baud Rate:%d\n",serial_device[com_port],baud_rate);

		if (rsi_zigb_framework_init())
	{
		printf("Failed to Configure the nlink %d\n",com_port);
		return 0;
	}


	if (clock_gettime(CLOCK_REALTIME, &ts_send_data) == -1)
	{
		printf("Failed to get Time");
	}
	sprintf(f_name,"ZigbeeLog_%ld.txt",ts_send_data.tv_sec);
	fLogPtr = fopen(f_name,"w");

	//rsi_zigb_oper_mode();
	main_thread_run();  
	fclose(fLogPtr);
	return 0;

}

/*===========================================================================
 *
 * @fn          int32_t rsi_zigb_framework_init(void)
 * @brief       Prepares the ZigBee Specific descriptor 
 * @param[in]   buffer- buffer pointer to fill the descriptor
 * @param[out]  none
 * @return      Buffer pointer after appending descriptor
 * @section description
 * This API is used to prepare the ZigBee specific descriptor
 *
 * ===========================================================================*/


int16_t rsi_frame_read(uint8_t *packet_buffer)
{
	/* Variables */
	rsi_linux_app_cb_t *linux_app_cbPtr = &rsi_linux_app_cb;

	rsi_pkt_t *rx_pkt = NULL;

	uint16_t length;

	/* Do actual deque from the RX queue */
	pthread_mutex_lock(&linux_app_cbPtr->mutex1);
	rx_pkt = (rsi_pkt_t *)rsi_dequeue_from_rcv_q();
	pthread_mutex_unlock(&linux_app_cbPtr->mutex1);

	if (rx_pkt)
	{
		//! calculate the length from the packet
		length = (rx_pkt->desc[0] | ((rx_pkt->desc[1] & 0xF)<<8)) + RSI_FRAME_DESC_LEN;

		//! copy the data 
		memcpy(packet_buffer,&rx_pkt->desc,length);

		//! Free the packet allocated
		free(rx_pkt);
			
		rx_pkt = NULL;

		return length;
	}

	return 0;
}

/*==============================================*/
/**
 * @fn           void measure_throughput(uint32_t pkt_length, uint32_t tx_rx)
 * @brief        measures throughput.
 * @param[in]    pkt_length of the received packet
 * @param[in]    tx_rx,if tx_rx = 0,indicates the tx pkt
 *                     if tx_rx = 1,indicated the rx pkt
 * @param[out]   none
 * @section description 
 * This API is used to measure throughput of packets recceived/transmitted
 */
void measure_throughput(uint32_t  pkt_length, uint32_t tx_rx)
{
	static uint32_t current_time;
	static uint32_t last_print_time;
	uint32_t total_bytes;
	struct timeval tv1;
	float through_put;
    struct tm t;
	char buf[100];

	gettimeofday(&tv1, NULL);
	localtime_r(&(tv1.tv_sec), &t);
	strftime(buf, 100, "%F %T", &t);

	current_time = tv1.tv_sec * 1000000 + tv1.tv_usec;
	if (tx_rx == 0)
	{
		total_tx_bytes += pkt_length;
		total_bytes = total_tx_bytes;
		last_print_time = last_tx_print_time;
	}
	else
	{
		total_rx_bytes += pkt_length;
		total_bytes = total_rx_bytes;
		last_print_time = last_rx_print_time;
	}
	if ((current_time-last_print_time)>=1000000)	//!for 1 sec
	{
		through_put = (float)(total_bytes*1000.0*8.0) / (current_time-last_print_time);
		if (tx_rx == 0)
		{
			log_and_print_info("%s: Bytes Transmitted %d,Throughput for last %d seconds is = %3.1f Kbps\n",buf,total_bytes,(current_time-last_print_time)/1000000, through_put);
			last_tx_print_time = current_time;
			total_tx_bytes = 0;
		}
		else
		{
			log_and_print_info("%s: Bytes Received %d,Throughput for last %d seconds is = %3.3f Kbps\n",buf,total_bytes,(current_time-last_print_time)/1000000, through_put);
			last_rx_print_time = current_time;
			total_rx_bytes = 0;
		}
	}
	return;
}
