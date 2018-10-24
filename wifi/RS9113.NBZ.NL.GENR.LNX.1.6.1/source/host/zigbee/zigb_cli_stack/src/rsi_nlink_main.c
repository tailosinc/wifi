#include "platform_specific.h"
#include "rsi_nl_app.h"
//#include "rsi_zb_types.h"
//#include "rsi_zigb_global.h"
#include "rsi_zigb_api.h"
#include <sys/socket.h>
#include <stdarg.h>
//#include "rsi_pkt_mgmt.h"
//#include "rsi_driver.h"

#include <semaphore.h>






extern unsigned long debug_enable_mask;
rsi_linux_app_cb_t   rsi_linux_app_cb;
pthread_t zigb_rcv_thd;
extern int enable_dut_debug;
extern sem_t  sem_cmd_rsp;
extern sem_t  sem_cmd_wait;
int32_t rsi_zigb_framework_init(void);
void  *zigb_read_pkt_thread(void *arg);
int16_t rsi_frame_read(uint8_t *packet_buffer);
int32_t zigb_main(uint8_t );
uint8_t *sim1_arr;
extern char temp_buffer[1000];

//



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
  
int32_t rsi_zigb_framework_init(void)
{
  int16_t retval = RSI_ZB_SUCCESS;

  retval = rsi_nl_socket_init();
  if (retval != RSI_ZB_SUCCESS) {
    //RSI_DPRINT(RSI_PL1,"\n Netlink socket creation failed \n");
    return RSI_ZB_FAIL;
  }

#ifdef ZB_DEBUG  
  //RSI_DPRINT(RSI_PL1,"NL Socket created\n");
#endif
  rsi_fill_genl_nl_hdrs_for_cmd();
#ifdef ZB_DEBUG  
  //RSI_DPRINT(RSI_PL1,"NL gnl hdr filled\n");
#endif
  
  if(pthread_create(&zigb_rcv_thd, NULL, zigb_read_pkt_thread, 0)) {
    //RSI_DPRINT(RSI_PL1,"\n Receive Thread creation failed \n");
    return RSI_ZB_FAIL;
  }
  rsi_platform_based_init();
  rsi_queues_init(&rsi_linux_app_cb.rcv_queue);



   return RSI_ZB_SUCCESS;
}

void  *zigb_read_pkt_thread(void *arg)
{
  
  int32_t rsp_len;
  uint16_t ii = 0;
  int32_t fromAddrLen = 0;
  //rsi_pkt_t *rcvPktPtr; 

  char *s = arg;

#ifdef RSI_DEBUG_PRINT
  //RSI_DPRINT(RSI_PL14,"\nRecvThreadBody:\n");
#endif
  //RSI_DPRINT(RSI_PL13,"%s\n",s);
  while(1)
  {
    sim1_arr = (uint8_t *)rsi_malloc(MAX_PACKET_SIZE + RSI_RXPKT_HEAD_ROOM);

    
    rsi_linux_app_cb.rcvPktPtr = (rsi_pkt_t *) malloc( 1600 + sizeof(rsi_pkt_t));

     
    if(rsi_linux_app_cb.rcvPktPtr == NULL)
    {
#ifdef RSI_DEBUG_PRINT
            //RSI_DPRINT(RSI_PL13,"Allocation failed to recv packet\n");
#endif
      return NULL;
    }
    
   
    rsp_len = recv(rsi_linux_app_cb.nl_sd,sim1_arr, MAX_PACKET_SIZE, 0);
    memcpy(&(rsi_linux_app_cb.rcvPktPtr->desc),sim1_arr+24,16);
    memcpy(&(rsi_linux_app_cb.rcvPktPtr->data),sim1_arr+40,(*(sim1_arr+24)));
    
    
   
 
     //printf("\n **RX PACKET received** \n");
  for(ii = 0; ii < ((*(sim1_arr+24)+16)) ; ii++)
  {
    if(ii && ((ii % 16 ) == 0) )
    {
      //printf("\n");
    }
    //printf(" 0x%.2x ",(sim1_arr+24)[ii]); 
  }
  //printf("\n ");
  free(sim1_arr);
     
    if(rsp_len < 0)
    {
      perror("recv");
      return NULL;
    }
    pthread_mutex_lock(&rsi_linux_app_cb.mutex1);
    rsi_enqueue_to_rcv_q(rsi_linux_app_cb.rcvPktPtr);
    if (sem_post(&sem_cmd_rsp) == -1)
		{
			printf("\nFailed to post packet"); 
		}
		if (sem_post(&sem_cmd_wait) == -1)
		{
			printf("\nFailed to post packet"); 
		}


    //rsi_enqueue_pkt(&rsi_linux_app_cb.rcv_queue, rsi_linux_app_cb.rcvPktPtr);//reyaz
    //rsi_set_event(RSI_RX_EVENT);//reyaz
       pthread_mutex_unlock(&rsi_linux_app_cb.mutex1);
         }
}
  


void rsi_queues_init(pkt_queue_t *queue)
{
	//! Initialize head pointer to NULL
	queue->head = NULL;

	//! Initialize tail pointer to NULL
	queue->tail = NULL;

	//! Initialize pending packet count to zero
	queue->pending_pkt_count = 0;

}


extern void *response_thread_handler(void* args);
/*==============================================*/
/**
 * @fn          void rsi_platform_based_init(void)
 * @brief       initialises the mutex for uart packet handlig and 
 *              creates thread to receive the packets from module
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @section description 
 * initialises the mutex for uart packet handlig and 
 * creates thread to receive the packets from module
 *
 */
pthread_t     thread1;
pthread_t     response_handler_thread;

void rsi_platform_based_init(void)
{
	int err;
	
	//! Mutex initialisation
	if (pthread_mutex_init(&rsi_linux_app_cb.mutex1, NULL) != 0)
	{
		rsi_os_printf(RSI_PL1,"\n ERROR occured in Mutex initialisation \n");
	}
	//! create the thread for reading data coming on UART interface
		if ((err = pthread_create(&response_handler_thread, NULL, &response_thread_handler, NULL)))
	{
		rsi_os_printf(RSI_PL1,"\n ERROR occured in thread creation\n");
	}
  
}
void rsi_os_printf(int mask,char *fmt,...)
{
	if (enable_dut_debug)
	{
		if ( debug_enable_mask & mask )
		{
			va_list args;
			va_start(args, fmt);
			vsprintf(temp_buffer, fmt, args);
			va_end(args);
			printf(temp_buffer);
			return;
		}
	}
}


