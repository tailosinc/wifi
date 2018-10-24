/**
 * @file        rsi_uart.c
 * Copyright(C) 2013 Redpine Signals Inc.
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * T:vshis program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief MAIN, Top level file, it all starts here
 *
 * @section Description
 * This file contains API's to write and read to serial device 
 * It also has RecvThread and serial device initialization API's too 
 * 
 */



#include <sys/time.h>
#include <stdio.h>
#include <semaphore.h>

#include "rsi_serial.h"
#include "rsi_common_types.h"
#include "platform_specific.h"
#include "rsi_zigb_global.h"
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdarg.h>


#define BAUDRATE B115200
#define RSI_PRE_DESC_LEN 4
//! Skip card ready if in UART mode
#define RSI_SKIP_CARD_READY      0

char *serial_device[] = {"/dev/ttyUSB0", "/dev/ttyUSB1","/dev/ttyACM0","/dev/ttyACM1"};
extern sem_t  sem_cmd_rsp,sem_cmd_wait;



uint8_t card_ready_frame[16] = { 0x0, 0x40, 0x89, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
int rsi_uart_read_term(uint8_t *buf, uint16_t len);
int rsi_uart_dummy_read(uint16_t len);
int rsi_uart_read(uint8_t *buf, uint16_t len);
struct termios tio;
uint8_t rsi_uart_byte_read(void);
extern rsi_linux_app_cb_t   rsi_linux_app_cb;
extern uint16_t rsi_bytes2R_to_uint16(uint8_t *dBuf);
extern int enable_dut_debug;
extern int debug_enable_mask;
extern char temp_buffer[1000];

unsigned long Packet_Sent=0;
unsigned long Packet_Received=0;
unsigned long unidirectional_packet_Sent=0;
/*==============================================*/
/**
 * @fn          int16_t rsi_serial_frame_write(uint8_t *packet_buffer, int16_t length)
 * @brief       frame writing api 
 * @param[in]   packet buffer that has to be written on to the socket
 * @return      errCode
 *              0  = SUCCESS
 *              else failure
 */
int16_t rsi_serial_frame_write(uint8_t *packet_buffer, int16_t length)
{
	int16_t retval = 0;
	uint16_t ii = 0;
	struct timespec ts;
	char temp_buffer[200];

	clock_gettime(CLOCK_REALTIME, &ts);
	

	if (length ==82)
	{
		unidirectional_packet_Sent++;
		sprintf(temp_buffer,"UDP TX PACKET ID:%d",unidirectional_packet_Sent);
		DebugDumpBytes(packet_buffer,length,temp_buffer);
	}
	else
	{
		Packet_Sent++;
		sprintf(temp_buffer,"Management TX PACKET [%d:%d]",Packet_Sent,Packet_Received);
		DebugDumpBytes(packet_buffer,length,temp_buffer);
	}
	retval = write(rsi_linux_app_cb.ttyfd, packet_buffer, length);
	/* Return success */
	return retval;
}
int send_serial_data(uint8_t *packet_buffer, int16_t length)
{
	int16_t retval = 0;
	retval = write(rsi_linux_app_cb.ttyfd, packet_buffer, length);
	return retval;
}

/*==============================================*/
/**
 * @fn          void rsi_serial_init(void)
 * @brief       Serial device initialization
 * @param[out]  int16_t
 * @return      errCode
 *              0  = SUCCESS
 *              else failure
 * @section description
 * This is to initialize serial device for communicating
 * over UART/USB-serial.
 */
int fd;
int16_t rsi_serial_init(uint8_t port)
{
#ifndef WINDOWS
	struct termios newtio;
	char port_name[128]; 
	strcpy(port_name,serial_device[port]);
#endif
#ifdef WINDOWS
	COMMTIMEOUTS timeouts;
	DCB port;
	DWORD mode;
	char port_name[128] = serial_device[port];
	//! open the comm port.
	fd = CreateFile(port_name,
					GENERIC_READ | GENERIC_WRITE,
					0, 
					NULL, 
					OPEN_EXISTING,
					FILE_ATTRIBUTE_NORMAL,
					NULL);

	if ( INVALID_HANDLE_VALUE == fd)
	{
		system_error("opening file\n");
		return 1;
	}
	//! get the current DCB, and adjust a few bits to our liking.
	memset(&port, 0, sizeof(port));
	port.DCBlength = sizeof(port);
	if ( !GetCommState(fd, &port))
		system_error("getting comm state");
	if (!BuildCommDCB("baud=115200 parity=n data=8 stop=1", &port))
		system_error("building comm DCB");
	if (!SetCommState(fd, &port))
		system_error("adjusting port settings");

	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutMultiplier = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 50;
	timeouts.WriteTotalTimeoutConstant = 50;
	if (!SetCommTimeouts(fd, &timeouts))
		system_error("setting port time-outs.");
	rsi_linux_app_cb.ttyfd = fd;
	return 0;
#else
	//! Open modem device for reading and writing and not as controlling tty
	//! because we don't want to get killed if linenoise sends CTRL-C. 
	fd = open(port_name, O_RDWR/* | O_NOCTTY */); 

	if (fd <0)
	{
		//open("/dev/ttyACM1", O_RDWR/* | O_NOCTTY */); 
		
		perror(port_name);
		printf("\nProvide values \n0 - ttyUSB0\t1 - ttyUSB1\t2 - ttyACM0\t3 - ttyACM1\n"); 
		exit(-1); 
	}
	/*else
	{
	  perror(port_name); 
	  exit(-1); 
  
	}*/

	//! clear struct for new port settings 
	bzero(&newtio, sizeof(newtio)); 

	//! BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	//! CRTSCTS : output hardware flow control (only used if the cable has
	//!  all necessary lines. See sect. 7 of Serial-HOWTO)
	//!  CS8     : 8n1 (8bit,no parity,1 stopbit)
	//!  CLOCAL  : local connection, no modem contol
	//!  CREAD   : enable receiving characters
#if UART_HW_FLOW_CONTROL
	newtio.c_cflag = BAUDRATE  | CS8 | CLOCAL | CREAD| CRTSCTS;
#else
	newtio.c_cflag = BAUDRATE  | CS8 | CLOCAL | CREAD;
#endif

	//! IGNPAR  : ignore bytes with parity errors
	//! ICRNL   : map CR to NL (otherwise a CR input on the other computer
	//! will not terminate input)
	//! otherwise make device raw (no other input processing)

	newtio.c_iflag = IGNPAR;// | ICRNL;

	//! Raw output 
	newtio.c_oflag = 0;

	//! ICANON  : enable canonical input
	//! disable all echo functionality, and don't send signals to calling program

	newtio.c_lflag = 0;

	//! initialize all control characters 
	//! default values can be found in /usr/include/termios.h, and are given
	//! in the comments, but we don't need them here
	newtio.c_cc[VINTR]    = 0;	  //! Ctrl-c  
	newtio.c_cc[VQUIT]    = 0;	  //! Ctrl- 
	newtio.c_cc[VERASE]   = 0;	  //! del 
	newtio.c_cc[VKILL]    = 0;	  //! @ 
	newtio.c_cc[VEOF]     = 4;	  //! Ctrl-d 
	newtio.c_cc[VTIME]    = 0;	  //! inter-character timer unused 
	newtio.c_cc[VMIN]     = 1;	  //! blocking read until 1 character arrives 
	newtio.c_cc[VSWTC]    = 0;	  //! '\0' 
	newtio.c_cc[VSTART]   = 0;	  //! Ctrl-q  
	newtio.c_cc[VSTOP]    = 0;	  //! Ctrl-s 
	newtio.c_cc[VSUSP]    = 0;	  //! Ctrl-z 
	newtio.c_cc[VEOL]     = 0;	  //! '\0' 
	newtio.c_cc[VREPRINT] = 0;	  //! Ctrl-r 
	newtio.c_cc[VDISCARD] = 0;	  //! Ctrl-u 
	newtio.c_cc[VWERASE]  = 0;	  //! Ctrl-w 
	newtio.c_cc[VLNEXT]   = 0;	  //! Ctrl-v 
	newtio.c_cc[VEOL2]    = 0;	  //! '\0' 

	//! now clean the modem line and activate the settings for the port
	tcflush(fd, TCIFLUSH);

	//! set the current attributes
	tcsetattr(fd,TCSANOW,&newtio);

	//! assign the file descriptor to global variable
	rsi_linux_app_cb.ttyfd = fd;
	return 0;

#endif
}
/*==============================================*/
/**
 *
 * @fn         void rsi_queues_init(rsi_queue_cb_t *queue)
 * @brief      Initializes queue
 * @param[in]  queue , pointer to queue 
 * @param[out] None
 * @return     None
 *
 * @section description
 * This function initializes queue
 *
 *
 */
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
	if ((err = pthread_create(&thread1, NULL, &recvThread, NULL)))
	{
		rsi_os_printf(RSI_PL1,"\n ERROR occured in thread creation\n");
	}
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

/*==============================================*/
/**
 * @fn          void rsi_recv_pkt_serial(void)
 * @brief       Recieve the packet from MCU
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @section description
 * This function will read the number of bytes return by the
 * module 
 * 
 *
 */


/*==============================================*/
/**
 * @fn          void recvThread(void *args)
 * @brief       Recieve the packet from MCU
 * @param[in]   args
 * @return      none
 * @section description
 * This function is the thread which is created to read the
 * data coming from the module on UART interface
 * 
 *
 */

extern int interactive_mode_state;
char temp_buffer[1000];
void *recvThread(void* args)
{
	uint16_t dummy_byte_len=0;
	//int errq=0;

	uint16_t payload_length = 0;
	uint16_t ii = 0;
	rsi_os_printf(RSI_PL1,"\n Thread started \n");
	while (1)
	{
		if(interactive_mode_state==1)
		{
			read(rsi_linux_app_cb.ttyfd, temp_buffer, 1000);
			printf(temp_buffer);
			continue;

		}
		

		if (rsi_linux_app_cb.rx_is_in_progress ==  RSI_UART_IDLE_STATE)
		{
			//! Malloc the packet to receive RX packet
			rsi_linux_app_cb.rcvPktPtr = (rsi_pkt_t *) malloc( 1600 + sizeof(rsi_pkt_t));
			//! If packet is not allocated,return 
			if (rsi_linux_app_cb.rcvPktPtr == NULL)
			{
				rsi_os_printf(RSI_PL1,"\n ERROR : Packet Allocation failed \n");
				return;
			}

			//! Change the state to pre descriptor receive state
			rsi_linux_app_cb.rx_is_in_progress = RSI_UART_LEN_DESC_RECV_STATE;

			//! set the byte count to 0
			rsi_linux_app_cb.byte_count = 0;
		}
		//! Reading first four bytes to calculate packet size and offset
		if (rsi_linux_app_cb.rx_is_in_progress == RSI_UART_LEN_DESC_RECV_STATE)
		{
			//! get each byte of predescriptor
			rsi_linux_app_cb.pre_desc_buf[rsi_linux_app_cb.byte_count++] = rsi_uart_byte_read();

			//! If all the 4 bytes of pre descriptor is received, then change the state to host descriptor receive state
			if (rsi_linux_app_cb.byte_count == RSI_PRE_DESC_LEN)
			{
				rsi_linux_app_cb.rx_is_in_progress = RSI_UART_WAIT_FOR_HOST_DESC_STATE;

				//! calculate the dummy length using pre descriptor
				dummy_byte_len = rsi_bytes2R_to_uint16(&rsi_linux_app_cb.pre_desc_buf[2]) - RSI_PRE_DESC_LEN;
			}
			continue;
		}
		//! Reading dummy Bytes if any
		if (rsi_linux_app_cb.rx_is_in_progress == RSI_UART_WAIT_FOR_HOST_DESC_STATE)
		{
			if (dummy_byte_len)
			{
				//! Read dummy bytes
				rsi_uart_byte_read();
				dummy_byte_len--;
				continue;
			}

			//! handle zero lenght packets
			if (!dummy_byte_len)
			{
				//! Change the status to payload receive state
				rsi_linux_app_cb.rx_is_in_progress = RSI_UART_PAYLOAD_RECV_STATE;
				//! Calculate the payload length
				rsi_linux_app_cb.payload_size = rsi_bytes2R_to_uint16(&rsi_linux_app_cb.pre_desc_buf[0]) - rsi_bytes2R_to_uint16(&rsi_linux_app_cb.pre_desc_buf[2]);

				//! get the payload size from pre descriptor
				payload_length = rsi_linux_app_cb.payload_size;
				//! assign the byte count 
				rsi_linux_app_cb.byte_count = 0;

				//! Handle zero lenght packets
				if (!rsi_linux_app_cb.payload_size)
				{
					//! If payload length received  is zero then don't collect the packet
					//! and free the buffer and revert the state machine to IDLE 
					rsi_linux_app_cb.rx_is_in_progress = RSI_UART_IDLE_STATE;

					//! If zero length packet is received, then free the packet 
					if (rsi_linux_app_cb.rcvPktPtr)
					{
						free(rsi_linux_app_cb.rcvPktPtr);
					}

					//! Make the pointer zero
					rsi_linux_app_cb.rcvPktPtr = NULL;
					return;
				}
			}
		}

		//! If the packet is in payload receive state
		if (rsi_linux_app_cb.rx_is_in_progress == RSI_UART_PAYLOAD_RECV_STATE)
		{

			//! If payload is present
			if (rsi_linux_app_cb.payload_size)
			{

				//! Read the payload bytes
				rsi_linux_app_cb.rcvPktPtr->desc[rsi_linux_app_cb.byte_count++] = rsi_uart_byte_read();

				//! decrement the paylaod size by 1 after reading each byte
				rsi_linux_app_cb.payload_size--;

			}

			//! After reading payload
			if (rsi_linux_app_cb.payload_size == 0)
			{
				//! Make the state to IDLE 
				rsi_linux_app_cb.rx_is_in_progress = RSI_UART_IDLE_STATE;

				//! enqueue packet
				pthread_mutex_lock(&rsi_linux_app_cb.mutex1);
				rsi_enqueue_to_rcv_q(rsi_linux_app_cb.rcvPktPtr);
				pthread_mutex_unlock(&rsi_linux_app_cb.mutex1);
				if (rsi_linux_app_cb.rcvPktPtr)
				{
					//free(rsi_linux_app_cb.rcvPktPtr);
					rsi_linux_app_cb.rcvPktPtr = NULL;
				}
			}
		}
		if (sem_post(&sem_cmd_rsp) == -1)
		{
			printf("\nFailed to post packet"); 
		}
		if (sem_post(&sem_cmd_wait) == -1)
		{
			printf("\nFailed to post packet"); 
		}
	}
	return 0;
}

/*==================================================================*/
/**
 * @fn         uint8_t rsi_uart_byte_read()
 * @param[in]  none
 * @param[out] None
 * @return     read character
 * @description 
 * This API is used to read the byte data from module through the UART interface.
 *
 *
 */

uint8_t rsi_uart_byte_read(void)
{
	uint8_t ch = 0;

	//! read each character 
	read(rsi_linux_app_cb.ttyfd, &ch, 1);

	//! return the read character
	return ch;
}

