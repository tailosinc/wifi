/**
 * @file     rsi_serial.h
 * @version  1.0
 * @date     2014-Sep-6
 *
 * Copyright(C) Redpine Signals 2014
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief HEADER  
 *
 * @section Description
 * 
 *
 */


#ifndef __RSI_SERIAL_H_
#define __RSI_SERIAL_H_

//#include "rsi_defines.h"
#include "rsi_common_types.h"
#include "platform_specific.h"
//#include "rsi_global.h"



int16_t rsi_serial_frame_write(uint8_t *packet_buffer, int16_t length);
int16_t rsi_serial_init(uint8_t port);
void rsi_enqueue_to_rcv_q(rsi_pkt_t *pkt);
void * recvThread(void * arg );

#endif //__RSI_SERIAL_H_
