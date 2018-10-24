/**
 * @file         rsi_app_util.h
 * @version      3.1
 * @date         2012-Dec-21
 *
 * Copyright(C) Redpine Signals 2012
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief HEADER UTIL: Util Header file, the things that are useful for application 
 *
 * @section Description
 * This is the rsi_app_util.h file for the utility functions used by applications that
 * are using this library.
 *
 *
 */


#ifndef _RSIAPPUTIL_H_
#define _RSIAPPUTIL_H_
#include "rsi_global.h"

void   rsi_strcat(void *src, void *dst, uint8 len);
uint8  rsi_strrev(void *src, void *dst, uint8 len);
uint16 rsi_bytes2_to_uint16(uint8 *dBuf);
uint32 rsi_bytes4_to_uint32(uint8 *dBuf);
void   rsi_swap_2bytes(uint8 *buffer);
void   rsi_swap_4bytes(uint8 *buffer);
void   rsi_print_n_chars (int8* st, int16 n);
void   rsi_print_uint8_as_binary(uint8 number);
int16  rsi_in_uint8_list(uint8 arg, uint8 *list, uint8 nargs);
int8   *rsi_bytes6_to_ascii_mac_addr(uint8 *hexAddr,uint8 *strBuf);
int8   *rsi_bytes4_to_ascii_dot_addr(uint8 *hexAddr,uint8 *strBuf);
void   rsi_ascii_dot_address_to_4bytes(uint8 *hexAddr, int8 *asciiDotAddress);
void   rsi_ascii_mac_address_to_6bytes(uint8 *hexAddr, int8 *asciiMacAddress);
uint32 rsi_bytes4R_to_uint32(uint8 *dBuf);
uint16 rsi_bytes2R_to_uint16(uint8 *dBuf);
int8   asciihex_2_num(int8 ascii_hex_in);
int8   rsi_charHex_2_Dec ( int8 *cBuf);
void   register_socket_protocol(void);
uint16 rsi_aToi(uint8 *src, uint8 src_len);
int16  rsi_wifi_init(void);
int8   rsi_charhex_2_dec ( int8 *cBuf);
void   unregister_socket_protocol(void);
uint16 rs22_aToi(uint8 *src, uint8 src_len);
uint16 rsi_convert_ip_to_string(uint8 *num_buff, uint8 *ip_buff);
uint32 parseHex(uint8 *str );
void   parse_ipv6_address(uint8 *v6, uint8 *buf);
uint8  convert_lower_case_to_upper_case(uint8 lwrcase);
void rsi_uint32_to_4bytes(uint8 *dBuf, uint32 val);
void rsi_uint16_to_2bytes(uint8 *dBuf, uint16 val);
void string2array(uint8 *dst, uint8 *src, uint32 length);
#endif
