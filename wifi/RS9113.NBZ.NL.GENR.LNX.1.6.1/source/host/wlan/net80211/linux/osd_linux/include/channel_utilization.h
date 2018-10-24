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

#ifndef _CHANNEL_UTILIZATION_
#define _CHANNEL_UTILIZATION_


void channel_utilization(struct bpf_if *vap,int rssi_of_pkt,int length_of_pkt,int data_rate,char ch_bw,char crc_pkt);

/**      
 * @brief   compute duration for a tx packet
 * @param   rate  tx rate of packet
 * @param   length  tx packet length
 * @param   ch_bw current cahnnel band width 
 * @param   spoof_len spoof length calculated is filled here
 * @return  duration
 */

uint16_t get_duration(uint16_t rate, uint16_t length, uint8_t ch_bw, uint16_t *spoof_len);
#define DOT11AC_RATE 3
#define DOT11N_RATE  2
#define DOT11G_RATE  1
#define DOT11B_RATE  0

uint16_t ndbps_11ac[][4] = {
	/* 20MHz 40MHz    80MHz   160MHz*/
	{    26,    54,   117,    234 }, /*  0: BPSK */
	{    52,   108,   234,    468 }, /*  1: QPSK 1/2 */
	{    78,   162,   351,    702 }, /*  2: QPSK 3/4 */
	{   104,   216,   468,    936 }, /*  3: 16-QAM 1/2 */
	{   156,   324,   702,   1404 }, /*  4: 16-QAM 3/4 */
	{   208,   432,   936,   1872 }, /*  5: 64-QAM 2/3 */
	{   234,   486,  1053,   2106 }, /*  6: 64-QAM 3/4 */
	{   260,   540,  1170,   2340 }, /*  7: 64-QAM 5/6 */
	{   312,   648,  1404,   2808 }, /*  8: BPSK */
	{     0,   720,  1560,   3120 }, /*  9: QPSK 1/2 */
};

/* TX VECTOR CONTROL INFO DEFINES */
#define SHORT_GI_MODE       BIT(9)

#define HALF_GI_SYMBOL_TIME(_x)     (((_x) * 9 + 9) / 10 )                  

#define L_STF                       8
#define L_LTF                       8
#define L_SIG                       4
#define VHT_SIG_A                   8
#define VHT_STF                     4
#define VHT_LTF                     4
#define VHT_SIG_B                   4
#define HT_SIG                      8
#define HT_STF                      4
#define HT_GF_STF                   8
#define HT_LTF1                     8
#define HT_LTF(_ns)                 (4 * (_ns))


#define PREAMBLE_11AC(_nltfs)       (L_STF + L_LTF + L_SIG + VHT_SIG_A + VHT_STF + VHT_SIG_B + ((_nltfs) * VHT_LTF))
#define HT_RC_2_MCS(_rc)            ((_rc) & 0x0f)
#define OFDM_PLCP_BITS              22
#define NES_PLCP_BITS               6

const uint16_t bits_per_symbol[][2] = {
  //! 20MHz 40MHz 
  {    26,    54 }, /*  0: BPSK */
  {    52,   108 }, /*  1: QPSK 1/2 */
  {    78,   162 }, /*  2: QPSK 3/4 */
  {   104,   216 }, /*  3: 16-QAM 1/2 */
  {   156,   324 }, /*  4: 16-QAM 3/4 */
  {   208,   432 }, /*  5: 64-QAM 2/3 */
  {   234,   486 }, /*  6: 64-QAM 3/4 */
  {   260,   540 }, /*  7: 64-QAM 5/6 */
};

#define GET_NLTFS(_rate)    ((_rate >> 11) & 0x3)
#define CHK_STBC_EN(_rate)  (_rate & BIT(13))
#define GREENFIELD          (short)BIT(7) 


const char rates_11g [] = {   /* index = rate_code - 8 */
  48,  /* rate_code = 8 */
  24,  /* rate_code = 9 */
  12,  /* rate_code = a */
  6,   /* rate_code = b */
  54,  /* rate_code = c */
  36,  /* rate_code = d */
  18,  /* rate_code = e */
  9    /* rate_code = f */
};
#define OFDM_SYMBOL_TIME            4
#define OFDM_PLCP_BITS              22
#define OFDM_PREAMBLE_TIME          20
#define OFDM_SYMBOL_TIME            4


const char rates_11b [] = {  /* index = rate_code */
  1,  /* rate_code = 0 */
  0,  /* rate_code = 1 */
  2,  /* rate_code = 2 */
  0,  /* rate_code = 3 */
  11, /* rate_code = 4 */
  0,  /* rate_code = 5 */
  11  /* rate_code = 6 */
};
#define SHORT_PREAMBLE      BIT(9)
#define CCK_PREAMBLE_BITS           144
#define CCK_PLCP_BITS               48

#endif
