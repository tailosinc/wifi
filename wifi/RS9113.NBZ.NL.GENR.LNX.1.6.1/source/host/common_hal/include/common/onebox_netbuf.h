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

#ifndef __ONEBOX_NETBUF_H__
#define __ONEBOX_NETBUF_H__

typedef struct netbuf_ctrl_block_s 
{
	struct netbuf_ctrl_block_s *next; // next ptr
	uint8_t                    *data;          // Data address
	void                       *head;          // head address
	void                       *dev;
	void                       *pkt_addr;
	void                       *ni;                   //store the node reference in ni 
	uint16_t                    len;            // length of the packet
	uint16_t                    aggr_len;       // Total aggregation length
	uint8_t                     mac_hdr_len;    // MAC header length(80211)
	uint8_t                     hdr_pad_len;    // Header padding length
	uint8_t                     sta_id;         // Station id
	uint8_t                     tid;            // Traffic identifier
	uint16_t                     flags;          // Flags
	uint8_t                     aggr_flag;      // Aggregation flags
	uint8_t                     aggr_pcnt;      // No. of AMPDU aggregated packets
	uint8_t                     agg_retry;      // Is it a retry
	uint8_t                     retry_count;    // No. of times this packet retried
	uint8_t                     priority;      
	uint8_t                     skb_priority;
	uint8_t          	    bt_pkt_type;
	uint8_t          	    zigb_pkt_type;
	uint8_t          	    tx_pkt_type;
	uint8_t          	    rx_pkt_type;
	uint8_t                     vap[6];        // Pointer to vap structure reserved
	/* Removed frag_flags, need to use flags in case required */
}netbuf_ctrl_block_t;

#endif
