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

#include "onebox_datatypes.h"


static uint32 checksum_addition(uint8 *buf, uint32 size, uint32 prev_sum)
{
	uint32 sum = prev_sum;
	uint32 cnt;
	uint32 cnt_limit;
	uint32 dword;

	if (size == 0)
	{
 		return sum;
	}

	cnt_limit = (size & (~0x3));
	/* Accumulate checksum */
	for (cnt = 0; cnt < cnt_limit; cnt += 4)
	{
		dword = *(uint32 *) &buf[cnt];
		sum += dword;
		if(sum < dword)
		{
			/* In addition operation, if result is lesser than any one of the operand
			 * it means carry is generated. 
			 * Incrementing the sum to get ones compliment addition */

			sum++;
		}
	}


	/* Handle non dword-sized case */
  if(size & 0x3) {
		dword = 0xffffffff;
		dword = ~(dword << (8 * (size & 0x3)));
		/* Keeping only valid bytes and making upper bytes zeroes. */
		dword = (*(uint32 *) &buf[cnt]) & dword;
		sum += dword;
		if(sum < dword)
		{
			sum++;
		}
	}

	return sum;
}

uint32 checksum_32bit(uint8 **scatter_addr, uint32 *scatter_len, uint32 no_of_scatters)
{
	uint32 sum = 0;
	uint32 cnt;
	uint8 *buf;
	uint32 size;

	for (cnt = 0; cnt < no_of_scatters; cnt++)
	{
		buf  = scatter_addr[cnt];
		size = scatter_len[cnt];
		sum = checksum_addition(buf, size, sum);
	}
	
	/* Invert to get the negative in ones-complement arithmetic */
	return ~sum;
}

