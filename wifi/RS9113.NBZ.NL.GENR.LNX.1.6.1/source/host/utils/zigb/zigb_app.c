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

#include "zigb_app.h"

#define SUCCESS 1

#define ONEBOX_ZIGB_SEND        SIOCIWLASTPRIV - 0x1  //: check for exact value b4 execution
#define ONEBOX_ZIGB_RECV        SIOCIWLASTPRIV - 0x2  //: check for exact value b4 execution


struct zigb_priv {
	char **argv;
	int argc;
	int socket_fd;
	char if_name[IFNAMSIZ];
	struct iwreq   iwr;
	struct iw_freq ifrq;
	char buf[200];
} zigb_priv;

int build_connection(struct zigb_priv *zb_priv)
{
	int s = SUCCESS;
	int sock_fd = 0;

	sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
	if (sock_fd < 0) {
		perror("ZIGB : socket connection - ");
		return -1;
	}
	perror("ZIGB : socket connection - ");

	zb_priv->socket_fd = sock_fd;
	strncpy(zb_priv->iwr.ifr_name,"zigb0",IFNAMSIZ);
	zb_priv->iwr.u.data.pointer = &zb_priv->buf[0];
	
	return  s;
}

int main(int argc, char *argv[])
{
	int i, s;

	zigb_priv.argv = argv;
	zigb_priv.argc = argc;

	strcpy(zigb_priv.if_name, "zigb0");

	s = build_connection(&zigb_priv);
	if (s < 0) {
		printf("ZIGB : Unable to establish connection\n ");
		return 0;
	}
	printf("ZIGB : connection is established\n");

	/*
	 * TX data
	 */ 
	zigb_priv.iwr.u.data.length = 50;
	
	for (i = 0; i < 50; i++) 
		zigb_priv.buf[i] = i;
#if 0
	i =1;
	while(i++)
#else
	i =500000;
	while(i--)
#endif
	{
		printf("**Iteration %d\n",i);
		s = ioctl(zigb_priv.socket_fd, ONEBOX_ZIGB_SEND, &zigb_priv.iwr);
		if (s < 0) {
			perror("ZIGB : IOCTL status -");
			//continue;
			return s;
		}
#if 0
		if(sleep(1)!= 0)
		{
			printf("unable to sleep\n");
			return 0;
		}
#endif
	}
#if 0
	s = ioctl(zigb_priv.socket_fd, ONEBOX_ZIGB_RECV, &zigb_priv.iwr);
	if (s < 0) {
		perror("ZIGB : IOCTL status -");
		return s;
	}

	if (zigb_priv.iwr.u.data.flags == 1)
		printf("ZIGB : More packets are pending the HAL ZIBG QUEUE\n");
	else
		printf("ZIGB : No More pending packets \n");
		
	
	for (i = 0; i < 50; i++) {
		printf("%x ", zigb_priv.buf[i]);
		if (!(i%16))
			printf("\n");
	}
	printf("\n");
#endif	
	return 0;
}
