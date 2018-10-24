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

/**
 * Includes
 */
#include "../rsi_nl_app.h"

#define FRAME_DESC_LEN 16

static struct rsi_nlpriv *nlpriv;
/**
 * @fn          int32 rsi_sendto_fd(int32 s, const uint8 *buf, int32 bufLen)
 * @brief       Sends the netlink message to Kernel
 * @param[in]   int32 s, socket descriptor
 * @param[in]   const uint8 *buf, message buffer to send
 * @param[in]   int32 bufLen, Length of message buffer
 * @param[out]  none
 * @return      errCode
 *              -1 = ERROR
 *              0  = SUCCESS
 * @section description
 * This API is used to send the netlink message to Kernel.
 */
int32 rsi_sendto_fd(int32 s, const uint8 *buf, int32 bufLen)
{
	struct sockaddr_nl nladdr;
	int32 r;

	memset(&nladdr, 0, sizeof(nladdr));
	nladdr.nl_family = AF_NETLINK;

	while ((r = sendto(s, buf, bufLen, 0, (struct sockaddr *)&nladdr,
			   sizeof(nladdr))) < bufLen) {
		if (r > 0) {
			buf += r;
			bufLen -= r;
		}
		else if (errno != EAGAIN)
			return -1;
	}

	return 0;
}


/**
 * @fn          int32 rsi_netlink_init(void)
 * @brief       initialize the netlink sockets
 * @param[in]   void
 * @return      
 *              0  = SUCCESS
 *
 * @section description
 * This API is used to initialize the netlink sockets.
 */
int rsi_netlink_init(void)
{
	int32 rc = -1;
	struct sockaddr_nl local;

	nlpriv = rsi_malloc(sizeof(*nlpriv));
	if (!nlpriv) {
		rc = -ENOMEM;
		goto err;
	}

	memset(nlpriv, 0, sizeof(*nlpriv));

	nlpriv->fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_GENERIC);
	if (nlpriv->fd < 0) {
		perror("Socket");
		goto err_sock;
	}

	memset(&local, 0, sizeof(local));
	local.nl_family = AF_NETLINK;
	local.nl_groups = 0;

	rc = bind(nlpriv->fd, (struct sockaddr *) &local, sizeof(local));
	if (rc < 0) {
		perror("Bind");
		goto err_bind;
	}

	nlpriv->family_id = rsi_get_family_id(nlpriv->fd);
	if (nlpriv->family_id <= 0) {
		printf("netlinkinit: Invalid family id %d\n", nlpriv->family_id);
		rc = -1;
		goto err_bind;
	}

//	printf("netlinkinit: netlink init success "
//	       "fd %d family_id %d\n",
//	       nlpriv->fd, nlpriv->family_id);

	return  0;

err_bind:
	close(nlpriv->fd);
err_sock:
	free(nlpriv);
	nlpriv = NULL;
err:
	return rc;
}

/**
 * @fn          int32 rsi_netlink_deinit(void)
 * @brief       deinitialize the netlink sockets
 * @param[in]   void
 * @return      0  = SUCCESS
 *
 * @section description
 * This API is used to deinitialize a netlink socket.
 */
int rsi_netlink_deinit(void)
{
	if (!nlpriv)
		return 0;

	close(nlpriv->fd);
	rsi_free(nlpriv);
	nlpriv = NULL;
	return 0;

}

uint8 *rsi_add_desc_to_data(uint8 *buf, uint32 type, uint32 len)
{
	int32 rc;
	uint8 *desc, *data, *packet;

	packet = rsi_malloc(len + FRAME_DESC_LEN);
	if (!packet)
		return NULL;

	desc = packet;
	data = packet + FRAME_DESC_LEN;

	/*
	 * add to frame descriptor
	 */
	memset(desc, 0, FRAME_DESC_LEN);

	*(uint16 *)&desc[0] = type;
	*(uint16 *)&desc[2] = len;

	/*
	 * copy the data
	 */
	rsi_memcpy(&data[0], &buf[0], len);

	return packet;
}

/**
 * @fn          int rsi_send_to_drv(uint8 *buf, uint32 type, uint32 len)
 * @brief       Sends the netlink message to Kernel
 * @param[in]   uint8 *buf, which points to the payload.
 * @param[in]   uint32 type, type of the packet
 * @param[in]   uint32 len, Length of message buffer
 * @param[out]  none
 * @return      errCode
 *              -1 = ERROR
 *              0  = SUCCESS
 * @section description
 * This API is used to send the netlink message to Kernel.
 */
int rsi_send_to_drv(uint8 *buf, uint32 type, uint32 len)
{
	int rc, l;
	rsi_nlPkt_t rq;
	struct nlattr *na = NULL;
	int fid, sd;
	char *pkt;

	if (!nlpriv)
		return -1;

	fid = nlpriv->family_id;
	sd = nlpriv->fd;
	
	memset(&rq, 0, sizeof(rsi_nlPkt_t));

	rq.n.nlmsg_type  = fid; 
	rq.n.nlmsg_flags = NLM_F_REQUEST;
	rq.n.nlmsg_seq   = 0; 
	rq.n.nlmsg_pid   = getpid();
	rq.n.nlmsg_len   = NLMSG_LENGTH(GENL_HDRLEN);
	rq.g.cmd = 1;

	len &= 0x0FFF;
	type &= 0x00FF;

	pkt = rsi_add_desc_to_data(buf, type, len);	
	if (!pkt)
		return -ENOMEM;

	len += FRAME_DESC_LEN;
	
	na = (struct nlattr *)GENLMSG_DATA(&rq);
	na->nla_type = 1;
	na->nla_len  = len + 1 + NLA_HDRLEN;

	rsi_memcpy((char *)NLA_DATA(na), pkt, len);
	
	rq.n.nlmsg_len += NLMSG_ALIGN(na->nla_len);

	rc = rsi_sendto_fd(sd, (char *)&rq, rq.n.nlmsg_len);
	if (rc < 0)
		goto send_err;
	
  free(pkt);
	return rc;

send_err:
	free(pkt);
	
	return rc;
}

/**
 * @fn          int rsi_parse_rcv_frame(uint8 *b, uint32 *t, uint32 *l)
 * @brief       parses the received frame for type and length
 * @param[in]   uint8  *buf, which points to the frame descriptor.
 * @param[out]  uint32 *type, type of the packet
 * @param[out]  uint32 *len, Length of message buffer
 * @return      length of the received pkt
 *
 * @section description
 * This API is used to get the length and type of 
 * the packet from its frame descriptor.
 */
int rsi_parse_rcv_frame(uint8 *buf, uint32 *type, uint32 *len)
{
#define FD_PACKET_LEN_INDEX   0   /* "Length" in frame descriptor */
#define FD_PACKET_TYP_INDEX   14  /* "PacketType" in Frame Descriptor */
	uint8 *fd = buf;

	*len  = (uint32)(*(uint16 *)&fd[FD_PACKET_LEN_INDEX] & 0x0FFF);
	*type = (uint32)(*(uint16 *)&fd[FD_PACKET_TYP_INDEX] & 0x000F);

//  printf("******* RX len = %d type = %d\n", *len, *type);

	return *len;
}

/**
 * @fn          int rsi_recv_from_drv(uint8 *buf, uint32 *type, uint32 *len)
 * @brief       receives a message from the netlink socket.
 * @param[out]  uint8  *buf, which points valid writable location of MAX_PACKET_LEN size.
 * @param[out]  uint32 *type,type of the packet will be updated.
 * @param[out]  uint32 *len, Length of message buffer will be update.
 * @return      length of the received pkt
 *
 * @section description
 * This API is used receive a packet from the netlink socket
 */
int recv_data(uint8 *nl_buffer)
{
  uint32 nl_type = 0;
  uint32 nl_len = 0;

	int32 rc = 0;

	rc = rsi_recv_from_drv(nl_buffer, &nl_type, &nl_len);

	if( rc < 0 )
	{
		printf("Unable to recv\n");
		return rc;
	}
	return rc;
}
int rsi_recv_from_drv(uint8 *buf, uint32 *type, uint32 *len)
{
	int rc, fd, l;
	uint8 *b = NULL;
	struct nlattr *na = NULL;
	rsi_nlPkt_t rs;

	if (!nlpriv)
		return -1;

	if (!buf) {
		rc = -ENOMEM;
		goto err;
	}

	fd = nlpriv->fd;


	memset(&rs, 0, sizeof(rs));
	l = recv(fd, &rs, sizeof(rs), 0);
	if (l < 0) {
		perror("Recv");
		rc = -1;
		goto err;
	}

	if (rs.n.nlmsg_type == NLMSG_ERROR ||
	    !NLMSG_OK((&rs.n), l)) {
		struct nlmsgerr *err = NLMSG_DATA(&rs);
		//printf("recv_drv: fatal reply error, errno %d\n",
	  //		err->error);
		rc = err->error;
		goto err;
	}

	na = (struct nlattr *)GENLMSG_DATA(&rs);
	l = GENLMSG_PAYLOAD(&rs.n);
	b = (char *)NLA_DATA(na);

#if 0
  int i;
  printf("Printing RX package len =%d\n", l);
  for (i = 0; i < 100 ; i++) {
    if (!(i % 16))
      printf("\n");
    printf(" 0x%X ", b[i]);
  }
  printf("\n");
#endif

//	l = rsi_parse_rcv_frame(b, type, len);

	/*
         * Skip the FD, send only the Payload
         */ 
	memcpy(buf, b + FRAME_DESC_LEN, l);
	//memcpy(buf, b, l);

	return 0;

err:
	return rc;
}

/**
 * @fn          int rsi_get_family_id(int sd)
 * @brief       Gets the family id of netlink socket.
 * @param[in]   int sd, socket descriptor of netlink socket
 * @return      family_id
 *
 * @section description
 * This API is used to get the family id of a netlink socket
 */
int rsi_get_family_id(int sd)
{
	int fid = 0, rc, l;
	uint8 *req_buff = NULL, *rsp_buff = NULL;
	rsi_nlPkt_t *request = NULL, *response = NULL;
	struct nlattr *na = NULL;
//	uint8 family_name[25] = "CTRL_PKT_TXRX";
	uint8 family_name[25] = "Obx-BTgenl";

	req_buff = rsi_malloc(NLMSG_LENGTH(GENL_HDRLEN) +
			      NLMSG_ALIGN(strlen((char*)family_name) +
                                          1 +
                                          NLA_HDRLEN));
	if (!req_buff) {
		rc = -ENOMEM;
		goto err;
	}

	/*
         * Resolve the family ID
         */
	request = (rsi_nlPkt_t *)req_buff;
	request->n.nlmsg_type  = GENL_ID_CTRL;
	request->n.nlmsg_flags = NLM_F_REQUEST;
	request->n.nlmsg_seq   = 0;
	request->n.nlmsg_pid   = getpid();
	request->n.nlmsg_len   = NLMSG_LENGTH(GENL_HDRLEN);
	request->g.cmd         = CTRL_CMD_GETFAMILY;
	request->g.version     = 0x1;

	na = (struct nlattr *)GENLMSG_DATA(req_buff);
	na->nla_type = CTRL_ATTR_FAMILY_NAME;
	na->nla_len  = strlen((char*)family_name) + 1 + NLA_HDRLEN;
	strcpy((char*)NLA_DATA(na),(char*)family_name);

	request->n.nlmsg_len += NLMSG_ALIGN(na->nla_len);

	rc = rsi_sendto_fd(sd, req_buff, request->n.nlmsg_len);
	if (rc < 0) {
		printf("getfamilyid: sendto failed(%d)", rc);
		goto err1;
	}

	rsp_buff = rsi_malloc(MAX_RCV_SIZE);
	if (!rsp_buff) {
		rc = -ENOMEM;
		goto err1;
	}

	response = (rsi_nlPkt_t *)rsp_buff;

	l = recv(sd, response, MAX_RCV_SIZE, 0);
	if (l < 0) {
		perror("Recv");
		rc = -1;
		goto err2;
	}

	if (!NLMSG_OK((&response->n), l)) {
		printf("genlfamilyid: Invalid reply message\n");
		rc = -1;
		goto err2;
	}

	if (response->n.nlmsg_type == NLMSG_ERROR) {
		struct nlmsgerr *err = NLMSG_DATA(&response);
		printf("genlfamilyid: fatal reply error, errno %d\n", err->error);
		rc = err->error;
		goto err2;
	}

	na = (struct nlattr *)GENLMSG_DATA(response);
	na = (struct nlattr *)((char *) na + NLA_ALIGN(na->nla_len));
	if (na->nla_type == CTRL_ATTR_FAMILY_ID)
		fid = *(__u16 *) NLA_DATA(na);

	rsi_free(req_buff);
	rsi_free(rsp_buff);

	return fid;

err2:
	rsi_free(rsp_buff);
err1:
	rsi_free(req_buff);
err:
	return -1;
}
