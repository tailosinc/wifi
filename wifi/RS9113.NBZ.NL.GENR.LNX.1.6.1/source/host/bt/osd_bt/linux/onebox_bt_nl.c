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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/errno.h>

#include <net/genetlink.h>
#include <net/sock.h>

#include "bt_common.h"
#include "onebox_linux.h"
#include "onebox_coex.h"

static struct genl_cb *global_gcb;
int32 onebox_genlrecv(struct sk_buff *rskb, struct genl_info *info);

#define ONEBOX_BT_GENL_FAMILY "Obx-BTgenl"
			     
#define GET_ADAPTER_FROM_GENLCB(gcb) \
		(BT_ADAPTER)((gcb)->gc_drvpriv)

/* 
 * attribute policy: defines which attribute has 
 * which type (e.g int, char * etc)
 * possible values defined in net/netlink.h
 */
static struct nla_policy bt_genl_policy[RSI_USER_A_MAX + 1] = {
	[RSI_USER_A_MSG] = { .type = NLA_NUL_STRING },
};

/* family definition */
static struct genl_family bt_genl_family = {
	.id      = 0,
	.hdrsize = 0,
	.name    = ONEBOX_BT_GENL_FAMILY,
	.version = RSI_VERSION_NR,
	.maxattr = RSI_USER_A_MAX,
};

static struct genl_ops bt_genl_ops = {
	.cmd    = RSI_USER_C_CMD,
	.flags  = 0,
	.policy = bt_genl_policy,
	.doit   = onebox_genlrecv,
	.dumpit = NULL,
};

/**
 * @fn          onebox_genlrecv
 *              
 * @brief       Gets the command request from user space
 *              over netlink socket
 * @param[in]   struct sk_buff *skb_2, pointer to sk_buff structure
 * @param[in]   struct genl_info *info, read command info pointer
 * @param[out]  none
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 * @section description
 * This API is used to read the command from user over netlink
 * socket.
 */
int32 onebox_genlrecv(struct sk_buff *skb, struct genl_info *info)
{
	uint8 *data;
	int32 rc = -1, len, pkttype;
  int status = 0;
	struct genl_cb *gcb;
	netbuf_ctrl_block_t *netbuf_cb;
	BT_ADAPTER bt_adapter = NULL;
	bb_rf_params_t *bb_rf_params;

	if (!(gcb = global_gcb))
		return -1;

	if (!(bt_adapter = GET_ADAPTER_FROM_GENLCB(global_gcb)))
		return -1;

#if 0
	if( bt_adapter->driver_mode == BT_E2E_MODE_ON )
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Driver mode is NOT RF_EVAL_MODE\n"),
				 __func__)); 
		//return ONEBOX_STATUS_FAILURE;
	}
#endif
	gcb->gc_info = info;
	gcb->gc_skb = skb;

	data = bt_adapter->os_intf_ops->onebox_genl_recv_handle(gcb);
	if (!data) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: genlrecv handler fail on family `%s'\n"),
				 __func__, gcb->gc_name)); 
		goto err;
	}

	gcb->gc_info = NULL;
	gcb->gc_skb  = NULL;

	pkttype = *(uint16 *)&data[0];
	len     = *(uint16 *)&data[2];

	data += 16;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("%s: len %x pkttype %x\n"), __func__, len, pkttype));

	bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO, data, len);
	if( bt_adapter->driver_mode != BT_E2E_MODE_ON ) { 
		bt_adapter->read_cmd = 0;
		bt_adapter->wait_cmd = 0;
		switch( pkttype)
		{
			case BT_CW:
				if( bt_adapter->osi_bt_ops->onebox_bt_cw(bt_adapter, data,len) != ONEBOX_STATUS_SUCCESS )
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
							(TEXT("%s: Unable to perform PER operations.\n"), __func__));
					return ONEBOX_STATUS_FAILURE;
				}
				break;
			case BT_PER:
				bb_rf_params = (bb_rf_params_t *)data;
				switch( bb_rf_params->value)
				{
					case BB_READ:
					case RF_READ:
					case BT_PER_STATS:
					case BUFFER_READ:
						bt_adapter->read_cmd = 1;
						break;
					case BUFFER_WRITE:
#ifndef PER_ENHANCEMENTS
						bt_adapter->wait_cmd = 1;
#endif
						break;
          case GET_DRV_COEX_MODE:
            if( bt_adapter->driver_mode == RF_EVAL_MODE_ON )
            {
              ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" bt_adapter->coex_mode = %d\n"), bt_adapter->coex_mode));
              if ((bt_adapter->oper_mode & (OP_BT_CLASSIC_MODE | OP_BT_LE_MODE)) == OP_BT_CLASSIC_MODE)
                return RF_EVAL_CLASSIC;
              else if((bt_adapter->oper_mode & (OP_BT_CLASSIC_MODE | OP_BT_LE_MODE)) == OP_BT_LE_MODE)
                return RF_EVAL_LE;
              else if ((bt_adapter->oper_mode & (OP_BT_CLASSIC_MODE | OP_BT_LE_MODE)) == OP_BT_DUAL_MODE)
                return RF_EVAL_DUAL_MODE;
              else 
                return ONEBOX_STATUS_FAILURE;
            }
            break;
          case TX_STATUS:
            if( bt_adapter->tx_is_in_progress )
            {
              ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
                  (TEXT("%s:ALREADY IN TRANSMIT SO STOP THE TRASMIT FIRST \n"), __func__)); 
              return TX_IS_IN_PROG;
            }
            ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
                (TEXT("%s:TRANSMIT YET TO BE STARTED \n"), __func__)); 
            return TX_IS_NOT_IN_PROG;
          case BT_PER_TRANSMIT:
          case PER_BR_EDR_TRANSMIT:
          case PER_BLE_TRANSMIT:
            bt_adapter->tx_is_in_progress =bb_rf_params->Data[0] ;
				}
				netbuf_cb = bt_adapter->os_intf_ops->onebox_alloc_skb(len + 
						REQUIRED_HEADROOM_FOR_BT_HAL);
				if (!netbuf_cb) {
					rc = -ENOMEM;
					goto err;
				}

				bt_adapter->os_intf_ops->onebox_reserve_data(netbuf_cb, 
						REQUIRED_HEADROOM_FOR_BT_HAL);

				bt_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);

				bt_adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, data, len);

				netbuf_cb->bt_pkt_type = pkttype; 

				rc = bt_adapter->osi_bt_ops->onebox_bt_xmit(bt_adapter, netbuf_cb);
				if (rc) 
					goto err;
				break;
			case BT_BER:
				if( bt_adapter->osi_bt_ops->onebox_bt_ber(bt_adapter, data,len) != ONEBOX_STATUS_SUCCESS )
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
							(TEXT("%s: Unable to perform PER operations.\n"), __func__));
					return ONEBOX_STATUS_FAILURE;
				}
				break;
			default:
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
						(TEXT("%s: Invalid Pkt type %d\n"), __func__, rc)); 
				return ONEBOX_STATUS_FAILURE;
		}
		if( bt_adapter->read_cmd || bt_adapter->wait_cmd)
		{
			bt_adapter->os_intf_ops->onebox_reset_event(&(bt_adapter->bt_per_event));
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Going to WAIT\n")));
      status = bt_adapter->os_intf_ops->onebox_wait_event(&(bt_adapter->bt_per_event), EVENT_WAIT_FOREVER);
      if(status)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
						(TEXT("%s:NO RESPONSE BEFORE SPECIFIED TIME \n"), __func__)); 
				return ONEBOX_STATUS_FAILURE;
			}
#if 0
			ret_val = copy_to_user(wrq->u.data.pointer, &bt_adapter->bb_rf_params.Data[0], sizeof(bt_adapter->bb_rf_params));
#endif
		}
		return ONEBOX_STATUS_SUCCESS;
	} else {
		netbuf_cb = bt_adapter->os_intf_ops->onebox_alloc_skb(len + 
				REQUIRED_HEADROOM_FOR_BT_HAL);
		if (!netbuf_cb) {
			rc = -ENOMEM;
			goto err;
		}

		bt_adapter->os_intf_ops->onebox_reserve_data(netbuf_cb, 
				REQUIRED_HEADROOM_FOR_BT_HAL);

		bt_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);

		bt_adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, data, len);

		netbuf_cb->bt_pkt_type = pkttype; 

		rc = bt_adapter->osi_bt_ops->onebox_bt_xmit(bt_adapter, netbuf_cb);
		if (rc) 
			goto err;

	}
		return ONEBOX_STATUS_SUCCESS;

err:
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("%s: error(%d) occured \n"), __func__, rc)); 
	return rc;
}

/**
 * @fn          send_pkt_to_btgenl
 *              
 * @brief       Sends a message to the userspace
 *              over netlink socket
 * @param[in]   BT_ADAPTER bt_adapter pointer to sk_buff bt_adapter
 * @param[in]   netbuf_ctrl_block_t *netbuf_cb pointer to the message 
 * 		which has to be sent to the application.
 * @param[out]  none
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 * @section description
 * This API is used to send a command to the user over netlink
 * socket.
 */
int32 send_pkt_to_btgenl(BT_ADAPTER bt_adapter, 
			 netbuf_ctrl_block_t *netbuf_cb)
{
	int32 rc = -1;
	uint8 *data;
	uint32 len;
	struct genl_cb *gcb;

	if (!(gcb = bt_adapter->genl_cb))
		return -EFAULT;

	data = netbuf_cb->data;
	len  = netbuf_cb->len;

	if (!data || !len) 
		return -ENODATA;

	rc = bt_adapter->os_intf_ops->onebox_genl_app_send(gcb, netbuf_cb);
	if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		   (TEXT("%s: netbuf_cb %p fail(%d) to send\n"),
		   __func__, netbuf_cb, rc));
		return rc;
	}

	return rc;
}

/**
 * @fn          btgenl_init
 * @brief       Registers genl family and operations
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 * @section description
 * This API is used to register genl ops and family.
 */
int32 btgenl_init(BT_ADAPTER bt_adapter)
{
	int32 rc = -1;
	struct genl_cb *gcb;

	gcb = bt_adapter->os_intf_ops->onebox_mem_zalloc(sizeof(*gcb),
			GFP_KERNEL);
	if (!gcb) 
		return -ENOMEM;

	gcb->gc_drvpriv = bt_adapter;
	global_gcb = bt_adapter->genl_cb = gcb;
	
	gcb->gc_family  = &bt_genl_family;
	gcb->gc_policy  = &bt_genl_policy[0];
	gcb->gc_ops     = &bt_genl_ops;	
	gcb->gc_n_ops   = 1;
	gcb->gc_name    = ONEBOX_BT_GENL_FAMILY;
	gcb->gc_pid     = gcb->gc_done = 0;
	gcb->gc_assetid = BT_ID;

	rc = bt_adapter->os_intf_ops->onebox_genl_init(gcb);
	if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		   (TEXT("%s: family %s genl "
		         "registration fail %d\n"),
		   __func__, gcb->gc_name, rc)); 
		goto out_genl_init;
	}

	return 0;

out_genl_init:
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
	   (TEXT("%s: failed %d\n"), __func__, rc));

	bt_adapter->os_intf_ops->onebox_mem_free(gcb);
	bt_adapter->genl_cb = global_gcb = NULL;

	return rc;
}

/**
 * @fn          btgenl_deinit
 * @brief       Unregisters genl family and operations
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 * @section description
 * This API is used to unregister genl related ops.
 */
int32 btgenl_deinit(BT_ADAPTER bt_adapter)
{
	int32 rc = -1;
	struct genl_cb *gcb;

	gcb = bt_adapter->genl_cb;

	rc = bt_adapter->os_intf_ops->onebox_genl_deinit(gcb);
	if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		   (TEXT("%s: family %s genl deinit fail(%d)"),
		   __func__, gcb->gc_name, rc)); 
		return rc;
	}

	if (gcb) {
		bt_adapter->os_intf_ops->onebox_mem_free(gcb);
		global_gcb = bt_adapter->genl_cb = NULL;
	}

	return rc;
}
