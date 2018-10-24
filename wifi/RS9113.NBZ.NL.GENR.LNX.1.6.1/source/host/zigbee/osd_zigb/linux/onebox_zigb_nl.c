/**
 * @file      	onebox_zigb_nl.c
 * @version	1.0
 * @date        2014-Nov-20
 *
 * Copyright(C) Redpine Signals 2013
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief This contains all the functions with netlink socket
 * usage
 *
 * @section Description
 * This file contains the following functions.
 *      zigb_genlrecv
 *      zigb_register_genl
 *      zigb_unregister_genl
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/errno.h>

#include <net/genetlink.h>
#include <net/sock.h>

#include "zb_common.h"
#include "onebox_linux.h"
#include "onebox_coex.h"

static struct genl_cb *global_gcb;

int32 zigb_genlrecv(struct sk_buff *skb, struct genl_info *info);

#define ONEBOX_ZIGB_GENL_FAMILY "Obx-ZIGBgenl"

#define GET_ADAPTER_FROM_GENLCB(gcb) \
		(ZB_ADAPTER)(gcb->gc_drvpriv)

/* 
 * attribute policy: defines which attribute has 
 * which type (e.g int, char * etc)
 * possible values defined in net/netlink.h
 */
static struct nla_policy zigb_genl_policy[RSI_USER_A_MAX + 1] = {
	[RSI_USER_A_MSG] = { .type = NLA_NUL_STRING },
};

/* family definition */
static struct genl_family zigb_genl_family = {
	.id      = 0,
	.hdrsize = 0,
	.name    = ONEBOX_ZIGB_GENL_FAMILY,
	.version = RSI_VERSION_NR,
	.maxattr = RSI_USER_A_MAX,
};

static struct genl_ops zigb_genl_ops = {
	.cmd    = RSI_USER_C_CMD,
	.flags  = 0,
	.policy = zigb_genl_policy,
	.doit   = zigb_genlrecv,
	.dumpit = NULL,
};

/*==========================================================================/
 * @fn          zigb_genlrecv(struct sk_buff *rskb, struct genl_info *info)
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
 *===========================================================================*/
int32 zigb_genlrecv(struct sk_buff *skb, struct genl_info *info)
{
	uint8 *data;
	uint16 desc;
	int32  rc = -1, len, pkttype;
	struct genl_cb *gcb;
	netbuf_ctrl_block_t *netbuf_cb;
	ZB_ADAPTER zb_adapter = NULL;
	bb_rf_params_t *bb_rf_params;

	if(!(gcb = global_gcb))
		return -1;
	
	zb_adapter = GET_ADAPTER_FROM_GENLCB(global_gcb);
	if (!zb_adapter)
		return -1;

	gcb->gc_info = info;
	gcb->gc_skb  = skb;

	data = zb_adapter->os_intf_ops->onebox_genl_recv_handle(gcb);
	if (!data) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		    (TEXT("%s: genlrecv handler fail on family `%s'\n"),
		    __func__, gcb->gc_name)); 
		goto err;
	}

	gcb->gc_info = NULL;
	gcb->gc_skb  = NULL;

	desc = *(uint16 *)&data[0];
	len = desc & 0x0FFF;
	pkttype = ((desc & 0xF000) >> 12);

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("%s: rx data, desc %x len %x pkttype %x\n"),
			 __func__, desc, len, pkttype));

	zb_adapter->osi_zigb_ops->onebox_dump(ONEBOX_ZONE_INFO, data, len + 16);

	zb_adapter->read_cmd = 0;
	zb_adapter->wait_cmd = 0;
	if( zb_adapter->driver_mode == RF_EVAL_MODE_ON )
	{
		if (data[15] == ZIGB_PER)
		{
			bb_rf_params = (bb_rf_params_t *)(data + 16);
			switch( bb_rf_params->value)
			{
				case BB_READ:
				case RF_READ:
				case ZB_PER_STATS:
				case BUFFER_READ:
					zb_adapter->read_cmd = 1;
					break;
				case BUFFER_WRITE:
				case SET_ZIGB_CHAN:
					zb_adapter->read_cmd = 1;
					break;
				case TX_STATUS:
					if( zb_adapter->tx_is_in_progress)
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
								(TEXT("%s:ALREADY IN TRANSMIT SO STOP THE TRASMIT FIRST \n"), __func__)); 
						return TX_IS_IN_PROG;
					}
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
							(TEXT("%s:TRANSMIT YET TO BE STARTED \n"), __func__)); 
    					return TX_IS_NOT_IN_PROG;
				case ZIGB_PER_TRANSMIT:
					zb_adapter->tx_is_in_progress = bb_rf_params->Data[0];
					break;

					case ZIGB_CW_MODE:
							{
									printk("in ZIGB_CW\n");
									if( zb_adapter->osi_zigb_ops->onebox_zb_cw(zb_adapter, data,len) != ONEBOX_STATUS_SUCCESS )
									{
											ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
															(TEXT("%s: Unable to perform PER operations.\n"), __func__));
											return ONEBOX_STATUS_FAILURE;
									}
									return ONEBOX_STATUS_SUCCESS;
							}
							break;
					/*default:
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
											(TEXT("%s: Invalid Pkt type %d\n"), __func__, rc)); 
							return ONEBOX_STATUS_FAILURE;
							* break is not needed here */
			}
		}
		else
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid Functionality in RF_EVAL Mode\n")));
			return ONEBOX_STATUS_FAILURE;
		}
	}
	else
	{
		if (data[15] == ZIGB_PER)
    {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid Functionality in END-to-END Mode\n")));
			return ONEBOX_STATUS_FAILURE;
    }
	}	

	netbuf_cb = zb_adapter->os_intf_ops->onebox_alloc_skb(len +16); 
	if (!netbuf_cb) {
		rc = -ENOMEM;
		goto err;
	}

	zb_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len + 16);

	zb_adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, data, (len + 16));

	rc = zb_adapter->osi_zigb_ops->onebox_send_pkt(zb_adapter, netbuf_cb);
	if (rc) 
		goto err;

	if( zb_adapter->read_cmd || zb_adapter->wait_cmd)
	{
		zb_adapter->os_intf_ops->onebox_reset_event(&(zb_adapter->zb_per_event));
		if( !zb_adapter->cnfm_flag )
		{
			if((zb_adapter->os_intf_ops->onebox_wait_event(&(zb_adapter->zb_per_event), 10000)  == 0))
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
						(TEXT("%s:NO RESPONSE BEFORE SPECIFIED TIME \n"), __func__)); 
				return ONEBOX_STATUS_FAILURE;
			}
		}

	}

	return ONEBOX_STATUS_SUCCESS;


err:
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("%s: error(%d) occured \n"), __func__, rc)); 
	return rc;
}

/*================================================================/
 * @fn          int32 zigb_register_genl(ZB_ADAPTER zb_adapter)
 * @brief       Registers genl family and operations
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 * @section description
 * This API is used to register genl ops and family.
 *==================================================================*/
int32 zigb_register_genl(ZB_ADAPTER zb_adapter)
{
	int32 rc = -1;
	struct genl_cb *gcb;

	gcb = zb_adapter->os_intf_ops->onebox_mem_zalloc(sizeof(*gcb), 
			GFP_KERNEL);
	if (!gcb) {
		rc = -ENOMEM;
		return rc;
	}

	gcb->gc_drvpriv = zb_adapter;
	global_gcb = zb_adapter->genl_cb = gcb;
	
	gcb->gc_family = &zigb_genl_family;
	gcb->gc_policy = &zigb_genl_policy[0];
	gcb->gc_ops    = &zigb_genl_ops;	
	gcb->gc_name   = ONEBOX_ZIGB_GENL_FAMILY;
	gcb->gc_n_ops  = 1;
	gcb->gc_pid    = gcb->gc_done = 0;
	   
	rc = zb_adapter->os_intf_ops->onebox_genl_init(gcb);
	if (rc != 0) 
		goto err;

	return rc;
	
err:
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		(TEXT("%s: error(%d) occured\n"), __func__, rc));
	zb_adapter->os_intf_ops->onebox_mem_free(gcb);
	zb_adapter->genl_cb = global_gcb = NULL;

	return rc;
}

/*================================================================*/
/**
 * @fn          int32 zigb_unregister_genl(ZB_ADAPTER zb_adapter)
 * @brief       Unregisters genl family and operations
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 * @section description
 * This API is used to unregister genl related ops.
 *================================================================*/
int32 zigb_unregister_genl(ZB_ADAPTER zb_adapter)
{
	int rc;
	struct genl_cb *gcb;

	if (!(gcb = zb_adapter->genl_cb))
		return -ENODEV;
     
	rc = zb_adapter->os_intf_ops->onebox_genl_deinit(gcb);
	if (rc != 0) 
		goto err;

	return rc;

err:
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		(TEXT("%s: error(%d) occured\n"), __func__, rc));
	return rc;

}
