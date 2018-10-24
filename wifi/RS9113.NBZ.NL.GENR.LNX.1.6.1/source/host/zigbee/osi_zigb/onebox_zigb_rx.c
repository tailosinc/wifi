/**
 *
* @file   onebox_dev_ops.c
* @author 
* @version 1.0
*
* @section LICENSE
*
* This software embodies materials and concepts that are confidential to Redpine
* Signals and is made available solely pursuant to the terms of a written license
* agreement with Redpine Signals
*
* @section DESCRIPTION
*
*/

/* include files */
#include "zb_common.h"
#include "onebox_linux.h"
#include "onebox_zigbee_pktpro.h"

/**
 * This function read frames from the SD card.
 *
 * @param  Pointer to driver zb_adapter structure.  
 * @param  Pointer to received packet.  
 * @param  Pointer to length of the received packet.  
 * @return 0 if success else -1. 
 */
ONEBOX_STATUS zigb_read_pkt(ZB_ADAPTER zb_adapter, netbuf_ctrl_block_t *netbuf_cb1)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("zb_adapter = %p\n"),
	             zb_adapter));
  	/* Queuing the packets here so that zigbee will poll and receive them*/
	if( zb_adapter->read_cmd || zb_adapter->wait_cmd )
	{
       zb_adapter->os_intf_ops->onebox_set_event(&(zb_adapter->zb_per_event));
	   zb_adapter->cnfm_flag = 1;
	}

	if((zb_adapter->driver_mode != RF_EVAL_MODE_ON) || (zb_adapter->read_cmd))
	{
  		zb_adapter->os_intf_ops->onebox_genl_app_send(zb_adapter->genl_cb, netbuf_cb1);
	}
	zb_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb1, 0);
	FUNCTION_EXIT(ONEBOX_ZONE_INFO);

	return ONEBOX_STATUS_SUCCESS;
}
EXPORT_SYMBOL(zigb_read_pkt);

