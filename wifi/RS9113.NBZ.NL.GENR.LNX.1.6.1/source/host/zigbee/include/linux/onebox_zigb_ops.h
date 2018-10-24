
/**
 * @file onebox_hal_ops.h
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
 * This file contians the function prototypes of the callbacks used across
 * differnet layers in the driver
 * 
 */
#include "onebox_common.h"

struct onebox_osi_zigb_ops {
	int32	(*onebox_core_init)(ZB_ADAPTER zb_adapter);
	int32	(*onebox_core_deinit)(ZB_ADAPTER zb_adapter);
	void	(*onebox_dump)(int32 dbg_zone_l, PUCHAR msg_to_print_p,int32 len);
	ONEBOX_STATUS	(*onebox_send_pkt)(ZB_ADAPTER zb_adapter,
					   netbuf_ctrl_block_t *netbuf_cb);
	int32	(*onebox_zb_cw)(ZB_ADAPTER adapter,
	                               unsigned char *,int );};

struct onebox_zigb_osd_operations {
  int32	(*onebox_zigb_register_genl)(ZB_ADAPTER zb_adapter);
  int32	(*onebox_zigb_deregister_genl)(ZB_ADAPTER zb_adapter);
  int32	(*onebox_zigb_app_send)(ZB_ADAPTER zb_adapter, netbuf_ctrl_block_t *netbuf_cb);
};
/* EOF */
