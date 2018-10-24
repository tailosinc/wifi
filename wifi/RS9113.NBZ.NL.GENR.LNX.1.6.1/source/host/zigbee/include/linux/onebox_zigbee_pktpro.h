
/**
 * @file onebox_pktpro.h
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
 * This file contians the function prototypes used for sending /receiving packets to/from the
 * driver
 * 
 */

#ifndef __ONEBOX_PKTPRO_H__
#define __ONEBOX_PKTPRO_H__

#include "onebox_datatypes.h"
#include "onebox_common.h"

ONEBOX_STATUS send_zigb_pkt(ZB_ADAPTER zb_adapter, netbuf_ctrl_block_t *netbuf_cb);
ONEBOX_STATUS device_init(ZB_ADAPTER zb_adapter);
ONEBOX_STATUS device_deinit(ZB_ADAPTER zb_adapter);

#endif

