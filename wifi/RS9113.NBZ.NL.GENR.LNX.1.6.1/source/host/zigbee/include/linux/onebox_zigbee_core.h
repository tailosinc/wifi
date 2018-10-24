/**
 * @file onebox_core.h
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
 * This file contians the function prototypes used in the core module
 * 
 */
#ifndef __ONEBOX_CORE_H__
#define __ONEBOX_CORE_H__

#include "onebox_common.h"

void onebox_print_dump(int32 dbg_zone_l, PUCHAR msg_to_print_p,int32 len);
int32 core_zigb_init(ZB_ADAPTER zb_adapter);
int32 core_zigb_deinit(ZB_ADAPTER zb_adapter);
int32 zigb_cw(ZB_ADAPTER zb_adapter, uint8 *data, int32 len);

#endif
