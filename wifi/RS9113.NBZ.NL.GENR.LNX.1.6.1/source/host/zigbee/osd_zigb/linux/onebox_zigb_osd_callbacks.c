/**
 * @file
 * @author  Sowjanya
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
 * This file contains all the function pointer initializations for os
 * interface.
 */
#include "zb_common.h"
#include "onebox_linux.h"

/* osd operations can be called by Core */
static struct onebox_zigb_osd_operations zigb_osd_ops = {
	.onebox_zigb_register_genl   = zigb_register_genl,
	.onebox_zigb_deregister_genl = zigb_unregister_genl,
	//.onebox_zigb_app_send        = onebox_genlsend,
};

struct onebox_zigb_osd_operations *onebox_get_zigb_osd_ops(void)
{
  return (&zigb_osd_ops);
}



