#include "zb_common.h"
#include "onebox_zigbee_core.h"
#include "onebox_zigbee_pktpro.h"
#include "onebox_linux.h"

static struct onebox_osi_zigb_ops osi_zigb_ops = {
	.onebox_core_init   = core_zigb_init,
	.onebox_core_deinit = core_zigb_deinit,
	.onebox_dump        = onebox_print_dump,
	.onebox_send_pkt    = send_zigb_pkt,
	.onebox_zb_cw       = zigb_cw,
};

struct onebox_osi_zigb_ops *onebox_get_osi_zigb_ops(void)
{
	return &osi_zigb_ops;	
}
EXPORT_SYMBOL(onebox_get_osi_zigb_ops);
