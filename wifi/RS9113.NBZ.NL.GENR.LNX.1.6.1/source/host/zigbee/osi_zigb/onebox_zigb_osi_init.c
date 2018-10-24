#include <linux/module.h>
#include <linux/kernel.h>
#include "zb_common.h"

uint32 onebox_zigb_zone_enabled = ONEBOX_ZONE_INFO |
				  ONEBOX_ZONE_INIT |
				  ONEBOX_ZONE_OID |
				  ONEBOX_ZONE_MGMT_SEND |
				  ONEBOX_ZONE_MGMT_RCV |
				  ONEBOX_ZONE_DATA_SEND |
				  ONEBOX_ZONE_DATA_RCV |
				  ONEBOX_ZONE_FSM |
				  ONEBOX_ZONE_ISR |
				  ONEBOX_ZONE_MGMT_DUMP |
				  ONEBOX_ZONE_DATA_DUMP |
				  ONEBOX_ZONE_DEBUG |
				  ONEBOX_ZONE_AUTORATE |
				  ONEBOX_ZONE_PWR_SAVE |
				  ONEBOX_ZONE_ERROR |
				  0;
EXPORT_SYMBOL(onebox_zigb_zone_enabled);

ONEBOX_STATIC int32 onebox_zigb_nongpl_module_init(VOID)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("registering the zigbee nongpl driver\n")));
	return 0;
}

ONEBOX_STATIC VOID onebox_zigb_nongpl_module_exit(VOID)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("unregistering the zigbee nongpl driver\n")));
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("Zigbee NON GPL Module exit\n")));
	return;
}

module_init(onebox_zigb_nongpl_module_init);
module_exit(onebox_zigb_nongpl_module_exit);
