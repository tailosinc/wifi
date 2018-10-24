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


#include "onebox_common.h"
#include "onebox_linux.h"
#include "onebox_sdio_intf.h"
#include "onebox_host_intf_ops.h"
#include "onebox_zone.h"

/**
 * This function is invoked when the module is loaded into the
 * kernel. It registers the client driver.
 *
 * @param  VOID.  
 * @return On success 0 is returned else a negative value. 
 */
ONEBOX_STATIC int32 __init onebox_module_init(VOID)
{

	ONEBOX_STATUS status = 0;
	struct onebox_osd_host_intf_operations *osd_host_intf_ops ;

#ifdef USE_USB_INTF
	osd_host_intf_ops = onebox_get_usb_osd_host_intf_operations();
	/* registering the client driver */ 
	if (osd_host_intf_ops->onebox_register_drv() == 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
				(TEXT("%s: Successfully registered USB gpl driver\n"),
				 __func__));
			status = ONEBOX_STATUS_SUCCESS;
	} else {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to register USB driver\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		/*returning a negative value to imply error condition*/
	} /* End if <condition> */    
#endif

#ifdef USE_SDIO_INTF
	osd_host_intf_ops = onebox_get_sdio_osd_host_intf_operations();

	if (osd_host_intf_ops->onebox_register_drv() == 0) {
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT,
				(TEXT("%s: Successfully registered SDIO gpl driver\n"),
				 __func__));
			status = ONEBOX_STATUS_SUCCESS;
	} else {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
				(TEXT("%s: Unable to register SDIO gpl driver\n"), __func__));
		/*returning a negative value to imply error condition*/
		status = ONEBOX_STATUS_FAILURE;
	} /* End if <condition> */    
#endif
	return status;
}/* End of <onebox_module_init> */

/**
 * At the time of removing/unloading the module, this function is 
 * called. It unregisters the client driver.
 *
 * @param  VOID.  
 * @return VOID. 
 */
ONEBOX_STATIC VOID __exit onebox_module_exit(VOID)
{
	struct onebox_osd_host_intf_operations *osd_host_intf_ops;
	
#ifdef USE_USB_INTF
	osd_host_intf_ops = onebox_get_usb_osd_host_intf_operations();
	osd_host_intf_ops->onebox_unregister_drv();
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	             (TEXT("%s: Unregistered the USB driver\n"), __func__));
#endif

#ifdef USE_SDIO_INTF
	osd_host_intf_ops = onebox_get_sdio_osd_host_intf_operations();

	osd_host_intf_ops->onebox_unregister_drv();
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("%s: Unregistered the SDIO driver\n"), __func__));
#endif
	return;
}

module_init(onebox_module_init);
module_exit(onebox_module_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Redpine Signals, Inc.");
MODULE_DESCRIPTION("Driver for Redpine Signals' RS9113 module based USB/SDIO cards.");
MODULE_SUPPORTED_DEVICE("Redpine Signals' RS9113 module based USB/SDIO cards.");
