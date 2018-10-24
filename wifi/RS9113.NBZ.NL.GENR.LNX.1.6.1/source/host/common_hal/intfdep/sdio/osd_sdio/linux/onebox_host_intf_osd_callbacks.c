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
#include "onebox_intf_ops.h"

static struct onebox_osd_host_intf_operations sdio_osd_host_intf_ops =
{
	.onebox_read_register  = read_register,
	.onebox_write_register = write_register,
	.onebox_read_multiple  = read_register_multiple,
	.onebox_write_multiple = write_register_multiple,
	.onebox_register_drv   = register_driver,
	.onebox_unregister_drv = (void *)unregister_driver,
	.onebox_remove         = sdio_remove,
	.onebox_deregister_irq = deregister_sdio_irq,
	.onebox_master_reg_read   = sdio_master_reg_read,
	.onebox_master_reg_write  = sdio_master_reg_write
};

struct onebox_osd_host_intf_operations *onebox_get_sdio_osd_host_intf_operations(void)
{
	return &sdio_osd_host_intf_ops;
} 

//EXPORT_SYMBOL(onebox_get_sdio_osd_host_intf_operations);
