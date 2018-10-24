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

#define IMX_GPIO_CONTROL 1
#ifdef IMX_GPIO_CONTROL

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>
#include <mach/gpio.h>
#include <asm-generic/gpio.h>
#include <mach/../../../mach-mx5/mx51_pins.h>
//#include <mach/../../../mach-mx5/iomux.h>
//#include <mach/iomux-mx51.h>
#include <mach/iomux-v3.h>





#define MX51_GPIO_PAD_CTRL_OUTPUT (PAD_CTL_PKE |        \
				   PAD_CTL_PUE |        \
				   PAD_CTL_SRE_FAST |   \
				   PAD_CTL_PUS_22K_UP | \
				   PAD_CTL_DSE_MAX)

#define MX51_GPIO_PAD_CTRL_PSM		(PAD_CTL_DSE_HIGH | PAD_CTL_PKE | \
		PAD_CTL_SRE_FAST | PAD_CTL_ODE)

#define MX51_GPIO_PAD_CTRL_IP		(0 | \
		PAD_CTL_SRE_FAST | PAD_CTL_PUS_22K_UP | PAD_CTL_DSE_MAX)

#define MX51_PAD_CTRL_4_V 	(PAD_CTL_SRE_SLOW | PAD_CTL_PKE)

#define MX51_GPIO_PAD_CTRL		(PAD_CTL_DSE_HIGH | PAD_CTL_PKE | \
		PAD_CTL_SRE_FAST)

//#define MX51_PAD_GPIO_1_3__GPIO_1_3_V		IOMUX_PAD(0x7D8, 0x3D0, IOMUX_CONFIG_SION, 0x0, 0, MX51_GPIO_PAD_CTRL)
#define MX51_PAD_GPIO_1_3__GPIO_1_3_V		IOMUX_PAD(0x7D8, 0x3D0, IOMUX_CONFIG_GPIO, 0x0, 0, MX51_GPIO_PAD_CTRL_OUTPUT)
//#define MX51_PAD_GPIO_1_9__GPIO_1_9_V	  IOMUX_PAD(0x818, 0x3EC, IOMUX_CONFIG_SION, 0x0, 0, MX51_GPIO_PAD_CTRL)
#define MX51_PAD_GPIO_1_9__GPIO_1_9_V	  IOMUX_PAD(0x818, 0x3EC, IOMUX_CONFIG_GPIO, 0x0, 0, MX51_PAD_CTRL_4_V)

#define MX51_PAD_EIM_D20__GPIO_2_4_V  	IOMUX_PAD(0x400, 0x06c, 1, 0x0, 0, MX51_GPIO_PAD_CTRL)
#define MX51_PAD_EIM_D23__GPIO_2_7_V  	IOMUX_PAD(0x40c, 0x078, 1, 0x0, 0, MX51_GPIO_PAD_CTRL)
#define MX51_PAD_EIM_A18__GPIO_2_12_V	  IOMUX_PAD(0x438, 0x0a4, 1, 0x0,   0, MX51_GPIO_PAD_CTRL)
#define MX51_PAD_EIM_A21__GPIO_2_15_V 	IOMUX_PAD(0x444, 0x0b0, 1, 0x0,   0, MX51_GPIO_PAD_CTRL)
#define MX51_PAD_EIM_A25__GPIO_2_19_V 	IOMUX_PAD(0x454, 0x0c0, 1, 0x0,   0, MX51_GPIO_PAD_CTRL)
#define MX51_PAD_EIM_A26__GPIO_2_20_V 	IOMUX_PAD(0x458, 0x0c4, 1, 0x0,   0, MX51_GPIO_PAD_CTRL)
#define MX51_PAD_NANDF_D10__GPIO_3_30_V	IOMUX_PAD(0x550, 0x168, 3, 0x0, 0, MX51_GPIO_PAD_CTRL)
#define MX51_PAD_CSI2_D19__GPIO_4_12_V		IOMUX_PAD(0x5D8, 0x1E8, 3, 0x0, 0, MX51_GPIO_PAD_CTRL)
#define MX51_PAD_CSI2_VSYNC__GPIO_4_13_V	IOMUX_PAD(0x5DC, 0x1EC, 3, 0x0, 0, MX51_GPIO_PAD_CTRL_PSM)



#define GPIO1_3  (unsigned int)((0*32)+3)
#define GPIO1_9  (unsigned int)((0*32)+9)
#define GPIO2_4  (unsigned int)((1*32)+4)
#define GPIO2_7  (unsigned int)((1*32)+7)
#define GPIO2_12 (unsigned int)((1*32)+12)
#define GPIO2_15 (unsigned int)((1*32)+15)
#define GPIO2_19 (unsigned int)((1*32)+19)
#define GPIO2_20 (unsigned int)((1*32)+20)
#define GPIO3_30 (unsigned int)((2*32)+30)
#define GPIO4_12 (unsigned int)((3*32)+12)
#define GPIO4_13 (unsigned int)((3*32)+13)

#ifdef RSI_IMX51
#define RSI_GPIO_READ GPIO1_9
#define RSI_GPIO_WRITE GPIO1_3
#endif


#endif
