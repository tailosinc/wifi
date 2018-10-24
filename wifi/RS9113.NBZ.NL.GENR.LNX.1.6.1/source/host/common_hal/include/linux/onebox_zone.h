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

#define ONEBOX_BIT(n)                   (1 << (n))
#define ONEBOX_ZONE_ERROR               ONEBOX_BIT(0)  /* For Error Msgs              */
#define ONEBOX_ZONE_WARN                ONEBOX_BIT(1)  /* For Warning Msgs            */
#define ONEBOX_ZONE_INFO                ONEBOX_BIT(2)  /* For General Status Msgs     */
#define ONEBOX_ZONE_INIT                ONEBOX_BIT(3)  /* For Driver Init Seq Msgs    */
#define ONEBOX_ZONE_OID                 ONEBOX_BIT(4)  /* For IOCTL Path Msgs         */
#define ONEBOX_ZONE_MGMT_SEND           ONEBOX_BIT(5)  /* For TX Mgmt Path Msgs       */
#define ONEBOX_ZONE_MGMT_RCV            ONEBOX_BIT(6)  /* For RX Mgmt Path Msgs       */
#define ONEBOX_ZONE_DATA_SEND           ONEBOX_BIT(7)  /* For TX Data Path Msgs       */
#define ONEBOX_ZONE_DATA_RCV            ONEBOX_BIT(8)  /* For RX Data Path Msgs       */
#define ONEBOX_ZONE_FSM                 ONEBOX_BIT(9)  /* For State Machine Msgs      */
#define ONEBOX_ZONE_ISR                 ONEBOX_BIT(10) /* For Interrupt Specific Msgs */
#define ONEBOX_ZONE_MGMT_DUMP           ONEBOX_BIT(11) /* For Dumping Mgmt Pkts       */
#define ONEBOX_ZONE_DATA_DUMP           ONEBOX_BIT(12) /* For Dumping Data Pkts       */
#define ONEBOX_ZONE_CLASSIFIER          ONEBOX_BIT(13) /* For Classification Msgs     */
#define ONEBOX_ZONE_PROBE               ONEBOX_BIT(14) /* For Probe Req & Rsp Msgs    */
#define ONEBOX_ZONE_EXIT                ONEBOX_BIT(15) /* For Driver Unloading Msgs   */
#define ONEBOX_ZONE_DEBUG               ONEBOX_BIT(16) /* For Extra Debug Messages    */
#define ONEBOX_ZONE_PWR_SAVE             ONEBOX_BIT(17) /* For Powersave Blk Msgs      */
#define ONEBOX_ZONE_AGGR                ONEBOX_BIT(18) /* For Aggregation Msgs        */
#define ONEBOX_ZONE_DAGGR               ONEBOX_BIT(19) /* For Deaggregation Msgs      */
#define ONEBOX_ZONE_AUTORATE            ONEBOX_BIT(20) /* For Autorate Msgs           */
#define ONEBOX_ZONE_USB_DEBUG_INFO      ONEBOX_BIT(21) /* For USB Debug               */
#define ONEBOX_ZONE_STRUCT_PRGM_INFO      ONEBOX_BIT(22) /* For RF Structure programming Information */

#define ONEBOX_STATUS_SUCCESS       0
#define ONEBOX_STATUS_FAILURE      -1

/***************************** START MACROS ********************************/
#define ONEBOX_PRINT printk
#define ONEBOX_SPRINTF  sprintf

#ifdef ONEBOX_DEBUG_ENABLE
#define ONEBOX_DEBUG1    ONEBOX_PRINT

 extern uint32 onebox_zone_enabled;


#define PRINT_MAC_ADDR(zone, buf) do {\
	if (zone & onebox_zone_enabled) {\
               ONEBOX_PRINT("%02x:%02x:%02x:%02x:%02x:%02x\n",\
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);\
	}\
} while (0);

#define ONEBOX_ASSERT(exp) do {\
	if (!(exp)) {\
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,\
		             ("===> ASSERTION FAILED <===\n"));\
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,\
		             ("Expression: %s\n", #exp));\
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,\
		             ("Function        : %s()\n", __FUNCTION__));\
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,\
		             ("File: %s:%d\n", __FILE__, __LINE__));\
	}\
} while (0)

#define FUNCTION_ENTRY(zone) do {\
	if (zone & onebox_zone_enabled) {\
            ONEBOX_PRINT("+%s()\n", __FUNCTION__);\
	}\
} while (0);

#define FUNCTION_EXIT(zone) do {\
	if (zone & onebox_zone_enabled) {\
             ONEBOX_PRINT("-%s()\n", __FUNCTION__);\
	}\
} while (0);
#else
 #define PRINT_MAC_ADDR(zone, buf)
 #define ONEBOX_ASSERT(exp)
 #define FUNCTION_ENTRY(zone)
 #define FUNCTION_EXIT(zone)
#endif

