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

ONEBOX_STATUS read_register(PONEBOX_ADAPTER adapter, uint32 Addr,uint8 *data);  
ONEBOX_STATUS usb_read_register(PONEBOX_ADAPTER adapter, uint32 Addr,uint8 *data);  
int write_register(PONEBOX_ADAPTER adapter,uint8 reg_dmn,uint32 Addr,uint8 *data);
int usb_write_register(PONEBOX_ADAPTER adapter,uint8 reg_dmn,uint32 Addr,uint8 *data);
ONEBOX_STATUS sdio_host_intf_read_pkt(PONEBOX_ADAPTER adapter,uint8 *pkt,uint32 Len);
ONEBOX_STATUS usb_host_intf_read_pkt(PONEBOX_ADAPTER adapter,uint8 *pkt,uint32 Len);
ONEBOX_STATUS usb_host_intf_write_pkt(PONEBOX_ADAPTER adapter,uint8 *pkt,uint32 Len, uint8 queueno, netbuf_ctrl_block_t *netbuf_cb);
ONEBOX_STATUS sdio_host_intf_write_pkt(PONEBOX_ADAPTER adapter,uint8 *pkt,uint32 Len, uint8 queueno, netbuf_ctrl_block_t *netbuf_cb);
ONEBOX_STATUS read_register_multiple(PONEBOX_ADAPTER adapter, 
                                     uint32 Addr,
                                     uint32 Count,
                                     uint8 *data );
ONEBOX_STATUS usb_read_register_multiple(PONEBOX_ADAPTER adapter, 
                                     uint32 Addr,
                                     uint32 Count,
                                     uint8 *data );
ONEBOX_STATUS write_register_multiple(PONEBOX_ADAPTER adapter,
                                      uint32 Addr,
                                      uint8 *data,
                                      uint32 Count, netbuf_ctrl_block_t *netbuf_cb);
ONEBOX_STATUS usb_write_register_multiple(PONEBOX_ADAPTER adapter,
                                      uint32 Addr,
                                      uint8 *data,
                                      uint32 Count,
                                      netbuf_ctrl_block_t *netbuf_cb);

ONEBOX_STATUS sdio_ack_interrupt(PONEBOX_ADAPTER adapter,uint8 INT_BIT);
int trace_firmware (struct driver_assets *d_assets);
ONEBOX_STATUS sdio_fnx_cmd52 (PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data, uint8 function_no, uint8 write);
ONEBOX_STATUS sdio_fnx_cmd53 (PONEBOX_ADAPTER adapter, uint32 Addr, uint8 *data,uint8 write, uint8 Count);
void onebox_rx_urb_submit (PONEBOX_ADAPTER adapter, struct urb *urb, uint8 ep_num);
ONEBOX_STATUS usb_write_ta_register_multiple(PONEBOX_ADAPTER adapter,
                                      uint32 Addr,
                                      uint8 *data,
                                      uint32 Count);
ONEBOX_STATUS usb_read_ta_register_multiple(PONEBOX_ADAPTER adapter,
                                      uint32 Addr,
                                      uint8 *data,
                                      uint32 Count);

int register_driver(void);
int register_usb_driver(void);
void unregister_driver(void);
void unregister_usb_driver(void);
ONEBOX_STATUS usb_remove(void);
ONEBOX_STATUS sdio_remove(void);
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
struct net_device * onebox_getcontext(PSDFUNCTION pfunction);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
void* onebox_getcontext(struct sdio_func *pfunction);
#endif
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
VOID onebox_sdio_claim_host(PSDFUNCTION pFunction);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
VOID onebox_sdio_claim_host(struct sdio_func *pfunction);
#endif
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
VOID onebox_sdio_release_host(PSDFUNCTION pFunction);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
VOID onebox_sdio_release_host(struct sdio_func *pfunction);
#endif
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
VOID onebox_setcontext(PSDFUNCTION pFunction, void *adapter);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
VOID onebox_setcontext(struct sdio_func *pfunction, void *adapter);
#endif
ONEBOX_STATUS onebox_setupcard(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS init_usb_host_interface(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS init_sdio_host_interface(PONEBOX_ADAPTER adapter);
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
int32 onebox_sdio_release_irq(PSDFUNCTION pFunction);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
int32 onebox_sdio_release_irq(struct sdio_func *pfunction);
#endif

int32 deregister_sdio_irq(PONEBOX_ADAPTER adapter);

ONEBOX_STATUS onebox_setclock(PONEBOX_ADAPTER adapter, uint32 Freq);

ONEBOX_STATUS onebox_sdio_abort_handler(PONEBOX_ADAPTER Adapter );
ONEBOX_STATUS onebox_init_sdio_slave_regs(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS onebox_init_usb_slave_regs(PONEBOX_ADAPTER adapter);
ONEBOX_STATUS disable_sdio_interrupt(PONEBOX_ADAPTER adapter);

uint8 process_usb_rcv_pkt(PONEBOX_ADAPTER adapter, netbuf_ctrl_block_t *netbuf_recv_pkt, uint8 pkt_type);
ONEBOX_STATUS onebox_ulp_reg_write (struct usb_device *usbdev, uint32 reg,uint16 value, uint16 len);
ONEBOX_STATUS onebox_ulp_reg_read (struct usb_device *usbdev, uint32 reg, uint16 * value, uint16 len);
void onebox_rx_done_handler (struct urb *urb);

