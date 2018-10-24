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

/* include files */
#include "onebox_common.h"
#include "onebox_linux.h"
#include "onebox_mgmt.h"
#include "onebox_intf_ops.h"
#include "onebox_sdio_intf.h"
#include "onebox_zone.h"

/* The following structure is used to configure values */
struct rsi_config_vals_s dev_config_vals[] = {
	 {.lp_sleep_handshake = 0,
	  .ulp_sleep_handshake = 0,
	  .sleep_config_params = 0,
	  .host_wakeup_intr_enable = 0, 
	  .host_wakeup_intr_active_high = 0,
	  .ext_pa_or_bt_coex_en = 0,
	  //.lp_wakeup_threshold = 0,
	  //.ulp_wakeup_threshold = 5000 
	  },
};

rsi_ulp_gpio_vals unused_ulp_gpio_bitmap = {
   .rsi_ulp_gpio_vals_s_info = RSI_MOTION_SENSOR_GPIO_ULP_WAKEUP | RSI_SLEEP_INDICATION_FROM_DEVICE | RSI_ULP_GPIO_2 | RSI_PUSH_BUTTON_ULP_WAKEUP, 
};

rsi_soc_gpio unused_soc_gpio_bitmap = {
	.rsi_soc_gpio_vals_s_info = RSI_HOST_WAKEUP_INTR | RSI_UART1_RX | RSI_UART1_TX | RSI_UART1_RTS_I2S_CLK | RSI_UART1_CTS_I2S_WS | RSI_DEBUG_UART_RX_I2S_DIN | RSI_DEBUG_UART_TX_I2S_DOUT | RSI_LP_WAKEUP_BOOT_BYPASS  							| RSI_BT_COEXISTANCE_WLAN_ACTIVE_EXT_PA_ANT_SEL_A | RSI_BT_COEXISTANCE_BT_PRIORITY_EXT_PA_ANT_SEL_B | RSI_BT_COEXISTANCE_BT_ACTIVE_EXT_PA_ON_OFF | RSI_SLEEP_INDICATION_FROM_DEVICE_SOC ,
};

/**
 * configure_common_dev_params() - This function is used to send a frame to the
 * 			           firmware to configure some device parameters.
 * 			           The params are picked from the struct above.
 *
 * @adapter: The common driver structure.
 *
 * Return: ONEBOX_STATUS_SUCCESS on successful writing of frame, else 
 * 	   ONEBOX_STATUS_FAILURE
 */

ONEBOX_STATUS onebox_configure_common_dev_params(PONEBOX_ADAPTER adapter)
{

	onebox_mac_frame_t *mgmt_frame;
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	uint16 *frame_body;
	uint32 *unused_soc_gpio ;
	uint16 *unused_ulp_gpio ;
	struct driver_assets *d_assets = adapter->d_assets;

	netbuf_cb = adapter->os_intf_ops->onebox_alloc_skb(FRAME_DESC_SZ + sizeof(rsi_config_vals));
	if(netbuf_cb == NULL)
	{            
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		return ONEBOX_STATUS_FAILURE;

	}    
	adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, (FRAME_DESC_SZ + sizeof(rsi_config_vals)));
	adapter->os_intf_ops->onebox_memset(netbuf_cb->data,0,(FRAME_DESC_SZ));

	mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
	frame_body = (uint16 *)&netbuf_cb->data[16];
	adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, (FRAME_DESC_SZ + sizeof(rsi_config_vals)));

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16((sizeof(rsi_config_vals)) 
	                                              | (COEX_TX_Q << 12));
	netbuf_cb->tx_pkt_type = COEX_Q;
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(COMMON_DEV_CONFIG); 
	if (d_assets->lp_ps_handshake_mode == PACKET_HAND_SHAKE) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Ivalid Configuration of PACKET_HAND_SHAKE in LP_MODE\n"));
		return ONEBOX_STATUS_FAILURE;
	}

	frame_body[0] = ONEBOX_CPU_TO_LE16((uint16 )d_assets->lp_ps_handshake_mode);
	frame_body[0] |= ONEBOX_CPU_TO_LE16((uint16 )d_assets->ulp_ps_handshake_mode << 8);

	if (d_assets->rf_power_val == RF_POWER_3_3) {
	    frame_body[1] = BIT(1);
  }

  if(dev_config_vals[0].host_wakeup_intr_enable) { 
      unused_soc_gpio_bitmap.rsi_soc_gpio_vals_s_info &= ~RSI_HOST_WAKEUP_INTR;
      //frame_body[1] |= (((uint16 )(dev_config_vals[0].host_wakeup_intr_enable)) << 8);
      frame_body[1] |= BIT(2); //HOST_WAKEUP_INTR_EN
      if(dev_config_vals[0].host_wakeup_intr_active_high)
      {
          frame_body[1] |= BIT(3); //HOST_WAKEUP_INTR_ACTIVE_HIGH

      }
  }

   unused_ulp_gpio = (uint16 *)&unused_ulp_gpio_bitmap;
   //frame_body[1] |= ((uint16)unused_ulp_gpio_bitmap << 8 );
   frame_body[1] |= ONEBOX_CPU_TO_LE16((*unused_ulp_gpio) << 8);

  if ((d_assets->lp_ps_handshake_mode == GPIO_HAND_SHAKE) ||
          (d_assets->ulp_ps_handshake_mode == GPIO_HAND_SHAKE)) {
       
      if ((d_assets->lp_ps_handshake_mode == GPIO_HAND_SHAKE)) {
          unused_soc_gpio_bitmap.rsi_soc_gpio_vals_s_info &= ~RSI_LP_WAKEUP_BOOT_BYPASS ;
      }

      if ((d_assets->ulp_ps_handshake_mode == GPIO_HAND_SHAKE)) {
            unused_ulp_gpio_bitmap.rsi_ulp_gpio_vals_s_info &= ~RSI_MOTION_SENSOR_GPIO_ULP_WAKEUP;
      }


      if ((d_assets->device_gpio_type == TA_GPIO)) {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
                  (TEXT("%s : Line %d Invalid Configuration OF ULP_PS_HANDHSAKE_MODE Enabling Sleep_Indication_from_device GPIO \n"),
                   __func__, __LINE__));
          unused_soc_gpio_bitmap.rsi_soc_gpio_vals_s_info &= ~RSI_SLEEP_INDICATION_FROM_DEVICE_SOC ;				
      } else {
          unused_ulp_gpio_bitmap.rsi_ulp_gpio_vals_s_info &= ~RSI_SLEEP_INDICATION_FROM_DEVICE;
          frame_body[1] |= BIT(0); //ULP_GPIO for Handshake
      }
  }          


  dev_config_vals[0].opermode = d_assets->oper_mode;

  if(dev_config_vals[0].ext_pa_or_bt_coex_en ) {
       frame_body[4] = (uint8 )dev_config_vals[0].ext_pa_or_bt_coex_en;

       unused_soc_gpio_bitmap.rsi_soc_gpio_vals_s_info &= ~(RSI_BT_COEXISTANCE_WLAN_ACTIVE_EXT_PA_ANT_SEL_A | RSI_BT_COEXISTANCE_BT_PRIORITY_EXT_PA_ANT_SEL_B | RSI_BT_COEXISTANCE_BT_ACTIVE_EXT_PA_ON_OFF);  

  }
  frame_body[4] |= ONEBOX_CPU_TO_LE16((uint16 )dev_config_vals[0].opermode << 8);
  frame_body[5] |= ONEBOX_CPU_TO_LE16((uint16 )d_assets->wlan_rf_power_mode);
  frame_body[5] |= ONEBOX_CPU_TO_LE16((uint16 )d_assets->bt_rf_power_mode << 8);
  frame_body[6] |= ONEBOX_CPU_TO_LE16((uint16 )d_assets->zigb_rf_power_mode);
  frame_body[6] |= ONEBOX_CPU_TO_LE16((uint16 )adapter->Driver_Mode << 8);
  frame_body[7] = ONEBOX_CPU_TO_LE16((uint16 )d_assets->region_code);
  frame_body[7] |= ONEBOX_CPU_TO_LE16((uint16 )d_assets->obm_ant_sel_val << 8);
  frame_body[8] = ONEBOX_CPU_TO_LE16(d_assets->peer_dist); /*2 bytes*/
  frame_body[9] = ONEBOX_CPU_TO_LE16(d_assets->bt_feature_bitmap); /*2 bytes*/
#ifdef UART_DEBUG
  frame_body[10] = ONEBOX_CPU_TO_LE16(d_assets->uart_debug); /*2 bytes*/
#endif

  if(d_assets->device_model == RSI_DEV_9116) {
    //! In frame_body[11] BITS(3:0) are used for module type selection, BIT(4) is used for host interface on demand feature option
    frame_body[11] = ONEBOX_CPU_TO_LE16((d_assets->ext_opt & 0xF) | (d_assets->host_intf_on_demand << 4));
  }
    unused_soc_gpio = ((uint32 *)&unused_soc_gpio_bitmap);
  *(uint32 *)&frame_body[2] = ONEBOX_CPU_TO_LE32(*unused_soc_gpio);

  adapter->coex_osi_ops->onebox_dump(ONEBOX_ZONE_DEBUG,
		  (uint8 *)mgmt_frame,
		  FRAME_DESC_SZ + sizeof(rsi_config_vals)) ;
  send_pkt_to_coex(d_assets, netbuf_cb, COEX_Q);
  return status;


}
EXPORT_SYMBOL(onebox_configure_common_dev_params);
