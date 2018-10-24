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

#include <linux/module.h>
#include <linux/kernel.h>
#include "onebox_datatypes.h"
#include "onebox_common.h"
#include "onebox_pktpro.h"
#include "onebox_sdio_intf.h"
#include "onebox_regdomain.h"
#include "onebox_host_intf_ops.h"
#include "onebox_zone.h"


static ushort sdio_clock     = 0;
static bool enable_high_speed     = 0;
static bool enable_antenna_diversity = 0;

uint8 firmware_path[256]  = "/home/rsi/release/firmware/";
static uint16 driver_mode = 1;
static uint16 coex_mode = 1; /*Default coex mode is WIFI alone */
static uint16 oper_mode = 1; 
static uint16 ta_aggr = 0;
static uint16 skip_fw_load = 0; /* Default disable skipping fw loading */
static uint16 fw_load_mode = 1; /* Default fw loading mode is full flash with Secondary Boot loader*/
static uint16 lp_ps_handshake_mode = 0; /* Default No HandShake mode*/
static uint16 ulp_ps_handshake_mode = 2; /* Default PKT HandShake mode*/
static uint16 rf_power_val = 0; /* Default 1.9V */ 
static uint16 device_gpio_type = TA_GPIO; /* Default TA GPIO */ 
static uint16 country_code = 840; /* Default US */ 
static uint16 wlan_rf_power_mode = 0;
static uint16 bt_rf_power_mode = 0;
static uint16 zigb_rf_power_mode = 0;
static uint16 obm_ant_sel_val = 2;
static uint16 user_onboard_ant_val = 1;
static uint16 peer_dist = 0;
static uint16 retry_count = 15; /* Default retry count value is 15*/
static uint16 bt_feature_bitmap = 0;
static uint16 uart_debug = 0;
static uint16 module_type = 0;  /* Default module type is (0) B8 */
static uint16 pll_mode = 0;     /* Default pll mode is (0) PLL_MODE0 */
static uint16 rf_type_2g = 1;   /* Default rf type for 2g is (1) Internal RF */
static uint16 pwr_save_opt = 3; /* Default power save options is (3) Enable Duty Cycling & Undestined Packet Drop */
static uint16 ext_opt = 0;      /* Default extended options is (0) */
static bool standby_assoc_chain_sel = 1; /* Default chain selection in standby associated mode is (1) LP Chain */
static uint16 lmac_bcon_drop = 1;     /* Default option for LMAC Beacon Drop is (1) Enable LMAC BEACON DROP Feature */
static bool host_intf_on_demand = 1;  /* Default option for Host Interface on demand is (1) Enable Host Interface on Demand Feature */
static uint16 txpkt_lifetime = 0;     /* Default option for Tx Packet lifetime for LMAC.*/

module_param(lp_ps_handshake_mode,ushort,0);
MODULE_PARM_DESC(lp_ps_handshake_mode, "Enable (0) No HandShake Mode     \
					Enable (1) GPIO HandShake Mode");
module_param(ulp_ps_handshake_mode,ushort,0);
MODULE_PARM_DESC(ulp_ps_handshake_mode, "Enable (0) No HandShake Mode     \
					 Enable (1) GPIO HandShake Mode \
					 Enable (2) Packet HandShake Mode");
module_param(rf_power_val,ushort,0);
MODULE_PARM_DESC(rf_power_val, "Enable (0) 1.9 Volts power for RF \
				Enable (1) 3.3 Volts power for RF");
module_param(device_gpio_type,ushort,0);
MODULE_PARM_DESC(device_gpio_type, "Enable (TA_GPIO) selects TA GPIO \
				Enable (ULP_GPIO) selects ULP GPIO");

module_param(driver_mode,ushort,0);
MODULE_PARM_DESC(driver_mode, "Enable (1) WiFi mode or Enable (2) RF Eval mode \
                               Enable (3) RF_EVAL_LPBK_CALIB mode (4) RF_EVAL_LPBK mode \
                               Enable (5) QSPI BURNING mode");
module_param(coex_mode,ushort,0);
MODULE_PARM_DESC(coex_mode, "(1) WiFi ALONE (2) WIFI AND BT \
                               (3) WIFI AND ZIGBEE");
module_param(oper_mode,ushort,0);
MODULE_PARM_DESC(oper_mode, "(1)WIFI_AP  (2) WIFI_STA"); 

module_param(ta_aggr, ushort, 0);
MODULE_PARM_DESC(ta_aggr, "No of pkts to aggregate from TA to host");

module_param(country_code, ushort, 0);
MODULE_PARM_DESC(country_code, "Country Code to select region");

module_param(peer_dist, ushort, 0);
MODULE_PARM_DESC(peer_dist, "peer distance to configure ack timeout value");

module_param(bt_feature_bitmap, ushort, 0);
MODULE_PARM_DESC(bt_feature_bitmap, "Feature bitmap for BT");

module_param(uart_debug, ushort, 0);
MODULE_PARM_DESC(uart_debug, "Feature bitmap for uart debug");

module_param(wlan_rf_power_mode, ushort, 0);
MODULE_PARM_DESC(wlan_rf_power_mode, "RF Power Mode for WLAN");

module_param(bt_rf_power_mode, ushort, 0);
MODULE_PARM_DESC(bt_rf_power_mode, "RF Power Mode for BT");

module_param(zigb_rf_power_mode, ushort, 0);
MODULE_PARM_DESC(zigb_rf_power_mode, "RF Power Mode for ZIGB");

module_param(obm_ant_sel_val, ushort, 0);
MODULE_PARM_DESC(obm_ant_sel_val, "(2) For Internal Antenna Selection\
	                              (3) For External Antenna Selection");

module_param(user_onboard_ant_val , ushort, 0);
MODULE_PARM_DESC(user_onboard_ant_val, "(1) For ONBOARD ANTENNA\
	                              (0) For External Antenna ");

module_param(fw_load_mode, ushort, 0);
MODULE_PARM_DESC(fw_load_mode, " FW Download Mode	\
				1 - Full Flash mode with Secondary Boot Loader\
				2 - Full RAM mode with Secondary Boot Loader \
				3 - Flash + RAM mode with Secondary Boot Loader\
				4 - Flash + RAM mode without Secondary Boot Loader");

module_param(skip_fw_load, ushort, 0);
MODULE_PARM_DESC(skip_fw_load, " 1 to Skip fw loading else 2");

module_param(onebox_zone_enabled,uint,0);
module_param(sdio_clock, ushort, 0);
module_param(enable_antenna_diversity, bool, 0);
MODULE_PARM_DESC(enable_antenna_diversity, "Enable (1) or Disable (0) antenna diversity feature");
module_param(enable_high_speed, bool, 0);
MODULE_PARM_DESC(enable_high_speed, "Enable (1) or Disable (0) High speed mode");
MODULE_PARM_DESC(sdio_clock, "SDIO Clock frequency in MHz");
module_param_string(firmware_path, firmware_path, sizeof(firmware_path), 0);
MODULE_PARM_DESC(firmware_path, "Location of firmware files");
module_param(retry_count, ushort, 0);
MODULE_PARM_DESC(retry_count, "In order to set number of packet retries ");
module_param(module_type, ushort, 0);
MODULE_PARM_DESC(module_type, "Module Type (0) B8\
                                           (1) Q7\
                                           (2) M15B\
                                           (3) M15DB-T\
                                           (4) M15SB");
module_param(pll_mode, ushort, 0);
MODULE_PARM_DESC(pll_mode, "PLL Modes (0) PLL_MODE0\
                                      (1) PLL_MODE1\
                                      (2) PLL_MODE2");
module_param(rf_type_2g, ushort, 0);
MODULE_PARM_DESC(rf_type_2g, "RF type only for 2G (0) External RF\
                                                  (1) Internal RF");
module_param(pwr_save_opt, ushort, 0);
MODULE_PARM_DESC(pwr_save_opt, "Power save options (0) Disable Duty Cycling & End of Frame\
                                                   (1) Enable Duty Cycling\
                                                   (2) Enable End of Frame\
                                                   (3) Enable Duty Cycling & End of Frame");
module_param(standby_assoc_chain_sel, bool, 0);
MODULE_PARM_DESC(standby_assoc_chain_sel, "Standy Associated Mode Chain Selection (0) HP Chain Enabled\
                                                                                  (1) LP Chain Enabled");
module_param(lmac_bcon_drop, ushort, 0);
MODULE_PARM_DESC(lmac_bcon_drop, "LMAC Beacon Drop Feature (0) Disable LMAC BEACON DROP Feature\
                                                           (1) Enable LMAC BEACON DROP Feature");
module_param(host_intf_on_demand, bool, 0);
MODULE_PARM_DESC(host_intf_on_demand, "Host Interface on Demand Feature (0) Disable Host Interface on Demand Feature\
                                                                        (1) Enable Host Interface on Demand Feature");
module_param(ext_opt, ushort, 0);
MODULE_PARM_DESC(ext_opt, "Extended options - TBD");

module_param(txpkt_lifetime, ushort, 0);
MODULE_PARM_DESC(txpkt_lifetime, "LMAC Tx Packet Lifetime  (0) Use default Value.");

void obm_configure_region(struct driver_assets *d_assets, uint16 country_code)
{

	switch(country_code)
	{
		/* Countries in US Region */
		case CTRY_CANADA:
		case CTRY_MEXICO:				
		case CTRY_UNITED_STATES:  
			d_assets->country_code = country_code;
			d_assets->region_code = 0;			
			break;

			/* Countries in EU Region */
		case CTRY_FRANCE:
		case CTRY_GERMANY:
		case CTRY_ITALY:
		case CTRY_BELGIUM:
			d_assets->country_code = country_code;
			d_assets->region_code = 1;			
			break;

			/* Countries in Japan Region */
		case CTRY_JAPAN:
			d_assets->country_code = country_code;
			d_assets->region_code = 2;			
			break;
			/* Countries in China Region */
		case CTRY_CHINA:
			d_assets->country_code = country_code;
			d_assets->region_code = 4;
		break;
			/* Countries in Taiwan Region */
		case CTRY_TAIWAN:
			d_assets->country_code = country_code;
			d_assets->region_code = 5;
		break;
			/* Countries in Rest of the World Region */
		case CTRY_AUSTRALIA:
		case CTRY_INDIA:
		case CTRY_IRAN:
		case CTRY_MALAYSIA:
		case CTRY_NEW_ZEALAND:			
		case CTRY_RUSSIA:
		case CTRY_SINGAPORE:
		case CTRY_SOUTH_AFRICA:
			d_assets->country_code = country_code;
			d_assets->region_code = 3;
			break;
		default:  
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
					(TEXT("%s: Default Country Code %d selected\n"), __func__, country_code)); 
			d_assets->country_code = 0;
			d_assets->region_code = 3;			//Just an indication of the country, to refer index in regdm_table[]
			break;
	}

}

ONEBOX_STATUS onebox_set_fw_load_mode(PONEBOX_ADAPTER adapter) 
{
    if(adapter->device_model == RSI_DEV_9116) 
        adapter->fw_load_mode = FW_LOAD_WITH_DESC;	
    else
        adapter->fw_load_mode = fw_load_mode;

    switch (fw_load_mode) {
        case FULL_FLASH_SBL: 
            ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: Full flash mode with Secondary Bootloader\n"), __func__));    
            break;
        case FULL_RAM_SBL: 
            ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: Full RAM mode with Secondary Bootloader\n"), __func__));    
            break;
        case FLASH_RAM_SBL: 
            ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: Flash + RAM mode with Secondary Bootloader\n"), __func__));    
            break;
        case FLASH_RAM_NO_SBL: 
            ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: Flash + RAM mode without/No Secondary Bootloader\n"), __func__));    
            break;
        case FW_LOAD_WITH_DESC: 
            ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: Flash + RAM mode without/No Secondary Bootloader\n"), __func__));    
            break;
        default:
            ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: Default Full flash mode with Secondary Bootloader\n"), __func__));
            adapter->fw_load_mode = FULL_FLASH_SBL;
            break;
    }
    return ONEBOX_STATUS_SUCCESS;
}

ONEBOX_STATUS read_reg_parameters(PONEBOX_ADAPTER adapter)
{
	struct driver_assets *d_assets = adapter->d_assets;

	if(lp_ps_handshake_mode == 0)
	{
		d_assets->lp_ps_handshake_mode	= NO_HAND_SHAKE;
	}
	else if(lp_ps_handshake_mode == 1)
	{
		d_assets->lp_ps_handshake_mode	= GPIO_HAND_SHAKE;
	}

	if(ulp_ps_handshake_mode == 0)
	{
		d_assets->ulp_ps_handshake_mode	= NO_HAND_SHAKE;
	}
	else if(ulp_ps_handshake_mode == 2)
	{
		d_assets->ulp_ps_handshake_mode	= PACKET_HAND_SHAKE;
	}
	else if(ulp_ps_handshake_mode == 1)
	{
		d_assets->ulp_ps_handshake_mode	= GPIO_HAND_SHAKE;
	}

	if (rf_power_val == 0)
		d_assets->rf_power_val = RF_POWER_1_9;
	else
		d_assets->rf_power_val = RF_POWER_3_3;

	if (device_gpio_type == TA_GPIO)
		d_assets->device_gpio_type = TA_GPIO;
	else
		d_assets->device_gpio_type = ULP_GPIO;

	if (driver_mode == WIFI_MODE_ON)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: WiFi mode on\n"), __func__));    
		adapter->Driver_Mode = WIFI_MODE_ON;
	}
	else if (driver_mode == RF_EVAL_MODE_ON)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: RF Evaluation mode on\n"), __func__));    
		adapter->Driver_Mode = RF_EVAL_MODE_ON;
	}
	else if (driver_mode == RF_EVAL_LPBK_CALIB)
	{
    /*Try to optimize these conditions */
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: RF Eval LPBK CALIB mode on\n"), __func__));    
		adapter->Driver_Mode = RF_EVAL_LPBK_CALIB; /* RF EVAL mode */
	} 
	else if (driver_mode == RF_EVAL_LPBK)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: RF Eval LPBK mode on\n"), __func__));    
		adapter->Driver_Mode = RF_EVAL_LPBK; /* RF EVAL mode */
	} 
	else if (driver_mode == QSPI_FLASHING)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: QSPI_FLASHING mode on\n"), __func__));    
		adapter->Driver_Mode = QSPI_FLASHING;
		adapter->flashing_mode = QSPI_FLASHING;
	} 
	else if (driver_mode == QSPI_UPDATE)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: QSPI_UPDATE mode on\n"), __func__));    
		adapter->Driver_Mode = QSPI_FLASHING;
		adapter->flashing_mode = QSPI_UPDATE;
	}
	else if (driver_mode == SNIFFER_MODE)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: Sniffer mode on\n"), __func__));
		adapter->Driver_Mode = SNIFFER_MODE;
	}
	else if (driver_mode == SWBL_FLASHING_NOSBL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: Sw Bootloader Flashing mode on\n"), __func__));
		adapter->Driver_Mode = QSPI_FLASHING;
		adapter->flashing_mode = SWBL_FLASHING_NOSBL;
	}
	else if (driver_mode == SWBL_FLASHING_NOSBL_FILE)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: Sw Bootloader Flashing mode on with Calib from file\n"), __func__));
		adapter->Driver_Mode = QSPI_FLASHING;
		adapter->flashing_mode = SWBL_FLASHING_NOSBL_FILE;
	}
	else if (driver_mode == SWBL_FLASHING_SBL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: Sw Bootloader Flashing mode on\n"), __func__));
		adapter->Driver_Mode = QSPI_FLASHING;
		adapter->flashing_mode = SWBL_FLASHING_SBL;
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: WiFi mode on\n"), __func__));    
		adapter->Driver_Mode = WIFI_MODE_ON;
	} /* End if <condition> */
	
	memcpy(adapter->firmware_path, firmware_path, sizeof(firmware_path));

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: OPER MODE is %d\n"), __func__, oper_mode));    
	d_assets->oper_mode = coex_mode;

	if (d_assets->oper_mode && (d_assets->oper_mode < 4)) {
		adapter->coex_mode = WIFI_ALONE;
	} else if ((d_assets->oper_mode & (OP_BT_CLASSIC_MODE | OP_BT_LE_MODE)) && 
						 !(d_assets->oper_mode & OP_ZIGBEE_MODE)) {
							if((d_assets->oper_mode & (OP_WLAN_AP_MODE | OP_WLAN_STA_MODE)) == OP_WLAN_AP_MODE) {
								adapter->coex_mode  = WIFI_BT_LE;
							} else {
								adapter->coex_mode  = WIFI_BT_CLASSIC;
							}
	} else if ((d_assets->oper_mode & OP_ZIGBEE_MODE) && 
						 !(d_assets->oper_mode & (OP_BT_CLASSIC_MODE | OP_BT_LE_MODE))) {
		adapter->coex_mode  = WIFI_ZIGBEE;
	} else {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT
					("%s:<========== ERROR: INVALID OPERMODE %d ===========>\n"), __func__,
					d_assets->oper_mode));
		return ONEBOX_STATUS_FAILURE;
    }

    onebox_set_fw_load_mode(adapter);

	adapter->skip_fw_load = skip_fw_load;

#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18)
	if (enable_high_speed)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INIT, (TEXT("%s: High speed mode on\n"), __func__));
		adapter->sdio_high_speed_enable = 1;
	}
	else
	{
		adapter->sdio_high_speed_enable = 0;
	} /* End if <condition> */
#endif

	adapter->sdio_clock_speed = sdio_clock;
	d_assets->ta_aggr    = ta_aggr;	
	d_assets->asset_role = adapter->Driver_Mode;
	d_assets->coex_mode  = adapter->coex_mode;
	d_assets->antenna_diversity = enable_antenna_diversity;
	d_assets->wlan_rf_power_mode = wlan_rf_power_mode;
	d_assets->bt_rf_power_mode = bt_rf_power_mode;
	d_assets->zigb_rf_power_mode = zigb_rf_power_mode;
	d_assets->obm_ant_sel_val = obm_ant_sel_val;
	d_assets->user_onboard_ant_val = user_onboard_ant_val;
	d_assets->peer_dist = peer_dist;
	d_assets->bt_feature_bitmap = bt_feature_bitmap;
	d_assets->uart_debug = uart_debug;
	d_assets->w_9116_features.pll_mode = pll_mode;
	d_assets->w_9116_features.rf_type = rf_type_2g;
	d_assets->ext_opt = module_type;
	d_assets->w_9116_features.pwrsave_options = pwr_save_opt;
    //! The following variable sets the Bits required for lmac bcon drop & standby associated chain selection
    //! that should be given in 9116 features frame. Do not use BITS(3:0) as these are used for other power save options
	d_assets->wlan_pwrsave_options = (lmac_bcon_drop << 5) | (standby_assoc_chain_sel << 4);
	d_assets->lmac_bcon_en_dis_threshold = 4;
	d_assets->host_intf_on_demand = host_intf_on_demand;
	obm_configure_region(d_assets, country_code);
	if(retry_count < 7 || retry_count > 15 )
	{
		d_assets->retry_count = 15; //setting default value as 15 for pkt retry count
	}
	else	
	{
		d_assets->retry_count = retry_count;
	}
	if(txpkt_lifetime <= MAX_TXPKT_LIFETIME)
	{
		d_assets->txpkt_lifetime = txpkt_lifetime;
	}
	else
	{
		d_assets->txpkt_lifetime = TXPKT_LIFETIME_DEFAULT_VALUE;
	}
	return ONEBOX_STATUS_SUCCESS;
}

#ifdef USE_USB_INTF
static int32 update_wlan_usb_buf_status(void *global_priv)
{
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)global_priv;
	return atomic_read(&adapter->tx_pending_urb_cnt);
}
#endif

int disconnect_assets(struct driver_assets *d_assets)
{
	int s = ONEBOX_STATUS_SUCCESS;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s %d  moving card state to GS_CARD_DETACH\n"), __func__, __LINE__));
	d_assets->card_state = GS_CARD_DETACH;

	if (d_assets->techs[WLAN_ID].drv_state == MODULE_ACTIVE) {
		if (d_assets->techs[WLAN_ID].deregister_flags) {
			d_assets->techs[WLAN_ID].deregister_flags = 0;
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("In %s Line %d Waking up wait event for deregister\n", __func__, __LINE__));
			wake_up(&d_assets->techs[WLAN_ID].deregister_event);
		}
		if (d_assets->techs[WLAN_ID].disconnect) {
			d_assets->techs[WLAN_ID].disconnect(d_assets);
		}
		d_assets->techs[WLAN_ID].drv_state = MODULE_INSERTED;
	}

	if (d_assets->techs[BT_ID].drv_state == MODULE_ACTIVE) {
		if (d_assets->techs[BT_ID].deregister_flags) {
			d_assets->techs[BT_ID].deregister_flags = 0;
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("In %s Line %d Waking up wait event for deregister\n", __func__, __LINE__));
			wake_up(&d_assets->techs[BT_ID].deregister_event);
		}
		if (d_assets->techs[BT_ID].disconnect) {
			d_assets->techs[BT_ID].disconnect(d_assets);
		}
		d_assets->techs[BT_ID].drv_state = MODULE_INSERTED;
	}

	if (d_assets->techs[ZB_ID].drv_state == MODULE_ACTIVE) {
		if (d_assets->techs[ZB_ID].deregister_flags) {
			d_assets->techs[ZB_ID].deregister_flags = 0;
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("In %s Line %d Waking up wait event for deregister\n", __func__, __LINE__));
			wake_up(&d_assets->techs[ZB_ID].deregister_event);
		}
		if (d_assets->techs[ZB_ID].disconnect) {
			d_assets->techs[ZB_ID].disconnect(d_assets);
		}
		d_assets->techs[ZB_ID].drv_state = MODULE_INSERTED;
	}
	return s;
}

void common_hal_deinit(struct driver_assets *d_assets, PONEBOX_ADAPTER adapter)
{
	int count;
    adapter->os_intf_ops->onebox_delete_event(&(adapter->flash_event));

	if (adapter->device_model == RSI_DEV_9116) {
    adapter->os_intf_ops->onebox_delete_event(&(adapter->common_bb_rf_event));
	}

#ifdef USE_WORKQUEUES	
	adapter->os_intf_ops->onebox_deinit_workq(adapter->int_work_queue);
#endif
	adapter->os_intf_ops->onebox_queue_purge(&adapter->deferred_rx_queue);

#ifdef USE_TASKLETS
	tasklet_kill(&adapter->int_bh_tasklet);
#endif
	/* Remove upper layer protocols firstly */
	disconnect_assets(d_assets); 

	/* deinit device */
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s: calling device deinitialization\n"), __func__));
	adapter->coex_osi_ops->onebox_device_deinit(adapter);

#ifdef SDIO_DEBUG
	adapter->os_intf_ops->onebox_remove_timer(&adapter->sdio_debug_timer);//NEED TO TEST
#endif
	/* Remove the proc file system created for the device*/
	adapter->os_intf_ops->onebox_remove_proc_entry(adapter->onebox_entry, adapter->proc_name);

	for (count = 0; count < COEX_SOFT_QUEUES; count++) 
		adapter->os_intf_ops->onebox_queue_purge(&adapter->coex_queues[count]);

	adapter->os_intf_ops->onebox_mem_free(adapter->DataRcvPacket);
	adapter->DataRcvPacket = NULL;
	adapter->os_intf_ops->onebox_kill_thread(&adapter->sdio_scheduler_thread_handle); 
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s: Uninitialized Procfs\n"), __func__));   
	return;
}

#ifdef SDIO_DEBUG
void sdio_debug_stats(PONEBOX_ADAPTER adapter)
{

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
			(TEXT("SDIO DEBUG Timer Expired Below are the stats: \n"
				"total_sdio_interrupts_recvd %d\n"
				"total_sdio_msdu_pending_intr %d\n"
				"total_sdio_unknown_intr %d\n"
				"total_interrupts_with_status_zero %d \n"
				"total_buf_status_interrupts %d\n"
				"total_cmd52_skip_count %d\n"),
				adapter->timer_sdio_int_counter, 
				adapter->timer_total_sdio_msdu_pending_intr,
				adapter->timer_total_sdio_unknown_intr,
				adapter->timer_sdio_intr_status_zero,
				adapter->timer_buf_status_interrupts,
				adapter->total_cmd52_skipped));

				adapter->timer_sdio_int_counter = 0;
				adapter->timer_total_sdio_msdu_pending_intr = 0;
				adapter->timer_total_sdio_unknown_intr = 0;
				adapter->timer_sdio_intr_status_zero =0;
				adapter->timer_buf_status_interrupts = 0;
				adapter->total_cmd52_skipped = 0;
				adapter->os_intf_ops->onebox_mod_timer(&adapter->sdio_debug_timer, msecs_to_jiffies(1000));
}
#endif

/* This function initializes the common hal */
ONEBOX_STATUS common_hal_init(struct driver_assets *d_assets, PONEBOX_ADAPTER adapter)
{
	//struct onebox_os_intf_operations *os_intf_ops = onebox_get_os_intf_operations_from_origin();
	struct onebox_coex_osi_operations *coex_osi_ops = onebox_get_coex_osi_operations();
	int count;
	
	ONEBOX_DEBUG( ONEBOX_ZONE_INIT, ("In %s Line %d initializing common Hal init \n", __func__, __LINE__));
	for (count = 0; count < COEX_SOFT_QUEUES; count++) 
		adapter->os_intf_ops->onebox_netbuf_queue_init(&adapter->coex_queues[count]);

	adapter->os_intf_ops->onebox_netbuf_queue_init(&adapter->deferred_rx_queue);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Mutex init successfull\n"), __func__));

	//adapter->os_intf_ops->onebox_init_event(&(adapter->coex_tx_event));
	adapter->os_intf_ops->onebox_init_event(&(adapter->flash_event));

	if (adapter->device_model == RSI_DEV_9116) {
    adapter->os_intf_ops->onebox_init_event(&(adapter->common_bb_rf_event));
	}

	adapter->os_intf_ops->onebox_init_dyn_mutex(&d_assets->tx_access_lock);
	adapter->os_intf_ops->onebox_init_dyn_mutex(&d_assets->wlan_init_lock);
	adapter->os_intf_ops->onebox_init_dyn_mutex(&d_assets->bt_init_lock);
	adapter->os_intf_ops->onebox_init_dyn_mutex(&d_assets->zigbee_init_lock);
	d_assets->update_tx_status = &update_tx_status;
#ifdef USE_USB_INTF
	d_assets->update_usb_buf_status = &update_wlan_usb_buf_status;
#endif

	if (adapter->os_intf_ops->onebox_init_thread(&(adapter->sdio_scheduler_thread_handle),
	                                    "COEX-TX-Thread",
	                                    0,
	                                    coex_osi_ops->onebox_coex_transmit_thread, 
	                                    adapter) != ONEBOX_STATUS_SUCCESS)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Unable to initialize thrd\n"), __func__));
		adapter->os_intf_ops->onebox_mem_free(adapter->DataRcvPacket);
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("In %s Line %d initializing common Hal init \n", __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}

	d_assets->common_send_pkt_to_coex = send_pkt_to_coex;
	d_assets->onebox_common_read_register = onebox_common_read_register;
	d_assets->onebox_common_write_register = onebox_common_write_register;
	d_assets->onebox_common_read_multiple = onebox_common_read_multiple;
	d_assets->onebox_common_write_multiple = onebox_common_write_multiple;
#ifdef USE_USB_INTF
	d_assets->onebox_common_ta_write_multiple = onebox_common_ta_write_multiple;
	d_assets->onebox_common_ta_read_multiple = onebox_common_ta_read_multiple;
#endif
	d_assets->onebox_common_master_reg_read = onebox_common_master_reg_read;
    d_assets->onebox_common_master_reg_write = onebox_common_master_reg_write;
    d_assets->onebox_send_coex_configuration = onebox_send_coex_configuration;
	if (adapter->device_model == RSI_DEV_9116) {
    d_assets->onebox_wait_for_rf_prog_frame_rsp = onebox_wait_for_rf_prog_frame_rsp;
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Initialized thread & Event\n"), __func__));

	/* start the transmit thread */
	adapter->os_intf_ops->onebox_start_thread( &(adapter->sdio_scheduler_thread_handle));

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT, ("Common hal: Init proc entry call\n"));
	/* Create proc filesystem */

	if (adapter->host_intf_type == HOST_INTF_USB)
		snprintf(adapter->proc_name, 23, "%s%d%d", "onebox-hal", 
				 adapter->usbdev->devnum, adapter->usbdev->bus->busnum);
	else
		snprintf(adapter->proc_name, 16, "%s%s", "onebox-hal-", 
				 sdio_func_id(adapter->sdio_pfunction));

	adapter->onebox_entry = adapter->os_intf_ops->onebox_init_proc(adapter, adapter->proc_name);
	if(adapter->onebox_entry == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Failed to initialize procfs\n"), __func__));
		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("In %s Line %d initializing common Hal init \n", __func__, __LINE__));
		return ONEBOX_STATUS_FAILURE;
	}
	d_assets->common_hal_fsm = COMMAN_HAL_WAIT_FOR_CARD_READY;
	ONEBOX_DEBUG( ONEBOX_ZONE_INIT, ("In %s Line %d initializing common Hal init \n", __func__, __LINE__));

#ifdef USE_WORKQUEUES
/* coex workqeue */
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("HAL : Using WORKQUEUES as the interrupt bottom halfs\n"));
	adapter->int_work_queue = adapter->os_intf_ops->onebox_create_work_queue("onebox_workerQ");
	if (!adapter->int_work_queue) {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Unable to create work queue\n"));
		return ONEBOX_STATUS_FAILURE;
	}

	INIT_WORK((struct work_struct *)&adapter->defer_work, &deferred_rx_packet_parser);
#endif

#ifdef USE_TASKLETS
	ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Using TASKLETS as the interrupt bottom halfs\n"));
	tasklet_init(&adapter->int_bh_tasklet,
		     &deferred_rx_tasklet,
		     adapter);
#endif
#ifdef SDIO_DEBUG
						if(!adapter->sdio_debug_timer.function) {
								adapter->os_intf_ops->onebox_init_sw_timer(&adapter->sdio_debug_timer, (unsigned long)adapter,
												(void *)&sdio_debug_stats, msecs_to_jiffies(1000));
						}

#endif
#ifdef GPIO_HANDSHAKE
	adapter->os_intf_ops->onebox_gpio_init();
#endif
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This Function Initializes The HAL
 *
 * @param pointer to HAL control block
 * @return
 *  ONEBOX_STATUS_SUCCESS on success, ONEBOX_STATUS_FAILURE on failure
 */
ONEBOX_STATUS device_init(PONEBOX_ADAPTER adapter, uint8 fw_load)
{
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	uint32 regout_val = 0;
	uint8 bl_req = 0;

	FUNCTION_ENTRY(ONEBOX_ZONE_INIT);
	if(adapter->Driver_Mode == QSPI_FLASHING)
		adapter->fsm_state = FSM_CARD_NOT_READY;
	else
		adapter->fsm_state = FSM_DEVICE_READY;

	if(adapter->fw_load_mode == FULL_FLASH_SBL ||
			adapter->fw_load_mode == FULL_RAM_SBL ||
			adapter->fw_load_mode == FLASH_RAM_SBL) {
		bl_req = 1;
	} else if((adapter->fw_load_mode == FLASH_RAM_NO_SBL) ||
       (adapter->fw_load_mode == FW_LOAD_WITH_DESC)) {
		bl_req = 0;
	} else {
		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Unexpected fw load mode %d.. Returning failure\n",adapter->fw_load_mode));
		goto fail;
	}

	if(fw_load && !adapter->skip_fw_load) {
		bl_cmd_start_timer(adapter, BL_CMD_TIMEOUT);
		while(!adapter->bl_timer_expired) {
			if(adapter->osd_host_intf_ops->onebox_master_reg_read(adapter, 
						SWBL_REGOUT, &regout_val, 2) < 0) {
				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d REGOUT reading failed..\n",__func__,
						__LINE__));
				bl_cmd_stop_timer(adapter);
				goto fail;
			}
			adapter->os_intf_ops->onebox_msec_delay(1);	
			if((regout_val >> 8) == REGOUT_VALID) {
				break;
			}
		}
		if(adapter->bl_timer_expired) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d REGOUT reading timed out..\n",__func__,
					__LINE__));
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Software Boot loader Not Present\n"));
			if(bl_req) {
				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Expecting Software boot loader tobe present."
						"Enable/flash Software boot loader\n")); 
				goto fail;
			} else {
				ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Software boot loader is disabled as expected\n"));
			}
		} else {
			bl_cmd_stop_timer(adapter);
			ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Software Boot loader Present\n"));
			if(bl_req) {
				ONEBOX_DEBUG( ONEBOX_ZONE_INIT, ("Software boot loader is enabled as expected\n"));
			} 
		}

		ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Bootloader version:%x\n",regout_val & 0xf));
		adapter->bl_ver = regout_val & 0xff;

		if((adapter->osd_host_intf_ops->onebox_master_reg_write(adapter, 
						SWBL_REGOUT, (REGOUT_INVALID | REGOUT_INVALID << 8), 2)) < 0) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s:%d REGOUT writing failed..\n",__func__,
					__LINE__));
			goto fail;
		}
		adapter->os_intf_ops->onebox_msec_delay(1);	

		if((adapter->fw_load_mode == FLASH_RAM_NO_SBL) ||
				(adapter->fw_load_mode == FW_LOAD_WITH_DESC)) {
			if (adapter->device_model == RSI_DEV_9116) {
				if (adapter->host_intf_type == HOST_INTF_USB) {
					if(bl_cmd(adapter, POLLING_MODE, CMD_PASS, "POLLING_MODE") < 0) {
						goto fail;
					}
				}
			}
			status = load_ta_instructions(adapter);
			if (adapter->device_model == RSI_DEV_9116) {
				if (adapter->host_intf_type == HOST_INTF_USB) {
					if(bl_cmd(adapter, JUMP_TO_ZERO_PC, CMD_PASS, "JUMP_TO_ZERO") < 0) {
						goto fail;
					}
					ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("Jump to zero command successful\n"));
				}
			}
		}
		else
			status = load_fw_thru_sbl(adapter);
	}
	FUNCTION_EXIT(ONEBOX_ZONE_INIT);
	return status;
fail:
	return ONEBOX_STATUS_FAILURE;
} /* onebox_device_init */

/**
 * This Function Free The Memory Allocated By HAL Module
 *
 * @param pointer to HAL control block
 *
 * @return
 * ONEBOX_STATUS_SUCCESS on success, ONEBOX_STATUS_FAILURE on failure
 */
ONEBOX_STATUS device_deinit(PONEBOX_ADAPTER adapter)
{
	FUNCTION_ENTRY(ONEBOX_ZONE_INFO);

	//adapter->coex_osi_ops->onebox_core_deinit(adapter);

#ifdef GPIO_HANDSHAKE
	adapter->os_intf_ops->onebox_gpio_deinit();
#endif
	FUNCTION_EXIT(ONEBOX_ZONE_INFO);
	return ONEBOX_STATUS_SUCCESS;
}

static struct onebox_os_intf_operations    *os_intf_ops;

int onebox_register_os_intf_operations (struct onebox_os_intf_operations *os_intf_opeartions)
{
	os_intf_ops = os_intf_opeartions;
	return 0;
}
struct onebox_os_intf_operations *onebox_get_os_intf_operations(void)
{
	return os_intf_ops;
}

ONEBOX_STATIC int32 onebox_nongpl_module_init(VOID)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("onebox_nongpl_module_init called and registering the nongpl driver\n")));
	return 0;
}

ONEBOX_STATIC VOID onebox_nongpl_module_exit(VOID)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("onebox_nongpl_module_exit called and unregistering the nongpl driver\n")));
	return;
}

EXPORT_SYMBOL(onebox_register_os_intf_operations);
EXPORT_SYMBOL(onebox_get_os_intf_operations);
module_init(onebox_nongpl_module_init);
module_exit(onebox_nongpl_module_exit);
MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("Redpine Signals, Inc.");
MODULE_DESCRIPTION("Driver for Redpine Signals' RS9113 module based USB/SDIO cards.");
MODULE_SUPPORTED_DEVICE("Redpine Signals' RS9113 module based USB/SDIO cards.");
