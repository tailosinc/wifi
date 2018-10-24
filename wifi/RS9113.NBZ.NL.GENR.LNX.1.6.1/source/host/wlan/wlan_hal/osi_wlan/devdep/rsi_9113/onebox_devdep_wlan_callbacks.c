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

#include "wlan_common.h"
#include "onebox_linux.h"
#include "onebox_wlan_pktpro.h"

static struct onebox_devdep_operations devdep_ops =
{
	/* In case if u want to change the order of these member assignments, you should change
	 * the order in the declarations also */
	NULL,//interrupt_handler,          /* .onebox_handle_interrupt          = interrupt_handler,*/
	NULL,//load_ta_instructions,       /* .onebox_load_ta_instructions      = load_ta_instructions,*/
	onebox_mgmt_pkt_recv,       /* .onebox_snd_cmd_pkt               = onebox_mgmt_pkt_recv,*/
	//send_onair_mgmt_pkt,        /* .onebox_snd_mgmt_pkt              = send_onair_mgmt_pkt,*/
	send_beacon,                /* .onebox_send_beacon               = send_beacon,*/
	send_onair_data_pkt,        /* .onebox_send_data_pkt             = send_onair_data_pkt,*/
	NULL,//send_bt_pkt, /*.onebox_send_pkt = send_bt_pkt */
	NULL,//device_init,                /* .onebox_device_init               = device_init,*/
	NULL,//device_deinit,              /* .onebox_device_deinit             = device_deinit,*/
	set_channel,                /* .onebox_set_channel               = set_channel,*/
	bb_reset_req, 
	hal_set_sec_wpa_key,        /* .onebox_hal_set_sec_wpa_key       = hal_set_sec_wpa_key,*/
	hal_send_sta_notify_frame,  /* .onebox_hal_send_sta_notify_frame = hal_send_sta_notify_frame,*/
	schedule_pkt_for_tx,        /* .onebox_schedule_pkt_for_tx       = schedule_pkt_for_tx,*/
	sdio_scheduler_thread,      /* .onebox_sdio_scheduler_thread     = sdio_scheduler_thread,*/
//	send_qos_conf_frame,        /*  onebox_send_qos_conf_frame       = send_qos_conf_frame,*/
	program_bb_rf,              /*  onebox_program_bb_rf             = program_bb_rf*/
	update_device_op_params,    /*  onebox_update_device_op_params   = update_device_op_params */
//	start_tx_rx,                /*  onebox_start_tx_rx               = start_tx_rx */
	start_autorate_stats,       /*  onebox_start_autorate_stats      = start_autorate_stats */
	hal_load_key,               /*  onebox_hal_load_key              = hal_load_key*/
	set_vap_capabilities,       /*  onebox_set_vap_capabilities      = set_vap_capabilities */
	//hw_queue_status,          /*  onebox_hw_queue_status           = hw_queue_status*/
	band_check,         	    /*  onebox_band_check                = band_check*/
	set_bb_rf_values,         	/*  onebox_set_bb_rf_values          = set_bb_rf_values*/
#ifdef PWR_SAVE_SUPPORT
//	onebox_send_ps_params,		/*  onebox_send_ps_params		     = onebox_send_ps_params */
	//reissue_ioctl,
	//update_traffic_psp_params,
	//update_uapsd_params,
#endif
	set_cw_mode,         	    /*  onebox_cw_mode          = set_bb_rf_values*/
	onebox_send_internal_mgmt_frame,  /* onebox_send_internal_mgmt_frame = send_internal_mgmt_frame */
	eeprom_read,         	      /*  onebox_eeprom_rd                 = eeprom_read */
	NULL,//flash_write,  			/*onebox_flash_write              = flash_write */
#ifdef BYPASS_TX_DATA_PATH
	onebox_send_block_unblock,
#endif
	onebox_ant_sel,         	    /*  onebox_program_ant_sel          = onebox_ant_sel*/
	onebox_do_master_ops,					/*	onebox_do_master_ops						=	onebox_do_master_ops*/
	onebox_send_debug_frame,
	onebox_conf_beacon_recv,
	get_tx_power,
#ifdef RADAR_AUTO
  	onebox_send_radar_req_frame,
#endif
	check_scan,										/* onebox_check_scan */
	process_eeprom_write,       /* .onebox_process_eeprom_write      = process_eeprom_write,*/ 
};

struct onebox_devdep_operations *onebox_get_devdep_wlan_operations(void)
{
	return (&devdep_ops);
}
EXPORT_SYMBOL(onebox_get_devdep_wlan_operations);
